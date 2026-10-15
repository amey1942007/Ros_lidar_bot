#!/usr/bin/env python3
"""
amr_telemetry_bridge — Pi → Supabase Realtime → https://amrslam.tech/control

Runs on the robot (RPi5) beside robot_dashboard_node.py. Reads the local SSE
feed and POSTs commands back to the dashboard HTTP API.

Network path (preferred):
  Pi ── Tailscale ── laptop/ops   (management)
  Pi ── Cloudflare Tunnel / Supabase Realtime ── browsers on /control

Env (see /etc/amr-bridge.env — also loaded by amr_bridge.launch.py):
  AMR_BOT_ID=amr4|hardware-bot
  AMR_SUPABASE_URL=https://….supabase.co
  AMR_SUPABASE_KEY=…                  # publishable/anon key (Realtime + touch RPC)
  AMR_BRIDGE_TOKEN=…                  # shared secret → public.touch_amr_bot
  AMR_SUPABASE_SERVICE_KEY=…          # optional legacy; not required
  AMR_DASHBOARD_URL=http://127.0.0.1:8080
  AMR_CAMERA_URL=                     # optional public MJPEG (CF tunnel)
  AMR_CAMERA_JPEG_MAX_KB=24           # inline JPEG budget when scraping /camera.mjpg
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import logging
import os
import signal
import threading
import time
from typing import Any, Optional

import httpx

from . import __version__
from .protocol import BOARD_RATE_HZ, BOT_IDS, BRIDGE_VERSION, FULL_RATE_HZ, channel_name

log = logging.getLogger("amr_bridge")


def _env(name: str, default: str | None = None) -> str | None:
    v = os.environ.get(name, default)
    return v.strip() if isinstance(v, str) and v.strip() else default


class DashboardClient:
    def __init__(self, base: str):
        self.base = base.rstrip("/")
        self._http = httpx.AsyncClient(timeout=8.0)

    async def close(self) -> None:
        await self._http.aclose()

    async def post(self, path: str, payload: dict | None = None) -> dict:
        r = await self._http.post(f"{self.base}{path}", json=payload or {})
        r.raise_for_status()
        try:
            return r.json()
        except Exception:
            return {"ok": True}

    async def fetch_camera_jpeg(self, max_kb: int) -> Optional[str]:
        """Grab one JPEG from the multipart MJPEG stream (best-effort)."""
        try:
            async with self._http.stream("GET", f"{self.base}/camera.mjpg") as r:
                if r.status_code != 200:
                    return None
                buf = b""
                async for chunk in r.aiter_bytes():
                    buf += chunk
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2)
                    if start >= 0 and end > start:
                        jpeg = buf[start : end + 2]
                        if len(jpeg) > max_kb * 1024:
                            return None
                        return base64.b64encode(jpeg).decode("ascii")
                    if len(buf) > 512_000:
                        return None
        except Exception as exc:
            log.debug("camera scrape failed: %s", exc)
            return None
        return None


class RealtimeHub:
    """Thin sync supabase-py wrapper used from the asyncio loop via to_thread."""

    def __init__(self, url: str, key: str, bot_id: str, on_command, bridge_token: str | None):
        from supabase import create_client

        self._bot_id = bot_id
        self._on_command = on_command
        self._bridge_token = bridge_token
        self._client = create_client(url, key)
        self._channel = self._client.channel(channel_name(bot_id))
        self._ready = threading.Event()

    def connect(self) -> None:
        def _cb(message: dict) -> None:
            payload = message.get("payload") if isinstance(message, dict) else message
            if isinstance(payload, str):
                try:
                    payload = json.loads(payload)
                except json.JSONDecodeError:
                    return
            if isinstance(payload, dict):
                self._on_command(payload)

        # supabase-py 2.x broadcast API
        try:
            self._channel.on_broadcast(event="command", callback=_cb)
        except Exception:
            self._channel.on("broadcast", {"event": "command"}, _cb)

        self._channel.subscribe()
        self._ready.set()
        log.info("realtime subscribed (%s)", channel_name(self._bot_id))

    def send(self, event: str, payload: dict) -> None:
        if not self._ready.is_set():
            return
        try:
            self._channel.send_broadcast(event, payload)
        except Exception:
            try:
                self._channel.send({"type": "broadcast", "event": event, "payload": payload})
            except Exception as exc:
                log.debug("broadcast %s failed: %s", event, exc)

    def touch_bot_row(self, camera_url: str | None) -> None:
        if not self._bridge_token:
            return
        try:
            self._client.rpc("touch_amr_bot", {
                "p_bot_id": self._bot_id,
                "p_token": self._bridge_token,
                "p_bridge_version": BRIDGE_VERSION,
                "p_camera_url": camera_url,
            }).execute()
        except Exception as exc:
            log.debug("amr_bots touch skipped: %s", exc)

    def close(self) -> None:
        try:
            self._client.remove_channel(self._channel)
        except Exception:
            pass


class Bridge:
    def __init__(
        self,
        bot_id: str,
        supabase_url: str,
        api_key: str,
        bridge_token: str | None,
        dashboard_url: str,
        camera_url: str | None,
        camera_jpeg_max_kb: int,
    ):
        if bot_id not in BOT_IDS:
            raise SystemExit(f"AMR_BOT_ID must be one of {BOT_IDS}")
        self.bot_id = bot_id
        self.supabase_url = supabase_url.rstrip("/")
        self.api_key = api_key
        self.bridge_token = bridge_token
        self.dash = DashboardClient(dashboard_url)
        self.camera_url = camera_url
        self.camera_jpeg_max_kb = camera_jpeg_max_kb
        self.mode = "full"
        self._seq = 0
        self._hub: RealtimeHub | None = None
        self._stop = asyncio.Event()
        self._cam_every = 0
        self._last_pub = 0.0
        self._loop: asyncio.AbstractEventLoop | None = None

    def _queue_command(self, env: dict) -> None:
        if self._loop and self._loop.is_running():
            asyncio.run_coroutine_threadsafe(self._handle_command(env), self._loop)

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._hub = RealtimeHub(
            self.supabase_url, self.api_key, self.bot_id, self._queue_command, self.bridge_token,
        )
        await asyncio.to_thread(self._hub.connect)
        await asyncio.gather(
            self._sse_loop(),
            self._heartbeat_loop(),
            self._presence_touch_loop(),
        )

    async def _handle_command(self, env: dict) -> None:
        if env.get("botId") not in (self.bot_id, None):
            return
        cmd = env.get("cmd") or {}
        typ = cmd.get("type")
        rank = int((env.get("from") or {}).get("rank") or 0)
        log.info("command %s from rank=%s", typ, rank)
        try:
            if typ == "set_rate":
                mode = cmd.get("mode")
                if mode in ("full", "board"):
                    self.mode = mode
                    log.info("rate mode → %s", mode)
                return
            if rank < 1 and typ != "set_rate":
                return
            if typ == "estop":
                await self.dash.post("/api/estop")
            elif typ in ("nudge", "drive"):
                await self.dash.post("/api/nudge", {
                    "lin": float(cmd.get("lin", 0)),
                    "ang": float(cmd.get("ang", 0)),
                })
            elif typ == "joy":
                # Prefer web_gamepad_node HTTP fallback when configured.
                gamepad = os.environ.get("AMR_GAMEPAD_URL", "").rstrip("/")
                axes = cmd.get("axes") or [0.0] * 8
                buttons = int(cmd.get("buttons") or 0)
                if gamepad:
                    await self.dash._http.post(
                        f"{gamepad}/api/joy",
                        json={"a": list(axes)[:8], "b": buttons},
                    )
                else:
                    # Differential approx from left stick: axes[1]=fwd, axes[0]=yaw
                    ax = list(axes) + [0.0] * 8
                    lin = max(-0.4, min(0.4, -float(ax[1]) * 0.35))
                    ang = max(-1.0, min(1.0, -float(ax[0]) * 0.9))
                    if abs(lin) > 0.02 or abs(ang) > 0.02:
                        await self.dash.post("/api/nudge", {"lin": lin, "ang": ang})
            elif typ == "nav_goal":
                await self.dash.post("/api/nav_goal", {
                    "x": float(cmd.get("x", 0)),
                    "y": float(cmd.get("y", 0)),
                })
            elif typ == "nav_cancel":
                await self.dash.post("/api/nav_cancel")
            elif typ == "vision":
                await self.dash.post("/api/vision", {"on": bool(cmd.get("on"))})
            elif typ == "save_map":
                if rank < 2:
                    return
                await self.dash.post("/api/save_map", {"name": cmd.get("name")})
            elif typ == "run_tool":
                if rank < 1:
                    return
                await self.dash.post("/api/run", {
                    "tool": cmd.get("tool"),
                    "params": cmd.get("params") or {},
                })
            elif typ == "stop_tool":
                await self.dash.post("/api/stop")
            elif typ == "blacklist":
                if rank < 1:
                    return
                if cmd.get("action") == "clear":
                    await self.dash.post("/api/blacklist", {"action": "clear"})
                else:
                    await self.dash.post("/api/blacklist", {
                        "x": float(cmd.get("x", 0)),
                        "y": float(cmd.get("y", 0)),
                        "radius": float(cmd.get("radius", 0.8)),
                        "duration": float(cmd.get("duration", 300)),
                    })
        except Exception as exc:
            log.warning("command failed: %s", exc)

    async def _broadcast(self, event: str, payload: dict) -> None:
        if not self._hub:
            return
        await asyncio.to_thread(self._hub.send, event, payload)

    async def _sse_loop(self) -> None:
        url = f"{self.dash.base}/events"
        backoff = 1.0
        while not self._stop.is_set():
            try:
                log.info("connecting SSE %s", url)
                async with self.dash._http.stream(
                    "GET", url, headers={"Accept": "text/event-stream"},
                ) as r:
                    r.raise_for_status()
                    backoff = 1.0
                    buf = ""
                    async for text in r.aiter_text():
                        if self._stop.is_set():
                            break
                        buf += text
                        while "\n\n" in buf:
                            chunk, buf = buf.split("\n\n", 1)
                            data_lines = [
                                ln[5:].lstrip() if ln.startswith("data:") else ""
                                for ln in chunk.split("\n")
                                if ln.startswith("data:")
                            ]
                            if not data_lines:
                                continue
                            raw = "\n".join(data_lines)
                            try:
                                event = json.loads(raw)
                            except json.JSONDecodeError:
                                continue
                            await self._handle_sse_event(event)
            except Exception as exc:
                log.warning("SSE disconnected: %s — retry in %.1fs", exc, backoff)
                await asyncio.sleep(backoff)
                backoff = min(30.0, backoff * 1.7)

    async def _handle_sse_event(self, event: dict) -> None:
        if event.get("t") == "map":
            await self._broadcast("map", {
                "t": "map",
                "botId": self.bot_id,
                "ts": int(time.time() * 1000),
                "w": event.get("w"),
                "h": event.get("h"),
                "res": event.get("res"),
                "origin": event.get("origin"),
                "tf": event.get("tf"),
                "data": event.get("data"),
            })
            return
        if event.get("t") == "log":
            await self._broadcast("log", {
                "botId": self.bot_id,
                "ts": int(time.time() * 1000),
                "time": event.get("time") or time.strftime("%H:%M:%S"),
                "level": event.get("level") or "info",
                "text": event.get("text") or "",
            })
            return
        if "pose" not in event:
            return

        hz = BOARD_RATE_HZ if self.mode == "board" else FULL_RATE_HZ
        now = time.monotonic()
        if now - self._last_pub < (1.0 / hz) * 0.85:
            return
        self._last_pub = now

        self._seq += 1
        mode = self.mode
        payload: dict[str, Any] = {
            "ts": int(time.time() * 1000),
            "botId": self.bot_id,
            "seq": self._seq,
            "mode": mode,
            "online": True,
            "pose": event.get("pose") or [0, 0, 0],
            "mapPose": event.get("map_pose"),
            "vel": event.get("vel") or [0, 0],
            "cmd": event.get("cmd") or [0, 0],
            "imu": event.get("imu"),
            "rates": event.get("rates"),
            "nodes": event.get("nodes"),
            "tf": event.get("tf"),
            "nav": event.get("nav"),
            "path": event.get("path") if mode == "full" else [],
            "goal": event.get("goal"),
            "yoloDet": event.get("yolo_det") if mode == "full" else [],
            "sem": event.get("sem") if mode == "full" else [],
            "frontier": event.get("frontier") if mode == "full" else {"zones": [], "queue": []},
            "sys": event.get("sys"),
            "tool": event.get("tool"),
            "vision": bool(event.get("vision") or event.get("yolo_det") or event.get("sem")),
            "saving": event.get("saving"),
            "cameraUrl": self.camera_url,
            "scan": event.get("scan") if mode == "full" else [],
        }

        if mode == "full" and not self.camera_url:
            self._cam_every = (self._cam_every + 1) % 15
            if self._cam_every == 0:
                jpeg = await self.dash.fetch_camera_jpeg(self.camera_jpeg_max_kb)
                if jpeg:
                    payload["cameraJpegB64"] = jpeg

        await self._broadcast("telemetry", payload)

    async def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            await self._broadcast("hb", {
                "t": "hb",
                "botId": self.bot_id,
                "ts": int(time.time() * 1000),
                "bridgeVersion": BRIDGE_VERSION,
            })
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                pass

    async def _presence_touch_loop(self) -> None:
        while not self._stop.is_set():
            if self._hub:
                await asyncio.to_thread(self._hub.touch_bot_row, self.camera_url)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=5.0)
            except asyncio.TimeoutError:
                pass

    async def stop(self) -> None:
        self._stop.set()
        await self.dash.close()
        if self._hub:
            await asyncio.to_thread(self._hub.close)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    p = argparse.ArgumentParser(description="AMR cloud telemetry bridge")
    p.add_argument("--bot-id", default=_env("AMR_BOT_ID", "amr4"))
    p.add_argument("--supabase-url", default=_env("AMR_SUPABASE_URL") or _env("NEXT_PUBLIC_SUPABASE_URL"))
    p.add_argument(
        "--api-key",
        default=_env("AMR_SUPABASE_KEY")
        or _env("NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY")
        or _env("AMR_SUPABASE_SERVICE_KEY"),
    )
    p.add_argument("--bridge-token", default=_env("AMR_BRIDGE_TOKEN"))
    p.add_argument("--dashboard", default=_env("AMR_DASHBOARD_URL", "http://127.0.0.1:8080"))
    p.add_argument("--camera-url", default=_env("AMR_CAMERA_URL"))
    p.add_argument("--camera-jpeg-max-kb", type=int, default=int(_env("AMR_CAMERA_JPEG_MAX_KB", "24") or 24))
    args = p.parse_args(argv)

    if not args.supabase_url or not args.api_key:
        raise SystemExit("AMR_SUPABASE_URL and AMR_SUPABASE_KEY are required")

    bridge = Bridge(
        bot_id=args.bot_id,
        supabase_url=args.supabase_url,
        api_key=args.api_key,
        bridge_token=args.bridge_token,
        dashboard_url=args.dashboard,
        camera_url=args.camera_url,
        camera_jpeg_max_kb=args.camera_jpeg_max_kb,
    )

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    def _sig(*_a: object) -> None:
        log.info("signal — shutting down")
        loop.create_task(bridge.stop())

    for s in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(s, _sig)
        except NotImplementedError:
            pass

    log.info("amr_telemetry_bridge %s starting (bot=%s)", __version__, args.bot_id)
    try:
        loop.run_until_complete(bridge.start())
    finally:
        loop.run_until_complete(bridge.stop())
        loop.close()


if __name__ == "__main__":
    main()
