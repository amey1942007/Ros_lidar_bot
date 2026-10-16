#!/usr/bin/env python3
"""
amr_telemetry_bridge — Pi → Supabase Realtime → https://amrslam.tech/control

Runs on the robot (RPi5) beside robot_dashboard_node.py. Reads the local SSE
feed and POSTs commands back to the dashboard HTTP API. Motion always goes
through that API (or the gamepad HTTP API). Both publish /cmd_vel, so
safety_stop stays in front of the motors. This process never publishes
/cmd_vel_safe.

Env (see /etc/amr-bridge.env — also loaded by amr_bridge.launch.py):
  AMR_BOT_ID=hardware-bot
  AMR_SUPABASE_URL=https://….supabase.co
  AMR_SUPABASE_KEY=…                  # publishable key only
  AMR_BRIDGE_TOKEN=…                  # shared secret → public.touch_amr_bot
  AMR_BRIDGE_EMAIL=…                  # bridge Auth user (private channels)
  AMR_BRIDGE_PASSWORD=…
  AMR_PUBLIC_CHANNEL=0                # 1 keeps legacy amr-bot-<id> during cut-over
  AMR_PRIVATE_CHANNEL=1               # bot:<id>:status|cmd|cmdq|ctl
  AMR_DASHBOARD_URL=http://127.0.0.1:8080
  AMR_CAMERA_URL=                     # optional public MJPEG (CF tunnel)
  AMR_CAMERA_JPEG_MAX_KB=24           # inline JPEG budget when scraping /camera.mjpg

AMR_SUPABASE_SERVICE_KEY is ignored. Do not set it on the robot.
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
from .control import (
    ControlTracker,
    MotionGuard,
    clamp_poll_interval,
    flag_enabled,
    looks_like_service_key,
    motion_command,
    sanitize_command,
)
from .protocol import (
    BOARD_RATE_HZ,
    BOT_IDS,
    BRIDGE_VERSION,
    CMD_EVENT,
    CTL_EVENT,
    FULL_RATE_HZ,
    STATUS_EVENTS,
    channel_name,
    private_topic,
)

log = logging.getLogger("amr_bridge")

# How often to check whether the bridge JWT should be refreshed.
AUTH_REFRESH_CHECK_S = 15.0
# Refresh this long before expires_at, then re-apply Realtime auth.
AUTH_REFRESH_SKEW_S = 90.0
# Command and zero posts. Short so a hung dashboard cannot pin the stop latch.
COMMAND_HTTP_TIMEOUT_S = 2.0
# Realtime callbacks waiting on that HTTP. Extra ones are closed, not queued.
MAX_SCHEDULED_COMMANDS = 4
# How soon a failed zero is tried again.
ZERO_RETRY_S = 0.25


def _env(name: str, default: str | None = None) -> str | None:
    v = os.environ.get(name, default)
    return v.strip() if isinstance(v, str) and v.strip() else default


class DashboardClient:
    def __init__(self, base: str):
        self.base = base.rstrip("/")
        self._http = httpx.AsyncClient(timeout=8.0)

    async def close(self) -> None:
        await self._http.aclose()

    async def post(
        self,
        path: str,
        payload: dict | None = None,
        *,
        timeout: float | None = None,
    ) -> dict:
        r = await self._http.post(
            f"{self.base}{path}",
            json=payload or {},
            timeout=timeout,
        )
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


def _create_supabase(url: str, key: str):
    """Publishable key only. Session is not written to disk."""
    from supabase import create_client

    options = None
    try:
        from supabase import ClientOptions
    except ImportError:
        try:
            from supabase.lib.client_options import ClientOptions  # type: ignore
        except ImportError:
            ClientOptions = None  # type: ignore
    if ClientOptions is not None:
        try:
            options = ClientOptions(auto_refresh_token=False, persist_session=False)
        except Exception:
            options = None
    if options is not None:
        try:
            return create_client(url, key, options=options)
        except TypeError:
            pass
    return create_client(url, key)


def _session_parts(res: Any) -> tuple[str, str | None, float | None]:
    session = getattr(res, "session", None)
    if session is None and isinstance(res, dict):
        session = res.get("session")
    if session is None:
        raise RuntimeError("bridge sign-in returned no session")

    def field(name: str) -> Any:
        if isinstance(session, dict):
            return session.get(name)
        return getattr(session, name, None)

    access = field("access_token")
    refresh = field("refresh_token")
    expires = field("expires_at")
    if not access:
        raise RuntimeError("bridge sign-in returned no access token")
    exp = float(expires) if expires else None
    return str(access), (str(refresh) if refresh else None), exp


class RealtimeHub:
    """Sync supabase-py wrapper. Call it from the asyncio loop via to_thread."""

    def __init__(
        self,
        url: str,
        key: str,
        bot_id: str,
        bridge_token: str | None,
        *,
        email: str | None,
        password: str | None,
        public_enabled: bool,
        private_enabled: bool,
        on_public_command,
        on_private_command,
        on_control,
        on_link,
    ):
        self._bot_id = bot_id
        self._bridge_token = bridge_token
        self._email = email
        self._password = password
        self._public_enabled = public_enabled
        self._private_enabled = private_enabled
        self._on_public_command = on_public_command
        self._on_private_command = on_private_command
        self._on_control = on_control
        self._on_link = on_link
        self._client = _create_supabase(url, key)
        self._channels: list[Any] = []
        self._public_channel = None
        self._status_channel = None
        self._public_ready = False
        self._private_ready = False
        self._signed_in = False
        self._access_token: str | None = None
        self._refresh_token: str | None = None
        self._expires_at = 0.0
        self._need_reauth = False
        self._closed = False

    def connect(self) -> None:
        if self._public_enabled:
            self._subscribe_public()
        if self._private_enabled:
            try:
                self._sign_in()
                self._subscribe_private()
            except Exception as exc:
                log.error("private channel setup failed: %s", self._redact(exc))

    def send(self, event: str, payload: dict) -> None:
        if event not in STATUS_EVENTS:
            return
        if self._public_ready and self._public_channel is not None:
            self._send_one(self._public_channel, event, payload)
        if self._private_ready and self._status_channel is not None:
            self._send_one(self._status_channel, event, payload)

    def touch_bot_row(self, camera_url: str | None) -> bool | None:
        """Heartbeat public.touch_amr_bot. Logs a false result. Never logs the token."""
        if not self._bridge_token:
            log.warning("touch_amr_bot skipped: AMR_BRIDGE_TOKEN is unset")
            return None
        try:
            res = self._client.rpc("touch_amr_bot", {
                "p_bot_id": self._bot_id,
                "p_token": self._bridge_token,
                "p_bridge_version": BRIDGE_VERSION,
                "p_camera_url": camera_url,
            }).execute()
        except Exception as exc:
            log.warning("touch_amr_bot failed: %s", type(exc).__name__)
            return None
        ok = res.data
        if isinstance(ok, list):
            ok = ok[0] if ok else None
        if ok is False:
            log.warning(
                "touch_amr_bot returned false for bot %s "
                "(bad token, or the server locked the RPC after 10 bad tokens this minute)",
                self._bot_id,
            )
            return False
        if ok is True:
            return True
        log.warning("touch_amr_bot returned an unexpected result")
        return None

    def poll_control(self) -> tuple[dict, dict | None]:
        control = (
            self._client.table("bot_control")
            .select("bot_id,controller,epoch,lease_expires_at")
            .eq("bot_id", self._bot_id)
            .limit(1)
            .execute()
        )
        token = (
            self._client.table("bot_control_tokens")
            .select("bot_id,epoch,token")
            .eq("bot_id", self._bot_id)
            .limit(1)
            .execute()
        )
        rows = control.data or []
        if not rows:
            raise RuntimeError("bot_control returned no row")
        token_rows = token.data or []
        return rows[0], (token_rows[0] if token_rows else None)

    def refresh_auth_if_due(self) -> bool:
        """Refresh the bridge JWT when it is near expiry and re-apply Realtime auth.

        Also retries sign-in after a failed startup. Returns True when a new
        access token was applied.
        """
        if not self._private_enabled or self._closed:
            return False
        if not self._signed_in:
            self._sign_in()
        if not self._private_ready:
            self._subscribe_private()
            return True
        now = time.time()
        if self._expires_at and now < self._expires_at - AUTH_REFRESH_SKEW_S:
            return False
        if self._refresh_token:
            res = self._client.auth.refresh_session(self._refresh_token)
        else:
            res = self._client.auth.refresh_session()
        self._store_session(res)
        self._apply_realtime_auth(self._access_token or "")
        log.info("bridge auth refreshed; realtime auth re-applied")
        return True

    def close(self) -> None:
        self._closed = True
        for channel in self._channels:
            try:
                self._client.remove_channel(channel)
            except Exception:
                pass
        self._channels.clear()

    def _subscribe_public(self) -> None:
        topic = channel_name(self._bot_id)
        channel = self._client.channel(topic)
        self._bind_broadcast(channel, CMD_EVENT, self._public_cb)
        channel.subscribe()
        self._public_channel = channel
        self._channels.append(channel)
        self._public_ready = True
        log.info("realtime subscribed (%s)", topic)

    def _subscribe_private(self) -> None:
        if self._private_ready or self._closed:
            return
        if not self._access_token:
            raise RuntimeError("private subscribe without a bridge access token")
        self._apply_realtime_auth(self._access_token)
        specs = (
            ("cmd", CMD_EVENT, self._cmd_cb),
            ("cmdq", CMD_EVENT, self._cmdq_cb),
            ("ctl", CTL_EVENT, self._ctl_cb),
            ("status", None, None),
        )
        made = []
        try:
            for kind, event, callback in specs:
                topic = private_topic(self._bot_id, kind)
                channel = self._open_private_channel(topic)
                if event and callback:
                    self._bind_broadcast(channel, event, callback)
                try:
                    channel.subscribe(self._status_cb)
                except TypeError:
                    channel.subscribe()
                made.append(channel)
                if kind == "status":
                    self._status_channel = channel
        except Exception:
            for channel in made:
                try:
                    self._client.remove_channel(channel)
                except Exception:
                    pass
            self._status_channel = None
            raise
        self._channels.extend(made)
        self._private_ready = True
        log.info(
            "realtime subscribed private (%s)",
            ", ".join(private_topic(self._bot_id, kind) for kind, _, _ in specs),
        )

    def _open_private_channel(self, topic: str):
        params = {"config": {"private": True}}
        try:
            return self._client.channel(topic, params)
        except TypeError:
            return self._client.channel(topic, params=params)

    def _bind_broadcast(self, channel, event: str, callback) -> None:
        try:
            channel.on_broadcast(event=event, callback=callback)
        except Exception:
            channel.on("broadcast", {"event": event}, callback)

    def _public_cb(self, message) -> None:
        payload = _unwrap_callback(message)
        if isinstance(payload, dict):
            self._on_public_command(payload)

    def _cmd_cb(self, message) -> None:
        payload = _unwrap_callback(message)
        if isinstance(payload, dict):
            self._on_private_command(payload, "cmd")

    def _cmdq_cb(self, message) -> None:
        payload = _unwrap_callback(message)
        if isinstance(payload, dict):
            self._on_private_command(payload, "cmdq")

    def _ctl_cb(self, message) -> None:
        payload = _unwrap_callback(message)
        if isinstance(payload, dict):
            self._on_control(payload)

    def _status_cb(self, *args) -> None:
        status = args[0] if args else ""
        name = str(getattr(status, "value", status) or "")
        if "SUBSCRIBED" in name:
            self._reapply_if_needed()
            if self._on_link:
                self._on_link(True)
            return
        if any(token in name for token in ("CLOSED", "CHANNEL_ERROR", "TIMED_OUT")):
            self._need_reauth = True
            log.warning("realtime private channel %s", name)

    def _reapply_if_needed(self) -> None:
        if not self._need_reauth or not self._access_token:
            return
        self._need_reauth = False
        try:
            self._apply_realtime_auth(self._access_token)
            log.info("realtime auth re-applied after reconnect")
        except Exception as exc:
            self._need_reauth = True
            log.warning("realtime auth re-apply failed: %s", type(exc).__name__)

    def _sign_in(self) -> None:
        if not self._email or not self._password:
            raise RuntimeError("AMR_BRIDGE_EMAIL and AMR_BRIDGE_PASSWORD are required")
        res = self._client.auth.sign_in_with_password({
            "email": self._email,
            "password": self._password,
        })
        self._store_session(res)
        self._apply_realtime_auth(self._access_token or "")
        self._signed_in = True
        log.info("bridge auth signed in")

    def _store_session(self, res: Any) -> None:
        access, refresh, expires = _session_parts(res)
        self._access_token = access
        if refresh:
            self._refresh_token = refresh
        self._expires_at = expires if expires else time.time() + 3600
        self._signed_in = True

    def _apply_realtime_auth(self, access_token: str) -> None:
        if not access_token:
            raise RuntimeError("missing bridge access token")
        realtime = self._client.realtime
        connect = getattr(realtime, "connect", None)
        if callable(connect):
            try:
                connect()
            except Exception as exc:
                text = str(exc).lower()
                if "already" not in text and "connected" not in text:
                    log.debug("realtime connect before set_auth: %s", type(exc).__name__)
        set_auth = getattr(realtime, "set_auth", None)
        if not callable(set_auth):
            raise RuntimeError("supabase realtime client has no set_auth")
        try:
            set_auth(access_token)
        except Exception as exc:
            text = str(exc).lower()
            if callable(connect) and "connect" in text:
                connect()
                set_auth(access_token)
                return
            raise

    def _send_one(self, channel, event: str, payload: dict) -> None:
        try:
            channel.send_broadcast(event, payload)
        except Exception:
            try:
                channel.send({"type": "broadcast", "event": event, "payload": payload})
            except Exception as exc:
                log.debug("broadcast %s failed: %s", event, type(exc).__name__)

    def _redact(self, exc: BaseException) -> str:
        text = f"{type(exc).__name__}: {exc}"
        if self._password and self._password in text:
            text = text.replace(self._password, "[redacted]")
        if self._access_token and self._access_token in text:
            text = text.replace(self._access_token, "[redacted]")
        return text[:400]


def _unwrap_callback(message: Any) -> Any:
    """Return the broadcast body. A missing payload keeps the message itself."""
    if isinstance(message, str):
        try:
            message = json.loads(message)
        except json.JSONDecodeError:
            return None
    if not isinstance(message, dict):
        return message
    payload = message.get("payload")
    if isinstance(payload, str):
        try:
            payload = json.loads(payload)
        except json.JSONDecodeError:
            return None
    if isinstance(payload, dict):
        return payload
    return message


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
        *,
        bridge_email: str | None = None,
        bridge_password: str | None = None,
        public_channel: bool = False,
        private_channel: bool = True,
        control_poll_s: float = 2.0,
    ):
        if bot_id not in BOT_IDS:
            raise SystemExit(f"AMR_BOT_ID must be one of {BOT_IDS}")
        self.bot_id = bot_id
        self.supabase_url = supabase_url.rstrip("/")
        self.api_key = api_key
        self.bridge_token = bridge_token
        self.bridge_email = bridge_email
        self._bridge_password = bridge_password
        self.public_channel = public_channel
        self.private_channel = private_channel and bool(bridge_email) and bool(bridge_password)
        if private_channel and not self.private_channel:
            log.warning(
                "private channels skipped: set AMR_BRIDGE_EMAIL and AMR_BRIDGE_PASSWORD "
                "to sign in as the bridge user"
            )
        if not self.public_channel and not self.private_channel:
            log.warning("no Realtime channel is enabled; telemetry will not be published")
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
        self._gate = ControlTracker(bot_id)
        self._guard = MotionGuard()
        self._poll_s = clamp_poll_interval(control_poll_s)
        self._poll_lock = asyncio.Lock()
        self._io_lock = asyncio.Lock()
        self._sched_lock = threading.Lock()
        self._scheduled = 0
        self._last_zero_mono = 0.0

    def _queue_public(self, env: dict) -> None:
        self._schedule(self._handle_public_command(env))

    def _queue_private(self, env: dict, source: str) -> None:
        self._schedule(self._handle_private_command(env, source))

    def _queue_control(self, env: dict) -> None:
        self._schedule(self._handle_control(env))

    def _queue_link(self, up: bool) -> None:
        # Subscribe is not contact. note_contact runs after a poll or a command.
        del up

    def _schedule(self, coro) -> None:
        """Hop a Realtime callback onto the bridge loop, with a hard cap.

        A hung HTTP call used to leave every later callback queued behind it.
        Past the cap the coroutine is closed and dropped.
        """
        if not (self._loop and self._loop.is_running()):
            coro.close()
            return
        with self._sched_lock:
            if self._scheduled >= MAX_SCHEDULED_COMMANDS:
                coro.close()
                log.warning(
                    "dropping realtime callback; %s already scheduled",
                    MAX_SCHEDULED_COMMANDS,
                )
                return
            self._scheduled += 1

        async def _bounded() -> None:
            try:
                await coro
            finally:
                with self._sched_lock:
                    self._scheduled -= 1

        asyncio.run_coroutine_threadsafe(_bounded(), self._loop)

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._hub = RealtimeHub(
            self.supabase_url,
            self.api_key,
            self.bot_id,
            self.bridge_token,
            email=self.bridge_email,
            password=self._bridge_password,
            public_enabled=self.public_channel,
            private_enabled=self.private_channel,
            on_public_command=self._queue_public,
            on_private_command=self._queue_private,
            on_control=self._queue_control,
            on_link=self._queue_link,
        )
        await asyncio.to_thread(self._hub.connect)
        tasks = [
            self._sse_loop(),
            self._heartbeat_loop(),
            self._presence_touch_loop(),
            self._watchdog_loop(),
        ]
        if self.private_channel:
            tasks.append(self._control_poll_loop())
            tasks.append(self._auth_refresh_loop())
        await asyncio.gather(*tasks)

    async def _handle_control(self, env: dict) -> None:
        # :ctl carries no token and no lease. Poll is the only authority.
        if self._gate.ctl_needs_poll(env):
            await self._poll_once()

    async def _handle_public_command(self, env: dict) -> None:
        # Legacy public topic. Rank gates stay until cut-over turns this path off.
        if env.get("botId") not in (self.bot_id, None):
            return
        cmd = env.get("cmd") or {}
        typ = cmd.get("type")
        rank = int((env.get("from") or {}).get("rank") or 0)
        log.info("command %s from rank=%s", typ, rank)
        try:
            if typ == "set_rate":
                clean, reason = sanitize_command(cmd)
                if clean is None:
                    log.info("rejected public command type=%s: %s", typ, reason)
                    return
                self._apply_rate(clean)
                return
            if rank < 1:
                return
            if typ == "save_map" and rank < 2:
                return
            clean, reason = sanitize_command(cmd)
            if clean is None:
                log.info("rejected public command type=%s: %s", typ, reason)
                return
            await self._run_accepted(clean)
        except Exception as exc:
            log.warning("command failed: %s", exc)

    async def _handle_private_command(self, env: dict, source: str) -> None:
        now_s = time.time()
        now_mono = time.monotonic()
        result = self._gate.verify(env, source=source, now_s=now_s, now_mono=now_mono)
        if not result.ok:
            log.info("rejected %s command type=%s: %s", source, result.cmd_type, result.reason)
            return
        log.info("accepted %s command type=%s", source, result.cmd_type)
        cmd = result.cmd or {}
        try:
            if cmd.get("type") == "set_rate":
                self._apply_rate(cmd)
                return
            await self._run_accepted(cmd)
        except Exception as exc:
            log.warning("command failed: %s", exc)

    async def _run_accepted(self, cmd: dict) -> None:
        """Post one accepted command. A hard stop zeroes again after motion."""
        if motion_command(cmd) and not self._gate.fault_active(time.time(), time.monotonic()):
            # Contact just succeeded, or the lease is still open: this motion
            # supersedes a deadman zero. A hard fault stays latched.
            self._guard.set_hard(False)
            self._guard.release_for_motion()
        await self._dispatch_cmd(cmd)

    def _apply_rate(self, cmd: dict) -> None:
        mode = cmd.get("mode")
        if mode in ("full", "board"):
            self.mode = mode
            log.info("rate mode → %s", mode)

    async def _dispatch_cmd(self, cmd: dict) -> None:
        """Forward a command through the dashboard / gamepad HTTP APIs.

        Those processes publish /cmd_vel. safety_stop is the only publisher of
        /cmd_vel_safe. Do not add a publisher here.
        """
        typ = cmd.get("type")
        timeout = COMMAND_HTTP_TIMEOUT_S
        async with self._io_lock:
            if typ == "estop":
                await self.dash.post("/api/estop", timeout=timeout)
            elif typ in ("nudge", "drive"):
                await self.dash.post("/api/nudge", {
                    "lin": float(cmd.get("lin", 0)),
                    "ang": float(cmd.get("ang", 0)),
                }, timeout=timeout)
            elif typ == "joy":
                await self._post_joy(cmd, timeout)
            elif typ == "nav_goal":
                await self.dash.post("/api/nav_goal", {
                    "x": float(cmd.get("x", 0)),
                    "y": float(cmd.get("y", 0)),
                }, timeout=timeout)
            elif typ == "nav_cancel":
                await self.dash.post("/api/nav_cancel", timeout=timeout)
            elif typ == "vision":
                await self.dash.post("/api/vision", {"on": bool(cmd.get("on"))}, timeout=timeout)
            elif typ == "save_map":
                await self.dash.post("/api/save_map", {"name": cmd.get("name")}, timeout=timeout)
            elif typ == "run_tool":
                await self.dash.post("/api/run", {
                    "tool": cmd.get("tool"),
                    "params": cmd.get("params") or {},
                }, timeout=timeout)
            elif typ == "stop_tool":
                await self.dash.post("/api/stop", timeout=timeout)
            elif typ == "blacklist":
                if cmd.get("action") == "clear":
                    await self.dash.post("/api/blacklist", {"action": "clear"}, timeout=timeout)
                else:
                    await self.dash.post("/api/blacklist", {
                        "x": float(cmd.get("x", 0)),
                        "y": float(cmd.get("y", 0)),
                        "radius": float(cmd.get("radius", 0.8)),
                        "duration": float(cmd.get("duration", 300)),
                    }, timeout=timeout)
            if motion_command(cmd) and (self._guard.hard or self._guard.owed):
                log.warning("watchdog: zero motion (after command)")
                owed = self._guard.owed_gen
                if await self._post_zero():
                    if owed is not None:
                        self._guard.mark_zeroed(owed)

    async def _post_joy(self, cmd: dict, timeout: float) -> None:
        """Caller holds ``_io_lock``. ``a``/``b`` win over ``axes``/``buttons``."""
        gamepad = os.environ.get("AMR_GAMEPAD_URL", "").rstrip("/")
        if "a" in cmd and cmd.get("a") is not None:
            axes = list(cmd["a"])
        else:
            axes = list(cmd.get("axes") or [])
        if "b" in cmd and cmd.get("b") is not None:
            buttons = int(cmd["b"])
        else:
            buttons = int(cmd["buttons"]) if cmd.get("buttons") is not None else 0
        if gamepad:
            response = await self.dash._http.post(
                f"{gamepad}/api/joy",
                json={"a": axes[:8], "b": buttons},
                timeout=timeout,
            )
            response.raise_for_status()
            return
        # Differential approx from left stick: axes[1]=fwd, axes[0]=yaw.
        ax = axes + [0.0] * 8
        lin = max(-0.3, min(0.3, -float(ax[1]) * 0.35))
        ang = max(-1.0, min(1.0, -float(ax[0]) * 0.9))
        if abs(lin) > 0.02 or abs(ang) > 0.02:
            await self.dash.post("/api/nudge", {"lin": lin, "ang": ang}, timeout=timeout)

    async def _post_zero(self) -> bool:
        """Caller holds ``_io_lock``. False means the watchdog must try again.

        A zero nudge streams Twist zeros on /cmd_vel. A zero joy packet makes
        joy_teleop publish one zero Twist and then yield. Neither path writes
        /cmd_vel_safe.
        """
        timeout = COMMAND_HTTP_TIMEOUT_S
        try:
            await self.dash.post("/api/nudge", {"lin": 0.0, "ang": 0.0}, timeout=timeout)
        except Exception as exc:
            log.warning("watchdog zero nudge failed: %s", exc)
            return False
        gamepad = os.environ.get("AMR_GAMEPAD_URL", "").rstrip("/")
        if not gamepad:
            return True
        try:
            response = await self.dash._http.post(
                f"{gamepad}/api/joy",
                json={"a": [0.0] * 8, "b": 0},
                timeout=timeout,
            )
            response.raise_for_status()
        except Exception as exc:
            log.warning("watchdog zero joy failed: %s", exc)
            return False
        return True

    async def _try_zero(self, gen: int, reason: str) -> None:
        async with self._io_lock:
            if not self._guard.still_owed(gen):
                return
            log.warning("watchdog: zero motion (%s)", reason)
            if await self._post_zero():
                self._guard.mark_zeroed(gen)

    async def _apply_watchdog(self, reasons: list[str]) -> None:
        if not reasons:
            return
        hard = any(reason in ("lease_expired", "supabase_unreachable") for reason in reasons)
        gen = self._guard.arm(hard=hard)
        self._last_zero_mono = time.monotonic()
        await self._try_zero(gen, ",".join(reasons))

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

    async def _watchdog_loop(self) -> None:
        while not self._stop.is_set():
            now_s = time.time()
            now_mono = time.monotonic()
            reasons = self._gate.tick(now_s, now_mono)
            self._guard.set_hard(self._gate.fault_active(now_s, now_mono))
            gen: int | None = None
            reason = ""
            if reasons:
                hard = self._guard.hard or any(
                    item in ("lease_expired", "supabase_unreachable") for item in reasons
                )
                gen = self._guard.arm(hard=hard)
                reason = ",".join(reasons)
                self._last_zero_mono = 0.0
            elif self._guard.owed and now_mono - self._last_zero_mono >= ZERO_RETRY_S:
                gen = self._guard.owed_gen
                reason = "retry"
            if gen is not None:
                self._last_zero_mono = now_mono
                await self._try_zero(gen, reason)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=0.05)
            except asyncio.TimeoutError:
                pass

    async def _control_poll_loop(self) -> None:
        while not self._stop.is_set():
            await self._poll_once()
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=self._poll_s)
            except asyncio.TimeoutError:
                pass

    async def _poll_once(self) -> None:
        if not self._hub or self._stop.is_set():
            return
        async with self._poll_lock:
            try:
                control, token = await asyncio.to_thread(self._hub.poll_control)
            except Exception as exc:
                log.warning("control poll failed: %s", type(exc).__name__)
                return
            # The query itself is contact, even when the row is empty or stale.
            now_mono = time.monotonic()
            self._gate.note_contact(now_mono)
            reasons = self._gate.apply_poll(control, token, now_mono)
            await self._apply_watchdog(reasons)

    async def _auth_refresh_loop(self) -> None:
        while not self._stop.is_set():
            if self._hub:
                try:
                    await asyncio.to_thread(self._hub.refresh_auth_if_due)
                except Exception as exc:
                    log.warning("bridge auth refresh failed: %s", type(exc).__name__)
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=AUTH_REFRESH_CHECK_S)
            except asyncio.TimeoutError:
                pass

    async def stop(self) -> None:
        self._stop.set()
        await self.dash.close()
        if self._hub:
            await asyncio.to_thread(self._hub.close)


def main(argv: list[str] | None = None) -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if _env("AMR_SUPABASE_SERVICE_KEY"):
        log.warning(
            "AMR_SUPABASE_SERVICE_KEY is set and is ignored. "
            "Use AMR_SUPABASE_KEY (the publishable key) only."
        )
    p = argparse.ArgumentParser(description="AMR cloud telemetry bridge")
    p.add_argument("--bot-id", default=_env("AMR_BOT_ID", "hardware-bot"))
    p.add_argument("--supabase-url", default=_env("AMR_SUPABASE_URL") or _env("NEXT_PUBLIC_SUPABASE_URL"))
    p.add_argument(
        "--api-key",
        default=_env("AMR_SUPABASE_KEY") or _env("NEXT_PUBLIC_SUPABASE_PUBLISHABLE_KEY"),
    )
    p.add_argument("--bridge-token", default=_env("AMR_BRIDGE_TOKEN"))
    p.add_argument("--bridge-email", default=_env("AMR_BRIDGE_EMAIL"))
    p.add_argument("--bridge-password", default=_env("AMR_BRIDGE_PASSWORD"))
    p.add_argument("--public-channel", default=_env("AMR_PUBLIC_CHANNEL", "0"))
    p.add_argument("--private-channel", default=_env("AMR_PRIVATE_CHANNEL", "1"))
    p.add_argument(
        "--control-poll-s",
        type=float,
        default=float(_env("AMR_CONTROL_POLL_S", "2") or 2),
    )
    p.add_argument("--dashboard", default=_env("AMR_DASHBOARD_URL", "http://127.0.0.1:8080"))
    p.add_argument("--camera-url", default=_env("AMR_CAMERA_URL"))
    p.add_argument("--camera-jpeg-max-kb", type=int, default=int(_env("AMR_CAMERA_JPEG_MAX_KB", "24") or 24))
    args = p.parse_args(argv)

    if not args.supabase_url or not args.api_key:
        raise SystemExit("AMR_SUPABASE_URL and AMR_SUPABASE_KEY are required")
    if looks_like_service_key(args.api_key):
        raise SystemExit(
            "Refusing to start: AMR_SUPABASE_KEY looks like a service key "
            "(sb_secret_ prefix or a JWT with role service_role). "
            "Use the publishable key only."
        )

    bridge = Bridge(
        bot_id=args.bot_id,
        supabase_url=args.supabase_url,
        api_key=args.api_key,
        bridge_token=args.bridge_token,
        dashboard_url=args.dashboard,
        camera_url=args.camera_url,
        camera_jpeg_max_kb=args.camera_jpeg_max_kb,
        bridge_email=args.bridge_email,
        bridge_password=args.bridge_password,
        public_channel=flag_enabled(str(args.public_channel) if args.public_channel is not None else None, False),
        private_channel=flag_enabled(str(args.private_channel) if args.private_channel is not None else None, True),
        control_poll_s=args.control_poll_s,
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
