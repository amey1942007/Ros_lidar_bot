#!/usr/bin/env python3
"""Cloud smoke test: publish a FULL demo telemetry suite to Supabase Realtime.

Sends every event/field the control panel understands:
  - hb
  - telemetry (pose/odom, vel, cmd, imu, scan, rates, nodes, tf, nav, path,
    goal, yoloDet, sem, frontier, sys, tool, vision, …)
  - map (zlib+base64 occupancy grid)
  - log

Also calls touch_amr_bot so amr_bots.last_seen_at / online update.

Env: AMR_SUPABASE_URL, AMR_SUPABASE_KEY, AMR_BRIDGE_TOKEN, AMR_BOT_ID,
     AMR_DEMO_SECONDS (default 30)
"""

from __future__ import annotations

import asyncio
import base64
import math
import os
import sys
import time
import zlib

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from amr_telemetry_bridge.protocol import BRIDGE_VERSION, STATUS_EVENTS, channel_name


def _env(name: str, default: str | None = None) -> str | None:
    v = os.environ.get(name, default)
    return v.strip() if isinstance(v, str) and v.strip() else default


def _fake_scan(n: int = 90, radius: float = 2.5) -> list[list[float]]:
    pts = []
    for i in range(n):
        ang = (2 * math.pi * i) / n
        r = radius + 0.3 * math.sin(3 * ang)
        pts.append([round(ang, 3), round(r, 2)])
    return pts


def _fake_map() -> dict:
    """Small 32x32 occupancy grid: unknown border, free center, occupied wall."""
    w = h = 32
    cells = []
    for y in range(h):
        for x in range(w):
            if x == 0 or y == 0 or x == w - 1 or y == h - 1:
                cells.append(0)  # unknown
            elif 10 <= x <= 12:
                cells.append(2)  # occupied wall
            else:
                cells.append(1)  # free
    packed = base64.b64encode(zlib.compress(bytes(cells), 6)).decode()
    return {
        "t": "map",
        "w": w,
        "h": h,
        "res": 0.05,
        "origin": [-0.8, -0.8],
        "tf": [0.0, 0.0, 0.0],
        "data": packed,
    }


def _full_telemetry(bot_id: str, seq: int, now_ms: int) -> dict:
    ang = (seq * 0.12) % (2 * math.pi)
    x, y = round(math.cos(ang), 3), round(math.sin(ang), 3)
    include_scan = seq % 2 == 1  # alternate like the real bridge
    return {
        "ts": now_ms,
        "botId": bot_id,
        "seq": seq,
        "mode": "full",
        "online": True,
        "pose": [x, y, round(ang, 4)],
        "mapPose": [x, y, round(ang, 4)],
        "vel": [0.12, 0.05],
        "cmd": [0.12, 0.05],
        "imu": {"gz": round(0.05 * math.sin(ang), 4), "ax": 0.01, "ay": -0.02},
        "scan": _fake_scan() if include_scan else None,
        "rates": {
            "/scan": {"hz": 8.5, "age": 0.05},
            "/odom": {"hz": 50.0, "age": 0.01},
            "/odom_raw": {"hz": 50.0, "age": 0.01},
            "/imu": {"hz": 100.0, "age": 0.0},
            "/map": {"hz": 0.5, "age": 1.2},
            "/cmd_vel": {"hz": 20.0, "age": 0.02},
        },
        "nodes": {
            "driver_node": True,
            "odom_node": True,
            "imu_node": True,
            "sllidar_node": True,
            "ekf_filter_node": True,
            "slam_toolbox": True,
            "safety_stop": True,
            "amr_telemetry_bridge": True,
        },
        "tf": {
            "map→odom": {"ok": True, "age": 0.0},
            "odom→base_footprint": {"ok": True, "age": 0.0},
            "base_footprint→laser_frame": {"ok": True, "age": 0.0},
        },
        "nav": {"status": "idle", "feedback": "demo path"},
        "path": [[round(math.cos(ang + i * 0.1), 2), round(math.sin(ang + i * 0.1), 2)] for i in range(12)],
        "goal": [1.5, 0.5],
        "yoloDet": ["person 0.82", "chair 0.61"],
        "sem": [[0.5, 0.2, "door"], [-0.3, 0.8, "table"]],
        "frontier": {
            "zones": [[1.0, 1.0, 0.4, "frontier"], [-1.0, 0.5, 0.3, "frontier"]],
            "queue": [[1.0, 1.0], [-1.0, 0.5]],
        },
        "sys": {
            "cpu": 27.5,
            "load": 0.8,
            "mem": [420, 8192],
            "temp": 48.0,
            "disk": [12, 58],
            "up": "0:12",
            "throttled": 0,
        },
        "tool": None,
        "vision": True,
        "saving": False,
        "cameraUrl": None,
        "seqGoals": {"goals": [[1.5, 0.5], [2.0, -0.5]], "idx": 0},
    }


async def main() -> int:
    url = _env("AMR_SUPABASE_URL")
    key = _env("AMR_SUPABASE_KEY")
    token = _env("AMR_BRIDGE_TOKEN")
    bot_id = _env("AMR_BOT_ID", "hardware-bot") or "hardware-bot"
    seconds = float(_env("AMR_DEMO_SECONDS", "30") or "30")
    if not url or not key or not token:
        print("Need AMR_SUPABASE_URL, AMR_SUPABASE_KEY, AMR_BRIDGE_TOKEN", file=sys.stderr)
        return 2

    from realtime import AsyncRealtimeClient
    from supabase import create_client

    rest = create_client(url, key)
    topic = channel_name(bot_id)
    print(f"bot={bot_id} topic={topic}")

    touch = rest.rpc(
        "touch_amr_bot",
        {
            "p_bot_id": bot_id,
            "p_token": token,
            "p_bridge_version": f"demo-{BRIDGE_VERSION}",
            "p_camera_url": None,
        },
    ).execute()
    print("touch_amr_bot →", touch.data)
    if touch.data is not True:
        print("touch_amr_bot FAILED", file=sys.stderr)
        return 3

    received: dict[str, int] = {}
    rt_url = url.rstrip("/") + "/realtime/v1"
    pub = AsyncRealtimeClient(rt_url, token=key, auto_reconnect=False, max_retries=1)
    sub = AsyncRealtimeClient(rt_url, token=key, auto_reconnect=False, max_retries=1)

    listen = sub.channel(
        topic,
        {"config": {"broadcast": {"ack": False, "self": True}, "private": False}},
    )

    def _on(name: str):
        def _cb(_msg):
            received[name] = received.get(name, 0) + 1
            print(f"  ← got {name} (total {received[name]})")
        return _cb

    for ev in STATUS_EVENTS:
        try:
            listen.on_broadcast(event=ev, callback=_on(ev))
        except Exception:
            listen.on("broadcast", {"event": ev}, _on(ev))

    sub_ok = asyncio.Event()
    pub_ok = asyncio.Event()

    def _status(ev: asyncio.Event, label: str):
        def _cb(status, err=None):
            name = str(getattr(status, "value", status)).upper()
            print(f"  {label} status={name} err={err}")
            if name == "SUBSCRIBED":
                ev.set()
            if name in ("CHANNEL_ERROR", "TIMED_OUT", "CLOSED"):
                print(f"  {label} DOWN: {name}", file=sys.stderr)
        return _cb

    await listen.subscribe(_status(sub_ok, "listener"))
    try:
        await asyncio.wait_for(sub_ok.wait(), timeout=20)
    except asyncio.TimeoutError:
        print("FAIL: listener subscribe timeout (Realtime public access?)", file=sys.stderr)
        return 4

    pub_ch = pub.channel(
        topic,
        {"config": {"broadcast": {"ack": False, "self": False}, "private": False}},
    )
    await pub_ch.subscribe(_status(pub_ok, "publisher"))
    try:
        await asyncio.wait_for(pub_ok.wait(), timeout=20)
    except asyncio.TimeoutError:
        print("FAIL: publisher subscribe timeout", file=sys.stderr)
        return 5

    # One map + one log up front
    map_payload = _fake_map()
    map_payload["botId"] = bot_id
    map_payload["ts"] = int(time.time() * 1000)
    await pub_ch.send_broadcast("map", map_payload)
    print("  → sent map")
    await pub_ch.send_broadcast("log", {
        "botId": bot_id,
        "ts": int(time.time() * 1000),
        "time": time.strftime("%H:%M:%S"),
        "level": "info",
        "text": "cloud demo: full telemetry suite starting",
    })
    print("  → sent log")

    t0 = time.monotonic()
    seq = 0
    sent = {"hb": 0, "telemetry": 0, "map": 1, "log": 1}
    while time.monotonic() - t0 < seconds:
        seq += 1
        now_ms = int(time.time() * 1000)
        await pub_ch.send_broadcast("hb", {
            "t": "hb",
            "botId": bot_id,
            "ts": now_ms,
            "bridgeVersion": f"demo-{BRIDGE_VERSION}",
            "public": True,
            "private": False,
            "seq": seq,
        })
        sent["hb"] += 1
        await pub_ch.send_broadcast("telemetry", _full_telemetry(bot_id, seq, now_ms))
        sent["telemetry"] += 1
        if seq % 8 == 0:
            map_payload = _fake_map()
            map_payload["botId"] = bot_id
            map_payload["ts"] = now_ms
            await pub_ch.send_broadcast("map", map_payload)
            sent["map"] += 1
            await pub_ch.send_broadcast("log", {
                "botId": bot_id,
                "ts": now_ms,
                "time": time.strftime("%H:%M:%S"),
                "level": "run",
                "text": f"demo frame seq={seq} pose rotating",
            })
            sent["log"] += 1
            rest.rpc(
                "touch_amr_bot",
                {
                    "p_bot_id": bot_id,
                    "p_token": token,
                    "p_bridge_version": f"demo-{BRIDGE_VERSION}",
                    "p_camera_url": None,
                },
            ).execute()
        await asyncio.sleep(0.5)

    await asyncio.sleep(2.0)
    # Final presence touch (amr_bots SELECT is authenticated-only; RPC is the probe).
    final_touch = rest.rpc(
        "touch_amr_bot",
        {
            "p_bot_id": bot_id,
            "p_token": token,
            "p_bridge_version": f"demo-{BRIDGE_VERSION}",
            "p_camera_url": None,
        },
    ).execute()
    print("sent", sent)
    print("received", received)
    print("final touch_amr_bot →", final_touch.data)

    for client, ch in ((pub, pub_ch), (sub, listen)):
        try:
            await ch.unsubscribe()
        except Exception:
            pass
        try:
            await client.close()
        except Exception:
            pass

    need = ("hb", "telemetry", "map", "log")
    missing = [e for e in need if received.get(e, 0) < 1]
    if missing:
        print("FAIL: missing events:", missing, file=sys.stderr)
        return 6
    if final_touch.data is not True:
        print("FAIL: final touch_amr_bot not true", file=sys.stderr)
        return 7
    if received.get("telemetry", 0) < 5 or received.get("hb", 0) < 5:
        print("FAIL: too few telemetry/hb frames", file=sys.stderr)
        return 8
    print("SUCCESS: full demo suite reached Supabase Realtime + touch_amr_bot")
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
