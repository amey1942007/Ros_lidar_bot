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
  AMR_PUBLIC_CHANNEL=0                # 1 reopens the legacy topic (spoofable rank, no deadman)
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
import inspect
import json
import logging
import math
import os
import re
import signal
import threading
import time
from collections import deque
from typing import Any, Optional

import httpx

from . import __version__
from .control import (
    ControlTracker,
    MotionGuard,
    clamp_poll_interval,
    flag_enabled,
    MOTION_WATCHDOG_S,
    command_seq,
    looks_like_service_key,
    motion_command,
    peek_command_type,
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
from .ros_source import RosSource, resolve_source

log = logging.getLogger("amr_bridge")

# How often to check whether the bridge JWT should be refreshed.
AUTH_REFRESH_CHECK_S = 15.0
# Refresh this long before expires_at, then re-apply Realtime auth.
AUTH_REFRESH_SKEW_S = 90.0
# Command and zero posts. Short so a hung dashboard cannot pin the stop latch.
COMMAND_HTTP_TIMEOUT_S = 2.0
# Realtime callbacks waiting on that HTTP. E-stop and :ctl are not counted.
# Joy and nudge do not use this queue: they share one latest-wins slot.
MAX_SCHEDULED_COMMANDS = 4
_MOTION_SLOT_TYPES = ("joy", "nudge", "drive")
# How soon a failed zero is tried again.
ZERO_RETRY_S = 0.25
# Bound the unverified e-stop queue. Older entries fall off the left.
ESTOP_QUEUE_MAX = 16
# /api/estop does not latch. The dashboard streams zeros for 1.5 s, cancels
# nav, and kills the tool; the gamepad then keeps publishing the last axes
# for 1.5 s. Ignore stick commands for this long after an accepted e-stop
# unless their seq is newer than the e-stop's.
ESTOP_MOTION_HOLD_S = 1.0
# PostgREST default is 120s. A hung touch_amr_bot RPC must not stall the 5s loop.
RPC_TIMEOUT_S = 8.0
# Broadcasts are best-effort. Do not sit on a dead socket.
REALTIME_SEND_TIMEOUT_S = 2.0
# Per-channel CLOSED / CHANNEL_ERROR / TIMED_OUT waits this long, then doubles.
# A dead socket still reconnects every channel. One channel does not.
RECONNECT_MIN_S = 1.0
RECONNECT_MAX_S = 30.0
_DOWN_STATES = ("CLOSED", "CHANNEL_ERROR", "TIMED_OUT")
# Same private join for :cmd, :cmdq, :ctl, and :status. broadcast is explicit
# so the payload is not "broadcast": null (omitted config replaces defaults).
_PRIVATE_CHANNEL_PARAMS = {
    "config": {
        "broadcast": {"ack": False, "self": False},
        "private": True,
    },
}
# Public telemetry join. Explicit so it does not depend on library defaults.
_PUBLIC_CHANNEL_PARAMS = {
    "config": {
        "broadcast": {"ack": False, "self": False},
        "private": False,
    },
}
# First send failure of each exception type, then quiet for this long.
SEND_FAIL_LOG_S = 30.0
# Optional REST broadcast ceiling (AMR_PUBLIC_REST=1).
REST_BROADCAST_MIN_S = 0.2
REST_BROADCAST_TIMEOUT_S = 2.0
# Oversize warning, once per kind per minute.
OVERSIZE_LOG_S = 60.0
STATS_LOG_S = 10.0
SCAN_MAX_POINTS_DEFAULT = 90
SCAN_HZ_DEFAULT = 5
TELEMETRY_MAX_BYTES_DEFAULT = 200_000
_JWT_RE = re.compile(r"eyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}")
_SECRET_FIELD_RE = re.compile(
    r"(?i)("
    r'"?(?:access_token|refresh_token|lease_token|bridge_token|api_key|apikey|'
    r'authorization|p_token|password|jwt|token)"?'
    r"\s*[:=]\s*"
    r")"
    r'(?:"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[^\s,}\]]+)'
)


def _public_rank(env: Any) -> int:
    """Rank on a public envelope. A non-dict ``from`` or a non-numeric rank is 0."""
    if not isinstance(env, dict):
        return 0
    sender = env.get("from")
    if not isinstance(sender, dict):
        return 0
    raw = sender.get("rank")
    if isinstance(raw, bool) or raw is None:
        return 0
    if isinstance(raw, int):
        return raw
    if isinstance(raw, float):
        if not math.isfinite(raw) or not raw.is_integer():
            return 0
        return int(raw)
    if isinstance(raw, str):
        text = raw.strip()
        if not text:
            return 0
        try:
            return int(text, 10)
        except ValueError:
            return 0
    return 0


def _env(name: str, default: str | None = None) -> str | None:
    v = os.environ.get(name, default)
    return v.strip() if isinstance(v, str) and v.strip() else default


def _clamp_int(raw: str | None, default: int, lo: int, hi: int) -> int:
    try:
        value = int(str(raw).strip())
    except (TypeError, ValueError):
        value = default
    return max(lo, min(hi, value))


def _scan_max_points() -> int:
    return _clamp_int(_env("AMR_SCAN_MAX_POINTS", str(SCAN_MAX_POINTS_DEFAULT)), SCAN_MAX_POINTS_DEFAULT, 20, 360)


def _scan_hz() -> int:
    return _clamp_int(_env("AMR_SCAN_HZ", str(SCAN_HZ_DEFAULT)), SCAN_HZ_DEFAULT, 1, 10)


def _telemetry_max_bytes() -> int:
    return _clamp_int(
        _env("AMR_TELEMETRY_MAX_BYTES", str(TELEMETRY_MAX_BYTES_DEFAULT)),
        TELEMETRY_MAX_BYTES_DEFAULT,
        20_000,
        250_000,
    )


def _json_size(payload: dict) -> int:
    return len(json.dumps(payload, separators=(",", ":"), default=str))


def _downsample_scan(scan: Any, max_points: int) -> list:
    """Every k-th [angle, range], order kept. Angle 3 dp, range 2 dp."""
    if not isinstance(scan, list) or not scan:
        return []
    step = 1 if len(scan) <= max_points else math.ceil(len(scan) / max_points)
    out: list = []
    for point in scan[::step]:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            continue
        try:
            angle = round(float(point[0]), 3)
            dist = round(float(point[1]), 2)
        except (TypeError, ValueError):
            continue
        out.append([angle, dist])
    return out


def _fmt_pose(pose: Any) -> str:
    try:
        return f"{float(pose[0]):.3f},{float(pose[1]):.3f},{float(pose[2]):.3f}"
    except (TypeError, ValueError, IndexError):
        return "0,0,0"


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


def _load_client_options():
    try:
        from supabase import ClientOptions
        return ClientOptions
    except ImportError:
        try:
            from supabase.lib.client_options import ClientOptions  # type: ignore
            return ClientOptions
        except ImportError:
            return None


def _create_supabase(url: str, key: str):
    """Publishable key only. Session is not written to disk.

    ``postgrest_client_timeout`` bounds ``rpc()`` and table reads. The sync
    client's ``.channel()`` is not used: it raises ``NotImplementedError``.
    """
    from supabase import create_client

    ClientOptions = _load_client_options()
    options = None
    if ClientOptions is not None:
        try:
            options = ClientOptions(
                auto_refresh_token=False,
                persist_session=False,
                postgrest_client_timeout=RPC_TIMEOUT_S,
            )
        except TypeError:
            try:
                options = ClientOptions(auto_refresh_token=False, persist_session=False)
            except Exception:
                options = None
        except Exception:
            options = None
    if options is not None:
        try:
            client = create_client(url, key, options=options)
        except TypeError:
            client = create_client(url, key)
    else:
        client = create_client(url, key)
    # Build the REST client now so rpc()/table reads pick up RPC_TIMEOUT_S.
    # postgrest-py warns that the timeout argument is deprecated; the httpx
    # session still uses it, and that is what bounds touch_amr_bot.
    import warnings

    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="The 'timeout' parameter is deprecated",
            category=DeprecationWarning,
        )
        warnings.filterwarnings(
            "ignore",
            message="The 'verify' parameter is deprecated",
            category=DeprecationWarning,
        )
        client.postgrest
    return client


def _realtime_http_url(supabase_url: str) -> str:
    """Base URL AsyncRealtimeClient turns into ``.../realtime/v1/websocket``."""
    return supabase_url.rstrip("/") + "/realtime/v1"


def _default_realtime_factory(url: str, api_key: str, *, reconnect_min_s: float):
    """Async Realtime socket. ``api_key`` is the publishable key, not a user JWT.

    ``auto_reconnect`` stays off and ``max_retries`` is 1 so a dead socket
    fails once. The bridge supervisor owns backoff and resubscribe.
    """
    from realtime import AsyncRealtimeClient

    return AsyncRealtimeClient(
        url,
        token=api_key,
        auto_reconnect=False,
        max_retries=1,
        initial_backoff=reconnect_min_s,
    )


def _status_name(status: Any) -> str:
    raw = getattr(status, "value", status)
    return str(raw or "").upper()


def _channel_state(channel: Any) -> str:
    state = getattr(channel, "state", None)
    return str(getattr(state, "value", state) or "").lower()


def assert_realtime_api() -> None:
    """Fail loudly if this install cannot open an async Realtime channel.

    No network. ``channel()`` on the async client only builds the object.
    ``subscribe()`` is intentionally not called: it opens the socket.
    Sync ``channel()`` raises ``NotImplementedError`` in every realtime-py
    release and is not a supported path.
    """
    from realtime import AsyncRealtimeChannel, AsyncRealtimeClient

    client = AsyncRealtimeClient(
        "https://example.supabase.co/realtime/v1",
        token="sb_publishable_smoke",
        auto_reconnect=False,
        max_retries=1,
    )
    try:
        channel = client.channel("amr-realtime-smoke")
        private = client.channel("bot:smoke:cmd", _PRIVATE_CHANNEL_PARAMS)
    except NotImplementedError as exc:
        raise RuntimeError(
            "AsyncRealtimeClient.channel() raised NotImplementedError. "
            "The bridge cannot subscribe to Realtime."
        ) from exc
    if client.is_connected:
        raise RuntimeError("AsyncRealtimeClient.channel() opened a socket")
    for label, obj in (("public", channel), ("private", private)):
        if not isinstance(obj, AsyncRealtimeChannel):
            raise RuntimeError(
                f"{label} channel() returned {type(obj).__name__}, "
                "expected AsyncRealtimeChannel"
            )
    for name in ("subscribe", "on_broadcast", "send_broadcast", "unsubscribe"):
        method = getattr(channel, name, None)
        if not callable(method):
            raise RuntimeError(f"AsyncRealtimeChannel is missing {name}()")
    if not inspect.iscoroutinefunction(channel.subscribe):
        raise RuntimeError("AsyncRealtimeChannel.subscribe is not async")
    if not inspect.iscoroutinefunction(channel.send_broadcast):
        raise RuntimeError("AsyncRealtimeChannel.send_broadcast is not async")
    if not inspect.iscoroutinefunction(client.set_auth):
        raise RuntimeError(
            "AsyncRealtimeClient.set_auth is not async "
            "(the sync client's set_auth is a no-op)"
        )


class _RtSlot:
    def __init__(self, kind: str, gen: int, channel: Any, topic: str):
        self.kind = kind
        self.gen = gen
        self.channel = channel
        self.topic = topic
        self.armed = False


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
    """REST stays on the sync client. Realtime runs on one asyncio thread.

    Call the sync methods from the bridge loop via ``to_thread``. They hand
    channel work to that thread with ``run_coroutine_threadsafe``. Sync
    ``Client.channel()`` raises ``NotImplementedError``; sync ``set_auth``
    returns without sending the JWT. A failed socket never kills this process:
    ``touch_amr_bot`` keeps running on the sync client.
    """

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
        realtime_factory=None,
        reconnect_min_s: float = RECONNECT_MIN_S,
        reconnect_max_s: float = RECONNECT_MAX_S,
        public_rest: bool = False,
    ):
        self._supabase_url = url
        self._api_key = key
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
        self._realtime_factory = realtime_factory or _default_realtime_factory
        self._reconnect_min_s = reconnect_min_s
        self._reconnect_max_s = reconnect_max_s
        self._client = None
        self._rt = None
        self._channels: list[Any] = []
        self._slots: dict[str, _RtSlot] = {}
        self._gens: dict[str, int] = {}
        self._fail_latch: set[str] = set()
        self._public_channel = None
        self._status_channel = None
        self._public_ready = False
        self._private_ready = False
        self._retry_tasks: dict[str, asyncio.Task] = {}
        self._retry_delay: dict[str, float] = {}
        self._signed_in = False
        self._access_token: str | None = None
        self._refresh_token: str | None = None
        self._expires_at = 0.0
        self._need_reauth = False
        self._stable = False
        self._socket_up = False
        self._closed = False
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._stop: asyncio.Event | None = None
        self._down: asyncio.Event | None = None
        self._setup_lock: asyncio.Lock | None = None
        self._private_task: asyncio.Task | None = None
        self._public_rest = public_rest
        self._stats_lock = threading.Lock()
        self._sent_ok: dict[str, int] = {}
        self._sent_fail: dict[str, int] = {}
        self._dropped_not_ready: dict[str, int] = {}
        self._send_fail_log_at: dict[str, float] = {}
        self._public_denied_logged = False
        self._public_key_logged = False
        self._rest_last = 0.0

    def connect(self) -> None:
        """Start the Realtime thread. Never raises.

        Subscribe runs on that thread. The 5 s touch loop does not wait for
        the socket and does not die if the socket fails.
        """
        if self._thread is not None:
            return
        try:
            self._thread = threading.Thread(
                target=self._thread_main,
                name="amr-realtime",
                daemon=True,
            )
            self._thread.start()
            if not self._ready.wait(timeout=5):
                raise RuntimeError("realtime loop did not start")
        except Exception as exc:
            log.error(
                "realtime setup failed: %s — touch loop continues without channels",
                self._redact(exc),
            )

    def send(self, event: str, payload: dict) -> None:
        # Status events go out on the public topic whenever it is SUBSCRIBED,
        # even if the private :status join is down. Only STATUS_EVENTS.
        try:
            if event not in STATUS_EVENTS:
                return
            public = self._public_ready and self._public_channel is not None
            private = self._private_ready and self._status_channel is not None
            if not public and not private:
                self._bump_stat(self._dropped_not_ready, event)
                return
            if public:
                self._send_one(self._public_channel, event, payload)
            if private:
                self._send_one(self._status_channel, event, payload)
            if public and self._public_rest:
                self._rest_broadcast_maybe(event, payload)
        except Exception as exc:
            log.warning("realtime send failed: %s", self._redact(exc))

    def stats(self) -> dict:
        with self._stats_lock:
            return {
                "public_ready": self._public_ready,
                "private_ready": self._private_ready,
                "sent_ok": dict(self._sent_ok),
                "sent_fail": dict(self._sent_fail),
                "dropped": dict(self._dropped_not_ready),
            }

    def _bump_stat(self, bucket: dict[str, int], event: str) -> None:
        with self._stats_lock:
            bucket[event] = bucket.get(event, 0) + 1

    def touch_bot_row(self, camera_url: str | None) -> bool | None:
        """Heartbeat public.touch_amr_bot. Logs a false result. Never logs the token."""
        if not self._bridge_token:
            log.warning("touch_amr_bot skipped: AMR_BRIDGE_TOKEN is unset")
            return None
        try:
            res = self._rest().rpc("touch_amr_bot", {
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
            self._rest().table("bot_control")
            .select("bot_id,controller,epoch,lease_expires_at")
            .eq("bot_id", self._bot_id)
            .limit(1)
            .execute()
        )
        token = (
            self._rest().table("bot_control_tokens")
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
        access token was applied. Channel subscribe stays on the Realtime
        thread; this only pushes ``set_auth``.
        """
        if not self._private_enabled or self._closed:
            return False
        if not self._signed_in:
            self._sign_in()
            self._push_auth()
            return True
        now = time.time()
        if self._expires_at and now < self._expires_at - AUTH_REFRESH_SKEW_S:
            return False
        if self._refresh_token:
            res = self._rest().auth.refresh_session(self._refresh_token)
        else:
            res = self._rest().auth.refresh_session()
        self._store_session(res)
        applied = self._push_auth()
        if applied:
            log.info("bridge auth refreshed; realtime auth re-applied")
        else:
            log.info("bridge auth refreshed; realtime auth will apply on subscribe")
        return True

    def close(self) -> None:
        self._closed = True
        loop = self._loop
        if loop is not None and loop.is_running():
            try:
                loop.call_soon_threadsafe(self._signal_stop)
            except RuntimeError:
                pass
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=5)

    def _rest(self):
        if self._client is None:
            self._client = _create_supabase(self._supabase_url, self._api_key)
        return self._client

    def _thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._boot())
        except Exception:
            log.exception("realtime loop exited; touch loop continues")
        finally:
            try:
                loop.close()
            except Exception:
                pass
            self._loop = None

    async def _boot(self) -> None:
        self._stop = asyncio.Event()
        self._down = asyncio.Event()
        self._setup_lock = asyncio.Lock()
        self._ready.set()
        supervise = asyncio.create_task(self._supervise())
        try:
            await self._stop.wait()
        finally:
            self._closed = True
            supervise.cancel()
            try:
                await supervise
            except (asyncio.CancelledError, Exception):
                pass
            try:
                await self._teardown_channels()
            except Exception:
                log.debug("realtime teardown failed", exc_info=True)

    async def _supervise(self) -> None:
        backoff = self._reconnect_min_s
        while not self._closed:
            self._signal_down_clear()
            try:
                await self._session_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.error(
                    "realtime setup failed: %s — touch continues, retrying channels",
                    self._redact(exc),
                )
            if self._closed:
                break
            self._public_ready = False
            self._private_ready = False
            try:
                await self._teardown_channels()
            except Exception as exc:
                log.debug("realtime teardown failed: %s", type(exc).__name__)
            log.warning("realtime reconnect in %.1fs", backoff)
            if await self._sleep_or_stop(backoff):
                break
            if self._stable:
                backoff = self._reconnect_min_s
                self._stable = False
            else:
                backoff = min(self._reconnect_max_s, backoff * 2)

    def _signal_down_clear(self) -> None:
        down = self._down
        if down is not None and down.is_set():
            down.clear()

    async def _session_once(self) -> None:
        self._fail_latch.clear()
        self._retry_delay.clear()
        self._bump("socket")
        self._socket_up = False
        if not self._public_enabled and not self._private_enabled:
            await self._stop.wait()
            return
        await self._ensure_client()
        # Public topic carries telemetry even when AMR_PUBLIC_CHANNEL is off.
        # The command listener is bound only when public commands are enabled.
        if self._public_enabled or self._private_enabled:
            await self._subscribe_public()
        if self._private_enabled:
            self._private_task = asyncio.create_task(self._private_until_ready())
        try:
            await self._wait_until_down()
        finally:
            task = self._private_task
            self._private_task = None
            if task is not None:
                task.cancel()
                try:
                    await task
                except (asyncio.CancelledError, Exception):
                    pass

    async def _private_until_ready(self) -> None:
        """Open :cmd, :cmdq, :ctl, and :status. Retry without dropping a live public channel."""
        delay = self._reconnect_min_s
        while not self._closed and not self._down_is_set():
            if "status" in self._slots:
                return
            try:
                await self._ensure_signed_in()
                await self._subscribe_private()
                return
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                log.error("private channel setup failed: %s", self._redact(exc))
                if not self._access_token:
                    self._log_public_publishable_key()
            if await self._sleep_or_stop(delay):
                return
            delay = min(self._reconnect_max_s, delay * 2)

    async def _wait_until_down(self) -> None:
        while not self._closed:
            if self._down_is_set():
                return
            self._poll_health()
            if self._down_is_set():
                return
            down = self._down
            if down is None:
                return
            try:
                await asyncio.wait_for(down.wait(), timeout=0.5)
                return
            except asyncio.TimeoutError:
                continue

    def _down_is_set(self) -> bool:
        down = self._down
        return down is not None and down.is_set()

    async def _sleep_or_stop(self, delay: float) -> bool:
        """True when the hub is closing."""
        if self._closed:
            return True
        stop = self._stop
        if stop is None:
            await asyncio.sleep(delay)
            return self._closed
        try:
            await asyncio.wait_for(stop.wait(), timeout=delay)
            return True
        except asyncio.TimeoutError:
            return self._closed

    async def _ensure_client(self) -> None:
        if self._rt is not None:
            return
        self._rt = self._realtime_factory(
            _realtime_http_url(self._supabase_url),
            self._api_key,
            reconnect_min_s=self._reconnect_min_s,
        )

    async def _ensure_signed_in(self) -> None:
        if not (self._signed_in and self._access_token):
            await asyncio.to_thread(self._sign_in)
        await self._apply_realtime_auth(self._access_token or "")

    async def _subscribe_public(self) -> None:
        topic = channel_name(self._bot_id)
        log.info("realtime joining (%s)", topic)
        if not self._private_enabled:
            self._log_public_publishable_key()
        event = CMD_EVENT if self._public_enabled else None
        callback = self._public_cb if self._public_enabled else None
        await self._subscribe_kind("public", topic, event, callback, private=False)

    def _log_public_publishable_key(self) -> None:
        """Public join has no bridge JWT. The socket keeps the publishable key."""
        if self._public_key_logged:
            return
        self._public_key_logged = True
        log.info("public channel using publishable key")

    async def _subscribe_private(self) -> None:
        if self._closed or "status" in self._slots:
            return
        if not self._access_token:
            raise RuntimeError("private subscribe without a bridge access token")
        await self._apply_realtime_auth(self._access_token)
        specs = (
            ("cmd", CMD_EVENT, self._cmd_cb),
            ("cmdq", CMD_EVENT, self._cmdq_cb),
            ("ctl", CTL_EVENT, self._ctl_cb),
            ("status", None, None),
        )
        log.info(
            "realtime joining private (%s)",
            ", ".join(private_topic(self._bot_id, kind) for kind, _, _ in specs),
        )
        made: list[str] = []
        try:
            for kind, event, callback in specs:
                topic = private_topic(self._bot_id, kind)
                await self._subscribe_kind(kind, topic, event, callback, private=True)
                made.append(kind)
        except Exception:
            for kind in made:
                await self._drop_slot(kind)
            self._status_channel = None
            raise

    async def _subscribe_kind(
        self,
        kind: str,
        topic: str,
        event: str | None,
        callback,
        *,
        private: bool,
    ) -> None:
        client = self._rt
        if client is None:
            raise RuntimeError("realtime client is not running")
        gen = self._bump(kind)
        if private:
            channel = self._open_private_channel(topic)
        else:
            channel = self._open_public_channel(topic)
        if event and callback:
            self._bind_broadcast(channel, event, callback)
        slot = _RtSlot(kind, gen, channel, topic)
        self._slots[kind] = slot
        self._channels.append(channel)
        if kind == "public":
            self._public_channel = channel
        if kind == "status":
            self._status_channel = channel
        # Ready flips in the SUBSCRIBED callback, not when subscribe() returns.
        await channel.subscribe(self._status_cb_for(kind, gen))
        slot.armed = True
        self._socket_up = True

    def _open_channel(self, topic: str, params: dict):
        client = self._rt
        if client is None:
            raise RuntimeError("realtime client is not running")
        try:
            return client.channel(topic, params)
        except TypeError:
            return client.channel(topic, params=params)

    def _open_public_channel(self, topic: str):
        return self._open_channel(topic, _PUBLIC_CHANNEL_PARAMS)

    def _open_private_channel(self, topic: str):
        return self._open_channel(topic, _PRIVATE_CHANNEL_PARAMS)

    def _status_cb_for(self, kind: str, gen: int):
        def _cb(*args) -> None:
            status = args[0] if args else ""
            error = args[1] if len(args) > 1 else None
            try:
                self._handle_status(kind, gen, status, error)
            except Exception:
                log.exception("realtime status callback failed")

        return _cb

    def _handle_status(self, kind: str, gen: int, status: Any, error: Any = None) -> None:
        if self._gens.get(kind) != gen:
            return
        name = _status_name(status)
        if name == "SUBSCRIBED":
            self._fail_latch.discard(kind)
            self._retry_delay[kind] = self._reconnect_min_s
            self._stable = True
            slot = self._slots.get(kind)
            if kind == "public":
                self._public_ready = True
            if kind == "status":
                self._private_ready = True
            topic = slot.topic if slot is not None else kind
            log.info("realtime subscribed (%s)", topic)
            self._reapply_if_needed()
            if self._on_link and kind == "status":
                self._on_link(True)
            return
        if name not in _DOWN_STATES and name not in ("CLOSED", "ERRORED"):
            return
        if kind in self._fail_latch:
            return
        self._fail_latch.add(kind)
        self._need_reauth = True
        label = "CHANNEL_ERROR" if name == "ERRORED" else name
        detail = self._redact_detail(error)
        if kind == "socket":
            self._public_ready = False
            self._private_ready = False
            if detail:
                log.warning("realtime socket %s: %s", label, detail)
            else:
                log.warning("realtime socket %s", label)
            down = self._down
            if down is not None:
                down.set()
            return
        # One channel's join error must not drop the others.
        self._clear_ready(kind)
        if detail:
            log.warning("realtime %s channel %s: %s", kind, label, detail)
        else:
            log.warning("realtime %s channel %s", kind, label)
        if kind == "public" and label == "CHANNEL_ERROR" and not self._public_denied_logged:
            self._public_denied_logged = True
            log.error(
                "public join denied: check Supabase Realtime Settings > Allow public access"
            )
        self._ensure_channel_retry(kind)

    def _clear_ready(self, kind: str) -> None:
        if kind == "public":
            self._public_ready = False
            self._public_channel = None
        elif kind == "status":
            self._private_ready = False
            self._status_channel = None

    def _kind_is_ready(self, kind: str) -> bool:
        if kind == "public":
            return self._public_ready
        if kind == "status":
            return self._private_ready
        slot = self._slots.get(kind)
        if slot is None or not slot.armed or slot.gen != self._gens.get(kind):
            return False
        return _channel_state(slot.channel) == "joined"

    def _join_pending(self, kind: str) -> bool:
        slot = self._slots.get(kind)
        if slot is None or not slot.armed or slot.gen != self._gens.get(kind):
            return False
        return _channel_state(slot.channel) == "joining"

    def _private_listener(self, kind: str):
        if kind == "cmd":
            return CMD_EVENT, self._cmd_cb
        if kind == "cmdq":
            return CMD_EVENT, self._cmdq_cb
        if kind == "ctl":
            return CTL_EVENT, self._ctl_cb
        return None, None

    def _ensure_channel_retry(self, kind: str) -> None:
        if self._closed or kind == "socket" or self._down_is_set():
            return
        existing = self._retry_tasks.get(kind)
        if existing is not None and not existing.done():
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return
        self._retry_tasks[kind] = loop.create_task(self._retry_channel(kind))

    async def _retry_channel(self, kind: str) -> None:
        """Resubscribe one topic. Leave every other channel on the socket."""
        try:
            while not self._closed and not self._down_is_set():
                delay = self._retry_delay.get(kind, self._reconnect_min_s)
                if await self._sleep_or_stop(delay):
                    return
                if self._closed or self._down_is_set() or self._kind_is_ready(kind):
                    self._retry_delay[kind] = self._reconnect_min_s
                    return
                # Drop the latch so this attempt's error is logged once.
                self._fail_latch.discard(kind)
                try:
                    if kind != "public":
                        await self._ensure_signed_in()
                    lock = self._setup_lock
                    if lock is None:
                        await self._resubscribe_locked(kind)
                    else:
                        async with lock:
                            await self._resubscribe_locked(kind)
                except asyncio.CancelledError:
                    raise
                except Exception as exc:
                    log.error(
                        "realtime %s channel retry failed: %s",
                        kind,
                        self._redact(exc),
                    )
                if self._closed or self._down_is_set() or self._kind_is_ready(kind):
                    self._retry_delay[kind] = self._reconnect_min_s
                    return
                # subscribe() returned while the join ack is still outstanding.
                if self._join_pending(kind):
                    return
                current = self._retry_delay.get(kind, delay)
                self._retry_delay[kind] = min(
                    self._reconnect_max_s,
                    max(current, self._reconnect_min_s) * 2,
                )
        finally:
            task = self._retry_tasks.get(kind)
            if task is not None and task is asyncio.current_task():
                self._retry_tasks.pop(kind, None)

    async def _resubscribe_locked(self, kind: str) -> None:
        await self._drop_slot(kind)
        if kind == "public":
            await self._subscribe_public()
            return
        event, callback = self._private_listener(kind)
        topic = private_topic(self._bot_id, kind)
        log.info("realtime rejoining (%s)", topic)
        await self._subscribe_kind(kind, topic, event, callback, private=True)

    def _poll_health(self) -> None:
        client = self._rt
        if client is not None and self._socket_up:
            dead = False
            task = getattr(client, "_listen_task", None)
            ws = getattr(client, "_ws_connection", None)
            if task is not None and task.done():
                dead = True
            elif ws is not None and getattr(ws, "close_code", None) is not None:
                dead = True
            if dead:
                self._handle_status("socket", self._gens.get("socket", 0), "CLOSED")
                return
        for slot in list(self._slots.values()):
            if not slot.armed:
                continue
            value = _channel_state(slot.channel)
            if value == "closed":
                self._handle_status(slot.kind, slot.gen, "CLOSED")
            elif value == "errored":
                self._handle_status(slot.kind, slot.gen, "CHANNEL_ERROR")

    def _bump(self, kind: str) -> int:
        gen = self._gens.get(kind, 0) + 1
        self._gens[kind] = gen
        return gen

    def _bind_broadcast(self, channel, event: str, callback) -> None:
        wrapped = self._guard_cb(callback)
        try:
            channel.on_broadcast(event=event, callback=wrapped)
        except Exception:
            channel.on("broadcast", {"event": event}, wrapped)

    def _guard_cb(self, callback):
        def _wrapped(message) -> None:
            try:
                callback(message)
            except Exception:
                log.exception("realtime broadcast callback failed")

        return _wrapped

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

    def _reapply_if_needed(self) -> None:
        """On the Realtime thread. Re-push the user JWT after SUBSCRIBED."""
        if not self._need_reauth or not self._access_token:
            return
        token = self._access_token
        self._need_reauth = False

        async def _go() -> None:
            try:
                await self._apply_realtime_auth(token)
                log.info("realtime auth re-applied after reconnect")
            except Exception as exc:
                self._need_reauth = True
                log.warning("realtime auth re-apply failed: %s", type(exc).__name__)

        try:
            asyncio.get_running_loop()
        except RuntimeError:
            return
        asyncio.create_task(_go())

    def _sign_in(self) -> None:
        if not self._email or not self._password:
            raise RuntimeError("AMR_BRIDGE_EMAIL and AMR_BRIDGE_PASSWORD are required")
        res = self._rest().auth.sign_in_with_password({
            "email": self._email,
            "password": self._password,
        })
        self._store_session(res)
        log.info("bridge auth signed in")

    def _store_session(self, res: Any) -> None:
        access, refresh, expires = _session_parts(res)
        self._access_token = access
        if refresh:
            self._refresh_token = refresh
        self._expires_at = expires if expires else time.time() + 3600
        self._signed_in = True

    def _push_auth(self) -> bool:
        """Hop ``set_auth`` onto the Realtime thread. False if that thread is down."""
        token = self._access_token or ""
        if not token:
            raise RuntimeError("missing bridge access token")
        self._need_reauth = True
        loop = self._loop
        if loop is None or not loop.is_running():
            return False
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            return False
        self._call(self._apply_realtime_auth(token), timeout=RPC_TIMEOUT_S)
        return True

    async def _apply_realtime_auth(self, access_token: str) -> None:
        if not access_token:
            raise RuntimeError("missing bridge access token")
        client = self._rt
        if client is None:
            raise RuntimeError("realtime client is not running")
        await client.set_auth(access_token)
        self._need_reauth = False

    def _send_one(self, channel, event: str, payload: dict) -> None:
        """Schedule an async broadcast. Same signature ``send_status`` can call."""
        if channel is None:
            return
        try:
            self._call(
                self._send_async(channel, event, payload),
                timeout=REALTIME_SEND_TIMEOUT_S,
            )
        except Exception as exc:
            self._bump_stat(self._sent_fail, event)
            self._log_send_failure(event, exc)
            return
        self._bump_stat(self._sent_ok, event)

    def _log_send_failure(self, event: str, exc: BaseException) -> None:
        kind = type(exc).__name__
        now = time.monotonic()
        with self._stats_lock:
            last = self._send_fail_log_at.get(kind, 0.0)
            if now - last < SEND_FAIL_LOG_S:
                return
            self._send_fail_log_at[kind] = now
        log.warning("broadcast %s failed: %s", event, self._redact(exc))

    def _rest_broadcast_maybe(self, event: str, payload: dict) -> None:
        """Optional public REST broadcast. Off unless AMR_PUBLIC_REST=1. At most 5 Hz."""
        now = time.monotonic()
        if now - self._rest_last < REST_BROADCAST_MIN_S:
            return
        self._rest_last = now
        try:
            url = self._supabase_url.rstrip("/") + "/realtime/v1/api/broadcast"
            body = {
                "messages": [{
                    "topic": channel_name(self._bot_id),
                    "event": event,
                    "payload": payload,
                    "private": False,
                }],
            }
            self._post_rest(
                url,
                {"apikey": self._api_key, "Content-Type": "application/json"},
                body,
            )
        except Exception as exc:
            self._log_send_failure(event, exc)

    def _post_rest(self, url: str, headers: dict, body: dict) -> None:
        httpx.post(url, headers=headers, json=body, timeout=REST_BROADCAST_TIMEOUT_S)

    async def _send_async(self, channel, event: str, payload: dict) -> None:
        await channel.send_broadcast(event, payload)

    def _call(self, coro, timeout: float):
        loop = self._loop
        if self._closed or loop is None or not loop.is_running():
            coro.close()
            raise RuntimeError("realtime loop is not running")
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is loop:
            coro.close()
            raise RuntimeError("cannot wait on the realtime loop from itself")
        fut = asyncio.run_coroutine_threadsafe(coro, loop)
        return fut.result(timeout)

    def _signal_stop(self) -> None:
        if self._stop is not None:
            self._stop.set()
        if self._down is not None:
            self._down.set()

    async def _teardown_channels(self) -> None:
        # Cancel retries before taking the lock. A retry waiting on the lock
        # cannot finish if teardown holds it and then awaits that retry.
        await self._cancel_retries()
        lock = self._setup_lock
        if lock is None:
            await self._drop_all_channels()
            return
        async with lock:
            await self._drop_all_channels()

    async def _cancel_retries(self) -> None:
        tasks = [task for task in self._retry_tasks.values() if task is not None]
        self._retry_tasks.clear()
        for task in tasks:
            task.cancel()
        current = asyncio.current_task()
        for task in tasks:
            if task is current:
                continue
            try:
                await task
            except (asyncio.CancelledError, Exception):
                pass

    async def _drop_all_channels(self) -> None:
        self._socket_up = False
        for kind in list(self._slots):
            self._bump(kind)
        self._bump("socket")
        channels = list(self._channels)
        self._channels.clear()
        self._slots.clear()
        self._public_channel = None
        self._status_channel = None
        self._public_ready = False
        self._private_ready = False
        client = self._rt
        self._rt = None
        for channel in channels:
            await self._remove_channel(client, channel)
        if client is not None:
            close = getattr(client, "close", None)
            if close is not None:
                try:
                    await close()
                except Exception:
                    pass

    async def _drop_slot(self, kind: str) -> None:
        # Bump before unsubscribe so this channel's own CLOSED is ignored.
        slot = self._slots.pop(kind, None)
        self._bump(kind)
        self._clear_ready(kind)
        if slot is None:
            return
        slot.armed = False
        try:
            self._channels.remove(slot.channel)
        except ValueError:
            pass
        await self._remove_channel(self._rt, slot.channel)

    async def _remove_channel(self, client, channel) -> None:
        if channel is None:
            return
        try:
            remove = getattr(client, "remove_channel", None) if client is not None else None
            if remove is not None:
                await remove(channel)
                return
            unsub = getattr(channel, "unsubscribe", None)
            if unsub is not None:
                await unsub()
        except Exception:
            pass

    def _redact(self, exc: BaseException) -> str:
        return self._redact_text(f"{type(exc).__name__}: {exc}")

    def _redact_detail(self, error: Any) -> str:
        if error is None:
            return ""
        if isinstance(error, BaseException):
            text = str(error) or type(error).__name__
        else:
            text = str(error)
        return self._redact_text(text)

    def _redact_text(self, text: str) -> str:
        for secret in (
            self._password,
            self._access_token,
            self._refresh_token,
            self._api_key,
            self._bridge_token,
        ):
            if secret and secret in text:
                text = text.replace(secret, "[redacted]")
        text = _JWT_RE.sub("[redacted]", text)
        text = _SECRET_FIELD_RE.sub(r"\1[redacted]", text)
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
        source: str = "dashboard",
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
        self.source = resolve_source(source)
        if private_channel and not self.private_channel:
            log.warning(
                "private channels skipped: set AMR_BRIDGE_EMAIL and AMR_BRIDGE_PASSWORD "
                "to sign in as the bridge user"
            )
        if not self.public_channel and not self.private_channel:
            log.warning("no Realtime channel is enabled; telemetry will not be published")
        self.dash = DashboardClient(dashboard_url) if self.source == "dashboard" else None
        self._ros: RosSource | None = None
        self.camera_url = camera_url
        self.camera_jpeg_max_kb = camera_jpeg_max_kb
        self.scan_max_points = _scan_max_points()
        self.scan_hz = _scan_hz()
        self.telemetry_max_bytes = _telemetry_max_bytes()
        self.public_rest = flag_enabled(_env("AMR_PUBLIC_REST", "0"), False)
        self.mode = "full"
        self._seq = 0
        self._telemetry_frames = 0
        self._sse_state_frames = 0
        self._last_scan_pts = 0
        self._last_pose: list = [0, 0, 0]
        self._oversize_log_at: dict[str, float] = {}
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
        # Latest joy / nudge / drive only. A newer stick replaces one not yet posted.
        # _motion_gen tracks that replacement. _estop_gen changes only when an
        # e-stop drops the slot, so a newer stick does not look like a stop.
        self._motion_lock = threading.Lock()
        self._motion_pending: tuple[dict, str] | None = None
        self._motion_gen = 0
        self._estop_gen = 0
        self._motion_wake = asyncio.Event()
        self._estop_seq: int | None = None
        self._motion_hold_until = 0.0
        # E-stop is its own task. The queue is not the capped scheduler.
        self._estop_lock = threading.Lock()
        self._estop_q: deque[tuple[dict, str]] = deque(maxlen=ESTOP_QUEUE_MAX)
        self._estop_needed = False
        self._estop_wake = asyncio.Event()
        # :ctl is coalesced and is not subject to the callback cap.
        self._ctl_lock = threading.Lock()
        self._ctl_latest: dict | None = None
        self._ctl_running = False

    def _queue_public(self, env: dict) -> None:
        typ = peek_command_type(env)
        if typ == "estop":
            self._offer_estop(env, "public")
            return
        if typ in _MOTION_SLOT_TYPES:
            self._offer_motion(env, "public")
            return
        self._schedule(self._handle_public_command(env))

    def _queue_private(self, env: dict, source: str) -> None:
        typ = peek_command_type(env)
        if typ == "estop":
            self._offer_estop(env, source)
            return
        if typ in _MOTION_SLOT_TYPES:
            self._offer_motion(env, source)
            return
        self._schedule(self._handle_private_command(env, source))

    def _queue_control(self, env: dict) -> None:
        with self._ctl_lock:
            self._ctl_latest = env
            start = not self._ctl_running
            if start:
                self._ctl_running = True
        if start:
            self._spawn(self._ctl_drain())

    def _offer_motion(self, env: dict, source: str) -> None:
        with self._motion_lock:
            self._motion_gen += 1
            self._motion_pending = (env, source)
        self._signal(self._motion_wake)

    def _take_motion(self) -> tuple[tuple[dict, str] | None, int, int]:
        with self._motion_lock:
            item = self._motion_pending
            gen = self._motion_gen
            estop_gen = self._estop_gen
            self._motion_pending = None
            return item, gen, estop_gen

    def _clear_motion(self) -> None:
        """Drop a stick that has not been posted. Does not bump the stick generation.

        An in-flight post watches ``_estop_gen`` and zeroes only when this
        runs. A newer stick must not take that path, or the robot stutters
        whenever a post is slower than the stick rate.
        """
        with self._motion_lock:
            self._estop_gen += 1
            self._motion_pending = None

    def _motion_current(self, gen: int) -> bool:
        with self._motion_lock:
            return gen == self._motion_gen

    def _estop_current(self, gen: int) -> bool:
        with self._motion_lock:
            return gen == self._estop_gen

    def _arm_estop_hold(self, env: dict) -> None:
        """Ignore stick commands briefly, unless they are newer than this stop."""
        self._estop_seq = command_seq(env)
        self._motion_hold_until = time.monotonic() + ESTOP_MOTION_HOLD_S

    def _motion_held(self, env: dict) -> bool:
        if time.monotonic() >= self._motion_hold_until:
            return False
        seq = command_seq(env)
        held = self._estop_seq
        if held is not None and seq is not None and seq > held:
            return False
        return True

    def _offer_estop(self, env: dict, source: str) -> None:
        with self._estop_lock:
            self._estop_q.append((env, source))
        self._signal(self._estop_wake)

    def _drain_estop(self) -> list[tuple[dict, str]]:
        with self._estop_lock:
            items = list(self._estop_q)
            self._estop_q.clear()
            return items

    def _signal(self, event: asyncio.Event) -> None:
        loop = self._loop
        if loop is not None and loop.is_running():
            try:
                loop.call_soon_threadsafe(event.set)
                return
            except RuntimeError:
                pass
        event.set()

    def _spawn(self, coro) -> None:
        """Run a coroutine on the bridge loop with no in-flight cap."""
        if not (self._loop and self._loop.is_running()):
            coro.close()
            return
        asyncio.run_coroutine_threadsafe(coro, self._loop)

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
            public_rest=self.public_rest,
        )
        await asyncio.to_thread(self._hub.connect)
        if self.source == "ros":
            log.info("AMR_SOURCE=ros — reading ROS topics directly (robot_dashboard not required)")
            self._ros = RosSource(on_event=self._queue_ros_event)
            await asyncio.to_thread(self._ros.start)
            feed = self._ros_idle_loop()
        else:
            log.info("AMR_SOURCE=dashboard — SSE from %s", self.dash.base if self.dash else "?")
            feed = self._sse_loop()
        tasks = [
            feed,
            self._heartbeat_loop(),
            self._presence_touch_loop(),
            self._watchdog_loop(),
            self._motion_slot_loop(),
            self._estop_loop(),
            self._stats_loop(),
        ]
        if self.private_channel:
            tasks.append(self._control_poll_loop())
            tasks.append(self._auth_refresh_loop())
        await asyncio.gather(*tasks)

    def _queue_ros_event(self, event: dict) -> None:
        """Hop a ROS callback onto the bridge asyncio loop.

        Uses ``_spawn`` (uncapped), not ``_schedule`` — the command queue cap
        would drop 10 Hz telemetry frames.
        """
        self._spawn(self._handle_sse_event(event))

    async def _ros_idle_loop(self) -> None:
        """Keep the gather alive while the ROS thread pushes events."""
        while not self._stop.is_set():
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                pass

    async def _ctl_drain(self) -> None:
        while not self._stop.is_set():
            with self._ctl_lock:
                env = self._ctl_latest
                self._ctl_latest = None
                if env is None:
                    self._ctl_running = False
                    return
            try:
                await self._handle_control(env)
            except Exception as exc:
                # Keep the loop so a :ctl that arrived during the failure is
                # still polled. Dropping it would wait for the next cue.
                log.warning("control event failed: %s", exc)

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
        rank = _public_rank(env)
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

    def _accept_motion(self, env: dict, source: str) -> dict | None:
        if source == "public":
            if not isinstance(env, dict) or env.get("botId") not in (self.bot_id, None):
                return None
            cmd = env.get("cmd") if isinstance(env.get("cmd"), dict) else {}
            rank = _public_rank(env)
            if rank < 1:
                return None
            clean, reason = sanitize_command(cmd)
            if clean is None or clean.get("type") not in _MOTION_SLOT_TYPES:
                if reason:
                    log.info("rejected public motion: %s", reason)
                return None
            if self._motion_held(env):
                log.info("public motion ignored after e-stop")
                return None
            # Public commands do not arm the private deadman.
            return clean
        if not isinstance(env, dict):
            return None
        if self._motion_held(env):
            log.info("motion ignored after e-stop")
            return None
        result = self._gate.verify(
            env, source=source, now_s=time.time(), now_mono=time.monotonic(),
        )
        if not result.ok:
            log.info("rejected %s command type=%s: %s", source, result.cmd_type, result.reason)
            return None
        cmd = result.cmd or {}
        if cmd.get("type") not in _MOTION_SLOT_TYPES:
            return None
        log.info("accepted %s command type=%s", source, cmd.get("type"))
        if motion_command(cmd) and not self._gate.fault_active(time.time(), time.monotonic()):
            self._guard.set_hard(False)
            self._guard.release_for_motion()
        return cmd

    def _accept_estop(self, env: dict, source: str) -> bool:
        if source == "public":
            if not isinstance(env, dict) or env.get("botId") not in (self.bot_id, None):
                return False
            cmd = env.get("cmd") if isinstance(env.get("cmd"), dict) else {}
            if cmd.get("type") != "estop":
                return False
            rank = _public_rank(env)
            if rank < 1:
                log.info("rejected public e-stop: rank")
                return False
            log.warning("public e-stop accepted")
            self._arm_estop_hold(env)
            return True
        if not isinstance(env, dict):
            return False
        result = self._gate.verify(
            env, source=source, now_s=time.time(), now_mono=time.monotonic(),
        )
        if not result.ok or result.cmd_type != "estop":
            log.info("rejected %s e-stop: %s", source, result.reason)
            return False
        log.warning("e-stop accepted on %s", source)
        self._arm_estop_hold(env)
        return True

    async def _motion_slot_loop(self) -> None:
        """Post at most one joy or nudge. A newer one replaces a pending one."""
        while not self._stop.is_set():
            try:
                await self._motion_slot_turn()
            except Exception:
                log.exception("motion slot failed")

    async def _motion_slot_turn(self) -> None:
        item, gen, estop_gen = self._take_motion()
        if item is None:
            self._motion_wake.clear()
            item, gen, estop_gen = self._take_motion()
            if item is None:
                try:
                    await asyncio.wait_for(self._motion_wake.wait(), timeout=0.5)
                except asyncio.TimeoutError:
                    return
                return
        if self._stop.is_set() or not self._motion_current(gen) or not self._estop_current(estop_gen):
            return
        accepted_mono = time.monotonic()
        cmd = self._accept_motion(item[0], item[1])
        if cmd is None or not self._motion_current(gen) or not self._estop_current(estop_gen):
            return
        try:
            await self._post_motion_cmd(cmd)
        except Exception as exc:
            log.warning("motion command failed: %s", exc)
        late = time.monotonic() - accepted_mono >= MOTION_WATCHDOG_S
        estop_won = not self._estop_current(estop_gen)
        if late or estop_won:
            # A newer stick is not a reason to zero. A stop, or a post that
            # outlived the deadman, is. The guard may already have been cleared
            # by the watchdog zero that this delivery just undid.
            reason = "late command" if late else "e-stop"
            log.warning("watchdog: zero motion (%s)", reason)
            await self._post_zero()
            return
        if motion_command(cmd) and (self._guard.hard or self._guard.owed):
            log.warning("watchdog: zero motion (after command)")
            owed = self._guard.owed_gen
            if await self._post_zero() and owed is not None:
                self._guard.mark_zeroed(owed)

    async def _estop_loop(self) -> None:
        """Verify e-stops and post the stop until the dashboard accepts it.

        This task is not part of the capped callback queue and does not take
        the motion lock, so a stuck joy post cannot delay it.
        """
        while not self._stop.is_set():
            try:
                await self._estop_turn()
            except Exception:
                log.exception("e-stop loop failed")

    async def _estop_turn(self) -> None:
        batch = self._drain_estop()
        if not batch and not self._estop_needed:
            self._estop_wake.clear()
            batch = self._drain_estop()
            if not batch:
                try:
                    await asyncio.wait_for(self._estop_wake.wait(), timeout=0.5)
                except asyncio.TimeoutError:
                    return
                return
        accepted = False
        for env, source in batch:
            try:
                if self._accept_estop(env, source):
                    accepted = True
            except Exception:
                log.exception("e-stop accept failed")
        if accepted:
            self._estop_needed = True
            self._clear_motion()
        if not self._estop_needed:
            return
        if await self._estop_once():
            self._estop_needed = False
        else:
            self._estop_wake.clear()
            try:
                await asyncio.wait_for(self._estop_wake.wait(), timeout=ZERO_RETRY_S)
            except asyncio.TimeoutError:
                pass

    async def _estop_once(self) -> bool:
        """Zero motion immediately. ROS source publishes /cmd_vel; dashboard uses HTTP."""
        timeout = COMMAND_HTTP_TIMEOUT_S

        async def _stop_primary() -> None:
            if self._ros is not None:
                await asyncio.to_thread(self._ros.estop)
                return
            assert self.dash is not None
            await self.dash.post("/api/estop", timeout=timeout)

        async def _zero_nudge() -> None:
            if self._ros is not None:
                await asyncio.to_thread(self._ros.nudge, 0.0, 0.0)
                return
            assert self.dash is not None
            await self.dash.post(
                "/api/nudge", {"lin": 0.0, "ang": 0.0}, timeout=timeout,
            )

        async def _zero_joy() -> None:
            gamepad = os.environ.get("AMR_GAMEPAD_URL", "").rstrip("/")
            if not gamepad or self.dash is None:
                return
            response = await self.dash._http.post(
                f"{gamepad}/api/joy",
                json={"a": [0.0] * 8, "b": 0},
                timeout=timeout,
            )
            response.raise_for_status()

        results = await asyncio.gather(
            _stop_primary(), _zero_nudge(), _zero_joy(), return_exceptions=True,
        )
        ok = True
        for item in results:
            if isinstance(item, Exception):
                log.warning("e-stop post failed: %s", item)
                ok = False
        if ok:
            log.warning("e-stop posts accepted")
        return ok

    def _apply_rate(self, cmd: dict) -> None:
        mode = cmd.get("mode")
        if mode in ("full", "board"):
            self.mode = mode
            log.info("rate mode → %s", mode)

    async def _dispatch_cmd(self, cmd: dict) -> None:
        """Forward a command to ROS (/cmd_vel) or the dashboard / gamepad HTTP APIs.

        Motion always lands on /cmd_vel so safety_stop stays in front of the motors.
        Do not publish /cmd_vel_safe from this process.
        """
        typ = cmd.get("type")
        timeout = COMMAND_HTTP_TIMEOUT_S
        # Stick commands stay off this lock. E-stop is posted by its own loop.
        if typ in _MOTION_SLOT_TYPES:
            await self._post_motion_cmd(cmd)
            if motion_command(cmd) and (self._guard.hard or self._guard.owed):
                log.warning("watchdog: zero motion (after command)")
                owed = self._guard.owed_gen
                if await self._post_zero() and owed is not None:
                    self._guard.mark_zeroed(owed)
            return
        async with self._io_lock:
            if self._ros is not None:
                if typ == "nav_goal":
                    await asyncio.to_thread(
                        self._ros.nav_goal, float(cmd.get("x", 0)), float(cmd.get("y", 0)),
                    )
                elif typ == "nav_cancel":
                    await asyncio.to_thread(self._ros.cancel_nav)
                elif typ in ("vision", "save_map", "run_tool", "stop_tool", "blacklist"):
                    result = self._ros.unsupported(typ)
                    log.info("ros source: %s", result.get("error"))
                return
            assert self.dash is not None
            if typ == "nav_goal":
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

    async def _post_motion_cmd(self, cmd: dict) -> None:
        timeout = COMMAND_HTTP_TIMEOUT_S
        typ = cmd.get("type")
        if typ in ("nudge", "drive"):
            lin, ang = float(cmd.get("lin", 0)), float(cmd.get("ang", 0))
            if self._ros is not None:
                await asyncio.to_thread(self._ros.nudge, lin, ang)
                return
            assert self.dash is not None
            await self.dash.post("/api/nudge", {"lin": lin, "ang": ang}, timeout=timeout)
            return
        await self._post_joy(cmd, timeout)

    async def _post_joy(self, cmd: dict, timeout: float) -> None:
        """``a``/``b`` win over ``axes``/``buttons``. Does not take ``_io_lock``."""
        gamepad = os.environ.get("AMR_GAMEPAD_URL", "").rstrip("/")
        if "a" in cmd and cmd.get("a") is not None:
            axes = list(cmd["a"])
        else:
            axes = list(cmd.get("axes") or [])
        if "b" in cmd and cmd.get("b") is not None:
            buttons = int(cmd["b"])
        else:
            buttons = int(cmd["buttons"]) if cmd.get("buttons") is not None else 0
        if gamepad and self.dash is not None:
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
            if self._ros is not None:
                await asyncio.to_thread(self._ros.nudge, lin, ang)
                return
            assert self.dash is not None
            await self.dash.post("/api/nudge", {"lin": lin, "ang": ang}, timeout=timeout)

    async def _post_zero(self) -> bool:
        """False means the watchdog must try again. Does not take ``_io_lock``.

        A zero nudge streams Twist zeros on /cmd_vel. A zero joy packet makes
        joy_teleop publish one zero Twist and then yield. Neither path writes
        /cmd_vel_safe.
        """
        timeout = COMMAND_HTTP_TIMEOUT_S
        try:
            if self._ros is not None:
                await asyncio.to_thread(self._ros.nudge, 0.0, 0.0)
            else:
                assert self.dash is not None
                await self.dash.post("/api/nudge", {"lin": 0.0, "ang": 0.0}, timeout=timeout)
        except Exception as exc:
            log.warning("watchdog zero nudge failed: %s", exc)
            return False
        gamepad = os.environ.get("AMR_GAMEPAD_URL", "").rstrip("/")
        if not gamepad or self.dash is None:
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
        assert self.dash is not None
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
        try:
            await self._handle_sse_event_inner(event)
        except Exception:
            log.exception("sse event failed")

    async def _handle_sse_event_inner(self, event: dict) -> None:
        if event.get("t") == "map":
            payload = {
                "t": "map",
                "botId": self.bot_id,
                "ts": int(time.time() * 1000),
                "w": event.get("w"),
                "h": event.get("h"),
                "res": event.get("res"),
                "origin": event.get("origin"),
                "tf": event.get("tf"),
                "data": event.get("data"),
            }
            if self._map_fits(payload):
                await self._broadcast("map", payload)
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

        self._sse_state_frames += 1
        pose = event.get("pose") or [0, 0, 0]
        self._last_pose = list(pose) if isinstance(pose, list) else [0, 0, 0]
        if isinstance(event.get("scan"), list):
            self._last_scan_pts = len(_downsample_scan(event.get("scan"), self.scan_max_points))

        hz = BOARD_RATE_HZ if self.mode == "board" else FULL_RATE_HZ
        now = time.monotonic()
        if now - self._last_pub < (1.0 / hz) * 0.85:
            return
        self._last_pub = now

        self._seq += 1
        self._telemetry_frames += 1
        mode = self.mode
        every = max(1, math.ceil(FULL_RATE_HZ / self.scan_hz))
        include_scan = mode == "full" and (self._telemetry_frames % every == 1)
        payload: dict[str, Any] = {
            "ts": int(time.time() * 1000),
            "botId": self.bot_id,
            "seq": self._seq,
            "mode": mode,
            "online": True,
            "pose": pose,
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
            "scan": _downsample_scan(event.get("scan"), self.scan_max_points) if include_scan else (
                [] if mode != "full" else None
            ),
        }

        if mode == "full" and not self.camera_url and self.dash is not None:
            self._cam_every = (self._cam_every + 1) % 15
            if self._cam_every == 0:
                jpeg = await self.dash.fetch_camera_jpeg(self.camera_jpeg_max_kb)
                if jpeg:
                    payload["cameraJpegB64"] = jpeg

        payload = self._shrink_telemetry(payload)
        await self._broadcast("telemetry", payload)

    def _shrink_telemetry(self, payload: dict) -> dict:
        """Drop optional fields until the frame fits. Pose and vitals stay. Never raises."""
        limit = self.telemetry_max_bytes
        try:
            if _json_size(payload) <= limit:
                return payload
        except Exception:
            self._warn_oversize("telemetry", "telemetry oversize, dropped fields: unserializable")
            return payload
        dropped: list[str] = []
        if "cameraJpegB64" in payload:
            payload.pop("cameraJpegB64", None)
            dropped.append("cameraJpegB64")
        if self._payload_over(payload, limit):
            payload["scan"] = []
            dropped.append("scan")
        for field in ("frontier", "yoloDet", "sem", "path"):
            if not self._payload_over(payload, limit):
                break
            if field in payload:
                payload.pop(field, None)
                dropped.append(field)
        if dropped:
            self._warn_oversize(
                "telemetry",
                "telemetry oversize, dropped fields: " + ", ".join(dropped),
            )
        return payload

    def _map_fits(self, payload: dict) -> bool:
        try:
            if _json_size(payload) <= self.telemetry_max_bytes:
                return True
        except Exception:
            self._warn_oversize("map", "map oversize, dropped frame")
            return False
        self._warn_oversize("map", "map oversize, dropped frame")
        return False

    def _payload_over(self, payload: dict, limit: int) -> bool:
        try:
            return _json_size(payload) > limit
        except Exception:
            return True

    def _warn_oversize(self, kind: str, message: str) -> None:
        now = time.monotonic()
        if now - self._oversize_log_at.get(kind, 0.0) < OVERSIZE_LOG_S:
            return
        self._oversize_log_at[kind] = now
        log.warning("%s", message)

    async def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            try:
                hub = self._hub
                self._seq += 1
                await self._broadcast("hb", {
                    "t": "hb",
                    "botId": self.bot_id,
                    "ts": int(time.time() * 1000),
                    "bridgeVersion": BRIDGE_VERSION,
                    "public": bool(hub and hub._public_ready),
                    "private": bool(hub and hub._private_ready),
                    "seq": self._seq,
                })
            except Exception:
                log.exception("heartbeat failed")
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                pass

    async def _stats_loop(self) -> None:
        while not self._stop.is_set():
            try:
                self._log_stats()
            except Exception:
                log.exception("bridge stats failed")
            try:
                await asyncio.wait_for(self._stop.wait(), timeout=STATS_LOG_S)
            except asyncio.TimeoutError:
                pass

    def _log_stats(self) -> None:
        hub = self._hub
        snap = hub.stats() if hub is not None else {}
        sent = snap.get("sent_ok") or {}
        fail = snap.get("sent_fail") or {}
        dropped = snap.get("dropped") or {}
        log.info(
            "bridge stats: public_ready=%s private_ready=%s telemetry_ok=%s hb_ok=%s "
            "fail=%s dropped_not_ready=%s sse_state_frames=%s last_scan_pts=%s last_pose=%s",
            bool(snap.get("public_ready")),
            bool(snap.get("private_ready")),
            int(sent.get("telemetry", 0)),
            int(sent.get("hb", 0)),
            int(sum(fail.values()) if isinstance(fail, dict) else 0),
            int(sum(dropped.values()) if isinstance(dropped, dict) else 0),
            self._sse_state_frames,
            self._last_scan_pts,
            _fmt_pose(self._last_pose),
        )

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
        if self._ros is not None:
            await asyncio.to_thread(self._ros.stop)
            self._ros = None
        if self.dash is not None:
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
    p.add_argument(
        "--source",
        default=_env("AMR_SOURCE", "dashboard"),
        help="dashboard = SSE from robot_dashboard; ros = read ROS topics on this host",
    )
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
    source = resolve_source(args.source)
    if source == "ros" and not args.dashboard:
        args.dashboard = "http://127.0.0.1:8080"  # unused when source=ros

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
        source=source,
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
