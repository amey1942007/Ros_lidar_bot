"""Pure control-lease checks for the private bridge path.

No network, no ROS. bridge.py applies the results. Velocity is never decided
here; the caller zeros motion through the existing dashboard / gamepad HTTP
paths, which publish /cmd_vel so safety_stop stays in front of the motors.

Checks the private path actually applies:

- ``:cmd`` requires ``token`` (compared with ``hmac.compare_digest``).
  ``:cmdq`` trusts the server-stamped ``sender``. A token on ``:cmdq``, if
  present, must still match. ``from`` / ``rank`` are ignored.
- ``:cmd`` and ``:cmdq`` share one seq per epoch. A higher seq is accepted
  (gaps are fine, so a time-based seq such as ``Date.now()`` works). An equal
  seq is a duplicate. A lower seq is a controller restart: it is accepted only
  after ``SEQ_RESET_GAP_S`` with no accepted command, then seq starts over.
  Replaying an old higher seq after that reset is not rejected — the site
  should use a time-based seq. E-stop does not consume seq.
- E-stop is idempotent and skips seq, timestamp, and lease checks. It still
  requires the bot, the current epoch, and the token (``:cmd``) or sender
  (``:cmdq``). A skewed clock cannot hide an e-stop behind an expired lease.
- Timestamp freshness is not enforced. The Pi clock can be minutes off before
  NTP. A non-finite timestamp is still rejected, except on e-stop.
- ``:ctl`` is not authority. The bridge treats it as a cue to poll. Epoch,
  controller, token, and lease come only from that poll.
- The motion deadman is 0.5 s from the last accepted joy/nudge/drive that is
  actually asking for motion. Discrete commands do not refresh it.
- "Supabase unreachable for a few seconds" is 5 s without a successful poll
  or an accepted command, and only after one of those has happened.
- Control poll default is 2 s, clamped to the spec's 2–5 s range.
- Joy accepts the website shape ``{type:'joy', a, b}`` and the older
  ``axes`` / ``buttons`` shape. Numbers must be finite. lin/ang and axes are
  clamped to the dashboard limits. Button bits above the first 16 are masked
  off. Nav goals and blacklist points are clamped to ±1000 m, and a blacklist
  duration is clamped to 0–24 h.
"""

from __future__ import annotations

import base64
import hmac
import json
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

# joy / nudge / drive must keep arriving, or motion is zeroed.
MOTION_WATCHDOG_S = 0.5
# Same gap the deadman uses. A browser reload can send seq 1 again once this
# long has passed with no accepted command, instead of locking the driver out
# until the lease expires.
SEQ_RESET_GAP_S = MOTION_WATCHDOG_S
# A few seconds, longer than one missed poll so a single blip does not stop the robot.
SUPABASE_UNREACHABLE_S = 5.0
CONTROL_POLL_MIN_S = 2.0
CONTROL_POLL_MAX_S = 5.0
CONTROL_POLL_S = 2.0

LIN_LIMIT = 0.3
ANG_LIMIT = 1.0
AXIS_LIMIT = 1.0
JOY_AXES = 8
# Xbox / the web gamepad use 16 button bits. Higher bits are not a command.
BUTTON_MASK = 0xFFFF
# Odom metres. Far enough for this robot, still rejects a runaway number.
POSE_LIMIT_M = 1000.0
# A no-go zone longer than a day is clamped. 0 stays "permanent" on the dashboard.
BLACKLIST_DURATION_MAX_S = 24.0 * 60.0 * 60.0

MOTION_TYPES = ("joy", "nudge", "drive")
# Same set public.send_command allows, plus the bridge-local rate switch.
ALLOWED_TYPES = frozenset({
    "estop",
    "nudge",
    "drive",
    "joy",
    "nav_goal",
    "nav_cancel",
    "vision",
    "save_map",
    "run_tool",
    "stop_tool",
    "blacklist",
    "set_rate",
})

# Dashboard TOOLS. Unknown tools and unknown param names are rejected.
# (default, min, max) — omitted params are left for the dashboard default.
_TOOL_PARAMS: dict[str, dict[str, tuple[float, float, float]]] = {
    "imu_test": {},
    "imu_calibration": {
        "speed": (0.5, 0.1, 1.0),
        "rotations": (2.0, 0.5, 4.0),
    },
    "drive_distance": {
        "distance": (1.0, -5.0, 5.0),
        "max_vel": (0.3, 0.05, 0.5),
    },
}

_MAP_NAME = re.compile(r"[A-Za-z0-9_-]{1,64}")
_STRIP_KEYS = ("from", "rank")


@dataclass(frozen=True)
class VerifyResult:
    ok: bool
    reason: str
    cmd_type: str | None = None
    cmd: dict | None = None


class MotionGuard:
    """Whether a zero is still owed, and whether new motion must be cancelled.

    ``hard`` is a lease expiry or a Supabase outage: a zero is retried until
    it succeeds, and motion posted while ``hard`` is set is followed by
    another zero. A deadman or an epoch change is soft: a newer accepted
    motion cancels it.
    """

    def __init__(self) -> None:
        self.hard = False
        self._gen = 0
        self._owed_gen: int | None = None

    @property
    def owed_gen(self) -> int | None:
        return self._owed_gen

    @property
    def owed(self) -> bool:
        return self._owed_gen is not None

    def set_hard(self, hard: bool) -> None:
        self.hard = hard

    def arm(self, *, hard: bool) -> int:
        self._gen += 1
        self._owed_gen = self._gen
        if hard:
            self.hard = True
        return self._gen

    def release_for_motion(self) -> None:
        """A legitimate motion command. Cancels a soft zero; a hard stop stays."""
        self._gen += 1
        if not self.hard:
            self._owed_gen = None

    def still_owed(self, gen: int) -> bool:
        return self._owed_gen == gen

    def mark_zeroed(self, gen: int) -> None:
        if self._owed_gen == gen:
            self._owed_gen = None


def flag_enabled(value: str | None, default: bool) -> bool:
    """Parse an env flag. Empty / unknown values keep the default."""
    if value is None:
        return default
    text = value.strip().lower()
    if not text:
        return default
    if text in ("0", "false", "no", "off"):
        return False
    if text in ("1", "true", "yes", "on"):
        return True
    return default


def clamp_poll_interval(seconds: float) -> float:
    try:
        value = float(seconds)
    except (TypeError, ValueError):
        return CONTROL_POLL_S
    if not math.isfinite(value):
        return CONTROL_POLL_S
    return min(CONTROL_POLL_MAX_S, max(CONTROL_POLL_MIN_S, value))


def looks_like_service_key(value: str | None) -> bool:
    """True for an sb_secret_ key or a JWT whose payload role is service_role.

    The signature is not checked. This only refuses to start; it is not auth.
    """
    if value is None:
        return False
    text = value.strip()
    if not text:
        return False
    if text.startswith("sb_secret_"):
        return True
    parts = text.split(".")
    if len(parts) != 3:
        return False
    payload = parts[1]
    pad = "=" * (-len(payload) % 4)
    try:
        raw = base64.urlsafe_b64decode(payload + pad)
        data = json.loads(raw.decode("utf-8"))
    except (ValueError, UnicodeDecodeError, json.JSONDecodeError):
        return False
    return isinstance(data, dict) and data.get("role") == "service_role"


def parse_timestamptz(value: Any) -> float | None:
    """Server timestamptz or unix time → seconds. None if it cannot be read."""
    if value is None or isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        number = float(value)
        if not math.isfinite(number):
            return None
        # Millisecond timestamps are far above any second-based lease.
        if abs(number) > 1e11:
            return number / 1000.0
        return number
    if not isinstance(value, str):
        return None
    text = value.strip()
    if not text:
        return None
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    if "T" not in text and " " in text:
        text = text.replace(" ", "T", 1)
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.timestamp()


def _as_int(value: Any) -> int | None:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        if not math.isfinite(value) or not value.is_integer():
            return None
        return int(value)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        try:
            return int(text, 10)
        except ValueError:
            return None
    return None


def _finite_float(value: Any) -> float | None:
    if isinstance(value, bool) or value is None:
        return None
    if isinstance(value, (int, float)):
        number = float(value)
    elif isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        try:
            number = float(text)
        except ValueError:
            return None
    else:
        return None
    if not math.isfinite(number):
        return None
    return number


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _casefold(value: Any) -> str | None:
    if value is None or isinstance(value, bool):
        return None
    text = str(value).strip()
    if not text:
        return None
    return text.casefold()


def _secrets_equal(presented: str | None, expected: str | None) -> bool:
    """Case-insensitive compare that does not leak the token through timing."""
    if not presented or not expected:
        return False
    left = presented.encode("utf-8")
    right = expected.encode("utf-8")
    if len(left) != len(right):
        return False
    return hmac.compare_digest(left, right)


def _strip_rank(body: dict) -> dict:
    """Drop client rank claims. They are not an authority signal."""
    out = {k: v for k, v in body.items() if k not in _STRIP_KEYS}
    cmd = out.get("cmd")
    if isinstance(cmd, dict):
        out["cmd"] = {k: v for k, v in cmd.items() if k not in _STRIP_KEYS}
    return out


def normalize_message(message: Any) -> dict | None:
    """Unwrap a Realtime broadcast down to the dict that carries the fields."""
    current = message
    if isinstance(current, str):
        try:
            current = json.loads(current)
        except json.JSONDecodeError:
            return None
    if not isinstance(current, dict):
        return None
    for _ in range(3):
        current = _strip_rank(current)
        inner = current.get("payload")
        if isinstance(inner, str):
            try:
                inner = json.loads(inner)
            except json.JSONDecodeError:
                return current
        if isinstance(inner, dict) and not _looks_actionable(current):
            current = inner
            continue
        return current
    return _strip_rank(current) if isinstance(current, dict) else None


def _looks_actionable(body: dict) -> bool:
    if "cmd" in body or "seq" in body or "sender" in body:
        return True
    if "epoch" in body and ("token" in body or "controller" in body or "cmd" in body):
        return True
    return False


def _command_dict(env: dict) -> dict | None:
    cmd = env.get("cmd")
    if isinstance(cmd, dict):
        return {k: v for k, v in cmd.items() if k not in _STRIP_KEYS}
    if "type" in env:
        skip = {"epoch", "token", "seq", "ts", "sender", "bot_id", "botId", "cmd", "payload"}
        return {k: v for k, v in env.items() if k not in skip and k not in _STRIP_KEYS}
    return None


def _auth_field(env: dict, cmd: dict, name: str) -> Any:
    if env.get(name) is not None:
        return env.get(name)
    return cmd.get(name)


def _joy_axes_raw(cmd: dict) -> Any:
    """Website sends ``a``. Older callers send ``axes``. 0 and [] are real values."""
    if "a" in cmd:
        return cmd.get("a")
    return cmd.get("axes")


def _joy_buttons_raw(cmd: dict) -> Any:
    if "b" in cmd:
        return cmd.get("b")
    return cmd.get("buttons")


def peek_command_type(message: Any) -> str | None:
    """Best-effort type peek so e-stop can bypass the normal callback queue."""
    env = normalize_message(message)
    if not env:
        return None
    cmd = env.get("cmd")
    if isinstance(cmd, dict) and isinstance(cmd.get("type"), str):
        return cmd["type"]
    typ = env.get("type")
    return typ if isinstance(typ, str) else None


def motion_command(cmd: dict) -> bool:
    """True when this command is asking the wheels to move."""
    typ = cmd.get("type")
    if typ in ("nudge", "drive"):
        lin = _finite_float(cmd.get("lin"))
        ang = _finite_float(cmd.get("ang"))
        if lin is None and ang is None:
            return False
        return abs(lin or 0.0) > 1e-3 or abs(ang or 0.0) > 1e-3
    if typ == "joy":
        axes = _joy_axes_raw(cmd)
        if not isinstance(axes, (list, tuple)):
            return False
        for axis in axes:
            number = _finite_float(axis)
            if number is not None and abs(number) > 0.02:
                return True
        return False
    return False


def sanitize_command(cmd: dict) -> tuple[dict | None, str | None]:
    """Return a cleaned command, or (None, \"value\") when a field is illegal.

    Clamps match the dashboard: lin ±0.3, ang ±1.0, joy axes [-1, 1].
    """
    if not isinstance(cmd, dict):
        return None, "value"
    typ = cmd.get("type")
    if typ not in ALLOWED_TYPES:
        return None, "type"

    if typ == "estop" or typ in ("nav_cancel", "stop_tool"):
        return {"type": typ}, None

    if typ in ("nudge", "drive"):
        lin = _finite_float(cmd.get("lin", 0))
        ang = _finite_float(cmd.get("ang", 0))
        if lin is None or ang is None:
            return None, "value"
        return {
            "type": typ,
            "lin": _clamp(lin, LIN_LIMIT),
            "ang": _clamp(ang, ANG_LIMIT),
        }, None

    if typ == "joy":
        raw_axes = _joy_axes_raw(cmd)
        if raw_axes is None:
            raw_axes = [0.0] * JOY_AXES
        if not isinstance(raw_axes, (list, tuple)):
            return None, "value"
        axes: list[float] = []
        for axis in list(raw_axes)[:JOY_AXES]:
            number = _finite_float(axis)
            if number is None:
                return None, "value"
            axes.append(_clamp(number, AXIS_LIMIT))
        while len(axes) < JOY_AXES:
            axes.append(0.0)
        raw_buttons = _joy_buttons_raw(cmd)
        if raw_buttons is None:
            raw_buttons = 0
        buttons = _as_int(raw_buttons)
        if buttons is None or buttons < 0:
            return None, "value"
        buttons &= BUTTON_MASK
        return {
            "type": "joy",
            "a": axes,
            "b": buttons,
            "axes": list(axes),
            "buttons": buttons,
        }, None

    if typ == "nav_goal":
        x = _finite_float(cmd.get("x"))
        y = _finite_float(cmd.get("y"))
        if x is None or y is None:
            return None, "value"
        return {
            "type": "nav_goal",
            "x": _clamp(x, POSE_LIMIT_M),
            "y": _clamp(y, POSE_LIMIT_M),
        }, None

    if typ == "vision":
        on = cmd.get("on", False)
        if isinstance(on, bool):
            flag = on
        elif on in (0, 1):
            flag = bool(on)
        else:
            return None, "value"
        return {"type": "vision", "on": flag}, None

    if typ == "save_map":
        name = cmd.get("name", "")
        if name is None:
            name = ""
        if not isinstance(name, str):
            return None, "value"
        if name != "" and _MAP_NAME.fullmatch(name) is None:
            return None, "value"
        return {"type": "save_map", "name": name}, None

    if typ == "run_tool":
        tool = cmd.get("tool")
        if not isinstance(tool, str) or tool not in _TOOL_PARAMS:
            return None, "value"
        spec = _TOOL_PARAMS[tool]
        params = cmd.get("params", {})
        if params is None:
            params = {}
        if not isinstance(params, dict):
            return None, "value"
        if any(key not in spec for key in params):
            return None, "value"
        clean: dict[str, float] = {}
        for key, value in params.items():
            number = _finite_float(value)
            if number is None:
                return None, "value"
            _default, lo, hi = spec[key]
            clean[key] = min(hi, max(lo, number))
        return {"type": "run_tool", "tool": tool, "params": clean}, None

    if typ == "blacklist":
        action = cmd.get("action")
        if action == "clear":
            return {"type": "blacklist", "action": "clear"}, None
        if action not in (None, "add"):
            return None, "value"
        x = _finite_float(cmd.get("x"))
        y = _finite_float(cmd.get("y"))
        if x is None or y is None:
            return None, "value"
        radius = _finite_float(cmd.get("radius", 0.8))
        duration = _finite_float(cmd.get("duration", 300))
        if radius is None or duration is None:
            return None, "value"
        return {
            "type": "blacklist",
            "x": _clamp(x, POSE_LIMIT_M),
            "y": _clamp(y, POSE_LIMIT_M),
            "radius": min(5.0, max(0.1, radius)),
            "duration": min(BLACKLIST_DURATION_MAX_S, max(0.0, duration)),
        }, None

    if typ == "set_rate":
        mode = cmd.get("mode")
        if mode not in ("full", "board"):
            return None, "value"
        return {"type": "set_rate", "mode": mode}, None

    return None, "type"


class ControlTracker:
    """Lease, token, seq, and the watchdog edges for one bot.

    Touch this from one thread (the bridge asyncio loop). Poll / Realtime
    callbacks must hop onto that loop before calling in.
    """

    def __init__(self, bot_id: str):
        self.bot_id = bot_id
        self.private_enabled = False
        self.epoch: int | None = None
        self.token: str | None = None
        self.controller: str | None = None
        self.lease_expires_at: float | None = None
        self._last_seq: dict[int, int] = {}
        self._last_seq_mono: dict[int, float] = {}
        self._last_motion_mono: float | None = None
        self._motion_latched = False
        self._lease_was_valid = False
        self._last_contact_mono: float | None = None
        self._zeroed: set[str] = set()

    def note_contact(self, now_mono: float) -> None:
        """A successful control poll, or a command that passed verification."""
        self.private_enabled = True
        self._last_contact_mono = now_mono
        self._zeroed.discard("supabase")

    def ctl_needs_poll(self, message: Any) -> bool:
        """``bot:<id>:ctl`` only means "poll now". It does not change epoch."""
        env = normalize_message(message)
        if not env:
            return False
        bot = env.get("bot_id") or env.get("botId")
        if bot and str(bot) != self.bot_id:
            return False
        return True

    def apply_poll(self, control: dict, token: dict | None, now_mono: float) -> list[str]:
        """Rows from bot_control and bot_control_tokens (token row may be absent)."""
        if not isinstance(control, dict):
            return []
        bot = control.get("bot_id")
        if bot and str(bot) != self.bot_id:
            return []
        epoch = _as_int(control.get("epoch"))
        if epoch is None:
            return []
        if self.epoch is not None and epoch < self.epoch:
            self.note_contact(now_mono)
            return []
        reasons = self._set_epoch(epoch)
        self.token = _token_for_epoch(token, epoch)
        self.lease_expires_at = parse_timestamptz(control.get("lease_expires_at"))
        controller = control.get("controller")
        self.controller = str(controller) if controller else None
        self.note_contact(now_mono)
        return reasons

    def verify(
        self,
        message: Any,
        *,
        source: str,
        now_s: float,
        now_mono: float,
    ) -> VerifyResult:
        env = normalize_message(message)
        if not env:
            return VerifyResult(False, "malformed")
        bot = env.get("bot_id") or env.get("botId")
        if bot and str(bot) != self.bot_id:
            return VerifyResult(False, "bot")
        cmd = _command_dict(env)
        if not cmd:
            return VerifyResult(False, "malformed")
        typ = cmd.get("type")
        if typ not in ALLOWED_TYPES:
            return VerifyResult(False, "type", cmd_type=typ if isinstance(typ, str) else None)
        if source not in ("cmd", "cmdq"):
            return VerifyResult(False, "source", cmd_type=typ, cmd=cmd)

        epoch = _as_int(_auth_field(env, cmd, "epoch"))
        if epoch is None or self.epoch is None or epoch != self.epoch:
            return VerifyResult(False, "epoch", cmd_type=typ, cmd=cmd)
        # E-stop ignores the lease. A skewed Pi clock, or a lease the bridge
        # has not learned yet, must not swallow a stop. Epoch and token stay.
        if typ != "estop" and (
            self.lease_expires_at is None or not (self.lease_expires_at > now_s)
        ):
            return VerifyResult(False, "lease", cmd_type=typ, cmd=cmd)

        if source == "cmd":
            presented = _casefold(_auth_field(env, cmd, "token"))
            expected = _casefold(self.token)
            if not _secrets_equal(presented, expected):
                return VerifyResult(False, "token", cmd_type=typ, cmd=cmd)
        else:
            sender = _casefold(env.get("sender"))
            controller = _casefold(self.controller)
            # Server stamp only. A sender buried inside cmd is the client's, ignore it.
            if not sender or not controller or sender != controller:
                return VerifyResult(False, "sender", cmd_type=typ, cmd=cmd)
            if env.get("token") is not None or cmd.get("token") is not None:
                presented = _casefold(_auth_field(env, cmd, "token"))
                expected = _casefold(self.token)
                if not _secrets_equal(presented, expected):
                    return VerifyResult(False, "token", cmd_type=typ, cmd=cmd)

        clean, reason = sanitize_command(cmd)
        if clean is None:
            return VerifyResult(False, reason or "value", cmd_type=typ, cmd=cmd)

        if typ != "estop":
            seq = _as_int(_auth_field(env, cmd, "seq"))
            if seq is None:
                return VerifyResult(False, "seq", cmd_type=typ, cmd=clean)
            if not self._seq_ok(epoch, seq, now_mono):
                return VerifyResult(False, "seq", cmd_type=typ, cmd=clean)
            ts = _auth_field(env, cmd, "ts")
            if _finite_float(ts) is None:
                return VerifyResult(False, "ts", cmd_type=typ, cmd=clean)
            self._last_seq[epoch] = seq
            self._last_seq_mono[epoch] = now_mono

        if typ in MOTION_TYPES:
            self._last_motion_mono = now_mono
            self._motion_latched = motion_command(clean)
        self.note_contact(now_mono)
        return VerifyResult(True, "ok", cmd_type=typ, cmd=clean)

    def fault_active(self, now_s: float, now_mono: float) -> bool:
        """Lease already observed and now expired, or contact has gone stale."""
        if not self.private_enabled:
            return False
        lease_open = self.lease_expires_at is not None and self.lease_expires_at > now_s
        if self._lease_was_valid and not lease_open:
            return True
        if (
            self._last_contact_mono is not None
            and now_mono - self._last_contact_mono >= SUPABASE_UNREACHABLE_S
        ):
            return True
        return False

    def tick(self, now_s: float, now_mono: float) -> list[str]:
        """Watchdog edges. Each fault zeros once until it clears."""
        if not self.private_enabled:
            return []
        reasons: list[str] = []

        lease_open = self.lease_expires_at is not None and self.lease_expires_at > now_s
        if lease_open:
            self._lease_was_valid = True
            self._zeroed.discard("lease")
        elif self._lease_was_valid and "lease" not in self._zeroed:
            reasons.append("lease_expired")
            self._zeroed.add("lease")

        unreachable = (
            self._last_contact_mono is not None
            and now_mono - self._last_contact_mono >= SUPABASE_UNREACHABLE_S
        )
        if unreachable and "supabase" not in self._zeroed:
            reasons.append("supabase_unreachable")
            self._zeroed.add("supabase")
        elif not unreachable:
            self._zeroed.discard("supabase")

        if (
            self._motion_latched
            and self._last_motion_mono is not None
            and now_mono - self._last_motion_mono >= MOTION_WATCHDOG_S
        ):
            reasons.append("command_timeout")
            self._motion_latched = False
        if lease_open is False and "lease_expired" in reasons:
            self._motion_latched = False
        if unreachable:
            self._motion_latched = False
        return reasons

    def _seq_ok(self, epoch: int, seq: int, now_mono: float) -> bool:
        last = self._last_seq.get(epoch)
        if last is None or seq > last:
            return True
        if seq == last:
            return False
        previous = self._last_seq_mono.get(epoch)
        if previous is None:
            return True
        # The exact boundary counts. Subtracting two floats can land a hair under.
        return (now_mono - previous) + 1e-9 >= SEQ_RESET_GAP_S

    def _set_epoch(self, epoch: int) -> list[str]:
        if self.epoch is None or epoch == self.epoch:
            self.epoch = epoch
            return []
        self.epoch = epoch
        self.token = None
        self.lease_expires_at = None
        self._lease_was_valid = False
        self._motion_latched = False
        self._last_seq.clear()
        self._last_seq_mono.clear()
        return ["epoch_change"]


def _token_for_epoch(token_row: dict | None, epoch: int) -> str | None:
    if not isinstance(token_row, dict):
        return None
    token_epoch = _as_int(token_row.get("epoch"))
    if token_epoch != epoch:
        return None
    raw = token_row.get("token")
    if raw is None or isinstance(raw, bool):
        return None
    text = str(raw).strip()
    return text or None
