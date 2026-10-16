"""Pure control-lease checks for the private bridge path.

No network, no ROS. bridge.py applies the results. Velocity is never decided
here; the caller zeros motion through the existing dashboard / gamepad HTTP
paths, which publish /cmd_vel so safety_stop stays in front of the motors.

Assumptions that the spec left open (see the PR notes):

- Command ``ts`` is fresh if it is within 2 s of now as unix milliseconds
  (what ``send_command`` stamps) or as unix seconds (``:cmd`` did not name a unit).
- ``:cmd`` requires ``token``. ``:cmdq`` does not: ``send_command`` stamps
  ``sender``, not a token. A token on ``:cmdq``, if present, must still match.
- ``:cmd`` and ``:cmdq`` share one strictly-increasing seq per epoch. Gaps are
  allowed. Seq restarts when the epoch changes.
- The motion deadman is 0.5 s from the last accepted joy/nudge/drive. Discrete
  commands do not refresh it and are not cancelled by the zero.
- "Supabase unreachable for a few seconds" is 5 s without a successful private
  contact, and only after the private path has connected once.
- Control poll default is 2 s, clamped to the spec's 2–5 s range.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

# joy / nudge / drive must keep arriving, or motion is zeroed.
MOTION_WATCHDOG_S = 0.5
# A few seconds, longer than one missed poll so a single blip does not stop the robot.
SUPABASE_UNREACHABLE_S = 5.0
TS_WINDOW_S = 2.0
CONTROL_POLL_MIN_S = 2.0
CONTROL_POLL_MAX_S = 5.0
CONTROL_POLL_S = 2.0

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

_STRIP_KEYS = ("from", "rank")


@dataclass(frozen=True)
class VerifyResult:
    ok: bool
    reason: str
    cmd_type: str | None = None
    cmd: dict | None = None


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


def _casefold(value: Any) -> str | None:
    if value is None or isinstance(value, bool):
        return None
    text = str(value).strip()
    if not text:
        return None
    return text.casefold()


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


def ts_within_window(ts: Any, now_s: float, window_s: float = TS_WINDOW_S) -> bool:
    if isinstance(ts, bool) or ts is None:
        return False
    try:
        value = float(ts)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(value):
        return False
    as_ms = abs(now_s * 1000.0 - value) <= window_s * 1000.0
    as_s = abs(now_s - value) <= window_s
    return as_ms or as_s


def _motion_active(cmd: dict) -> bool:
    typ = cmd.get("type")
    if typ in ("nudge", "drive"):
        try:
            lin = float(cmd.get("lin") or 0)
            ang = float(cmd.get("ang") or 0)
        except (TypeError, ValueError):
            return True
        return abs(lin) > 1e-3 or abs(ang) > 1e-3
    if typ == "joy":
        axes = cmd.get("axes") or []
        if not isinstance(axes, (list, tuple)):
            return True
        try:
            return any(abs(float(axis)) > 0.02 for axis in axes)
        except (TypeError, ValueError):
            return True
    return False


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
        self._last_motion_mono: float | None = None
        self._motion_latched = False
        self._lease_was_valid = False
        self._last_contact_mono: float | None = None
        self._zeroed: set[str] = set()

    def note_contact(self, now_mono: float) -> None:
        """A successful private poll, subscribe, or inbound event."""
        self.private_enabled = True
        self._last_contact_mono = now_mono
        self._zeroed.discard("supabase")

    def apply_ctl(self, message: Any, now_mono: float) -> list[str]:
        """bot:<id>:ctl event ``control``. Token and lease stay unknown until poll."""
        env = normalize_message(message)
        if not env:
            return []
        bot = env.get("bot_id") or env.get("botId")
        if bot and str(bot) != self.bot_id:
            return []
        epoch = _as_int(env.get("epoch"))
        if epoch is None:
            return []
        if self.epoch is not None and epoch < self.epoch:
            self.note_contact(now_mono)
            return []
        reasons = self._set_epoch(epoch)
        controller = env.get("controller")
        self.controller = str(controller) if controller else None
        self.note_contact(now_mono)
        return reasons

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
        if self.lease_expires_at is None or not (self.lease_expires_at > now_s):
            return VerifyResult(False, "lease", cmd_type=typ, cmd=cmd)

        if source == "cmd":
            presented = _casefold(_auth_field(env, cmd, "token"))
            expected = _casefold(self.token)
            if not presented or not expected or presented != expected:
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
                if not presented or not expected or presented != expected:
                    return VerifyResult(False, "token", cmd_type=typ, cmd=cmd)

        seq = _as_int(_auth_field(env, cmd, "seq"))
        if seq is None:
            return VerifyResult(False, "seq", cmd_type=typ, cmd=cmd)
        last = self._last_seq.get(epoch, 0)
        if seq <= last:
            return VerifyResult(False, "seq", cmd_type=typ, cmd=cmd)

        ts = _auth_field(env, cmd, "ts")
        if not ts_within_window(ts, now_s):
            return VerifyResult(False, "ts", cmd_type=typ, cmd=cmd)

        self._last_seq[epoch] = seq
        if typ in MOTION_TYPES:
            self._last_motion_mono = now_mono
            self._motion_latched = _motion_active(cmd)
        return VerifyResult(True, "ok", cmd_type=typ, cmd=cmd)

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
