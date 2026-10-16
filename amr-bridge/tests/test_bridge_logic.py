#!/usr/bin/env python3
"""Logic tests for amr_telemetry_bridge + launch wiring (no ROS required)."""

from __future__ import annotations

import ast
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BRIDGE_ROOT = ROOT / "amr-bridge"
sys.path.insert(0, str(BRIDGE_ROOT))

from amr_telemetry_bridge.control import (  # noqa: E402
    CONTROL_POLL_MAX_S,
    CONTROL_POLL_MIN_S,
    CONTROL_POLL_S,
    BLACKLIST_DURATION_MAX_S,
    BUTTON_MASK,
    MOTION_WATCHDOG_S,
    POSE_LIMIT_M,
    SEQ_RESET_GAP_S,
    SUPABASE_UNREACHABLE_S,
    ControlTracker,
    MotionGuard,
    clamp_poll_interval,
    flag_enabled,
    looks_like_service_key,
    motion_command,
    parse_timestamptz,
    sanitize_command,
)
from amr_telemetry_bridge.protocol import (  # noqa: E402
    BOARD_RATE_HZ,
    BOT_IDS,
    CMD_ACK_EVENT,
    CMD_EVENT,
    CTL_EVENT,
    FULL_RATE_HZ,
    HEARTBEAT_OFFLINE_MS,
    STATUS_EVENTS,
    channel_name,
    private_topic,
)


def _load_env_file(path: Path) -> dict[str, str]:
    """Mirror of launch/amr_bridge.launch.py::_load_env_file (values only)."""
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line.startswith("export "):
            line = line[len("export "):].strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        k, _, v = line.partition("=")
        out[k.strip()] = v.strip().strip('"').strip("'")
    return out


def _missing_required(env: dict[str, str]) -> list[str]:
    """Mirror of launch/amr_bridge.launch.py::_missing_required."""
    missing: list[str] = []
    if not env.get("AMR_SUPABASE_URL"):
        missing.append("AMR_SUPABASE_URL")
    if not env.get("AMR_SUPABASE_KEY"):
        missing.append("AMR_SUPABASE_KEY")
    if not env.get("AMR_BRIDGE_TOKEN"):
        missing.append("AMR_BRIDGE_TOKEN")
    return missing


class ProtocolTests(unittest.TestCase):
    def test_bot_ids(self):
        self.assertEqual(BOT_IDS, ("amr4", "hardware-bot"))

    def test_channel_name(self):
        self.assertEqual(channel_name("amr4"), "amr-bot-amr4")
        self.assertEqual(channel_name("hardware-bot"), "amr-bot-hardware-bot")

    def test_private_topics(self):
        self.assertEqual(private_topic("hardware-bot", "status"), "bot:hardware-bot:status")
        self.assertEqual(private_topic("hardware-bot", "cmd"), "bot:hardware-bot:cmd")
        self.assertEqual(private_topic("hardware-bot", "cmdq"), "bot:hardware-bot:cmdq")
        self.assertEqual(private_topic("hardware-bot", "ctl"), "bot:hardware-bot:ctl")
        self.assertEqual(STATUS_EVENTS, ("telemetry", "map", "log", "hb", "cmd_ack"))
        self.assertEqual(CMD_EVENT, "command")
        self.assertEqual(CMD_ACK_EVENT, "cmd_ack")
        self.assertEqual(CTL_EVENT, "control")
        with self.assertRaises(ValueError):
            private_topic("hardware-bot", "presence")

    def test_rates(self):
        self.assertEqual(FULL_RATE_HZ, 10)
        self.assertEqual(BOARD_RATE_HZ, 1)
        self.assertEqual(HEARTBEAT_OFFLINE_MS, 3000)


class EnvFileTests(unittest.TestCase):
    def test_missing_file(self):
        self.assertEqual(_load_env_file(Path("/tmp/does-not-exist-amr.env")), {})

    def test_parse_and_strip(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "env"
            p.write_text(
                "# comment\n"
                "AMR_BOT_ID=hardware-bot\n"
                "AMR_SUPABASE_URL='https://example.supabase.co'\n"
                'AMR_SUPABASE_KEY="sb_publishable_x"\n'
                "AMR_BRIDGE_TOKEN=tok123\n"
                "export AMR_DASHBOARD_URL=http://127.0.0.1:8080\n"
                "\n"
            )
            env = _load_env_file(p)
        self.assertEqual(env["AMR_BOT_ID"], "hardware-bot")
        self.assertEqual(env["AMR_SUPABASE_URL"], "https://example.supabase.co")
        self.assertEqual(env["AMR_SUPABASE_KEY"], "sb_publishable_x")
        self.assertEqual(env["AMR_BRIDGE_TOKEN"], "tok123")
        self.assertEqual(env["AMR_DASHBOARD_URL"], "http://127.0.0.1:8080")
        self.assertEqual(_missing_required(env), [])

    def test_missing_required_skips_start(self):
        self.assertEqual(
            _missing_required({"AMR_SUPABASE_URL": "x"}),
            ["AMR_SUPABASE_KEY", "AMR_BRIDGE_TOKEN"],
        )

    def test_service_key_does_not_satisfy_required(self):
        self.assertEqual(
            _missing_required({"AMR_SUPABASE_SERVICE_KEY": "svc"}),
            ["AMR_SUPABASE_URL", "AMR_SUPABASE_KEY", "AMR_BRIDGE_TOKEN"],
        )
        self.assertEqual(
            _missing_required({
                "AMR_SUPABASE_URL": "https://example.supabase.co",
                "AMR_SUPABASE_SERVICE_KEY": "svc",
            }),
            ["AMR_SUPABASE_KEY", "AMR_BRIDGE_TOKEN"],
        )


class LaunchWiringTests(unittest.TestCase):
    def test_launch_robot_ast_has_bridge_timer(self):
        src = (ROOT / "launch" / "launch_robot.launch.py").read_text()
        tree = ast.parse(src)
        text = ast.dump(tree)
        self.assertIn("enable_amr_bridge", src)
        self.assertIn("amr_bridge.launch.py", src)
        self.assertIn("hardware-bot", src)
        # Delayed include after dashboard
        self.assertIn("TimerAction(period=5.0, actions=[bridge])", src)
        self.assertIn('dashboard": "http://127.0.0.1:8080"', src.replace("'", '"') or src)
        self.assertIn("8765", src)
        self.assertIsInstance(tree, ast.Module)
        self.assertTrue(text)

    def test_amr_bridge_launch_resolves_package(self):
        src = (ROOT / "launch" / "amr_bridge.launch.py").read_text()
        self.assertIn("_resolve_bridge_root", src)
        self.assertIn("PYTHONPATH", src)
        self.assertIn("amr_telemetry_bridge", src)
        self.assertIn("[amr_bridge] SKIPPED:", src)
        self.assertIn('default_value="auto"', src)
        self.assertIn('"-u"', src)
        ast.parse(src)

    def test_vendored_package_importable(self):
        self.assertTrue((BRIDGE_ROOT / "amr_telemetry_bridge" / "bridge.py").is_file())
        self.assertTrue((BRIDGE_ROOT / ".env.pi.example").is_file())
        # Example must not look like a real token
        example = (BRIDGE_ROOT / ".env.pi.example").read_text()
        self.assertIn("REPLACE", example)
        self.assertIn("AMR_BRIDGE_TOKEN=REPLACE_WITH_BRIDGE_TOKEN", example)
        self.assertIn("AMR_BRIDGE_EMAIL=REPLACE_WITH_BRIDGE_EMAIL", example)
        self.assertIn("AMR_BRIDGE_PASSWORD=REPLACE_WITH_BRIDGE_PASSWORD", example)
        self.assertIn("AMR_PUBLIC_CHANNEL=0", example)
        self.assertNotIn("AMR_SUPABASE_SERVICE_KEY=", example)
        self.assertNotIn("eyJ", example)  # no JWT literals


class DashboardApiContractTests(unittest.TestCase):
    """Bridge command map must match robot_dashboard routes."""

    def test_dashboard_routes_exist(self):
        dash = (ROOT / "Ros_lidar_bot" / "robot_dashboard_node.py").read_text()
        for path in (
            "/events",
            "/camera.mjpg",
            "/api/estop",
            "/api/nudge",
            "/api/nav_goal",
            "/api/nav_cancel",
            "/api/vision",
            "/api/save_map",
            "/api/run",
            "/api/stop",
            "/api/blacklist",
        ):
            self.assertIn(path, dash)

    def test_bridge_posts_expected_paths(self):
        bridge = (BRIDGE_ROOT / "amr_telemetry_bridge" / "bridge.py").read_text()
        for path in (
            "/api/estop",
            "/api/nudge",
            "/api/nav_goal",
            "/api/nav_cancel",
            "/api/vision",
            "/api/save_map",
            "/api/run",
            "/api/stop",
            "/api/blacklist",
            "/events",
        ):
            self.assertIn(path, bridge)

    def test_web_gamepad_joy_route(self):
        gp = (ROOT / "Ros_lidar_bot" / "web_gamepad_node.py").read_text()
        self.assertIn("/api/joy", gp)
        self.assertIn("8765", gp)


class LaunchSafetyTests(unittest.TestCase):
    def test_vendored_launch_matches_package_launch(self):
        packaged = (ROOT / "launch" / "amr_bridge.launch.py").read_text()
        vendored = (BRIDGE_ROOT / "launch" / "amr_bridge.launch.py").read_text()
        self.assertEqual(packaged, vendored)

    def test_launch_drops_service_key(self):
        src = (ROOT / "launch" / "amr_bridge.launch.py").read_text()
        self.assertIn('env.pop("AMR_SUPABASE_SERVICE_KEY", None)', src)
        self.assertNotIn("or AMR_SUPABASE_SERVICE_KEY", src)


class EnvFlagTests(unittest.TestCase):
    def test_flag_defaults_and_words(self):
        self.assertTrue(flag_enabled(None, True))
        self.assertFalse(flag_enabled(None, False))
        self.assertTrue(flag_enabled("", True))
        self.assertTrue(flag_enabled("1", False))
        self.assertTrue(flag_enabled("yes", False))
        self.assertFalse(flag_enabled("0", True))
        self.assertFalse(flag_enabled("off", True))
        self.assertTrue(flag_enabled("maybe", True))

    def test_poll_interval_stays_in_spec_range(self):
        self.assertEqual(CONTROL_POLL_S, 2.0)
        self.assertGreaterEqual(CONTROL_POLL_S, CONTROL_POLL_MIN_S)
        self.assertLessEqual(CONTROL_POLL_S, CONTROL_POLL_MAX_S)
        self.assertEqual(clamp_poll_interval(1), 2.0)
        self.assertEqual(clamp_poll_interval(9), 5.0)
        self.assertEqual(clamp_poll_interval(3.5), 3.5)
        self.assertEqual(clamp_poll_interval(float("nan")), CONTROL_POLL_S)


NOW = 1_700_000_000.0


def _joy(**cmd_over):
    cmd = {"type": "joy", "axes": [0.0, 0.5, 0, 0, 0, 0, 0, 0], "buttons": 0}
    cmd.update(cmd_over)
    return cmd


class ControlTrackerTests(unittest.TestCase):
    def _arm(
        self,
        gate: ControlTracker | None = None,
        *,
        epoch: int = 3,
        token: str = "tok-1",
        controller: str = "user-1",
        lease_s: float = 30.0,
        now_mono: float = 0.0,
        now_s: float = NOW,
        lease_at: float | None = None,
    ):
        gate = gate or ControlTracker("hardware-bot")
        reasons = gate.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": controller,
                "epoch": epoch,
                "lease_expires_at": now_s + lease_s if lease_at is None else lease_at,
            },
            {"bot_id": "hardware-bot", "epoch": epoch, "token": token},
            now_mono,
        )
        return gate, reasons

    def _envelope(self, **over):
        body = {
            "epoch": 3,
            "token": "tok-1",
            "seq": 1,
            "ts": int(NOW * 1000),
            "cmd": _joy(),
        }
        body.update(over)
        return body

    def test_timestamptz_parse(self):
        parsed = parse_timestamptz("2026-10-16T12:00:00+00:00")
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed, parse_timestamptz("2026-10-16T12:00:00Z"))
        self.assertEqual(parse_timestamptz(NOW), NOW)
        self.assertEqual(parse_timestamptz(int(NOW * 1000)), NOW)
        self.assertIsNone(parse_timestamptz(None))
        self.assertIsNone(parse_timestamptz("not-a-time"))

    def test_valid_cmd_accepted_and_rank_ignored(self):
        gate, reasons = self._arm()
        self.assertEqual(reasons, [])
        env = self._envelope()
        env["from"] = {"rank": 0, "name": "nobody"}
        env["rank"] = 99
        env["cmd"] = _joy()
        env["cmd"]["from"] = {"rank": 0}
        env["cmd"]["rank"] = 3
        result = gate.verify(env, source="cmd", now_s=NOW, now_mono=0.0)
        self.assertTrue(result.ok, result.reason)
        self.assertEqual(result.cmd_type, "joy")
        self.assertNotIn("rank", result.cmd or {})
        self.assertNotIn("from", result.cmd or {})
        self.assertNotIn(env["token"], result.reason)

    def test_rank_does_not_bypass_token(self):
        gate, _ = self._arm()
        env = self._envelope(token="nope")
        env["from"] = {"rank": 3}
        result = gate.verify(env, source="cmd", now_s=NOW, now_mono=0.0)
        self.assertFalse(result.ok)
        self.assertEqual(result.reason, "token")

    def test_token_match_is_case_insensitive(self):
        gate, _ = self._arm(token="AbC-Def")
        result = gate.verify(
            self._envelope(token="abc-def"),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertTrue(result.ok, result.reason)

    def test_rejects_bad_epoch_lease_seq_and_ts(self):
        gate, _ = self._arm()
        stale_epoch = gate.verify(
            self._envelope(epoch=2), source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertEqual(stale_epoch.reason, "epoch")

        expired, _ = self._arm(lease_s=-1)
        self.assertEqual(
            expired.verify(self._envelope(), source="cmd", now_s=NOW, now_mono=1.0).reason,
            "lease",
        )

        gate2, _ = self._arm(now_mono=1.0)
        self.assertTrue(gate2.verify(
            self._envelope(seq=1), source="cmd", now_s=NOW, now_mono=1.0,
        ).ok)
        replay = gate2.verify(
            self._envelope(seq=1), source="cmd", now_s=NOW, now_mono=1.0,
        )
        self.assertEqual(replay.reason, "seq")
        self.assertEqual(
            gate2.verify(self._envelope(seq=0), source="cmd", now_s=NOW, now_mono=1.0).reason,
            "seq",
        )
        # Gaps are still accepted.
        self.assertTrue(gate2.verify(
            self._envelope(seq=4), source="cmd", now_s=NOW, now_mono=1.0,
        ).ok)

    def test_skewed_clock_ts_is_accepted_and_non_finite_is_not(self):
        gate, _ = self._arm()
        # A day off, in milliseconds. The Pi may not have NTP yet.
        late = gate.verify(
            self._envelope(seq=1, ts=int(NOW * 1000) - 86_400_000),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertTrue(late.ok, late.reason)
        # Seconds, also far from the bridge clock.
        ahead = gate.verify(
            self._envelope(seq=2, ts=NOW + 10_000),
            source="cmd", now_s=NOW, now_mono=0.1,
        )
        self.assertTrue(ahead.ok, ahead.reason)
        bad = gate.verify(
            self._envelope(seq=3, ts=float("nan")),
            source="cmd", now_s=NOW, now_mono=0.2,
        )
        self.assertEqual(bad.reason, "ts")
        missing = self._envelope(seq=3)
        del missing["ts"]
        self.assertEqual(
            gate.verify(missing, source="cmd", now_s=NOW, now_mono=0.2).reason,
            "ts",
        )
        # The bad timestamp did not consume seq 3.
        self.assertTrue(gate.verify(
            self._envelope(seq=3, ts=0),
            source="cmd", now_s=NOW, now_mono=0.3,
        ).ok)

    def test_cmdq_trusts_server_sender_not_rank_or_inner_sender(self):
        gate, _ = self._arm()
        env = {
            "sender": "user-1",
            "epoch": 3,
            "seq": 1,
            "ts": int(NOW * 1000),
            "from": {"rank": 0},
            "cmd": {"type": "estop", "sender": "someone-else", "rank": 3},
        }
        result = gate.verify(env, source="cmdq", now_s=NOW, now_mono=0.0)
        self.assertTrue(result.ok, result.reason)

        missing = dict(env)
        del missing["sender"]
        missing["seq"] = 2
        self.assertEqual(
            gate.verify(missing, source="cmdq", now_s=NOW, now_mono=0.0).reason,
            "sender",
        )
        wrong = dict(env)
        wrong["sender"] = "user-2"
        wrong["seq"] = 2
        self.assertEqual(
            gate.verify(wrong, source="cmdq", now_s=NOW, now_mono=0.0).reason,
            "sender",
        )
        bad_token = dict(env)
        bad_token["seq"] = 2
        bad_token["token"] = "nope"
        self.assertEqual(
            gate.verify(bad_token, source="cmdq", now_s=NOW, now_mono=0.0).reason,
            "token",
        )

    def test_cmd_and_cmdq_share_one_seq_per_epoch(self):
        gate, _ = self._arm()
        self.assertTrue(gate.verify(
            self._envelope(seq=1), source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        replay = {
            "sender": "user-1",
            "epoch": 3,
            "seq": 1,
            "ts": int(NOW * 1000),
            "cmd": {"type": "nav_goal", "x": 1, "y": 2},
        }
        self.assertEqual(
            gate.verify(replay, source="cmdq", now_s=NOW, now_mono=0.0).reason,
            "seq",
        )
        replay["seq"] = 2
        self.assertTrue(gate.verify(replay, source="cmdq", now_s=NOW, now_mono=0.0).ok)

    def test_ctl_only_triggers_poll_and_poll_moves_epoch(self):
        gate, _ = self._arm()
        self.assertTrue(gate.verify(
            self._envelope(seq=9), source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        self.assertTrue(gate.ctl_needs_poll({
            "bot_id": "hardware-bot",
            "controller": "user-2",
            "controller_name": "Two",
            "epoch": 4,
        }))
        self.assertFalse(gate.ctl_needs_poll({
            "bot_id": "amr4",
            "controller": "user-2",
            "epoch": 9,
        }))
        # :ctl is not authority. Epoch, token, and lease stay until a poll.
        self.assertEqual(gate.epoch, 3)
        self.assertEqual(gate.token, "tok-1")
        self.assertIsNotNone(gate.lease_expires_at)
        self.assertTrue(gate.verify(
            self._envelope(seq=10), source="cmd", now_s=NOW, now_mono=1.0,
        ).ok)

        reasons = gate.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": "user-2",
                "epoch": 4,
                "lease_expires_at": NOW + 15,
            },
            {"bot_id": "hardware-bot", "epoch": 4, "token": "tok-2"},
            1.2,
        )
        self.assertEqual(reasons, ["epoch_change"])
        self.assertEqual(gate.token, "tok-2")
        self.assertEqual(gate.controller, "user-2")
        # Stale poll must not roll the epoch or the token backwards.
        self.assertEqual(gate.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": "user-1",
                "epoch": 3,
                "lease_expires_at": NOW + 30,
            },
            {"bot_id": "hardware-bot", "epoch": 3, "token": "tok-1"},
            1.3,
        ), [])
        self.assertEqual(gate.epoch, 4)
        self.assertEqual(gate.token, "tok-2")
        self.assertTrue(gate.verify(
            self._envelope(epoch=4, token="tok-2", seq=1),
            source="cmd", now_s=NOW, now_mono=1.4,
        ).ok)

    def test_poll_epoch_change_installs_new_token(self):
        gate, _ = self._arm()
        reasons = gate.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": "user-2",
                "epoch": 4,
                "lease_expires_at": "2026-10-16T12:00:02Z",
            },
            {"epoch": 4, "token": "tok-2"},
            2.0,
        )
        self.assertEqual(reasons, ["epoch_change"])
        self.assertEqual(gate.token, "tok-2")
        self.assertEqual(gate.controller, "user-2")
        self.assertIsNotNone(gate.lease_expires_at)

    def test_wrapped_broadcast_and_wrong_bot(self):
        gate, _ = self._arm()
        wrapped = {"event": "command", "payload": self._envelope()}
        self.assertTrue(gate.verify(wrapped, source="cmd", now_s=NOW, now_mono=0.0).ok)
        other = self._envelope(seq=2, botId="amr4")
        self.assertEqual(
            gate.verify(other, source="cmd", now_s=NOW, now_mono=0.0).reason,
            "bot",
        )

    def test_unknown_type_and_missing_cmd(self):
        gate, _ = self._arm()
        self.assertEqual(
            gate.verify({"epoch": 3}, source="cmd", now_s=NOW, now_mono=0.0).reason,
            "malformed",
        )
        bad = self._envelope()
        bad["cmd"] = {"type": "teleport"}
        self.assertEqual(
            gate.verify(bad, source="cmd", now_s=NOW, now_mono=0.0).reason,
            "type",
        )

    def test_watchdog_motion_lease_and_supabase(self):
        self.assertEqual(MOTION_WATCHDOG_S, 0.5)
        self.assertEqual(SUPABASE_UNREACHABLE_S, 5.0)
        gate, _ = self._arm(now_mono=10.0)
        self.assertTrue(gate.verify(
            self._envelope(), source="cmd", now_s=NOW, now_mono=10.0,
        ).ok)
        gate.note_contact(10.0)
        self.assertEqual(gate.tick(NOW, 10.49), [])
        self.assertEqual(gate.tick(NOW, 10.5), ["command_timeout"])
        self.assertEqual(gate.tick(NOW, 11.0), [])

        # A later motion command can time out again. A zero stick clears the latch.
        self.assertTrue(gate.verify(
            self._envelope(seq=2), source="cmd", now_s=NOW, now_mono=12.0,
        ).ok)
        gate.note_contact(12.0)
        zero = self._envelope(seq=3, cmd=_joy(axes=[0.0] * 8))
        self.assertTrue(gate.verify(zero, source="cmd", now_s=NOW, now_mono=12.2).ok)
        gate.note_contact(12.2)
        self.assertEqual(gate.tick(NOW, 12.9), [])

        # Discrete commands do not refresh or latch the deadman.
        nav = self._envelope(seq=4, cmd={"type": "nav_goal", "x": 1, "y": 1})
        self.assertTrue(gate.verify(nav, source="cmd", now_s=NOW, now_mono=13.0).ok)
        gate.note_contact(20.0)
        self.assertEqual(gate.tick(NOW, 20.0), [])

        # Lease expiry, silence, and a fresh motion command all zero once.
        moving, _ = self._arm(lease_s=1.0, now_mono=0.0)
        self.assertTrue(moving.verify(
            self._envelope(), source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        # Observe the lease while it is still open, then let it expire.
        self.assertEqual(moving.tick(NOW, 0.0), [])
        reasons = moving.tick(NOW + 1.0, 5.0)
        self.assertEqual(reasons, ["lease_expired", "supabase_unreachable", "command_timeout"])
        self.assertEqual(moving.tick(NOW + 2.0, 6.0), [])
        moving.note_contact(6.0)
        # Renew the lease, then lose contact again.
        moving.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": "user-1",
                "epoch": 3,
                "lease_expires_at": NOW + 100,
            },
            {"epoch": 3, "token": "tok-1"},
            6.0,
        )
        self.assertEqual(moving.tick(NOW + 50, 11.0), ["supabase_unreachable"])

    def test_watchdog_stays_quiet_until_private_contact(self):
        gate = ControlTracker("hardware-bot")
        self.assertEqual(gate.tick(NOW + 100, 100.0), [])

    def test_iso_lease_opens_and_exact_expiry_closes(self):
        gate = ControlTracker("hardware-bot")
        gate.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": "user-1",
                "epoch": 1,
                "lease_expires_at": "2026-10-16T12:00:02+00:00",
            },
            {"epoch": 1, "token": "tok-1"},
            0.0,
        )
        opened = gate.lease_expires_at
        self.assertIsNotNone(opened)
        assert opened is not None
        self.assertTrue(gate.verify(
            {
                "epoch": 1,
                "token": "tok-1",
                "seq": 1,
                "ts": int(opened * 1000) - 500,
                "cmd": {"type": "estop"},
            },
            source="cmd",
            now_s=opened - 0.5,
            now_mono=0.0,
        ).ok)
        # E-stop ignores the lease, including the exact expiry instant.
        closed = gate.verify(
            {
                "epoch": 1,
                "token": "tok-1",
                "seq": 2,
                "ts": int(opened * 1000),
                "cmd": {"type": "estop"},
            },
            source="cmd",
            now_s=opened,
            now_mono=0.1,
        )
        self.assertTrue(closed.ok, closed.reason)
        held = gate.verify(
            {
                "epoch": 1,
                "token": "tok-1",
                "seq": 1,
                "ts": int(opened * 1000),
                "cmd": {"type": "nudge", "lin": 0.1, "ang": 0},
            },
            source="cmd",
            now_s=opened,
            now_mono=0.1,
        )
        self.assertEqual(held.reason, "lease")

    def test_website_joy_ab_arms_deadman_and_axes_still_work(self):
        self.assertEqual(SEQ_RESET_GAP_S, 0.5)
        gate, _ = self._arm(now_mono=10.0)
        website = {"type": "joy", "a": [0.0, 0.8, 0, 0, 0, 0, 0, 0], "b": 0}
        result = gate.verify(
            self._envelope(seq=1, cmd=website),
            source="cmd", now_s=NOW, now_mono=10.0,
        )
        self.assertTrue(result.ok, result.reason)
        self.assertEqual(result.cmd["a"][1], 0.8)
        self.assertEqual(result.cmd["b"], 0)
        self.assertTrue(motion_command(result.cmd or {}))
        self.assertEqual(gate.tick(NOW, 10.49), [])
        self.assertEqual(gate.tick(NOW, 10.5), ["command_timeout"])

        # All-zero website stick does not hold the deadman open.
        zero = gate.verify(
            self._envelope(seq=2, cmd={"type": "joy", "a": [0.0] * 8, "b": 0}),
            source="cmd", now_s=NOW, now_mono=12.0,
        )
        self.assertTrue(zero.ok, zero.reason)
        self.assertFalse(motion_command(zero.cmd or {}))
        self.assertEqual(gate.tick(NOW, 12.6), [])

        axes = gate.verify(
            self._envelope(seq=3, cmd=_joy()),
            source="cmd", now_s=NOW, now_mono=13.0,
        )
        self.assertTrue(axes.ok, axes.reason)
        self.assertEqual(axes.cmd["axes"][1], 0.5)
        self.assertEqual(gate.tick(NOW, 13.5), ["command_timeout"])

    def test_estop_is_idempotent_and_skips_seq_and_ts(self):
        gate, _ = self._arm()
        self.assertTrue(gate.verify(
            self._envelope(seq=5), source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        first = gate.verify(
            self._envelope(seq=1, ts=float("inf"), cmd={"type": "estop"}),
            source="cmd", now_s=NOW, now_mono=0.1,
        )
        self.assertTrue(first.ok, first.reason)
        again = gate.verify(
            self._envelope(seq=1, cmd={"type": "estop"}),
            source="cmd", now_s=NOW, now_mono=0.2,
        )
        self.assertTrue(again.ok, again.reason)
        bare = {"epoch": 3, "token": "tok-1", "cmd": {"type": "estop"}}
        self.assertTrue(gate.verify(
            bare, source="cmd", now_s=NOW, now_mono=0.3,
        ).ok)
        # E-stop did not move the seq cursor.
        self.assertEqual(
            gate.verify(
                self._envelope(seq=5), source="cmd", now_s=NOW, now_mono=0.4,
            ).reason,
            "seq",
        )
        self.assertTrue(gate.verify(
            self._envelope(seq=6), source="cmd", now_s=NOW, now_mono=0.4,
        ).ok)
        self.assertEqual(
            gate.verify(
                self._envelope(epoch=2, seq=1, cmd={"type": "estop"}),
                source="cmd", now_s=NOW, now_mono=0.5,
            ).reason,
            "epoch",
        )
        self.assertEqual(
            gate.verify(
                self._envelope(token="nope", seq=1, cmd={"type": "estop"}),
                source="cmd", now_s=NOW, now_mono=0.5,
            ).reason,
            "token",
        )
        held = {
            "sender": "other",
            "epoch": 3,
            "cmd": {"type": "estop"},
        }
        self.assertEqual(
            gate.verify(held, source="cmdq", now_s=NOW, now_mono=0.6).reason,
            "sender",
        )
        # Expired or unknown lease does not block e-stop. A drive command still stops.
        expired, _ = self._arm(lease_s=-5)
        self.assertTrue(expired.verify(
            self._envelope(cmd={"type": "estop"}),
            source="cmd", now_s=NOW, now_mono=1.0,
        ).ok)
        self.assertEqual(
            expired.verify(self._envelope(seq=1), source="cmd", now_s=NOW, now_mono=1.0).reason,
            "lease",
        )
        expired.lease_expires_at = None
        self.assertTrue(expired.verify(
            self._envelope(cmd={"type": "estop"}),
            source="cmd", now_s=NOW, now_mono=1.1,
        ).ok)
        fresh = ControlTracker("hardware-bot")
        self.assertEqual(
            fresh.verify(
                self._envelope(cmd={"type": "estop"}),
                source="cmd", now_s=NOW, now_mono=0.0,
            ).reason,
            "epoch",
        )

    def test_lower_seq_after_gap_and_time_based_seq(self):
        gate, _ = self._arm()
        self.assertTrue(gate.verify(
            self._envelope(seq=10), source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        early = gate.verify(
            self._envelope(seq=1), source="cmd", now_s=NOW, now_mono=0.1,
        )
        self.assertEqual(early.reason, "seq")
        # A time-based seq (Date.now()) is still higher, so a reload that
        # uses it is accepted immediately.
        stamped = int(NOW * 1000)
        self.assertTrue(gate.verify(
            self._envelope(seq=stamped), source="cmd", now_s=NOW, now_mono=0.2,
        ).ok)
        # And a browser that reset to 1 is accepted once the gap has passed,
        # not after a 30 s lockout.
        reset = gate.verify(
            self._envelope(seq=1),
            source="cmd", now_s=NOW, now_mono=0.2 + SEQ_RESET_GAP_S,
        )
        self.assertTrue(reset.ok, reset.reason)
        self.assertTrue(gate.verify(
            self._envelope(seq=2),
            source="cmd", now_s=NOW, now_mono=0.2 + SEQ_RESET_GAP_S,
        ).ok)

    def test_inputs_are_finite_clamped_and_allowlisted(self):
        gate, _ = self._arm()
        huge = gate.verify(
            self._envelope(seq=1, cmd={"type": "nudge", "lin": 9, "ang": -9}),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertTrue(huge.ok, huge.reason)
        self.assertEqual(huge.cmd["lin"], 0.3)
        self.assertEqual(huge.cmd["ang"], -1.0)

        bad = gate.verify(
            self._envelope(seq=2, cmd={"type": "nudge", "lin": float("nan"), "ang": 0}),
            source="cmd", now_s=NOW, now_mono=0.1,
        )
        self.assertEqual(bad.reason, "value")
        # Rejected values do not consume seq.
        self.assertTrue(gate.verify(
            self._envelope(seq=2, cmd={"type": "nudge", "lin": 0.1, "ang": 0}),
            source="cmd", now_s=NOW, now_mono=0.1,
        ).ok)

        joy = sanitize_command({
            "type": "joy",
            "a": [2, -2, 0, 0, 0, 0, 0, 0],
            "b": 0,
        })
        self.assertEqual(joy[0]["a"][0], 1.0)
        self.assertEqual(joy[0]["a"][1], -1.0)
        self.assertIsNone(sanitize_command({
            "type": "joy",
            "a": [0, float("inf"), 0, 0, 0, 0, 0, 0],
            "b": 1,
        })[0])
        masked = sanitize_command({"type": "joy", "a": [0.0] * 8, "b": BUTTON_MASK + 2})
        self.assertEqual(masked[0]["b"], 1)
        self.assertEqual(masked[0]["buttons"], 1)
        goal = sanitize_command({"type": "nav_goal", "x": 5000, "y": -5000})
        self.assertEqual(goal[0]["x"], POSE_LIMIT_M)
        self.assertEqual(goal[0]["y"], -POSE_LIMIT_M)

        tool = sanitize_command({
            "type": "run_tool",
            "tool": "imu_calibration",
            "params": {"speed": 9, "rotations": 0.1},
        })
        self.assertEqual(tool[0]["params"]["speed"], 1.0)
        self.assertEqual(tool[0]["params"]["rotations"], 0.5)
        self.assertIsNone(sanitize_command({
            "type": "run_tool",
            "tool": "shell",
            "params": {},
        })[0])
        self.assertIsNone(sanitize_command({
            "type": "run_tool",
            "tool": "imu_test",
            "params": {"speed": 0.2},
        })[0])
        self.assertEqual(
            sanitize_command({"type": "save_map", "name": "map_1"})[0]["name"],
            "map_1",
        )
        self.assertEqual(sanitize_command({"type": "save_map", "name": ""})[0]["name"], "")
        self.assertIsNone(sanitize_command({"type": "save_map", "name": "../etc"})[0])
        zone = sanitize_command({
            "type": "blacklist",
            "x": 1,
            "y": 2,
            "radius": 9,
            "duration": -5,
        })
        self.assertEqual(zone[0]["radius"], 5.0)
        self.assertEqual(zone[0]["duration"], 0.0)
        far = sanitize_command({
            "type": "blacklist",
            "x": 9_000,
            "y": -9_000,
            "duration": 1e12,
        })
        self.assertEqual(far[0]["x"], POSE_LIMIT_M)
        self.assertEqual(far[0]["y"], -POSE_LIMIT_M)
        self.assertEqual(far[0]["duration"], BLACKLIST_DURATION_MAX_S)
        self.assertEqual(
            sanitize_command({"type": "blacklist", "action": "clear"})[0]["action"],
            "clear",
        )

    def test_service_key_detector_and_token_compare(self):
        self.assertTrue(looks_like_service_key("sb_secret_example"))
        self.assertFalse(looks_like_service_key("sb_publishable_example"))
        self.assertFalse(looks_like_service_key(""))
        self.assertTrue(looks_like_service_key(_jwt("service_role")))
        self.assertFalse(looks_like_service_key(_jwt("anon")))
        gate, _ = self._arm(token="AbC")
        self.assertTrue(gate.verify(
            self._envelope(token="abc"),
            source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        self.assertEqual(
            gate.verify(
                self._envelope(token="abcd", seq=2),
                source="cmd", now_s=NOW, now_mono=0.1,
            ).reason,
            "token",
        )

    def test_rejected_command_does_not_refresh_contact(self):
        gate, _ = self._arm(now_mono=0.0)
        rejected = gate.verify(
            self._envelope(token="nope"),
            source="cmd", now_s=NOW, now_mono=4.9,
        )
        self.assertFalse(rejected.ok)
        self.assertEqual(gate.tick(NOW, 5.0), ["supabase_unreachable"])
        fresh, _ = self._arm(now_mono=0.0)
        self.assertTrue(fresh.verify(
            self._envelope(), source="cmd", now_s=NOW, now_mono=4.9,
        ).ok)
        self.assertEqual(fresh.tick(NOW, 5.0), [])


def _jwt(role: str) -> str:
    import base64
    import json
    header = base64.urlsafe_b64encode(b'{"alg":"none"}').decode().rstrip("=")
    payload = base64.urlsafe_b64encode(
        json.dumps({"role": role}).encode()
    ).decode().rstrip("=")
    return f"{header}.{payload}.sig"


class MotionGuardTests(unittest.TestCase):
    def test_soft_zero_is_cancelled_by_new_motion_and_hard_is_not(self):
        guard = MotionGuard()
        gen = guard.arm(hard=False)
        self.assertTrue(guard.still_owed(gen))
        guard.release_for_motion()
        self.assertFalse(guard.still_owed(gen))
        self.assertFalse(guard.owed)

        gen = guard.arm(hard=True)
        guard.release_for_motion()
        self.assertTrue(guard.hard)
        self.assertTrue(guard.still_owed(gen))
        guard.mark_zeroed(gen)
        self.assertFalse(guard.owed)


class BridgeSourceSafetyTests(unittest.TestCase):
    def test_velocity_stays_on_existing_http_path(self):
        src = (BRIDGE_ROOT / "amr_telemetry_bridge" / "bridge.py").read_text()
        self.assertIn('"/api/nudge"', src)
        self.assertIn("/api/joy", src)
        self.assertIn("zero motion", src)
        self.assertNotIn("create_publisher", src)
        self.assertNotIn('publish("/cmd_vel_safe"', src)
        self.assertNotIn("cmd_vel_topic", src)
        self.assertIn("COMMAND_HTTP_TIMEOUT_S", src)
        self.assertIn("MAX_SCHEDULED_COMMANDS", src)
        self.assertIn('"/api/estop"', src)
        self.assertIn("send_status", src)
        self.assertIn("ACK_QUEUE_MAX", src)
        self.assertIn("CMD_ACK_EVENT", src)
        protocol = (BRIDGE_ROOT / "amr_telemetry_bridge" / "protocol.py").read_text()
        self.assertIn('CMD_ACK_EVENT = "cmd_ack"', protocol)
        readme = (BRIDGE_ROOT / "README.md").read_text()
        self.assertIn("cmd_ack", readme)
        for reason in (
            "bad_token",
            "stale_epoch",
            "lower_seq",
            "no_lease",
            "invalid_payload",
            "unknown_type",
            "rate_limited",
        ):
            self.assertIn(f"`{reason}`", readme)
        control = (BRIDGE_ROOT / "amr_telemetry_bridge" / "control.py").read_text()
        self.assertIn("hmac.compare_digest", control)
        self.assertIn("sb_secret_", control)
        self.assertIn("service_role", control)

    def test_private_auth_and_touch_logging(self):
        src = (BRIDGE_ROOT / "amr_telemetry_bridge" / "bridge.py").read_text()
        self.assertIn("sign_in_with_password", src)
        self.assertIn("refresh_session", src)
        self.assertIn("set_auth", src)
        self.assertIn("AMR_BRIDGE_EMAIL", src)
        self.assertIn("AMR_BRIDGE_PASSWORD", src)
        self.assertIn('default=_env("AMR_PUBLIC_CHANNEL", "0")', src)
        self.assertIn("looks_like_service_key", src)
        self.assertIn('{"config": {"private": True}}', src)
        self.assertIn("touch_amr_bot returned false", src)
        self.assertNotIn('or _env("AMR_SUPABASE_SERVICE_KEY")', src)
        for event in ("telemetry", "map", "log", "hb"):
            self.assertIn(event, src)

    def test_setup_script_names_only(self):
        script = (ROOT / "scripts" / "setup_amr_bridge.sh").read_text()
        self.assertIn("AMR_BRIDGE_EMAIL=\n", script)
        self.assertIn("AMR_BRIDGE_PASSWORD=\n", script)
        self.assertIn("AMR_PUBLIC_CHANNEL=0\n", script)
        self.assertIn("Do NOT ever add AMR_SUPABASE_SERVICE_KEY", script)
        readme = (ROOT / "scripts" / "README.md").read_text()
        self.assertIn("AMR_BRIDGE_EMAIL", readme)
        self.assertIn("AMR_BRIDGE_PASSWORD", readme)


class _FakeResponse:
    def raise_for_status(self):
        return None


class _FakeHttp:
    def __init__(self, posts: list):
        self.posts = posts

    async def post(self, url, json=None, timeout=None):
        self.posts.append(("http", url, json, timeout))
        return _FakeResponse()


class _FakeDash:
    def __init__(self):
        self.posts: list[tuple] = []
        self.fail_zeros = 0
        self.http = _FakeHttp(self.posts)

    async def post(self, path, payload=None, timeout=None):
        self.posts.append(("dash", path, payload, timeout))
        if (
            path == "/api/nudge"
            and payload == {"lin": 0.0, "ang": 0.0}
            and self.fail_zeros
        ):
            self.fail_zeros -= 1
            raise RuntimeError("dashboard down")
        return {"ok": True}


def _dispatch_bridge():
    from amr_telemetry_bridge.bridge import Bridge

    return Bridge(
        bot_id="hardware-bot",
        supabase_url="https://example.supabase.co",
        api_key="sb_publishable_example",
        bridge_token="tok",
        dashboard_url="http://127.0.0.1:8080",
        camera_url=None,
        camera_jpeg_max_kb=24,
        public_channel=False,
        private_channel=False,
    )


def _stick(axis1: float) -> dict:
    return {"type": "joy", "a": [0.0, axis1, 0, 0, 0, 0, 0, 0], "b": 0}


def _cmd_env(seq: int, cmd: dict) -> dict:
    return {
        "epoch": 3,
        "token": "tok-1",
        "seq": seq,
        "ts": int(time.time() * 1000),
        "cmd": cmd,
    }


def _arm_gate(bridge) -> None:
    bridge._gate.apply_poll(
        {
            "bot_id": "hardware-bot",
            "controller": "user-1",
            "epoch": 3,
            "lease_expires_at": time.time() + 3600,
        },
        {"epoch": 3, "token": "tok-1"},
        time.monotonic(),
    )


def _recording_dash(posts: list):
    async def dash_post(path, payload=None, timeout=None):
        posts.append(("dash", path, payload, timeout))
        return {"ok": True}

    return dash_post


class _RecordingHttp:
    def __init__(self, post):
        self._post = post

    async def post(self, url, json=None, timeout=None):
        return await self._post(url, json=json, timeout=timeout)


async def _wait_until(predicate, timeout: float = 1.0) -> None:
    import asyncio

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        await asyncio.sleep(0.01)
    raise AssertionError("timed out")


async def _stop_loops(bridge, motion_task, original_http, old_gamepad, estop_task=None) -> None:
    import asyncio

    bridge._stop.set()
    bridge._motion_wake.set()
    bridge._estop_wake.set()
    tasks = [motion_task]
    if estop_task is not None:
        tasks.append(estop_task)
    await asyncio.wait(tasks, timeout=1.0)
    for task in tasks:
        task.cancel()
    if old_gamepad is None:
        os.environ.pop("AMR_GAMEPAD_URL", None)
    else:
        os.environ["AMR_GAMEPAD_URL"] = old_gamepad
    bridge.dash._http = original_http
    await bridge.dash.close()


def _close_bridge(bridge) -> None:
    import asyncio

    async def _close():
        await bridge.dash.close()

    asyncio.run(_close())


class BridgeDispatchTests(unittest.TestCase):
    def test_website_joy_arms_watchdog_and_zero_retries(self):
        import asyncio

        from amr_telemetry_bridge.bridge import COMMAND_HTTP_TIMEOUT_S, Bridge

        async def run():
            import time
            bridge = Bridge(
                bot_id="hardware-bot",
                supabase_url="https://example.supabase.co",
                api_key="sb_publishable_example",
                bridge_token="tok",
                dashboard_url="http://127.0.0.1:8080",
                camera_url=None,
                camera_jpeg_max_kb=24,
                public_channel=False,
                private_channel=False,
            )
            fake = _FakeDash()
            original_http = bridge.dash._http
            bridge.dash._http = fake.http
            bridge.dash.post = fake.post
            old_gamepad = os.environ.get("AMR_GAMEPAD_URL")
            os.environ["AMR_GAMEPAD_URL"] = "http://127.0.0.1:8765"
            try:
                bridge._gate.apply_poll(
                    {
                        "bot_id": "hardware-bot",
                        "controller": "user-1",
                        "epoch": 3,
                        "lease_expires_at": NOW + 30,
                    },
                    {"epoch": 3, "token": "tok-1"},
                    10.0,
                )
                website = {"type": "joy", "a": [0.0, 0.8, 0, 0, 0, 0, 0, 0], "b": 0}
                result = bridge._gate.verify(
                    {
                        "epoch": 3,
                        "token": "tok-1",
                        "seq": 1,
                        "ts": int(NOW * 1000),
                        "cmd": website,
                    },
                    source="cmd",
                    now_s=NOW,
                    now_mono=10.0,
                )
                self.assertTrue(result.ok, result.reason)
                self.assertEqual(bridge._gate.tick(NOW, 10.49), [])
                reasons = bridge._gate.tick(NOW, 10.5)
                self.assertEqual(reasons, ["command_timeout"])

                fake.fail_zeros = 1
                await bridge._apply_watchdog(reasons)
                self.assertTrue(bridge._guard.owed)
                self.assertEqual(
                    [item for item in fake.posts if item[1] == "/api/nudge"],
                    [("dash", "/api/nudge", {"lin": 0.0, "ang": 0.0}, COMMAND_HTTP_TIMEOUT_S)],
                )
                await bridge._try_zero(bridge._guard.owed_gen, "retry")
                self.assertFalse(bridge._guard.owed)
                zeros = [item for item in fake.posts if item[1] == "/api/nudge"]
                self.assertEqual(len(zeros), 2)

                # Lease and contact must look live on the real clock, or the
                # motion is treated as a hard fault and zeroed again.
                bridge._gate.lease_expires_at = time.time() + 3600
                bridge._gate._lease_was_valid = True
                bridge._gate.note_contact(time.monotonic())
                bridge._guard.set_hard(False)
                await bridge._run_accepted(result.cmd)
                driving = [
                    item for item in fake.posts
                    if item[0] == "http" and item[2]["a"][1] == 0.8
                ]
                self.assertEqual(len(driving), 1)
                self.assertEqual(driving[0][1], "http://127.0.0.1:8765/api/joy")
                self.assertEqual(driving[0][2]["b"], 0)
                self.assertEqual(driving[0][3], COMMAND_HTTP_TIMEOUT_S)
                # The new motion cancels the deadman zero.
                self.assertEqual(
                    len([item for item in fake.posts if item[1] == "/api/nudge"]),
                    2,
                )

                bridge._guard.set_hard(True)
                bridge._guard.arm(hard=True)
                await bridge._dispatch_cmd(result.cmd)
                driving = [
                    item for item in fake.posts
                    if item[0] == "http" and item[2]["a"][1] == 0.8
                ]
                self.assertEqual(len(driving), 2)
                idx = max(
                    i for i, item in enumerate(fake.posts)
                    if item[0] == "http" and item[2]["a"][1] == 0.8
                )
                follow = fake.posts[idx + 1]
                self.assertEqual(follow[1], "/api/nudge")
                self.assertEqual(follow[2], {"lin": 0.0, "ang": 0.0})
                self.assertFalse(bridge._guard.owed)
            finally:
                if old_gamepad is None:
                    os.environ.pop("AMR_GAMEPAD_URL", None)
                else:
                    os.environ["AMR_GAMEPAD_URL"] = old_gamepad
                bridge.dash._http = original_http
                await bridge.dash.close()

        asyncio.run(run())

    def test_schedule_cap_closes_extra_coroutines(self):
        import asyncio

        from amr_telemetry_bridge.bridge import MAX_SCHEDULED_COMMANDS, Bridge

        async def run():
            bridge = Bridge(
                bot_id="hardware-bot",
                supabase_url="https://example.supabase.co",
                api_key="sb_publishable_example",
                bridge_token="tok",
                dashboard_url="http://127.0.0.1:8080",
                camera_url=None,
                camera_jpeg_max_kb=24,
                public_channel=False,
                private_channel=False,
            )
            bridge._loop = asyncio.get_running_loop()
            release = asyncio.Event()
            started = 0

            async def hang():
                nonlocal started
                started += 1
                await release.wait()

            try:
                for _ in range(MAX_SCHEDULED_COMMANDS):
                    bridge._schedule(hang())
                await asyncio.sleep(0.05)
                self.assertEqual(started, MAX_SCHEDULED_COMMANDS)

                async def extra():
                    nonlocal started
                    started += 10

                bridge._schedule(extra())
                await asyncio.sleep(0.05)
                self.assertEqual(started, MAX_SCHEDULED_COMMANDS)
                self.assertEqual(bridge._scheduled, MAX_SCHEDULED_COMMANDS)
                release.set()
                await asyncio.sleep(0.05)
                self.assertEqual(bridge._scheduled, 0)
            finally:
                release.set()
                await bridge.dash.close()

        asyncio.run(run())

    def test_stalled_joy_does_not_starve_estop(self):
        import asyncio

        from amr_telemetry_bridge.bridge import MAX_SCHEDULED_COMMANDS, Bridge

        async def run():
            bridge = Bridge(
                bot_id="hardware-bot",
                supabase_url="https://example.supabase.co",
                api_key="sb_publishable_example",
                bridge_token="tok",
                dashboard_url="http://127.0.0.1:8080",
                camera_url=None,
                camera_jpeg_max_kb=24,
                public_channel=False,
                private_channel=False,
            )
            bridge._loop = asyncio.get_running_loop()
            posts: list[tuple] = []
            joy_entered = asyncio.Event()
            release_joy = asyncio.Event()
            estop_seen = asyncio.Event()

            class _Resp:
                def raise_for_status(self):
                    return None

            async def dash_post(path, payload=None, timeout=None):
                posts.append(("dash", path, payload, timeout))
                if path == "/api/estop":
                    estop_seen.set()
                return {"ok": True}

            async def http_post(url, json=None, timeout=None):
                axes = (json or {}).get("a") or [0.0]
                moving = len(axes) > 1 and abs(float(axes[1])) > 0.02
                if moving:
                    joy_entered.set()
                    await release_joy.wait()
                posts.append(("http", url, json, timeout))
                return _Resp()

            original_http = bridge.dash._http
            bridge.dash.post = dash_post

            class _Http:
                async def post(self, url, json=None, timeout=None):
                    return await http_post(url, json=json, timeout=timeout)

            bridge.dash._http = _Http()
            old_gamepad = os.environ.get("AMR_GAMEPAD_URL")
            os.environ["AMR_GAMEPAD_URL"] = "http://127.0.0.1:8765"
            cap_release = asyncio.Event()

            async def hang():
                await cap_release.wait()

            motion_task = asyncio.create_task(bridge._motion_slot_loop())
            estop_task = asyncio.create_task(bridge._estop_loop())
            try:
                bridge._gate.apply_poll(
                    {
                        "bot_id": "hardware-bot",
                        "controller": "user-1",
                        "epoch": 3,
                        "lease_expires_at": time.time() + 3600,
                    },
                    {"epoch": 3, "token": "tok-1"},
                    time.monotonic(),
                )
                for _ in range(MAX_SCHEDULED_COMMANDS):
                    bridge._schedule(hang())
                await asyncio.sleep(0.05)
                self.assertEqual(bridge._scheduled, MAX_SCHEDULED_COMMANDS)

                bridge._queue_private(
                    {
                        "epoch": 3,
                        "token": "tok-1",
                        "seq": 1,
                        "ts": int(time.time() * 1000),
                        "cmd": {"type": "joy", "a": [0.0, 0.8, 0, 0, 0, 0, 0, 0], "b": 0},
                    },
                    "cmd",
                )
                await asyncio.wait_for(joy_entered.wait(), timeout=1.0)
                # Lease is already dead. E-stop must still run, and must not
                # wait for the joy post or for a free scheduler slot.
                bridge._gate.lease_expires_at = time.time() - 10
                bridge._queue_private(
                    {
                        "epoch": 3,
                        "token": "tok-1",
                        "seq": 1,
                        "ts": float("nan"),
                        "cmd": {"type": "estop"},
                    },
                    "cmd",
                )
                async def _estop_finished() -> None:
                    while True:
                        kinds = [item[1] for item in posts]
                        zero_joy = any(
                            item[0] == "http"
                            and item[2]["a"] == [0.0] * 8
                            and item[2]["b"] == 0
                            for item in posts
                        )
                        if "/api/estop" in kinds and "/api/nudge" in kinds and zero_joy:
                            return
                        await asyncio.sleep(0.01)

                await asyncio.wait_for(_estop_finished(), timeout=1.0)
                self.assertFalse(release_joy.is_set())
                kinds = [item[1] for item in posts]
                self.assertFalse(any(
                    item[0] == "http" and abs(item[2]["a"][1]) > 0.02
                    for item in posts
                ))
            finally:
                release_joy.set()
                cap_release.set()
                bridge._stop.set()
                bridge._motion_wake.set()
                bridge._estop_wake.set()
                await asyncio.wait([motion_task, estop_task], timeout=1.0)
                motion_task.cancel()
                estop_task.cancel()
                if old_gamepad is None:
                    os.environ.pop("AMR_GAMEPAD_URL", None)
                else:
                    os.environ["AMR_GAMEPAD_URL"] = old_gamepad
                bridge.dash._http = original_http
                await bridge.dash.close()

        asyncio.run(run())

    def test_ctl_is_not_capped(self):
        import asyncio

        from amr_telemetry_bridge.bridge import MAX_SCHEDULED_COMMANDS, Bridge

        async def run():
            bridge = Bridge(
                bot_id="hardware-bot",
                supabase_url="https://example.supabase.co",
                api_key="sb_publishable_example",
                bridge_token="tok",
                dashboard_url="http://127.0.0.1:8080",
                camera_url=None,
                camera_jpeg_max_kb=24,
                public_channel=False,
                private_channel=False,
            )
            bridge._loop = asyncio.get_running_loop()

            class _Hub:
                def __init__(self):
                    self.calls = 0

                def poll_control(self):
                    self.calls += 1
                    return (
                        {
                            "bot_id": "hardware-bot",
                            "controller": "user-1",
                            "epoch": 3,
                            "lease_expires_at": time.time() + 30,
                        },
                        {"epoch": 3, "token": "tok-1"},
                    )

            hub = _Hub()
            bridge._hub = hub
            release = asyncio.Event()

            async def hang():
                await release.wait()

            try:
                for _ in range(MAX_SCHEDULED_COMMANDS):
                    bridge._schedule(hang())
                await asyncio.sleep(0.05)
                self.assertEqual(bridge._scheduled, MAX_SCHEDULED_COMMANDS)
                bridge._queue_control({
                    "bot_id": "hardware-bot",
                    "controller": "user-2",
                    "epoch": 4,
                })
                for _ in range(20):
                    if hub.calls:
                        break
                    await asyncio.sleep(0.05)
                self.assertGreaterEqual(hub.calls, 1)
                self.assertEqual(bridge._scheduled, MAX_SCHEDULED_COMMANDS)
            finally:
                release.set()
                await asyncio.sleep(0.05)
                await bridge.dash.close()

        asyncio.run(run())

    def test_slow_motion_post_is_zeroed_even_if_guard_is_clear(self):
        import asyncio

        from amr_telemetry_bridge.bridge import Bridge

        async def run():
            bridge = _dispatch_bridge()
            bridge._loop = asyncio.get_running_loop()
            posts: list[tuple] = []

            class _Resp:
                def raise_for_status(self):
                    return None

            async def http_post(url, json=None, timeout=None):
                axes = (json or {}).get("a") or [0.0]
                if len(axes) > 1 and abs(float(axes[1]) - 0.8) < 1e-6:
                    await asyncio.sleep(MOTION_WATCHDOG_S + 0.15)
                posts.append(("http", url, json, timeout))
                return _Resp()

            original_http = bridge.dash._http
            bridge.dash.post = _recording_dash(posts)
            bridge.dash._http = _RecordingHttp(http_post)
            old_gamepad = os.environ.get("AMR_GAMEPAD_URL")
            os.environ["AMR_GAMEPAD_URL"] = "http://127.0.0.1:8765"
            task = asyncio.create_task(bridge._motion_slot_loop())
            try:
                _arm_gate(bridge)
                self.assertFalse(bridge._guard.hard)
                self.assertFalse(bridge._guard.owed)
                bridge._queue_private(_cmd_env(1, _stick(0.4)), "cmd")
                await _wait_until(
                    lambda: any(
                        item[0] == "http" and abs(item[2]["a"][1] - 0.4) < 1e-6
                        for item in posts
                    ),
                )
                self.assertFalse(any(item[1] == "/api/nudge" for item in posts))

                bridge._queue_private(_cmd_env(2, _stick(0.8)), "cmd")
                await _wait_until(
                    lambda: any(item[1] == "/api/nudge" for item in posts),
                    timeout=2.0,
                )
                joy_at = max(
                    i for i, item in enumerate(posts)
                    if item[0] == "http" and abs(item[2]["a"][1] - 0.8) < 1e-6
                )
                self.assertGreater(
                    next(i for i, item in enumerate(posts) if item[1] == "/api/nudge"),
                    joy_at,
                )
                self.assertFalse(bridge._guard.hard)
                self.assertFalse(bridge._guard.owed)
            finally:
                await _stop_loops(bridge, task, original_http, old_gamepad)

        asyncio.run(run())

    def test_newer_stick_does_not_zero_between_posts(self):
        import asyncio

        import amr_telemetry_bridge.bridge as bridge_mod

        async def run():
            bridge = _dispatch_bridge()
            bridge._loop = asyncio.get_running_loop()
            posts: list[tuple] = []
            entered = asyncio.Event()
            release = asyncio.Event()
            held = {"on": True}
            # This case is about a newer stick, not a post that outlives the deadman.
            bridge_mod.MOTION_WATCHDOG_S = 30.0

            class _Resp:
                def raise_for_status(self):
                    return None

            async def http_post(url, json=None, timeout=None):
                axes = (json or {}).get("a") or [0.0]
                moving = len(axes) > 1 and abs(float(axes[1])) > 0.02
                if moving and held["on"]:
                    held["on"] = False
                    entered.set()
                    await release.wait()
                posts.append(("http", url, json, timeout))
                return _Resp()

            original_http = bridge.dash._http
            bridge.dash.post = _recording_dash(posts)
            bridge.dash._http = _RecordingHttp(http_post)
            old_gamepad = os.environ.get("AMR_GAMEPAD_URL")
            os.environ["AMR_GAMEPAD_URL"] = "http://127.0.0.1:8765"
            task = asyncio.create_task(bridge._motion_slot_loop())
            try:
                _arm_gate(bridge)
                bridge._queue_private(_cmd_env(1, _stick(0.8)), "cmd")
                await asyncio.wait_for(entered.wait(), timeout=1.0)
                bridge._queue_private(_cmd_env(2, _stick(0.4)), "cmd")
                release.set()
                await _wait_until(
                    lambda: sum(
                        1 for item in posts
                        if item[0] == "http" and abs(item[2]["a"][1]) > 0.02
                    ) >= 2,
                )
                self.assertFalse(any(item[1] == "/api/nudge" for item in posts))
                axes = [
                    item[2]["a"][1] for item in posts
                    if item[0] == "http" and abs(item[2]["a"][1]) > 0.02
                ]
                self.assertEqual(axes, [0.8, 0.4])
            finally:
                release.set()
                bridge_mod.MOTION_WATCHDOG_S = MOTION_WATCHDOG_S
                await _stop_loops(bridge, task, original_http, old_gamepad)

        asyncio.run(run())

    def test_public_rank_and_loops_survive_bad_input(self):
        import asyncio

        from amr_telemetry_bridge.bridge import _public_rank

        self.assertEqual(_public_rank({"from": "nope"}), 0)
        self.assertEqual(_public_rank({"from": ["rank", 1]}), 0)
        self.assertEqual(_public_rank({"from": {"rank": "high"}}), 0)
        self.assertEqual(_public_rank({"from": {"rank": True}}), 0)
        self.assertEqual(_public_rank({"from": {"rank": 1.5}}), 0)
        self.assertEqual(_public_rank({"from": {"rank": "2"}}), 2)

        async def run():
            bridge = _dispatch_bridge()
            bridge._loop = asyncio.get_running_loop()
            posts: list[tuple] = []
            self.assertIsNone(bridge._accept_motion(
                {"botId": "hardware-bot", "from": "nope", "cmd": _stick(0.5)},
                "public",
            ))
            self.assertFalse(bridge._accept_estop(
                {"botId": "hardware-bot", "from": {"rank": "high"}, "cmd": {"type": "estop"}},
                "public",
            ))

            motion_calls = {"n": 0}
            estop_calls = {"n": 0}
            original_motion = bridge._accept_motion
            original_estop = bridge._accept_estop

            def accept_motion(env, source):
                motion_calls["n"] += 1
                if motion_calls["n"] == 1:
                    raise RuntimeError("bad from")
                return original_motion(env, source)

            def accept_estop(env, source):
                estop_calls["n"] += 1
                if estop_calls["n"] == 1:
                    raise RuntimeError("bad from")
                return original_estop(env, source)

            bridge._accept_motion = accept_motion
            bridge._accept_estop = accept_estop
            original_http = bridge.dash._http
            bridge.dash.post = _recording_dash(posts)

            class _Resp:
                def raise_for_status(self):
                    return None

            async def http_post(url, json=None, timeout=None):
                posts.append(("http", url, json, timeout))
                return _Resp()

            bridge.dash._http = _RecordingHttp(http_post)
            old_gamepad = os.environ.get("AMR_GAMEPAD_URL")
            os.environ["AMR_GAMEPAD_URL"] = "http://127.0.0.1:8765"
            _arm_gate(bridge)
            estop = _cmd_env(1, {"type": "estop"})
            bridge._offer_estop(estop, "cmd")
            bridge._offer_estop(estop, "cmd")
            motion_task = asyncio.create_task(bridge._motion_slot_loop())
            estop_task = asyncio.create_task(bridge._estop_loop())
            try:
                await _wait_until(lambda: any(item[1] == "/api/estop" for item in posts))
                self.assertGreaterEqual(estop_calls["n"], 2)
                bridge._queue_private(_cmd_env(2, _stick(0.2)), "cmd")
                for _ in range(50):
                    if motion_calls["n"] >= 1:
                        break
                    await asyncio.sleep(0.02)
                self.assertGreaterEqual(motion_calls["n"], 1)
                bridge._queue_private(_cmd_env(3, _stick(0.6)), "cmd")
                await _wait_until(
                    lambda: any(
                        item[0] == "http" and abs(item[2]["a"][1] - 0.6) < 1e-6
                        for item in posts
                    ),
                )
                self.assertFalse(motion_task.done())
                self.assertFalse(estop_task.done())
            finally:
                await _stop_loops(
                    bridge, motion_task, original_http, old_gamepad, estop_task,
                )

        asyncio.run(run())

    def test_estop_hold_ignores_motion_unless_seq_is_newer(self):
        bridge = _dispatch_bridge()
        try:
            _arm_gate(bridge)
            estop = _cmd_env(10, {"type": "estop"})
            self.assertTrue(bridge._accept_estop(estop, "cmd"))
            self.assertIsNone(bridge._accept_motion(_cmd_env(10, _stick(0.4)), "cmd"))
            self.assertIsNone(bridge._accept_motion(_cmd_env(9, _stick(0.4)), "cmd"))
            resumed = bridge._accept_motion(_cmd_env(11, _stick(0.4)), "cmd")
            self.assertIsNotNone(resumed)
            self.assertEqual(resumed["type"], "joy")
            bridge._motion_hold_until = 0.0
            after = bridge._accept_motion(_cmd_env(12, _stick(0.3)), "cmd")
            self.assertIsNotNone(after)

            self.assertTrue(bridge._accept_estop(
                _cmd_env(1, {"type": "estop"}),
                "cmd",
            ))
            # This e-stop carries seq 1, so a later seq is a resume.
            bridge._estop_seq = None
            bridge._motion_hold_until = time.monotonic() + 5
            self.assertIsNone(bridge._accept_motion(_cmd_env(50, _stick(0.4)), "cmd"))
        finally:
            _close_bridge(bridge)

    def test_estop_queue_drops_oldest_past_16(self):
        bridge = _dispatch_bridge()
        try:
            for i in range(20):
                bridge._offer_estop({"cmd": {"type": "estop"}, "n": i}, "cmd")
            self.assertEqual(len(bridge._estop_q), 16)
            self.assertEqual(bridge._estop_q[0][0]["n"], 4)
            self.assertEqual(bridge._estop_q[-1][0]["n"], 19)
        finally:
            _close_bridge(bridge)

    def test_main_refuses_service_key_without_printing_it(self):
        from amr_telemetry_bridge.bridge import main

        secret = "sb_secret_example"
        with self.assertRaises(SystemExit) as caught:
            main([
                "--supabase-url", "https://example.supabase.co",
                "--api-key", secret,
                "--bridge-token", "tok",
            ])
        self.assertIn("service key", str(caught.exception))
        self.assertNotIn(secret, str(caught.exception))


class CmdAckTests(unittest.TestCase):
    """cmd_ack on bot:<id>:status for every private command."""

    def _open(self):
        bridge = _dispatch_bridge()
        _arm_gate(bridge)
        return bridge

    def _assert_ack(self, payload, *, seq, ok, reason=None):
        self.assertEqual(payload["seq"], seq)
        self.assertIs(payload["ok"], ok)
        if ok:
            self.assertEqual(set(payload), {"seq", "ok"})
        else:
            self.assertEqual(set(payload), {"seq", "ok", "reason"})
            self.assertEqual(payload["reason"], reason)

    def test_accepted_joy_and_cmdq(self):
        bridge = self._open()
        try:
            cmd = bridge._accept_motion(_cmd_env(1, _stick(0.4)), "cmd")
            self.assertEqual(cmd["type"], "joy")
            self._assert_ack(bridge._ack_q[-1], seq=1, ok=True)
            cmdq = {
                "sender": "user-1",
                "epoch": 3,
                "seq": 2,
                "ts": int(time.time() * 1000),
                "cmd": _stick(0.3),
            }
            self.assertEqual(bridge._accept_motion(cmdq, "cmdq")["type"], "joy")
            self._assert_ack(bridge._ack_q[-1], seq=2, ok=True)
            public = {
                "botId": "hardware-bot",
                "from": {"rank": 1},
                "cmd": _stick(0.2),
            }
            self.assertIsNotNone(bridge._accept_motion(public, "public"))
            self.assertEqual(len(bridge._ack_q), 2)
        finally:
            _close_bridge(bridge)

    def test_accepted_nav_on_handler(self):
        import asyncio

        async def run():
            bridge = self._open()
            posts = []
            bridge.dash.post = _recording_dash(posts)
            try:
                await bridge._handle_private_command(
                    _cmd_env(6, {"type": "nav_goal", "x": 1.0, "y": 2.0}),
                    "cmd",
                )
                self._assert_ack(bridge._ack_q[-1], seq=6, ok=True)
                self.assertTrue(any(item[1] == "/api/nav_goal" for item in posts))
            finally:
                await bridge.dash.close()

        asyncio.run(run())

    def test_bad_token_on_cmd_and_cmdq(self):
        bridge = self._open()
        secret = "lease-token-secret"
        try:
            bridge._gate.apply_poll(
                {
                    "bot_id": "hardware-bot",
                    "controller": "user-1",
                    "epoch": 3,
                    "lease_expires_at": time.time() + 3600,
                },
                {"epoch": 3, "token": secret},
                time.monotonic(),
            )
            with self.assertLogs("amr_bridge", level="INFO") as captured:
                env = _cmd_env(2, _stick(0.2))
                env["token"] = "presented-token-value"
                self.assertIsNone(bridge._accept_motion(env, "cmd"))
                cmdq = {
                    "sender": "user-1",
                    "epoch": 3,
                    "token": "presented-token-value",
                    "seq": 3,
                    "ts": int(time.time() * 1000),
                    "cmd": _stick(0.2),
                }
                self.assertIsNone(bridge._accept_motion(cmdq, "cmdq"))
            self._assert_ack(bridge._ack_q[0], seq=2, ok=False, reason="bad_token")
            self._assert_ack(bridge._ack_q[1], seq=3, ok=False, reason="bad_token")
            text = "\n".join(captured.output)
            self.assertNotIn(secret, text)
            self.assertNotIn("presented-token-value", text)
            self.assertNotIn(secret, str(list(bridge._ack_q)))
        finally:
            _close_bridge(bridge)

    def test_bad_sender(self):
        bridge = self._open()
        try:
            env = {
                "sender": "someone-else",
                "epoch": 3,
                "seq": 4,
                "ts": int(time.time() * 1000),
                "cmd": _stick(0.2),
            }
            self.assertIsNone(bridge._accept_motion(env, "cmdq"))
            self._assert_ack(bridge._ack_q[-1], seq=4, ok=False, reason="bad_sender")
        finally:
            _close_bridge(bridge)

    def test_stale_epoch(self):
        bridge = self._open()
        try:
            env = _cmd_env(2, _stick(0.2))
            env["epoch"] = 2
            self.assertIsNone(bridge._accept_motion(env, "cmd"))
            self._assert_ack(bridge._ack_q[-1], seq=2, ok=False, reason="stale_epoch")
        finally:
            _close_bridge(bridge)

    def test_lower_seq_and_duplicate_seq(self):
        bridge = self._open()
        try:
            self.assertIsNotNone(bridge._accept_motion(_cmd_env(5, _stick(0.2)), "cmd"))
            bridge._ack_q.clear()
            self.assertIsNone(bridge._accept_motion(_cmd_env(2, _stick(0.2)), "cmd"))
            self._assert_ack(bridge._ack_q[-1], seq=2, ok=False, reason="lower_seq")
            self.assertIsNone(bridge._accept_motion(_cmd_env(5, _stick(0.2)), "cmd"))
            self._assert_ack(bridge._ack_q[-1], seq=5, ok=False, reason="duplicate_seq")
        finally:
            _close_bridge(bridge)

    def test_no_lease_motion_estop_still_open(self):
        bridge = self._open()
        try:
            bridge._gate.lease_expires_at = time.time() - 5
            self.assertIsNone(bridge._accept_motion(_cmd_env(2, _stick(0.2)), "cmd"))
            self._assert_ack(bridge._ack_q[-1], seq=2, ok=False, reason="no_lease")
            # E-stop skips the lease. ok waits until the stop path starts.
            self.assertTrue(bridge._accept_estop(_cmd_env(1, {"type": "estop"}), "cmd"))
            self.assertEqual(len(bridge._ack_q), 1)
        finally:
            _close_bridge(bridge)

    def test_invalid_payload(self):
        import asyncio

        async def run():
            bridge = self._open()
            try:
                skewed = _cmd_env(4, _stick(0.2))
                skewed["ts"] = float("nan")
                self.assertIsNone(bridge._accept_motion(skewed, "cmd"))
                self._assert_ack(bridge._ack_q[-1], seq=4, ok=False, reason="invalid_payload")
                await bridge._handle_private_command({"epoch": 3, "token": "tok-1"}, "cmd")
                self._assert_ack(bridge._ack_q[-1], seq=0, ok=False, reason="invalid_payload")
                bridge._ack(
                    {"seq": 7, "token": "lease-token-secret"},
                    False,
                    "lease-token-secret",
                )
                self._assert_ack(bridge._ack_q[-1], seq=7, ok=False, reason="invalid_payload")
                self.assertNotIn("lease-token-secret", str(bridge._ack_q[-1]))
            finally:
                await bridge.dash.close()

        asyncio.run(run())

    def test_unknown_type(self):
        bridge = self._open()
        try:
            self.assertIsNone(bridge._accept_motion(_cmd_env(3, {"type": "teleport"}), "cmd"))
            self._assert_ack(bridge._ack_q[-1], seq=3, ok=False, reason="unknown_type")
        finally:
            _close_bridge(bridge)

    def test_estop_hold(self):
        bridge = self._open()
        try:
            self.assertTrue(bridge._accept_estop(_cmd_env(3, {"type": "estop"}), "cmd"))
            self.assertEqual(len(bridge._ack_q), 0)
            self.assertIsNone(bridge._accept_motion(_cmd_env(3, _stick(0.4)), "cmd"))
            self._assert_ack(bridge._ack_q[-1], seq=3, ok=False, reason="estop_hold")
        finally:
            _close_bridge(bridge)

    def test_rate_limited_when_cap_is_full_and_when_stick_is_replaced(self):
        import asyncio

        from amr_telemetry_bridge.bridge import ACK_QUEUE_MAX, MAX_SCHEDULED_COMMANDS

        bridge = self._open()
        try:
            bridge._offer_motion(_cmd_env(1, _stick(0.5)), "cmd")
            bridge._offer_motion(_cmd_env(2, _stick(0.6)), "cmd")
            self._assert_ack(bridge._ack_q[-1], seq=1, ok=False, reason="rate_limited")
            self.assertEqual(bridge._motion_pending[0]["seq"], 2)
            bridge._ack_q.clear()
            for i in range(ACK_QUEUE_MAX + 5):
                bridge._ack({"seq": i, "token": "lease-token-secret"}, False, "rate_limited")
            self.assertEqual(len(bridge._ack_q), ACK_QUEUE_MAX)
            self.assertEqual(bridge._ack_q[0]["seq"], 5)
            self.assertNotIn("lease-token-secret", str(list(bridge._ack_q)))
        finally:
            _close_bridge(bridge)

        async def run():
            bridge = self._open()
            bridge._loop = asyncio.get_running_loop()
            release = asyncio.Event()

            async def hang():
                await release.wait()

            try:
                for _ in range(MAX_SCHEDULED_COMMANDS):
                    self.assertEqual(bridge._schedule(hang()), "queued")
                await asyncio.sleep(0.05)
                secret = "lease-token-secret"
                env = _cmd_env(9, {"type": "nav_goal", "x": 1, "y": 1})
                env["token"] = secret
                bridge._queue_private(env, "cmd")
                self._assert_ack(bridge._ack_q[-1], seq=9, ok=False, reason="rate_limited")
                self.assertNotIn(secret, str(bridge._ack_q[-1]))
                # Loop is down: the command is not claimed as rate_limited.
                idle = self._open()
                try:
                    idle._queue_private(_cmd_env(1, {"type": "nav_cancel"}), "cmd")
                    self.assertEqual(len(idle._ack_q), 0)
                finally:
                    await idle.dash.close()
            finally:
                release.set()
                await asyncio.sleep(0.05)
                await bridge.dash.close()

        asyncio.run(run())

    def test_cmd_ack_stays_on_status_channel(self):
        import asyncio

        async def run():
            bridge = self._open()

            class _Hub:
                def __init__(self):
                    self.status = []
                    self.public = []

                def send_status(self, event, payload):
                    self.status.append((event, dict(payload)))

                def send(self, event, payload):
                    self.public.append((event, dict(payload)))

            hub = _Hub()
            bridge._hub = hub
            try:
                await bridge._broadcast("cmd_ack", {"seq": 1, "ok": True})
                await bridge._broadcast("hb", {"t": 1})
                self.assertEqual(hub.status, [("cmd_ack", {"seq": 1, "ok": True})])
                self.assertEqual(hub.public, [("hb", {"t": 1})])
            finally:
                await bridge.dash.close()

        asyncio.run(run())

    def test_estop_ack_not_blocked_by_stalled_joy(self):
        import asyncio
        import threading

        async def run():
            bridge = self._open()
            bridge._loop = asyncio.get_running_loop()
            posts = []
            order = []
            joy_entered = asyncio.Event()
            release_joy = asyncio.Event()
            release_send = threading.Event()
            estop_seq = 1
            secret = "lease-token-secret"
            bridge._gate.apply_poll(
                {
                    "bot_id": "hardware-bot",
                    "controller": "user-1",
                    "epoch": 3,
                    "lease_expires_at": time.time() + 3600,
                },
                {"epoch": 3, "token": secret},
                time.monotonic(),
            )

            class _Hub:
                def __init__(self):
                    self.sent = []

                def send_status(self, event, payload):
                    self.sent.append((event, dict(payload)))
                    if (
                        event == "cmd_ack"
                        and payload.get("seq") == estop_seq
                        and payload.get("ok") is True
                    ):
                        order.append("ack")
                    release_send.wait(timeout=3)

                def send(self, event, payload):
                    raise AssertionError(event)

            hub = _Hub()
            bridge._hub = hub

            class _Resp:
                def raise_for_status(self):
                    return None

            async def dash_post(path, payload=None, timeout=None):
                posts.append(("dash", path, payload, timeout))
                return {"ok": True}

            async def http_post(url, json=None, timeout=None):
                joy_entered.set()
                await release_joy.wait()
                posts.append(("http", url, json, timeout))
                return _Resp()

            original_http = bridge.dash._http
            bridge.dash.post = dash_post
            bridge.dash._http = _RecordingHttp(http_post)
            old_gamepad = os.environ.get("AMR_GAMEPAD_URL")
            os.environ["AMR_GAMEPAD_URL"] = "http://127.0.0.1:8765"
            original_estop = bridge._estop_once

            async def wrapped(accepted=None):
                order.append("started")
                return await original_estop(accepted)

            bridge._estop_once = wrapped
            motion_task = asyncio.create_task(bridge._motion_slot_loop())
            estop_task = asyncio.create_task(bridge._estop_loop())
            ack_task = asyncio.create_task(bridge._ack_loop())
            try:
                joy = _cmd_env(5, _stick(0.8))
                joy["token"] = secret
                bridge._queue_private(joy, "cmd")
                await asyncio.wait_for(joy_entered.wait(), timeout=1.0)
                bridge._gate.lease_expires_at = time.time() - 10
                bridge._queue_private(
                    {
                        "epoch": 3,
                        "token": secret,
                        "seq": estop_seq,
                        "ts": float("nan"),
                        "cmd": {"type": "estop"},
                    },
                    "cmd",
                )
                await _wait_until(lambda: any(item[1] == "/api/estop" for item in posts))
                self.assertFalse(release_joy.is_set())
                self.assertFalse(release_send.is_set())
                self.assertIn("started", order)
                self.assertNotIn("ack", order)
                release_send.set()
                await _wait_until(lambda: "ack" in order)
                self.assertFalse(release_joy.is_set())
                self.assertLess(order.index("started"), order.index("ack"))
                estop_ack = next(
                    item[1] for item in hub.sent
                    if item[0] == "cmd_ack" and item[1].get("seq") == estop_seq
                )
                self._assert_ack(estop_ack, seq=estop_seq, ok=True)
                blob = str(hub.sent)
                self.assertNotIn(secret, blob)
                self.assertFalse(any(
                    item[0] == "cmd_ack" and item[1].get("seq") == estop_seq and item[1].get("ok") is False
                    for item in hub.sent
                ))
            finally:
                release_send.set()
                release_joy.set()
                await _stop_loops(bridge, motion_task, original_http, old_gamepad, estop_task)
                ack_task.cancel()
                await asyncio.gather(ack_task, return_exceptions=True)

        asyncio.run(run())


if __name__ == "__main__":
    unittest.main(verbosity=2)
