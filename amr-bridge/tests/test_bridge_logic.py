#!/usr/bin/env python3
"""Logic tests for amr_telemetry_bridge + launch wiring (no ROS required)."""

from __future__ import annotations

import ast
import os
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BRIDGE_ROOT = ROOT / "amr-bridge"
sys.path.insert(0, str(BRIDGE_ROOT))

from amr_telemetry_bridge.control import (  # noqa: E402
    CONTROL_POLL_MAX_S,
    CONTROL_POLL_MIN_S,
    CONTROL_POLL_S,
    MOTION_WATCHDOG_S,
    SUPABASE_UNREACHABLE_S,
    TS_WINDOW_S,
    ControlTracker,
    clamp_poll_interval,
    flag_enabled,
    parse_timestamptz,
)
from amr_telemetry_bridge.protocol import (  # noqa: E402
    BOARD_RATE_HZ,
    BOT_IDS,
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
        self.assertEqual(STATUS_EVENTS, ("telemetry", "map", "log", "hb"))
        self.assertEqual(CMD_EVENT, "command")
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
        self.assertIn("AMR_PUBLIC_CHANNEL=1", example)
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
        # Gaps are still strictly increasing.
        self.assertTrue(gate2.verify(
            self._envelope(seq=4), source="cmd", now_s=NOW, now_mono=1.0,
        ).ok)

        late = gate.verify(
            self._envelope(seq=2, ts=int(NOW * 1000) - 2001),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertEqual(late.reason, "ts")
        future = gate.verify(
            self._envelope(seq=2, ts=int(NOW * 1000) + 2001),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertEqual(future.reason, "ts")
        edge = gate.verify(
            self._envelope(seq=2, ts=int(NOW * 1000) + 2000),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertTrue(edge.ok, edge.reason)

    def test_ts_seconds_and_retry_after_stale_ts_keeps_seq(self):
        gate, _ = self._arm()
        stale = gate.verify(
            self._envelope(seq=7, ts=NOW - 2.001),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertEqual(stale.reason, "ts")
        # Seconds form, exactly on the 2 s boundary, same seq (not consumed).
        ok = gate.verify(
            self._envelope(seq=7, ts=NOW + TS_WINDOW_S),
            source="cmd", now_s=NOW, now_mono=0.0,
        )
        self.assertTrue(ok.ok, ok.reason)

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

    def test_epoch_change_resets_seq_and_clears_token_until_poll(self):
        gate, _ = self._arm()
        self.assertTrue(gate.verify(
            self._envelope(seq=9), source="cmd", now_s=NOW, now_mono=0.0,
        ).ok)
        reasons = gate.apply_ctl(
            {
                "bot_id": "hardware-bot",
                "controller": "user-2",
                "controller_name": "Two",
                "epoch": 4,
            },
            now_mono=1.0,
        )
        self.assertEqual(reasons, ["epoch_change"])
        self.assertIsNone(gate.token)
        self.assertIsNone(gate.lease_expires_at)
        held = gate.verify(
            self._envelope(epoch=4, token="tok-1", seq=1),
            source="cmd", now_s=NOW, now_mono=1.0,
        )
        self.assertEqual(held.reason, "lease")
        # Stale poll must not roll the epoch or the token backwards.
        self.assertEqual(gate.apply_poll(
            {
                "bot_id": "hardware-bot",
                "controller": "user-1",
                "epoch": 3,
                "lease_expires_at": NOW + 30,
            },
            {"bot_id": "hardware-bot", "epoch": 3, "token": "tok-1"},
            1.1,
        ), [])
        self.assertEqual(gate.epoch, 4)
        self.assertIsNone(gate.token)

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
        self.assertEqual(reasons, [])
        self.assertTrue(gate.verify(
            self._envelope(epoch=4, token="tok-2", seq=1),
            source="cmd", now_s=NOW, now_mono=1.2,
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
        self.assertEqual(closed.reason, "lease")


class BridgeSourceSafetyTests(unittest.TestCase):
    def test_velocity_stays_on_existing_http_path(self):
        src = (BRIDGE_ROOT / "amr_telemetry_bridge" / "bridge.py").read_text()
        self.assertIn('"/api/nudge"', src)
        self.assertIn("/api/joy", src)
        self.assertIn("zero motion", src)
        self.assertNotIn("create_publisher", src)
        self.assertNotIn('publish("/cmd_vel_safe"', src)
        self.assertNotIn("cmd_vel_topic", src)

    def test_private_auth_and_touch_logging(self):
        src = (BRIDGE_ROOT / "amr_telemetry_bridge" / "bridge.py").read_text()
        self.assertIn("sign_in_with_password", src)
        self.assertIn("refresh_session", src)
        self.assertIn("set_auth", src)
        self.assertIn("AMR_BRIDGE_EMAIL", src)
        self.assertIn("AMR_BRIDGE_PASSWORD", src)
        self.assertIn("AMR_PUBLIC_CHANNEL", src)
        self.assertIn('{"config": {"private": True}}', src)
        self.assertIn("touch_amr_bot returned false", src)
        self.assertNotIn('or _env("AMR_SUPABASE_SERVICE_KEY")', src)
        for event in ("telemetry", "map", "log", "hb"):
            self.assertIn(event, src)

    def test_setup_script_names_only(self):
        script = (ROOT / "scripts" / "setup_amr_bridge.sh").read_text()
        self.assertIn("AMR_BRIDGE_EMAIL=\n", script)
        self.assertIn("AMR_BRIDGE_PASSWORD=\n", script)
        self.assertIn("AMR_PUBLIC_CHANNEL=1\n", script)
        self.assertIn("Do NOT ever add AMR_SUPABASE_SERVICE_KEY", script)
        readme = (ROOT / "scripts" / "README.md").read_text()
        self.assertIn("AMR_BRIDGE_EMAIL", readme)
        self.assertIn("AMR_BRIDGE_PASSWORD", readme)


if __name__ == "__main__":
    unittest.main(verbosity=2)
