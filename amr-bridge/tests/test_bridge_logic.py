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

from amr_telemetry_bridge.protocol import (  # noqa: E402
    BOARD_RATE_HZ,
    BOT_IDS,
    FULL_RATE_HZ,
    HEARTBEAT_OFFLINE_MS,
    channel_name,
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
    """Mirror: URL always required; KEY+TOKEN or SERVICE_KEY."""
    missing: list[str] = []
    if not env.get("AMR_SUPABASE_URL"):
        missing.append("AMR_SUPABASE_URL")
    has_pair = bool(env.get("AMR_SUPABASE_KEY")) and bool(env.get("AMR_BRIDGE_TOKEN"))
    if not has_pair and not env.get("AMR_SUPABASE_SERVICE_KEY"):
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

    def test_service_key_still_needs_url(self):
        self.assertEqual(
            _missing_required({"AMR_SUPABASE_SERVICE_KEY": "svc"}),
            ["AMR_SUPABASE_URL"],
        )
        self.assertEqual(
            _missing_required({
                "AMR_SUPABASE_URL": "https://example.supabase.co",
                "AMR_SUPABASE_SERVICE_KEY": "svc",
            }),
            [],
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
