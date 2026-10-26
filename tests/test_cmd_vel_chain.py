#!/usr/bin/env python3
"""Safety-chain tests that do not need the robot.

The graph half is skipped when rclpy is not installed. The policy, the
launch wiring, and the "who may publish /cmd_vel_safe" scan always run.
"""

from __future__ import annotations

import ast
import importlib.util
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from Ros_lidar_bot.cmd_vel_filter import (  # noqa: E402
    CMD_VEL_IN,
    CMD_VEL_OUT,
    DRIVER_NODE_NAME,
    EMERGENCY_STOP_TOPIC,
    SAFETY_NODE_NAME,
    filter_velocity,
)


def _load_check_script():
    path = ROOT / "scripts" / "check_cmd_vel_chain.py"
    spec = importlib.util.spec_from_file_location("check_cmd_vel_chain", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CHECK = _load_check_script()


def _parameter_defaults(path: Path) -> dict:
    tree = ast.parse(path.read_text())
    found = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else None
        if name != "declare_parameter" or len(node.args) < 2:
            continue
        key, val = node.args[0], node.args[1]
        if isinstance(key, ast.Constant) and isinstance(val, ast.Constant):
            found[key.value] = val.value
    return found


class FilterPolicyTests(unittest.TestCase):
    def test_estop_zeros_both_axes_ahead_of_a_clear_path(self):
        self.assertEqual(
            filter_velocity(0.4, 0.8, estop=True, odom_stale=False,
                            blocked_fwd=False, blocked_bwd=False),
            (0.0, 0.0),
        )

    def test_estop_zeros_reverse_and_outranks_the_scan_gate(self):
        self.assertEqual(
            filter_velocity(-0.3, -1.0, estop=True, odom_stale=False,
                            blocked_fwd=True, blocked_bwd=True),
            (0.0, 0.0),
        )

    def test_stale_odom_zeros_both_axes(self):
        self.assertEqual(
            filter_velocity(0.2, 0.5, estop=False, odom_stale=True,
                            blocked_fwd=False, blocked_bwd=False),
            (0.0, 0.0),
        )

    def test_forward_block_keeps_spin_and_reverse(self):
        self.assertEqual(
            filter_velocity(0.3, 0.7, estop=False, odom_stale=False,
                            blocked_fwd=True, blocked_bwd=False),
            (0.0, 0.7),
        )
        self.assertEqual(
            filter_velocity(-0.2, 0.4, estop=False, odom_stale=False,
                            blocked_fwd=True, blocked_bwd=False),
            (-0.2, 0.4),
        )

    def test_rear_block_keeps_forward_and_spin(self):
        self.assertEqual(
            filter_velocity(-0.2, 0.4, estop=False, odom_stale=False,
                            blocked_fwd=False, blocked_bwd=True),
            (0.0, 0.4),
        )
        self.assertEqual(
            filter_velocity(0.2, -0.3, estop=False, odom_stale=False,
                            blocked_fwd=False, blocked_bwd=True),
            (0.2, -0.3),
        )

    def test_clear_path_is_unchanged(self):
        self.assertEqual(
            filter_velocity(0.15, -0.6, estop=False, odom_stale=False,
                            blocked_fwd=False, blocked_bwd=False),
            (0.15, -0.6),
        )


class EvaluateChainTests(unittest.TestCase):
    def test_only_safety_publisher_passes(self):
        ok, lines = CHECK.evaluate_chain(
            [CMD_VEL_OUT],
            [SAFETY_NODE_NAME],
            [CMD_VEL_IN, "/scan", "/odom_raw"],
        )
        self.assertTrue(ok, lines)
        self.assertTrue(any(line.startswith("OK:") for line in lines))

    def test_rogue_motor_publisher_fails(self):
        ok, lines = CHECK.evaluate_chain(
            [CMD_VEL_OUT],
            [SAFETY_NODE_NAME, "imu_calibration_node"],
            [CMD_VEL_IN],
        )
        self.assertFalse(ok)
        self.assertTrue(any("imu_calibration_node" in line for line in lines))

    def test_driver_on_raw_cmd_vel_fails(self):
        ok, lines = CHECK.evaluate_chain(
            [CMD_VEL_IN],
            ["controller_server", "joy_teleop", "robot_dashboard"],
            [CMD_VEL_IN],
        )
        self.assertFalse(ok)
        self.assertTrue(any("not published by" in line for line in lines))

    def test_missing_driver_subscription_fails(self):
        ok, _lines = CHECK.evaluate_chain([], [SAFETY_NODE_NAME], None)
        self.assertFalse(ok)

    def test_two_twist_subscriptions_fail(self):
        ok, lines = CHECK.evaluate_chain(
            [CMD_VEL_OUT, CMD_VEL_IN],
            [SAFETY_NODE_NAME],
            None,
        )
        self.assertFalse(ok)
        self.assertTrue(any("2 Twist" in line for line in lines))

    def test_safety_not_listening_to_cmd_vel_fails(self):
        ok, lines = CHECK.evaluate_chain(
            [CMD_VEL_OUT],
            [SAFETY_NODE_NAME],
            ["/scan"],
        )
        self.assertFalse(ok)
        self.assertTrue(any(CMD_VEL_IN in line for line in lines))

    def test_script_constants_match_the_package(self):
        self.assertEqual(CHECK.SAFETY_NODE_NAME, SAFETY_NODE_NAME)
        self.assertEqual(CHECK.DRIVER_NODE_NAME, DRIVER_NODE_NAME)
        self.assertEqual(CHECK.CMD_VEL_IN, CMD_VEL_IN)


class SourceWiringTests(unittest.TestCase):
    def test_parameter_defaults_do_not_bypass_safety(self):
        imu = _parameter_defaults(ROOT / "Ros_lidar_bot" / "imu_calibration_node.py")
        drive = _parameter_defaults(ROOT / "Ros_lidar_bot" / "drive_distance_node.py")
        driver = _parameter_defaults(ROOT / "Ros_lidar_bot" / "driver_node.py")
        teleop = _parameter_defaults(ROOT / "Ros_lidar_bot" / "teleop_interface_node.py")
        self.assertEqual(imu["cmd_vel_topic"], CMD_VEL_IN)
        self.assertEqual(drive["topic"], CMD_VEL_IN)
        self.assertEqual(driver["cmd_vel_topic"], CMD_VEL_OUT)
        self.assertEqual(teleop["cmd_vel_topic"], CMD_VEL_IN)

    def test_only_safety_node_publishes_the_motor_topic(self):
        """A create_publisher aimed at /cmd_vel_safe is a bypass."""
        offenders = []
        for path in ROOT.rglob("*.py"):
            if any(part in {".venv", "build", "install", "__pycache__", "tests"} for part in path.parts):
                continue
            tree = ast.parse(path.read_text())
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                func = node.func
                name = func.attr if isinstance(func, ast.Attribute) else None
                if name != "create_publisher" or len(node.args) < 2:
                    continue
                topic = node.args[1]
                aimed_at_motor = (
                    isinstance(topic, ast.Constant) and topic.value == CMD_VEL_OUT
                ) or (isinstance(topic, ast.Name) and topic.id == "CMD_VEL_OUT")
                if aimed_at_motor:
                    offenders.append(str(path.relative_to(ROOT)))
        self.assertEqual(offenders, ["Ros_lidar_bot/safety_stop_node.py"])

    def test_cmd_vel_safe_is_not_a_publisher_default(self):
        allowed = {"Ros_lidar_bot/driver_node.py"}
        offenders = []
        for path in (ROOT / "Ros_lidar_bot").glob("*.py"):
            defaults = _parameter_defaults(path)
            if CMD_VEL_OUT in defaults.values():
                rel = str(path.relative_to(ROOT))
                if rel not in allowed:
                    offenders.append(rel)
        self.assertEqual(offenders, [])

    def test_launch_starts_safety_with_the_requested_distances(self):
        text = (ROOT / "launch" / "launch_robot.launch.py").read_text()
        self.assertRegex(text, r"(?m)^\s+safety_stop,\s*$")
        self.assertIn('"cmd_vel_topic": "/cmd_vel_safe"', text)
        self.assertNotIn('"cmd_vel_topic": "/cmd_vel"', text)
        for snippet in (
            '"min_safe_distance": 0.35',
            '"ignore_below": 0.30',
            '"front_opening_deg": 50.0',
            '"rear_opening_deg": 50.0',
            '"clear_margin": 0.10',
        ):
            self.assertIn(snippet, text)

    def test_dashboard_routes_motion_through_cmd_vel_and_latches_estop(self):
        text = (ROOT / "Ros_lidar_bot" / "robot_dashboard_node.py").read_text()
        self.assertIn('"/emergency_stop"', text)
        self.assertIn('"cmd_vel_topic:=/cmd_vel"', text)
        self.assertIn('"topic:=/cmd_vel"', text)
        self.assertIn('"safety_stop"', text)
        self.assertNotIn('"/cmd_vel_safe"', text)
        self.assertNotIn("'/cmd_vel_safe'", text)
        self.assertIn('"/api/estop"', text)
        self.assertIn('"ignored": "e-stop"', text)

    def test_launch_files_parse_as_python(self):
        for path in sorted((ROOT / "launch").glob("*.py")):
            compile(path.read_text(), str(path), "exec")

    def test_safety_node_uses_the_shared_policy(self):
        text = (ROOT / "Ros_lidar_bot" / "safety_stop_node.py").read_text()
        self.assertIn("filter_velocity", text)
        self.assertIn("EMERGENCY_STOP_TOPIC", text)
        self.assertIn(EMERGENCY_STOP_TOPIC, text)


def _ensure_rclpy():
    """Initialise ROS once. A second rclpy.init() after shutdown is not safe."""
    import atexit
    import rclpy
    from geometry_msgs.msg import Twist

    if not getattr(_ensure_rclpy, "ready", False):
        if not rclpy.ok():
            rclpy.init()

        def _shutdown():
            if rclpy.ok():
                rclpy.shutdown()

        atexit.register(_shutdown)
        _ensure_rclpy.ready = True
    return rclpy, Twist


class SafetyNodeBehaviorTests(unittest.TestCase):
    """Call the real node in-process. No hardware and no second process."""

    @classmethod
    def setUpClass(cls):
        try:
            cls.rclpy, cls.Twist = _ensure_rclpy()
        except ImportError:
            raise unittest.SkipTest("rclpy is not installed")

    def _node(self):
        from Ros_lidar_bot.safety_stop_node import SafetyStop

        node = SafetyStop()
        captured = []

        def _capture(msg):
            captured.append((msg.linear.x, msg.angular.z))

        node._pub.publish = _capture
        return node, captured

    def test_estop_drops_a_later_cmd_and_does_not_store_it(self):
        node, captured = self._node()
        try:
            from std_msgs.msg import Bool

            node._estop_cb(Bool(data=True))
            cmd = self.Twist()
            cmd.linear.x = 0.4
            cmd.angular.z = 0.8
            node._cmd_cb(cmd)
            self.assertEqual(captured[-1], (0.0, 0.0))
            self.assertEqual(node._last_cmd.linear.x, 0.0)
            self.assertEqual(node._last_cmd_time, 0.0)
            node._blocked_fwd = True
            node._publish_safe(cmd)
            self.assertEqual(captured[-1], (0.0, 0.0))
        finally:
            node.destroy_node()

    def test_without_estop_forward_block_keeps_yaw(self):
        node, captured = self._node()
        try:
            node._blocked_fwd = True
            cmd = self.Twist()
            cmd.linear.x = 0.3
            cmd.angular.z = 0.5
            node._cmd_cb(cmd)
            self.assertEqual(captured[-1], (0.0, 0.5))
            self.assertAlmostEqual(node._last_cmd.linear.x, 0.3)
        finally:
            node.destroy_node()


class LiveGraphTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            cls.rclpy, cls.Twist = _ensure_rclpy()
            from rclpy.node import Node
        except ImportError:
            raise unittest.SkipTest("rclpy is not installed")
        cls.Node = Node

    def test_graph_passes_then_fails_when_a_second_publisher_appears(self):
        from rclpy.executors import SingleThreadedExecutor

        driver = self.Node(DRIVER_NODE_NAME)
        safety = self.Node(SAFETY_NODE_NAME)
        checker = self.Node("cmd_vel_chain_check")
        driver.create_subscription(self.Twist, CMD_VEL_OUT, lambda _msg: None, 10)
        safety.create_subscription(self.Twist, CMD_VEL_IN, lambda _msg: None, 10)
        motor_pub = safety.create_publisher(self.Twist, CMD_VEL_OUT, 10)
        executor = SingleThreadedExecutor()
        for node in (driver, safety, checker):
            executor.add_node(node)
        rogue = None
        try:
            motor_pub.publish(self.Twist())
            ok = False
            lines: list[str] = []
            deadline = time_monotonic() + 8.0
            while time_monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                ok, lines = CHECK.evaluate_chain(
                    CHECK.read_chain(checker)["driver_topics"],
                    CHECK.read_chain(checker)["motor_publishers"],
                    CHECK.read_chain(checker)["safety_subs"] or None,
                )
                if ok:
                    break
            self.assertTrue(ok, lines)

            rogue = self.Node("imu_calibration_node")
            executor.add_node(rogue)
            rogue_pub = rogue.create_publisher(self.Twist, CMD_VEL_OUT, 10)
            rogue_pub.publish(self.Twist())
            fail_lines: list[str] = []
            deadline = time_monotonic() + 8.0
            while time_monotonic() < deadline:
                executor.spin_once(timeout_sec=0.1)
                snap = CHECK.read_chain(checker)
                if "imu_calibration_node" not in snap["motor_publishers"]:
                    continue
                chain_ok, fail_lines = CHECK.evaluate_chain(
                    snap["driver_topics"],
                    snap["motor_publishers"],
                    snap["safety_subs"] or None,
                )
                self.assertFalse(chain_ok, fail_lines)
                self.assertTrue(
                    any("imu_calibration_node" in line for line in fail_lines),
                    fail_lines,
                )
                return
            self.fail(f"rogue publisher was not visible: {fail_lines}")
        finally:
            executor.shutdown()
            if rogue is not None:
                rogue.destroy_node()
            for node in (driver, safety, checker):
                node.destroy_node()


def time_monotonic() -> float:
    import time
    return time.monotonic()


class LaunchSetupTests(unittest.TestCase):
    def test_launch_robot_actions_include_safety_and_filtered_driver(self):
        try:
            from launch import LaunchContext
        except ImportError:
            self.skipTest("launch is not installed")
        path = ROOT / "launch" / "launch_robot.launch.py"
        spec = importlib.util.spec_from_file_location("launch_robot_launch", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        module.get_package_share_directory = lambda name: f"/tmp/ros_share/{name}"
        context = LaunchContext()
        context.launch_configurations.update({
            "verbose": "false",
            "expect_frontier": "false",
            "enable_amr_bridge": "false",
            "amr_bot_id": "hardware-bot",
            "amr_bridge_python": "auto",
            "amr_bridge_env": "/etc/amr-bridge.env",
            "use_joystick": "false",
        })
        actions = module._launch_setup(context)
        nodes = []

        def walk(items):
            for item in items:
                # TimerAction stores its actions on .actions in some versions
                # and on the entity list in others. Collect Node-like objects.
                if hasattr(item, "node_executable") or type(item).__name__ == "Node":
                    nodes.append(item)
                nested = getattr(item, "actions", None)
                if nested:
                    walk(nested)

        walk(actions)
        # launch_ros Node may not expose actions; also scan by repr of the
        # entity list returned directly (the untimed nodes are in `actions`).
        names = []
        driver_params = None
        safety_params = None
        for item in actions:
            name = _launch_node_name(item)
            if name in ("driver_node", "safety_stop", "robot_dashboard"):
                names.append(name)
            if name == "driver_node":
                driver_params = _evaluated_params(item, context)
            if name == "safety_stop":
                safety_params = _evaluated_params(item, context)
        self.assertIn("safety_stop", names)
        self.assertIn("driver_node", names)
        self.assertEqual(driver_params.get("cmd_vel_topic"), "/cmd_vel_safe")
        self.assertEqual(safety_params.get("min_safe_distance"), 0.35)
        self.assertEqual(safety_params.get("ignore_below"), 0.30)
        self.assertEqual(safety_params.get("front_opening_deg"), 50.0)
        self.assertEqual(safety_params.get("rear_opening_deg"), 50.0)
        self.assertEqual(safety_params.get("clear_margin"), 0.10)


def _launch_node_name(item):
    # launch_ros Node.node_name raises until the action is executed.
    # The constructor stores the requested name on the private attribute.
    value = getattr(item, "_Node__node_name", None)
    return value if isinstance(value, str) else None


def _evaluated_params(item, context) -> dict:
    from launch.utilities import perform_substitutions

    raw = getattr(item, "_Node__parameters", None) or []
    flat = {}
    for entry in raw:
        if not isinstance(entry, dict):
            continue
        for key, value in entry.items():
            if isinstance(key, tuple):
                name = perform_substitutions(context, list(key))
            else:
                name = str(key)
            if isinstance(value, tuple):
                import yaml
                text = perform_substitutions(context, list(value))
                flat[name] = yaml.safe_load(text)
            else:
                flat[name] = value
    return flat


if __name__ == "__main__":
    unittest.main()
