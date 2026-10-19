"""ROS 2 direct I/O for the cloud bridge (no robot_dashboard HTTP hop).

Use when ``AMR_SOURCE=ros`` on the Pi. The bridge then:

* subscribes to ``/odom``, ``/scan``, ``/imu``, ``/map``, ``/plan``, ``/cmd_vel``
* publishes motion on ``/cmd_vel`` (still in front of ``safety_stop``)
* optionally sends Nav2 goals

The local browser UI (``robot_dashboard``) is unrelated to amrslam.tech/control.
It is optional for the cloud path. Keep it only if you still want the Pi web UI
or a Tailscale **relay** host that cannot join the robot's ROS graph.

This module imports ``rclpy`` lazily so laptop/relay installs without ROS still
work with ``AMR_SOURCE=dashboard``.
"""

from __future__ import annotations

import base64
import json
import logging
import math
import os
import threading
import time
import zlib
from typing import Any, Callable, Optional

log = logging.getLogger("amr_bridge.ros")

MAP_MAX_CELLS = 220
STATE_HZ = 10.0


class RosSource:
    """Background rclpy node → same event dicts the dashboard SSE used to emit."""

    def __init__(self, on_event: Callable[[dict], None]):
        self._on_event = on_event
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._node = None
        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "pose": [0.0, 0.0, 0.0],
            "vel": [0.0, 0.0],
            "cmd": [0.0, 0.0],
            "imu": None,
            "scan": [],
            "path": [],
            "map_pose": None,
            "nav": None,
            "sys": None,
            "tool": None,
            "vision": False,
            "saving": False,
            "frontier": {"zones": [], "queue": []},
            "yolo_det": [],
            "sem": [],
            "rates": {},
            "nodes": {},
            "tf": {},
            "goal": None,
        }
        self._latest_map = None
        self._map_dirty = False
        self._cmd_pub = None
        self._nudge_until = 0.0
        self._estop_until = 0.0
        self._nudge_cmd = (0.0, 0.0)
        self._nav_client = None
        self._tf_buffer = None
        self._prev_cpu = None
        self._last_map_push = 0.0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._thread_main, name="amr-ros-source", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=5)
        self._thread = None

    def nudge(self, lin: float, ang: float) -> dict:
        lin = max(-0.3, min(0.3, float(lin)))
        ang = max(-1.0, min(1.0, float(ang)))
        self._nudge_cmd = (lin, ang)
        self._nudge_until = time.monotonic() + 0.5
        return {"ok": True}

    def estop(self) -> dict:
        self._nudge_until = 0.0
        self._estop_until = time.monotonic() + 1.5
        self.cancel_nav()
        log.warning("ROS e-stop — streaming zero /cmd_vel for 1.5s")
        return {"ok": True}

    def nav_goal(self, x: float, y: float) -> dict:
        client = self._nav_client
        if client is None:
            return {"ok": False, "error": "nav2 not available in ros source"}
        try:
            from geometry_msgs.msg import PoseStamped
            from nav2_msgs.action import NavigateToPose
        except ImportError:
            return {"ok": False, "error": "nav2_msgs not installed"}
        if not client.wait_for_server(timeout_sec=0.5):
            return {"ok": False, "error": "navigate_to_pose server not available"}
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        goal.pose = pose
        client.send_goal_async(goal)
        with self._lock:
            self._state["goal"] = [round(float(x), 3), round(float(y), 3)]
        return {"ok": True}

    def cancel_nav(self) -> dict:
        client = self._nav_client
        if client is not None:
            try:
                client._cancel_goal_async  # noqa: B018 — existence check
            except Exception:
                pass
            try:
                # Best-effort: cancel all goals if the client supports it.
                cancel = getattr(client, "cancel_all_goals_async", None) or getattr(
                    client, "_cancel_goal_async", None
                )
                if cancel is not None:
                    cancel()
            except Exception as exc:
                log.debug("nav cancel failed: %s", exc)
        with self._lock:
            self._state["goal"] = None
        return {"ok": True}

    def unsupported(self, action: str) -> dict:
        return {
            "ok": False,
            "error": (
                f"{action} needs robot_dashboard (tool/vision/map-save). "
                "Set AMR_SOURCE=dashboard or keep the dashboard node for those commands."
            ),
        }

    def _thread_main(self) -> None:
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
            from geometry_msgs.msg import Twist
            from nav_msgs.msg import OccupancyGrid, Odometry, Path
            from sensor_msgs.msg import Imu, LaserScan
        except ImportError as exc:
            log.error(
                "AMR_SOURCE=ros requires ROS 2 Python (rclpy) on this host: %s. "
                "Use AMR_SOURCE=dashboard on a Tailscale relay instead.",
                exc,
            )
            return

        rclpy.init(args=None)
        node = Node("amr_telemetry_ros_source")
        self._node = node
        self._cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)

        node.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        node.create_subscription(Imu, "/imu", self._imu_cb, 20)
        node.create_subscription(LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data)
        node.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        node.create_subscription(Path, "/plan", self._plan_cb, 5)
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, "/map", self._map_cb, map_qos)

        try:
            import tf2_ros
            self._tf_buffer = tf2_ros.Buffer()
            tf2_ros.TransformListener(self._tf_buffer, node)
        except Exception:
            self._tf_buffer = None

        try:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
            self._nav_client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
        except Exception:
            self._nav_client = None

        node.create_timer(1.0 / STATE_HZ, self._tick)
        node.create_timer(0.05, self._motion_tick)
        node.create_timer(5.0, self._sys_tick)

        executor = SingleThreadedExecutor()
        executor.add_node(node)
        log.info("ROS source up — reading /odom /scan /map, publishing /cmd_vel")
        try:
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.1)
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._node = None

    def _odom_cb(self, m) -> None:
        q = m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self._state["pose"] = [
                round(m.pose.pose.position.x, 3),
                round(m.pose.pose.position.y, 3),
                round(yaw, 4),
            ]
            self._state["vel"] = [
                round(m.twist.twist.linear.x, 3),
                round(m.twist.twist.angular.z, 3),
            ]

    def _imu_cb(self, m) -> None:
        with self._lock:
            self._state["imu"] = {
                "gz": round(m.angular_velocity.z, 4),
                "ax": round(m.linear_acceleration.x, 3),
                "ay": round(m.linear_acceleration.y, 3),
            }

    def _cmd_cb(self, m) -> None:
        with self._lock:
            self._state["cmd"] = [round(m.linear.x, 3), round(m.angular.z, 3)]

    def _scan_cb(self, m) -> None:
        pts = []
        angle = m.angle_min
        step = max(1, len(m.ranges) // 360)
        for i, r in enumerate(m.ranges):
            if i % step != 0:
                angle += m.angle_increment
                continue
            if m.range_min < r < m.range_max and math.isfinite(r):
                pts.append([round(angle, 3), round(float(r), 2)])
            angle += m.angle_increment
        with self._lock:
            self._state["scan"] = pts

    def _plan_cb(self, m) -> None:
        if not m.poses:
            with self._lock:
                self._state["path"] = []
            return
        step = max(1, len(m.poses) // 60)
        path = []
        for p in m.poses[::step]:
            path.append([round(p.pose.position.x, 2), round(p.pose.position.y, 2)])
        with self._lock:
            self._state["path"] = path

    def _map_cb(self, m) -> None:
        with self._lock:
            self._latest_map = m
            self._map_dirty = True

    def _motion_tick(self) -> None:
        if self._cmd_pub is None:
            return
        from geometry_msgs.msg import Twist

        now = time.monotonic()
        msg = Twist()
        if now < self._estop_until:
            self._cmd_pub.publish(msg)
        elif now < self._nudge_until:
            msg.linear.x, msg.angular.z = self._nudge_cmd
            self._cmd_pub.publish(msg)
        elif self._nudge_until and now < self._nudge_until + 0.3:
            self._cmd_pub.publish(msg)

    def _sys_tick(self) -> None:
        info: dict[str, Any] = {}
        try:
            with open("/proc/stat") as f:
                vals = list(map(int, f.readline().split()[1:8]))
            idle, total = vals[3] + vals[4], sum(vals)
            if self._prev_cpu:
                dt = total - self._prev_cpu[1]
                if dt > 0:
                    info["cpu"] = round(100.0 * (1 - (idle - self._prev_cpu[0]) / dt), 1)
            self._prev_cpu = (idle, total)
        except (OSError, ValueError):
            pass
        try:
            info["load"] = round(os.getloadavg()[0], 2)
        except OSError:
            pass
        try:
            with open("/proc/meminfo") as f:
                kv = {}
                for line in f:
                    parts = line.split()
                    if len(parts) >= 2:
                        kv[parts[0].rstrip(":")] = int(parts[1])
                total = kv.get("MemTotal", 0) / 1024
                avail = kv.get("MemAvailable", 0) / 1024
                if total:
                    info["mem"] = [round(total - avail, 0), round(total, 0)]
        except (OSError, ValueError):
            pass
        with self._lock:
            self._state["sys"] = info or None

    def _tick(self) -> None:
        with self._lock:
            event = dict(self._state)
            dirty = self._map_dirty
            latest = self._latest_map
            if dirty:
                self._map_dirty = False
        # Pose-bearing state frame (same shape as dashboard SSE).
        try:
            self._on_event(event)
        except Exception:
            log.exception("ros state event failed")
        if dirty and latest is not None and time.monotonic() - self._last_map_push > 1.0:
            self._last_map_push = time.monotonic()
            try:
                self._push_map(latest)
            except Exception:
                log.exception("ros map push failed")

    def _push_map(self, m) -> None:
        import rclpy

        tf = [0.0, 0.0, 0.0]
        if self._tf_buffer is not None:
            try:
                tr = self._tf_buffer.lookup_transform("odom", "map", rclpy.time.Time())
                q = tr.transform.rotation
                tf = [
                    tr.transform.translation.x,
                    tr.transform.translation.y,
                    math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                ]
            except Exception:
                pass
        w, h = m.info.width, m.info.height
        f = max(1, (max(w, h) + MAP_MAX_CELLS - 1) // MAP_MAX_CELLS)
        data = m.data
        rows = []
        for r in range(0, h, f):
            base = r * w
            rows.append(
                bytes(0 if v < 0 else (2 if v > 50 else 1) for v in data[base : base + w : f])
            )
        if not rows:
            return
        packed = base64.b64encode(zlib.compress(b"".join(rows), 6)).decode()
        self._on_event({
            "t": "map",
            "w": len(rows[0]),
            "h": len(rows),
            "res": round(m.info.resolution * f, 4),
            "origin": [
                round(m.info.origin.position.x, 3),
                round(m.info.origin.position.y, 3),
            ],
            "tf": [round(tf[0], 3), round(tf[1], 3), round(tf[2], 4)],
            "data": packed,
        })


def resolve_source(raw: Optional[str]) -> str:
    """Return ``ros`` or ``dashboard`` (default)."""
    v = (raw or "dashboard").strip().lower()
    if v in ("ros", "rclpy", "direct"):
        return "ros"
    return "dashboard"
