#!/usr/bin/env python3
"""
bringup_status_node.py — Live bringup health board for Ros_lidar_bot.

Prints a compact terminal dashboard covering:
  - expected nodes (staged by launch timers)
  - Nav2 lifecycle states
  - topic rate / freshness
  - TF chain presence, age, and whether dynamic edges are updating
  - QoS mismatches on critical topics
  - rolling ISSUES feed with actionable hints

Designed to be the only screen-facing process when launch runs with verbose:=false.
"""

from __future__ import annotations

import glob
import os
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time

from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, LaserScan

try:
    import tf2_ros
    from tf2_ros import TransformException
    TF_OK = True
except ImportError:
    TF_OK = False
    TransformException = Exception  # type: ignore[misc,assignment]


# ── ANSI ─────────────────────────────────────────────────────────────────────
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
CLEAR = "\033[2J\033[H"
ALT_SCREEN_ON = "\033[?1049h"
ALT_SCREEN_OFF = "\033[?1049l"
HIDE_CURSOR = "\033[?25l"
SHOW_CURSOR = "\033[?25h"

STATE_COLORS = {
    "HEALTHY": GREEN,
    "STARTING": CYAN,
    "STALE": YELLOW,
    "INACTIVE": YELLOW,
    "MISSING": RED,
    "FAILED": RED,
    "OK": GREEN,
    "DEGRADED": YELLOW,
    "WARN": YELLOW,
    "ERROR": RED,
}


@dataclass
class TopicSpec:
    name: str
    msg_type: type
    min_hz: float
    stale_sec: float
    due_sec: float
    hint: str
    qos: Optional[QoSProfile] = None


@dataclass
class NodeSpec:
    name: str
    due_sec: float
    hint: str
    lifecycle: bool = False


@dataclass
class TfEdge:
    parent: str
    child: str
    due_sec: float
    dynamic: bool
    stale_sec: float
    frozen_sec: float
    hint: str


@dataclass
class Issue:
    level: str  # WARN | ERROR
    text: str
    count: int = 1
    last_t: float = field(default_factory=time.monotonic)


@dataclass
class TopicStats:
    stamps: Deque[float] = field(default_factory=lambda: deque(maxlen=64))
    last_wall: float = 0.0

    def on_msg(self) -> None:
        now = time.monotonic()
        self.stamps.append(now)
        self.last_wall = now

    def hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        dt = self.stamps[-1] - self.stamps[0]
        if dt <= 1e-6:
            return 0.0
        return (len(self.stamps) - 1) / dt

    def age(self) -> Optional[float]:
        if self.last_wall <= 0.0:
            return None
        return time.monotonic() - self.last_wall


class BringupStatusNode(Node):
    def __init__(self) -> None:
        super().__init__("bringup_status")

        self._expect_frontier = bool(self.declare_parameter("expect_frontier", False).value)
        self._update_hz = float(self.declare_parameter("update_hz", 2.0).value)
        self._verbose_issues = bool(self.declare_parameter("verbose_issues", True).value)
        self._clear_screen = bool(self.declare_parameter("clear_screen", True).value)
        self._qos_period = float(self.declare_parameter("qos_check_period_sec", 5.0).value)
        self._max_issues = int(self.declare_parameter("max_issues", 12).value)
        self._expected_scan_frame = str(
            self.declare_parameter("expected_scan_frame", "laser_frame").value
        )

        self._t0 = time.monotonic()
        self._last_qos_check = 0.0
        self._issues: Dict[str, Issue] = {}
        self._seen_nodes: set[str] = set()
        self._lifecycle_clients: Dict[str, any] = {}
        self._lifecycle_cache: Dict[str, Tuple[str, float]] = {}
        self._lifecycle_futures: Dict[str, any] = {}
        self._scan_frame_id: Optional[str] = None
        self._last_board: str = ""
        self._out = None  # set in _open_tty

        # TF motion tracking: child -> (x, y, yaw_approx, last_change_wall)
        self._tf_pose: Dict[str, Tuple[float, float, float]] = {}
        self._tf_last_change: Dict[str, float] = {}

        self._node_specs = self._build_node_specs()
        self._topic_specs = self._build_topic_specs()
        self._tf_edges = self._build_tf_edges()
        self._topic_stats: Dict[str, TopicStats] = {
            t.name: TopicStats() for t in self._topic_specs
        }

        self._open_tty()
        self._subscribe_topics()
        if TF_OK:
            self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        else:
            self._tf_buffer = None
            self._add_issue("ERROR", "tf2_ros unavailable — TF checks disabled")

        period = 1.0 / max(self._update_hz, 0.2)
        self.create_timer(period, self._tick)

        # Own logger stays quiet so it cannot pollute the board.
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.FATAL)
        if self._out is not None and self._clear_screen and self._out.isatty():
            self._out.write(ALT_SCREEN_ON + HIDE_CURSOR)
            self._out.flush()

    # ── specs ────────────────────────────────────────────────────────────────

    def _build_node_specs(self) -> List[NodeSpec]:
        specs = [
            NodeSpec("robot_state_publisher", 0.0, "URDF / rsp.launch failed"),
            NodeSpec("joint_state_publisher", 0.0, "joint_state_publisher missing"),
            NodeSpec("imu_node", 0.0, "check /dev/ttyACM1 and Arduino IMU"),
            NodeSpec("driver_node", 0.0, "check /dev/ttyACM0 and DDSM115"),
            NodeSpec("odom_node", 0.0, "odom_node not running"),
            NodeSpec("rplidar_node", 0.0, "check eth0 / ping 192.168.11.2 / lidar power"),
            NodeSpec("safety_stop", 0.0, "safety_stop_node not running"),
            NodeSpec("ekf_filter_node", 0.0, "robot_localization EKF not running"),
            NodeSpec("slam_toolbox", 5.0, "SLAM Toolbox failed to start (due T+5s)"),
            NodeSpec("bt_navigator", 8.0, "Nav2 bt_navigator not up", lifecycle=True),
            NodeSpec("controller_server", 8.0, "Nav2 controller_server not up", lifecycle=True),
            NodeSpec("planner_server", 8.0, "Nav2 planner_server not up", lifecycle=True),
            NodeSpec("behavior_server", 8.0, "Nav2 behavior_server not up", lifecycle=True),
            NodeSpec("smoother_server", 8.0, "Nav2 smoother_server not up", lifecycle=True),
            NodeSpec("velocity_smoother", 8.0, "Nav2 velocity_smoother not up", lifecycle=True),
            NodeSpec("collision_monitor", 8.0, "Nav2 collision_monitor not up", lifecycle=True),
            NodeSpec(
                "lifecycle_manager_navigation",
                8.0,
                "Nav2 lifecycle_manager not up",
            ),
        ]
        if self._expect_frontier:
            specs.append(
                NodeSpec(
                    "frontier_explorer",
                    22.0,
                    "frontier_explorer failed after Nav2 ready",
                )
            )
        return specs

    def _build_topic_specs(self) -> List[TopicSpec]:
        sensor_qos = qos_profile_sensor_data
        return [
            TopicSpec(
                "/imu", Imu, 20.0, 0.5, 2.0,
                "no IMU — check /dev/ttyACM1 / Arduino power",
                sensor_qos,
            ),
            TopicSpec(
                "/scan", LaserScan, 8.0, 1.0, 3.0,
                "no /scan — eth0 192.168.11.1, ping 192.168.11.2, lidar power",
                sensor_qos,
            ),
            TopicSpec(
                "/odom_raw", Odometry, 10.0, 0.5, 2.0,
                "no /odom_raw — check driver_node /dev/ttyACM0",
                sensor_qos,
            ),
            TopicSpec(
                "/odom", Odometry, 15.0, 0.5, 3.0,
                "no /odom — EKF not fusing (need /odom_raw + /imu)",
                sensor_qos,
            ),
            TopicSpec(
                "/map", OccupancyGrid, 0.0, 90.0, 12.0,
                "no /map — SLAM not publishing (need /scan + odom TF)",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]

    def _build_tf_edges(self) -> List[TfEdge]:
        return [
            TfEdge(
                "odom", "base_footprint", 2.0, True, 0.5, 3.0,
                "EKF not publishing odom→base_footprint (need /odom_raw+/imu)",
            ),
            TfEdge(
                "map", "odom", 12.0, True, 2.0, 8.0,
                "SLAM not publishing map→odom (need /scan + odom TF)",
            ),
            TfEdge(
                "base_footprint", "base_link", 2.0, False, 5.0, 0.0,
                "URDF/RSP missing base_footprint→base_link",
            ),
            TfEdge(
                "base_link", "laser_frame", 2.0, False, 5.0, 0.0,
                "URDF/RSP missing base_link→laser_frame",
            ),
            TfEdge(
                "base_link", "imu_link", 2.0, False, 5.0, 0.0,
                "URDF/RSP missing base_link→imu_link",
            ),
        ]

    # ── subscriptions ────────────────────────────────────────────────────────

    def _subscribe_topics(self) -> None:
        for spec in self._topic_specs:
            qos = spec.qos if spec.qos is not None else 10

            if spec.name == "/scan":
                def _scan_cb(msg: LaserScan, name=spec.name):
                    self._topic_stats[name].on_msg()
                    self._scan_frame_id = msg.header.frame_id
                    if (
                        self._expected_scan_frame
                        and msg.header.frame_id
                        and msg.header.frame_id != self._expected_scan_frame
                    ):
                        self._add_issue(
                            "ERROR",
                            f"/scan frame_id='{msg.header.frame_id}' but URDF/TF "
                            f"expects '{self._expected_scan_frame}' — RViz scan will "
                            f"detach/rotate with the robot. Set frame_id:=laser_frame "
                            f"(do not use official sllidar_s2e_launch alone).",
                        )

                self.create_subscription(spec.msg_type, spec.name, _scan_cb, qos)
            else:
                def _cb(_msg, name=spec.name):
                    self._topic_stats[name].on_msg()

                self.create_subscription(spec.msg_type, spec.name, _cb, qos)

        # Optional cmd pipeline — only used for presence/QoS, not rate alarms while idle.
        self.create_subscription(Twist, "/cmd_vel", lambda _m: None, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda _m: None, 10)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _open_tty(self) -> None:
        """Prefer /dev/tty so launch-multiplexed stdout from other nodes cannot
        permanently corrupt the board (we redraw on the real terminal)."""
        try:
            self._out = open("/dev/tty", "w", encoding="utf-8", buffering=1)
        except OSError:
            self._out = sys.stdout

    def _write_board(self, text: str) -> None:
        out = self._out or sys.stdout
        if self._clear_screen and out.isatty():
            out.write(CLEAR + text + "\n")
        else:
            if text != self._last_board:
                out.write("\n" + text + "\n")
        out.flush()
        self._last_board = text

    def destroy_node(self) -> bool:
        out = self._out
        if out is not None and out is not sys.stdout:
            try:
                if self._clear_screen and out.isatty():
                    out.write(SHOW_CURSOR + ALT_SCREEN_OFF)
                    out.flush()
                out.close()
            except Exception:
                pass
            self._out = None
        return super().destroy_node()

    def _elapsed(self) -> float:
        return time.monotonic() - self._t0

    def _add_issue(self, level: str, text: str) -> None:
        key = f"{level}:{text}"
        now = time.monotonic()
        if key in self._issues:
            self._issues[key].count += 1
            self._issues[key].last_t = now
        else:
            self._issues[key] = Issue(level=level, text=text, last_t=now)
            # Cap size — drop oldest by last_t
            if len(self._issues) > self._max_issues * 2:
                ordered = sorted(self._issues.items(), key=lambda kv: kv[1].last_t)
                for k, _ in ordered[: len(ordered) - self._max_issues]:
                    del self._issues[k]

    def _latest_log_dir(self) -> str:
        base = os.path.expanduser("~/.ros/log")
        if not os.path.isdir(base):
            return base
        dirs = sorted(
            (p for p in glob.glob(os.path.join(base, "*")) if os.path.isdir(p)),
            key=os.path.getmtime,
            reverse=True,
        )
        return dirs[0] if dirs else base

    def _node_names_short(self) -> set[str]:
        names = set()
        try:
            for full in self.get_node_names():
                # get_node_names returns bare names without namespace in many distros
                names.add(full.split("/")[-1])
        except Exception as exc:
            self._add_issue("WARN", f"node graph query failed: {exc}")
        return names

    def _lifecycle_label(self, node_name: str) -> Optional[str]:
        """Non-blocking lifecycle poll: start async call, read result on later ticks."""
        now = time.monotonic()
        cached = self._lifecycle_cache.get(node_name)
        if cached and (now - cached[1]) < 2.0:
            return cached[0]

        if node_name not in self._lifecycle_clients:
            self._lifecycle_clients[node_name] = self.create_client(
                GetState, f"/{node_name}/get_state"
            )
        client = self._lifecycle_clients[node_name]

        pending = self._lifecycle_futures.get(node_name)
        if pending is not None and pending.done():
            try:
                resp = pending.result()
                if resp is not None:
                    label = {
                        LifecycleState.PRIMARY_STATE_UNKNOWN: "unknown",
                        LifecycleState.PRIMARY_STATE_UNCONFIGURED: "unconfigured",
                        LifecycleState.PRIMARY_STATE_INACTIVE: "inactive",
                        LifecycleState.PRIMARY_STATE_ACTIVE: "active",
                        LifecycleState.PRIMARY_STATE_FINALIZED: "finalized",
                    }.get(
                        resp.current_state.id,
                        resp.current_state.label or "unknown",
                    )
                    self._lifecycle_cache[node_name] = (label, now)
                    self._lifecycle_futures.pop(node_name, None)
                    return label
            except Exception as exc:
                self._add_issue("WARN", f"lifecycle get_state({node_name}) failed: {exc}")
            self._lifecycle_futures.pop(node_name, None)
        elif pending is not None:
            return cached[0] if cached else None

        if client.service_is_ready() and node_name not in self._lifecycle_futures:
            self._lifecycle_futures[node_name] = client.call_async(GetState.Request())

        return cached[0] if cached else None

    # ── evaluation ───────────────────────────────────────────────────────────

    def _eval_nodes(self, present: set[str]) -> List[Tuple[str, str, str]]:
        rows = []
        t = self._elapsed()
        grace = 4.0  # extra seconds after due before MISSING/FAILED
        nav_grace = 12.0

        for spec in self._node_specs:
            due = spec.due_sec
            name = spec.name
            if name in present:
                self._seen_nodes.add(name)

            if t < due:
                rows.append((name, "STARTING", f"due at t={due:.0f}s"))
                continue

            if name not in present:
                if name in self._seen_nodes:
                    rows.append((name, "FAILED", f"was running, now gone — {spec.hint}"))
                    self._add_issue("ERROR", f"{name} died — {spec.hint}")
                elif t > due + grace:
                    rows.append((name, "MISSING", spec.hint))
                    self._add_issue("ERROR", f"{name} missing — {spec.hint}")
                else:
                    rows.append((name, "STARTING", f"waiting (due t={due:.0f}s)"))
                continue

            if spec.lifecycle:
                life = self._lifecycle_label(name)
                if life is None:
                    if t > due + nav_grace:
                        rows.append((name, "INACTIVE", "lifecycle state unknown"))
                        self._add_issue("WARN", f"{name}: lifecycle get_state not ready")
                    else:
                        rows.append((name, "STARTING", "waiting lifecycle"))
                elif life == "active":
                    rows.append((name, "HEALTHY", f"lifecycle={life}"))
                elif life in ("unconfigured", "inactive", "unknown"):
                    if t > due + nav_grace:
                        rows.append((name, "INACTIVE", f"lifecycle={life}"))
                        self._add_issue(
                            "ERROR",
                            f"{name} stuck lifecycle={life} — Nav2 bringup failed",
                        )
                    else:
                        rows.append((name, "STARTING", f"lifecycle={life}"))
                elif life == "finalized":
                    rows.append((name, "FAILED", f"lifecycle={life}"))
                    self._add_issue("ERROR", f"{name} finalized")
                else:
                    rows.append((name, "INACTIVE", f"lifecycle={life}"))
            else:
                rows.append((name, "HEALTHY", "on graph"))
        return rows

    def _eval_topics(self) -> List[Tuple[str, str, str]]:
        rows = []
        t = self._elapsed()
        for spec in self._topic_specs:
            st = self._topic_stats[spec.name]
            age = st.age()
            hz = st.hz()

            if t < spec.due_sec:
                rows.append((spec.name, "STARTING", f"due at t={spec.due_sec:.0f}s"))
                continue

            if age is None:
                rows.append((spec.name, "MISSING", f"no messages — {spec.hint}"))
                self._add_issue("ERROR", f"{spec.name} silent — {spec.hint}")
                continue

            if age > spec.stale_sec:
                rows.append(
                    (spec.name, "STALE", f"age={age:.2f}s hz={hz:.1f} — {spec.hint}")
                )
                self._add_issue(
                    "ERROR",
                    f"{spec.name} stale age={age:.2f}s — {spec.hint}",
                )
                continue

            detail = f"{hz:.1f} Hz  age={age:.2f}s"
            # /map is latched / infrequently republished — do not rate-alarm it.
            if spec.name == "/map":
                rows.append((spec.name, "HEALTHY", f"latched  age={age:.1f}s"))
                continue
            if hz + 1e-6 < spec.min_hz and spec.min_hz > 0.0:
                rows.append((spec.name, "STALE", f"{detail} (min {spec.min_hz:.1f})"))
                self._add_issue(
                    "WARN",
                    f"{spec.name} rate {hz:.1f} Hz below min {spec.min_hz:.1f}",
                )
            else:
                rows.append((spec.name, "HEALTHY", detail))
        return rows

    def _eval_tf(self) -> List[Tuple[str, str, str]]:
        rows = []
        t = self._elapsed()
        if not TF_OK or self._tf_buffer is None:
            rows.append(("tf2", "FAILED", "tf2_ros not available"))
            return rows

        now_msg = self.get_clock().now()
        for edge in self._tf_edges:
            label = f"{edge.parent} -> {edge.child}"
            if t < edge.due_sec:
                rows.append((label, "STARTING", f"due at t={edge.due_sec:.0f}s"))
                continue

            try:
                tf = self._tf_buffer.lookup_transform(
                    edge.parent,
                    edge.child,
                    Time(),  # latest
                    timeout=Duration(seconds=0.05),
                )
            except TransformException as exc:
                rows.append((label, "MISSING", f"{exc} — {edge.hint}"))
                self._add_issue("ERROR", f"TF {label} missing — {edge.hint}")
                continue
            except Exception as exc:
                rows.append((label, "FAILED", str(exc)))
                self._add_issue("ERROR", f"TF {label} lookup error: {exc}")
                continue

            stamp = Time.from_msg(tf.header.stamp)
            if stamp.nanoseconds == 0:
                age = 0.0
            else:
                age = max(0.0, (now_msg - stamp).nanoseconds / 1e9)

            tr = tf.transform.translation
            # Approximate yaw from quaternion z/w for freeze detection
            q = tf.transform.rotation
            yaw_approx = float(q.z)  # enough to detect change
            pose = (float(tr.x), float(tr.y), yaw_approx)
            key = label
            prev = self._tf_pose.get(key)
            if prev is None:
                self._tf_pose[key] = pose
                self._tf_last_change[key] = time.monotonic()
            else:
                dx = abs(pose[0] - prev[0]) + abs(pose[1] - prev[1]) + abs(pose[2] - prev[2])
                if dx > 1e-4:
                    self._tf_pose[key] = pose
                    self._tf_last_change[key] = time.monotonic()

            if edge.dynamic and age > edge.stale_sec:
                rows.append((label, "STALE", f"age={age:.2f}s — {edge.hint}"))
                self._add_issue("ERROR", f"TF {label} stale age={age:.2f}s — {edge.hint}")
                continue

            if edge.dynamic and edge.frozen_sec > 0.0:
                # Only flag frozen after robot has been up long enough that
                # some motion OR continuous EKF republish should occur.
                # EKF republishes even when still — stamp should keep moving.
                # If stamp is fresh but translation never changes, that is OK
                # while parked; only warn if stamp itself stops (handled above).
                # Extra: if age is fine but transform never updated AND elapsed
                # is large, note as INFO-level via WARN once for map→odom only
                # when age is also elevated — already covered by stale.
                pass

            kind = "updating" if edge.dynamic else "static"
            frame_note = ""
            if edge.child == "laser_frame" or edge.parent == "laser_frame":
                pass
            rows.append((label, "HEALTHY", f"age={age:.2f}s  {kind}{frame_note}"))

        # If the robot pose in odom is moving but map→odom is frozen, SLAM is
        # lagging and the scan appears to ride/rotate with the bot in Fixed=map.
        odom_key = "odom -> base_footprint"
        map_key = "map -> odom"
        odom_chg = self._tf_last_change.get(odom_key)
        map_chg = self._tf_last_change.get(map_key)
        if odom_chg and map_chg and self._elapsed() > 15.0:
            now = time.monotonic()
            if (now - odom_chg) < 1.0 and (now - map_chg) > 3.0:
                self._add_issue(
                    "WARN",
                    "map→odom frozen while odom→base_footprint is moving — "
                    "SLAM correction lag; scan may rotate with the robot in RViz",
                )

        if self._scan_frame_id:
            rows.append(
                (
                    f"/scan.frame_id",
                    "HEALTHY"
                    if self._scan_frame_id == self._expected_scan_frame
                    else "FAILED",
                    f"'{self._scan_frame_id}' "
                    f"(expect '{self._expected_scan_frame}')",
                )
            )

        return rows

    def _eval_qos(self) -> None:
        now = time.monotonic()
        if now - self._last_qos_check < self._qos_period:
            return
        self._last_qos_check = now

        critical = {t.name for t in self._topic_specs} | {"/cmd_vel", "/cmd_vel_safe", "/tf", "/tf_static"}
        try:
            topics = self.get_topic_names_and_types()
        except Exception as exc:
            self._add_issue("WARN", f"QoS scan failed: {exc}")
            return

        for topic_name, _types in topics:
            if topic_name not in critical:
                continue
            try:
                pubs = self.get_publishers_info_by_topic(topic_name)
                subs = self.get_subscriptions_info_by_topic(topic_name)
            except Exception:
                continue
            if not pubs or not subs:
                continue
            for pub in pubs:
                for sub in subs:
                    # Skip our own monitor subscriptions creating noise
                    if "bringup_status" in (sub.node_name or ""):
                        continue
                    rel_bad = (
                        sub.qos_profile.reliability == ReliabilityPolicy.RELIABLE
                        and pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                    )
                    dur_bad = (
                        sub.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                        and pub.qos_profile.durability == DurabilityPolicy.VOLATILE
                    )
                    if rel_bad or dur_bad:
                        bits = []
                        if rel_bad:
                            bits.append("reliability RELIABLE sub vs BEST_EFFORT pub")
                        if dur_bad:
                            bits.append("durability TRANSIENT_LOCAL sub vs VOLATILE pub")
                        self._add_issue(
                            "WARN",
                            f"QoS {topic_name}: {' & '.join(bits)} "
                            f"({pub.node_name} → {sub.node_name})",
                        )

    def _overall(self, node_rows, topic_rows, tf_rows) -> str:
        states = [s for _, s, _ in node_rows + topic_rows + tf_rows]
        if any(s in ("FAILED", "MISSING") for s in states):
            return "FAILED"
        if any(s in ("STALE", "INACTIVE") for s in states):
            return "DEGRADED"
        if any(s == "STARTING" for s in states):
            return "STARTING"
        if any(i.level == "ERROR" for i in self._issues.values()):
            return "FAILED"
        if any(i.level == "WARN" for i in self._issues.values()):
            return "DEGRADED"
        return "OK"

    # ── render ───────────────────────────────────────────────────────────────

    def _color_state(self, state: str) -> str:
        c = STATE_COLORS.get(state, RESET)
        return f"{c}{BOLD}{state:<9}{RESET}"

    def _render(
        self,
        node_rows: List[Tuple[str, str, str]],
        topic_rows: List[Tuple[str, str, str]],
        tf_rows: List[Tuple[str, str, str]],
        overall: str,
    ) -> str:
        t = self._elapsed()
        lines: List[str] = []
        oc = STATE_COLORS.get(overall, RESET)
        lines.append(
            f"{BOLD}=== Ros_lidar_bot bringup ==={RESET}  "
            f"t={t:5.1f}s   overall: {oc}{BOLD}{overall}{RESET}"
        )
        lines.append(f"{DIM}{CYAN}NODES{RESET}")
        for name, state, detail in node_rows:
            lines.append(f"  {name:<28} {self._color_state(state)} {detail}")

        lines.append(f"{DIM}{CYAN}TOPICS{RESET}")
        for name, state, detail in topic_rows:
            lines.append(f"  {name:<28} {self._color_state(state)} {detail}")

        lines.append(f"{DIM}{CYAN}TF{RESET}")
        for name, state, detail in tf_rows:
            lines.append(f"  {name:<28} {self._color_state(state)} {detail}")

        if self._verbose_issues:
            lines.append(f"{DIM}{CYAN}ISSUES{RESET} (newest first)")
            items = sorted(self._issues.values(), key=lambda i: i.last_t, reverse=True)
            items = items[: self._max_issues]
            if not items:
                lines.append(f"  {GREEN}(none){RESET}")
            else:
                for issue in items:
                    lc = STATE_COLORS.get(issue.level, RESET)
                    cnt = f" x{issue.count}" if issue.count > 1 else ""
                    lines.append(f"  {lc}[{issue.level}]{RESET}{cnt} {issue.text}")

        lines.append(f"{DIM}Full logs: {self._latest_log_dir()}{RESET}")
        lines.append(
            f"{DIM}RViz Fixed Frame MUST be 'map' (not laser_frame). "
            f"verbose:=true for full spam.{RESET}"
        )
        return "\n".join(lines)

    def _tick(self) -> None:
        present = self._node_names_short()
        node_rows = self._eval_nodes(present)
        topic_rows = self._eval_topics()
        tf_rows = self._eval_tf()
        self._eval_qos()
        overall = self._overall(node_rows, topic_rows, tf_rows)
        board = self._render(node_rows, topic_rows, tf_rows, overall)
        self._write_board(board)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BringupStatusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
