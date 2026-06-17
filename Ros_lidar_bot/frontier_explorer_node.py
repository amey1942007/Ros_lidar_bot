#!/usr/bin/env python3

import math
from collections import deque
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformException, TransformListener

GridCell = Tuple[int, int]


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__("frontier_explorer")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("goal_timeout_sec", 90.0)
        self.declare_parameter("min_frontier_size", 8)
        self.declare_parameter("goal_interval_sec", 3.0)
        self.declare_parameter("min_goal_distance", 0.4)
        self.declare_parameter("max_goal_distance", 6.0)
        self.declare_parameter("goal_reached_radius", 0.45)
        self.declare_parameter("goal_blacklist_radius", 0.5)
        self.declare_parameter("blacklist_timeout_sec", 90.0)
        self.declare_parameter("size_weight", 1.5)
        self.declare_parameter("distance_weight", 1.0)
        self.declare_parameter("heading_weight", 0.8)
        self.declare_parameter("openness_weight", 0.4)
        self.declare_parameter("improvement_margin", 0.25)
        self.declare_parameter("goal_cooldown_sec", 2.0)
        self.declare_parameter("progress_timeout_sec", 18.0)
        self.declare_parameter("min_progress_delta", 0.15)
        self.declare_parameter("max_range_expansion_factor", 2.0)
        self.declare_parameter("no_frontier_done_ticks", 4)
        self.declare_parameter("wall_safe_distance", 0.5)
        self.declare_parameter("max_goal_distance_cap", 12.0)

        p = self.get_parameter
        self.map_topic = p("map_topic").value
        self.base_frame = p("base_frame").value
        self.map_frame = p("map_frame").value
        self.goal_timeout_sec = float(p("goal_timeout_sec").value)
        self.min_frontier_size = int(p("min_frontier_size").value)
        self.min_goal_distance = float(p("min_goal_distance").value)
        self.max_goal_distance = float(p("max_goal_distance").value)
        self.goal_reached_radius = float(p("goal_reached_radius").value)
        self.goal_blacklist_radius = float(p("goal_blacklist_radius").value)
        self.blacklist_timeout_sec = float(p("blacklist_timeout_sec").value)
        self.size_weight = float(p("size_weight").value)
        self.distance_weight = float(p("distance_weight").value)
        self.heading_weight = float(p("heading_weight").value)
        self.openness_weight = float(p("openness_weight").value)
        self.improvement_margin = float(p("improvement_margin").value)
        self.goal_cooldown_sec = float(p("goal_cooldown_sec").value)
        self.progress_timeout_sec = float(p("progress_timeout_sec").value)
        self.min_progress_delta = float(p("min_progress_delta").value)
        self.max_range_expansion_factor = float(p("max_range_expansion_factor").value)
        self.no_frontier_done_ticks = int(p("no_frontier_done_ticks").value)
        self.wall_safe_distance = float(p("wall_safe_distance").value)
        self.max_goal_distance_cap = float(p("max_goal_distance_cap").value)

        # Keep the config value as the floor so calibration can only raise it,
        # and relaxation tracks the current active value.
        self._cfg_min_frontier_size = self.min_frontier_size
        self._cfg_min_goal_distance = self.min_goal_distance
        self._cfg_max_goal_distance = self.max_goal_distance
        self._calibrated = False
        # Large-frontier threshold for standby queue — set properly in _calibrate_from_map
        self._queue_large_threshold: int = max(self.min_frontier_size * 3, 15)

        self.latest_map: Optional[OccupancyGrid] = None
        self.goal_handle = None
        self.goal_start_time = None
        self.goal_in_progress = False
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_score = -float("inf")
        self.failed_goals: Dict[Tuple[float, float], rclpy.time.Time] = {}
        # Track how many times each area has been blacklisted.
        # On repeat failures the blacklist radius doubles — prevents the robot
        # from endlessly returning to a genuinely unreachable zone.
        self._blacklist_hits:     Dict[Tuple[float, float], int]           = {}
        self._blacklist_timeouts: Dict[Tuple[float, float], float]         = {}
        # Permanently blocked map cells — enclosed unknown regions with ALL
        # sides surrounded by occupied cells.  These never expire; the robot
        # will never be able to explore them regardless of how long it waits.
        # _permanent_dead_zones  : centroid per region — used only for RViz markers
        # _perm_zone_cells       : exact cell footprint (int grid indices) used for
        #                          blacklist checks — precise shape, not a circle
        self._permanent_dead_zones: Set[Tuple[float, float]]               = set()
        self._perm_zone_cells:      Set[Tuple[int,   int  ]]               = set()
        self._map_res: float = 0.05   # set precisely in _calibrate_from_map
        # Phantom success counter — tracks how many times the robot "reached"
        # a goal (STATUS_SUCCEEDED) but the frontier cells near that position
        # still didn't clear.  This happens for permanently occluded areas
        # (e.g. behind solid furniture) where the lidar can never see through.
        # After _PHANTOM_THRESHOLD arrivals with no map change the area is
        # force-blacklisted with a 10-minute timeout.
        self._phantom_successes: Dict[Tuple[float, float], int] = {}
        self._PHANTOM_THRESHOLD = 2       # arrivals without clearing → blacklist
        self._PHANTOM_TIMEOUT   = 600.0   # 10 min — much longer than normal 300 s
        # Position of the last cancelled goal — used to enforce a minimum
        # distance for the next queued frontier so the robot can't immediately
        # pick something that's just outside the blacklist radius.
        self._last_cancel_xy: Optional[Tuple[float, float]] = None
        # Set True before cancel_goal_async() when the cancel is clean (frontier
        # already mapped by the lidar mid-trip).  Prevents _goal_result_cb from
        # blacklisting an area that is perfectly navigable.
        self._clean_cancel: bool = False
        # Monotonically increasing goal generation.  Captured at send time and
        # checked in all action callbacks — any callback whose generation no longer
        # matches _goal_gen is stale (a newer goal was already sent) and must not
        # modify shared state.  This prevents goal_in_progress, active_goal_xy,
        # active_goal_score, last_goal_end_time, and last_progress_time from being
        # overwritten by old callbacks firing after an immediate switch.
        self._goal_gen: int = 0
        self.last_goal_end_time = self.get_clock().now()
        self.last_distance_remaining: Optional[float] = None
        self.last_progress_time = self.get_clock().now()
        self._no_frontier_ticks = 0

        # ── Frontier queue — pre-computed candidates for immediate switching ──
        # Stores (position, score) pairs sorted best-first. Re-validated
        # (position still unknown, not blacklisted) before each use so stale
        # entries are silently skipped. Queue is refreshed every planning tick.
        self._frontier_queue: List[Tuple[Tuple[float, float], float]] = []
        self._queue_stamp: Optional[rclpy.time.Time] = None
        self._queue_max_age: float = 20.0    # seconds before queue is considered stale
        self._queue_size: int = 5            # how many candidates to keep

        # ── Safety stop signal ────────────────────────────────────────────────
        # True while safety_stop_node is blocking forward motion.
        # We switch to the next queued frontier after SAFETY_SWITCH_SEC of blocking.
        self._safety_blocked = False
        self._safety_blocked_since: Optional[rclpy.time.Time] = None
        self._SAFETY_SWITCH_SEC = 3.0        # seconds blocked before switching frontier

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client  = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._debug_pub  = self.create_publisher(MarkerArray, "/frontier_debug", 10)
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.create_subscription(Bool, "/safety_blocked", self._safety_cb, 10)
        self.create_timer(float(p("goal_interval_sec").value), self._tick)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        if not self._calibrated:
            self._calibrate_from_map(msg)

    def _safety_cb(self, msg: Bool) -> None:
        if msg.data:
            if self._safety_blocked_since is None:
                self._safety_blocked_since = self.get_clock().now()
            self._safety_blocked = True
        else:
            self._safety_blocked = False
            self._safety_blocked_since = None

    def _calibrate_from_map(self, msg: OccupancyGrid) -> None:
        """One-time calibration on first map — scales parameters to actual room size."""
        self._map_res = msg.info.resolution
        w_m = msg.info.width  * msg.info.resolution
        h_m = msg.info.height * msg.info.resolution
        larger_dim = max(w_m, h_m)

        # max_goal_distance: 50% of larger dimension, capped
        calibrated = min(larger_dim * 0.50, self.max_goal_distance_cap)
        if calibrated > self.max_goal_distance:
            self.max_goal_distance = calibrated

        # no_frontier_done_ticks: larger rooms need more patience.
        # Formula: 2 ticks per metre of the larger dimension, minimum from config.
        # 10 m room → 20 ticks, 15 m warehouse → 30 ticks, 25 m large hall → 50 ticks.
        scaled_ticks = int(larger_dim * 2.0)
        if scaled_ticks > self.no_frontier_done_ticks:
            self.no_frontier_done_ticks = scaled_ticks

        # Large-frontier threshold for the standby queue — scales with map area so
        # "large" means something relative to the room being explored.
        # Formula: 1.5 cells per 1000 map cells, floored at max(min_frontier_size*3, 15).
        map_cells = msg.info.width * msg.info.height
        self._queue_large_threshold = max(
            self.min_frontier_size * 3, 15,
            int(map_cells * 0.0015),
        )

        self._calibrated = True
        self.get_logger().info(
            f"[auto-calibrate] map {w_m:.0f}×{h_m:.0f} m → "
            f"max_goal_distance={self.max_goal_distance:.1f} m  "
            f"done_ticks={self.no_frontier_done_ticks}  "
            f"queue_large_threshold={self._queue_large_threshold} cells"
        )

    def _try_relax(self) -> None:
        """Progressive relaxation — called every tick that finds no frontier.

        Every 3 ticks without a frontier, one constraint is loosened.
        This is faster than waiting for sparse milestones and scales naturally
        with the number of ticks (which is already proportional to room size
        via auto-calibration of no_frontier_done_ticks).

          tick  3 : min_frontier_size − 1  (smaller clusters now count)
          tick  6 : min_frontier_size − 1  again + max range extended to cap
          tick  9 : min_frontier_size − 1  again + min_goal_distance halved
          tick 12+: every 3 ticks min_frontier_size reduced until floor (1)

        Relaxation is sticky — once loosened, parameters stay loose.
        """
        t = self._no_frontier_ticks
        changed: list[str] = []

        # Every 3 ticks: reduce min_frontier_size by 1 (floor: 1)
        if t > 0 and t % 3 == 0 and self.min_frontier_size > 1:
            self.min_frontier_size = max(1, self.min_frontier_size - 1)
            changed.append(f"min_frontier_size → {self.min_frontier_size}")

        # At tick 6: extend max search range to full cap
        if t == 6 and self.max_goal_distance < self.max_goal_distance_cap:
            self.max_goal_distance = self.max_goal_distance_cap
            changed.append(f"max_goal_distance → {self.max_goal_distance:.1f} m (cap)")

        # At tick 9: halve the minimum goal distance (accept near frontiers)
        if t == 9 and self.min_goal_distance > 0.15:
            self.min_goal_distance = max(0.15, self.min_goal_distance * 0.5)
            changed.append(f"min_goal_distance → {self.min_goal_distance:.2f} m")

        if changed:
            self.get_logger().info(
                f"[auto-relax tick {t}] {', '.join(changed)}"
            )

    def _tick(self) -> None:
        if self.latest_map is None:
            return

        if self.goal_in_progress:
            if self.goal_start_time is None:
                return
            elapsed = self.get_clock().now() - self.goal_start_time

            # ── Early-cancel: if the area we're heading to is already mapped ──
            # If there are no unknown cells near the active goal the frontier is
            # gone — no point completing the trip, pick a new frontier now.
            if self.active_goal_xy is not None and self.latest_map is not None:
                if self._frontier_vanished(self.active_goal_xy, self.latest_map):
                    self._clean_cancel = True   # area is fine — don't blacklist
                    self.goal_handle.cancel_goal_async()
                    self.goal_in_progress = False
                    tf = self._lookup_robot_pose()
                    if tf is not None:
                        rx = tf.transform.translation.x
                        ry = tf.transform.translation.y
                        nxt_xy, nxt_score = self._pop_queued_frontier(rx, ry)
                        if nxt_xy is not None:
                            self.get_logger().info(
                                "Active frontier mapped — switching to queued frontier"
                            )
                            self._send_goal(
                                self._build_goal_pose(nxt_xy[0], nxt_xy[1], rx, ry),
                                nxt_score,
                            )
                            return
                    # Queue empty — wait for next tick rescan (apply cooldown)
                    self.get_logger().info(
                        "Active frontier mapped, queue empty — rescanning next tick"
                    )
                    self.last_goal_end_time = self.get_clock().now()
                    return

            # ── Safety-triggered immediate switch ─────────────────────────────
            # If the safety stop has been blocking forward motion for
            # _SAFETY_SWITCH_SEC, don't wait for the full progress timeout.
            # Blacklist current position and jump to the next queued frontier.
            if (self._safety_blocked
                    and self._safety_blocked_since is not None
                    and (self.get_clock().now() - self._safety_blocked_since)
                        > Duration(seconds=self._SAFETY_SWITCH_SEC)):
                tf = self._lookup_robot_pose()
                if tf is not None:
                    rx = tf.transform.translation.x
                    ry = tf.transform.translation.y
                    ryaw = self._yaw_from_quaternion(tf.transform.rotation)
                    nxt_xy, nxt_score = self._pop_queued_frontier(rx, ry)
                    if nxt_xy is not None:
                        self.get_logger().info(
                            f"Safety blocked {self._SAFETY_SWITCH_SEC:.0f}s — "
                            f"switching to queued frontier {nxt_xy}"
                        )
                        if self.active_goal_xy is not None:
                            self._blacklist_goal(self.active_goal_xy)
                        self.goal_handle.cancel_goal_async()
                        self.goal_in_progress = False
                        self._safety_blocked_since = None
                        self._send_goal(
                            self._build_goal_pose(nxt_xy[0], nxt_xy[1], rx, ry),
                            nxt_score,
                        )
                        return

            if elapsed > Duration(seconds=self.goal_timeout_sec):
                self.get_logger().warn("Goal timeout — blacklisting, trying queued frontier")
                if self.active_goal_xy is not None:
                    self._blacklist_goal(self.active_goal_xy)
                self.goal_handle.cancel_goal_async()
                self.goal_in_progress = False
                tf = self._lookup_robot_pose()
                if tf is not None:
                    rx = tf.transform.translation.x
                    ry = tf.transform.translation.y
                    nxt_xy, nxt_score = self._pop_queued_frontier(rx, ry)
                    if nxt_xy is not None:
                        self._send_goal(
                            self._build_goal_pose(nxt_xy[0], nxt_xy[1], rx, ry),
                            nxt_score,
                        )
                        return
            elif (self.get_clock().now() - self.last_progress_time) > Duration(
                seconds=self.progress_timeout_sec
            ):
                self.get_logger().warn("No progress — blacklisting, trying next queued frontier")
                if self.active_goal_xy is not None:
                    self._blacklist_goal(self.active_goal_xy)
                self.goal_handle.cancel_goal_async()
                self.goal_in_progress = False
                # Immediately try next queued frontier instead of waiting for next tick
                tf = self._lookup_robot_pose()
                if tf is not None:
                    rx = tf.transform.translation.x
                    ry = tf.transform.translation.y
                    ryaw = self._yaw_from_quaternion(tf.transform.rotation)
                    nxt_xy, nxt_score = self._pop_queued_frontier(rx, ry)
                    if nxt_xy is not None:
                        self._send_goal(
                            self._build_goal_pose(nxt_xy[0], nxt_xy[1], rx, ry),
                            nxt_score,
                        )
                        return

            # ── Background queue refresh while navigating ─────────────────────
            # If the standby queue is stale (>20 s), rebuild it now so it's
            # ready for an instant switch if needed.  The active goal is not
            # changed — only the queue side-effect of _select_frontier_goal
            # is used.
            if self._queue_stamp is None or (
                self.get_clock().now() - self._queue_stamp
            ).nanoseconds / 1e9 > self._queue_max_age:
                tf = self._lookup_robot_pose()
                if tf is not None:
                    self._select_frontier_goal(
                        self.latest_map,
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        self._yaw_from_quaternion(tf.transform.rotation),
                    )
            return

        if (self.get_clock().now() - self.last_goal_end_time) < Duration(
            seconds=self.goal_cooldown_sec
        ):
            return

        transform = self._lookup_robot_pose()
        if transform is None:
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        robot_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        self._prune_failed_goals()

        if self.active_goal_xy is not None and math.hypot(
            self.active_goal_xy[0] - robot_x, self.active_goal_xy[1] - robot_y
        ) <= self.goal_reached_radius:
            self.active_goal_xy = None

        # ── Try queue first (already-scored candidates, re-validated fresh) ──
        # The queue stores the top-N large frontiers from the last planning tick.
        # Using Q1 directly avoids a full map rescan and responds faster.
        # Only skip to full rescan if the queue is empty/stale/all-invalid.
        target, score = self._pop_queued_frontier(robot_x, robot_y)
        if target is None:
            # Queue empty or all stale — run full frontier rescan
            target, score = self._select_frontier_goal(
                self.latest_map, robot_x, robot_y, robot_yaw
            )

        if target is None:
            self._no_frontier_ticks += 1
            self._try_relax()
            self._publish_debug_markers()   # keep blacklist zones visible while idle
            if self._no_frontier_ticks >= self.no_frontier_done_ticks:
                self.get_logger().info(
                    f"No frontiers for {self._no_frontier_ticks} ticks — exploration complete"
                )
            else:
                self.get_logger().info(
                    f"No valid frontier (tick {self._no_frontier_ticks}/{self.no_frontier_done_ticks})"
                )
            return

        self._no_frontier_ticks = 0

        if (
            self.active_goal_xy is not None
            and score < self.active_goal_score + self.improvement_margin
        ):
            return

        self._send_goal(
            self._build_goal_pose(target[0], target[1], robot_x, robot_y), score
        )
        self._publish_debug_markers()

    def _frontier_vanished(
        self, goal_xy: Tuple[float, float], map_msg: OccupancyGrid
    ) -> bool:
        """Return True if the area around goal_xy no longer has unknown cells.

        When the robot is navigating to a frontier and the lidar maps that
        area before the robot arrives, the frontier disappears from the map.
        Continuing to that location wastes time — cancel early and pick a
        new frontier instead.
        """
        res = map_msg.info.resolution
        ox  = map_msg.info.origin.position.x
        oy  = map_msg.info.origin.position.y
        w   = map_msg.info.width
        h   = map_msg.info.height
        data = map_msg.data

        cgx = int((goal_xy[0] - ox) / res)
        cgy = int((goal_xy[1] - oy) / res)
        radius = int(0.8 / res)          # check 0.8 m radius around goal

        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = cgx + dx, cgy + dy
                if 0 <= nx < w and 0 <= ny < h:
                    if data[ny * w + nx] == -1:   # unknown cell still exists
                        return False
        return True   # no unknown cells left — frontier is gone

    def _pop_queued_frontier(
        self, robot_x: float, robot_y: float
    ) -> Tuple[Optional[Tuple[float, float]], float]:
        """Return the next valid frontier from the queue, re-validating each entry.

        Validation before use (not at queue-build time) keeps decisions fresh:
        - Skip if area is already mapped (frontier_vanished)
        - Skip if blacklisted
        - Skip if now too close to robot
        - Queue is discarded entirely if older than _queue_max_age
        """
        if not self._frontier_queue or self.latest_map is None:
            return None, -float("inf")

        if self._queue_stamp is not None:
            age = (self.get_clock().now() - self._queue_stamp).nanoseconds / 1e9
            if age > self._queue_max_age:
                self._frontier_queue.clear()
                return None, -float("inf")

        # Minimum distance from the last cancellation point (fix 3).
        # Prevents immediately picking a frontier that's just outside the
        # blacklist radius of where the robot just got stuck.
        _MIN_FROM_CANCEL = self.goal_blacklist_radius * 2.0

        while self._frontier_queue:
            xy, _ = self._frontier_queue.pop(0)
            # Re-validate and re-score from current robot position
            if self._is_blacklisted(xy[0], xy[1]):
                continue
            if self._frontier_vanished(xy, self.latest_map):
                continue
            dist = math.hypot(xy[0] - robot_x, xy[1] - robot_y)
            if dist < self.min_goal_distance or dist > self.max_goal_distance_cap:
                continue
            # Minimum distance from last stuck point
            if self._last_cancel_xy is not None:
                dist_from_cancel = math.hypot(
                    xy[0] - self._last_cancel_xy[0],
                    xy[1] - self._last_cancel_xy[1],
                )
                if dist_from_cancel < _MIN_FROM_CANCEL:
                    continue
            # Re-score so the decision is fresh
            score = -dist / self.max_goal_distance
            return xy, score

        return None, -float("inf")

    def _lookup_robot_pose(self) -> Optional[TransformStamped]:
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None

    def _build_goal_pose(
        self, gx: float, gy: float, rx: float, ry: float
    ) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = gx
        pose.pose.position.y = gy
        pose.pose.position.z = 0.0
        pose.pose.orientation = self._yaw_to_quaternion(math.atan2(gy - ry, gx - rx))
        return pose

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _send_goal(self, target_pose: PoseStamped, score: float) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for navigate_to_pose action server")
            return
        self._goal_gen += 1
        gen = self._goal_gen          # captured — all callbacks for THIS goal use this value
        goal = NavigateToPose.Goal()
        goal.pose = target_pose
        future = self.nav_client.send_goal_async(
            goal, feedback_callback=lambda fb: self._feedback_cb(fb, gen)
        )
        future.add_done_callback(lambda f: self._goal_response_cb(f, gen))
        self.goal_in_progress = True
        self.goal_start_time = self.get_clock().now()
        self.active_goal_xy = (target_pose.pose.position.x, target_pose.pose.position.y)
        self.active_goal_score = score
        self.last_distance_remaining = None
        self.last_progress_time = self.get_clock().now()
        self.get_logger().info(
            f"Frontier goal: ({target_pose.pose.position.x:.2f}, "
            f"{target_pose.pose.position.y:.2f}) score={score:.3f}"
        )

    def _goal_response_cb(self, future, gen: int) -> None:
        if gen != self._goal_gen:
            return   # stale — a newer goal was already sent, ignore this accept/reject
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by nav2")
            self.goal_in_progress = False
            self.active_goal_xy = None
            self.active_goal_score = -float("inf")
            self.last_goal_end_time = self.get_clock().now()
            return
        self.goal_handle.get_result_async().add_done_callback(
            lambda f: self._goal_result_cb(f, gen)
        )

    def _goal_result_cb(self, future, gen: int) -> None:
        if gen != self._goal_gen:
            return   # stale — a newer goal is already in flight, don't touch shared state
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            pos = self.active_goal_xy
            # Phantom-success check: did the frontier actually clear?
            # If not, the area is permanently occluded (e.g. behind furniture).
            # Track arrivals and force-blacklist after _PHANTOM_THRESHOLD repeats.
            if pos is not None and self.latest_map is not None:
                if not self._frontier_vanished(pos, self.latest_map):
                    # Frontier still there despite "reaching" it
                    merge_r2 = (self.goal_blacklist_radius * 1.5) ** 2
                    key = min(self._phantom_successes.keys() or [pos],
                              key=lambda k: (k[0]-pos[0])**2 + (k[1]-pos[1])**2)
                    if ((key[0]-pos[0])**2+(key[1]-pos[1])**2) > merge_r2:
                        key = pos
                    count = self._phantom_successes.get(key, 0) + 1
                    self._phantom_successes[key] = count
                    self.get_logger().info(
                        f"Frontier reached but not cleared ({count}/{self._PHANTOM_THRESHOLD})"
                        " — permanent occlusion candidate"
                    )
                    if count >= self._PHANTOM_THRESHOLD:
                        old_t = self.blacklist_timeout_sec
                        self.blacklist_timeout_sec = self._PHANTOM_TIMEOUT
                        self._blacklist_goal(pos)
                        self.blacklist_timeout_sec = old_t
                        self._blacklist_hits[pos] = 10   # force max radius
                        self._phantom_successes.pop(key, None)
                        self.get_logger().warn(
                            f"Permanently occluded area at {pos} — "
                            f"blacklisted 10 min, radius {self.goal_blacklist_radius*2:.2f} m"
                        )
                else:
                    # Frontier cleared — genuine success, reset phantom count
                    self._phantom_successes.pop(pos, None)
            self.get_logger().info("Frontier goal reached")
            self.active_goal_xy = None
            self.active_goal_score = -float("inf")
            self._last_cancel_xy = None   # success — lift the min-distance restriction
        else:
            pos = self.active_goal_xy
            if pos is not None:
                if status == GoalStatus.STATUS_ABORTED:
                    # Nav2 aborted — could be a genuine unreachable zone OR a
                    # temporary costmap-resize failure (SLAM expanding the map).
                    # Don't dead-zone immediately on the FIRST abort; use the
                    # adaptive hit counter instead:
                    #   1st abort → standard blacklist (300 s, 0.8 m)
                    #   2nd abort at same area → dead zone (600 s, 1.6 m)
                    # This prevents costmap-resize false positives from
                    # permanently marking navigable areas.
                    self._blacklist_goal(pos)
                    hits = self._blacklist_hits.get(pos, 1)
                    if hits >= 2:
                        # Genuinely unreachable — escalate to dead zone
                        self._blacklist_hits[pos] = 10
                        self._blacklist_timeouts[pos] = self._PHANTOM_TIMEOUT
                        self.get_logger().warn(
                            f"DEAD ZONE at {pos} — aborted {hits}× "
                            f"(radius {self.goal_blacklist_radius*2:.2f} m, 10 min)"
                        )
                    else:
                        self.get_logger().warn(
                            f"Nav2 aborted at {pos} (1st time — standard blacklist 300s; "
                            "may be costmap-resize false positive)"
                        )
                # STATUS_CANCELLED always means WE called cancel_goal_async().
                # We already call _blacklist_goal() explicitly BEFORE cancelling
                # in every timeout/safety case, so NEVER blacklist here.
                # By the time this callback fires, active_goal_xy may already
                # point to the next goal — blacklisting pos here would incorrectly
                # penalise a valid frontier we just chose.
                elif self._clean_cancel:
                    self.get_logger().info("Clean cancel — frontier mapped mid-trip")
                else:
                    self.get_logger().info(
                        f"Goal cancelled (status {status}) — already blacklisted before cancel"
                    )
                self.active_goal_xy = None
            self.active_goal_score = -float("inf")
        self.goal_in_progress = False
        self.last_goal_end_time = self.get_clock().now()
        self.last_distance_remaining = None
        self._clean_cancel = False   # reset for next goal

    def _feedback_cb(self, feedback_msg, gen: int) -> None:
        if gen != self._goal_gen:
            return   # stale feedback for an old goal — don't update progress timer
        remaining = float(getattr(feedback_msg.feedback, "distance_remaining", 0.0))
        if self.last_distance_remaining is None:
            self.last_distance_remaining = remaining
            self.last_progress_time = self.get_clock().now()
            return
        if (self.last_distance_remaining - remaining) > self.min_progress_delta:
            self.last_distance_remaining = remaining
            self.last_progress_time = self.get_clock().now()

    def _select_frontier_goal(
        self,
        map_msg: OccupancyGrid,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
    ) -> Tuple[Optional[Tuple[float, float]], float]:
        width = map_msg.info.width
        height = map_msg.info.height
        res = map_msg.info.resolution
        ox = map_msg.info.origin.position.x
        oy = map_msg.info.origin.position.y
        data = map_msg.data

        frontiers = self._extract_frontiers(data, width, height, map_msg)
        if not frontiers:
            self.get_logger().debug("No frontier cells found in map")
            return None, -float("inf")

        self.get_logger().debug(
            f"Found {len(frontiers)} frontier clusters "
            f"(sizes: {sorted((len(f) for f in frontiers), reverse=True)[:5]})"
        )
        # collect_all=True on the first call so the queue gets the top-N
        # candidates for immediate switching when blocked.
        result = self._score_frontiers(
            frontiers, data, width, height, res, ox, oy,
            robot_x, robot_y, robot_yaw, self.max_goal_distance,
            collect_all=True,
        )
        if result[0] is not None:
            return result

        # Expand search range once when nothing is reachable in normal range.
        expanded = min(
            self.max_goal_distance * self.max_range_expansion_factor,
            self.max_goal_distance_cap,
        )
        return self._score_frontiers(
            frontiers, data, width, height, res, ox, oy,
            robot_x, robot_y, robot_yaw, expanded,
            collect_all=True,
        )

    def _score_frontiers(
        self,
        frontiers: List[List[GridCell]],
        data,
        width: int,
        height: int,
        res: float,
        ox: float,
        oy: float,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        max_dist: float,
        collect_all: bool = False,
    ) -> Tuple[Optional[Tuple[float, float]], float]:
        """Score frontiers and return the best one.

        If collect_all=True, also stores all valid (position, score) pairs
        sorted best-first into self._frontier_queue for immediate switching.
        """
        max_size = max(len(f) for f in frontiers)
        best_goal: Optional[Tuple[float, float]] = None
        best_score = -float("inf")
        n_small = n_no_target = n_blacklist = n_dist = n_edge = 0
        all_valid: List[Tuple[Tuple[float, float], float]] = [] if collect_all else []

        # Minimum cells from map boundary — prevents worldToMap failures and
        # wall collisions. Converts the wall_safe_distance param to grid cells.
        edge_margin = max(1, int(self.wall_safe_distance / res))

        for frontier in frontiers:
            if len(frontier) < self.min_frontier_size:
                n_small += 1
                continue
            goal = self._frontier_target(frontier, data, width, height, res, ox, oy)
            if goal is None:
                n_no_target += 1
                continue
            cx, cy = goal

            # Reject goals at or near the map boundary (causes worldToMap failures
            # and drives the robot into walls on hardware).
            gx_cell = int((cx - ox) / res)
            gy_cell = int((cy - oy) / res)
            if not (edge_margin <= gx_cell < width - edge_margin
                    and edge_margin <= gy_cell < height - edge_margin):
                n_edge += 1
                continue

            if self._is_blacklisted(cx, cy):
                n_blacklist += 1
                continue

            dist = math.hypot(cx - robot_x, cy - robot_y)
            if dist < self.min_goal_distance or dist > max_dist:
                n_dist += 1
                continue

            heading_error = abs(
                self._normalize_angle(math.atan2(cy - robot_y, cx - robot_x) - robot_yaw)
            )
            openness_term = self._openness_score(
                cx, cy, data, width, height, res, ox, oy
            )

            # Hard floor on TARGET clearance: if < 15 % of the cells around the
            # target point are free, the robot would be wedged against an obstacle
            # to reach it (e.g. directly beside a bookshelf panel).  These are
            # physically unreachable — skip rather than send the robot there.
            # Note: this is tighter than the old 0.25 floor that was removed;
            # 0.15 only rejects extreme cases like solid-furniture occlusions.
            if openness_term < 0.15:
                n_edge += 1   # reuse counter — logged as 'edge' in debug output
                continue

            # openness_term also contributes to the score (not just a filter) so
            # open-area frontiers are preferred over tight-but-valid ones.

            size_term     = len(frontier) / float(max_size)
            distance_term = dist / max_dist
            heading_term  = heading_error / math.pi
            frontier_size = len(frontier)

            score = (
                self.size_weight    * size_term
                - self.distance_weight * distance_term
                - self.heading_weight  * heading_term
                + self.openness_weight * openness_term
            )
            if score > best_score:
                best_score = score
                best_goal  = (cx, cy)
            if collect_all:
                # Store size alongside so queue can filter by it
                all_valid.append(((cx, cy), score, frontier_size, dist))

        # ── Far-frontier bonus ────────────────────────────────────────────────
        # When ALL valid frontiers are beyond the "near zone" (35 % of
        # max_dist) the robot has no close options — distance should matter
        # less.  Large frontiers are worth the trip so they get the biggest
        # bonus, making them rise above small far-away clusters in score.
        if all_valid:
            near_zone   = max_dist * 0.35
            min_dist_av = min(d for _, _, _, d in all_valid)
            if min_dist_av > near_zone:
                # All frontiers are far — recompute scores with reduced penalty
                new_best_score = -float("inf")
                new_best_goal  = None
                rescored = []
                for xy, old_score, fsize, d in all_valid:
                    # Bonus: up to 0.4 reduction on distance term, scaled by
                    # size_ratio so large frontiers benefit more.
                    size_r    = fsize / float(max_size) if max_size > 0 else 0.5
                    far_bonus = min(0.40, size_r * 0.45)
                    dt        = max(0.0, d / max_dist - far_bonus)
                    ht        = abs(self._normalize_angle(
                        math.atan2(xy[1] - robot_y, xy[0] - robot_x) - robot_yaw
                    )) / math.pi
                    st        = fsize / float(max_size) if max_size > 0 else 0.5
                    ot        = self._openness_score(
                        xy[0], xy[1], data, width, height, res, ox, oy
                    )
                    sc = (self.size_weight    * st
                          - self.distance_weight * dt
                          - self.heading_weight  * ht
                          + self.openness_weight * ot)
                    rescored.append((xy, sc, fsize, d))
                    if sc > new_best_score:
                        new_best_score = sc
                        new_best_goal  = xy
                all_valid   = rescored
                best_goal   = new_best_goal
                best_score  = new_best_score

        if collect_all and all_valid:
            all_valid.sort(key=lambda x: x[1], reverse=True)

            # ── Geographically diverse queue of LARGE frontiers only ──────────
            # Small frontiers get mapped naturally as the robot moves nearby.
            # Threshold is map-relative (set in _calibrate_from_map).
            queue_large_threshold = self._queue_large_threshold
            min_spread = self.goal_blacklist_radius * 2.5
            diverse: List[Tuple[Tuple[float, float], float]] = []

            for xy, sc, fsize, _ in all_valid[1:]:
                if len(diverse) >= self._queue_size:
                    break
                if fsize < queue_large_threshold:
                    continue   # too small — robot will map it in passing
                too_close = any(
                    math.hypot(xy[0] - q[0][0], xy[1] - q[0][1]) < min_spread
                    for q in diverse
                )
                if not too_close:
                    diverse.append((xy, sc))

            # Fallback: if no large frontiers left, accept any size
            # (late-stage exploration with only tiny pockets remaining)
            if not diverse:
                seen: Set[Tuple[float, float]] = set()
                for xy, sc, _, _ in all_valid[1:]:
                    if len(diverse) >= self._queue_size:
                        break
                    if xy not in seen:
                        diverse.append((xy, sc))
                        seen.add(xy)

            self._frontier_queue = diverse
            self._queue_stamp    = self.get_clock().now()

        self.get_logger().debug(
            f"score_frontiers(max_dist={max_dist:.1f}): total={len(frontiers)} "
            f"too_small={n_small} no_target={n_no_target} "
            f"edge={n_edge} blacklisted={n_blacklist} dist_fail={n_dist} "
            f"queued={len(self._frontier_queue)} "
            f"result={'found' if best_goal else 'none'}"
        )
        return best_goal, best_score

    @staticmethod
    def _flood_fill_reachable(free_or_unknown: np.ndarray) -> np.ndarray:
        """Return a boolean mask of cells reachable from the map boundary.

        Uses 4-connectivity BFS seeded from all boundary cells that are
        free or unknown.  Any unknown cell NOT reachable is completely
        enclosed by occupied cells and can never be explored — the lidar
        cannot see through solid obstacles into those voids.
        """
        h, w = free_or_unknown.shape
        reachable = np.zeros((h, w), dtype=bool)
        q: deque = deque()

        def _seed(y: int, x: int) -> None:
            if free_or_unknown[y, x] and not reachable[y, x]:
                reachable[y, x] = True
                q.append((y, x))

        # Seed from all four border rows/columns
        for x in range(w):
            _seed(0, x); _seed(h - 1, x)
        for y in range(h):
            _seed(y, 0); _seed(y, w - 1)

        while q:
            cy, cx = q.popleft()
            for ny, nx in ((cy-1, cx), (cy+1, cx), (cy, cx-1), (cy, cx+1)):
                if 0 <= ny < h and 0 <= nx < w:
                    if free_or_unknown[ny, nx] and not reachable[ny, nx]:
                        reachable[ny, nx] = True
                        q.append((ny, nx))
        return reachable

    @staticmethod
    def _label_enclosed(enclosed: np.ndarray) -> Tuple[np.ndarray, int]:
        """Connected-component labelling of enclosed unknown cells (4-connectivity).
        Returns (label_array, num_regions).  Region IDs start at 1."""
        h, w       = enclosed.shape
        labels     = np.zeros((h, w), dtype=np.int32)
        region_id  = 0
        q: deque   = deque()
        for sy in range(h):
            for sx in range(w):
                if enclosed[sy, sx] and labels[sy, sx] == 0:
                    region_id += 1
                    labels[sy, sx] = region_id
                    q.append((sy, sx))
                    while q:
                        cy, cx = q.popleft()
                        for ny, nx in ((cy-1,cx),(cy+1,cx),(cy,cx-1),(cy,cx+1)):
                            if 0<=ny<h and 0<=nx<w and enclosed[ny,nx] and labels[ny,nx]==0:
                                labels[ny, nx] = region_id
                                q.append((ny, nx))
        return labels, region_id

    def _extract_frontiers(
        self, data, width: int, height: int,
        map_msg: Optional[OccupancyGrid] = None
    ) -> List[List[GridCell]]:
        """Vectorized frontier extraction via numpy; BFS clustering in Python."""
        grid = np.asarray(data, dtype=np.int8).reshape((height, width))
        unknown = grid == -1
        free = grid == 0

        # ── Enclosed-region elimination ───────────────────────────────────────
        # Unknown cells completely surrounded by occupied cells (e.g. the void
        # inside a sofa or behind solid shelves) will never be reachable by the
        # lidar.  Flood-fill from the map boundary: any unknown cell that cannot
        # be reached through free/unknown space from the edge is enclosed.
        # Treat those cells as occupied so no frontier ever forms there.
        traversable = free | unknown
        reachable   = self._flood_fill_reachable(traversable)
        enclosed    = unknown & ~reachable
        if enclosed.any():
            n_enclosed = int(enclosed.sum())
            grid = grid.copy()
            grid[enclosed] = 100   # virtually occupied — won't form frontiers
            unknown = grid == -1
            free    = grid == 0

            # Register enclosed region centroids as PERMANENT dead zones.
            # These have no timeout — the map geometry means they can never
            # be explored no matter how long the robot waits.
            res = map_msg.info.resolution       if map_msg is not None else 0.05
            ox  = map_msg.info.origin.position.x if map_msg is not None else 0.0
            oy  = map_msg.info.origin.position.y if map_msg is not None else 0.0
            # Use connected-component centroids so each enclosed pocket gets one marker
            labeled, n_regions = self._label_enclosed(enclosed)
            merge_r2 = (self.goal_blacklist_radius * 2.0) ** 2
            # 1-cell buffer so frontier targets right on the enclosed boundary
            # are also caught by the exact geometry check.
            buf = 1
            for region_id in range(1, n_regions + 1):
                ys_r, xs_r = np.where(labeled == region_id)
                if len(xs_r) < 3:
                    continue   # single-cell noise, skip
                cx_w = float(ox + (xs_r.mean() + 0.5) * res)
                cy_w = float(oy + (ys_r.mean() + 0.5) * res)
                # Skip if this region is already covered by an existing entry
                if any((cx_w - px)**2 + (cy_w - py)**2 <= merge_r2
                       for (px, py) in self._permanent_dead_zones):
                    continue
                # Centroid for RViz marker
                self._permanent_dead_zones.add((round(cx_w, 2), round(cy_w, 2)))
                # Exact cell footprint (+ buffer) stored as world-coordinate integers
                # (units = 1 map cell = res metres).  World coords are stable even
                # as SLAM expands the grid and shifts its origin, so these indices
                # remain valid for the whole session.  O(1) set lookup in _is_blacklisted.
                for gx_c, gy_c in zip(xs_r.tolist(), ys_r.tolist()):
                    wx_c = ox + (gx_c + 0.5) * res
                    wy_c = oy + (gy_c + 0.5) * res
                    for dx in range(-buf, buf + 1):
                        for dy in range(-buf, buf + 1):
                            self._perm_zone_cells.add((
                                round((wx_c + dx * res) / res),
                                round((wy_c + dy * res) / res),
                            ))

            self.get_logger().debug(
                f"Enclosed-region fill: {n_enclosed} cells in "
                f"{n_regions} pocket(s) → permanent dead zones"
            )

        # 8-connectivity: unknown cells adjacent (incl. diagonal) to any free cell
        fu = np.roll(free, 1, axis=0)
        fd = np.roll(free, -1, axis=0)
        fl = np.roll(free, 1, axis=1)
        fr = np.roll(free, -1, axis=1)
        has_free_nbr = (
            fu | fd | fl | fr
            | np.roll(fu, 1, axis=1)   # up-left
            | np.roll(fu, -1, axis=1)  # up-right
            | np.roll(fd, 1, axis=1)   # down-left
            | np.roll(fd, -1, axis=1)  # down-right
        )
        # np.roll wraps edges — zero out boundary artifacts
        has_free_nbr[0, :] = False
        has_free_nbr[-1, :] = False
        has_free_nbr[:, 0] = False
        has_free_nbr[:, -1] = False

        ys, xs = np.where(unknown & has_free_nbr)
        if len(xs) == 0:
            return []

        frontier_cells: Set[GridCell] = set(zip(xs.tolist(), ys.tolist()))
        visited: Set[GridCell] = set()
        clusters: List[List[GridCell]] = []
        for cell in frontier_cells:
            if cell not in visited:
                cluster = self._grow_frontier_cluster(cell, frontier_cells, visited)
                if cluster:
                    clusters.append(cluster)
        return clusters

    @staticmethod
    def _grow_frontier_cluster(
        start: GridCell, frontier_cells: Set[GridCell], visited: Set[GridCell]
    ) -> List[GridCell]:
        q = deque([start])
        visited.add(start)
        cluster: List[GridCell] = []
        while q:
            cx, cy = q.popleft()
            cluster.append((cx, cy))
            for nx in (cx - 1, cx, cx + 1):
                for ny in (cy - 1, cy, cy + 1):
                    if (nx, ny) != (cx, cy):
                        n = (nx, ny)
                        if n in frontier_cells and n not in visited:
                            visited.add(n)
                            q.append(n)
        return cluster

    @staticmethod
    def _frontier_target(
        frontier_cells: List[GridCell],
        data,
        width: int,
        height: int,
        res: float,
        ox: float,
        oy: float,
    ) -> Optional[Tuple[float, float]]:
        n = len(frontier_cells)
        cgx = round(sum(x for x, _ in frontier_cells) / n)
        cgy = round(sum(y for _, y in frontier_cells) / n)

        # Free cells adjacent (8-connectivity) to any frontier cell
        candidates: Set[GridCell] = set()
        for fx, fy in frontier_cells:
            for nx in (fx - 1, fx, fx + 1):
                for ny in (fy - 1, fy, fy + 1):
                    if (nx, ny) != (fx, fy) and 0 <= nx < width and 0 <= ny < height:
                        if data[ny * width + nx] == 0:
                            candidates.add((nx, ny))

        if not candidates:
            return None

        # Nearest candidate to grid centroid; integer d² avoids sqrt
        best_gx, best_gy = min(
            candidates, key=lambda c: (c[0] - cgx) ** 2 + (c[1] - cgy) ** 2
        )
        return ox + (best_gx + 0.5) * res, oy + (best_gy + 0.5) * res

    @staticmethod
    def _openness_score(
        wx: float,
        wy: float,
        data,
        width: int,
        height: int,
        res: float,
        ox: float,
        oy: float,
        radius: int = 3,
    ) -> float:
        """Fraction of cells within `radius` grid-cells of target that are free."""
        gx = int((wx - ox) / res)
        gy = int((wy - oy) / res)
        total = free_count = 0
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < width and 0 <= ny < height:
                    total += 1
                    if data[ny * width + nx] == 0:
                        free_count += 1
        return free_count / max(total, 1)

    @staticmethod
    def _yaw_from_quaternion(quat: Quaternion) -> float:
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _blacklist_goal(self, xy: Tuple[float, float]) -> None:
        """Write xy to the blacklist and track how many times this area has failed.

        On the first failure the standard radius applies.
        On every subsequent failure within merge_r the effective blacklist radius
        doubles — the robot learns that this zone is genuinely unreachable and
        gives it an increasingly wide berth.
        Also records _last_cancel_xy so _pop_queued_frontier can enforce the
        minimum-distance rule on the next queued candidate.
        """
        merge_r2 = (self.goal_blacklist_radius * 1.5) ** 2

        # Find closest existing blacklist entry (might be the same area)
        closest_key = None
        closest_d2  = float("inf")
        for bk in self.failed_goals:
            d2 = (xy[0] - bk[0]) ** 2 + (xy[1] - bk[1]) ** 2
            if d2 < closest_d2:
                closest_d2, closest_key = d2, bk

        if closest_key is not None and closest_d2 <= merge_r2:
            hits = self._blacklist_hits.get(closest_key, 1) + 1
            self._blacklist_hits[closest_key] = hits
            self.failed_goals[closest_key] = self.get_clock().now()
            if hits > 1:
                # Scale timeout with hits so robot stays away longer each time:
                # hit 2 → 600 s, hit 3 → 1200 s, hit 4+ → 2400 s (capped)
                scaled = min(self.blacklist_timeout_sec * (2 ** (hits - 1)), 2400.0)
                self._blacklist_timeouts[closest_key] = scaled
                self.get_logger().warn(
                    f"Area {closest_key} blacklisted {hits}× — "
                    f"radius {self.goal_blacklist_radius * 2.0:.2f} m, "
                    f"timeout {scaled:.0f} s"
                )
        else:
            self._blacklist_hits[xy] = 1
            self._blacklist_timeouts[xy] = self.blacklist_timeout_sec
            self.failed_goals[xy] = self.get_clock().now()

        self._last_cancel_xy = xy

    def _is_blacklisted(self, gx: float, gy: float) -> bool:
        """Return True if (gx, gy) falls inside any blacklisted zone.

        Zones that have been hit more than once use double the normal radius
        so the robot gives a wider berth to repeatedly unreachable areas.
        """
        base_r  = self.goal_blacklist_radius
        base_r2 = base_r ** 2
        dbl_r2  = (base_r * 2.0) ** 2

        for (bx, by), stamp in self.failed_goals.items():
            hits = self._blacklist_hits.get((bx, by), 1)
            r2   = dbl_r2 if hits > 1 else base_r2
            if (gx - bx) ** 2 + (gy - by) ** 2 <= r2:
                return True
        # Check permanent dead zones using exact cell geometry (not a circle).
        # Convert query point to the same world-coordinate integer units used
        # when the cells were stored — O(1) set lookup per query.
        if self._perm_zone_cells:
            qi = round(gx / self._map_res)
            qj = round(gy / self._map_res)
            if (qi, qj) in self._perm_zone_cells:
                return True
        return False

    def _prune_failed_goals(self) -> None:
        now  = self.get_clock().now()
        keep = {
            goal: stamp
            for goal, stamp in self.failed_goals.items()
            if (now - stamp) <= Duration(
                seconds=self._blacklist_timeouts.get(goal, self.blacklist_timeout_sec)
            )
        }
        for gone in set(self.failed_goals) - set(keep):
            self._blacklist_hits.pop(gone, None)
            self._blacklist_timeouts.pop(gone, None)
        self.failed_goals = keep


    def _publish_debug_markers(self) -> None:
        """Publish /frontier_debug MarkerArray for RViz2 debugging.

        Shows:
          - Blacklisted zones   — red/dark-red cylinders, label with countdown
          - Permanent dead zones — black cylinders, label 'PERM'
          - Top-5 queued frontiers — numbered green spheres + score labels
        Add a MarkerArray display in RViz2 → topic /frontier_debug.
        """
        arr  = MarkerArray()
        now  = self.get_clock().now().to_msg()
        mid  = 0
        base = self.goal_blacklist_radius

        # Delete all previous markers first so stale ones never linger
        clr        = Marker()
        clr.action = Marker.DELETEALL
        clr.header.frame_id = self.map_frame
        clr.header.stamp    = now
        arr.markers.append(clr)

        # ── Blacklisted zones ─────────────────────────────────────────────────
        for (bx, by), stamp in self.failed_goals.items():
            hits    = self._blacklist_hits.get((bx, by), 1)
            radius  = base * 2.0 if hits > 1 else base
            timeout = self._blacklist_timeouts.get((bx, by), self.blacklist_timeout_sec)
            elapsed = (self.get_clock().now() - stamp).nanoseconds / 1e9
            remaining = max(0.0, timeout - elapsed)

            # Circle (cylinder) — red for standard, dark-red for dead zone (hits>1)
            m           = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp    = now
            m.ns              = "blacklist"
            m.id              = mid; mid += 1
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position.x = bx
            m.pose.position.y = by
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = radius * 2.0
            m.scale.y = radius * 2.0
            m.scale.z = 0.05
            m.color   = ColorRGBA(r=0.8 if hits == 1 else 0.5,
                                  g=0.0, b=0.0,
                                  a=0.35)
            m.lifetime.sec = 0
            arr.markers.append(m)

            # Text label showing hit count and time remaining
            t           = Marker()
            t.header    = m.header
            t.ns        = "blacklist_label"
            t.id        = mid; mid += 1
            t.type      = Marker.TEXT_VIEW_FACING
            t.action    = Marker.ADD
            t.pose.position.x = bx
            t.pose.position.y = by
            t.pose.position.z = 0.35
            t.pose.orientation.w = 1.0
            t.scale.z   = 0.18
            t.text      = f"BL×{hits}  {remaining:.0f}s"
            t.color     = ColorRGBA(r=1.0, g=0.4 if hits == 1 else 0.1,
                                    b=0.1, a=1.0)
            t.lifetime.sec = 0
            arr.markers.append(t)

        # ── Permanent dead zones (black, no timeout label) ───────────────────
        perm_r = self.goal_blacklist_radius * 2.0
        for (px, py) in self._permanent_dead_zones:
            pm           = Marker()
            pm.header.frame_id = self.map_frame
            pm.header.stamp    = now
            pm.ns              = "perm_dead"
            pm.id              = mid; mid += 1
            pm.type            = Marker.CYLINDER
            pm.action          = Marker.ADD
            pm.pose.position.x = px
            pm.pose.position.y = py
            pm.pose.position.z = 0.05
            pm.pose.orientation.w = 1.0
            pm.scale.x = pm.scale.y = perm_r * 2.0
            pm.scale.z = 0.05
            pm.color   = ColorRGBA(r=0.1, g=0.1, b=0.1, a=0.45)
            pm.lifetime.sec = 0
            arr.markers.append(pm)
            pt           = Marker()
            pt.header    = pm.header
            pt.ns        = "perm_dead_label"
            pt.id        = mid; mid += 1
            pt.type      = Marker.TEXT_VIEW_FACING
            pt.action    = Marker.ADD
            pt.pose.position.x = px
            pt.pose.position.y = py
            pt.pose.position.z = 0.35
            pt.pose.orientation.w = 1.0
            pt.scale.z   = 0.16
            pt.text      = "PERM"
            pt.color     = ColorRGBA(r=0.8, g=0.8, b=0.8, a=1.0)
            pt.lifetime.sec = 0
            arr.markers.append(pt)

        # ── Top-5 queued frontiers ────────────────────────────────────────────
        for rank, (xy, score) in enumerate(self._frontier_queue[:5]):
            fx, fy = xy

            s           = Marker()
            s.header.frame_id = self.map_frame
            s.header.stamp    = now
            s.ns              = "frontier_queue"
            s.id              = mid; mid += 1
            s.type            = Marker.SPHERE
            s.action          = Marker.ADD
            s.pose.position.x = fx
            s.pose.position.y = fy
            s.pose.position.z = 0.25
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = 0.25
            # Colour gradient: #1 bright green → #5 yellow-green
            g = 0.9 - rank * 0.1
            s.color   = ColorRGBA(r=0.1, g=g, b=0.2, a=0.85)
            s.lifetime.sec = 0
            arr.markers.append(s)

            lbl           = Marker()
            lbl.header    = s.header
            lbl.ns        = "frontier_queue_label"
            lbl.id        = mid; mid += 1
            lbl.type      = Marker.TEXT_VIEW_FACING
            lbl.action    = Marker.ADD
            lbl.pose.position.x = fx
            lbl.pose.position.y = fy
            lbl.pose.position.z = 0.50
            lbl.pose.orientation.w = 1.0
            lbl.scale.z   = 0.18
            lbl.text      = f"Q{rank+1}  {score:.2f}"
            lbl.color     = ColorRGBA(r=0.9, g=1.0, b=0.9, a=1.0)
            lbl.lifetime.sec = 0
            arr.markers.append(lbl)

        if arr.markers:
            self._debug_pub.publish(arr)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
