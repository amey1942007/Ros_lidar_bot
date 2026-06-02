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
from std_msgs.msg import Bool
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
        self._blacklist_hits: Dict[Tuple[float, float], int] = {}
        # Position of the last cancelled goal — used to enforce a minimum
        # distance for the next queued frontier so the robot can't immediately
        # pick something that's just outside the blacklist radius.
        self._last_cancel_xy: Optional[Tuple[float, float]] = None
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
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
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

        self._calibrated = True
        self.get_logger().info(
            f"[auto-calibrate] map {w_m:.0f}×{h_m:.0f} m → "
            f"max_goal_distance={self.max_goal_distance:.1f} m  "
            f"done_ticks={self.no_frontier_done_ticks}"
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
                    self.get_logger().info(
                        "Active frontier already mapped — cancelling early, picking new goal"
                    )
                    self.goal_handle.cancel_goal_async()
                    self.goal_in_progress = False
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
                self.get_logger().warn("Goal timeout, cancelling")
                if self.active_goal_xy is not None:
                    self._blacklist_goal(self.active_goal_xy)
                self.goal_handle.cancel_goal_async()
                self.goal_in_progress = False
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

        target, score = self._select_frontier_goal(
            self.latest_map, robot_x, robot_y, robot_yaw
        )

        if target is None:
            self._no_frontier_ticks += 1
            self._try_relax()
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
        goal = NavigateToPose.Goal()
        goal.pose = target_pose
        future = self.nav_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(self._goal_response_cb)
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

    def _goal_response_cb(self, future) -> None:
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal rejected by nav2")
            self.goal_in_progress = False
            self.active_goal_xy = None
            self.active_goal_score = -float("inf")
            self.last_goal_end_time = self.get_clock().now()
            return
        self.goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future) -> None:
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Frontier goal reached")
            self.active_goal_xy = None
            self.active_goal_score = -float("inf")
            self._last_cancel_xy = None   # success — lift the min-distance restriction
        else:
            self.get_logger().warn(f"Goal ended with status {status}")
            if self.active_goal_xy is not None:
                self._blacklist_goal(self.active_goal_xy)
                self.active_goal_xy = None
            self.active_goal_score = -float("inf")
        self.goal_in_progress = False
        self.last_goal_end_time = self.get_clock().now()
        self.last_distance_remaining = None

    def _feedback_cb(self, feedback_msg) -> None:
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

        frontiers = self._extract_frontiers(data, width, height)
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

            # openness_term is used only in scoring (not as a hard filter) so that
            # tight-space frontiers (between shelf posts, near dividers) are still
            # considered when no open-area frontiers remain — they just score lower.

            size_term = len(frontier) / float(max_size)
            distance_term = dist / max_dist
            heading_term = heading_error / math.pi

            score = (
                self.size_weight * size_term
                - self.distance_weight * distance_term
                - self.heading_weight * heading_term
                + self.openness_weight * openness_term
            )
            if score > best_score:
                best_score = score
                best_goal = (cx, cy)
            if collect_all:
                all_valid.append(((cx, cy), score))

        if collect_all and all_valid:
            # Sort all candidates best-first and store in queue (skip index 0
            # which is the goal we're about to return — it goes to the caller)
            all_valid.sort(key=lambda x: x[1], reverse=True)
            self._frontier_queue = all_valid[1:1 + self._queue_size]
            self._queue_stamp = self.get_clock().now()

        self.get_logger().debug(
            f"score_frontiers(max_dist={max_dist:.1f}): total={len(frontiers)} "
            f"too_small={n_small} no_target={n_no_target} "
            f"edge={n_edge} blacklisted={n_blacklist} dist_fail={n_dist} "
            f"queued={len(self._frontier_queue)} "
            f"result={'found' if best_goal else 'none'}"
        )
        return best_goal, best_score

    def _extract_frontiers(
        self, data, width: int, height: int
    ) -> List[List[GridCell]]:
        """Vectorized frontier extraction via numpy; BFS clustering in Python."""
        grid = np.asarray(data, dtype=np.int8).reshape((height, width))
        unknown = grid == -1
        free = grid == 0

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
            # Same area — increment hit counter on the existing key
            hits = self._blacklist_hits.get(closest_key, 1) + 1
            self._blacklist_hits[closest_key] = hits
            self.failed_goals[closest_key] = self.get_clock().now()
            if hits > 1:
                self.get_logger().warn(
                    f"Area {closest_key} blacklisted {hits}× — "
                    f"effective radius now {self.goal_blacklist_radius * 2.0:.2f} m"
                )
        else:
            # New area
            self._blacklist_hits[xy] = 1
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
        return False

    def _prune_failed_goals(self) -> None:
        now = self.get_clock().now()
        keep = {
            goal: stamp
            for goal, stamp in self.failed_goals.items()
            if (now - stamp) <= Duration(seconds=self.blacklist_timeout_sec)
        }
        # Clean up hit counters for pruned entries
        for gone in set(self.failed_goals) - set(keep):
            self._blacklist_hits.pop(gone, None)
        self.failed_goals = keep


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
