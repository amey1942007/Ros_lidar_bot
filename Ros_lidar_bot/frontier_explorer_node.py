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

        self.latest_map: Optional[OccupancyGrid] = None
        self.goal_handle = None
        self.goal_start_time = None
        self.goal_in_progress = False
        self.active_goal_xy: Optional[Tuple[float, float]] = None
        self.active_goal_score = -float("inf")
        self.failed_goals: Dict[Tuple[float, float], rclpy.time.Time] = {}
        self.last_goal_end_time = self.get_clock().now()
        self.last_distance_remaining: Optional[float] = None
        self.last_progress_time = self.get_clock().now()
        self._no_frontier_ticks = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(OccupancyGrid, self.map_topic, self._map_cb, 10)
        self.create_timer(float(p("goal_interval_sec").value), self._tick)

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg

    def _tick(self) -> None:
        if self.latest_map is None:
            return

        if self.goal_in_progress:
            if self.goal_start_time is None:
                return
            elapsed = self.get_clock().now() - self.goal_start_time
            if elapsed > Duration(seconds=self.goal_timeout_sec):
                self.get_logger().warn("Goal timeout, cancelling")
                self.goal_handle.cancel_goal_async()
                self.goal_in_progress = False
            elif (self.get_clock().now() - self.last_progress_time) > Duration(
                seconds=self.progress_timeout_sec
            ):
                self.get_logger().warn("No progress, cancelling")
                self.goal_handle.cancel_goal_async()
                self.goal_in_progress = False
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
        else:
            self.get_logger().warn(f"Goal ended with status {status}")
            if self.active_goal_xy is not None:
                self.failed_goals[self.active_goal_xy] = self.get_clock().now()
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
        result = self._score_frontiers(
            frontiers, data, width, height, res, ox, oy,
            robot_x, robot_y, robot_yaw, self.max_goal_distance,
        )
        if result[0] is not None:
            return result

        # Expand search range once when nothing is reachable in normal range.
        return self._score_frontiers(
            frontiers, data, width, height, res, ox, oy,
            robot_x, robot_y, robot_yaw,
            self.max_goal_distance * self.max_range_expansion_factor,
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
    ) -> Tuple[Optional[Tuple[float, float]], float]:
        max_size = max(len(f) for f in frontiers)
        best_goal: Optional[Tuple[float, float]] = None
        best_score = -float("inf")
        n_small = n_no_target = n_blacklist = n_dist = n_edge = 0

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

        self.get_logger().debug(
            f"score_frontiers(max_dist={max_dist:.1f}): total={len(frontiers)} "
            f"too_small={n_small} no_target={n_no_target} "
            f"edge={n_edge} blacklisted={n_blacklist} dist_fail={n_dist} "
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

    def _is_blacklisted(self, gx: float, gy: float) -> bool:
        r2 = self.goal_blacklist_radius ** 2
        return any(
            (gx - bx) ** 2 + (gy - by) ** 2 <= r2 for bx, by in self.failed_goals
        )

    def _prune_failed_goals(self) -> None:
        now = self.get_clock().now()
        self.failed_goals = {
            goal: stamp
            for goal, stamp in self.failed_goals.items()
            if (now - stamp) <= Duration(seconds=self.blacklist_timeout_sec)
        }


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
