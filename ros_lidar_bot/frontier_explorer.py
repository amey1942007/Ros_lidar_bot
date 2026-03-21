#!/usr/bin/env python3
"""
frontier_explorer.py — ros_lidar_bot

Autonomous frontier-based exploration node.

How it works:
  1. Subscribes to /map (OccupancyGrid published by SLAM Toolbox).
  2. Subscribes to /odom to track the robot's actual position.
  3. Detects 'frontier' cells: cells that are FREE (value 0) and are
     adjacent to at least one UNKNOWN (value -1) cell.
  4. Clusters nearby frontier cells via connected components.
  5. Picks the centroid of the closest cluster to the robot.
  6. Sends that centroid as a NavigateToPose action goal to Nav2.
  7. Waits for Nav2 to finish, then repeats until no frontiers remain.

Run:
  ros2 run ros_lidar_bot frontier_explorer
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from visualization_msgs.msg import Marker

import numpy as np
from scipy import ndimage
import math


class FrontierExplorer(Node):

    # Cell values from OccupancyGrid
    FREE = 0
    UNKNOWN = -1

    # Minimum cluster size (cells) to be considered a valid frontier
    MIN_FRONTIER_SIZE = 3

    def __init__(self):
        super().__init__('frontier_explorer')

        # Current map and robot position
        self._map: OccupancyGrid | None = None
        self._map_data: np.ndarray | None = None
        
        # TF2 Setup for finding robot position in the map
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Map subscriber
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, 10)

        # Marker publisher for RViz
        self._marker_pub = self.create_publisher(
            Marker, '/frontier_goal_marker', 10)

        # Flag: are we currently executing a nav goal?
        self._navigating = False
        
        # Track the active goal so we know if it matches a blacklisted one
        self._active_goal: tuple[float, float] | None = None
        
        # Goal position to avoid sending the same goal twice in a row
        self._last_goal: tuple[float, float] | None = None
        self._same_goal_count = 0
        
        # Blacklist for frontiers that Nav2 fails to reach
        self._unreachable_frontiers: list[tuple[float, float]] = []

        # Timer: poll for new frontiers every 4 seconds when idle
        self._timer = self.create_timer(4.0, self._explore_tick)

        self.get_logger().info(
            '🤖 Frontier Explorer started! Waiting for /map and TF...'
        )

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        """Store the latest map and convert to a numpy array."""
        self._map = msg
        raw = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self._map_data = raw

    def _get_robot_pose(self) -> tuple[float, float] | None:
        """Looks up the robot's coordinates in the map frame."""
        try:
            now = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                'map', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0))
            return (trans.transform.translation.x, trans.transform.translation.y)
        except tf2_ros.TransformException as ex:
            self.get_logger().info(f'Could not transform map to base_link: {ex}', throttle_duration_sec=5.0)
            return None

    # ── Exploration logic ─────────────────────────────────────────────────────

    def _publish_marker(self, x: float, y: float, action: int = Marker.ADD):
        """Publish a sphere marker in RViz to show the target frontier."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = action
        
        # Position
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1  # slightly above ground
        
        # Identity orientation
        marker.pose.orientation.w = 1.0
        
        # Size (0.4m sphere)
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        
        # Color: Bright Green, fully opaque
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self._marker_pub.publish(marker)

    def _is_blacklisted(self, x: float, y: float) -> bool:
        """Check if a coordinate is within 0.5m of a blacklisted frontier."""
        for bx, by in self._unreachable_frontiers:
            if math.hypot(x - bx, y - by) < 0.5:
                return True
        return False

    def _explore_tick(self):
        """Called periodically. If idle, find the next frontier and go there."""
        if self._navigating:
            return
            
        if self._map_data is None:
            self.get_logger().info('Waiting for map...', throttle_duration_sec=5.0)
            return
            
        pose = self._get_robot_pose()
        if pose is None:
            return
        robot_x, robot_y = pose

        self.get_logger().info(
            f'🔍 Robot at ({robot_x:.2f}, {robot_y:.2f}). '
            f'Scanning for frontiers...'
        )

        frontier_goal = self._find_best_frontier(robot_x, robot_y)
        if frontier_goal is None:
            self.get_logger().info(
                '✅ No more reachable frontiers found! Exploration complete.')
            self._timer.cancel()
            # Clear marker since we're done
            self._publish_marker(0.0, 0.0, Marker.DELETE)
            return

        # Skip if this is the same goal as last time to avoid spinning
        # But if we wait too long (e.g., 3 ticks = 12s) and map hasn't updated,
        # blacklist it and move on.
        if (self._last_goal is not None and
                abs(frontier_goal[0] - self._last_goal[0]) < 0.3 and
                abs(frontier_goal[1] - self._last_goal[1]) < 0.3):
            self._same_goal_count += 1
            if self._same_goal_count >= 3:
                self.get_logger().warn(
                    f'Map hasn\'t cleared frontier at {frontier_goal}. Blacklisting it.')
                self._unreachable_frontiers.append(frontier_goal)
                self._last_goal = None
                self._same_goal_count = 0
            else:
                self.get_logger().info(
                    'Same frontier as before — waiting for map to update...')
            return

        self.get_logger().info(
            f'🗺️  Sending goal to frontier at '
            f'({frontier_goal[0]:.2f}, {frontier_goal[1]:.2f})'
        )
        self._last_goal = frontier_goal
        self._active_goal = frontier_goal
        self._same_goal_count = 0
        
        # Announce our target target visually
        self._publish_marker(frontier_goal[0], frontier_goal[1], Marker.ADD)
        
        self._send_nav_goal(frontier_goal[0], frontier_goal[1])

    def _find_best_frontier(self, robot_x: float, robot_y: float):
        """
        Detect frontier cells, cluster them, and return the centroid of the closest
        reachable cluster to the robot.
        """
        grid = self._map_data
        info = self._map.info
        res = info.resolution
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        # Build boolean masks
        free_mask = (grid == self.FREE)
        unknown_mask = (grid == self.UNKNOWN)

        # Dilate unknown mask by 1 cell (8-connectivity) and intersect with free
        struct = ndimage.generate_binary_structure(2, 2)
        unknown_dilated = ndimage.binary_dilation(unknown_mask, structure=struct)
        frontier_mask = free_mask & unknown_dilated

        if not frontier_mask.any():
            return None

        # Label connected frontier regions
        labeled, num_features = ndimage.label(frontier_mask, structure=struct)
        if num_features == 0:
            return None

        # Convert robot world position → grid pixel position
        robot_col = int((robot_x - origin_x) / res)
        robot_row = int((robot_y - origin_y) / res)

        best_centroid = None
        best_dist = float('inf')

        for label_id in range(1, num_features + 1):
            cluster = (labeled == label_id)
            size = int(cluster.sum())
            if size < self.MIN_FRONTIER_SIZE:
                continue

            rows, cols = np.where(cluster)
            cy = int(rows.mean())
            cx = int(cols.mean())

            # Convert pixel centroid → world coordinates
            world_x = cx * res + origin_x + res / 2.0
            world_y = cy * res + origin_y + res / 2.0
            # Reject if we already know we can't get here
            if self._is_blacklisted(world_x, world_y):
                continue

            row = float(robot_row)
            col = float(robot_col)
            dist = float(np.hypot(float(cy) - row, float(cx) - col))
            if dist < best_dist:
                best_dist = dist
                best_centroid = (world_x, world_y)

        return best_centroid

    # ── Nav2 action plumbing ──────────────────────────────────────────────────

    def _send_nav_goal(self, x: float, y: float):
        """Send a NavigateToPose goal to Nav2."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                'Nav2 action server not available yet. Will retry next tick.')
            return

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose

        self._navigating = True
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(
                'Nav2 rejected the goal immediately. Blacklisting it.')
            if self._active_goal is not None:
                self._unreachable_frontiers.append(self._active_goal)
            self._navigating = False
            return

        self.get_logger().info('✅ Nav2 accepted goal! Robot is driving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('✅ Reached frontier!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal cancelled — moving to next frontier.')
        else:
            self.get_logger().warn(
                f'Goal aborted with status {status}. Blacklisting this frontier.')
            if self._active_goal is not None:
                self._unreachable_frontiers.append(self._active_goal)

        self._navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Frontier Explorer stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
