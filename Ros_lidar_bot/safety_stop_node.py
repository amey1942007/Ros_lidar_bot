#!/usr/bin/env python3
"""
Scan-based safety stop — hardware-critical velocity filter.

Sits between Nav2/teleop and the Gazebo bridge (or real robot driver):

  Nav2 / teleop  →  /cmd_vel
  safety_stop    →  /cmd_vel_safe  →  bridge / driver  →  robot

When any laser reading in the forward arc falls below min_safe_distance the
node zeroes linear.x (forward motion) while preserving angular.z so Nav2's
recovery behaviours (spin in place) can still operate.
Same logic applies to the rear arc for reverse motion.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyStop(Node):
    def __init__(self) -> None:
        super().__init__("safety_stop")

        self.declare_parameter("min_safe_distance", 0.25)  # metres
        self.declare_parameter("front_opening_deg", 120.0)  # total forward arc
        self.declare_parameter("rear_opening_deg", 60.0)   # total rear arc

        self._min_dist = float(self.get_parameter("min_safe_distance").value)
        self._front_half = math.radians(
            float(self.get_parameter("front_opening_deg").value) / 2.0
        )
        self._rear_half = math.radians(
            float(self.get_parameter("rear_opening_deg").value) / 2.0
        )

        self._blocked_fwd = False
        self._blocked_bwd = False
        self._last_cmd = Twist()

        # Sensor-data QoS: best-effort, keep only last scan
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan", self._scan_cb, scan_qos)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _scan_cb(self, msg: LaserScan) -> None:
        min_front = min_rear = float("inf")
        angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                # Normalise to [-pi, pi]
                a = (angle + math.pi) % (2.0 * math.pi) - math.pi
                if abs(a) <= self._front_half:
                    if r < min_front:
                        min_front = r
                elif abs(a) >= (math.pi - self._rear_half):
                    if r < min_rear:
                        min_rear = r
            angle += msg.angle_increment

        prev_fwd = self._blocked_fwd
        self._blocked_fwd = min_front < self._min_dist
        self._blocked_bwd = min_rear < self._min_dist

        if self._blocked_fwd and not prev_fwd:
            self.get_logger().warn(
                f"SAFETY STOP: obstacle {min_front:.2f} m ahead — forward motion blocked",
                throttle_duration_sec=1.0,
            )
        elif not self._blocked_fwd and prev_fwd:
            self.get_logger().info("Safety stop cleared — path is open")

        # Immediately republish with current safety state applied
        self._publish_safe(self._last_cmd)

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._publish_safe(msg)

    # ── core logic ─────────────────────────────────────────────────────────────

    def _publish_safe(self, cmd: Twist) -> None:
        out = Twist()

        if self._blocked_fwd and cmd.linear.x > 0.0:
            out.linear.x = 0.0          # block forward into obstacle
        elif self._blocked_bwd and cmd.linear.x < 0.0:
            out.linear.x = 0.0          # block reversing into obstacle
        else:
            out.linear.x = cmd.linear.x

        # Always pass through rotation — required for Nav2 spin recovery
        out.angular.z = cmd.angular.z
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
