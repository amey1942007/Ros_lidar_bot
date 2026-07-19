#!/usr/bin/env python3
"""
Scan-based safety stop — hardware-critical velocity filter.

Sits between Nav2/teleop and the real robot driver:

  Nav2 / teleop  →  /cmd_vel
  safety_stop    →  /cmd_vel_safe  →  driver  →  robot

When any laser reading in the forward arc falls below min_safe_distance the
node zeroes linear.x (forward motion) while preserving angular.z so Nav2's
recovery behaviours (spin in place) can still operate.
Same logic applies to the rear arc for reverse motion.

Ranges closer than ignore_below are treated as the robot's own body and
ignored — otherwise the chassis always trips a forward block and Nav2
"plans but never drives".

If /odom_raw goes silent (driver/encoder drop), ALL motion is zeroed — Nav2
must not keep spinning on a frozen/drifting EKF pose.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class SafetyStop(Node):
    def __init__(self) -> None:
        super().__init__("safety_stop")

        self.declare_parameter("min_safe_distance", 0.50)  # metres
        self.declare_parameter("ignore_below", 0.45)       # ignore chassis (≤~0.45 m)
        self.declare_parameter("front_opening_deg", 90.0)  # total forward arc
        self.declare_parameter("rear_opening_deg", 50.0)   # total rear arc
        self.declare_parameter("odom_raw_timeout_sec", 0.5)

        self._min_dist = float(self.get_parameter("min_safe_distance").value)
        self._ignore_below = float(self.get_parameter("ignore_below").value)
        self._front_half = math.radians(
            float(self.get_parameter("front_opening_deg").value) / 2.0
        )
        self._rear_half = math.radians(
            float(self.get_parameter("rear_opening_deg").value) / 2.0
        )
        self._odom_timeout = float(self.get_parameter("odom_raw_timeout_sec").value)

        self._blocked_fwd = False
        self._blocked_bwd = False
        self._odom_stale = False
        self._last_cmd = Twist()
        self._last_cmd_time: float = 0.0
        self._last_odom_raw_time: float = 0.0
        self._blocked_since: float = 0.0

        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(LaserScan, "/scan", self._scan_cb, scan_qos)
        self.create_subscription(Odometry, "/odom_raw", self._odom_raw_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self._blocked_pub = self.create_publisher(Bool, "/safety_blocked", 1)
        self.create_timer(0.1, self._watchdog_tick)

        self.get_logger().info(
            f"SafetyStop ready: stop<{self._min_dist:.2f} m, "
            f"ignore_self<{self._ignore_below:.2f} m, "
            f"front±{math.degrees(self._front_half):.0f}°, "
            f"odom_raw_timeout={self._odom_timeout:.1f}s"
        )

    def _odom_raw_cb(self, _msg: Odometry) -> None:
        self._last_odom_raw_time = time.monotonic()
        if self._odom_stale:
            self._odom_stale = False
            self.get_logger().info("odom_raw restored — motion re-enabled")

    def _watchdog_tick(self) -> None:
        if self._last_odom_raw_time <= 0.0:
            return  # not seen yet at startup — wait for first sample
        age = time.monotonic() - self._last_odom_raw_time
        if age > self._odom_timeout:
            if not self._odom_stale:
                self._odom_stale = True
                self.get_logger().error(
                    f"odom_raw silent for {age:.1f}s — ZEROING cmd_vel_safe "
                    "(do not drive on dead wheel odometry)"
                )
            self._pub.publish(Twist())
            self._blocked_pub.publish(Bool(data=True))

    def _scan_cb(self, msg: LaserScan) -> None:
        min_front = min_rear = float("inf")
        angle = msg.angle_min
        # Ignore returns on/inside the chassis so we do not permanently block.
        lo = max(float(msg.range_min), self._ignore_below)

        for r in msg.ranges:
            if lo < r < msg.range_max:
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
            self._blocked_since = time.monotonic()
            self.get_logger().warn(
                f"SAFETY STOP: obstacle {min_front:.2f} m ahead — forward motion blocked",
                throttle_duration_sec=1.0,
            )
        elif not self._blocked_fwd and prev_fwd:
            self._blocked_since = 0.0
            self.get_logger().info("Safety stop cleared — path is open")

        self._blocked_pub.publish(Bool(data=bool(self._blocked_fwd or self._odom_stale)))

        if time.monotonic() - self._last_cmd_time < 0.5:
            self._publish_safe(self._last_cmd)

    def _cmd_cb(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = time.monotonic()
        self._publish_safe(msg)

    def _publish_safe(self, cmd: Twist) -> None:
        if self._odom_stale:
            self._pub.publish(Twist())
            return

        out = Twist()

        if self._blocked_fwd and cmd.linear.x > 0.0:
            out.linear.x = 0.0
        elif self._blocked_bwd and cmd.linear.x < 0.0:
            out.linear.x = 0.0
        else:
            out.linear.x = cmd.linear.x

        out.angular.z = cmd.angular.z
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
