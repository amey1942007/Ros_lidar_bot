#!/usr/bin/env python3
"""
Strip near-field LaserScan returns (robot chassis / mast) from /scan.

sllidar publishes every hardware return on /scan_raw. This node republishes
/scan with ranges below min_range replaced by +inf and range_min raised, so
RViz, SLAM, Nav2, and safety_stop all see the same cleaned cloud.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanMinRangeFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_min_range_filter")
        self._min = float(self.declare_parameter("min_range", 0.45).value)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(LaserScan, "scan", qos)
        self.create_subscription(LaserScan, "scan_raw", self._cb, qos)
        self.get_logger().info(
            f"Filtering /scan_raw → /scan: drop ranges < {self._min:.2f} m"
        )

    def _cb(self, msg: LaserScan) -> None:
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = max(float(msg.range_min), self._min)
        out.range_max = msg.range_max
        out.intensities = list(msg.intensities)
        ranges = []
        for r in msg.ranges:
            if math.isfinite(r) and r < self._min:
                ranges.append(float("inf"))
            else:
                ranges.append(r)
        out.ranges = ranges
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMinRangeFilter()
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
