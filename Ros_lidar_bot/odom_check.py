#!/usr/bin/env python3
"""
odom_check.py  –  Live odometry diagnostics for Ros_lidar_bot
──────────────────────────────────────────────────────────────
Usage:
  ros2 run Ros_lidar_bot odom_check

What it reports every second:
  • Current odom pose  (x, y, yaw)
  • Pose covariance diagonals  → high values mean sensor is uncertain
  • Velocity  (vx, vyaw)
  • Cumulative drift from start
  • WARNINGs when:
      - positional covariance  > 0.5
      - angular covariance      > 0.1
      - unexpected lateral vel  > 0.05 m/s  (diff-drive bug indicator)
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomChecker(Node):
    def __init__(self):
        super().__init__('odom_checker')
        self._start_x = None
        self._start_y = None
        self._count   = 0
        self.create_subscription(Odometry, '/odom', self._cb, 10)
        self.get_logger().info(
            '\n'
            '╔══════════════════════════════════════════╗\n'
            '║      ODOMETRY DIAGNOSTIC NODE STARTED    ║\n'
            '║  Watching /odom — reporting every msg    ║\n'
            '╚══════════════════════════════════════════╝'
        )

    def _cb(self, msg: Odometry):
        p   = msg.pose.pose.position
        q   = msg.pose.pose.orientation
        cov = msg.pose.covariance   # 6×6 row-major
        vx  = msg.twist.twist.linear.x
        vy  = msg.twist.twist.linear.y
        wz  = msg.twist.twist.angular.z

        # Convert quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Record start pose
        if self._start_x is None:
            self._start_x, self._start_y = p.x, p.y

        drift = math.hypot(p.x - self._start_x, p.y - self._start_y)
        self._count += 1
        if self._count % 10 != 0:   # print every 10th message (~1 Hz at 10 Hz odom)
            return

        cov_x   = cov[0]   # xx
        cov_y   = cov[7]   # yy
        cov_yaw = cov[35]  # yaw-yaw

        sep = '─' * 44
        self.get_logger().info(
            f'\n{sep}\n'
            f'  Pose  x={p.x:+.3f}  y={p.y:+.3f}  yaw={math.degrees(yaw):+.1f}°\n'
            f'  Vel   vx={vx:+.3f} m/s   vy={vy:+.3f} m/s   wz={wz:+.3f} rad/s\n'
            f'  Cov   x={cov_x:.4f}  y={cov_y:.4f}  yaw={cov_yaw:.4f}\n'
            f'  Drift from start: {drift:.3f} m\n'
            f'{sep}'
        )

        # Warnings
        if cov_x > 0.5 or cov_y > 0.5:
            self.get_logger().warn(
                f'⚠  HIGH POSITIONAL COVARIANCE  (x={cov_x:.3f}, y={cov_y:.3f}) '
                '— odometry is UNCERTAIN. Check wheel slip or diff-drive plugin.'
            )
        if cov_yaw > 0.1:
            self.get_logger().warn(
                f'⚠  HIGH ANGULAR COVARIANCE  (yaw={cov_yaw:.3f}) '
                '— heading is drifting. Check IMU or encoder calibration.'
            )
        if abs(vy) > 0.05:
            self.get_logger().warn(
                f'⚠  LATERAL VELOCITY vy={vy:.3f} m/s on a DIFF-DRIVE robot! '
                'This is physically impossible — likely a bridge or TF issue.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
