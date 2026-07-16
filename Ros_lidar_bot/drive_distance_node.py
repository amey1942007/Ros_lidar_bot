#!/usr/bin/env python3
"""
drive_distance_node.py — Drive a precise distance with a trapezoidal/triangular velocity profile.

Publishes to: /cmd_vel_safe (geometry_msgs/Twist)
Subscribes to: /odom (nav_msgs/Odometry) (optional, for logging / distance verification)
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class DriveDistance(Node):
    def __init__(self) -> None:
        super().__init__("drive_distance")

        # Declare parameters for distance, limits, and rates
        self.declare_parameter("distance", 2.0)       # Target distance in meters (negative to drive backward)
        self.declare_parameter("max_vel", 0.4)        # Maximum linear velocity (m/s)
        self.declare_parameter("accel", 0.2)          # Acceleration (m/s^2)
        self.declare_parameter("decel", 0.2)          # Deceleration (m/s^2)
        self.declare_parameter("topic", "/cmd_vel") # Publish topic
        self.declare_parameter("rate", 50.0)          # Publishing rate (Hz)

        self.distance = float(self.get_parameter("distance").value)
        self.max_vel = float(self.get_parameter("max_vel").value)
        self.accel = float(self.get_parameter("accel").value)
        self.decel = float(self.get_parameter("decel").value)
        self.topic_name = self.get_parameter("topic").value
        self.rate_hz = float(self.get_parameter("rate").value)

        # Basic constraints check
        if self.max_vel <= 0:
            raise ValueError("max_vel must be greater than zero.")
        if self.accel <= 0:
            raise ValueError("accel must be greater than zero.")
        if self.decel <= 0:
            raise ValueError("decel must be greater than zero.")

        # Determine direction
        self.sign = 1.0 if self.distance >= 0.0 else -1.0
        self.abs_distance = abs(self.distance)

        # Calculate profile
        self._calculate_profile()

        # Publisher and Subscriber
        self._pub = self.create_publisher(Twist, self.topic_name, 10)
        self._sub = self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        # Odometry tracking
        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        # Timing and state
        self.start_time = None
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_cb)

        self.get_logger().info(f"--- Drive Distance Node Initialized ---")
        self.get_logger().info(f"Target Distance: {self.distance:.3f} m")
        self.get_logger().info(f"Velocity Limit: {self.max_vel:.3f} m/s")
        self.get_logger().info(f"Acceleration: {self.accel:.3f} m/s^2")
        self.get_logger().info(f"Deceleration: {self.decel:.3f} m/s^2")
        self.get_logger().info(f"Calculated Profile:")
        self.get_logger().info(f"  Peak Velocity:  {self.v_peak:.3f} m/s")
        self.get_logger().info(f"  Accel Phase:    {self.t_a:.2f} s")
        self.get_logger().info(f"  Constant Phase: {self.t_c:.2f} s")
        self.get_logger().info(f"  Decel Phase:    {self.t_d:.2f} s")
        self.get_logger().info(f"  Total Duration: {self.T:.2f} s")
        self.get_logger().info(f"--------------------------------------")

    def _calculate_profile(self) -> None:
        # Distance needed to reach max_vel and decelerate to zero
        d_acc = (self.max_vel ** 2) / (2.0 * self.accel)
        d_dec = (self.max_vel ** 2) / (2.0 * self.decel)

        if (d_acc + d_dec) <= self.abs_distance:
            # Trapezoidal profile
            self.v_peak = self.max_vel
            self.t_a = self.v_peak / self.accel
            self.t_d = self.v_peak / self.decel
            d_const = self.abs_distance - (d_acc + d_dec)
            self.t_c = d_const / self.v_peak
            self.T = self.t_a + self.t_c + self.t_d
        else:
            # Triangular profile (cannot reach max_vel in the given distance)
            self.v_peak = math.sqrt(2.0 * self.abs_distance * self.accel * self.decel / (self.accel + self.decel))
            self.t_a = self.v_peak / self.accel
            self.t_d = self.v_peak / self.decel
            self.t_c = 0.0
            self.T = self.t_a + self.t_d

    def _odom_cb(self, msg: Odometry) -> None:
        pos = msg.pose.pose.position
        if self.start_x is None:
            self.start_x = pos.x
            self.start_y = pos.y
        self.current_x = pos.x
        self.current_y = pos.y

    def _timer_cb(self) -> None:
        now = self.get_clock().now()
        if self.start_time is None:
            self.start_time = now
            self.get_logger().info("Starting movement trajectory...")
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed < self.t_a:
            # Phase 1: Accelerating
            v = self.accel * elapsed
            phase = "ACCEL"
        elif elapsed < (self.t_a + self.t_c):
            # Phase 2: Constant Velocity
            v = self.v_peak
            phase = "CONST"
        elif elapsed < self.T:
            # Phase 3: Decelerating
            t_dec = elapsed - (self.t_a + self.t_c)
            v = self.v_peak - self.decel * t_dec
            phase = "DECEL"
        else:
            # Done
            v = 0.0
            phase = "DONE"

        # Apply sign for direction
        vel_x = self.sign * v

        # Publish Twist command
        cmd = Twist()
        cmd.linear.x = vel_x
        cmd.angular.z = 0.0
        self._pub.publish(cmd)

        # Calculate actual odometer displacement if available
        odom_dist = 0.0
        if self.start_x is not None and self.current_x is not None:
            odom_dist = math.sqrt((self.current_x - self.start_x) ** 2 + (self.current_y - self.start_y) ** 2)

        # Log status
        self.get_logger().info(
            f"[{phase}] Elapsed: {elapsed:.2f}/{self.T:.2f}s | "
            f"Cmd: {vel_x:+.3f} m/s | Odom Dist: {odom_dist:.3f} m"
        )

        # Terminate when done
        if elapsed >= self.T + 0.5:
            self.get_logger().info("Trajectory execution complete! Stopping node...")
            self.timer.cancel()
            
            # Send redundant stop commands to ensure safety
            for _ in range(5):
                self._pub.publish(Twist())
                time.sleep(0.02)
                
            rclpy.shutdown()

def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = DriveDistance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in node: {e}")
    finally:
        # Final safety check: publish zeros
        try:
            # Create a temporary node to publish final zeros in case drive node failed
            temp_node = rclpy.create_node("drive_distance_cleanup")
            temp_pub = temp_node.create_publisher(Twist, "/cmd_vel", 10)
            for _ in range(3):
                temp_pub.publish(Twist())
                time.sleep(0.02)
            temp_node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
