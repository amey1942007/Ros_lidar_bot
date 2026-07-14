#!/usr/bin/env python3
"""\nnon_safety_teleop_node.py — Keyboard teleop node that bypasses safety stop limits.

================================================================================
UNDERLYING SYSTEM & DATA FLOW
================================================================================
Publishes to: /cmd_vel_safe (geometry_msgs/Twist)
- Bypasses `/cmd_vel` filters (like `safety_stop_node.py`) in development.
- Directly controls the wheels via DDSM115 hardware motor drivers.

================================================================================
CONTROL KEYS & SPEED SCALING
================================================================================
  w / s : increase/decrease linear velocity (forward/back)
  a / d : increase/decrease angular velocity (turn left/right)
  q / z : increase/decrease max linear velocity limit by 0.05 m/s
  e / c : increase/decrease max angular velocity limit by 0.1 rad/s
  space or x : STOP motion immediately
  Ctrl+C : exit teleop safely\n"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg_banner = """
Control Your Robot Directly (Bypassing Safety Stop-filter)
----------------------------------------------------------
Moving around:        Speed limits:
        w                 q / z : +/- max linear speed limit
   a    s    d            e / c : +/- max angular speed limit

Space or x : force quit / stop moving
Ctrl+C to exit

Press keys to adjust velocity / speed limits.
"""

class NonSafetyTeleop(Node):
    def __init__(self):
        super().__init__('non_safety_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # Current speeds
        self.linear = 0.0
        self.angular = 0.0

        # Maximum limits & step sizes
        self.max_lin = 0.5     # default max linear speed (m/s)
        self.max_ang = 1.0     # default max angular speed (rad/s)
        
        # Incremental step is always 10% of the maximum speed
        self.lin_step = self.max_lin / 10.0
        self.ang_step = self.max_ang / 10.0

        self.get_logger().info("Non-Safety Teleop Node starting. Direct command to: /cmd_vel_safe")

    def update_steps(self):
        self.lin_step = self.max_lin / 10.0
        self.ang_step = self.max_ang / 10.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)
        
        # Clear line and print status
        status = (f"Speed Limits: [lin: {self.max_lin:.2f} m/s | ang: {self.max_ang:.2f} rad/s]  "
                  f"Target Vel: [lin: {self.linear:.2f} m/s | ang: {self.angular:.2f} rad/s]")
        print(f"\r\033[K{status}", end="", flush=True)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clamp(val, minv, maxv):
    return max(min(val, maxv), minv)

def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = NonSafetyTeleop()

    print(msg_banner)
    node.publish()

    try:
        while rclpy.ok():
            key = get_key(settings)

            # Velocity Controls
            if key == 'w':
                node.linear += node.lin_step
            elif key == 's':
                node.linear -= node.lin_step
            elif key == 'a':
                node.angular += node.ang_step
            elif key == 'd':
                node.angular -= node.ang_step
                
            # Limit scaling Options
            elif key == 'q':
                node.max_lin = round(node.max_lin + 0.05, 3)
                node.update_steps()
            elif key == 'z':
                node.max_lin = max(0.05, round(node.max_lin - 0.05, 3))
                node.update_steps()
            elif key == 'e':
                node.max_ang = round(node.max_ang + 0.1, 3)
                node.update_steps()
            elif key == 'c':
                node.max_ang = max(0.1, round(node.max_ang - 0.1, 3))
                node.update_steps()
                
            # Stop command
            elif key in (' ', 'x'):
                node.linear = 0.0
                node.angular = 0.0
            elif key == '\x03':  # Ctrl+C
                break

            # Clamp velocities
            node.linear = clamp(node.linear, -node.max_lin, node.max_lin)
            node.angular = clamp(node.angular, -node.max_ang, node.max_ang)

            node.publish()

    except Exception as e:
        print(f"\nError in teleop loop: {e}")
    finally:
        # Reset and zero on exit
        node.linear = 0.0
        node.angular = 0.0
        node.publish()
        print("\nTeleop node exiting. Velocities zeroed.")
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
