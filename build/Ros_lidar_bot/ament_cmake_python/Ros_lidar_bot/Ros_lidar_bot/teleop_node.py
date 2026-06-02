#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, termios, tty

class MyTeleop(Node):
    def __init__(self):
        super().__init__('my_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.linear = 0.0
        self.angular = 0.0

        self.lin_step = 0.1
        self.ang_step = 0.1
        self.max_lin = 1.0
        self.max_ang = 2.0

    def publish(self):
        msg = Twist()
        msg.linear.x = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

def get_key():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clamp(val, minv, maxv):
    return max(min(val, maxv), minv)

def main():
    rclpy.init()
    node = MyTeleop()

    global settings
    settings = termios.tcgetattr(sys.stdin)

    try:
        while rclpy.ok():
            key = get_key()

            if key == 'i':       # forward
                node.linear += node.lin_step
            elif key == 'k':     # backward
                node.linear -= node.lin_step
            elif key == 'j':     # left
                node.angular += node.ang_step
            elif key == 'l':     # right
                node.angular -= node.ang_step
            elif key == ' ':     # STOP
                node.linear = 0.0
                node.angular = 0.0
            elif key == '\x03':  # Ctrl+C
                break

            node.linear = clamp(node.linear, -node.max_lin, node.max_lin)
            node.angular = clamp(node.angular, -node.max_ang, node.max_ang)

            node.publish()

    finally:
        node.linear = 0.0
        node.angular = 0.0
        node.publish()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
