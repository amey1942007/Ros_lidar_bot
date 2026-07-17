#!/usr/bin/env python3
"""
joy_teleop_node.py — Gamepad teleop (Bluetooth or USB controller).

Pairs with the standard `joy` package's joy_node (SDL), which reads the
controller and publishes sensor_msgs/Joy on /joy. This node converts /joy
into /cmd_vel (geometry_msgs/Twist).

Controls (Xbox-style layout, xpad driver mapping):
  Left stick up/down    — forward / reverse at current linear speed
  Left stick left/right — turn left / right at current angular speed
  RT (right trigger)    — increase linear speed by lin_step per press
  LT (left trigger)     — decrease linear speed by lin_step per press
  RB (right bumper)     — increase angular speed by ang_step per press
  LB (left bumper)      — decrease angular speed by ang_step per press

Behaviour:
  - Stick deflection scales speed proportionally up to the current setpoint.
  - Stick centered → one zero Twist is published, then this node goes silent
    so Nav2 goals on /cmd_vel can drive the robot (same yield scheme the old
    keyboard teleop used).
  - If /joy stops arriving mid-motion (Bluetooth dropout / dongle yanked)
    → immediate stop.

Axis/button indices are parameters. Defaults match the Linux xpad driver:
  axes:    0=LX  1=LY  2=LT  3=RX  4=RY  5=RT   (triggers rest +1, pressed -1)
  buttons: 0=A 1=B 2=X 3=Y 4=LB 5=RB 6=back 7=start
Bluetooth pads often enumerate DIFFERENTLY than the same pad over USB
(trigger axes especially). Verify with:  ros2 topic echo /joy
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # ── Mapping parameters (override if `ros2 topic echo /joy` disagrees) ─
        self.declare_parameter('axis_linear', 1)     # left stick vertical
        self.declare_parameter('axis_angular', 0)    # left stick horizontal
        self.declare_parameter('axis_rt', 5)         # right trigger
        self.declare_parameter('axis_lt', 2)         # left trigger
        self.declare_parameter('button_rb', 5)       # right bumper
        self.declare_parameter('button_lb', 4)       # left bumper

        # ── Speed setpoints ───────────────────────────────────────────────────
        self.declare_parameter('lin_speed', 0.25)    # m/s at full stick
        self.declare_parameter('ang_speed', 0.8)     # rad/s at full stick
        self.declare_parameter('lin_step', 0.05)     # m/s per RT/LT press
        self.declare_parameter('ang_step', 0.1)      # rad/s per RB/LB press
        self.declare_parameter('lin_min', 0.05)
        self.declare_parameter('lin_max', 0.8)
        self.declare_parameter('ang_min', 0.2)
        self.declare_parameter('ang_max', 2.0)

        self.declare_parameter('deadzone', 0.15)     # stick idle threshold
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('joy_timeout', 0.5)   # s without /joy → stop

        gp = lambda n: self.get_parameter(n).value
        self._ax_lin = gp('axis_linear')
        self._ax_ang = gp('axis_angular')
        self._ax_rt = gp('axis_rt')
        self._ax_lt = gp('axis_lt')
        self._btn_rb = gp('button_rb')
        self._btn_lb = gp('button_lb')
        self._lin_speed = gp('lin_speed')
        self._ang_speed = gp('ang_speed')
        self._lin_step = gp('lin_step')
        self._ang_step = gp('ang_step')
        self._lin_min, self._lin_max = gp('lin_min'), gp('lin_max')
        self._ang_min, self._ang_max = gp('ang_min'), gp('ang_max')
        self._deadzone = gp('deadzone')
        self._joy_timeout = gp('joy_timeout')

        self._cmd_lin = 0.0
        self._cmd_ang = 0.0
        self._last_joy_time = 0.0
        # After the stick returns to center, publish one zero then stay silent
        # so Nav2 owns /cmd_vel until the stick moves again.
        self._yielded_to_nav = True

        # Edge detection for speed-adjust inputs (act once per press).
        # Triggers: xpad rests near +1 (or 0 until first touched — driver
        # quirk) and reads -1 fully pressed, so "pressed" = value < -0.5.
        self._rt_was_pressed = False
        self._lt_was_pressed = False
        self._rb_was_pressed = False
        self._lb_was_pressed = False

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._sub = self.create_subscription(Joy, '/joy', self._joy_cb, 10)
        self._timer = self.create_timer(1.0 / gp('publish_hz'), self._publish_cb)

        self.get_logger().info(
            f'Gamepad teleop ready — lin {self._lin_speed:.2f} m/s, '
            f'ang {self._ang_speed:.2f} rad/s at full stick.')

    # ── /joy callback ──────────────────────────────────────────────────────────
    def _joy_cb(self, msg: Joy):
        self._last_joy_time = time.monotonic()

        def axis(i):
            return msg.axes[i] if i < len(msg.axes) else 0.0

        def button(i):
            return bool(msg.buttons[i]) if i < len(msg.buttons) else False

        # Speed adjustment — edge triggered, once per press
        rt = axis(self._ax_rt) < -0.5
        lt = axis(self._ax_lt) < -0.5
        rb = button(self._btn_rb)
        lb = button(self._btn_lb)

        if rt and not self._rt_was_pressed:
            self._adjust_lin(+self._lin_step)
        if lt and not self._lt_was_pressed:
            self._adjust_lin(-self._lin_step)
        if rb and not self._rb_was_pressed:
            self._adjust_ang(+self._ang_step)
        if lb and not self._lb_was_pressed:
            self._adjust_ang(-self._ang_step)

        self._rt_was_pressed, self._lt_was_pressed = rt, lt
        self._rb_was_pressed, self._lb_was_pressed = rb, lb

        # Movement — proportional to stick deflection
        lin_in = axis(self._ax_lin)
        ang_in = axis(self._ax_ang)
        if abs(lin_in) < self._deadzone:
            lin_in = 0.0
        if abs(ang_in) < self._deadzone:
            ang_in = 0.0

        self._cmd_lin = lin_in * self._lin_speed
        self._cmd_ang = ang_in * self._ang_speed

    def _adjust_lin(self, delta):
        self._lin_speed = round(
            max(self._lin_min, min(self._lin_max, self._lin_speed + delta)), 3)
        self.get_logger().info(f'Linear speed → {self._lin_speed:.2f} m/s')

    def _adjust_ang(self, delta):
        self._ang_speed = round(
            max(self._ang_min, min(self._ang_max, self._ang_speed + delta)), 3)
        self.get_logger().info(f'Angular speed → {self._ang_speed:.2f} rad/s')

    # ── Publisher timer ────────────────────────────────────────────────────────
    def _publish_cb(self):
        # Dongle unplugged / joy_node died mid-motion → stop.
        if (time.monotonic() - self._last_joy_time) > self._joy_timeout:
            self._cmd_lin = 0.0
            self._cmd_ang = 0.0

        moving = abs(self._cmd_lin) > 1e-6 or abs(self._cmd_ang) > 1e-6
        if not moving:
            if not self._yielded_to_nav:
                self._pub.publish(Twist())   # one stop pulse, then yield
                self._yielded_to_nav = True
            return

        self._yielded_to_nav = False
        msg = Twist()
        msg.linear.x = self._cmd_lin
        msg.angular.z = self._cmd_ang
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._pub.publish(Twist())   # motors zeroed on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
