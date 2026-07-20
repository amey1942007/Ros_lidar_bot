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
  LT+RT+LB+RB together  — start IMU calibration (robot spins in place!)
  B                     — save map as map_YYYYMMDD_HHMMSS (~/maps/)
  X                     — toggle the semantic vision pipeline on/off

The three actions above are fired through the robot dashboard's HTTP API
(localhost:8080), so they share its one-tool-at-a-time management and
their output appears in the dashboard console. Each fires once per press
with a 2 s cooldown; confirmation is a long strong rumble pulse.

Behaviour:
  - Stick deflection scales speed proportionally up to the current setpoint.
  - Stick centered → one zero Twist is published, then this node goes silent
    so Nav2 goals on /cmd_vel can drive the robot (same yield scheme the old
    keyboard teleop used).
  - If /joy stops arriving mid-motion (Bluetooth dropout / dongle yanked)
    → immediate stop.

Axis/button indices are parameters. Defaults match this robot's pad over
BLUETOOTH (re-measured live on /joy, 2026-07-20):
  axes:    0=LX  1=LY  2=RX  3=RY  4=LT  5=RT   (triggers rest +1, pressed -1)
  buttons: 6=LB  7=RB  13=L3  14=R3   (sparse Xbox-BT hid layout)
NOTE: the same pad over a USB dongle (xpad) usually has LT on axis 2 and
RT on axis 5 instead. If controls act wrong after changing transport,
verify with:  ros2 topic echo /joy  and override the parameters.

Feedback: every accepted speed change fires a short rumble pulse on
/joy/set_feedback (strong double-length pulse when hitting a limit), since
robot logs are suppressed on the quiet bringup.
"""

import json
import threading
import time
import urllib.request

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy, JoyFeedback, JoyFeedbackArray


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')

        # ── Mapping parameters (override if `ros2 topic echo /joy` disagrees) ─
        self.declare_parameter('axis_linear', 1)     # left stick vertical
        self.declare_parameter('axis_angular', 0)    # left stick horizontal
        self.declare_parameter('axis_rt', 5)         # right trigger
        self.declare_parameter('axis_lt', 4)         # left trigger (BT; USB xpad = 2)
        self.declare_parameter('button_rb', 7)       # right bumper
        self.declare_parameter('button_lb', 6)       # left bumper
        self.declare_parameter('button_save_map', 1) # B (stick clicks were unreliable)
        self.declare_parameter('button_vision', 3)   # X
        self.declare_parameter('dashboard_url', 'http://127.0.0.1:8080')

        # ── Speed setpoints ───────────────────────────────────────────────────
        self.declare_parameter('lin_speed', 0.25)    # m/s at full stick
        self.declare_parameter('ang_speed', 0.8)     # rad/s at full stick
        self.declare_parameter('lin_step', 0.05)     # m/s per RT/LT press
        self.declare_parameter('ang_step', 0.1)      # rad/s per RB/LB press
        self.declare_parameter('lin_min', 0.05)
        # 0.5 matches the Nav2 velocity_smoother cap — teleop publishes raw
        # to /cmd_vel, so nothing else limits it. 0.8 indoors was too hot.
        self.declare_parameter('lin_max', 0.5)
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
        self._btn_save = gp('button_save_map')
        self._btn_vision = gp('button_vision')
        self._dash_url = str(gp('dashboard_url')).rstrip('/')
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
        # Triggers rest near +1 (or 0 until first touched — driver quirk)
        # and read -1 fully pressed, so "pressed" = value < 0.0 (≈ half
        # pull; -0.5 required a near-full pull and felt unresponsive).
        self._rt_was_pressed = False
        self._lt_was_pressed = False
        self._rb_was_pressed = False
        self._lb_was_pressed = False
        self._combo_was_pressed = False
        self._save_was_pressed = False
        self._vision_was_pressed = False
        self._action_last = {}   # action name → monotonic time of last fire

        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._rumble_pub = self.create_publisher(
            JoyFeedbackArray, '/joy/set_feedback', 10)
        self._rumble_off_timer = None
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
        rt = axis(self._ax_rt) < 0.0
        lt = axis(self._ax_lt) < 0.0
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

        # Dashboard actions — edge triggered, 2 s cooldown each.
        # (The four-button combo also fires the ± speed steps above, but the
        # +/− pairs cancel out, so the setpoints are unchanged.)
        combo = rt and lt and rb and lb
        if combo and not self._combo_was_pressed:
            self._fire_action('imu_cal')
        self._combo_was_pressed = combo

        sv = button(self._btn_save)
        if sv and not self._save_was_pressed:
            self._fire_action('save_map')
        self._save_was_pressed = sv

        vi = button(self._btn_vision)
        if vi and not self._vision_was_pressed:
            self._fire_action('vision')
        self._vision_was_pressed = vi

        # Movement — proportional to stick deflection
        lin_in = axis(self._ax_lin)
        ang_in = axis(self._ax_ang)
        if abs(lin_in) < self._deadzone:
            lin_in = 0.0
        if abs(ang_in) < self._deadzone:
            ang_in = 0.0

        self._cmd_lin = lin_in * self._lin_speed
        self._cmd_ang = ang_in * self._ang_speed

    # ── Gamepad → dashboard actions ────────────────────────────────────────────
    # Fired through the dashboard HTTP API so tool management stays in one
    # place (one at a time, output in the dashboard console). Runs in a
    # daemon thread — the 20 Hz cmd_vel loop must never block on HTTP.
    _ACTIONS = {
        'imu_cal':  ('/api/run', {'tool': 'imu_calibration', 'params': {}},
                     'IMU calibration (LT+RT+LB+RB)'),
        'save_map': ('/api/save_map', {'name': ''},   # empty → map_YYYYMMDD_HHMMSS
                     'save map (B)'),
        'vision':   ('/api/vision', {'toggle': True},
                     'vision on/off (X)'),
    }

    def _fire_action(self, name):
        now = time.monotonic()
        # Vision launch needs several seconds (reclaim + ros2 launch + camera);
        # a second X within the generic 2s window still aborts mid-start.
        cooldown = 5.0 if name == 'vision' else 2.0
        if now - self._action_last.get(name, 0.0) < cooldown:
            return
        self._action_last[name] = now
        endpoint, payload, desc = self._ACTIONS[name]
        self.get_logger().info(f'gamepad action → {desc}')
        self._rumble(at_limit=True)   # long strong pulse = action registered
        threading.Thread(target=self._post_dashboard,
                         args=(endpoint, payload, desc), daemon=True).start()

    def _post(self, endpoint, payload):
        req = urllib.request.Request(
            self._dash_url + endpoint,
            data=json.dumps(payload).encode(),
            headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=3.0) as resp:
            return json.loads(resp.read() or b'{}')

    def _post_dashboard(self, endpoint, payload, desc):
        # Robot logs are suppressed on the quiet bringup, so the press and
        # any failure are echoed into the dashboard console instead.
        try:
            self._post('/api/log', {'text': f'🎮 {desc}', 'level': 'run'})
        except Exception:
            pass
        try:
            out = self._post(endpoint, payload)
            if not out.get('ok', False):
                err = out.get('error', '?')
                self.get_logger().error(f'{desc} failed: {err}')
                try:
                    self._post('/api/log',
                               {'text': f'🎮 {desc} FAILED: {err}', 'level': 'error'})
                except Exception:
                    pass
        except Exception as exc:
            self.get_logger().error(f'{desc} request failed: {exc}')

    def _adjust_lin(self, delta):
        new = round(
            max(self._lin_min, min(self._lin_max, self._lin_speed + delta)), 3)
        self._rumble(at_limit=(new == self._lin_speed))
        self._lin_speed = new
        self.get_logger().info(f'Linear speed → {self._lin_speed:.2f} m/s')

    def _adjust_ang(self, delta):
        new = round(
            max(self._ang_min, min(self._ang_max, self._ang_speed + delta)), 3)
        self._rumble(at_limit=(new == self._ang_speed))
        self._ang_speed = new
        self.get_logger().info(f'Angular speed → {self._ang_speed:.2f} rad/s')

    # ── Haptic feedback ────────────────────────────────────────────────────────
    # Robot logs are hidden on the quiet bringup, so a rumble pulse is the
    # only confirmation a trigger/bumper press registered. Short soft pulse
    # per step; long strong pulse when already at the min/max limit.
    def _rumble(self, at_limit: bool):
        fb = JoyFeedback()
        fb.type = JoyFeedback.TYPE_RUMBLE
        fb.id = 0
        fb.intensity = 1.0 if at_limit else 0.5
        msg = JoyFeedbackArray()
        msg.array = [fb]
        self._rumble_pub.publish(msg)

        if self._rumble_off_timer is not None:
            self._rumble_off_timer.cancel()
        self._rumble_off_timer = self.create_timer(
            0.4 if at_limit else 0.15, self._rumble_off)

    def _rumble_off(self):
        if self._rumble_off_timer is not None:
            self._rumble_off_timer.cancel()
            self._rumble_off_timer = None
        fb = JoyFeedback()
        fb.type = JoyFeedback.TYPE_RUMBLE
        fb.id = 0
        fb.intensity = 0.0
        msg = JoyFeedbackArray()
        msg.array = [fb]
        self._rumble_pub.publish(msg)

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
        # Best-effort motor zero: on launch SIGINT the context is already dead
        # and publish raises — the driver's 1 s cmd_vel timeout stops the
        # motors regardless, so never let this crash the exit path.
        try:
            node._pub.publish(Twist())   # motors zeroed on exit
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
