#!/usr/bin/env python3
"""
teleop_interface_node.py — Keyboard Teleoperation & Control Interface for Ros_lidar_bot.

Replaces the hardware gamepad controller with a rich terminal interface.

Controls:
  Bot Control (Standard ROS Teleop Keys):
    i : Move Forward
    , / k : Move Backward / Stop
    j : Turn Left
    l : Turn Right
    u : Forward-Left
    o : Forward-Right
    m : Backward-Left
    . : Backward-Right
    SPACE : Emergency Stop
    q / z : Increase / Decrease max linear speed (step 0.05 m/s)
    e / c : Increase / Decrease max angular speed (step 0.1 rad/s)

  Camera Movement (WASD Keys):
    w : Tilt Up
    s : Tilt Down
    a : Pan Left
    d : Pan Right
    x : Center / Reset Camera

  Custom Numpad / Number Actions:
    1 (Numpad 1) : Save Map (~/maps/ map_YYYYMMDD_HHMMSS)
    2 (Numpad 2) : Toggle Vision Pipeline (YOLO-World)
    3 (Numpad 3) : Execute Sequential Goal Pose / Trajectory (Ref: Drive Distance Profile)
"""

import json
import math
import sys
import select
import termios
import tty
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


msg_banner = """
===================================================================
               Ros_lidar_bot TELEOP INTERFACE
===================================================================
  BOT MOVEMENT (Standard ROS Keys):
    u    i    o         [i] Forward         [u/o] Diagonals
    j    k    l         [j/l] Turn L/R      [m/.] Back Diagonals
    m    ,    .         [,] Backward        [SPACE/k] Stop

    Speed Controls:
    q/z : Max Linear Speed  (+/- 0.05 m/s)
    e/c : Max Angular Speed (+/- 0.1 rad/s)

  CAMERA CONTROL (WASD Keys):
    w : Tilt Up        s : Tilt Down
    a : Pan Left       d : Pan Right
    x : Center Camera

  CUSTOM NUMPAD ACTIONS:
    [1 / Numpad 1] : Save Map
    [2 / Numpad 2] : Toggle Vision Pipeline (YOLO-World)
    [3 / Numpad 3] : Sequential Goal Pose (Drive Distance Sequence)

  Press CTRL+C to exit.
===================================================================
"""


class TeleopInterfaceNode(Node):
    def __init__(self):
        super().__init__('teleop_interface')

        # Parameters
        self.declare_parameter('lin_speed', 0.25)    # Default linear speed m/s
        self.declare_parameter('ang_speed', 0.8)     # Default angular speed rad/s
        self.declare_parameter('lin_step', 0.05)
        self.declare_parameter('ang_step', 0.1)
        self.declare_parameter('lin_min', 0.05)
        self.declare_parameter('lin_max', 0.5)
        self.declare_parameter('ang_min', 0.2)
        self.declare_parameter('ang_max', 2.0)
        self.declare_parameter('dashboard_url', 'http://127.0.0.1:8080')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('camera_cmd_topic', '/camera/cmd_vel')

        gp = lambda n: self.get_parameter(n).value
        self.lin_speed = float(gp('lin_speed'))
        self.ang_speed = float(gp('ang_speed'))
        self.lin_step = float(gp('lin_step'))
        self.ang_step = float(gp('ang_step'))
        self.lin_min, self.lin_max = float(gp('lin_min')), float(gp('lin_max'))
        self.ang_min, self.ang_max = float(gp('ang_min')), float(gp('ang_max'))
        self.dash_url = str(gp('dashboard_url')).rstrip('/')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, gp('cmd_vel_topic'), 10)
        self.cam_pub = self.create_publisher(Twist, gp('camera_cmd_topic'), 10)

        # State tracking
        self.target_lin = 0.0
        self.target_ang = 0.0
        self.seq_running = False

        self.get_logger().info("Teleop Interface Node Ready.")

    def update_speeds(self, d_lin, d_ang):
        if d_lin != 0.0:
            self.lin_speed = round(max(self.lin_min, min(self.lin_max, self.lin_speed + d_lin)), 3)
            self.get_logger().info(f"Linear Speed -> {self.lin_speed:.2f} m/s")
        if d_ang != 0.0:
            self.ang_speed = round(max(self.ang_min, min(self.ang_max, self.ang_speed + d_ang)), 3)
            self.get_logger().info(f"Angular Speed -> {self.ang_speed:.2f} rad/s")

    def publish_bot_twist(self, x, z):
        self.target_lin = x
        self.target_ang = z
        cmd = Twist()
        cmd.linear.x = float(x)
        cmd.angular.z = float(z)
        self.cmd_pub.publish(cmd)

    def publish_camera_cmd(self, pan_rate, tilt_rate):
        cmd = Twist()
        cmd.angular.z = float(pan_rate)   # Pan rate
        cmd.angular.y = float(tilt_rate)  # Tilt rate
        self.cam_pub.publish(cmd)
        self.get_logger().info(f"Camera Command -> Pan Rate: {pan_rate:.2f}, Tilt Rate: {tilt_rate:.2f}")

    # ── Custom Numpad 1: Save Map ─────────────────────────────────────────────
    def action_save_map(self):
        self.get_logger().info("Numpad 1 Pressed: Initiating Map Save...")
        threading.Thread(target=self._post_dashboard, args=('/api/save_map', {'name': ''}, 'Save Map'), daemon=True).start()

    # ── Custom Numpad 2: Toggle Vision Pipeline ──────────────────────────────
    def action_toggle_vision(self):
        self.get_logger().info("Numpad 2 Pressed: Toggling Vision Pipeline (YOLO-World)...")
        threading.Thread(target=self._post_dashboard, args=('/api/vision', {'toggle': True}, 'Toggle Vision'), daemon=True).start()

    # ── Custom Numpad 3: Sequential Goal Pose (Drive Distance Trajectory) ────
    def action_sequential_goal_pose(self):
        if self.seq_running:
            self.get_logger().warn("Sequential goal execution already in progress!")
            return
        self.get_logger().info("Numpad 3 Pressed: Starting Sequential Goal Pose Trajectory...")
        threading.Thread(target=self._run_sequential_trajectory, daemon=True).start()

    def _post_dashboard(self, endpoint, payload, desc):
        try:
            req = urllib.request.Request(
                self.dash_url + endpoint,
                data=json.dumps(payload).encode(),
                headers={'Content-Type': 'application/json'}
            )
            with urllib.request.urlopen(req, timeout=4.0) as resp:
                data = json.loads(resp.read() or b'{}')
                self.get_logger().info(f"Action '{desc}' dashboard response: {data}")
        except Exception as e:
            self.get_logger().error(f"Action '{desc}' failed via dashboard ({e}).")

    def _run_sequential_trajectory(self):
        """
        Executes a sequence of goal poses & distances inspired by drive_distance_node.py.
        Sequence:
          1. Forward 1.0 m
          2. Rotate 90 deg Left
          3. Forward 1.0 m
          4. Rotate 90 deg Right
        """
        self.seq_running = True
        sequence = [
            ("Forward 1.0m", 1.0, 0.0),
            ("Turn Left 90 deg", 0.0, math.pi / 2.0),
            ("Forward 1.0m", 1.0, 0.0),
            ("Turn Right 90 deg", 0.0, -math.pi / 2.0),
        ]

        try:
            for desc, dist, angle in sequence:
                if not rclpy.ok():
                    break
                self.get_logger().info(f"[Sequence] Executing stage: {desc}")
                
                # Execute linear move
                if abs(dist) > 0.001:
                    self._execute_trapezoidal_move(distance=dist, max_v=self.lin_speed, is_rotation=False)
                # Execute angular turn
                if abs(angle) > 0.001:
                    self._execute_trapezoidal_move(distance=angle, max_v=self.ang_speed, is_rotation=True)

                time.sleep(0.5)

            self.get_logger().info("[Sequence] Sequential Goal Pose execution complete!")
        except Exception as err:
            self.get_logger().error(f"[Sequence] Execution error: {err}")
        finally:
            self.publish_bot_twist(0.0, 0.0)
            self.seq_running = False

    def _execute_trapezoidal_move(self, distance, max_v, is_rotation=False):
        """Reference trajectory profile generator from drive_distance_node.py"""
        accel = 0.2
        decel = 0.2
        abs_d = abs(distance)
        sign = 1.0 if distance >= 0 else -1.0

        d_acc = (max_v ** 2) / (2.0 * accel)
        d_dec = (max_v ** 2) / (2.0 * decel)

        if (d_acc + d_dec) <= abs_d:
            v_peak = max_v
            t_a = v_peak / accel
            t_d = v_peak / decel
            t_c = (abs_d - (d_acc + d_dec)) / v_peak
            T_total = t_a + t_c + t_d
        else:
            v_peak = math.sqrt(2.0 * abs_d * accel * decel / (accel + decel))
            t_a = v_peak / accel
            t_d = v_peak / decel
            t_c = 0.0
            T_total = t_a + t_d

        start_t = time.monotonic()
        rate = 20.0
        dt = 1.0 / rate

        while rclpy.ok():
            elapsed = time.monotonic() - start_t
            if elapsed >= T_total:
                break

            if elapsed < t_a:
                v = accel * elapsed
            elif elapsed < (t_a + t_c):
                v = v_peak
            else:
                t_dec = elapsed - (t_a + t_c)
                v = v_peak - decel * t_dec

            vel = sign * max(0.0, v)
            if is_rotation:
                self.publish_bot_twist(0.0, vel)
            else:
                self.publish_bot_twist(vel, 0.0)

            time.sleep(dt)

        self.publish_bot_twist(0.0, 0.0)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            rlist_seq, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist_seq:
                seq = sys.stdin.read(2)
                key += seq
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopInterfaceNode()
    chmod_x = lambda: None

    settings = termios.tcgetattr(sys.stdin)
    print(msg_banner)

    # Key mappings for standard ROS teleop bot movement:
    move_bindings = {
        'i': (1.0, 0.0),      # Forward
        'k': (0.0, 0.0),      # Stop
        ',': (-1.0, 0.0),     # Backward
        'j': (0.0, 1.0),      # Turn Left
        'l': (0.0, -1.0),     # Turn Right
        'u': (1.0, 1.0),      # Forward-Left
        'o': (1.0, -1.0),     # Forward-Right
        'm': (-1.0, 1.0),     # Backward-Left
        '.': (-1.0, -1.0),    # Backward-Right
        ' ': (0.0, 0.0),      # Emergency Stop
    }

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)
            if not key:
                continue

            # Exit
            if key == '\x03' or key == '\x11':  # CTRL+C or CTRL+Q
                break

            # ── Standard ROS Bot Teleop Keys ─────────────────────────────────
            if key in move_bindings:
                lin_mult, ang_mult = move_bindings[key]
                node.publish_bot_twist(lin_mult * node.lin_speed, ang_mult * node.ang_speed)

            # ── Speed Adjustments ─────────────────────────────────────────────
            elif key == 'q':
                node.update_speeds(+node.lin_step, 0.0)
            elif key == 'z':
                node.update_speeds(-node.lin_step, 0.0)
            elif key == 'e':
                node.update_speeds(0.0, +node.ang_step)
            elif key == 'c':
                node.update_speeds(0.0, -node.ang_step)

            # ── WASD Camera Control ───────────────────────────────────────────
            elif key == 'w':
                node.publish_camera_cmd(pan_rate=0.0, tilt_rate=0.5)    # Tilt Up
            elif key == 's':
                node.publish_camera_cmd(pan_rate=0.0, tilt_rate=-0.5)   # Tilt Down
            elif key == 'a':
                node.publish_camera_cmd(pan_rate=0.5, tilt_rate=0.0)    # Pan Left
            elif key == 'd':
                node.publish_camera_cmd(pan_rate=-0.5, tilt_rate=0.0)   # Pan Right
            elif key == 'x':
                node.publish_camera_cmd(pan_rate=0.0, tilt_rate=0.0)    # Reset/Center

            # ── Custom Numpad Actions ─────────────────────────────────────────
            elif key == '1' or key == '\x1bOP':  # 1 or Numpad 1
                node.action_save_map()
            elif key == '2' or key == '\x1bOQ':  # 2 or Numpad 2
                node.action_toggle_vision()
            elif key == '3' or key == '\x1bOR':  # 3 or Numpad 3
                node.action_sequential_goal_pose()

    except Exception as e:
        print(f"Teleop interface error: {e}")
    finally:
        node.publish_bot_twist(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
