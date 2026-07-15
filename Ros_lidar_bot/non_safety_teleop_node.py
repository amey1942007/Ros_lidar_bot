#!/usr/bin/env python3
"""
non_safety_teleop_node.py — Hold-to-move keyboard teleop (bypasses safety stop).

Controls:
  Hold W / S        — move forward / reverse at current linear speed
  Hold A / D        — turn left / right at current angular speed
  Arrow UP / DOWN   — increase / decrease linear speed by 0.05 m/s
  Arrow LEFT/RIGHT  — decrease / increase angular speed by 0.1 rad/s
  Space             — instant stop (zeros velocity immediately)
  Ctrl+C            — exit safely (motors zeroed before quit)

Behaviour:
  - While a movement key (W/A/S/D) is held, the robot moves at the set speed.
  - As soon as the key is released (no key pressed within KEY_TIMEOUT), velocity
    drops to zero automatically — just like official teleop_twist_keyboard.
  - Arrow keys adjust the speed setpoint without affecting current motion.
  - Space zeroes velocity regardless of what is held.

Publishes: /cmd_vel_safe (geometry_msgs/Twist)
  Bypasses safety_stop_node — robot will move even near obstacles.
"""

import select
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

# ── Tunables ───────────────────────────────────────────────────────────────────
KEY_TIMEOUT   = 0.20   # seconds — if no movement key within this window → stop
                        # (0.20 s tolerates xterm/system jitter; was 0.12 which caused
                        #  spurious auto-stops during held-key teleop on a loaded RPi)
PUBLISH_HZ    = 20.0   # publish rate for velocity commands
LIN_STEP      = 0.05   # m/s per arrow-key press
ANG_STEP      = 0.1    # rad/s per arrow-key press
LIN_DEFAULT   = 0.2    # starting linear speed (m/s)
ANG_DEFAULT   = 0.5    # starting angular speed (rad/s)
LIN_MIN       = 0.05   # minimum linear speed limit
ANG_MIN       = 0.1    # minimum angular speed limit
LIN_MAX       = 1.0    # hard cap linear speed
ANG_MAX       = 3.0    # hard cap angular speed

BANNER = """
\033[1;36m╔══════════════════════════════════════════════════════╗
║        Non-Safety Teleop  —  Hold to Move            ║
╚══════════════════════════════════════════════════════╝\033[0m

  \033[1mMovement\033[0m  (hold key → move, release → stop):
        \033[1mW\033[0m
    \033[1mA\033[0m   \033[1mS\033[0m   \033[1mD\033[0m        W=forward  S=reverse  A=left  D=right

  \033[1mSpeed adjust\033[0m  (tap to change setpoint):
    \033[1m↑\033[0m / \033[1m↓\033[0m  — linear speed  +/- {lin_step} m/s
    \033[1m→\033[0m / \033[1m←\033[0m  — angular speed +/- {ang_step} rad/s

  \033[1mSpace\033[0m  — instant STOP
  \033[1mCtrl+C\033[0m — exit (motors zeroed)

\033[33m⚠  Safety stop BYPASSED — robot moves near obstacles!\033[0m
""".format(lin_step=LIN_STEP, ang_step=ANG_STEP)


# ── Arrow-key escape sequence reader ──────────────────────────────────────────
def _read_key_raw(fd, timeout=KEY_TIMEOUT):
    """
    Read one keypress from fd.
    Handles 3-byte arrow-key escape sequences: ESC [ A/B/C/D
    Returns a string token:
      'w','a','s','d'         — movement keys
      'UP','DOWN','LEFT','RIGHT' — arrow keys
      'SPACE'                 — space bar
      '\x03'                  — Ctrl+C
      ''                      — timeout / no key
    """
    ready, _, _ = select.select([fd], [], [], timeout)
    if not ready:
        return ''

    ch = sys.stdin.read(1)

    # Arrow keys arrive as 3-byte sequence: ESC [ X
    if ch == '\x1b':
        # Peek for the next two bytes with a short timeout
        r2, _, _ = select.select([fd], [], [], 0.05)
        if not r2:
            return '\x1b'   # bare ESC key
        ch2 = sys.stdin.read(1)
        if ch2 == '[':
            r3, _, _ = select.select([fd], [], [], 0.05)
            if r3:
                ch3 = sys.stdin.read(1)
                return {'A': 'UP', 'B': 'DOWN', 'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, '')
        return ''

    if ch == ' ':
        return 'SPACE'
    return ch   # 'w', 'a', 's', 'd', '\x03', etc.


# ── ROS Node ───────────────────────────────────────────────────────────────────
class NonSafetyTeleop(Node):
    def __init__(self):
        super().__init__('non_safety_teleop')
        self._pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)

        # Speed setpoints (what we drive at when a key is held)
        self._lin_speed  = LIN_DEFAULT   # m/s
        self._ang_speed  = ANG_DEFAULT   # rad/s

        # Current commanded velocity (set by key thread, read by publisher timer)
        self._cmd_linear  = 0.0
        self._cmd_angular = 0.0
        self._lock        = threading.Lock()

        # Timestamp of last movement key press
        self._last_move_time = 0.0
        self._stopped        = True

        # Publisher timer
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish_cb)

    # ── Publish callback (runs on ROS timer) ──────────────────────────────────
    def _publish_cb(self):
        with self._lock:
            lin = self._cmd_linear
            ang = self._cmd_angular

        msg = Twist()
        msg.linear.x  = lin
        msg.angular.z = ang
        self._pub.publish(msg)
        self._print_status(lin, ang)

    # ── Called by key thread to set motion ────────────────────────────────────
    def set_motion(self, linear: float, angular: float):
        with self._lock:
            self._cmd_linear  = linear
            self._cmd_angular = angular
            self._last_move_time = time.monotonic()
            self._stopped = False

    def stop(self):
        with self._lock:
            self._cmd_linear  = 0.0
            self._cmd_angular = 0.0
            self._stopped = True

    def adjust_linear(self, delta: float):
        self._lin_speed = round(
            max(LIN_MIN, min(LIN_MAX, self._lin_speed + delta)), 3)

    def adjust_angular(self, delta: float):
        self._ang_speed = round(
            max(ANG_MIN, min(ANG_MAX, self._ang_speed + delta)), 3)

    @property
    def lin_speed(self):
        return self._lin_speed

    @property
    def ang_speed(self):
        return self._ang_speed

    # ── Status line ───────────────────────────────────────────────────────────
    def _print_status(self, lin: float, ang: float):
        moving = "MOVING" if (abs(lin) > 0.001 or abs(ang) > 0.001) else "STOPPED"
        color  = "\033[32m" if moving == "MOVING" else "\033[33m"
        print(
            f"\r\033[K"
            f"  Lin setpoint: \033[1m{self._lin_speed:.2f}\033[0m m/s  "
            f"Ang setpoint: \033[1m{self._ang_speed:.2f}\033[0m rad/s  │  "
            f"Cmd: lin=\033[1m{lin:+.2f}\033[0m  ang=\033[1m{ang:+.2f}\033[0m  │  "
            f"{color}{moving}\033[0m",
            end="", flush=True
        )


# ── Key-reading loop (runs in background thread) ───────────────────────────────
def _key_loop(node: NonSafetyTeleop, orig_settings):
    """
    Continuously reads keypresses and updates motion commands.
    Movement keys (W/A/S/D): set velocity while held, zero when released.
    Arrow keys: adjust speed setpoints.
    Space: instant stop.
    """
    fd = sys.stdin.fileno()
    tty.setraw(fd)

    try:
        while rclpy.ok():
            key = _read_key_raw(fd, timeout=KEY_TIMEOUT)

            # ── Movement keys (hold to move) ──────────────────────────────────
            if key == 'w':
                node.set_motion(+node.lin_speed, 0.0)

            elif key == 's':
                node.set_motion(-node.lin_speed, 0.0)

            elif key == 'a':
                node.set_motion(0.0, +node.ang_speed)

            elif key == 'd':
                node.set_motion(0.0, -node.ang_speed)

            # ── Speed adjustment (arrow keys) ─────────────────────────────────
            elif key == 'UP':
                node.adjust_linear(+LIN_STEP)

            elif key == 'DOWN':
                node.adjust_linear(-LIN_STEP)

            elif key == 'RIGHT':
                node.adjust_angular(+ANG_STEP)

            elif key == 'LEFT':
                node.adjust_angular(-ANG_STEP)

            # ── Instant stop ──────────────────────────────────────────────────
            elif key == 'SPACE':
                node.stop()

            # ── Exit ──────────────────────────────────────────────────────────
            elif key == '\x03':   # Ctrl+C
                break

            # ── Timeout (no key held) → auto stop ─────────────────────────────
            elif key == '':
                node.stop()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, orig_settings)


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    orig_settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = NonSafetyTeleop()

    print(BANNER)

    # Key reading runs in a daemon thread; ROS spinning on main thread
    key_thread = threading.Thread(
        target=_key_loop, args=(node, orig_settings), daemon=True
    )
    key_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        # Let the final zero-velocity publish go out
        time.sleep(0.15)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)
        print("\n\033[1mTeleop exiting — motors zeroed.\033[0m")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
