#!/usr/bin/env python3
"""
drive_distance_node.py — Interactive terminal app to drive the AMR4 a precise distance.
========================================================================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble · SSH-friendly terminal UI

Usage (after full bringup)
--------------------------
    ros2 run Ros_lidar_bot drive_distance

The node opens an interactive prompt:

    ╔══════════════════════════════════════╗
    ║   AMR4  Drive-to-Distance  (SSH)     ║
    ╚══════════════════════════════════════╝
    Current position : x=0.000 m  y=0.000 m

    Enter target offset from current position
      X (forward +, backward -) in metres: 1.5
      Y (left strafe +, right strafe -) in metres: 0.0

    ► Driving  Δx=+1.500 m  Δy=+0.000 m  (dist=1.500 m)
      [ACCEL ████░░░░░░░░░░░░░░░░░░]  0.32 m/s  0.12 m → 1.50 m
      ...
    ✓ Done. Final error: 0.018 m
    Drive another? [y/N]:

How it stops precisely
----------------------
Two complementary mechanisms work together:

1. Time-based trapezoidal velocity profile
   The node computes exact accel / constant / decel phases and sends the
   profile as Twist commands to /cmd_vel_safe at 20 Hz.  This gives a smooth
   commanded stop at the mathematically correct time.

2. Odometry feedback (EKF /odom)
   The node tracks the actual displacement from /odom (the EKF-fused output).
   If the odometry distance reaches within EARLY_STOP_THRESH of the target
   BEFORE the time profile ends, the node sends an immediate stop — catching
   any case where the robot is faster than modelled (wheel slip, wrong params).

   An OVERSHOOT_GUARD also triggers immediately if the robot travels more than
   OVERSHOOT_MARGIN beyond the target — protecting against runaway.

Publishes    : /cmd_vel_safe  (geometry_msgs/Twist)
Subscribes   : /odom          (nav_msgs/Odometry  — EKF output)

Parameters (ROS 2, settable on the command line)
------------------------------------------------
    cmd_topic      (str)   default "/cmd_vel_safe"
    max_vel        (float) default 0.35   m/s    peak velocity
    accel          (float) default 0.20   m/s²   acceleration rate
    decel          (float) default 0.25   m/s²   deceleration rate (slightly
                                                  harder than accel so it stops
                                                  precisely without coasting)
    rate_hz        (float) default 20.0   Hz     cmd_vel publish rate
    odom_timeout   (float) default 3.0    s      warn if /odom silent this long
    early_stop_m   (float) default 0.02   m      stop early if within this of target
    overshoot_m    (float) default 0.05   m      emergency stop if past target by this
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# ──────────────────────────────────────────────────────────────────────────────
# Terminal helpers (ANSI — works fine over SSH)
# ──────────────────────────────────────────────────────────────────────────────
def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m"

BOLD  = "1"
GREEN = "1;32"
CYAN  = "1;36"
YELL  = "1;33"
RED   = "1;31"
DIM   = "2"


def _progress_bar(fraction: float, width: int = 24) -> str:
    """Return an ASCII progress bar like [████░░░░░░░░]."""
    filled = max(0, min(width, int(fraction * width)))
    return "[" + "█" * filled + "░" * (width - filled) + "]"


def _ask_float(prompt: str) -> float:
    """Prompt user for a float.  Loops until valid input."""
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print(_c(YELL, "  ↳ Invalid number — try again."))
        except (EOFError, KeyboardInterrupt):
            print()
            raise


# ──────────────────────────────────────────────────────────────────────────────
class DriveDistanceNode(Node):
    """
    ROS 2 node that drives the AMR4 a user-specified (dx, dy) offset
    using a trapezoidal velocity profile, stopping on:
      • profile completion (time-based)
      • odometry early-stop (within early_stop_m of target)
      • overshoot guard (more than overshoot_m past target)
    """

    def __init__(self):
        super().__init__("drive_distance")

        # ── Parameters ────────────────────────────────────────────────────────
        self._cmd_topic    = self.declare_parameter("cmd_topic",   "/cmd_vel_safe").value
        self._max_vel      = self.declare_parameter("max_vel",      0.35).value
        self._accel        = self.declare_parameter("accel",        0.20).value
        self._decel        = self.declare_parameter("decel",        0.25).value
        self._rate_hz      = self.declare_parameter("rate_hz",      20.0).value
        self._odom_timeout = self.declare_parameter("odom_timeout", 3.0).value
        self._early_stop_m = self.declare_parameter("early_stop_m", 0.02).value
        self._overshoot_m  = self.declare_parameter("overshoot_m",  0.05).value

        self._dt = 1.0 / self._rate_hz

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)

        # ── Odometry state (updated from /odom) ───────────────────────────────
        self._odom_lock  = threading.Lock()
        self._odom_x: float | None = None
        self._odom_y: float | None = None
        self._odom_last  = time.monotonic()

        self._sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        # Wait up to 3 s for the first /odom message so we have a start position
        self._wait_for_odom()

    # ──────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            self._odom_last = time.monotonic()

    def _wait_for_odom(self):
        """Spin until we get the first /odom message (max 5 s)."""
        deadline = time.monotonic() + 5.0
        print(_c(DIM, "  Waiting for /odom …"), end="", flush=True)
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            with self._odom_lock:
                if self._odom_x is not None:
                    print(_c(GREEN, " ✓"))
                    return
        print(_c(YELL, "\n  ⚠ No /odom yet — will use time-only stop (odom guard disabled)"))

    def _get_odom(self) -> tuple[float | None, float | None]:
        with self._odom_lock:
            return self._odom_x, self._odom_y

    # ──────────────────────────────────────────────────────────────────────────
    # Velocity profile
    # ──────────────────────────────────────────────────────────────────────────
    def _plan_profile(self, dist: float) -> dict:
        """
        Compute a trapezoidal (or triangular if dist is very short) velocity
        profile for a total displacement of *dist* metres.

        Returns a dict with keys: v_peak, t_a, t_c, t_d, T
        """
        a, d = self._accel, self._decel
        v_max = self._max_vel

        d_acc = v_max ** 2 / (2.0 * a)
        d_dec = v_max ** 2 / (2.0 * d)

        if d_acc + d_dec <= dist:
            # Trapezoidal — full cruise phase
            v_peak = v_max
            t_a    = v_peak / a
            t_d    = v_peak / d
            t_c    = (dist - d_acc - d_dec) / v_peak
        else:
            # Triangular — too short to reach max_vel
            v_peak = math.sqrt(2.0 * dist * a * d / (a + d))
            t_a    = v_peak / a
            t_d    = v_peak / d
            t_c    = 0.0

        T = t_a + t_c + t_d
        return dict(v_peak=v_peak, t_a=t_a, t_c=t_c, t_d=t_d, T=T)

    def _velocity_at(self, elapsed: float, p: dict) -> float:
        """Return the commanded speed (always positive) at *elapsed* seconds."""
        t_a, t_c, t_d = p["t_a"], p["t_c"], p["t_d"]
        v_peak, T = p["v_peak"], p["T"]

        if elapsed < t_a:
            return self._accel * elapsed
        elif elapsed < t_a + t_c:
            return v_peak
        elif elapsed < T:
            t_dec = elapsed - (t_a + t_c)
            return max(0.0, v_peak - self._decel * t_dec)
        else:
            return 0.0

    def _phase_name(self, elapsed: float, p: dict) -> str:
        t_a, t_c, T = p["t_a"], p["t_c"], p["T"]
        if elapsed < t_a:
            return "ACCEL"
        elif elapsed < t_a + t_c:
            return "CRUISE"
        elif elapsed < T:
            return "DECEL"
        return "DONE"

    # ──────────────────────────────────────────────────────────────────────────
    # Core movement executor
    # ──────────────────────────────────────────────────────────────────────────
    def _execute_move(self, dx: float, dy: float):
        """
        Drive the robot by (dx, dy) metres from its current position.
        Blocks until the move finishes (or is aborted).
        """
        total_dist = math.sqrt(dx * dx + dy * dy)
        if total_dist < 0.001:
            print(_c(DIM, "  (zero distance — skipping)"))
            return

        # Unit vector in the (dx, dy) direction
        ux = dx / total_dist
        uy = dy / total_dist

        # Build velocity profile
        p = self._plan_profile(total_dist)

        # Latch start position from /odom
        start_x, start_y = self._get_odom()
        if start_x is None:
            start_x, start_y = 0.0, 0.0
            print(_c(YELL, "  ⚠ No odom start position — odometry guard disabled."))
            odom_guard = False
        else:
            odom_guard = True

        print(
            f"\n  {_c(CYAN, '►')} Driving  "
            f"Δx={_c(BOLD, f'{dx:+.3f}')} m  "
            f"Δy={_c(BOLD, f'{dy:+.3f}')} m  "
            f"(dist={_c(BOLD, f'{total_dist:.3f}')} m)"
        )
        print(
            f"    Profile: peak={p['v_peak']:.3f} m/s  "
            f"accel={self._accel:.2f} m/s²  "
            f"decel={self._decel:.2f} m/s²  "
            f"total={p['T']:.2f} s"
        )
        print()

        start_time = time.monotonic()
        stop_reason = "profile"
        odom_dist   = 0.0

        while rclpy.ok():
            elapsed = time.monotonic() - start_time

            # ── Compute odom displacement ──────────────────────────────────────
            if odom_guard:
                cx, cy = self._get_odom()
                if cx is not None:
                    odom_dist = math.sqrt(
                        (cx - start_x) ** 2 + (cy - start_y) ** 2
                    )

            # ── Stop conditions ────────────────────────────────────────────────
            # 1. Time profile finished
            if elapsed >= p["T"]:
                stop_reason = "profile"
                break

            # 2. Odom reached target (early stop)
            if odom_guard and odom_dist >= total_dist - self._early_stop_m:
                stop_reason = "odom_early"
                break

            # 3. Overshoot guard (safety)
            if odom_guard and odom_dist >= total_dist + self._overshoot_m:
                stop_reason = "overshoot"
                print(_c(RED, f"\n  ⚠ OVERSHOOT GUARD — stopping! odom={odom_dist:.3f} m > target={total_dist:.3f} m"))
                break

            # ── Compute and publish Twist ──────────────────────────────────────
            speed = self._velocity_at(elapsed, p)
            cmd        = Twist()
            cmd.linear.x = ux * speed
            cmd.linear.y = uy * speed
            self._pub.publish(cmd)

            # ── Draw progress bar ──────────────────────────────────────────────
            frac  = min(1.0, elapsed / p["T"])
            phase = self._phase_name(elapsed, p)
            bar   = _progress_bar(frac)
            print(
                f"\r  {_c(CYAN, phase):20s} {bar}  "
                f"{speed:5.3f} m/s  "
                f"{odom_dist:5.3f} m → {total_dist:.3f} m  ",
                end="",
                flush=True,
            )

            # ── Spin ROS callbacks then sleep ──────────────────────────────────
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self._dt)

        # ── Hard stop ─────────────────────────────────────────────────────────
        print()   # newline after progress bar
        for _ in range(5):
            self._pub.publish(Twist())
            time.sleep(0.02)

        # ── Result summary ────────────────────────────────────────────────────
        if odom_guard:
            cx, cy = self._get_odom()
            if cx is not None:
                odom_dist = math.sqrt((cx - start_x) ** 2 + (cy - start_y) ** 2)
            error = abs(odom_dist - total_dist)
            if stop_reason == "overshoot":
                colour = RED
                verdict = "OVERSHOOT STOP"
            elif error < 0.05:
                colour = GREEN
                verdict = "✓ OK"
            else:
                colour = YELL
                verdict = "⚠ CHECK"
            print(
                f"  {_c(colour, verdict)}  "
                f"target={total_dist:.3f} m  actual={odom_dist:.3f} m  "
                f"error={_c(colour, f'{error:.3f}')} m  "
                f"[stop: {stop_reason}]"
            )
        else:
            print(_c(DIM, f"  Done (time-based). [stop: {stop_reason}]"))

    # ──────────────────────────────────────────────────────────────────────────
    # Interactive REPL
    # ──────────────────────────────────────────────────────────────────────────
    def run_interactive(self):
        """Main interactive loop — runs in the main thread."""
        print()
        print(_c(BOLD, "╔══════════════════════════════════════════════╗"))
        print(_c(BOLD, "║    AMR4  Drive-to-Distance   (SSH terminal)  ║"))
        print(_c(BOLD, "╚══════════════════════════════════════════════╝"))
        print(
            f"  cmd_vel topic : {_c(CYAN, self._cmd_topic)}\n"
            f"  odom topic    : {_c(CYAN, '/odom')}\n"
            f"  max_vel       : {_c(BOLD, f'{self._max_vel:.2f}')} m/s\n"
            f"  accel / decel : {self._accel:.2f} / {self._decel:.2f} m/s²\n"
            f"  early stop    : ±{self._early_stop_m*100:.0f} cm from target\n"
        )

        while rclpy.ok():
            # Show current position from odom
            cx, cy = self._get_odom()
            if cx is not None:
                print(_c(DIM, f"  Current position  x={cx:+.3f} m  y={cy:+.3f} m"))

            print(_c(BOLD, "\n  Enter target offset from current position"))
            print(_c(DIM,  "  (forward = +X,  left strafe = +Y,  Ctrl-C to exit)\n"))

            try:
                dx = _ask_float(_c(CYAN, "    X (forward +, backward -) metres: "))
                dy = _ask_float(_c(CYAN, "    Y (left +, right -)        metres: "))
            except (KeyboardInterrupt, EOFError):
                print(_c(YELL, "\n\n  Exiting drive_distance — sending stop …"))
                for _ in range(5):
                    self._pub.publish(Twist())
                    time.sleep(0.02)
                break

            if dx == 0.0 and dy == 0.0:
                print(_c(DIM, "  (both zero — nothing to do)\n"))
                continue

            # Sanity warning for large distances
            dist = math.sqrt(dx*dx + dy*dy)
            if dist > 5.0:
                print(_c(YELL, f"  ⚠  Large distance ({dist:.2f} m) — make sure the path is clear!"))
                try:
                    confirm = input("  Proceed? [y/N]: ").strip().lower()
                except (EOFError, KeyboardInterrupt):
                    break
                if confirm not in ("y", "yes"):
                    print(_c(DIM, "  Cancelled.\n"))
                    continue

            try:
                self._execute_move(dx, dy)
            except KeyboardInterrupt:
                print(_c(YELL, "\n  Ctrl-C — aborting move, sending stop …"))
                for _ in range(5):
                    self._pub.publish(Twist())
                    time.sleep(0.02)

            print()
            try:
                again = input(_c(BOLD, "  Drive another? [y/N]: ")).strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if again not in ("y", "yes"):
                break

        # Final hard stop on exit
        print(_c(DIM, "\n  Sending final stop commands …"))
        for _ in range(5):
            self._pub.publish(Twist())
            time.sleep(0.02)
        print(_c(GREEN, "  drive_distance exited cleanly.\n"))


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = DriveDistanceNode()
    try:
        node.run_interactive()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
