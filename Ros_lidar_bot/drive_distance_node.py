#!/usr/bin/env python3
"""
drive_distance_node.py — Interactive terminal app to drive the AMR4 a precise distance.
========================================================================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble · SSH-friendly terminal UI

Usage (odom accuracy test — no lidar/SLAM/Nav2)
-----------------------------------------------
    # Terminal 1
    ros2 launch Ros_lidar_bot launch_odom_test.launch.py
    # Terminal 2 — confirm /odom is live, then:
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
   profile as Twist commands to /cmd_vel at 20 Hz with angular.z=0.
   At move start it latches the current BNO055 heading on /hdrive/heading
   so amr4_driver sends explicit HDRIVE,vy,vx,heading to the Mega (heading
   held for the whole move, not updated every telemetry tick).

2. Odometry feedback — stop condition, not path tracking
   Travelled distance is measured from odometry and the move ends on early
   stop / overshoot. The commanded direction is fixed at the start of the
   move: this closes the loop on HOW FAR the robot has gone, not on how far
   it has strayed sideways. Heading itself is held by the Arduino HDRIVE PID.

Odometry source
---------------
Prefers the EKF-fused /odom (wheel FK + BNO055 gyro). If the EKF is not
running, it says so and falls back to /odom_raw (wheel FK only). With
neither, it degrades to a time-only profile with the odom guard disabled.

Publishes    : /cmd_vel  (geometry_msgs/Twist)
               /hdrive/heading  (std_msgs/Float64 — latched BNO055 deg, -1 clears)
Subscribes   : /odom          (nav_msgs/Odometry — EKF fused, preferred)
               /odom_raw      (nav_msgs/Odometry — wheel FK, fallback)
               /imu           (sensor_msgs/Imu — BNO055 heading for HDRIVE latch)

Parameters (ROS 2, settable on the command line)
------------------------------------------------
    cmd_topic      (str)   default "/cmd_vel"
    hdrive_heading_topic (str) default "/hdrive/heading"
    odom_topic     (str)   default "/odom"       preferred (EKF fused)
    odom_fallback_topic (str) default "/odom_raw"
    distance       (float) default 0.0    forward metres (non-interactive)
    strafe         (float) default 0.0    lateral metres (non-interactive)
    non_interactive (bool) default false   run one move from distance/strafe and exit
    max_vel        (float) default 0.35   m/s    peak velocity
    accel          (float) default 0.20   m/s²   acceleration rate
    decel          (float) default 0.25   m/s²   deceleration rate (slightly
                                                  harder than accel so it stops
                                                  precisely without coasting)
    rate_hz        (float) default 20.0   Hz     cmd_vel publish rate
    odom_timeout   (float) default 3.0    s      warn if odom silent this long
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
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64


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


def _quat_yaw(q) -> float:
    """ROS yaw (rad, CCW from +X) from a geometry quaternion."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _ros_yaw_to_bno_hdg(yaw_rad: float) -> float:
    """Match amr4_driver: yaw_rad = -radians(HDG)."""
    return (-math.degrees(yaw_rad)) % 360.0


def _odom_to_body(ux: float, uy: float, yaw_rad: float) -> tuple[float, float]:
    """Map-frame unit direction → body-frame vx/vy for HDRIVE translation."""
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    return c * ux + s * uy, -s * ux + c * uy


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
        self._cmd_topic    = self.declare_parameter("cmd_topic",   "/cmd_vel").value
        self._hdrive_topic = self.declare_parameter(
            "hdrive_heading_topic", "/hdrive/heading").value
        # EKF-fused odometry preferred; wheel-FK-only odometry as fallback.
        self._odom_topic   = self.declare_parameter("odom_topic",  "/odom").value
        self._odom_fallback_topic = self.declare_parameter(
            "odom_fallback_topic", "/odom_raw").value
        self._distance     = self.declare_parameter("distance", 0.0).value
        self._strafe       = self.declare_parameter("strafe",   0.0).value
        self._non_interactive = self.declare_parameter(
            "non_interactive", False).value
        self._max_vel      = self.declare_parameter("max_vel",      0.35).value
        self._accel        = self.declare_parameter("accel",        0.20).value
        self._decel        = self.declare_parameter("decel",        0.25).value
        self._rate_hz      = self.declare_parameter("rate_hz",      20.0).value
        self._odom_timeout = self.declare_parameter("odom_timeout", 3.0).value
        self._early_stop_m = self.declare_parameter("early_stop_m", 0.02).value
        self._overshoot_m  = self.declare_parameter("overshoot_m",  0.05).value

        self._dt = 1.0 / self._rate_hz

        # ── Publishers ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._pub_hdrive = self.create_publisher(Float64, self._hdrive_topic, 10)

        # ── Odometry state ────────────────────────────────────────────────────
        self._odom_lock  = threading.Lock()
        self._odom_x: float | None = None
        self._odom_y: float | None = None
        self._odom_yaw: float | None = None
        self._odom_last  = time.monotonic()
        # Topic actually feeding the stop guard, resolved at startup.
        self._odom_source: str | None = None

        # BNO055 heading for explicit HDRIVE latch (from /imu).
        self._imu_lock = threading.Lock()
        self._imu_hdg_deg: float | None = None
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)

        self._sub_odom = None
        self._select_odom_source()

    # ──────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        with self._odom_lock:
            self._odom_x = msg.pose.pose.position.x
            self._odom_y = msg.pose.pose.position.y
            self._odom_yaw = _quat_yaw(msg.pose.pose.orientation)
            self._odom_last = time.monotonic()

    def _imu_cb(self, msg: Imu):
        with self._imu_lock:
            self._imu_hdg_deg = _ros_yaw_to_bno_hdg(_quat_yaw(msg.orientation))

    def _try_odom_topic(self, topic: str, timeout: float) -> bool:
        """Subscribe to *topic* and spin until a pose arrives or time runs out."""
        if self._sub_odom is not None:
            self.destroy_subscription(self._sub_odom)
        with self._odom_lock:
            self._odom_x = None
            self._odom_y = None
            self._odom_yaw = None
        self._sub_odom = self.create_subscription(
            Odometry, topic, self._odom_cb, 10
        )

        deadline = time.monotonic() + timeout
        print(_c(DIM, f"  Waiting for {topic} …"), end="", flush=True)
        dots = 0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            with self._odom_lock:
                if self._odom_x is not None:
                    print(_c(GREEN, " ✓"))
                    self._odom_source = topic
                    return True
            dots += 1
            if dots % 5 == 0:
                print(".", end="", flush=True)
        print()
        return False

    def _select_odom_source(self):
        """
        Use the EKF-fused topic when it is live, otherwise the raw wheel FK.

        The fused pose is the better stop reference because the gyro corrects
        wheel slip, so it is tried first and only given a short window — a
        running EKF publishes at 30 Hz and answers immediately.
        """
        if self._try_odom_topic(self._odom_topic, 2.0):
            print(_c(GREEN, f"  Odometry source: {self._odom_topic}  (EKF fused)"))
            return

        print(_c(YELL, f"  ⚠ {self._odom_topic} not found — falling back to "
                       f"{self._odom_fallback_topic}"))
        print(_c(DIM,  "    (wheel FK only: no gyro correction for slip)"))

        if self._try_odom_topic(self._odom_fallback_topic, 8.0):
            print(_c(YELL, f"  Odometry source: {self._odom_fallback_topic}  (wheel FK)"))
            return

        self._odom_source = None
        print(_c(RED, "\n  ⚠ No odometry at all — time-only stop (odom guard disabled)"))
        print(_c(DIM, "  Debug chain:"))
        print(_c(DIM, "    ros2 topic hz /encoder     # amr4_driver"))
        print(_c(DIM, "    ros2 topic hz /odom_raw    # odom_node"))
        print(_c(DIM, "    ros2 topic hz /odom        # ekf_filter_node"))
        print(_c(DIM, "    ros2 node list | grep -E 'odom_node|ekf'"))

    def _get_odom(self) -> tuple[float | None, float | None, float | None]:
        with self._odom_lock:
            return self._odom_x, self._odom_y, self._odom_yaw

    def _get_imu_hdg(self) -> float | None:
        with self._imu_lock:
            return self._imu_hdg_deg

    def _wait_imu_hdg(self, timeout: float = 3.0) -> float | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            hdg = self._get_imu_hdg()
            if hdg is not None:
                return hdg
            rclpy.spin_once(self, timeout_sec=0.1)
        return None

    def _latch_hdrive_heading(self) -> float | None:
        """Publish BNO055 heading so amr4_driver sends HDRIVE,vy,vx,heading."""
        hdg = self._wait_imu_hdg()
        if hdg is None:
            print(_c(YELL, "  ⚠ /imu not available — HDRIVE will use live HDG from driver"))
            return None
        msg = Float64()
        msg.data = hdg
        for _ in range(3):
            self._pub_hdrive.publish(msg)
            time.sleep(0.02)
        return hdg

    def _clear_hdrive_heading(self):
        msg = Float64()
        msg.data = -1.0
        for _ in range(3):
            self._pub_hdrive.publish(msg)
            time.sleep(0.02)

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

        # Unit vector in the odom-frame (dx, dy) direction
        ux = dx / total_dist
        uy = dy / total_dist

        # Build velocity profile
        p = self._plan_profile(total_dist)

        # Latch start pose from the selected odometry source
        start_x, start_y, start_yaw = self._get_odom()
        if start_x is None:
            start_x, start_y, start_yaw = 0.0, 0.0, 0.0
            print(_c(YELL, "  ⚠ No odom start position — odometry guard disabled."))
            odom_guard = False
        else:
            odom_guard = True

        if start_yaw is None:
            start_yaw = 0.0

        # Body-frame direction for HDRIVE (heading held in map frame).
        bx, by = _odom_to_body(ux, uy, start_yaw)

        latched_hdg = self._latch_hdrive_heading()
        hdg_label = f"{latched_hdg:.1f}°" if latched_hdg is not None else "live HDG"

        print(
            f"\n  {_c(CYAN, '►')} Driving  "
            f"Δx={_c(BOLD, f'{dx:+.3f}')} m  "
            f"Δy={_c(BOLD, f'{dy:+.3f}')} m  "
            f"(dist={_c(BOLD, f'{total_dist:.3f}')} m)  "
            f"{_c(DIM, f'[HDRIVE heading={hdg_label}]')}"
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

        try:
            while rclpy.ok():
                elapsed = time.monotonic() - start_time

                # ── Compute odom displacement ──────────────────────────────────
                if odom_guard:
                    cx, cy, _ = self._get_odom()
                    if cx is not None:
                        odom_dist = math.sqrt(
                            (cx - start_x) ** 2 + (cy - start_y) ** 2
                        )

                # ── Stop conditions ────────────────────────────────────────────
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

                # ── Publish Twist — body-frame HDRIVE translation ───────────────
                speed = self._velocity_at(elapsed, p)
                cmd = Twist()
                cmd.linear.x = bx * speed
                cmd.linear.y = by * speed
                cmd.angular.z = 0.0
                self._pub.publish(cmd)

                # ── Draw progress bar ──────────────────────────────────────────
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

                # ── Spin ROS callbacks then sleep ──────────────────────────────
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(self._dt)
        finally:
            self._clear_hdrive_heading()

        # ── Hard stop ─────────────────────────────────────────────────────────
        print()   # newline after progress bar
        for _ in range(5):
            self._pub.publish(Twist())
            time.sleep(0.02)

        # ── Result summary ────────────────────────────────────────────────────
        if odom_guard:
            cx, cy, _ = self._get_odom()
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
        odom_label = self._odom_source or "none — time-only"
        print(
            f"  cmd_vel topic : {_c(CYAN, self._cmd_topic)}\n"
            f"  hdrive topic  : {_c(CYAN, self._hdrive_topic)}\n"
            f"  odom topic    : {_c(CYAN, odom_label)}\n"
            f"  max_vel       : {_c(BOLD, f'{self._max_vel:.2f}')} m/s\n"
            f"  accel / decel : {self._accel:.2f} / {self._decel:.2f} m/s²\n"
            f"  early stop    : ±{self._early_stop_m*100:.0f} cm from target\n"
        )

        while rclpy.ok():
            # Show current position from odom
            cx, cy, _ = self._get_odom()
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
        if node._non_interactive:
            dx = float(node._distance)
            dy = float(node._strafe)
            if dx == 0.0 and dy == 0.0:
                print(_c(YELL, "non_interactive: distance and strafe are both zero — nothing to do."))
            else:
                node._execute_move(dx, dy)
        else:
            node.run_interactive()
    finally:
        node._clear_hdrive_heading()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
