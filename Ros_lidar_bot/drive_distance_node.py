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

    # One-shot (used by the dashboard):
    ros2 run Ros_lidar_bot drive_distance --ros-args \
        -p non_interactive:=true -p distance:=1.0

    # Differential-drive base (no strafe — Y is disabled):
    ros2 run Ros_lidar_bot drive_distance --ros-args -p holonomic:=false

The node opens an interactive prompt:

    ╔══════════════════════════════════════╗
    ║   AMR4  Drive-to-Distance  (SSH)     ║
    ╚══════════════════════════════════════╝
    Enter target offset from current position
      X (forward +, backward -) metres: 1.0
      Y (left +, right -)        metres: 0.0

    ► Driving  X=+1.000 m  Y=+0.000 m  (dist=1.000 m)
      CRUISE [██████████░░░░░░░░░░░░░░]  cmd 0.35  odom 0.34 m/s  0.412/1.000 m  lat +0.3 cm
      ...
      ✓ OK   odom recorded 0.994 m  (target 1.000 m, error +0.006 m short)

Target frame
------------
X / Y are relative to the robot at the start of the move: X = straight ahead,
Y = to the robot's left. They are NOT odom-frame coordinates, so "X=1" always
means "drive 1 m forward" no matter which way the robot is facing. A pure X
move publishes only linear.x, which is exactly what a differential drive
needs. Y (strafe) needs the mecanum base; set holonomic:=false to disable it.

Motion — closed-loop, rest-to-rest
----------------------------------
Every tick (rate_hz) the commanded speed is recomputed from the distance
still remaining, measured on odometry:

    v_brake = sqrt(2 · decel · remaining)      can still stop in time
    v_land  = final_gain · remaining           soft exponential landing
    v_goal  = clamp(min(max_vel, v_brake, v_land), min_vel, max_vel)

The command moves toward v_goal with acceleration ramped by `jerk`, so the
start is an S-curve rather than a step. Remaining distance is predicted
forward by `odom_latency` to cover the ~10 Hz odometry and wheel-PID lag.
The robot therefore starts and ends at rest, and finishes on what odometry
actually measured rather than on a timer. min_vel keeps the wheels above
the motor deadband during the final creep so the robot never stalls short.

At move start the current BNO055 heading is latched on /hdrive/heading so
amr4_driver sends HDRIVE,vy,vx,heading — the Mega holds that heading for the
whole move.

What odometry is reported
-------------------------
Progress is the odometry displacement projected onto the commanded
direction (along-track). Sideways drift (lateral) and heading change are
shown separately, so drift never inflates the distance. After stopping the
node waits settle_time for the last odometry frames before printing the
final recorded distance.

The odom source is /odom (falls back to /odom_raw). Its publisher is shown
at startup: odom_node = wheel odometry, ekf_filter_node = EKF fused. With no
odometry at all the move runs on the commanded profile only (time-based).

Publishes    : /cmd_vel          (geometry_msgs/Twist)
               /hdrive/heading   (std_msgs/Float64 — latched BNO055 deg, -1 clears)
Subscribes   : /odom             (nav_msgs/Odometry — preferred)
               /odom_raw         (nav_msgs/Odometry — fallback)
               /imu              (sensor_msgs/Imu — BNO055 heading for HDRIVE latch)

Parameters (ROS 2, settable on the command line)
------------------------------------------------
    cmd_topic            (str)   "/cmd_vel"
    hdrive_heading_topic (str)   "/hdrive/heading"
    odom_topic           (str)   "/odom"
    odom_fallback_topic  (str)   "/odom_raw"
    non_interactive      (bool)  false   run one move from distance/strafe and exit
    distance             (float) 0.0  m  forward (non-interactive)
    strafe               (float) 0.0  m  left (non-interactive, holonomic only)
    holonomic            (bool)  true    false = differential drive, no strafe
    max_vel              (float) 0.35 m/s
    accel                (float) 0.20 m/s²
    decel                (float) 0.25 m/s²   braking-curve deceleration
    jerk                 (float) 0.80 m/s³   accel ramp rate (smooth start)
    min_vel              (float) 0.03 m/s    creep speed near the target
    final_gain           (float) 1.5  1/s    landing taper (higher = brisker)
    odom_latency         (float) 0.15 s      odometry + wheel-PID lag compensation
    tolerance_m          (float) 0.01 m      arrival tolerance
    overshoot_m          (float) 0.05 m      abort if odom passes target by this
    odom_timeout         (float) 1.0  s      abort if odom goes silent mid-move
    settle_time          (float) 0.6  s      wait after stop before final reading
    rate_hz              (float) 20.0 Hz     control / cmd_vel rate
"""

import math
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
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


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


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
    Drives the AMR4 a robot-relative (X forward, Y left) offset, rest to rest,
    closing the loop on odometry and reporting the distance odometry recorded.
    """

    def __init__(self):
        super().__init__("drive_distance")

        # ── Parameters ────────────────────────────────────────────────────────
        p = self.declare_parameter
        self._cmd_topic    = p("cmd_topic", "/cmd_vel").value
        self._hdrive_topic = p("hdrive_heading_topic", "/hdrive/heading").value
        self._odom_topic   = p("odom_topic", "/odom").value
        self._odom_fallback_topic = p("odom_fallback_topic", "/odom_raw").value
        self._non_interactive = p("non_interactive", False).value
        self._distance     = p("distance", 0.0).value
        self._strafe       = p("strafe", 0.0).value
        self._holonomic    = p("holonomic", True).value
        self._max_vel      = p("max_vel", 0.35).value
        self._accel        = p("accel", 0.20).value
        self._decel        = p("decel", 0.25).value
        self._jerk         = p("jerk", 0.80).value
        self._min_vel      = p("min_vel", 0.03).value
        self._final_gain   = p("final_gain", 1.5).value
        self._odom_latency = p("odom_latency", 0.15).value
        self._tolerance_m  = p("tolerance_m", 0.01).value
        self._overshoot_m  = p("overshoot_m", 0.05).value
        self._odom_timeout = p("odom_timeout", 1.0).value
        self._settle_time  = p("settle_time", 0.6).value
        self._rate_hz      = p("rate_hz", 20.0).value

        self._dt = 1.0 / self._rate_hz

        # ── Publishers ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Twist, self._cmd_topic, 10)
        self._pub_hdrive = self.create_publisher(Float64, self._hdrive_topic, 10)

        # ── Odometry state ────────────────────────────────────────────────────
        # Latest (x, y, yaw, speed, monotonic receive time) per odom topic.
        self._odom_lock = threading.Lock()
        self._odom_latest: dict[str, tuple] = {}
        # Topic actually feeding the controller, resolved in start().
        self._odom_source: str | None = None
        self._odom_label = "none — time-only"
        for topic in (self._odom_topic, self._odom_fallback_topic):
            self.create_subscription(
                Odometry, topic, lambda m, t=topic: self._odom_cb(t, m), 10
            )

        # BNO055 heading for explicit HDRIVE latch (from /imu).
        self._imu_lock = threading.Lock()
        self._imu_hdg_deg: float | None = None
        self.create_subscription(Imu, "/imu", self._imu_cb, 10)

        # Callbacks run on a background thread so odometry stays fresh while
        # the main thread blocks in input() at the prompt.
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)

    def start(self):
        self._spin_thread.start()
        self._select_odom_source()

    def shutdown(self):
        self._executor.shutdown()

    # ──────────────────────────────────────────────────────────────────────────
    # Odometry / IMU
    # ──────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, topic: str, msg: Odometry):
        sample = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            _quat_yaw(msg.pose.pose.orientation),
            math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
            time.monotonic(),
        )
        with self._odom_lock:
            self._odom_latest[topic] = sample

    def _imu_cb(self, msg: Imu):
        with self._imu_lock:
            self._imu_hdg_deg = _ros_yaw_to_bno_hdg(_quat_yaw(msg.orientation))

    def _try_odom_topic(self, topic: str, timeout: float) -> bool:
        """Wait until a pose arrives on *topic* or time runs out."""
        deadline = time.monotonic() + timeout
        print(_c(DIM, f"  Waiting for {topic} …"), end="", flush=True)
        dots = 0
        while time.monotonic() < deadline:
            with self._odom_lock:
                if topic in self._odom_latest:
                    print(_c(GREEN, " ✓"))
                    self._odom_source = topic
                    return True
            time.sleep(0.2)
            dots += 1
            if dots % 5 == 0:
                print(".", end="", flush=True)
        print()
        return False

    def _describe_source(self, topic: str) -> str:
        """Name the node behind *topic* so the user knows what is measuring."""
        nodes = sorted({i.node_name for i in self.get_publishers_info_by_topic(topic)})
        if not nodes:
            return f"{topic}"
        kind = "EKF fused (wheels + IMU)" if any("ekf" in n for n in nodes) \
            else "wheel odometry"
        return f"{topic}  ← {', '.join(nodes)}  ({kind})"

    def _select_odom_source(self):
        """Use /odom when it is live, otherwise /odom_raw."""
        for topic, timeout in ((self._odom_topic, 3.0),
                               (self._odom_fallback_topic, 5.0)):
            if self._try_odom_topic(topic, timeout):
                self._odom_label = self._describe_source(topic)
                colour = GREEN if topic == self._odom_topic else YELL
                print(_c(colour, f"  Odometry source: {self._odom_label}"))
                return
            print(_c(YELL, f"  ⚠ {topic} not found"))

        self._odom_source = None
        print(_c(RED, "\n  ⚠ No odometry at all — time-only moves (no feedback)"))
        print(_c(DIM, "  Debug chain:"))
        print(_c(DIM, "    ros2 topic hz /encoder     # amr4_driver"))
        print(_c(DIM, "    ros2 topic hz /odom        # odom_node (or EKF)"))
        print(_c(DIM, "    ros2 node list | grep -E 'odom_node|ekf'"))

    def _get_odom(self):
        """(x, y, yaw, speed, receive_time) from the selected source, or Nones."""
        with self._odom_lock:
            sample = self._odom_latest.get(self._odom_source)
        return sample if sample is not None else (None, None, None, 0.0, 0.0)

    def _wait_fresh_odom(self, timeout: float = 2.0) -> bool:
        """True once the selected source has a sample younger than odom_timeout."""
        deadline = time.monotonic() + timeout
        while True:
            stamp = self._get_odom()[4]
            if time.monotonic() - stamp <= self._odom_timeout:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)

    # ──────────────────────────────────────────────────────────────────────────
    # HDRIVE heading latch
    # ──────────────────────────────────────────────────────────────────────────
    def _wait_imu_hdg(self, timeout: float = 3.0) -> float | None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._imu_lock:
                if self._imu_hdg_deg is not None:
                    return self._imu_hdg_deg
            time.sleep(0.1)
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
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _estimate_duration(self, dist: float) -> float:
        """Trapezoid duration — used only for the ETA and the safety timeout."""
        a, d, v = self._accel, self._decel, self._max_vel
        d_acc, d_dec = v * v / (2 * a), v * v / (2 * d)
        if d_acc + d_dec <= dist:
            return v / a + v / d + (dist - d_acc - d_dec) / v
        v_peak = math.sqrt(2.0 * dist * a * d / (a + d))
        return v_peak / a + v_peak / d

    def _send_stop(self, seconds: float = 0.1):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._pub.publish(Twist())
            time.sleep(0.02)

    # ──────────────────────────────────────────────────────────────────────────
    # Core movement executor
    # ──────────────────────────────────────────────────────────────────────────
    def _execute_move(self, dx: float, dy: float):
        """
        Drive dx metres forward and dy metres left of the robot's current pose,
        rest to rest. Blocks until the move finishes or is aborted.
        """
        if not self._holonomic and dy != 0.0:
            print(_c(YELL, f"  ⚠ holonomic:=false — ignoring Y={dy:+.3f} m (no strafe)"))
            dy = 0.0

        total = math.hypot(dx, dy)
        if total < 0.001:
            print(_c(DIM, "  (zero distance — skipping)"))
            return

        # Commanded direction in the robot frame (fixed for the whole move).
        bx, by = dx / total, dy / total

        if self._odom_source is not None and not self._wait_fresh_odom():
            print(_c(RED, f"  ⚠ {self._odom_source} has gone silent — not moving."))
            print(_c(DIM, "    Check: ros2 topic hz /encoder /odom"))
            return

        sx, sy, syaw, _, _ = self._get_odom()
        odom_guard = sx is not None
        if not odom_guard:
            sx, sy, syaw = 0.0, 0.0, 0.0
            print(_c(YELL, "  ⚠ No odometry — moving on the commanded profile only."))
        if syaw is None:
            syaw = 0.0

        # Same direction in the odom frame, for measuring progress.
        c, s = math.cos(syaw), math.sin(syaw)
        ox, oy = c * bx - s * by, s * bx + c * by

        eta = self._estimate_duration(total)
        timeout = 2.0 * eta + 5.0

        latched_hdg = self._latch_hdrive_heading()
        hdg_label = f"{latched_hdg:.1f}°" if latched_hdg is not None else "live HDG"

        print(
            f"\n  {_c(CYAN, '►')} Driving  "
            f"X={_c(BOLD, f'{dx:+.3f}')} m  "
            f"Y={_c(BOLD, f'{dy:+.3f}')} m  "
            f"(dist={_c(BOLD, f'{total:.3f}')} m)  "
            f"{_c(DIM, f'[HDRIVE heading={hdg_label}]')}"
        )
        print(
            f"    max={self._max_vel:.2f} m/s  accel={self._accel:.2f}  "
            f"decel={self._decel:.2f} m/s²  jerk={self._jerk:.2f} m/s³  "
            f"est≈{eta:.1f} s\n"
        )

        v_cmd = 0.0
        a_cur = 0.0
        along = 0.0
        lateral = 0.0
        peak = 0.0
        stop_reason = "arrived"

        t0 = time.monotonic()
        last = t0
        next_tick = t0

        try:
            while rclpy.ok():
                now = time.monotonic()
                tick_dt = now - last
                last = now
                elapsed = now - t0

                # ── Progress along the commanded direction ─────────────────────
                odom_v = 0.0
                if odom_guard:
                    cx, cy, _, odom_v, stamp = self._get_odom()
                    ex, ey = cx - sx, cy - sy
                    along = ex * ox + ey * oy
                    lateral = -ex * oy + ey * ox
                    if now - stamp > self._odom_timeout:
                        stop_reason = "odom_lost"
                        print(_c(RED, f"\n  ⚠ Odometry silent for {now - stamp:.1f} s — stopping!"))
                        break
                else:
                    along += v_cmd * tick_dt

                remaining = total - along

                # ── Stop conditions ────────────────────────────────────────────
                if remaining < -self._overshoot_m:
                    stop_reason = "overshoot"
                    print(_c(RED, f"\n  ⚠ OVERSHOOT — odom {along:.3f} m > target {total:.3f} m"))
                    break
                rem_pred = remaining - (v_cmd * self._odom_latency if odom_guard else 0.0)
                if rem_pred <= self._tolerance_m:
                    stop_reason = "arrived"
                    break
                if elapsed > timeout:
                    stop_reason = "timeout"
                    print(_c(RED, f"\n  ⚠ Timeout after {elapsed:.1f} s — stopping!"))
                    break

                # ── Speed goal from remaining distance ─────────────────────────
                v_brake = math.sqrt(2.0 * self._decel * rem_pred)
                v_land = self._final_gain * rem_pred
                v_goal = max(min(self._max_vel, v_brake, v_land), self._min_vel)

                # ── Jerk-limited approach to the goal ──────────────────────────
                if v_goal > v_cmd:
                    a_cur = min(a_cur + self._jerk * self._dt, self._accel)
                    v_cmd = min(v_cmd + a_cur * self._dt, v_goal)
                else:
                    a_cur = 0.0
                    v_cmd = max(v_goal, v_cmd - 1.5 * self._decel * self._dt)

                if v_cmd >= self._max_vel - 1e-3:
                    phase = "CRUISE"
                elif a_cur > 0.0:
                    phase = "ACCEL"
                elif v_cmd > 2.0 * self._min_vel:
                    phase = "BRAKE"
                else:
                    phase = "LAND"
                peak = max(peak, v_cmd)

                cmd = Twist()
                cmd.linear.x = bx * v_cmd
                cmd.linear.y = by * v_cmd
                cmd.angular.z = 0.0
                self._pub.publish(cmd)

                # ── Live readout ───────────────────────────────────────────────
                bar = _progress_bar(along / total)
                odom_txt = f"odom {odom_v:4.2f}" if odom_guard else "odom  -- "
                print(
                    f"\r  {_c(CYAN, f'{phase:<6}')} {bar}  "
                    f"cmd {v_cmd:4.2f}  {odom_txt} m/s  "
                    f"{along:6.3f}/{total:.3f} m  "
                    f"lat {lateral * 100:+5.1f} cm  ",
                    end="",
                    flush=True,
                )

                # ── Fixed-rate tick ────────────────────────────────────────────
                next_tick += self._dt
                if next_tick < time.monotonic():
                    next_tick = time.monotonic()
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            self._clear_hdrive_heading()

        # ── Stop and let the last odometry frames arrive ─────────────────────
        move_time = time.monotonic() - t0
        print()
        self._send_stop(self._settle_time if odom_guard else 0.1)

        if not odom_guard:
            print(_c(DIM, f"  Done (no odometry — commanded {along:.3f} m). "
                          f"[stop: {stop_reason}]"))
            return

        cx, cy, cyaw, _, _ = self._get_odom()
        ex, ey = cx - sx, cy - sy
        along = ex * ox + ey * oy
        lateral = -ex * oy + ey * ox
        dyaw = math.degrees(_wrap_pi(cyaw - syaw)) if cyaw is not None else 0.0
        error = total - along

        if stop_reason in ("overshoot", "odom_lost", "timeout"):
            colour, verdict = RED, stop_reason.upper()
        elif abs(error) < 0.03:
            colour, verdict = GREEN, "✓ OK"
        else:
            colour, verdict = YELL, "⚠ CHECK"
        side = "short" if error >= 0 else "over"

        print(
            f"  {_c(colour, verdict)}  odom recorded {_c(BOLD, f'{along:.3f}')} m  "
            f"(target {total:.3f} m, error {_c(colour, f'{abs(error):.3f}')} m {side})"
        )
        print(_c(DIM,
            f"    lateral drift {lateral * 100:+.1f} cm  ·  heading Δ {dyaw:+.1f}°  ·  "
            f"peak {peak:.2f} m/s  ·  {move_time:.1f} s  ·  stop: {stop_reason}\n"
            f"    measured by {self._odom_label}"
        ))

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
            f"  odom          : {_c(CYAN, self._odom_label)}\n"
            f"  drive type    : {'mecanum (X + Y)' if self._holonomic else 'differential (X only)'}\n"
            f"  max_vel       : {_c(BOLD, f'{self._max_vel:.2f}')} m/s\n"
            f"  accel / decel : {self._accel:.2f} / {self._decel:.2f} m/s²\n"
            f"  tolerance     : ±{self._tolerance_m * 100:.0f} cm\n"
        )

        while rclpy.ok():
            cx, cy, cyaw, _, _ = self._get_odom()
            if cx is not None:
                print(_c(DIM, f"  Odom pose  x={cx:+.3f} m  y={cy:+.3f} m  "
                              f"yaw={math.degrees(cyaw or 0.0):+.1f}°"))

            print(_c(BOLD, "\n  Enter target offset from the robot's current pose"))
            print(_c(DIM,  "  (X = ahead, Y = robot's left,  Ctrl-C to exit)\n"))

            try:
                dx = _ask_float(_c(CYAN, "    X (forward +, backward -) metres: "))
                dy = (_ask_float(_c(CYAN, "    Y (left +, right -)        metres: "))
                      if self._holonomic else 0.0)
            except (KeyboardInterrupt, EOFError):
                print(_c(YELL, "\n\n  Exiting drive_distance — sending stop …"))
                self._send_stop()
                break

            if dx == 0.0 and dy == 0.0:
                print(_c(DIM, "  (both zero — nothing to do)\n"))
                continue

            dist = math.hypot(dx, dy)
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
                self._send_stop()

            print()
            try:
                again = input(_c(BOLD, "  Drive another? [y/N]: ")).strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if again not in ("y", "yes"):
                break

        print(_c(DIM, "\n  Sending final stop commands …"))
        self._send_stop()
        print(_c(GREEN, "  drive_distance exited cleanly.\n"))


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = DriveDistanceNode()
    try:
        node.start()
        if node._non_interactive:
            dx = float(node._distance)
            dy = float(node._strafe)
            if dx == 0.0 and dy == 0.0:
                print(_c(YELL, "non_interactive: distance and strafe are both zero — nothing to do."))
            else:
                node._execute_move(dx, dy)
        else:
            node.run_interactive()
    except KeyboardInterrupt:
        node._send_stop()
    finally:
        node._clear_hdrive_heading()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
