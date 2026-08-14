#!/usr/bin/env python3
"""
amr4_driver_node.py — AMR4 Motor Driver Node
=============================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Hardware : Arduino Mega 2560 running DriveMaster.ino
           Connected via USB-Serial (e.g. /dev/ttyACM0)

What this node does
-------------------
1. Subscribes to /cmd_vel (geometry_msgs/Twist) from either:
     • joy_teleop_node  (gamepad controller)
     • Nav2             (autonomous navigation)

2. Converts Twist to an ASCII command and writes it to the Arduino over serial:
     • If angular.z ≈ 0 (translating only):
         "HDRIVE,vy,vx,current_heading_deg\\n"
         The Arduino's heading-hold PID locks the heading so the bot does NOT
         drift sideways or rotate during pure translation (mecanum wheel scrub).
         The current heading is latched from the /imu topic.
     • If |angular.z| > OMEGA_THRESHOLD (intentional rotation):
         "DRIVE,vy,vx,omega\\n"
         Plain drive — heading-hold is cleared on the Arduino side automatically.

3. Reads ASCII telemetry lines back from the Arduino and re-publishes:
     • /odom_raw  (std_msgs/Float32MultiArray)  [W1..W4 actual RPM, 4 values]
     • /driver/status (std_msgs/String)          [raw telemetry string for debug]

   NOTE: IMU data is NOT re-published from this node. The dedicated imu_node.py
   reads BNO055 JSON from its own serial port (/dev/ttyACM1) and publishes /imu.
   This node only subscribes to /imu to latch the current heading for HDRIVE.

4. Buffer management: flushes the serial input buffer every 1 second to prevent
   stale telemetry from accumulating (the Arduino sends telemetry at 10 Hz,
   which at 115200 baud slowly fills the OS buffer if we don't drain it).

Serial protocol (DriveMaster.ino)
----------------------------------
  HOST → ARDUINO  : "HDRIVE,vy,vx,heading_deg\\n"   (translating)
                     "DRIVE,vy,vx,omega\\n"           (rotating)
                     vy    = linear.y  (m/s, left strafe positive)
                     vx    = linear.x  (m/s, forward positive)
                     omega = angular.z (rad/s, CCW positive)
                     heading_deg = absolute target heading from BNO055 (0..360)
  ARDUINO → HOST  : CSV telemetry line every 100 ms, fields include
                     W1_SP, W1_RPM, W2_SP, W2_RPM, W3_SP, W3_RPM,
                     W4_SP, W4_RPM, HDG, ROLL, PITCH, GX,GY,GZ,
                     AX,AY,AZ, HHOLD, HTGT, CAL …

Parameters (all ROS 2 parameters, set in launch file)
------------------------------------------------------
  serial_port      (str)   : e.g. "/dev/ttyACM0"
  baud_rate        (int)   : must match BAUD_HOST in Config.h (default 115200)
  cmd_vel_topic    (str)   : input topic (default "/cmd_vel_safe")
  cmd_timeout      (float) : seconds without a cmd_vel before sending STOP
                             (default 0.5 s — safety watchdog)
  max_send_rate    (float) : max Hz to write commands to Arduino (default 5.0)
                             Lower = less buffer pressure at 115200 baud.
  omega_threshold  (float) : |angular.z| above this → use DRIVE, not HDRIVE
  odom_raw_topic   (str)   : topic for raw encoder output (default "/encoder")
                             (default 0.05 rad/s)
  frame_id         (str)   : frame for published messages (default "base_footprint")
  flush_rate       (float) : Hz at which to flush the serial input buffer
                             (default 1.0 — prevents slow stale-telemetry backlog)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, String


# ──────────────────────────────────────────────────────────────────────────────
# Helper: parse a "KEY:VALUE" telemetry line into a dict
# ──────────────────────────────────────────────────────────────────────────────
def _parse_telemetry(line: str) -> dict:
    """
    Parse a DriveMaster telemetry CSV line into a dict.
    Example:  "T:12345,W1_SP:0.0,W1_RPM:0.0000, ..."
    Returns {} on parse error.
    """
    result = {}
    try:
        for token in line.strip().split(","):
            if ":" in token:
                k, v = token.split(":", 1)
                result[k.strip()] = v.strip()
    except Exception:
        pass
    return result


# ──────────────────────────────────────────────────────────────────────────────
class AMR4DriverNode(Node):
    """
    ROS 2 Humble node: bridges /cmd_vel ↔ DriveMaster.ino over USB-Serial.

    Drive mode selection:
      • |angular.z| ≤ omega_threshold  →  HDRIVE (heading-hold, no drift)
      • |angular.z| >  omega_threshold  →  DRIVE  (free rotation)

    Publishes /odom_raw as Float32MultiArray([rpm1, rpm2, rpm3, rpm4]).
    Does NOT re-publish IMU data — imu_node.py owns /imu.
    """

    def __init__(self):
        super().__init__("amr4_driver_node")

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter("serial_port",     "/dev/ttyACM0")
        self.declare_parameter("baud_rate",        115200)
        self.declare_parameter("cmd_vel_topic",    "/cmd_vel_safe")
        self.declare_parameter("cmd_timeout",      0.5)    # seconds
        self.declare_parameter("max_send_rate",    5.0)    # Hz — 5 Hz to prevent buffer buildup
        self.declare_parameter("omega_threshold",  0.05)   # rad/s
        self.declare_parameter("frame_id",         "base_footprint")
        self.declare_parameter("flush_rate",       1.0)    # Hz — periodic RX flush
        self.declare_parameter("encoder_topic",    "/encoder")

        self._port_name       = self.get_parameter("serial_port").value
        self._baud            = self.get_parameter("baud_rate").value
        self._cmd_topic       = self.get_parameter("cmd_vel_topic").value
        self._cmd_timeout     = self.get_parameter("cmd_timeout").value
        self._max_send_rate   = self.get_parameter("max_send_rate").value
        self._omega_threshold = self.get_parameter("omega_threshold").value
        self._frame_id        = self.get_parameter("frame_id").value
        self._flush_rate      = self.get_parameter("flush_rate").value
        self._encoder_topic   = self.get_parameter("encoder_topic").value

        self._min_send_interval = (
            1.0 / self._max_send_rate if self._max_send_rate > 0 else 0.0
        )

        # ── Internal state ────────────────────────────────────────────────────
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._last_cmd_time = time.monotonic()
        self._last_serial_write_time = 0.0
        self._last_sent_cmd: tuple | None = None
        self._stopped = False

        # Current heading latched from /imu (degrees, BNO055 frame 0..360).
        # Starts at 0.0 — used for HDRIVE target. Updated from /imu callback.
        self._current_heading_deg: float = 0.0
        self._imu_received: bool = False     # True once we have a real heading

        # ── Serial open (with retry) ──────────────────────────────────────────
        self._open_serial()

        # ── QoS profiles ─────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ───────────────────────────────────────────────────────
        # /encoder: raw 4-wheel RPMs straight from Arduino telemetry.
        # odom_node subscribes here, runs mecanum FK and publishes /odom_raw.
        encoder_topic = self.get_parameter("encoder_topic").value
        self._pub_encoder = self.create_publisher(
            Float32MultiArray, encoder_topic, 10
        )
        # /driver/status: raw telemetry string for debugging
        self._pub_status = self.create_publisher(
            String, "/driver/status", 10
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        # /cmd_vel: motion commands (from safety_stop or teleop)
        self._sub_cmd = self.create_subscription(
            Twist,
            self._cmd_topic,
            self._cmd_vel_callback,
            10,
        )
        # /imu: latch current heading from imu_node for HDRIVE
        self._sub_imu = self.create_subscription(
            Imu,
            "/imu",
            self._imu_callback,
            best_effort_qos,
        )

        # ── Watchdog timer (fires every cmd_timeout/2 seconds) ───────────────
        watchdog_period = max(0.1, self._cmd_timeout / 2.0)
        self._watchdog_timer = self.create_timer(
            watchdog_period, self._watchdog_callback
        )

        # ── Periodic serial RX flush timer ───────────────────────────────────
        # Drains stale unread telemetry bytes so the OS buffer never fills up.
        # At 115200 baud + 10 Hz telemetry (~150 bytes/frame) the buffer would
        # reach the OS limit in ~30 s without this flush. At 5 Hz cmd rate the
        # output side is already light; this keeps the input side clean.
        flush_period = 1.0 / self._flush_rate if self._flush_rate > 0 else 1.0
        self._flush_timer = self.create_timer(flush_period, self._flush_serial_buffer)

        # ── Serial reader thread ──────────────────────────────────────────────
        self._reader_thread = threading.Thread(
            target=self._serial_reader_loop, daemon=True
        )
        self._reader_thread.start()

        self.get_logger().info(
            f"AMR4 driver node started  port={self._port_name}  baud={self._baud}"
        )
        self.get_logger().info(
            f"cmd_vel topic='{self._cmd_topic}'  "
            f"cmd_timeout={self._cmd_timeout:.1f}s  "
            f"max_send_rate={self._max_send_rate:.1f} Hz  "
            f"omega_threshold={self._omega_threshold:.3f} rad/s"
        )
        self.get_logger().info(
            "Drive mode: HDRIVE (heading-lock) when |omega| < threshold, "
            "DRIVE (free rotate) otherwise."
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _open_serial(self):
        """Open (or re-open) the serial port.  Retries every 3 s on failure."""
        while rclpy.ok():
            try:
                self._serial = serial.Serial(
                    self._port_name,
                    self._baud,
                    timeout=1.0,
                )
                time.sleep(2.0)  # let Arduino reset after DTR toggle
                # Flush any boot/banner bytes so telemetry starts clean
                try:
                    self._serial.reset_input_buffer()
                except Exception:
                    pass
                self.get_logger().info(
                    f"Serial port {self._port_name} opened at {self._baud} baud"
                )
                return
            except serial.SerialException as exc:
                self.get_logger().error(
                    f"Cannot open {self._port_name}: {exc}  — retrying in 3 s"
                )
                time.sleep(3.0)

    def _write_cmd(self, cmd: str):
        """Send a newline-terminated command string to the Arduino (thread-safe)."""
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write((cmd + "\n").encode())
                    self._serial.flush()
                except serial.SerialException as exc:
                    self.get_logger().error(f"Serial write error: {exc}")
                    self._serial = None

    def _flush_serial_buffer(self):
        """
        Periodically flush the serial RX input buffer.

        The Arduino sends telemetry at 10 Hz (~150 bytes/frame = 1500 B/s).
        The reader thread processes lines as fast as they arrive, so under
        normal operation the buffer stays near zero.  However, if the reader
        thread falls behind (high CPU, serial hiccup) the OS buffer fills and
        old commands begin to pile up.  Calling reset_input_buffer() once per
        second discards any bytes that the reader has not yet consumed,
        preventing the classic 'commands arrive late in bursts after silence'
        symptom that was observed at 10 Hz cmd_vel.
        """
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.reset_input_buffer()
                except Exception:
                    pass

    def _send_hdrive(self, vx: float, vy: float, heading_deg: float):
        """
        Send HDRIVE command — Arduino locks to heading_deg while translating.

        DriveMaster.ino mecanumIK signature:  mecanumIK(vy, vx, omega)
          vy          → linear.y  (strafe left positive)
          vx          → linear.x  (forward positive)
          heading_deg → absolute BNO055 heading target (0..360 deg)
        """
        cmd = f"HDRIVE,{vy:.4f},{vx:.4f},{heading_deg:.2f}"
        self._write_cmd(cmd)

    def _send_drive(self, vx: float, vy: float, omega: float):
        """
        Send plain DRIVE command — no heading-hold.

        DriveMaster.ino mecanumIK signature:  mecanumIK(vy, vx, omega)
          vy    → linear.y  (strafe left positive)
          vx    → linear.x  (forward positive)
          omega → angular.z (CCW positive, rad/s)
        """
        cmd = f"DRIVE,{vy:.4f},{vx:.4f},{omega:.4f}"
        self._write_cmd(cmd)

    def _send_stop(self):
        """Send a zero-velocity DRIVE command (robot stops, heading-hold cleared)."""
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.reset_output_buffer()
                except Exception:
                    pass
        self._write_cmd("DRIVE,0.0000,0.0000,0.0000")

    # ──────────────────────────────────────────────────────────────────────────
    # /imu callback — latch heading for HDRIVE
    # ──────────────────────────────────────────────────────────────────────────
    def _imu_callback(self, msg: Imu):
        """
        Extract yaw from the imu_node quaternion and convert to BNO055-frame
        degrees (0..360, increasing clockwise) for HDRIVE.

        imu_node.py publishes orientation in ROS convention (CCW positive).
        BNO055 heading is CW from North = mathematical (CCW) yaw negated.
        We simply take the ROS yaw and convert: heading = (-yaw_rad) mod 2pi → deg.
        """
        q = msg.orientation
        # Quaternion → yaw (ROS CCW convention, radians)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(siny_cosp, cosy_cosp)  # -pi..pi

        # Convert to BNO055 heading convention: 0=forward, increasing clockwise
        heading_deg = (-math.degrees(yaw_rad)) % 360.0
        self._current_heading_deg = heading_deg
        self._imu_received = True

    # ──────────────────────────────────────────────────────────────────────────
    # /cmd_vel callback
    # ──────────────────────────────────────────────────────────────────────────
    def _cmd_vel_callback(self, msg: Twist):
        """
        Receive a Twist from joystick / Nav2 and forward to the Arduino.

        Selects drive mode:
          HDRIVE — when |angular.z| ≤ omega_threshold (translation-dominant).
                   Uses the current IMU heading so Arduino actively prevents drift.
          DRIVE  — when |angular.z| >  omega_threshold (intentional rotation).
                   Plain open-loop omega, heading-hold cleared on Arduino side.
        """
        now = time.monotonic()
        self._last_cmd_time = now
        self._stopped = False

        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z

        is_stop_cmd = (vx == 0.0 and vy == 0.0 and omega == 0.0)

        # Deduplicate: skip if same command as last sent
        if is_stop_cmd:
            current_cmd = (0.0, 0.0, 0.0, "STOP")
        elif abs(omega) > self._omega_threshold:
            current_cmd = (round(vx, 4), round(vy, 4), round(omega, 4), "DRIVE")
        else:
            # For HDRIVE, heading changes each callback so we can't deduplicate
            # purely on velocity — always send at the rate-limiter cadence.
            current_cmd = (round(vx, 4), round(vy, 4), 0.0, "HDRIVE")

        # Stop commands always go through immediately (safety)
        if not is_stop_cmd:
            if current_cmd == self._last_sent_cmd:
                return  # same DRIVE/STOP command, skip

            # Rate-limiting for non-stop commands
            if (now - self._last_serial_write_time) < self._min_send_interval:
                return

        self._last_sent_cmd = current_cmd
        self._last_serial_write_time = now

        if is_stop_cmd:
            self._send_stop()
        elif abs(omega) > self._omega_threshold:
            # Intentional rotation — use plain DRIVE
            self._send_drive(vx, vy, omega)
        else:
            # Pure translation — use HDRIVE with current heading
            # If we haven't received an IMU message yet, fall back to DRIVE
            if not self._imu_received:
                self.get_logger().warn(
                    "No /imu data yet — using DRIVE instead of HDRIVE. "
                    "Check imu_node.py is running.",
                    throttle_duration_sec=5.0,
                )
                self._send_drive(vx, vy, omega)
            else:
                self._send_hdrive(vx, vy, self._current_heading_deg)

    # ──────────────────────────────────────────────────────────────────────────
    # Watchdog: stop the robot if /cmd_vel goes silent
    # ──────────────────────────────────────────────────────────────────────────
    def _watchdog_callback(self):
        """Send STOP if no cmd_vel has arrived within cmd_timeout seconds."""
        age = time.monotonic() - self._last_cmd_time
        if age > self._cmd_timeout and not self._stopped:
            self.get_logger().warn(
                f"No cmd_vel for {age:.2f}s — sending STOP to Arduino"
            )
            self._send_stop()
            self._stopped = True

        # Try to re-open serial if it dropped
        if self._serial is None or not self._serial.is_open:
            self.get_logger().warn("Serial port closed — attempting reconnect …")
            self._open_serial()

    # ──────────────────────────────────────────────────────────────────────────
    # Serial reader thread — reads telemetry back from Arduino
    # ──────────────────────────────────────────────────────────────────────────
    def _serial_reader_loop(self):
        """
        Background thread: read lines from the Arduino and re-publish
        wheel RPMs as /odom_raw for the odom_node.

        IMU data in the telemetry is intentionally ignored here.
        The dedicated imu_node.py is the sole publisher of /imu.
        """
        while rclpy.ok():
            # Guard: wait for a valid port
            if self._serial is None or not self._serial.is_open:
                time.sleep(0.1)
                continue

            try:
                with self._serial_lock:
                    raw = self._serial.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial read error: {exc}")
                self._serial = None
                continue

            if not raw:
                continue

            try:
                line = raw.decode("ascii", errors="replace").strip()
            except Exception:
                continue

            if not line:
                continue

            # Publish raw status string (useful for debugging)
            status_msg = String()
            status_msg.data = line
            self._pub_status.publish(status_msg)

            # Only parse lines that look like telemetry (start with "T:")
            if not line.startswith("T:"):
                continue

            data = _parse_telemetry(line)
            if not data:
                continue

            # ── /encoder: raw 4-wheel RPMs from Arduino telemetry ───────────
            # Layout: [rpm_FL, rpm_FR, rpm_RL, rpm_RR]
            #   W1 = FL (front-left),  W2 = FR (front-right)
            #   W3 = RL (rear-left),   W4 = RR (rear-right)
            # odom_node consumes this, runs mecanum FK, and publishes
            # nav_msgs/Odometry on /odom_raw for the EKF to fuse.
            try:
                enc_msg = Float32MultiArray()
                enc_msg.data = [
                    float(data.get("W1_RPM", 0.0)),
                    float(data.get("W2_RPM", 0.0)),
                    float(data.get("W3_RPM", 0.0)),
                    float(data.get("W4_RPM", 0.0)),
                ]
                self._pub_encoder.publish(enc_msg)
            except Exception:
                pass

    # ──────────────────────────────────────────────────────────────────────────
    # Shutdown
    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        """Send STOP and close serial before shutdown."""
        self.get_logger().info("AMR4 driver node shutting down — sending STOP")
        try:
            self._send_stop()
        except Exception:
            pass
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
        super().destroy_node()


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = AMR4DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
