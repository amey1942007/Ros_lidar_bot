#!/usr/bin/env python3
"""
amr4_driver_node.py — AMR4 Motor Driver + IMU Bridge Node
=============================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Hardware : Arduino Mega 2560 running DriveMaster.ino (ONE Mega — no second Arduino)
           BNO055 IMU connected to the SAME Mega over I2C.
           Connected to the Jetson via USB-Serial on /dev/ttyACM0.

What this node does
-------------------
1. Subscribes to /cmd_vel_safe (geometry_msgs/Twist) and forwards to Arduino:
     • If |angular.z| ≤ omega_threshold (translating only):
         "HDRIVE,vy,vx,current_heading_deg\n"
         Arduino's heading-hold PID (driven by its onboard BNO055) stabilises
         the heading so the bot doesn't rotate during pure mecanum translation.
         current_heading_deg is latched from HDG in the same telemetry stream.
     • If |angular.z| > omega_threshold (intentional rotation):
         "DRIVE,vy,vx,omega\n"
         Plain drive — heading-hold is cleared on the Arduino side.

2. Reads ASCII telemetry back from the Arduino (one CSV line per TELEMETRY_MS).
   The telemetry format is:
     T:<ms>,W1_SP:,W1_RPM:,...,W4_SP:,W4_RPM:,
     LIFT:,RLIFT:,LLIFT:,KFSLIFT:,GRIP:,KFSGRIP:,ARM:,...,
     HHOLD:,HTGT:,
     IMU_OK:1,HDG:,ROLL:,PITCH:,GX:,GY:,GZ:,AX:,AY:,AZ:,CAL:

   This node parses and re-publishes TWO topics from that single UART stream:

   a) /encoder  (std_msgs/Float32MultiArray) — [W1_RPM,W2_RPM,W3_RPM,W4_RPM]
      Consumed by odom_node which runs mecanum FK and publishes /odom_raw.

   b) /imu  (sensor_msgs/Imu) — parsed from IMU_OK/HDG/ROLL/PITCH/GX../AX..
      BNO055 Euler angles are converted to a quaternion (ROS convention).
      Angular velocity in rad/s, linear acceleration in m/s².
      This is the SOLE /imu publisher — there is NO separate imu_node.
      The EKF fuses /odom_raw + /imu → /odom.

3. Buffer management: optional RX flush (flush_rate:=0 by default). Blind
   resets wipe encoder/IMU frames and starve /odom_raw → safety_stop zeros
   /cmd_vel_safe. Only wipe when in_waiting is already huge.

Serial protocol (DriveMaster.ino)
----------------------------------
  HOST → ARDUINO  : "HDRIVE,vy,vx,heading_deg\n"   (translating)
                     "DRIVE,vy,vx,omega\n"           (rotating)
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
  max_send_rate    (float) : max Hz to write commands to Arduino (default 15.0)
                             Latest cmd is queued under rate limit (never dropped).
  omega_threshold  (float) : |angular.z| above this → use DRIVE, not HDRIVE
                             (default 0.05 rad/s)
  encoder_topic    (str)   : topic for raw encoder output (default "/encoder")
  frame_id         (str)   : frame for published messages (default "base_footprint")
  flush_rate       (float) : Hz for backlog-only RX wipe (default 0.0 = off)
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial

from geometry_msgs.msg import Twist
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
    ROS 2 Humble node: bridges /cmd_vel_safe ↔ DriveMaster.ino over USB-Serial.

    Drive mode selection:
      • |angular.z| ≤ omega_threshold  →  HDRIVE (heading-hold, no drift)
      • |angular.z| >  omega_threshold  →  DRIVE  (free rotation)

    Publishes from DriveMaster telemetry:
      /encoder  (Float32MultiArray) — [rpm_FL, rpm_FR, rpm_RL, rpm_RR]
      /imu      (sensor_msgs/Imu)   — BNO055 orientation + gyro + accel

    There is NO separate imu_node — this node is the sole /imu publisher.
    """

    def __init__(self):
        super().__init__("amr4_driver_node")

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter("serial_port",     "/dev/ttyACM0")
        self.declare_parameter("baud_rate",        115200)
        self.declare_parameter("cmd_vel_topic",    "/cmd_vel_safe")
        self.declare_parameter("cmd_timeout",      0.5)    # seconds
        self.declare_parameter("max_send_rate",    15.0)   # Hz
        self.declare_parameter("omega_threshold",  0.05)   # rad/s
        self.declare_parameter("frame_id",         "base_footprint")
        self.declare_parameter("imu_frame_id",     "imu_link")   # frame for /imu
        self.declare_parameter("flush_rate",       0.0)    # Hz (0 = off)
        self.declare_parameter("encoder_topic",    "/encoder")

        self._port_name       = self.get_parameter("serial_port").value
        self._baud            = self.get_parameter("baud_rate").value
        self._cmd_topic       = self.get_parameter("cmd_vel_topic").value
        self._cmd_timeout     = self.get_parameter("cmd_timeout").value
        self._max_send_rate   = self.get_parameter("max_send_rate").value
        self._omega_threshold = self.get_parameter("omega_threshold").value
        self._frame_id        = self.get_parameter("frame_id").value
        self._imu_frame_id    = self.get_parameter("imu_frame_id").value
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
        self._pending_cmd: tuple | None = None  # latest cmd waiting for rate window
        self._last_reconnect_attempt = 0.0

        # Current heading latched from DriveMaster HDG (deg, 0..360 CW).
        self._current_heading_deg: float = 0.0
        self._imu_received: bool = False

        # ── QoS — RELIABLE so odom_node / EKF always match ───────────────────
        self._reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Non-blocking open so launch never hangs without ACM0.
        self._open_serial(blocking=False)

        # ── Publishers ───────────────────────────────────────────────────────
        encoder_topic = self.get_parameter("encoder_topic").value
        self._pub_encoder = self.create_publisher(
            Float32MultiArray, encoder_topic, self._reliable_qos
        )
        self._pub_imu = self.create_publisher(Imu, "/imu", self._reliable_qos)
        self._pub_status = self.create_publisher(
            String, "/driver/status", self._reliable_qos
        )

        # ── Subscribers ──────────────────────────────────────────────────────
        self._sub_cmd = self.create_subscription(
            Twist,
            self._cmd_topic,
            self._cmd_vel_callback,
            self._reliable_qos,
        )

        # ── Watchdog timer ───────────────────────────────────────────────────
        watchdog_period = max(0.1, self._cmd_timeout / 2.0)
        self._watchdog_timer = self.create_timer(
            watchdog_period, self._watchdog_callback
        )

        # Flush pending drive cmds at max_send_rate (never drop the latest).
        send_period = (
            1.0 / self._max_send_rate if self._max_send_rate > 0 else 0.05
        )
        self._send_timer = self.create_timer(send_period, self._flush_pending_cmd)

        # Optional RX flush — disabled by default (flush_rate:=0). Blind flushes
        # were wiping encoder/IMU telemetry and starving /odom_raw.
        if self._flush_rate > 0:
            flush_period = 1.0 / self._flush_rate
            self._flush_timer = self.create_timer(
                flush_period, self._flush_serial_buffer
            )

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
            "Drive mode: HDRIVE when translating, DRIVE when rotating."
        )

    # ──────────────────────────────────────────────────────────────────────────
    # Serial helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _open_serial(self, blocking: bool = True):
        """
        Open (or re-open) the serial port.

        blocking=True  — retry forever (startup only if needed).
        blocking=False — single attempt; return False if port missing so
                         __init__/watchdog never hang the ROS executor.
        """
        while rclpy.ok():
            try:
                self._serial = serial.Serial(
                    self._port_name,
                    self._baud,
                    timeout=0.05,
                )
                # DTR reset pause — only safe off the ROS executor (init/reader).
                time.sleep(2.0)
                try:
                    self._serial.reset_input_buffer()
                except Exception:
                    pass
                self.get_logger().info(
                    f"Serial port {self._port_name} opened at {self._baud} baud"
                )
                return True
            except serial.SerialException as exc:
                self.get_logger().error(
                    f"Cannot open {self._port_name}: {exc}"
                    + ("" if blocking else " — will retry in background")
                )
                if not blocking:
                    self._serial = None
                    return False
                time.sleep(3.0)
        return False

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
        Drop RX only if the OS input buffer is dangerously full.

        Telemetry is ~150 B at 10 Hz. The reader thread consumes it. A blind
        reset_input_buffer() every second also dumps encoder/IMU frames, which
        makes /odom_raw go stale and trips safety_stop. Only wipe when the
        backlog is already unusable (several KB of old lines).
        """
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    if self._serial.in_waiting > 4096:
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
    # /cmd_vel — queue latest; flush at max_send_rate (never drop)
    # ──────────────────────────────────────────────────────────────────────────
    def _cmd_vel_callback(self, msg: Twist):
        """
        Receive a Twist from joystick / Nav2 and queue for Arduino.

        Selects drive mode from which stick teleop used:
          HDRIVE — omega ≈ 0 and (vx or vy) nonzero  → left stick / translate
                   Arduino holds heading via BNO055.
          DRIVE  — |omega| > omega_threshold           → right stick / Nav2 rotate
                   Plain omega; heading-hold cleared.
        """
        now = time.monotonic()
        self._last_cmd_time = now
        self._stopped = False

        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z

        translating = abs(vx) > 1e-6 or abs(vy) > 1e-6
        rotating    = abs(omega) > self._omega_threshold
        is_stop_cmd = (not translating) and (not rotating)

        if is_stop_cmd:
            current_cmd = (0.0, 0.0, 0.0, "STOP")
        elif rotating:
            current_cmd = (round(vx, 4), round(vy, 4), round(omega, 4), "DRIVE")
        else:
            current_cmd = (round(vx, 4), round(vy, 4), 0.0, "HDRIVE")

        # Always keep the newest command; stop flushes immediately.
        self._pending_cmd = current_cmd
        if is_stop_cmd:
            self._flush_pending_cmd(force=True)

    def _flush_pending_cmd(self, force: bool = False):
        """Send the latest pending drive command if the rate window allows."""
        pending = self._pending_cmd
        if pending is None:
            return

        now = time.monotonic()
        mode = pending[3]
        is_stop = mode == "STOP"

        if not force and not is_stop:
            if pending == self._last_sent_cmd:
                self._pending_cmd = None
                return
            if (now - self._last_serial_write_time) < self._min_send_interval:
                return

        self._pending_cmd = None
        self._last_sent_cmd = pending
        self._last_serial_write_time = now

        vx, vy, omega, mode = pending
        if mode == "STOP":
            self._send_stop()
        elif mode == "DRIVE":
            self._send_drive(vx, vy, omega)
        else:
            if not self._imu_received:
                self.get_logger().warn(
                    "No HDG yet — using DRIVE instead of HDRIVE. "
                    "Waiting for BNO055 telemetry from DriveMaster.",
                    throttle_duration_sec=5.0,
                )
                self._send_drive(vx, vy, 0.0)
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
            self._pending_cmd = (0.0, 0.0, 0.0, "STOP")
            self._flush_pending_cmd(force=True)
            self._stopped = True

        # Reconnect is done only in the reader thread — never call
        # _open_serial from here (2s Arduino reset would freeze the executor).
        if self._serial is None or not self._serial.is_open:
            self.get_logger().warn(
                "Serial port closed — reader thread will reconnect",
                throttle_duration_sec=5.0,
            )

    # ──────────────────────────────────────────────────────────────────────────
    # Serial reader thread — reads telemetry back from Arduino
    # ──────────────────────────────────────────────────────────────────────────
    def _serial_reader_loop(self):
        """
        Background thread: read DriveMaster telemetry and publish
        /encoder (wheel RPMs) plus /imu (BNO055).
        """
        while rclpy.ok():
            # Guard: wait for a valid port; try light reconnect from this thread
            if self._serial is None or not self._serial.is_open:
                now = time.monotonic()
                if now - self._last_reconnect_attempt >= 3.0:
                    self._last_reconnect_attempt = now
                    self._open_serial(blocking=False)
                time.sleep(0.1)
                continue

            try:
                # Hold lock only for the short-timeout readline, not 1s.
                with self._serial_lock:
                    raw = self._serial.readline() if self._serial else b""
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

            # ── /imu: BNO055 data parsed from DriveMaster telemetry ─────────
            try:
                imu_ok = int(data.get("IMU_OK", "0"))
                if imu_ok == 1:
                    hdg_deg   = float(data.get("HDG",   0.0))
                    roll_deg  = float(data.get("ROLL",  0.0))
                    pitch_deg = float(data.get("PITCH", 0.0))

                    # Latch HDG for HDRIVE directly (no self-subscribe lag).
                    self._current_heading_deg = hdg_deg
                    self._imu_received = True

                    yaw_rad   = -math.radians(hdg_deg)
                    roll_rad  =  math.radians(roll_deg)
                    pitch_rad =  math.radians(pitch_deg)

                    cy, sy = math.cos(yaw_rad   / 2), math.sin(yaw_rad   / 2)
                    cp, sp = math.cos(pitch_rad / 2), math.sin(pitch_rad / 2)
                    cr, sr = math.cos(roll_rad  / 2), math.sin(roll_rad  / 2)

                    qw = cr * cp * cy + sr * sp * sy
                    qx = sr * cp * cy - cr * sp * sy
                    qy = cr * sp * cy + sr * cp * sy
                    qz = cr * cp * sy - sr * sp * cy

                    gx = math.radians(float(data.get("GX", 0.0)))
                    gy = math.radians(float(data.get("GY", 0.0)))
                    gz = math.radians(float(data.get("GZ", 0.0)))

                    ax = float(data.get("AX", 0.0))
                    ay = float(data.get("AY", 0.0))
                    az = float(data.get("AZ", 0.0))

                    imu_msg = Imu()
                    imu_msg.header.stamp    = self.get_clock().now().to_msg()
                    imu_msg.header.frame_id = self._imu_frame_id

                    imu_msg.orientation.x = qx
                    imu_msg.orientation.y = qy
                    imu_msg.orientation.z = qz
                    imu_msg.orientation.w = qw

                    imu_msg.orientation_covariance = [
                        0.002, 0.0,   0.0,
                        0.0,   0.002, 0.0,
                        0.0,   0.0,   0.005,
                    ]

                    imu_msg.angular_velocity.x = gx
                    imu_msg.angular_velocity.y = gy
                    imu_msg.angular_velocity.z = gz
                    imu_msg.angular_velocity_covariance = [
                        0.003, 0.0,   0.0,
                        0.0,   0.003, 0.0,
                        0.0,   0.0,   0.003,
                    ]

                    imu_msg.linear_acceleration.x = ax
                    imu_msg.linear_acceleration.y = ay
                    imu_msg.linear_acceleration.z = az
                    imu_msg.linear_acceleration_covariance = [
                        0.1,   0.0,   0.0,
                        0.0,   0.1,   0.0,
                        0.0,   0.0,   0.1,
                    ]

                    self._pub_imu.publish(imu_msg)
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
