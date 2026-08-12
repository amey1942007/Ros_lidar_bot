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
     • Nav2             (autonomous navigation — future)
2. Converts Twist → "DRIVE,vy,vx,omega\\n" ASCII command and writes
   it to the Arduino over serial.  The Arduino's mecanumIK() function
   converts these three values into 4 individual wheel RPM setpoints
   and runs per-wheel PID + feedforward — all kinematics live in the .ino.
3. Reads ASCII telemetry lines back from the Arduino and re-publishes:
     • /wheel_rpms (std_msgs/Float32MultiArray)  [W1..W4 actual RPM]
     • /imu/raw    (sensor_msgs/Imu)             [BNO055 fused data, if available]
     • /driver/status (std_msgs/String)          [raw telemetry string for debug]

Serial protocol (DriveMaster.ino)
----------------------------------
  HOST → ARDUINO  : "DRIVE,vy,vx,omega\\n"
                     vy    = linear.y  (m/s, left strafe positive)
                     vx    = linear.x  (m/s, forward positive)
                     omega = angular.z (rad/s, CCW positive)
  ARDUINO → HOST  : CSV telemetry line every 100 ms, fields include
                     W1_SP, W1_RPM, W2_SP, W2_RPM, W3_SP, W3_RPM,
                     W4_SP, W4_RPM, HDG, ROLL, PITCH, GX,GY,GZ,
                     AX,AY,AZ, HHOLD, HTGT, CAL …

Parameters (all ROS 2 parameters, set in launch file)
------------------------------------------------------
  serial_port   (str)   : e.g. "/dev/ttyACM0"
  baud_rate     (int)   : must match BAUD_HOST in Config.h (default 115200)
  cmd_vel_topic (str)   : input topic (default "/cmd_vel")
  cmd_timeout   (float) : seconds without a cmd_vel before sending STOP
                          (default 0.5 s — safety watchdog)
  publish_imu   (bool)  : re-publish BNO055 data as sensor_msgs/Imu
                          (default True)
  frame_id      (str)   : frame for Imu messages (default "imu_link")
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
from builtin_interfaces.msg import Time as RosTime


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
    """

    def __init__(self):
        super().__init__("amr4_driver_node")

        # ── Declare parameters ────────────────────────────────────────────────
        self.declare_parameter("serial_port",   "/dev/ttyACM0")
        self.declare_parameter("baud_rate",      115200)
        self.declare_parameter("cmd_vel_topic",  "/cmd_vel")
        self.declare_parameter("cmd_timeout",    0.5)   # seconds
        self.declare_parameter("publish_imu",    True)
        self.declare_parameter("frame_id",       "imu_link")

        self._port_name  = self.get_parameter("serial_port").value
        self._baud       = self.get_parameter("baud_rate").value
        self._cmd_topic  = self.get_parameter("cmd_vel_topic").value
        self._cmd_timeout = self.get_parameter("cmd_timeout").value
        self._pub_imu    = self.get_parameter("publish_imu").value
        self._frame_id   = self.get_parameter("frame_id").value

        # ── Internal state ────────────────────────────────────────────────────
        self._serial: serial.Serial | None = None
        self._serial_lock = threading.Lock()
        self._last_cmd_time = time.monotonic()
        self._stopped = False   # tracks whether we've already sent STOP

        # ── Serial open (with retry) ──────────────────────────────────────────
        self._open_serial()

        # ── Publishers ───────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._pub_rpms = self.create_publisher(
            Float32MultiArray, "/wheel_rpms", 10
        )
        self._pub_status = self.create_publisher(
            String, "/driver/status", 10
        )
        if self._pub_imu:
            self._pub_imu_msg = self.create_publisher(
                Imu, "/imu/raw", best_effort_qos
            )

        # ── Subscriber ───────────────────────────────────────────────────────
        self._sub_cmd = self.create_subscription(
            Twist,
            self._cmd_topic,
            self._cmd_vel_callback,
            10,
        )

        # ── Watchdog timer (fires every cmd_timeout/2 seconds) ───────────────
        watchdog_period = max(0.1, self._cmd_timeout / 2.0)
        self._watchdog_timer = self.create_timer(
            watchdog_period, self._watchdog_callback
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
            f"Subscribing to '{self._cmd_topic}'  "
            f"cmd_timeout={self._cmd_timeout:.1f}s"
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
                except serial.SerialException as exc:
                    self.get_logger().error(f"Serial write error: {exc}")
                    self._serial = None

    def _send_drive(self, vx: float, vy: float, omega: float):
        """
        Format and send a DRIVE command to the Arduino.

        DriveMaster.ino mecanumIK signature:  mecanumIK(vy, vx, omega)
          vy    → linear.y  (strafe left positive)
          vx    → linear.x  (forward positive)
          omega → angular.z (CCW positive, rad/s)

        The Arduino clamps to MAX_LINEAR_VEL and MAX_ANGULAR_VEL internally.
        """
        cmd = f"DRIVE,{vy:.4f},{vx:.4f},{omega:.4f}"
        self._write_cmd(cmd)

    def _send_stop(self):
        """Send a zero-velocity DRIVE command (robot stops)."""
        self._write_cmd("DRIVE,0.0000,0.0000,0.0000")

    # ──────────────────────────────────────────────────────────────────────────
    # /cmd_vel callback
    # ──────────────────────────────────────────────────────────────────────────
    def _cmd_vel_callback(self, msg: Twist):
        """Receive a Twist from joystick / Nav2 and forward to the Arduino."""
        self._last_cmd_time = time.monotonic()
        self._stopped = False

        vx    = msg.linear.x
        vy    = msg.linear.y
        omega = msg.angular.z

        self._send_drive(vx, vy, omega)

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
        wheel RPMs and (optionally) IMU data.
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

            # ── Wheel RPMs ────────────────────────────────────────────────────
            try:
                rpm_msg = Float32MultiArray()
                rpm_msg.data = [
                    float(data.get("W1_RPM", 0.0)),
                    float(data.get("W2_RPM", 0.0)),
                    float(data.get("W3_RPM", 0.0)),
                    float(data.get("W4_RPM", 0.0)),
                ]
                self._pub_rpms.publish(rpm_msg)
            except Exception:
                pass

            # ── IMU data (BNO055 fused output) ───────────────────────────────
            if self._pub_imu and "IMU_OK" in data:
                try:
                    if data["IMU_OK"] == "1":
                        self._publish_imu(data)
                except Exception:
                    pass

    # ──────────────────────────────────────────────────────────────────────────
    # IMU publisher
    # ──────────────────────────────────────────────────────────────────────────
    def _publish_imu(self, data: dict):
        """
        Convert BNO055 telemetry fields to sensor_msgs/Imu and publish.

        Fields used from DriveMaster telemetry:
          HDG   (deg, 0..360)         → orientation quaternion (yaw only)
          GX,GY,GZ (deg/s)           → angular_velocity
          AX,AY,AZ (m/s², gravity removed) → linear_acceleration

        Note: The BNO055 provides full 9-DOF fused orientation (Euler angles).
        For a proper quaternion we would need all three Euler angles; here we
        publish a yaw-only quaternion for now.  Connect a dedicated imu_node.py
        for full fusion if needed.
        """
        imu_msg = Imu()
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self._frame_id

        # Heading (yaw) from BNO055 Euler — convert degrees → radians
        hdg_deg = float(data.get("HDG", 0.0))
        yaw = math.radians(hdg_deg)

        # Yaw-only quaternion  (roll=0, pitch=0)
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = math.sin(yaw / 2.0)
        imu_msg.orientation.w = math.cos(yaw / 2.0)
        # Covariance unknown (-1 diagonal signals "unknown")
        imu_msg.orientation_covariance[0] = -1.0

        # Angular velocity (deg/s → rad/s)
        imu_msg.angular_velocity.x = math.radians(float(data.get("GX", 0.0)))
        imu_msg.angular_velocity.y = math.radians(float(data.get("GY", 0.0)))
        imu_msg.angular_velocity.z = math.radians(float(data.get("GZ", 0.0)))

        # Linear acceleration (already m/s², gravity removed by BNO055)
        imu_msg.linear_acceleration.x = float(data.get("AX", 0.0))
        imu_msg.linear_acceleration.y = float(data.get("AY", 0.0))
        imu_msg.linear_acceleration.z = float(data.get("AZ", 0.0))

        self._pub_imu_msg.publish(imu_msg)

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
