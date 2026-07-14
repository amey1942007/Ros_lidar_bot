#!/usr/bin/env python3
"""
imu_node.py — BNO055 IMU publisher via Arduino Mega UART bridge (Jazzy-targeted).

Hardware : Arduino Mega reads BNO055 over I2C, sends JSON lines over UART (USB/serial)
Publishes: sensor_msgs/Imu on /imu
              frame_id = "imu_for_urdf_1"   (matches URDF link name)

Expected JSON line format (one object per line):
{
  "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
  "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
  "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 0.0},
  "orientation_covariance": [0.0, ... 9 values],
  "angular_velocity_covariance": [0.0, ... 9 values],
  "linear_acceleration_covariance": [0.0, ... 9 values]
}

All values in SI units: quaternion normalized, rad/s, m/s².
Covariances are 3×3 row-major (9 floats each).

Unit notes (Rule 9):
  Arduino should send already-converted SI units.
  This node does NOT convert degrees→rad or raw→m/s².

Covariance (Rule 5 — never all-zero, values documented):
  If Arduino omits covariances, sensible defaults are applied.

Jazzy notes:
  • get_logger().warn(..., throttle_duration_sec=N) stable since Humble.
  • rclpy.time.Time.nanoseconds property stable across Humble/Iron/Jazzy.
  • No Jazzy-specific API differences affect this node.

Parameters (override via launch or --ros-args -p):
  serial_port        (string, default '/dev/ttyACM0')  — Arduino Mega USB port
  baud_rate          (int,    default 115200)          — must match Arduino Serial.begin()
  output_topic       (string, default '/imu')
  frame_id           (string, default 'imu_link')
  publish_rate       (float,  default 50.0)           — max Hz (throttles if Arduino faster)
  timeout            (float,  default 0.1)             — serial read timeout (s)
"""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:
    serial = None


# ── Default covariances (used if Arduino omits them) ────────────────────────────
# BNO055 typical noise at 100 Hz fusion mode:
#   Orientation (heading)     σ ≈ 1.0°  = 0.017 rad → σ² ≈ 3e-4  rad²
#   Orientation (roll/pitch)  σ ≈ 2.5°  = 0.044 rad → σ² ≈ 2e-3  rad²
#   Angular velocity          σ ≈ 0.01  rad/s        → σ² ≈ 1e-4  rad²/s²
#   Linear acceleration       σ ≈ 0.12  m/s²         → σ² ≈ 1.4e-2 m²/s⁴
_ORI_COV_ROLL_PITCH = 2e-3
_ORI_COV_YAW        = 3e-4
_GYR_COV            = 1e-4
_ACC_COV            = 1.44e-2

_DEFAULT_ORI_COV = [
    _ORI_COV_ROLL_PITCH, 0.0,                 0.0,
    0.0,                 _ORI_COV_ROLL_PITCH, 0.0,
    0.0,                 0.0,                 _ORI_COV_YAW,
]
_DEFAULT_GYR_COV = [
    _GYR_COV, 0.0,      0.0,
    0.0,      _GYR_COV, 0.0,
    0.0,      0.0,      _GYR_COV,
]
_DEFAULT_ACC_COV = [
    _ACC_COV, 0.0,      0.0,
    0.0,      _ACC_COV, 0.0,
    0.0,      0.0,      _ACC_COV,
]


# ── Helper: validate quaternion is normalized (Rule 6) ──────────────────────────
def _normalize_quat(q: dict) -> tuple[float, float, float, float]:
    """Return (x, y, z, w) normalized to unit length."""
    x, y, z, w = float(q["x"]), float(q["y"]), float(q["z"]), float(q["w"])
    norm = (x * x + y * y + z * z + w * w) ** 0.5
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


# ── Node ────────────────────────────────────────────────────────────────────────
class ImuNode(Node):
    """Reads JSON lines from Arduino Mega over UART and publishes sensor_msgs/Imu."""

    def __init__(self):
        super().__init__("imu_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self._port        = self.declare_parameter("serial_port", "/dev/ttyACM1").value
        self._baud        = self.declare_parameter("baud_rate", 500000).value
        self._topic       = self.declare_parameter("output_topic", "/imu").value
        self._frame_id    = self.declare_parameter("frame_id", "imu_link").value
        self._rate_hz     = self.declare_parameter("publish_rate", 50.0).value
        self._timeout     = self.declare_parameter("timeout", 0.1).value

        # ── Serial connection ────────────────────────────────────────────────
        self._serial = self._connect_serial()

        # ── Latest parsed IMU data (thread-safe) ─────────────────────────────
        self._latest_msg: Imu | None = None
        self._lock = threading.Lock()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        # ── Publisher + rate-limited timer ───────────────────────────────────
        self._pub = self.create_publisher(Imu, self._topic, 10)
        self._timer = self.create_timer(1.0 / self._rate_hz, self._publish_latest)

        self.get_logger().info(
            f"ImuNode ready: {self._port} @ {self._baud} baud → "
            f"{self._topic} @ {self._rate_hz} Hz  frame={self._frame_id}"
        )

    # ── Serial connection ────────────────────────────────────────────────────
    def _connect_serial(self):
        if serial is None:
            self.get_logger().fatal("pyserial not installed. Run: pip3 install pyserial")
            raise SystemExit(1)

        while rclpy.ok():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
                self.get_logger().info(f"Serial connected: {self._port} @ {self._baud}")
                return ser
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f"Failed to open {self._port}: {exc}. Retrying in 2 s...",
                    throttle_duration_sec=5.0,
                )
                time.sleep(2.0)

    # ── Background reader thread ─────────────────────────────────────────────
    def _read_loop(self):
        """Continuously read lines, parse JSON, build Imu message."""
        buffer = ""
        while rclpy.ok():
            try:
                data = self._serial.read(self._serial.in_waiting or 1)
                if not data:
                    continue
                buffer += data.decode("utf-8", errors="ignore")
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    self._parse_line(line)
            except serial.SerialException as exc:
                self.get_logger().error(f"Serial error: {exc}. Reconnecting...")
                time.sleep(1.0)
                self._serial = self._connect_serial()
            except Exception as exc:
                self.get_logger().warn(
                    f"Reader loop error: {exc}",
                    throttle_duration_sec=5.0,
                )

    # ── Parse one JSON line ──────────────────────────────────────────────────
    def _parse_line(self, line: str):
        try:
            obj = json.loads(line)
        except json.JSONDecodeError:
            self.get_logger().warn(
                f"Invalid JSON: {line[:80]}...",
                throttle_duration_sec=5.0,
            )
            return

        # Required fields
        try:
            qx, qy, qz, qw = _normalize_quat(obj["orientation"])
            gx, gy, gz = (
                float(obj["angular_velocity"]["x"]),
                float(obj["angular_velocity"]["y"]),
                float(obj["angular_velocity"]["z"]),
            )
            ax, ay, az = (
                float(obj["linear_acceleration"]["x"]),
                float(obj["linear_acceleration"]["y"]),
                float(obj["linear_acceleration"]["z"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"Missing/invalid field in JSON: {exc}",
                throttle_duration_sec=5.0,
            )
            return

        # Optional covariances — fall back to defaults
        ori_cov = obj.get("orientation_covariance", _DEFAULT_ORI_COV)
        gyr_cov = obj.get("angular_velocity_covariance", _DEFAULT_GYR_COV)
        acc_cov = obj.get("linear_acceleration_covariance", _DEFAULT_ACC_COV)

        # Build message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.orientation_covariance = [float(v) for v in ori_cov]

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz
        msg.angular_velocity_covariance = [float(v) for v in gyr_cov]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.linear_acceleration_covariance = [float(v) for v in acc_cov]

        with self._lock:
            self._latest_msg = msg

    # ── Timer callback: publish latest at configured rate ─────────────────────
    def _publish_latest(self):
        with self._lock:
            msg = self._latest_msg
        if msg is not None:
            # Update timestamp to now (Rule 4: stamp at publish time)
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(msg)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down ImuNode...")
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
        except Exception:
            pass
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
