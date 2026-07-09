#!/usr/bin/env python3
"""
imu_node.py – BNO055 IMU publisher node (Jazzy-targeted).

Hardware : Adafruit BNO055 over I2C (Raspberry Pi 5).
Publishes: sensor_msgs/Imu on /imu
             frame_id = "imu_link"   (matches TF tree: base_footprint → imu_link)

Unit notes (Rule 9):
  BNO055 euler[]            → degrees        (converted → radians here)
  BNO055 gyro[]             → rad/s          ✓ no conversion needed
  BNO055 linear_acceleration → m/s²          ✓ no conversion needed

Quaternion (Rule 6):
  Full 3-axis Euler (heading, roll, pitch) → quaternion via standard ZYX formula.
  Normalisation guaranteed by construction (sin²+cos²=1 per axis).
  ⚠ BNO055 Euler convention: euler[0]=heading(yaw), euler[1]=roll, euler[2]=pitch
    This differs from aerospace (roll,pitch,yaw) — order handled explicitly below.

Covariance (Rule 5 — never all-zero, values explained):
  BNO055 datasheet + typical measured noise at 100 Hz:
    Orientation (yaw)     σ ≈ 1.0°  = 0.017 rad → σ² ≈ 3e-4  rad²
    Orientation (r/p)     σ ≈ 2.5°  = 0.044 rad → σ² ≈ 2e-3  rad²
    Angular velocity      σ ≈ 0.01  rad/s        → σ² ≈ 1e-4  rad²/s²
    Linear acceleration   σ ≈ 0.12  m/s²         → σ² ≈ 0.014 m²/s⁴
  Roll/pitch orientation not reliable without full calibration on a 2-D robot →
  large but finite value (not -1, not 99999).  EKF will down-weight them.

Jazzy notes:
  • get_logger().warn(..., throttle_duration_sec=N) is available since Humble,
    stable on Jazzy — used here instead of manual time.monotonic() tracking.
  • No other Jazzy-specific differences affect this node.

Parameters:
  output_topic  (string, default '/imu')
  frame_id      (string, default 'imu_link')
  publish_rate  (float,  default 50.0 Hz)   BNO055 fusion output ≤ 100 Hz
  i2c_address   (int,    default 0x28)      alt address: 0x29
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import adafruit_bno055
    import board
except ImportError:
    adafruit_bno055 = None
    board = None


# ── Covariance constants (see module docstring for derivation) ────────────────

# Orientation [roll, pitch, yaw] diagonal (rad²)
_ORI_COV_ROLL_PITCH = 2e-3   # larger — less reliable on a flat robot
_ORI_COV_YAW        = 3e-4   # BNO055 heading noise at full calibration

# Angular velocity [x, y, z] diagonal (rad²/s²)
_GYR_COV = 1e-4              # BNO055 gyro noise floor

# Linear acceleration [x, y, z] diagonal (m²/s⁴)
_ACC_COV = 1.44e-2           # ≈ 0.12² m/s²


# ── Helper ────────────────────────────────────────────────────────────────────

def euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """
    Convert ZYX Euler angles (rad) to a normalised quaternion (x, y, z, w).

    Rotation order: first yaw (Z), then pitch (Y), then roll (X).
    This is the standard ROS / aerospace convention.

    BNO055 provides: euler[0]=heading(yaw), euler[1]=roll, euler[2]=pitch
    Caller is responsible for mapping sensor fields to this function's arguments.

    Math (standard ZYX):
      cy = cos(yaw/2),  sy = sin(yaw/2)
      cp = cos(pitch/2), sp = sin(pitch/2)
      cr = cos(roll/2),  sr = sin(roll/2)

      w =  cr·cp·cy + sr·sp·sy
      x =  sr·cp·cy - cr·sp·sy
      y =  cr·sp·cy + sr·cp·sy
      z =  cr·cp·sy - sr·sp·cy

    ||q||² = w²+x²+y²+z² = 1  (guaranteed by trig identity, no explicit
    normalization step needed).
    """
    cy = math.cos(yaw   * 0.5);  sy = math.sin(yaw   * 0.5)
    cp = math.cos(pitch * 0.5);  sp = math.sin(pitch * 0.5)
    cr = math.cos(roll  * 0.5);  sr = math.sin(roll  * 0.5)

    w =  cr * cp * cy + sr * sp * sy
    x =  sr * cp * cy - cr * sp * sy
    y =  cr * sp * cy + sr * cp * sy
    z =  cr * cp * sy - sr * sp * cy
    return x, y, z, w


# ── Node ──────────────────────────────────────────────────────────────────────

class ImuNode(Node):
    """Reads BNO055 via I2C and publishes sensor_msgs/Imu on /imu."""

    def __init__(self):
        super().__init__('imu_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self._topic       = self.declare_parameter('output_topic', '/imu').value
        self._frame_id    = self.declare_parameter('frame_id',     'imu_link').value
        self._rate        = self.declare_parameter('publish_rate', 50.0).value
        self._i2c_address = self.declare_parameter('i2c_address',  0x28).value

        # ── Hardware connection ────────────────────────────────────────────────
        self._sensor = self._connect_bno055()

        # ── Publisher + timer (Rule 7: timer-driven, no sleep loops) ──────────
        self._pub   = self.create_publisher(Imu, self._topic, 10)
        self._timer = self.create_timer(1.0 / self._rate, self._publish_imu)

        self.get_logger().info(
            f'ImuNode ready: I2C 0x{self._i2c_address:02x} → '
            f'{self._topic} @ {self._rate} Hz  frame={self._frame_id}'
        )

    # ── Hardware ──────────────────────────────────────────────────────────────

    def _connect_bno055(self):
        if adafruit_bno055 is None or board is None:
            self.get_logger().fatal(
                'adafruit-circuitpython-bno055 not installed.\n'
                'Run: pip3 install adafruit-circuitpython-bno055'
            )
            raise SystemExit(1)

        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c, address=self._i2c_address)
        self.get_logger().info(
            f'BNO055 connected at I2C address 0x{self._i2c_address:02x}'
        )
        return sensor

    # ── Timer callback ────────────────────────────────────────────────────────

    def _publish_imu(self) -> None:
        """Read BNO055 and publish sensor_msgs/Imu. Called by rclpy timer."""

        # ── Read sensor (all three fields in one go) ───────────────────────────
        try:
            euler               = self._sensor.euler
            gyro                = self._sensor.gyro
            linear_acceleration = self._sensor.linear_acceleration
        except OSError as exc:
            self.get_logger().warn(
                f'BNO055 I2C read error: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        # ── Validate — BNO055 returns None until calibrated ───────────────────
        #   euler[0]=heading(yaw°), euler[1]=roll°, euler[2]=pitch°
        if euler is None or any(v is None for v in euler):
            self.get_logger().warn(
                'BNO055 Euler not ready — check calibration status.',
                throttle_duration_sec=5.0,
            )
            return

        if gyro is None or any(v is None for v in gyro):
            self.get_logger().warn(
                'BNO055 gyro not ready.',
                throttle_duration_sec=5.0,
            )
            return

        if linear_acceleration is None or any(v is None for v in linear_acceleration):
            self.get_logger().warn(
                'BNO055 linear acceleration not ready.',
                throttle_duration_sec=5.0,
            )
            return

        # ── Unit conversion: BNO055 Euler is in degrees → radians (Rule 9) ────
        #   BNO055 field order: [heading/yaw, roll, pitch]  (non-standard!)
        #   ROS EKF expects ZYX convention: yaw applied first.
        yaw_rad   = math.radians(euler[0])   # heading: 0=North, CW positive
        roll_rad  = math.radians(euler[1])
        pitch_rad = math.radians(euler[2])

        # ── Quaternion from full 3-axis Euler (Rule 6: normalised) ────────────
        qx, qy, qz, qw = euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)

        # ── Build message ──────────────────────────────────────────────────────
        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()   # Rule 4
        msg.header.frame_id = self._frame_id                    # "imu_link"

        # Orientation — full 3-axis, normalised quaternion
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Orientation covariance 3×3 row-major [roll, pitch, yaw] (Rule 5)
        # BNO055 datasheet heading accuracy: ±1° when fully calibrated (σ²≈3e-4 rad²)
        # Roll/pitch less reliable on a flat robot: larger variance used.
        msg.orientation_covariance = [
            _ORI_COV_ROLL_PITCH, 0.0,                 0.0,
            0.0,                 _ORI_COV_ROLL_PITCH,  0.0,
            0.0,                 0.0,                  _ORI_COV_YAW,
        ]

        # Angular velocity — all 3 axes from BNO055 gyro (rad/s, no conversion)
        # ⚠ Previous version hardcoded x=y=0 — now uses real sensor values.
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])

        # Angular velocity covariance 3×3 row-major [x, y, z]
        # BNO055 gyro noise: ~0.01 rad/s → σ² ≈ 1e-4 rad²/s²
        msg.angular_velocity_covariance = [
            _GYR_COV, 0.0,      0.0,
            0.0,      _GYR_COV, 0.0,
            0.0,      0.0,      _GYR_COV,
        ]

        # Linear acceleration — all 3 axes (m/s², no conversion needed)
        msg.linear_acceleration.x = float(linear_acceleration[0])
        msg.linear_acceleration.y = float(linear_acceleration[1])
        msg.linear_acceleration.z = float(linear_acceleration[2])

        # Linear acceleration covariance 3×3 row-major [x, y, z]
        # BNO055 accel noise: ~0.12 m/s² → σ² ≈ 0.014 m²/s⁴
        msg.linear_acceleration_covariance = [
            _ACC_COV, 0.0,      0.0,
            0.0,      _ACC_COV, 0.0,
            0.0,      0.0,      _ACC_COV,
        ]

        self._pub.publish(msg)


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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
