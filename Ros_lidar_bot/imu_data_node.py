#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

try:
    import adafruit_bno055
    import board
except ImportError:
    adafruit_bno055 = None
    board = None


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class ImuDataNode(Node):
    def __init__(self):
        super().__init__('imu_data_node')

        self.output_topic = self.declare_parameter('output_topic', '/imu').value
        self.frame_id = self.declare_parameter(
            'frame_id',
            'my_bot/base_footprint/imu_sensor',
        ).value
        self.publish_rate = self.declare_parameter('publish_rate', 30.0).value
        self.i2c_address = self.declare_parameter('i2c_address', 0x28).value

        self.last_warning_time = 0.0
        self.sensor = self._connect_bno055()

        self.publisher = self.create_publisher(Imu, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_imu)

        self.get_logger().info(
            f'Reading BNO055 on I2C address 0x{self.i2c_address:02x} '
            f'and publishing IMU data to {self.output_topic}'
        )

    def _connect_bno055(self):
        if adafruit_bno055 is None or board is None:
            raise RuntimeError(
                'Missing BNO055 Python libraries. Install with: '
                'pip3 install adafruit-circuitpython-bno055'
            )

        i2c = board.I2C()
        return adafruit_bno055.BNO055_I2C(i2c, address=self.i2c_address)

    def publish_imu(self):
        try:
            euler = self.sensor.euler
            gyro = self.sensor.gyro
            linear_acceleration = self.sensor.linear_acceleration
        except OSError as exc:
            self._warn_throttled(f'Could not read BNO055 over I2C: {exc}')
            return

        if euler is None or euler[0] is None:
            self._warn_throttled('BNO055 yaw is not ready yet. Check sensor calibration.')
            return
        if gyro is None or gyro[2] is None:
            self._warn_throttled('BNO055 gyro data is not ready yet.')
            return
        if linear_acceleration is None:
            self._warn_throttled('BNO055 linear acceleration is not ready yet.')
            return

        yaw = math.radians(euler[0])
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # EKF is configured for 2D mode, so publish yaw only.
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = self._value_or_zero(linear_acceleration[0])
        msg.linear_acceleration.y = self._value_or_zero(linear_acceleration[1])
        msg.linear_acceleration.z = self._value_or_zero(linear_acceleration[2])

        msg.orientation_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 99999.0, 0.0,
            0.0, 0.0, 0.05,
        ]
        msg.angular_velocity_covariance = [
            99999.0, 0.0, 0.0,
            0.0, 99999.0, 0.0,
            0.0, 0.0, 0.05,
        ]
        msg.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.1,
        ]

        self.publisher.publish(msg)

    def _warn_throttled(self, message):
        now = time.monotonic()
        if now - self.last_warning_time >= 5.0:
            self.get_logger().warn(message)
            self.last_warning_time = now

    @staticmethod
    def _value_or_zero(value):
        if value is None:
            return 0.0
        return float(value)


def main(args=None):
    rclpy.init(args=args)
    node = ImuDataNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
