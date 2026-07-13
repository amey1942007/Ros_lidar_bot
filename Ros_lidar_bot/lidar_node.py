#!/usr/bin/env python3
"""
lidar_node.py – ROS 2 node for the RPLidar A1 laser scanner.

Reads raw 360° scan data from the RPLidar A1 over a serial port and
publishes a sensor_msgs/LaserScan message on the /scan topic, which is
the standard topic consumed by Nav2, slam_toolbox, etc.

Dependencies (install once on the robot):
    pip3 install rplidar-roboticia
    # or the community fork:  pip3 install rplidar

Parameters (all overridable from a launch file or CLI):
    serial_port   (string,  default '/dev/ttyUSB0')  – serial device
    serial_baud   (int,     default 115200)          – baud rate
    scan_topic    (string,  default '/scan')          – publish topic
    frame_id      (string,  default 'laser')          – LaserScan frame
    min_range     (float,   default 0.15  m)          – discard closer hits
    max_range     (float,   default 12.0  m)          – discard farther hits
    publish_rate  (float,   default 10.0  Hz)         – throttle (0 = free-running)
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from rplidar import RPLidar, RPLidarException
except ImportError:
    RPLidar = None
    RPLidarException = Exception


class LidarNode(Node):
    """Publishes RPLidar A1 scans as sensor_msgs/LaserScan on /scan."""

    # The A1 covers a full 360° sweep.
    ANGLE_MIN = 0.0
    ANGLE_MAX = 2.0 * math.pi

    def __init__(self):
        super().__init__('lidar_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.serial_port  = self.declare_parameter('serial_port',  '/dev/ttyUSB0').value
        self.serial_baud  = self.declare_parameter('serial_baud',  115200).value
        self.scan_topic   = self.declare_parameter('scan_topic',   '/scan').value
        self.frame_id     = self.declare_parameter('frame_id',     'laser_frame').value
        self.min_range    = self.declare_parameter('min_range',    0.15).value
        self.max_range    = self.declare_parameter('max_range',    12.0).value
        self.publish_rate = self.declare_parameter('publish_rate', 10.0).value
        self.motor_pwm    = self.declare_parameter('motor_pwm',    660).value

        # ── Publisher ──────────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(LaserScan, self.scan_topic, 10)

        # ── Throttle helpers ───────────────────────────────────────────────
        self._last_warn_time    = 0.0
        self._last_publish_time = 0.0
        self._min_publish_interval = (
            1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.0
        )

        # ── Connect to the lidar ───────────────────────────────────────────
        self.lidar = self._connect_lidar()

        # ── Timer drives the scan loop (fast poll, iterator blocks inside) ─
        self.timer = self.create_timer(0.01, self._scan_once)

        self.get_logger().info(
            f'LidarNode started: port={self.serial_port} baud={self.serial_baud} '
            f'topic={self.scan_topic} frame={self.frame_id}'
        )

    # ── Private helpers ────────────────────────────────────────────────────

    def _connect_lidar(self):
        """Instantiate the RPLidar driver and start motor + scan iterator."""
        if RPLidar is None:
            self.get_logger().fatal(
                'rplidar Python library not found. '
                'Install it with:  pip3 install rplidar-roboticia'
            )
            raise SystemExit(1)

        self.get_logger().info(
            f'Connecting to RPLidar A1 on {self.serial_port} '
            f'at {self.serial_baud} baud …'
        )
        lidar = RPLidar(self.serial_port, baudrate=self.serial_baud, timeout=3)

        info   = lidar.get_info()
        health = lidar.get_health()
        self.get_logger().info(f'RPLidar info:   {info}')
        self.get_logger().info(f'RPLidar health: {health}')

        # Set motor speed/PWM if specified (valid range: 0 to 1023)
        if 0 <= self.motor_pwm <= 1023:
            self.get_logger().info(f'Setting RPLidar motor PWM to: {self.motor_pwm}')
            try:
                lidar.set_pwm(self.motor_pwm)
            except Exception as e:
                self.get_logger().error(f'Failed to set motor PWM: {e}')

        # Persistent iterator – yields complete 360° scans as lists of
        # (quality, angle_deg, distance_mm) tuples.
        self._scan_iterator = lidar.iter_scans(max_buf_meas=500)

        return lidar

    def _scan_once(self):
        """Fetch the next complete scan from the iterator and publish it."""
        now = time.monotonic()

        # Optional rate-throttle
        if self._min_publish_interval > 0.0:
            if now - self._last_publish_time < self._min_publish_interval:
                return

        try:
            raw_scan = next(self._scan_iterator)
        except RPLidarException as exc:
            self._warn_throttled(f'RPLidar scan error: {exc}')
            return
        except StopIteration:
            self._warn_throttled('RPLidar iterator exhausted – reconnecting …')
            self._reconnect()
            return
        except Exception as exc:
            self._warn_throttled(f'Unexpected lidar error: {exc}')
            return

        if not raw_scan:
            return

        msg = self._build_laserscan(raw_scan)
        self.publisher_.publish(msg)
        self._last_publish_time = now

    def _build_laserscan(self, raw_scan):
        """Convert an RPLidar raw scan to a sensor_msgs/LaserScan message.

        Args:
            raw_scan: list of (quality, angle_deg, distance_mm) tuples
                      returned by rplidar.iter_scans().

        Returns:
            sensor_msgs.msg.LaserScan populated and ready to publish.
        """
        num_readings    = 360
        angle_increment = (self.ANGLE_MAX - self.ANGLE_MIN) / num_readings

        ranges      = [float('inf')] * num_readings
        intensities = [0.0]          * num_readings

        for quality, angle_deg, distance_mm in raw_scan:
            angle_deg  = angle_deg % 360.0
            idx        = int(angle_deg / 360.0 * num_readings) % num_readings
            distance_m = distance_mm / 1000.0

            if distance_m < self.min_range or distance_m > self.max_range:
                # Leave as inf – out of valid range
                continue

            # Keep the closest measurement when multiple points share a bucket
            if distance_m < ranges[idx]:
                ranges[idx]      = distance_m
                intensities[idx] = float(quality)

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = self.ANGLE_MIN
        msg.angle_max       = self.ANGLE_MAX - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment  = 0.0        # unknown per-point time
        msg.scan_time       = 1.0 / 5.5  # ~5.5 Hz native scan rate of A1

        msg.range_min  = self.min_range
        msg.range_max  = self.max_range
        msg.ranges     = ranges
        msg.intensities = intensities

        return msg

    def _warn_throttled(self, message: str, interval: float = 5.0):
        """Log a warning at most once every *interval* seconds."""
        now = time.monotonic()
        if now - self._last_warn_time >= interval:
            self.get_logger().warn(message)
            self._last_warn_time = now

    def _reconnect(self):
        """Stop the current lidar session and restart it."""
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception:
            pass
        time.sleep(2.0)
        self.lidar = self._connect_lidar()

    def destroy_node(self):
        """Clean shutdown: stop motor and close serial port."""
        self.get_logger().info('Shutting down LidarNode …')
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            self.lidar.disconnect()
        except Exception:
            pass
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
