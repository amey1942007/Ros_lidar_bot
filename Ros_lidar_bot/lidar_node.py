#!/usr/bin/env python3
"""
lidar_node.py – ROS 2 node for the RPLidar A1 laser scanner.

Reads raw 360° scan data from the RPLidar A1 over a serial port and
publishes a sensor_msgs/LaserScan message on the /scan topic, which is
the standard topic consumed by Nav2, slam_toolbox, etc.

Dependencies (install once on the robot):
    pip3 install pyrplidar

Parameters (all overridable from a launch file or CLI):
    serial_port      (string,  default '/dev/ttyUSB0')  – serial device
    serial_baud      (int,     default 115200)           – baud rate
    scan_topic       (string,  default '/scan')           – publish topic
    frame_id         (string,  default 'laser_frame')    – LaserScan frame
    min_range        (float,   default 0.15  m)           – discard closer hits
    max_range        (float,   default 12.0  m)           – discard farther hits
    publish_rate     (float,   default 10.0  Hz)          – throttle (0 = free-running)
    motor_pwm        (int,     default 660)               – motor PWM (0–1023)
    sensitivity_mode (bool,    default True)              – use Express/Sensitivity
                                                            scan mode (mode id=1).
                                                            Set False for standard scan.

Scan modes (RPLidar A1):
    Mode 0 – Standard    : lower sample rate, lower power
    Mode 1 – Sensitivity : Express scan, higher angular density & sensitivity
"""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from pyrplidar import PyRPlidar, PyRPlidarConnectionError, PyRPlidarProtocolError
    PYRPLIDAR_AVAILABLE = True
except ImportError:
    PYRPLIDAR_AVAILABLE = False
    PyRPlidar = None
    PyRPlidarConnectionError = Exception
    PyRPlidarProtocolError = Exception

# Sensitivity mode ID for RPLidar A1/A2 (Express scan)
SENSITIVITY_SCAN_MODE_ID = 1
STANDARD_SCAN_MODE_ID    = 0


class LidarNode(Node):
    """Publishes RPLidar A1 scans as sensor_msgs/LaserScan on /scan.

    Supports both Standard and Sensitivity (Express) scan modes via the
    `sensitivity_mode` ROS parameter. Sensitivity mode uses the express
    scan protocol which provides higher angular resolution and detection
    sensitivity compared to the standard scan mode.
    """

    # The A1 covers a full 360° sweep.
    ANGLE_MIN = 0.0
    ANGLE_MAX = 2.0 * math.pi

    def __init__(self):
        super().__init__('lidar_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.serial_port      = self.declare_parameter('serial_port',      '/dev/ttyUSB0').value
        self.serial_baud      = self.declare_parameter('serial_baud',      115200).value
        self.scan_topic       = self.declare_parameter('scan_topic',        '/scan').value
        self.frame_id         = self.declare_parameter('frame_id',          'laser_frame').value
        self.min_range        = self.declare_parameter('min_range',         0.15).value
        self.max_range        = self.declare_parameter('max_range',         12.0).value
        self.publish_rate     = self.declare_parameter('publish_rate',      10.0).value
        self.motor_pwm        = self.declare_parameter('motor_pwm',         660).value
        self.sensitivity_mode = self.declare_parameter('sensitivity_mode',  True).value

        # ── Publisher ──────────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(LaserScan, self.scan_topic, 10)

        # ── Throttle helpers ───────────────────────────────────────────────
        self._last_warn_time    = 0.0
        self._last_publish_time = 0.0
        self._min_publish_interval = (
            1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.0
        )

        # ── Scan accumulation state ────────────────────────────────────────
        # Individual measurements are collected until a new-scan start flag
        # is seen, then the completed scan is published.
        self._pending_scan: list = []

        # ── Connect to the lidar ───────────────────────────────────────────
        self.lidar: Optional[PyRPlidar] = None
        self._scan_generator = None
        self._connect_lidar()

        # ── Timer drives the scan loop ─────────────────────────────────────
        self.timer = self.create_timer(0.001, self._scan_once)

        mode_name = 'Sensitivity (Express)' if self.sensitivity_mode else 'Standard'
        self.get_logger().info(
            f'LidarNode started: port={self.serial_port} baud={self.serial_baud} '
            f'topic={self.scan_topic} frame={self.frame_id} '
            f'scan_mode={mode_name}'
        )

    # ── Private helpers ────────────────────────────────────────────────────

    def _connect_lidar(self):
        """Instantiate the PyRPlidar driver and start the appropriate scan mode."""
        if not PYRPLIDAR_AVAILABLE:
            self.get_logger().fatal(
                'pyrplidar Python library not found. '
                'Install it with:  pip3 install pyrplidar'
            )
            raise SystemExit(1)

        self.get_logger().info(
            f'Connecting to RPLidar on {self.serial_port} '
            f'at {self.serial_baud} baud …'
        )

        lidar = PyRPlidar()
        lidar.connect(port=self.serial_port, baudrate=self.serial_baud, timeout=3)

        info   = lidar.get_info()
        health = lidar.get_health()
        self.get_logger().info(f'RPLidar info:   {info}')
        self.get_logger().info(f'RPLidar health: {health}')

        # ── Set motor PWM ──────────────────────────────────────────────────
        if 0 <= self.motor_pwm <= 1023:
            self.get_logger().info(f'Setting RPLidar motor PWM to: {self.motor_pwm}')
            try:
                lidar.set_motor_pwm(self.motor_pwm)
                time.sleep(0.5)          # let motor spin up
            except Exception as exc:
                self.get_logger().error(f'Failed to set motor PWM: {exc}')

        # ── Select scan mode ───────────────────────────────────────────────
        if self.sensitivity_mode:
            self.get_logger().info(
                f'Starting Sensitivity (Express) scan mode '
                f'[mode id={SENSITIVITY_SCAN_MODE_ID}] …'
            )
            try:
                scan_generator_factory = lidar.start_scan_express(
                    mode=SENSITIVITY_SCAN_MODE_ID
                )
                self._scan_generator = scan_generator_factory()
            except Exception as exc:
                self.get_logger().warn(
                    f'Sensitivity mode failed ({exc}); '
                    'falling back to Standard scan mode.'
                )
                scan_generator_factory = lidar.start_scan()
                self._scan_generator = scan_generator_factory()
        else:
            self.get_logger().info('Starting Standard scan mode …')
            scan_generator_factory = lidar.start_scan()
            self._scan_generator = scan_generator_factory()

        self.lidar = lidar
        self._pending_scan = []

    def _scan_once(self):
        """Fetch the next measurement from the generator; publish on complete scan."""
        try:
            measurement = next(self._scan_generator)
        except (PyRPlidarConnectionError, PyRPlidarProtocolError) as exc:
            self._warn_throttled(f'RPLidar scan error: {exc}')
            return
        except StopIteration:
            self._warn_throttled('RPLidar generator exhausted – reconnecting …')
            self._reconnect()
            return
        except Exception as exc:
            self._warn_throttled(f'Unexpected lidar error: {exc}')
            return

        # start_flag marks the beginning of a new 360° sweep.
        # When we see it, publish the *previous* accumulated sweep first.
        if measurement.start_flag and self._pending_scan:
            now = time.monotonic()
            if self._min_publish_interval == 0.0 or (
                now - self._last_publish_time >= self._min_publish_interval
            ):
                msg = self._build_laserscan(self._pending_scan)
                self.publisher_.publish(msg)
                self._last_publish_time = now
            self._pending_scan = []

        # Quality == 0 → invalid measurement (e.g. no reflection); skip it.
        if measurement.distance > 0:
            self._pending_scan.append(measurement)

    def _build_laserscan(self, raw_scan):
        """Convert a list of PyRPlidarMeasurement objects to LaserScan.

        Args:
            raw_scan: list of PyRPlidarMeasurement with attributes
                      .quality (int), .angle (float, degrees), .distance (float, mm).

        Returns:
            sensor_msgs.msg.LaserScan populated and ready to publish.
        """
        num_readings    = 360
        angle_increment = (self.ANGLE_MAX - self.ANGLE_MIN) / num_readings

        ranges      = [float('inf')] * num_readings
        intensities = [0.0]          * num_readings

        for m in raw_scan:
            angle_deg  = m.angle % 360.0
            idx        = int(angle_deg / 360.0 * num_readings) % num_readings
            distance_m = m.distance / 1000.0       # mm → m

            if distance_m < self.min_range or distance_m > self.max_range:
                continue

            # Keep the closest measurement when multiple points share a bucket
            if distance_m < ranges[idx]:
                ranges[idx]      = distance_m
                intensities[idx] = float(m.quality)

        msg = LaserScan()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = self.ANGLE_MIN
        msg.angle_max       = self.ANGLE_MAX - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment  = 0.0        # unknown per-point time
        msg.scan_time       = 1.0 / 5.5  # ~5.5 Hz native scan rate of A1

        msg.range_min   = self.min_range
        msg.range_max   = self.max_range
        msg.ranges      = ranges
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
            if self.lidar is not None:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
                self.lidar.disconnect()
        except Exception:
            pass
        time.sleep(2.0)
        self._connect_lidar()

    def destroy_node(self):
        """Clean shutdown: stop motor and close serial port."""
        self.get_logger().info('Shutting down LidarNode …')
        try:
            if self.lidar is not None:
                self.lidar.stop()
                self.lidar.set_motor_pwm(0)
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
