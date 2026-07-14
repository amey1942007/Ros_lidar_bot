#!/usr/bin/env python3
"""
lidar_node.py – ROS 2 node for the RPLidar A1 laser scanner.

================================================================================
PHYSICAL ARCHITECTURE & UART COMMUNICATION
================================================================================
The RPLidar A1 laser scanner utilizes a serial UART channel to communicate with
the host controller (Raspberry Pi 5 running Ubuntu/ROS 2 Jazzy).
- Physical Hardware Interface: Usually maps to USB serial converter /dev/ttyUSB0
- Default Baud Rate: 115200
- Native Scan rate: ~5.5 Hz rotating sweep
- Motor control: Driven by setting motor state (on/off).

================================================================================
DESIGN PATHWAY & THREADING MODEL
================================================================================
Modern ROS 2 executors spin nodes using single- or multi-threaded executors.
Because serial read calls to the LiDAR are blocking and depend heavily on
microsecond-level timing of incoming bytes, running serial iteration in the
main thread causes executor starvation.
To prevent this, this node spins a background thread (`_scan_loop` via Python's
`threading.Thread`) dedicated entirely to reading scans. Scans are formatted
and published to ROS 2 thread-safely.

================================================================================
DEPENDENCY DETAIL (JAZZY TARGETED)
================================================================================
Under ROS 2 Jazzy, system-wide pip libraries are strongly preferred over
virtual environments (venv) to prevent setup failures. Since standard 'pyrplidar'
requires compiler flags and virtual environments, this node uses the pure-Python
pyserial-based `rplidar-roboticia` library.
To install system-wide:
    pip3 install rplidar-roboticia

================================================================================
SCAN MODES & ROS parameters
================================================================================
- Standard (normal) mode (`sensitivity_mode = False`): Uses classic single-point
  ranging protocols. Good for lower power.
- Sensitivity (express) mode (`sensitivity_mode = True`): Uses packet-compressed
  Express Scan protocols to yield higher point density, mapping to sensitivity scan ID=1.

ROS parameters:
    serial_port      (string,  default '/dev/ttyUSB0')  – serial device port
    serial_baud      (int,     default 115200)           – baud rate
    scan_topic       (string,  default '/scan')           – topic to publish LaserScan
    frame_id         (string,  default 'laser_frame')    – LaserScan header frame_id
    min_range        (float,   default 0.15)             – range filter threshold (meters)
    max_range        (float,   default 12.0)             – maximum valid range (meters)
    publish_rate     (float,   default 10.0)             – max throttled publish frequency
    motor_pwm        (int,     default 660)               – motor PWM (unused by rplidar-roboticia)
    sensitivity_mode (bool,    default True)              – True = Express Scan mode, False = Standard Scan.
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from rplidar import RPLidar, RPLidarException
    RPLIDAR_AVAILABLE = True
except ImportError:
    RPLIDAR_AVAILABLE = False
    RPLidar = None
    RPLidarException = Exception


class LidarNode(Node):
    """Publishes RPLidar A1 scans as sensor_msgs/LaserScan on /scan.

    Supports both Standard ('normal') and Sensitivity ('express') scan modes via the
    `sensitivity_mode` ROS parameter.
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

        # ── Connection + Scan Thread State ────────────────────────────────
        self.lidar: Optional[RPLidar] = None
        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)

        if not RPLIDAR_AVAILABLE:
            self.get_logger().fatal(
                'rplidar-roboticia Python library not found. '
                'Install it system-wide with:  pip3 install rplidar-roboticia'
            )
            raise SystemExit(1)

        self._thread.start()

        mode_name = 'Sensitivity (Express)' if self.sensitivity_mode else 'Standard'
        self.get_logger().info(
            f'LidarNode started: port={self.serial_port} baud={self.serial_baud} '
            f'topic={self.scan_topic} frame={self.frame_id} '
            f'scan_mode={mode_name}'
        )

    def _warn_throttled(self, message: str, interval: float = 5.0):
        """Log a warning at most once every *interval* seconds."""
        now = time.monotonic()
        if now - self._last_warn_time >= interval:
            self.get_logger().warn(message)
            self._last_warn_time = now

    def _connect_lidar(self) -> bool:
        """Attempts to connect to the RPLidar device and start the motor."""
        self.get_logger().info(
            f'Connecting to RPLidar on {self.serial_port} at {self.serial_baud} baud …'
        )
        try:
            # RPLidar constructor opens the serial port connection
            self.lidar = RPLidar(self.serial_port, baudrate=self.serial_baud, timeout=3)
            
            info   = self.lidar.get_info()
            health = self.lidar.get_health()
            self.get_logger().info(f'RPLidar info:   {info}')
            self.get_logger().info(f'RPLidar health: {health}')
            
            self.get_logger().info(
                'Starting lidar motor (note: motor_pwm customization is not supported by rplidar-roboticia).'
            )
            self.lidar.start_motor()
            time.sleep(1.0)  # let motor spin up
            return True
        except Exception as exc:
            self.get_logger().error(f'Failed to connect to RPLidar: {exc}')
            self._disconnect_lidar()
            return False

    def _disconnect_lidar(self):
        """Stops scanning, stops motor, and closes serial port safely."""
        if self.lidar:
            try:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
            except Exception:
                pass
            self.lidar = None

    def _scan_loop(self):
        """Background thread that block-reads scans from RPLidar and publishes them."""
        while rclpy.ok() and self._running:
            if self.lidar is None:
                if not self._connect_lidar():
                    time.sleep(2.0)
                    continue

            try:
                # Express scan mode maps to rplidar's 'express', Standard maps to 'normal'
                scan_type = 'express' if self.sensitivity_mode else 'normal'
                self.get_logger().info(f'Starting scan stream (mode={scan_type}) …')
                
                # Retrieve individual sweeps. max_buf_meas=False prevents dropping data.
                for scan in self.lidar.iter_scans(scan_type=scan_type, max_buf_meas=False):
                    if not self._running or not rclpy.ok():
                        break

                    if not scan:
                        continue

                    now = time.monotonic()
                    if self._min_publish_interval == 0.0 or (
                        now - self._last_publish_time >= self._min_publish_interval
                    ):
                        msg = self._build_laserscan(scan)
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher_.publish(msg)
                        self._last_publish_time = now

            except RPLidarException as exc:
                self._warn_throttled(f'RPLidarException in scan loop: {exc}')
                self._disconnect_lidar()
                time.sleep(2.0)
            except Exception as exc:
                self._warn_throttled(f'Unexpected error in scan loop: {exc}')
                self._disconnect_lidar()
                time.sleep(2.0)

        self._disconnect_lidar()

    def _build_laserscan(self, scan):
        """Convert a list of (quality, angle, distance) tuples to LaserScan.

        Args:
            scan: list of (quality, angle, distance) tuples
                  quality: int, angle: float (degrees), distance: float (mm)

        Returns:
            sensor_msgs.msg.LaserScan populated and ready to publish.
        """
        num_readings    = 360
        angle_increment = (self.ANGLE_MAX - self.ANGLE_MIN) / num_readings

        ranges      = [float('inf')] * num_readings
        intensities = [0.0]          * num_readings

        for quality, angle, distance in scan:
            if quality is None or angle is None or distance is None:
                continue
            angle_deg  = angle % 360.0
            idx        = int(angle_deg / 360.0 * num_readings) % num_readings
            distance_m = distance / 1000.0       # mm → m

            if distance_m < self.min_range or distance_m > self.max_range:
                continue

            # Keep closest measurement if multiple fall into the same bin
            if distance_m < ranges[idx]:
                ranges[idx]      = distance_m
                intensities[idx] = float(quality)

        msg = LaserScan()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = self.ANGLE_MIN
        msg.angle_max       = self.ANGLE_MAX - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / 5.5  # ~5.5 Hz native scan rate of A1

        msg.range_min   = self.min_range
        msg.range_max   = self.max_range
        msg.ranges      = ranges
        msg.intensities = intensities

        return msg

    def destroy_node(self):
        """Clean shutdown."""
        self.get_logger().info('Shutting down LidarNode …')
        self._running = False
        self._disconnect_lidar()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
