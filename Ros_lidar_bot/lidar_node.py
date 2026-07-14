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
- Motor control: Driven via set_motor_pwm(pwm_value).

================================================================================
ANGLE CONVENTION — CRITICAL
================================================================================
The RPLidar A1 reports angles **clockwise** (0° = front, 90° = right).
ROS 2 LaserScan uses **counter-clockwise** (CCW) convention (0° = front,
positive angles increase CCW, i.e. to the left).

This node converts CW → CCW by mirroring:
    ros_angle_deg = (360.0 - lidar_angle_deg) % 360.0

Failure to do this results in scan points appearing mirrored in RViz2.

================================================================================
DESIGN PATHWAY & THREADING MODEL
================================================================================
To prevent ROS executor starvation and ensure real-time responsiveness, all
blocking serial operations and connection retries are run in a dedicated
background thread. This background thread reads measurements from the generator
as fast as they arrive, preventing any serial buffer backlog (which causes
the "spiral/lag" effect in RViz) while leaving the ROS executor free to handle
discovery and publishing.

================================================================================
DEPENDENCY DETAIL (JAZZY TARGETED)
================================================================================
Uses pyrplidar (pip3 install pyrplidar --break-system-packages).
pyrplidar supports the full RPLidar SDK including Express/Sensitivity scan mode.

================================================================================
SCAN MODES & ROS parameters
================================================================================
- Standard (normal) mode (`sensitivity_mode = False`): Mode 0.
- Sensitivity (express) mode (`sensitivity_mode = True`): Mode 1.
  Higher point density. Falls back to Standard if device does not support it.

ROS parameters:
    serial_port      (string,  default '/dev/ttyUSB0')  – serial device port
    serial_baud      (int,     default 115200)           – baud rate
    scan_topic       (string,  default '/scan')          – topic to publish LaserScan
    frame_id         (string,  default 'laser_frame')   – LaserScan header frame_id
    min_range        (float,   default 0.15)            – range filter threshold (meters)
    max_range        (float,   default 12.0)            – maximum valid range (meters)
    publish_rate     (float,   default 10.0)            – max throttled publish frequency
    motor_pwm        (int,     default 660)              – motor PWM value
    sensitivity_mode (bool,    default True)             – True = Express/Sensitivity (mode 1),
                                                          False = Standard (mode 0)
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    from pyrplidar import PyRPlidar, PyRPlidarConnectionError, PyRPlidarProtocolError
    PYRPLIDAR_AVAILABLE = True
except ImportError:
    PYRPLIDAR_AVAILABLE = False
    PyRPlidar = None                     # type: ignore[assignment,misc]
    PyRPlidarConnectionError = Exception # type: ignore[assignment,misc]
    PyRPlidarProtocolError   = Exception # type: ignore[assignment,misc]


class LidarNode(Node):
    """Publishes RPLidar A1 scans as sensor_msgs/LaserScan on /scan.

    Supports both Standard (mode 0) and Sensitivity/Express (mode 1) scan modes
    via the `sensitivity_mode` ROS parameter. Automatically falls back to Standard
    if the device does not support Sensitivity mode.

    ANGLE CONVENTION: RPLidar A1 reports angles clockwise (CW). This node converts
    them to the ROS CCW convention before building the LaserScan message.

    Spins a background thread to read measurements as they arrive to prevent
    serial port buffer backlog and avoid starving the ROS 2 executor.
    """

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
        self.lidar: Optional[PyRPlidar] = None
        self._generator                  = None   # current measurement generator
        self._pending_scan               = []     # measurements accumulating for one sweep
        self._using_fallback_normal      = False  # True after express→normal fallback
        
        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)

        if not PYRPLIDAR_AVAILABLE:
            self.get_logger().fatal(
                'pyrplidar Python library not found. '
                'Install it system-wide with:\n'
                '    pip3 install pyrplidar --break-system-packages'
            )
            raise SystemExit(1)

        # Start the background scan thread
        self._thread.start()

        mode_name = 'Sensitivity/Express (mode 1)' if self.sensitivity_mode else 'Standard (mode 0)'
        self.get_logger().info(
            f'LidarNode started:\n'
            f'  port={self.serial_port}  baud={self.serial_baud}\n'
            f'  topic={self.scan_topic}  frame={self.frame_id}\n'
            f'  scan_mode={mode_name}\n'
            f'  min_range={self.min_range}m  max_range={self.max_range}m\n'
            f'  publish_rate={self.publish_rate} Hz  motor_pwm={self.motor_pwm}'
        )

    # ── Helpers ──────────────────────────────────────────────────────────

    def _warn_throttled(self, message: str, interval: float = 5.0):
        """Log a warning at most once every *interval* seconds."""
        now = time.monotonic()
        if now - self._last_warn_time >= interval:
            self.get_logger().warn(message)
            self._last_warn_time = now

    # ── Connection Management ─────────────────────────────────────────────

    def _connect_lidar(self) -> bool:
        """Attempts to connect to the RPLidar device and start the motor + generator."""
        self.get_logger().info(
            f'Connecting to RPLidar on {self.serial_port} at {self.serial_baud} baud …'
        )
        try:
            lidar = PyRPlidar()
            lidar.connect(port=self.serial_port, baudrate=self.serial_baud, timeout=3)

            # Flush the serial port RX buffer immediately after connection to clear any junk bytes
            if lidar.lidar_serial and lidar.lidar_serial._serial:
                lidar.lidar_serial._serial.reset_input_buffer()

            info   = lidar.get_info()
            health = lidar.get_health()
            self.get_logger().info(f'RPLidar device info:   {info}')
            self.get_logger().info(f'RPLidar device health: {health}')

            self.get_logger().info(f'Setting motor PWM to {self.motor_pwm} …')
            lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(1.5)  # let motor spin up and stabilise

            # Flush again after motor spin up to clear any startup electrical/noise bytes
            if lidar.lidar_serial and lidar.lidar_serial._serial:
                lidar.lidar_serial._serial.reset_input_buffer()

            # Start the appropriate scan mode
            self._generator = self._start_generator(lidar)
            if self._generator is None:
                raise RuntimeError('Failed to obtain a scan generator.')

            self.lidar = lidar
            self._pending_scan = []
            return True

        except Exception as exc:
            self.get_logger().error(f'Failed to connect to RPLidar: {exc}')
            # Safe clean up of the local variable on error
            if 'lidar' in locals():
                self._safe_disconnect(lidar)
            self._safe_disconnect(self.lidar)
            self.lidar = None
            self._generator = None
            return False

    def _start_generator(self, lidar: 'PyRPlidar'):
        """Start the scan generator on *lidar*, respecting sensitivity_mode.

        Tries Sensitivity/Express (mode 1) first when requested.
        Falls back to Standard (mode 0) and logs a warning if express fails.
        Returns the generator, or None on failure.
        """
        if self.sensitivity_mode and not self._using_fallback_normal:
            try:
                self.get_logger().info('Starting Sensitivity/Express scan (mode 1) …')
                generator_factory = lidar.start_scan_express(mode=1)
                return generator_factory()
            except Exception as exc:
                self.get_logger().warn(
                    f'Sensitivity/Express scan failed ({exc}). '
                    f'Falling back to Standard (mode 0) permanently. '
                    f'To suppress this warning, set sensitivity_mode:=false.'
                )
                self._using_fallback_normal = True
                # Flush the serial port input buffer to clear any failed express bytes
                if lidar.lidar_serial and lidar.lidar_serial._serial:
                    lidar.lidar_serial._serial.reset_input_buffer()

        # Standard scan (mode 0) — also used as fallback
        self.get_logger().info('Starting Standard scan (mode 0) …')
        try:
            generator_factory = lidar.start_scan()
            return generator_factory()
        except Exception as exc:
            self.get_logger().error(f'Standard scan also failed: {exc}')
            return None

    def _safe_disconnect(self, lidar: Optional['PyRPlidar']):
        """Stop motor and disconnect *lidar* safely (ignores errors)."""
        if lidar is None:
            return
        try:
            lidar.stop()
        except Exception:
            pass
        try:
            lidar.set_motor_pwm(0)
        except Exception:
            pass
        try:
            lidar.disconnect()
        except Exception:
            pass

    def _disconnect_lidar(self):
        """Disconnect the current self.lidar and clear state."""
        self._safe_disconnect(self.lidar)
        self.lidar      = None
        self._generator = None

    # ── Scan Loop ─────────────────────────────────────────────────────────

    def _scan_loop(self):
        """Background thread that reads measurements from RPLidar and publishes them."""
        while rclpy.ok() and self._running:
            if self.lidar is None or self._generator is None:
                self._using_fallback_normal = False
                if not self._connect_lidar():
                    time.sleep(2.0)
                    continue

            try:
                measurement = next(self._generator)
                
                # Skip invalid distance returns
                if measurement.distance == 0:
                    continue

                # New sweep: publish the accumulated scan, then reset
                if measurement.start_flag and self._pending_scan:
                    now = time.monotonic()
                    if self._min_publish_interval == 0.0 or (
                        now - self._last_publish_time >= self._min_publish_interval
                    ):
                        msg = self._build_laserscan(self._pending_scan)
                        msg.header.stamp = self.get_clock().now().to_msg()
                        self.publisher_.publish(msg)
                        self.get_logger().info(f'Published scan with {len(self._pending_scan)} points.')
                        self._last_publish_time = now
                    self._pending_scan = []

                self._pending_scan.append(measurement)

            except StopIteration:
                self._warn_throttled('Scan generator exhausted — reconnecting …')
                self._disconnect_lidar()
                time.sleep(1.0)
            except PyRPlidarConnectionError as exc:
                self._warn_throttled(f'PyRPlidarConnectionError: {exc}')
                self._disconnect_lidar()
                time.sleep(2.0)
            except PyRPlidarProtocolError as exc:
                self._warn_throttled(f'PyRPlidarProtocolError: {exc}')
                self._disconnect_lidar()
                time.sleep(2.0)
            except Exception as exc:
                self._warn_throttled(f'Unexpected error in scan loop: {exc}')
                self._disconnect_lidar()
                time.sleep(2.0)

        self._disconnect_lidar()

    # ── LaserScan Builder ─────────────────────────────────────────────────

    def _build_laserscan(self, scan):
        """Convert a list of PyRPlidarMeasurement objects to sensor_msgs/LaserScan.

        ANGLE CONVENTION:
            RPLidar A1 reports angles **clockwise** (CW):
                0° = front, 90° = right, 180° = back, 270° = left.
            ROS 2 LaserScan uses **counter-clockwise** (CCW), -π to +π:
                0° = front, +90° = left, ±180° = back, -90° = right.

            Conversion: ros_angle_deg = (360.0 - lidar_angle_deg) % 360.0

        Args:
            scan: list of PyRPlidarMeasurement
                  .quality (int), .angle (float degrees CW), .distance (float mm)

        Returns:
            sensor_msgs.msg.LaserScan populated and ready to publish.
        """
        num_readings    = 360
        angle_increment = (2.0 * math.pi) / num_readings

        ranges      = [float('inf')] * num_readings
        intensities = [0.0]          * num_readings

        for measurement in scan:
            angle_deg = measurement.angle
            distance  = measurement.distance   # mm

            # Convert raw CW angle to CCW angle in [0, 360)
            ccw_angle_deg = (360.0 - (angle_deg % 360.0)) % 360.0

            # Map to array index
            idx = int(ccw_angle_deg / 360.0 * 360.0) % num_readings

            # Convert distance
            distance_m = distance / 1000.0

            # Discard if outside [min_range, max_range]
            if distance_m < self.min_range or distance_m > self.max_range:
                continue

            # Keep closest measurement if multiple fall into the same angular bin
            if distance_m < ranges[idx]:
                ranges[idx]      = distance_m
                intensities[idx] = float(measurement.quality)

        msg = LaserScan()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = 0.0
        msg.angle_max       = (2.0 * math.pi) - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / 5.5

        msg.range_min   = self.min_range
        msg.range_max   = self.max_range
        msg.ranges      = ranges
        msg.intensities = intensities

        return msg

    # ── Shutdown ──────────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown: stop scan, motor, and disconnect."""
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
