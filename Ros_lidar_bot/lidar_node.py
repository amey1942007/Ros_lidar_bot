#!/usr/bin/env python3
"""
lidar_node.py – ROS 2 node for the RPLidar A1 laser scanner.

================================================================================
PHYSICAL ARCHITECTURE & UART COMMUNICATION
================================================================================
The RPLidar A1 laser scanner utilizes a serial UART channel to communicate with
the host controller (Jetson Orin Nano running Ubuntu 22.04 / ROS 2 Humble).
- Physical Hardware Interface: USB serial converter → /dev/ttyUSB0
- Default Baud Rate: 115200
- Native Scan rate: ~5.5 Hz rotating sweep
- Motor control: Driven via set_motor_pwm(pwm_value).

================================================================================
SCAN MODE — SENSITIVITY / EXPRESS (MODE 1)
================================================================================
This node uses the RPLidar A1 Sensitivity / Express scan mode (mode 1) via
pyrplidar. Express mode provides significantly higher point density and better
range performance than the standard scan mode.

The pyrplidar library (NOT the standard rplidar_ros SDK) is required because it
exposes the start_scan_express() API needed for sensitivity mode. The official
rplidar_ros apt package does not support this.

Install on Jetson Orin Nano (Ubuntu 22.04):
    pip3 install pyrplidar --break-system-packages

If the device does not support sensitivity mode, the node falls back to
Standard scan (mode 0) automatically and logs a warning.

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
DEPENDENCY DETAIL (HUMBLE / JETSON TARGETED)
================================================================================
Uses pyrplidar (pip3 install pyrplidar --break-system-packages).
pyrplidar supports the full RPLidar SDK including Express/Sensitivity scan mode.

Do NOT use ros-humble-rplidar-ros from apt — that package only supports the
standard scan mode and will NOT work with sensitivity_mode=True.

================================================================================
ROS PARAMETERS
================================================================================
    serial_port      (string,  default '/dev/ttyUSB0')  – serial device port
    serial_baud      (int,     default 115200)           – baud rate
    scan_topic       (string,  default '/scan')          – topic to publish LaserScan
    frame_id         (string,  default 'laser_frame')   – LaserScan header frame_id
    min_range        (float,   default 0.15)            – range filter threshold (meters)
                                                          A1 minimum reliable range ~0.15 m
    max_range        (float,   default 12.0)            – maximum valid range (meters)
    motor_pwm        (int,     default 660)              – motor PWM value (A1 nominal: 600-700)
    sensitivity_mode (bool,    default True)             – True = Express/Sensitivity (mode 1)
                                                           False = Standard (mode 0)
    num_bins         (int,     default 360)              – angular bins per full revolution
    publish_rate     (float,   default 0.0)              – max throttled publish Hz
                                                           0.0 = publish every complete sweep
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
    """
    Publishes RPLidar A1 scans as sensor_msgs/LaserScan on /scan.

    Uses Express/Sensitivity mode (mode 1) via pyrplidar for higher point
    density and better range than the standard scan mode.  Falls back to
    Standard automatically if the device does not support Sensitivity mode.

    ANGLE CONVENTION: RPLidar A1 reports angles clockwise (CW). This node
    converts them to the ROS CCW convention before building the LaserScan.

    Spins a background thread to read measurements as they arrive, preventing
    serial port buffer backlog and avoiding starvation of the ROS 2 executor.
    """

    def __init__(self):
        super().__init__('lidar_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.serial_port      = self.declare_parameter('serial_port',      '/dev/ttyUSB0').value
        self.serial_baud      = self.declare_parameter('serial_baud',      115200).value
        self.scan_topic       = self.declare_parameter('scan_topic',        '/scan').value
        self.frame_id         = self.declare_parameter('frame_id',          'laser_frame').value
        self.min_range        = self.declare_parameter('min_range',         0.15).value
        self.max_range        = self.declare_parameter('max_range',         12.0).value
        self.motor_pwm        = self.declare_parameter('motor_pwm',         660).value
        self.sensitivity_mode = self.declare_parameter('sensitivity_mode',  True).value
        self.num_bins         = self.declare_parameter('num_bins',          360).value
        self.publish_rate     = self.declare_parameter('publish_rate',      0.0).value

        self._min_publish_interval = (
            1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.0
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self.publisher_ = self.create_publisher(LaserScan, self.scan_topic, 10)

        # ── Throttle helpers ──────────────────────────────────────────────────
        self._last_warn_time    = 0.0
        self._last_publish_time = 0.0

        # ── Connection + Scan Thread State ────────────────────────────────────
        self.lidar: Optional[PyRPlidar] = None
        self._generator                  = None   # current measurement generator
        self._pending_scan               = []     # measurements accumulating for one sweep
        self._using_fallback_normal      = False  # True after express→normal fallback
        self._scan_start_time            = None   # ROS time at sweep start

        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)

        if not PYRPLIDAR_AVAILABLE:
            self.get_logger().fatal(
                'pyrplidar Python library not found. '
                'Install on Jetson Orin Nano with:\n'
                '    pip3 install pyrplidar --break-system-packages\n'
                'Do NOT use the apt ros-humble-rplidar-ros package — it does '
                'not support sensitivity/express mode.'
            )
            raise SystemExit(1)

        # Start the background scan thread
        self._thread.start()

        mode_name = 'Sensitivity/Express (mode 1)' if self.sensitivity_mode else 'Standard (mode 0)'
        self.get_logger().info(
            f'LidarNode started (RPLidar A1 · Jetson Orin Nano):\n'
            f'  port={self.serial_port}  baud={self.serial_baud}\n'
            f'  topic={self.scan_topic}  frame={self.frame_id}\n'
            f'  scan_mode={mode_name}\n'
            f'  min_range={self.min_range} m  max_range={self.max_range} m\n'
            f'  num_bins={self.num_bins}  motor_pwm={self.motor_pwm}\n'
            f'  publish_rate={"unlimited (every sweep)" if self.publish_rate == 0.0 else f"{self.publish_rate} Hz"}'
        )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _warn_throttled(self, message: str, interval: float = 5.0):
        """Log a warning at most once every *interval* seconds."""
        now = time.monotonic()
        if now - self._last_warn_time >= interval:
            self.get_logger().warn(message)
            self._last_warn_time = now

    # ── Connection Management ─────────────────────────────────────────────────

    def _connect_lidar(self) -> bool:
        """Connect to the RPLidar A1 and start the motor + scan generator."""
        self.get_logger().info(
            f'Connecting to RPLidar A1 on {self.serial_port} at {self.serial_baud} baud …'
        )
        try:
            lidar = PyRPlidar()
            lidar.connect(port=self.serial_port, baudrate=self.serial_baud, timeout=3)

            # Flush the serial RX buffer immediately after connection to clear any
            # junk bytes from previous sessions or power-on garbage
            if lidar.lidar_serial and lidar.lidar_serial._serial:
                lidar.lidar_serial._serial.reset_input_buffer()

            info   = lidar.get_info()
            health = lidar.get_health()
            self.get_logger().info(f'RPLidar A1 device info:   {info}')
            self.get_logger().info(f'RPLidar A1 device health: {health}')

            self.get_logger().info(f'Setting motor PWM to {self.motor_pwm} …')
            lidar.set_motor_pwm(self.motor_pwm)
            time.sleep(1.5)  # let motor spin up and stabilise to rated RPM

            # Flush again after motor spin-up to discard electrical noise bytes
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
            self.get_logger().error(f'Failed to connect to RPLidar A1: {exc}')
            if 'lidar' in locals():
                self._safe_disconnect(lidar)
            self._safe_disconnect(self.lidar)
            self.lidar = None
            self._generator = None
            return False

    def _start_generator(self, lidar: 'PyRPlidar'):
        """
        Start the scan generator on *lidar* in sensitivity mode if requested.

        Sensitivity mode (Express mode 1) on the RPLidar A1 provides:
        - Higher angular resolution (more points per revolution)
        - Better sensitivity for detecting low-reflectance surfaces
        - Slightly higher CPU usage than standard mode

        Falls back to Standard (mode 0) automatically if Express fails.
        Returns the generator, or None on total failure.
        """
        if self.sensitivity_mode and not self._using_fallback_normal:
            try:
                self.get_logger().info(
                    'Starting RPLidar A1 in Sensitivity/Express scan mode (mode 1) …'
                )
                generator_factory = lidar.start_scan_express(mode=1)
                gen = generator_factory()
                self.get_logger().info(
                    'Sensitivity/Express scan mode active — higher point density enabled.'
                )
                return gen
            except Exception as exc:
                self.get_logger().warn(
                    f'Sensitivity/Express scan failed ({exc}). '
                    f'Falling back to Standard scan (mode 0) permanently. '
                    f'To suppress this warning, set sensitivity_mode:=false in the launch file.'
                )
                self._using_fallback_normal = True
                # Flush the RX buffer to clear any failed express mode bytes
                if lidar.lidar_serial and lidar.lidar_serial._serial:
                    lidar.lidar_serial._serial.reset_input_buffer()

        # Standard scan (mode 0) — also used as automatic fallback
        self.get_logger().info('Starting Standard scan (mode 0) …')
        try:
            generator_factory = lidar.start_scan()
            gen = generator_factory()
            return gen
        except Exception as exc:
            self.get_logger().error(f'Standard scan also failed: {exc}')
            return None

    def _safe_disconnect(self, lidar: Optional['PyRPlidar']):
        """Stop motor and disconnect *lidar* safely (ignores all errors)."""
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

    # ── Scan Loop ─────────────────────────────────────────────────────────────

    def _scan_loop(self):
        """
        Background thread: read measurements from RPLidar and publish LaserScan.

        Processes one measurement at a time from the generator. Accumulates
        measurements into self._pending_scan until a start_flag marks the
        beginning of a new sweep, then builds and publishes the complete scan.
        This approach ensures every sweep is complete before publishing.
        """
        while rclpy.ok() and self._running:
            if self.lidar is None or self._generator is None:
                # Reset fallback flag so each reconnect tries sensitivity first
                self._using_fallback_normal = False
                if not self._connect_lidar():
                    time.sleep(2.0)
                    continue

            try:
                measurement = next(self._generator)

                # Skip zero-distance returns (sensor blind spot / reflection errors)
                if measurement.distance == 0:
                    continue

                # Track scan start time for accurate timestamping
                # Using the time when the first point of the sweep arrived
                # reduces motion distortion compared to stamping at publish time.
                if not self._pending_scan:
                    self._scan_start_time = self.get_clock().now()

                # New sweep: publish the accumulated scan, then reset for next sweep
                if measurement.start_flag and self._pending_scan:
                    now = time.monotonic()
                    if self._min_publish_interval == 0.0 or (
                        now - self._last_publish_time >= self._min_publish_interval
                    ):
                        msg = self._build_laserscan(self._pending_scan)
                        stamp = self._scan_start_time if self._scan_start_time else self.get_clock().now()
                        msg.header.stamp = stamp.to_msg()
                        self.publisher_.publish(msg)
                        self._last_publish_time = now
                    self._pending_scan = []
                    self._scan_start_time = None

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

    # ── LaserScan Builder ─────────────────────────────────────────────────────

    def _build_laserscan(self, scan) -> LaserScan:
        """
        Convert a list of PyRPlidarMeasurement objects to sensor_msgs/LaserScan.

        ANGLE CONVENTION:
            RPLidar A1 reports angles clockwise (CW):
                0° = front, 90° = right, 180° = back, 270° = left.
            ROS 2 LaserScan uses counter-clockwise (CCW):
                0° = front, +90° = left, ±180° = back, -90° = right.

            Conversion: ros_angle_deg = (360.0 - lidar_angle_deg) % 360.0

        The output array spans [0, 2π) in angle_increment steps.
        When multiple measurements fall into the same angular bin, the closest
        one is kept (conservative: report real obstacles, not noise).

        Args:
            scan: list of PyRPlidarMeasurement
                  .quality (int), .angle (float degrees CW), .distance (float mm)

        Returns:
            sensor_msgs.msg.LaserScan populated and ready to publish.
        """
        num_readings    = self.num_bins
        angle_increment = (2.0 * math.pi) / num_readings

        ranges      = [float('inf')] * num_readings
        intensities = [0.0]          * num_readings

        for measurement in scan:
            angle_deg = measurement.angle
            distance  = measurement.distance   # mm

            # Convert raw CW angle to CCW angle in [0, 360)
            ccw_angle_deg = (360.0 - (angle_deg % 360.0)) % 360.0

            # Map CCW angle to array index
            idx = int(ccw_angle_deg / 360.0 * num_readings) % num_readings

            # Convert distance from mm to metres
            distance_m = distance / 1000.0

            # Discard if outside [min_range, max_range]
            if distance_m < self.min_range or distance_m > self.max_range:
                continue

            # Keep the closest measurement per angular bin (obstacle-conservative)
            if distance_m < ranges[idx]:
                ranges[idx]      = distance_m
                intensities[idx] = float(measurement.quality)

        msg = LaserScan()
        msg.header.frame_id = self.frame_id

        msg.angle_min       = 0.0
        msg.angle_max       = (2.0 * math.pi) - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment  = 0.0
        msg.scan_time       = 1.0 / 5.5    # RPLidar A1 nominal sweep rate

        msg.range_min   = self.min_range
        msg.range_max   = self.max_range
        msg.ranges      = ranges
        msg.intensities = intensities

        return msg

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        """Clean shutdown: stop scan, motor, and disconnect."""
        self.get_logger().info('Shutting down LidarNode (RPLidar A1) …')
        self._running = False
        self._disconnect_lidar()
        super().destroy_node()


# ── Entry point ───────────────────────────────────────────────────────────────

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
