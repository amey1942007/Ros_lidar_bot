#!/usr/bin/env python3
"""
driver_node.py — ROS 2 UART hardware driver for DDSM115 Direct Drive Smart Motors.

================================================================================
PHYSICAL ARCHITECTURE & USART/RS485 COMMUNICATION
================================================================================
DDSM115 smart motors communicate via a half-duplex RS485 bus, bridged to the host
controller (RPi 5) using a USB-to-RS485 serial UART dongle.
- Port: Typically /dev/ttyACM0 (USB virtual COM port)
- Default Baud Rate: 115200
- Protocol: Binary packets (10 bytes each) secured with CRC8/MAXIM.

================================================================================
DDSM115 BINARY PACKET STRUCTURE
================================================================================
Write Packet (Command velocity: 0x64):
  Byte 0: Motor ID (e.g. 0x01 left, 0x02 right)
  Byte 1: Command Code (0x64 = drive velocity)
  Byte 2-3: Velocity value in RPM (16-bit signed integer, Big-Endian)
  Byte 4-7: Reserved (0x00) or extra flags (0xFF for braking on Byte 7)
  Byte 8: Acceleration/Deceleration ramp parameter (or 0x00)
  Byte 9: CRC-8/MAXIM check byte

Read Packet / Feedback Request Query (0x74):
  Byte 0: Motor ID
  Byte 1: Query Code (0x74)
  Byte 2-8: 0x00 dummy values
  Byte 9: CRC-8/MAXIM check byte

Response Payload Formats:
  Byte 0: Motor ID
  Byte 1: Command Response Code
  Byte 2-3: Motor winding current raw (converts to Amperes via linear scaling)
  Byte 4-5: Winding rotational speed raw (converts to RPM)
  Byte 6: Reserved
  Byte 7: Current position raw (0 to 255 maps to 0 to 360 degrees)
  Byte 8: Error code byte
  Byte 9: CRC-8/MAXIM check byte

================================================================================
ROS 2 NODE DATA-FLOW & TIMING
================================================================================
- Subscribes to: /cmd_vel_safe (geometry_msgs/Twist)
- Converts body velocities (v, w) into left/right wheel RPMs.
- Publishes to: /encoder (sensor_msgs/JointState) carrying wheel position, velocity, and effort.
- An internal timer loop runs at the polling rate (10 Hz). To prevent RS485 bus
  collision during half-duplex transmit-receive sequences, a brief 2ms sleep is
  enforced between left and right motor transactions.
"""

import math
import struct
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:
    serial = None


def crc8_maxim(data: bytes) -> int:
    """CRC-8/MAXIM used by the DDSM115 protocol."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc & 0xFF


def make_packet(payload_9bytes) -> bytes:
    packet = bytearray(payload_9bytes)
    packet.append(crc8_maxim(packet))
    return bytes(packet)


def int16_to_bytes(value: int):
    value = int(max(min(value, 32767), -32768))
    return [(value & 0xFF00) >> 8, value & 0x00FF]


def raw_current_to_amps(raw: int) -> float:
    return (raw - (-32767)) * (8.0 - (-8.0)) / (32767 - (-32767)) + (-8.0)


def raw_position_to_degrees(raw: int) -> float:
    # Matches the working driver_control.py parser: reply byte 7 maps 0-255 to 0-360.
    return (raw / 255.0) * 360.0


class DriverNode(Node):
    def __init__(self):
        super().__init__("driver_node")

        self._port = self.declare_parameter("serial_port", "/dev/ttyACM0").value
        self._baud = self.declare_parameter("baud_rate", 115200).value
        self._left_name = self.declare_parameter("left_wheel_name", "left_wheel_joint").value
        self._right_name = self.declare_parameter("right_wheel_name", "right_wheel_joint").value
        self._id_left = self.declare_parameter("left_wheel_id", 1).value
        self._id_right = self.declare_parameter("right_wheel_id", 2).value
        self._r = self.declare_parameter("wheel_radius", 0.05035).value  # m  (dia=100.7 mm)
        self._b = self.declare_parameter("wheel_base", 0.33).value        # m  (centre-to-centre)
        # 20 Hz: doubles wheel-odometry density for the EKF (was 10 Hz — the
        # sparse, jittery updates showed up as steppy robot motion in RViz).
        # Loop budget at 20 Hz = 50 ms; typical cycle is ~2×(1 ms write +
        # ~4 ms reply) + 10 ms bus gap ≈ 20 ms, worst case with both reply
        # timeouts ≈ 52 ms — the timer then simply runs back-to-back.
        self._poll_rate = self.declare_parameter("poll_rate", 20.0).value
        self._cmd_timeout = self.declare_parameter("command_timeout", 1.0).value  # 1 s — tolerates publish jitter

        self._cmd_rpm_left = 0.0
        self._cmd_rpm_right = 0.0
        self._last_cmd_time = None

        self._fb_rpm_left = 0.0
        self._fb_rpm_right = 0.0
        self._fb_pos_left = 0.0
        self._fb_pos_right = 0.0
        self._fb_cur_left = 0.0
        self._fb_cur_right = 0.0

        self._lock = threading.Lock()
        self._serial = None
        self._connect_serial()

        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_vel_cb, 10)
        self._enc_pub = self.create_publisher(
            JointState,
            "/encoder",
            10,
        )
        self._timer = self.create_timer(1.0 / self._poll_rate, self._control_loop)

        self.get_logger().info(
            f"DriverNode ready on {self._port} at {self._baud} baud\n"
            f"  IDs: left={self._id_left}, right={self._id_right}\n"
            f"  Feedback via 0x64 velocity-command replies (0 RPM when idle)."
        )

    def _connect_serial(self):
        if serial is None:
            self.get_logger().fatal("pyserial not installed. Run: pip3 install pyserial")
            raise SystemExit(1)

        try:
            # timeout=0.005 (5 ms) per-byte: prevents each serial.read(1) call
            # from blocking up to 10 ms, which was causing cumulative timer
            # starvation (stutter) after 1–2 min of operation on a loaded RPi.
            self._serial = serial.Serial(self._port, self._baud, timeout=0.005)
            self.get_logger().info("UART connection established.")
        except serial.SerialException as exc:
            self.get_logger().fatal(f"Failed to open {self._port}: {exc}")
            self.get_logger().fatal("Check the port and dialout/tty permissions. Exiting.")
            raise SystemExit(1)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self._b / 2.0)
        v_right = v + (w * self._b / 2.0)
        mps_to_rpm = 60.0 / (2.0 * math.pi * self._r)

        with self._lock:
            self._cmd_rpm_left = v_left * mps_to_rpm
            self._cmd_rpm_right = v_right * mps_to_rpm
            self._last_cmd_time = time.monotonic()

    def _write_packet(self, packet: bytes) -> bool:
        if not self._serial or not self._serial.is_open:
            return False

        try:
            self._serial.write(packet)
            self._serial.flush()
            return True
        except serial.SerialException as exc:
            self.get_logger().error(f"UART write failed: {exc}", throttle_duration_sec=5.0)
            return False

    def _send_velocity_cmd(self, motor_id: int, rpm: float):
        rpm_hi, rpm_lo = int16_to_bytes(int(rpm))
        packet = make_packet([
            motor_id & 0xFF,
            0x64,
            rpm_hi,
            rpm_lo,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
        ])
        if self._serial and self._serial.is_open:
            try:
                self._serial.reset_input_buffer()
            except Exception:
                pass
        if self._write_packet(packet):
            return self._read_reply(motor_id)
        return None

    # timeout 0.02 s: a 10-byte DDSM115 reply at 115200 baud lands in ~1 ms +
    # a few ms of turnaround, so 20 ms is generous while keeping the worst-case
    # control-loop time inside the 20 Hz poll period.
    def _read_reply(self, expected_id: int, timeout: float = 0.02):
        if not self._serial or not self._serial.is_open:
            return None

        ring_buffer = bytearray()
        start_time = time.monotonic()

        while (time.monotonic() - start_time) <= timeout:
            try:
                data = self._serial.read(1)
            except serial.SerialException as exc:
                if "device reports readiness to read but returned no data" in str(exc):
                    # Benign Linux pyserial glitch under high load. Sleep 1ms to yield.
                    time.sleep(0.001)
                    continue
                self.get_logger().error(f"UART read failed: {exc}", throttle_duration_sec=5.0)
                return None

            if not data:
                continue

            byte = data[0]
            if len(ring_buffer) == 0:
                if byte == (expected_id & 0xFF):
                    ring_buffer.append(byte)
                continue

            if len(ring_buffer) == 1:
                if byte in (0x01, 0x02, 0x03):
                    ring_buffer.append(byte)
                else:
                    ring_buffer.clear()
                continue

            ring_buffer.append(byte)
            if len(ring_buffer) < 10:
                continue

            if crc8_maxim(ring_buffer[:-1]) != ring_buffer[9]:
                self.get_logger().warn(
                    f"DDSM115 CRC error from ID {expected_id}",
                    throttle_duration_sec=5.0,
                )
                ring_buffer.clear()
                continue

            cur_raw = struct.unpack(">h", ring_buffer[2:4])[0]
            rpm_raw = struct.unpack(">h", ring_buffer[4:6])[0]
            pos_raw = ring_buffer[7]
            error = ring_buffer[8]

            # CRC-8 is only 8 bits wide, so ~1-in-256 corrupted packets pass the
            # check by chance. The DDSM115's real range is ~±210 RPM (rated 115,
            # no-load max 200±10) — reject anything past a generous ±250 RPM
            # rather than let a garbage value get integrated into odom as a
            # multi-meter position jump.
            if abs(rpm_raw) > 250:
                self.get_logger().warn(
                    f"DDSM115 ID {expected_id} implausible RPM {rpm_raw}, discarding",
                    throttle_duration_sec=5.0,
                )
                ring_buffer.clear()
                continue

            if error:
                self.get_logger().warn(
                    f"DDSM115 ID {expected_id} error byte: {error}",
                    throttle_duration_sec=5.0,
                )

            return (
                float(rpm_raw),
                raw_position_to_degrees(pos_raw),
                raw_current_to_amps(cur_raw),
            )

        self.get_logger().warn(
            f"No DDSM115 feedback reply from ID {expected_id}",
            throttle_duration_sec=5.0,
        )
        return None

    def _control_loop(self):
        with self._lock:
            cmd_l = self._cmd_rpm_left
            cmd_r = self._cmd_rpm_right
            last_cmd_time = self._last_cmd_time

        has_recent_cmd = (
            last_cmd_time is not None
            and (time.monotonic() - last_cmd_time) <= self._cmd_timeout
        )

        # Always send a velocity command — 0 RPM when the last /cmd_vel_safe is
        # stale. The 0x64 reply carries the same feedback payload as a 0x74
        # poll, and the passive 0x74 poll was unreliable with the motor at rest
        # ("No DDSM115 feedback reply" warnings). Commanding 0 RPM on timeout
        # also acts as a failsafe stop if the teleop/planner dies mid-motion.
        if not has_recent_cmd:
            cmd_l = cmd_r = 0.0

        fb_l = self._send_velocity_cmd(self._id_left, cmd_l)
        if fb_l:
            self._fb_rpm_left, self._fb_pos_left, self._fb_cur_left = fb_l

        # Brief delay to prevent RS485 bus collision (increased to 10ms to fix CRC errors)
        time.sleep(0.010)

        fb_r = self._send_velocity_cmd(self._id_right, -cmd_r)

        if fb_r:
            rpm_r_raw, self._fb_pos_right, self._fb_cur_right = fb_r
            self._fb_rpm_right = -rpm_r_raw

        self._publish_encoder()

    def _publish_encoder(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self._left_name, self._right_name]
        msg.velocity = [self._fb_rpm_left, self._fb_rpm_right]
        # JointState.position is radians — joint_state_publisher feeds this
        # straight to the URDF wheel joints for RViz.
        msg.position = [math.radians(self._fb_pos_left), math.radians(self._fb_pos_right)]
        msg.effort = [self._fb_cur_left, self._fb_cur_right]
        self._enc_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("Shutting down DriverNode: stopping motors.")
        self._send_velocity_cmd(self._id_left, 0.0)
        self._send_velocity_cmd(self._id_right, 0.0)
        time.sleep(0.1)
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
