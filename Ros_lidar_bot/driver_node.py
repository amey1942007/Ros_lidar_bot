#!/usr/bin/env python3
"""
driver_node.py – UART Driver for DDSM115 motors (Jazzy-targeted).

Architecture:
  - Subscribes to /cmd_vel_safe (geometry_msgs/Twist)
  - Computes Differential Drive kinematics (Twist -> Wheel RPMs)
  - Communicates exclusively with DDSM115 via UART (RS485/Serial)
  - Publishes /encoder (sensor_msgs/JointState)

JointState field mapping for odom_node:
  name[]     – wheel joint name strings
  velocity[] – RPM (signed, + = forward)
  position[] – raw encoder degrees 0-360
  effort[]   – phase current in Amperes

Parameters:
  serial_port       (string, default '/dev/ttyUSB1')
  baud_rate         (int,    default 115200)
  left_wheel_name   (string, default 'left_wheel_joint')
  right_wheel_name  (string, default 'right_wheel_joint')
  left_wheel_id     (int,    default 1)
  right_wheel_id    (int,    default 2)
  wheel_radius      (float,  default 0.033)  m
  wheel_base        (float,  default 0.160)  m
  poll_rate         (float,  default 50.0)   Hz
"""

import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError:
    serial = None

# ── DDSM115 CRC8 Helper ───────────────────────────────────────────────────────

def crc8_maxim(data: bytes) -> int:
    """CRC-8/MAXIM computation for DDSM115 protocol."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
    return crc

# ── Node ──────────────────────────────────────────────────────────────────────

class DriverNode(Node):
    def __init__(self):
        super().__init__('driver_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self._port        = self.declare_parameter('serial_port', '/dev/ttyUSB1').value
        self._baud        = self.declare_parameter('baud_rate', 115200).value
        
        self._left_name   = self.declare_parameter('left_wheel_name', 'left_wheel_joint').value
        self._right_name  = self.declare_parameter('right_wheel_name', 'right_wheel_joint').value
        
        self._id_left     = self.declare_parameter('left_wheel_id', 1).value
        self._id_right    = self.declare_parameter('right_wheel_id', 2).value
        
        self._r           = self.declare_parameter('wheel_radius', 0.033).value
        self._b           = self.declare_parameter('wheel_base', 0.160).value
        self._poll_rate   = self.declare_parameter('poll_rate', 50.0).value

        # ── State ─────────────────────────────────────────────────────────────
        self._cmd_rpm_left  = 0.0
        self._cmd_rpm_right = 0.0
        
        # Feedback buffers
        self._fb_rpm_left  = 0.0
        self._fb_rpm_right = 0.0
        self._fb_pos_left  = 0.0
        self._fb_pos_right = 0.0
        self._fb_cur_left  = 0.0
        self._fb_cur_right = 0.0

        self._lock = threading.Lock()

        # ── UART Setup ────────────────────────────────────────────────────────
        self._serial = None
        self._connect_serial()

        # ── Publishers / Subscribers ──────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel_safe', self._cmd_vel_cb, 10)
        
        # Publisher for JointState (used by odom_node)
        self._enc_pub = self.create_publisher(
            JointState, 
            '/encoder', 
            QoSPresetProfiles.SENSOR_DATA.value
        )

        # ── Control Loop ──────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self._poll_rate, self._control_loop)

        self.get_logger().info(
            f'DriverNode ready on {self._port} at {self._baud} baud.\n'
            f'  IDs: Left={self._id_left}, Right={self._id_right}\n'
            f'  Kinematics: r={self._r}m, b={self._b}m'
        )

    def _connect_serial(self):
        if serial is None:
            self.get_logger().fatal("pyserial not installed. Run: pip3 install pyserial")
            raise SystemExit(1)
        
        try:
            self._serial = serial.Serial(self._port, self._baud, timeout=0.01)
            self.get_logger().info("UART connection established.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {self._port}: {e}")
            self.get_logger().error("Ensure the user has dialout/tty group permissions.")

    # ── Kinematics & Commands ─────────────────────────────────────────────────

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Convert Twist (v, w) -> Left and Right Wheel RPM."""
        v = msg.linear.x   # m/s
        w = msg.angular.z  # rad/s

        # Differential drive inverse kinematics
        # v_left = v - (w * b / 2)
        # v_right = v + (w * b / 2)
        v_left  = v - (w * self._b / 2.0)
        v_right = v + (w * self._b / 2.0)

        # Convert m/s to RPM
        # RPM = (v * 60) / (2 * pi * r)
        rad_to_rpm = 60.0 / (2.0 * math.pi * self._r)
        rpm_l = v_left * rad_to_rpm
        rpm_r = v_right * rad_to_rpm

        with self._lock:
            self._cmd_rpm_left = rpm_l
            self._cmd_rpm_right = rpm_r

    # ── Communication Loop ────────────────────────────────────────────────────

    def _send_velocity_cmd(self, motor_id: int, rpm: float):
        """Send velocity command to DDSM115."""
        if not self._serial or not self._serial.is_open:
            return

        # DDSM115 Protocol:
        # [ID, 0x64, RPM_H, RPM_L, 0x00, 0x00, 0x00, 0x00, 0x00, CRC8]
        rpm_int = int(max(min(rpm, 32767), -32768)) # Clamp to int16
        
        # Handle two's complement for negative RPM
        if rpm_int < 0:
            rpm_int = (1 << 16) + rpm_int

        cmd = bytearray([
            motor_id & 0xFF,
            0x64,
            (rpm_int >> 8) & 0xFF,
            rpm_int & 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00
        ])
        cmd.append(crc8_maxim(cmd))

        try:
            self._serial.write(cmd)
            self._serial.flush()
        except serial.SerialException as e:
            self.get_logger().error(f"UART write failed: {e}", throttle_duration_sec=5.0)

    def _read_feedback(self, expected_id: int):
        """Read and parse 10-byte feedback packet from DDSM115."""
        if not self._serial or not self._serial.is_open:
            return None

        try:
            # DDSM115 replies with exactly 10 bytes
            resp = self._serial.read(10)
            if len(resp) == 10:
                if crc8_maxim(resp[:-1]) == resp[9]:
                    recv_id = resp[0]
                    if recv_id == expected_id:
                        # Parse packet
                        # [ID, Mode, Cur_H, Cur_L, Spd_H, Spd_L, Pos_H, Pos_L, Error, CRC8]
                        cur_raw = struct.unpack('>h', resp[2:4])[0]
                        spd_raw = struct.unpack('>h', resp[4:6])[0]
                        pos_raw = struct.unpack('>H', resp[6:8])[0]
                        
                        current_a = cur_raw * 0.01  # Amps
                        rpm_fb    = float(spd_raw)
                        # Position maps 0-32767 to 0-360 degrees
                        position_deg = (pos_raw / 32767.0) * 360.0
                        
                        return (rpm_fb, position_deg, current_a)
        except serial.SerialException as e:
            self.get_logger().error(f"UART read failed: {e}", throttle_duration_sec=5.0)
            
        return None

    def _control_loop(self):
        """Main loop: send commands, read feedback, publish JointState."""
        with self._lock:
            cmd_l = self._cmd_rpm_left
            cmd_r = self._cmd_rpm_right

        # Comm with Left Motor
        self._send_velocity_cmd(self._id_left, cmd_l)
        fb_l = self._read_feedback(self._id_left)
        if fb_l:
            self._fb_rpm_left, self._fb_pos_left, self._fb_cur_left = fb_l

        # Comm with Right Motor
        # Invert right motor command if motors are mounted symmetrically
        # (Often one motor needs reversed commands, modify if needed)
        self._send_velocity_cmd(self._id_right, -cmd_r)
        fb_r = self._read_feedback(self._id_right)
        if fb_r:
            # Re-invert feedback for consistency
            rpm_r_raw, pos_r, cur_r = fb_r
            self._fb_rpm_right = -rpm_r_raw 
            self._fb_pos_right = pos_r
            self._fb_cur_right = cur_r

        # Publish JointState
        self._publish_encoder()

    def _publish_encoder(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # JointState array layout matching what odom_node expects
        msg.name     = [self._left_name, self._right_name]
        msg.velocity = [self._fb_rpm_left, self._fb_rpm_right]   # RPM
        msg.position = [self._fb_pos_left, self._fb_pos_right]   # Degrees 0-360
        msg.effort   = [self._fb_cur_left, self._fb_cur_right]   # Amps

        self._enc_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info('Shutting down DriverNode: stopping motors.')
        # Send zero velocity before closing
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

if __name__ == '__main__':
    main()
