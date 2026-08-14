#!/usr/bin/env python3
"""
odom_node.py — AMR4 Wheel Odometry Node
=========================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Hardware : Arduino Mega 2560 running DriveMaster.ino (mecanum bot)

What this node does
-------------------
Subscribes to /encoder (std_msgs/Float32MultiArray) — published by
amr4_driver_node.py — which carries the four wheel RPMs measured by the
Arduino's encoder feedback loop:

    data[0] = W1_RPM  (FL — front-left)
    data[1] = W2_RPM  (FR — front-right)
    data[2] = W3_RPM  (RL — rear-left)
    data[3] = W4_RPM  (RR — rear-right)

Using mecanum forward kinematics (FK) — the mathematical inverse of
DriveMaster.ino's mecanumIK() — it computes the robot body velocity
(vx forward, vy strafe, omega yaw) and integrates over time to maintain
a dead-reckoning pose estimate (x, y, yaw).

Publishes:
    /odom_raw  (nav_msgs/Odometry)  — FK pose + velocity + covariance.
                                      Consumed by robot_localization EKF
                                      which fuses it with /imu and outputs
                                      the final /odom.
    odom → base_footprint TF        — controlled by broadcast_tf param
                                      (default False — EKF owns TF).

Mecanum Forward Kinematics
--------------------------
DriveMaster mecanumIK():
    wFL = vy + vx + omega * KIN_LW       (W1, sign flipped in PPR)
    wFR = vy - vx + omega * KIN_LW       (W2)
    wRL = vy - vx - omega * KIN_LW       (W3, sign flipped in PPR)
    wRR = vy + vx - omega * KIN_LW       (W4)

where KIN_LW = (L + W) / 2, L = CHASSIS_L (front-rear), W = CHASSIS_W (left-right).
RPM setpoints have direction signs applied by MOTOR*_DIR in Config.h and by
the DriveMaster PID sign conventions.  The FK below accounts for the sign
conventions used in Config.h (MOTOR1_DIR=+1, MOTOR2_DIR=-1, MOTOR3_DIR=-1,
MOTOR4_DIR=-1) so the resulting velocities match the physical motion.

    w_rad_s = RPM / 60 * 2π * R   (convert RPM to wheel linear velocity)

FK (linear algebra inverse of the IK matrix):
    4·vx    =  w1 + w2 + w3 + w4
    4·vy    = -w1 + w2 - w3 + w4       (note: sign pattern differs from IK due
                                         to mecanum roller geometry)
    4·LW·omega = -w1 + w2 + w3 - w4

where w_i are the signed wheel linear velocities (m/s) after direction correction.

Parameters
----------
  encoder_topic   (str)   : default "/encoder"     ← raw RPMs from driver
  odom_topic      (str)   : default "/odom_raw"    ← FK odometry for EKF
  base_frame_id   (str)   : default "base_footprint"
  odom_frame_id   (str)   : default "odom"
  broadcast_tf    (bool)  : publish odom→base_footprint TF (default False)
                            Keep False — EKF is the sole TF publisher.

  Covariance tuning (diagonal of the 6×6 pose / twist covariance):
  pose_cov_x      (float) : default 0.01  (m²)
  pose_cov_y      (float) : default 0.01  (m²)
  pose_cov_yaw    (float) : default 0.01  (rad²)
  twist_cov_vx    (float) : default 0.01  (m²/s²)
  twist_cov_vy    (float) : default 0.01  (m²/s²)
  twist_cov_omega (float) : default 0.01  (rad²/s²)
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

try:
    from tf2_ros import TransformBroadcaster
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class OdomNode(Node):
    """
    Computes and publishes wheel odometry for the AMR4 mecanum robot.

    Subscribes to /odom_raw (4 wheel RPMs from amr4_driver_node).
    Publishes    /odom     (nav_msgs/Odometry with full pose+twist+covariance).
    Optionally   broadcasts odom→base_footprint TF.
    """

    def __init__(self):
        super().__init__("odom_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self._encoder_topic   = self.declare_parameter("encoder_topic",   "/encoder").value
        self._odom_topic      = self.declare_parameter("odom_topic",      "/odom_raw").value
        self._base_frame      = self.declare_parameter("base_frame_id",   "base_footprint").value
        self._odom_frame      = self.declare_parameter("odom_frame_id",   "odom").value
        self._broadcast_tf    = self.declare_parameter("broadcast_tf",    False).value

        # Chassis geometry — MUST match Config.h
        self._R   = self.declare_parameter("wheel_radius", 0.05).value   # metres
        self._L   = self.declare_parameter("chassis_l",    0.52).value   # metres
        self._W   = self.declare_parameter("chassis_w",    0.88).value   # metres
        self._lw  = (self._L + self._W) / 2.0   # KIN_LW in DriveMaster

        # Covariance (position diagonal entries; off-diagonal = 0 for dead-reckoning)
        self._pose_cov_x     = self.declare_parameter("pose_cov_x",      0.01).value
        self._pose_cov_y     = self.declare_parameter("pose_cov_y",      0.01).value
        self._pose_cov_yaw   = self.declare_parameter("pose_cov_yaw",    0.01).value
        self._twist_cov_vx   = self.declare_parameter("twist_cov_vx",    0.01).value
        self._twist_cov_vy   = self.declare_parameter("twist_cov_vy",    0.01).value
        self._twist_cov_omega= self.declare_parameter("twist_cov_omega", 0.01).value

        # ── Dead-reckoning pose state ─────────────────────────────────────────
        self._x   = 0.0   # metres
        self._y   = 0.0   # metres
        self._yaw = 0.0   # radians

        # ── Timing ────────────────────────────────────────────────────────────
        self._last_odom_time: float | None = None   # monotonic timestamp of last /odom_raw msg
        self._last_vx    = 0.0
        self._last_vy    = 0.0
        self._last_omega = 0.0
        self._last_data_time = time.monotonic()

        # ── QoS ──────────────────────────────────────────────────────────────
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        # /odom_raw: FK-integrated odometry for the EKF to fuse with /imu
        self._pub_odom = self.create_publisher(Odometry, self._odom_topic, 10)

        # ── TF broadcaster ────────────────────────────────────────────────────
        self._tf_broadcaster = None
        if self._broadcast_tf:
            if TF2_AVAILABLE:
                self._tf_broadcaster = TransformBroadcaster(self)
            else:
                self.get_logger().warn(
                    "tf2_ros not available — odom→base_footprint TF will NOT be broadcast. "
                    "Install tf2_ros: sudo apt install ros-humble-tf2-ros"
                )

        # ── Subscriber ────────────────────────────────────────────────────────
        # /encoder from amr4_driver_node: Float32MultiArray([rpm_FL, rpm_FR, rpm_RL, rpm_RR])
        # These are raw wheel RPMs from Arduino encoder feedback.
        # This node runs mecanum FK on them and publishes /odom_raw (Odometry)
        # which the EKF fuses with /imu to produce the final /odom.
        self._sub_odom_raw = self.create_subscription(
            Float32MultiArray,
            self._encoder_topic,
            self._odom_raw_callback,
            best_effort_qos,
        )

        # ── Watchdog: warn if no data arrives for > 2 s ───────────────────────
        self._watchdog_timer = self.create_timer(2.0, self._watchdog_callback)

        self.get_logger().info(
            f"OdomNode ready — subscribing '{self._encoder_topic}' "
            f"→ publishing '{self._odom_topic}'"
        )
        self.get_logger().info(
            f"  Chassis: L={self._L} m  W={self._W} m  R={self._R} m  "
            f"KIN_LW={self._lw:.4f} m"
        )
        self.get_logger().info(
            f"  Frames: odom='{self._odom_frame}' → base='{self._base_frame}'  "
            f"broadcast_tf={self._broadcast_tf}  (EKF owns TF when False)"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # /odom_raw callback — runs mecanum FK and integrates pose
    # ──────────────────────────────────────────────────────────────────────────
    def _odom_raw_callback(self, msg: Float32MultiArray):
        """
        Receive 4-wheel RPMs, run mecanum FK, integrate pose, publish /odom.

        Input layout (from amr4_driver_node /odom_raw):
            msg.data[0] = W1_RPM  FL — matches MOTOR1_DIR = +1 in Config.h
            msg.data[1] = W2_RPM  FR — matches MOTOR2_DIR = -1
            msg.data[2] = W3_RPM  RL — matches MOTOR3_DIR = -1
            msg.data[3] = W4_RPM  RR — matches MOTOR4_DIR = -1

        MOTOR*_DIR sign corrections are applied below so that positive RPM
        always means "wheel spinning in the direction that moves the robot
        forward" before FK is applied.
        """
        if len(msg.data) < 4:
            return

        now = time.monotonic()
        self._last_data_time = now

        # ── dt ───────────────────────────────────────────────────────────────
        if self._last_odom_time is None:
            self._last_odom_time = now
            return   # need at least two messages to compute dt
        dt = now - self._last_odom_time
        self._last_odom_time = now
        if dt <= 0.0 or dt > 1.0:
            # Sanity guard: ignore absurd dt (first message, serial gap, etc.)
            return

        # ── RPM → signed wheel linear velocity (m/s) ─────────────────────────
        # Apply the MOTOR*_DIR sign from Config.h so each value is positive
        # when the wheel moves the robot forward.
        # MOTOR1_DIR = +1,  MOTOR2_DIR = -1,  MOTOR3_DIR = -1,  MOTOR4_DIR = -1
        #
        # DriveMaster mecanumIK() also negates W1 and W3 setpoints:
        #   pid1.setpoint = -wFL * toRPM   →  measured rpm1 = negative for forward FL
        #   pid3.setpoint = -wRL * toRPM   →  measured rpm3 = negative for forward RL
        # So we need to negate W1 and W3 additionally (these wheels are "inverted"
        # in the kinematics sense — their positive RPM is backward motion for the body).
        #
        # Net sign correction per wheel:
        #   w1_signed = (-1) * MOTOR1_DIR * RPM1 = (-1)(+1) * RPM1 = -RPM1
        #   w2_signed = (+1) * MOTOR2_DIR * RPM2 = (+1)(-1) * RPM2 = -RPM2  … wait
        #
        # Let's derive carefully from mecanumIK:
        #   pid1.setpoint = -wFL * toRPM  → w1_body = -(rpm1 / toRPM)
        #   pid2.setpoint =  wFR * toRPM  → w2_body =  (rpm2 / toRPM)
        #   pid3.setpoint = -wRL * toRPM  → w3_body = -(rpm3 / toRPM)
        #   pid4.setpoint =  wRR * toRPM  → w4_body =  (rpm4 / toRPM)
        #
        # where toRPM = 60 / (2π · R), so v = RPM / toRPM * 1 = RPM * 2π·R/60

        to_rps = (2.0 * math.pi * self._R) / 60.0  # RPM → linear velocity (m/s)

        raw_rpm = msg.data
        w1 = -(float(raw_rpm[0])) * to_rps   # FL (negated per IK convention)
        w2 =  (float(raw_rpm[1])) * to_rps   # FR
        w3 = -(float(raw_rpm[2])) * to_rps   # RL (negated per IK convention)
        w4 =  (float(raw_rpm[3])) * to_rps   # RR

        # ── Mecanum Forward Kinematics ────────────────────────────────────────
        # Inverse of the IK matrix (4×3 → 3 body DOF):
        #   vx    = (w1 + w2 + w3 + w4) / 4
        #   vy    = (-w1 + w2 - w3 + w4) / 4     ← strafe (left positive)
        #   omega = (-w1 + w2 + w3 - w4) / (4 * lw)
        vx    = (w1 + w2 + w3 + w4) / 4.0
        vy    = (-w1 + w2 - w3 + w4) / 4.0
        omega = (-w1 + w2 + w3 - w4) / (4.0 * self._lw)

        self._last_vx    = vx
        self._last_vy    = vy
        self._last_omega = omega

        # ── Pose integration (Euler integration in odom frame) ────────────────
        # Project body velocity to odom frame using current heading
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)

        dx = (vx * cos_yaw - vy * sin_yaw) * dt
        dy = (vx * sin_yaw + vy * cos_yaw) * dt
        dyaw = omega * dt

        self._x   += dx
        self._y   += dy
        self._yaw += dyaw
        # Normalise yaw to [-π, π]
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        # ── Build and publish Odometry message ────────────────────────────────
        stamp = self.get_clock().now().to_msg()
        odom_msg = self._build_odom_msg(stamp, vx, vy, omega)
        self._pub_odom.publish(odom_msg)

        # ── Broadcast odom → base_footprint TF ───────────────────────────────
        if self._tf_broadcaster is not None:
            self._broadcast_tf_msg(stamp)

    # ──────────────────────────────────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────────────────────────────────
    def _build_odom_msg(self, stamp, vx: float, vy: float, omega: float) -> Odometry:
        """Build a complete nav_msgs/Odometry message from current state."""
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id  = self._base_frame

        # ── Pose ──────────────────────────────────────────────────────────────
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0

        # Yaw → quaternion
        half_yaw = self._yaw / 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        # 6×6 pose covariance (row-major, only diagonal relevant for EKF)
        # [x, y, z, roll, pitch, yaw]
        pose_cov = [0.0] * 36
        pose_cov[0]  = self._pose_cov_x    # x
        pose_cov[7]  = self._pose_cov_y    # y
        pose_cov[14] = 1e6                 # z  (flat-floor: very uncertain)
        pose_cov[21] = 1e6                 # roll  (no roll sensor)
        pose_cov[28] = 1e6                 # pitch (no pitch sensor)
        pose_cov[35] = self._pose_cov_yaw  # yaw
        msg.pose.covariance = pose_cov

        # ── Twist (body frame) ────────────────────────────────────────────────
        msg.twist.twist.linear.x  = vx
        msg.twist.twist.linear.y  = vy
        msg.twist.twist.linear.z  = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = omega

        twist_cov = [0.0] * 36
        twist_cov[0]  = self._twist_cov_vx     # vx
        twist_cov[7]  = self._twist_cov_vy     # vy
        twist_cov[14] = 1e6                    # vz
        twist_cov[21] = 1e6                    # roll rate
        twist_cov[28] = 1e6                    # pitch rate
        twist_cov[35] = self._twist_cov_omega  # yaw rate
        msg.twist.covariance = twist_cov

        return msg

    def _broadcast_tf_msg(self, stamp):
        """Broadcast the odom → base_footprint transform."""
        tf_msg = TransformStamped()
        tf_msg.header.stamp    = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id  = self._base_frame

        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0

        half_yaw = self._yaw / 2.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(half_yaw)
        tf_msg.transform.rotation.w = math.cos(half_yaw)

        self._tf_broadcaster.sendTransform(tf_msg)

    # ──────────────────────────────────────────────────────────────────────────
    # Watchdog — warn if data stops arriving
    # ──────────────────────────────────────────────────────────────────────────
    def _watchdog_callback(self):
        age = time.monotonic() - self._last_data_time
        if age > 2.0:
            self.get_logger().warn(
                f"\033[1;33m[ODOM WATCHDOG] No /encoder data for {age:.1f} s. "
                "Check amr4_driver_node is running and Arduino is connected.\033[0m",
                throttle_duration_sec=5.0,
            )


# ──────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
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
