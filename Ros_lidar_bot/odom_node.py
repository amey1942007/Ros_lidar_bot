#!/usr/bin/env python3
"""
odom_node.py — AMR4 Wheel Odometry Node
=========================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Hardware : Arduino Mega 2560 running DriveMaster.ino (mecanum bot)

What this node does
-------------------
Subscribes to /encoder (std_msgs/Float64MultiArray) — published by
amr4_driver_node.py — which carries the raw encoder counts and the Arduino
millis() timestamp they were sampled at:

    data[0] = C1  (FL — front-left)   counts
    data[1] = C2  (FR — front-right)  counts
    data[2] = C3  (RL — rear-left)    counts
    data[3] = C4  (RR — rear-right)   counts
    data[4] = T   (Arduino millis() at sample time)

Using mecanum forward kinematics (FK) — the mathematical inverse of
DriveMaster.ino's mecanumIK() — it converts the per-wheel count deltas into
a body displacement and integrates that into a dead-reckoning pose (x, y,
yaw), then divides by the Arduino dt to report body velocity.

Why counts and not RPM
----------------------
Counts are displacement; RPM is a rate averaged over the Arduino telemetry
period. Integrating "rate x dt" with dt measured on the Jetson turns USB
scheduling jitter into position error, because a late frame is credited with
extra travel that never happened. Differencing counts removes dt from the
position estimate entirely — the pose depends only on how far the wheels
actually turned. The Arduino timestamp is still used for the twist, which is
a rate by definition and only feeds the EKF.

Publishes:
    /odom_raw  (nav_msgs/Odometry)  — FK pose + velocity + covariance.
                                      Consumed by robot_localization EKF
                                      which fuses it with /imu and outputs
                                      the final /odom.
    odom → base_footprint TF        — controlled by broadcast_tf param
                                      (default False — EKF owns TF).

Mecanum Forward Kinematics
--------------------------
DriveMaster mecanumIK() (must invert exactly):
    wFL = vy + vx + omega * KIN_LW
    wFR = vy - vx + omega * KIN_LW
    wRL = vy - vx - omega * KIN_LW
    wRR = vy + vx - omega * KIN_LW

where KIN_LW = (L + W) / 2, L = CHASSIS_L (front-rear), W = CHASSIS_W (left-right).

PID setpoints then apply extra sign flips:
    pid1 = -wFL * toRPM,  pid2 = +wFR * toRPM
    pid3 = -wRL * toRPM,  pid4 = +wRR * toRPM

so measured wheel travel is converted back to IK wheel displacements with the
same flips before the inverse is applied.

Inverse of that IK matrix (identical for velocities or displacements):
    4·vy       =  wFL + wFR + wRL + wRR
    4·vx       =  wFL - wFR - wRL + wRR
    4·LW·omega =  wFL + wFR - wRL - wRR

Parameters
----------
  encoder_topic   (str)   : default "/encoder"     ← counts + T from driver
  odom_topic      (str)   : default "/odom_raw"    ← FK odometry for EKF
  base_frame_id   (str)   : default "base_footprint"
  odom_frame_id   (str)   : default "odom"
  broadcast_tf    (bool)  : publish odom→base_footprint TF (default False)
                            Keep False — EKF is the sole TF publisher.

  Encoder counts per wheel revolution (must match Config.h PPR1..PPR4):
  ppr1 / ppr2 / ppr3 / ppr4  (int) : defaults 1300 / 680 / 400 / 280
  standstill_counts (int) : per-wheel |delta| at or below this is treated
                            as zero, so encoder dither cannot creep the pose

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
from std_msgs.msg import Float64MultiArray

try:
    from tf2_ros import TransformBroadcaster
    TF2_AVAILABLE = True
except ImportError:
    TF2_AVAILABLE = False


class OdomNode(Node):
    """
    Computes and publishes wheel odometry for the AMR4 mecanum robot.

    Subscribes to /encoder (4 wheel counts + Arduino ms from amr4_driver_node).
    Publishes    /odom_raw (nav_msgs/Odometry with pose+twist+covariance).
    Optionally   broadcasts odom→base_footprint TF (keep off — EKF owns TF).
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
        self._W   = self.declare_parameter("chassis_w",    0.63).value   # metres
        self._lw  = (self._L + self._W) / 2.0   # KIN_LW in DriveMaster

        # Encoder counts per wheel revolution — MUST match Config.h PPR1..PPR4
        self._ppr = [
            float(self.declare_parameter("ppr1", 1300).value),
            float(self.declare_parameter("ppr2",  680).value),
            float(self.declare_parameter("ppr3",  400).value),
            float(self.declare_parameter("ppr4",  280).value),
        ]
        # Per-wheel count dither at rest, ignored so a parked robot cannot
        # integrate its own encoder noise into a creeping pose.
        self._standstill_counts = self.declare_parameter("standstill_counts", 2).value

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

        # ── Encoder / timing state (all from the Arduino, not the Jetson) ─────
        self._last_counts: list[float] | None = None
        self._last_arduino_ms: float | None = None
        self._last_vx    = 0.0
        self._last_vy    = 0.0
        self._last_omega = 0.0
        self._last_data_time = time.monotonic()

        # ── QoS ──────────────────────────────────────────────────────────────
        # Must match amr4_driver's /encoder publisher (RELIABLE, depth 10).
        # BEST_EFFORT here was silently dropping all encoder messages on some
        # RMW stacks → no /odom_raw → EKF never published /odom.
        encoder_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
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
        # /encoder from amr4_driver_node: Float64MultiArray([c1, c2, c3, c4, t_ms])
        self._sub_encoder = self.create_subscription(
            Float64MultiArray,
            self._encoder_topic,
            self._encoder_callback,
            encoder_qos,
        )

        # ── Watchdog: warn if no data arrives for > 2 s ───────────────────────
        self._watchdog_timer = self.create_timer(2.0, self._watchdog_callback)
        self._msg_count = 0

        self.get_logger().info(
            f"OdomNode ready — subscribing '{self._encoder_topic}' "
            f"→ publishing '{self._odom_topic}'"
        )
        self.get_logger().info(
            f"  Chassis: L={self._L} m  W={self._W} m  R={self._R} m  "
            f"KIN_LW={self._lw:.4f} m"
        )
        self.get_logger().info(
            f"  Counts/rev: {[int(p) for p in self._ppr]}  (must match Config.h PPR1..PPR4)"
        )
        self.get_logger().info(
            f"  Frames: odom='{self._odom_frame}' → base='{self._base_frame}'  "
            f"broadcast_tf={self._broadcast_tf}  (EKF owns TF when False)"
        )

    # ──────────────────────────────────────────────────────────────────────────
    # /encoder callback — runs mecanum FK and integrates pose
    # ──────────────────────────────────────────────────────────────────────────
    def _encoder_callback(self, msg: Float64MultiArray):
        """
        Receive 4-wheel counts, run mecanum FK, integrate pose, publish /odom_raw.

        Input layout (from amr4_driver_node /encoder):
            msg.data[0] = C1  FL counts
            msg.data[1] = C2  FR counts
            msg.data[2] = C3  RL counts
            msg.data[3] = C4  RR counts
            msg.data[4] = T   Arduino millis() when the counts were sampled
        """
        if len(msg.data) < 5:
            self.get_logger().warn(
                f"/encoder has {len(msg.data)} values, need 5 (C1..C4, T)",
                throttle_duration_sec=5.0,
            )
            return

        self._msg_count += 1
        if self._msg_count == 1:
            self.get_logger().info("First /encoder message received — odom pipeline live")

        self._last_data_time = time.monotonic()

        counts = [float(v) for v in msg.data[:4]]
        arduino_ms = float(msg.data[4])

        # First frame only latches the reference — there is nothing to
        # difference against yet.
        if self._last_counts is None:
            self._last_counts = counts
            self._last_arduino_ms = arduino_ms
            return

        dt = (arduino_ms - self._last_arduino_ms) / 1000.0
        deltas = [c - p for c, p in zip(counts, self._last_counts)]
        self._last_counts = counts
        self._last_arduino_ms = arduino_ms

        if dt <= 0.0:
            # Arduino clock went backwards: it rebooted (counts restart at 0)
            # or millis() wrapped. The delta is meaningless, so this frame
            # only re-syncs the reference.
            return

        # Standstill deadband: with every wheel this quiet the robot is parked
        # and the deltas are encoder dither, which would otherwise integrate
        # into a creeping pose and a slowly rotating lidar scan.
        if all(abs(d) <= self._standstill_counts for d in deltas):
            deltas = [0.0, 0.0, 0.0, 0.0]

        # Count delta → wheel travel (metres). Each wheel has its own PPR.
        circumference = 2.0 * math.pi * self._R
        travel = [(d / ppr) * circumference for d, ppr in zip(deltas, self._ppr)]

        # Recover IK wheel displacements from measured travel.
        # DriveMaster: pid1/pid3 setpoints are negated, pid2/pid4 are not.
        w_fl = -travel[0]
        w_fr =  travel[1]
        w_rl = -travel[2]
        w_rr =  travel[3]

        # Inverse of DriveMaster mecanumIK() — the same matrix works on
        # displacements as on velocities:
        #   wFL = vy + vx + r,  wFR = vy - vx + r
        #   wRL = vy - vx - r,  wRR = vy + vx - r
        d_x_body = ( w_fl - w_fr - w_rl + w_rr) / 4.0
        d_y_body = ( w_fl + w_fr + w_rl + w_rr) / 4.0
        d_yaw    = ( w_fl + w_fr - w_rl - w_rr) / (4.0 * self._lw)

        vx    = d_x_body / dt
        vy    = d_y_body / dt
        omega = d_yaw / dt

        self._last_vx    = vx
        self._last_vy    = vy
        self._last_omega = omega

        # ── Pose integration ─────────────────────────────────────────────────
        # Rotate the body displacement into the odom frame about the midpoint
        # heading of the interval, which tracks an arc far better than using
        # the heading from the start of the interval.
        mid_yaw = self._yaw + 0.5 * d_yaw
        cos_yaw = math.cos(mid_yaw)
        sin_yaw = math.sin(mid_yaw)

        self._x   += d_x_body * cos_yaw - d_y_body * sin_yaw
        self._y   += d_x_body * sin_yaw + d_y_body * cos_yaw
        self._yaw += d_yaw
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
