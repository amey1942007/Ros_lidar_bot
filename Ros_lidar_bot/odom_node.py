#!/usr/bin/env python3
"""\nodom_node.py – Differential-drive odometry node.

================================================================================
UNDERLYING SYSTEM & DATA FLOW
================================================================================
Subscribes to: /encoder (sensor_msgs/JointState)
- Field: JointState.velocity contains RPM (not rad/s) for left and right wheels
  as published by `driver_node.py`.
- Formulates: conversion of RPM -> Wheel Linear Velocity (m/s).
  v = RPM * (2 * pi * r / 60)
- Publishes to: /odom_raw (nav_msgs/Odometry)
  Frame ID: "odom", Child Frame ID: "base_footprint"
- Map options: Optionally broadcasts the "odom" -> "base_footprint" transform.

================================================================================
KINEMATIC MODEL: RUNGE-KUTTA 2ND ORDER (MIDPOINT) INTEGRATION
================================================================================
Traditional Euler Integration integrates states based on velocities at the start
of the time step (dx = v * cos(theta_k) * dt), which accumulates substantial drift
on curves because rotation happens continuously.
This node implements Runge-Kutta 2nd Order (RK2) Midpoint Integration instead:
  1. Midpoint orientation: theta_mid = theta_k + 0.5 * w * dt
  2. Planar integration using midpoint heading:
     x_next = x_k + v * cos(theta_mid) * dt
     y_next = y_k + v * sin(theta_mid) * dt
  3. Final orientation update: theta_next = wrap(theta_k + w * dt)
This reduces approximation error (O(dt³) vs O(dt²)) during rotational trajectories.

================================================================================
COVARIANCE DESIGN (DEAD-RECKONING DRIFT)
================================================================================
Unlike static covariances, this node estimates dead-reckoning drift:
- Linear travel increments linear (x, y) pose variances proportionally.
- Angular travel increments yaw (theta) pose variance.
- An additional time-based drift offset is accumulated on yaw variance.
Twist (velocity) covariance remains constant, representing sensor noise.

Parameters:
  left_wheel_name   (string, default 'left_wheel_joint')
  right_wheel_name  (string, default 'right_wheel_joint')
  wheel_radius      (float,  default 0.05035 m)   half of 100.7 mm diameter
  wheel_base        (float,  default 0.33    m)   centre-to-centre distance
  encoder_topic     (string, default '/encoder')
  odom_topic        (string, default '/odom_raw')
  odom_frame        (string, default 'odom')
  base_frame        (string, default 'base_footprint')
  broadcast_tf      (bool,   default True)\n"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import JointState
from nav_msgs.msg    import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion

try:
    from tf2_ros import TransformBroadcaster
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False


# ── Covariance growth constants (tunable) ────────────────────────────────────
#
#  Dead-reckoning error model for a differential-drive robot:
#    - Linear motion introduces longitudinal (x) error proportional to distance.
#      Typical wheel-slip σ ≈ 1-3% of distance → variance grows at ~1e-4 m²/m.
#    - Lateral (y) error accumulates mainly through heading error; grows faster.
#    - Yaw error grows both from angular motion (differential RPM noise) and
#      slowly from linear slip.  Typical gyro-free yaw: σ ≈ 0.02 rad/rad.
#
#  These are starting-point values.  Tune by:
#    1. Drive a known straight line, measure final lateral error → sets K_LINEAR_Y.
#    2. Rotate 360°, measure heading error → sets K_ANGULAR_YAW.
#    3. Use robot_localization/ekf_node diagnostics to refine.
#
K_LINEAR_X   = 5e-4   # var_x   increase per metre of linear travel   [m²/m]
K_LINEAR_Y   = 2e-3   # var_y   increase per metre (heading error leaks into y)
K_ANGULAR_YAW = 1e-2  # var_yaw increase per radian of angular travel  [rad²/rad]
K_DRIFT_YAW  = 1e-6   # var_yaw time-based drift                       [rad²/s]

# Twist (velocity) covariance — fixed, represents per-cycle RPM noise.
# The DDSM115 runs its own internal closed-loop velocity control and its RPM
# feedback has been bench-verified against measured distance — trust it.
#   σ_vx ≈ 0.02 m/s   (sub-RPM feedback resolution at these speeds)
#   σ_wz ≈ 0.05 rad/s (differential of two precise wheels across 0.33 m base;
#                      kept above the gyro's σ≈0.01 rad/s so the EKF lets the
#                      gyro lead during turns, while the exact-zero wheel
#                      reading still pins yaw rate when parked)
VAR_VX  = 4.0e-4   # 0.02²
VAR_WZ  = 2.5e-3   # 0.05²
LARGE   = 1.0e6    # unmeasured / not applicable DOF


# ── Pure-function helpers ─────────────────────────────────────────────────────

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """
    Convert a planar yaw angle (rad) to a normalised geometry_msgs/Quaternion.
    roll = pitch = 0 for a 2-D robot.

    q = [0, 0, sin(yaw/2), cos(yaw/2)]
    Normalisation check: sin²+cos² = 1 ✓ — always unit by construction.
    """
    half = yaw * 0.5
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q   # ||q|| = 1 by construction


def wrap_angle(angle: float) -> float:
    """Wrap angle to (-π, π]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# ── Node ──────────────────────────────────────────────────────────────────────

class OdomNode(Node):
    """
    Integrate wheel-encoder data from sensor_msgs/JointState into
    nav_msgs/Odometry using RK2 (midpoint) dead-reckoning.
    """

    def __init__(self):
        super().__init__('odom_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self._left_name  = self.declare_parameter(
            'left_wheel_name', 'left_wheel_joint').value
        self._right_name = self.declare_parameter(
            'right_wheel_name', 'right_wheel_joint').value
        self._r          = self.declare_parameter('wheel_radius', 0.05035).value  # m  (dia=100.7 mm)
        self._b          = self.declare_parameter('wheel_base',   0.33).value    # m  (centre-to-centre)
        enc_topic        = self.declare_parameter('encoder_topic', '/encoder').value
        odom_topic       = self.declare_parameter('odom_topic',   '/odom_raw').value
        self._odom_frame = self.declare_parameter('odom_frame',   'odom').value
        self._base_frame = self.declare_parameter('base_frame',   'base_footprint').value
        broadcast_tf     = self.declare_parameter('broadcast_tf',  True).value

        # ── Pose state ────────────────────────────────────────────────────────
        self._x     = 0.0   # m
        self._y     = 0.0   # m
        self._theta = 0.0   # rad, wrapped to (-π, π]

        # ── Accumulated pose covariance diagonal (σ²) ─────────────────────────
        # Start with a small non-zero seed; grows every cycle.
        self._var_x   = 1e-4   # m²
        self._var_y   = 1e-4   # m²
        self._var_yaw = 1e-4   # rad²

        # ── Previous encoder-message stamp ────────────────────────────────────
        # dt comes from the DRIVER's header.stamp (when the RPM was sampled),
        # not from our receipt time: serial transactions add 10-40 ms of
        # arrival jitter, and jittery dt × velocity = position noise that the
        # scan matcher then has to snap-correct.
        self._last_time_ns: int | None = None

        # ── Publisher ─────────────────────────────────────────────────────────
        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # ── TF broadcaster ────────────────────────────────────────────────────
        self._tf_br = None
        if broadcast_tf:
            if _TF2_AVAILABLE:
                self._tf_br = TransformBroadcaster(self)
            else:
                self.get_logger().warn(
                    'tf2_ros unavailable — TF broadcasting disabled.')

        # ── Subscriber ────────────────────────────────────────────────────────
        # Use SENSOR_DATA preset: BEST_EFFORT reliability, VOLATILE durability,
        # depth=5.  Matches what most hardware drivers use on Jazzy (same as
        # Humble/Iron — no change).
        self.create_subscription(
            JointState,
            enc_topic,
            self._encoder_cb,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.get_logger().info(
            f'OdomNode ready\n'
            f'  encoder : {enc_topic}  [{self._left_name}, {self._right_name}]\n'
            f'  odom    : {odom_topic}  ({self._odom_frame} → {self._base_frame})\n'
            f'  wheel_r : {self._r} m   wheel_base: {self._b} m\n'
            f'  ⚠ Expects velocity[] in RPM (DDSM115 driver convention).'
        )

    # ── Encoder callback ──────────────────────────────────────────────────────

    def _encoder_cb(self, msg: JointState) -> None:
        """
        Process one JointState tick from the DDSM115 driver.

        Unit note (Rule 9):
          msg.velocity[i] is in RPM   (driver publishes RPM, not rad/s).
          Conversion: ω_wheel [rad/s] = RPM × 2π / 60
                      v_wheel [m/s]   = ω_wheel × wheel_radius
        """
        # ── 1. Time base: the driver's sample stamp (fall back to our clock) ──
        stamp  = msg.header.stamp
        now_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if now_ns == 0:   # unstamped message — fall back to receipt time
            now    = self.get_clock().now()
            now_ns = now.nanoseconds
            stamp  = now.to_msg()

        if self._last_time_ns is None:
            self._last_time_ns = now_ns
            return   # need two stamps for a dt

        dt = (now_ns - self._last_time_ns) * 1e-9   # seconds
        if dt <= 0.0 or dt > 1.0:
            self._last_time_ns = now_ns
            return   # duplicate, clock jump, or stale gap — skip integration

        # ── 2. Extract left / right RPM by joint name ─────────────────────────
        names = list(msg.name)
        try:
            idx_l = names.index(self._left_name)
            idx_r = names.index(self._right_name)
        except ValueError:
            self.get_logger().warn(
                f'odom_node: wheel names not found in JointState.name.\n'
                f'  expected : [{self._left_name}, {self._right_name}]\n'
                f'  received : {names}',
                throttle_duration_sec=5.0,
            )
            return

        rpm_left  = msg.velocity[idx_l]   # RPM, signed
        rpm_right = msg.velocity[idx_r]   # RPM, signed

        # Defense in depth: driver_node already rejects implausible RPM at the
        # source, but skip integration here too rather than trust any /encoder
        # publisher blindly (the DDSM115's real range is ~±210 RPM).
        if abs(rpm_left) > 250 or abs(rpm_right) > 250:
            self.get_logger().warn(
                f'odom_node: implausible RPM (left={rpm_left}, right={rpm_right}), '
                f'skipping this tick',
                throttle_duration_sec=5.0,
            )
            self._last_time_ns = now_ns
            return

        # ── 3. RPM → wheel linear velocity (m/s) ─────────────────────────────
        #   ω  [rad/s] = RPM × 2π / 60
        #   v  [m/s]   = ω × r
        #
        #   Combined: v [m/s] = RPM × (2π × r / 60)
        _rpm_to_v = (2.0 * math.pi * self._r) / 60.0

        v_left  = rpm_left  * _rpm_to_v   # m/s, signed
        v_right = rpm_right * _rpm_to_v   # m/s, signed

        # ── 4. Differential-drive body velocities ─────────────────────────────
        #   v  [m/s]   = (v_right + v_left) / 2        — linear (forward +)
        #   ω  [rad/s] = (v_right - v_left) / b        — angular (CCW +)
        v_linear  = (v_right + v_left)  / 2.0          # m/s
        v_angular = (v_right - v_left)  / self._b      # rad/s

        # ── 5. RK2 (midpoint) pose integration ───────────────────────────────
        #
        #  Euler (first-order): integrates heading at start of step.
        #    x += v·cos(θ)·dt   ← heading error accumulates linearly
        #
        #  RK2 midpoint (second-order): integrates at mid-step heading.
        #    θ_mid  = θ_k + ½·ω·dt
        #    x_k+1  = x_k + v·cos(θ_mid)·dt
        #    y_k+1  = y_k + v·sin(θ_mid)·dt
        #    θ_k+1  = θ_k + ω·dt
        #
        #  RK2 error ∝ dt³ (vs Euler dt²), significantly better for curves.
        #
        theta_mid      = self._theta + 0.5 * v_angular * dt
        self._x       += v_linear * math.cos(theta_mid) * dt
        self._y       += v_linear * math.sin(theta_mid) * dt
        self._theta    = wrap_angle(self._theta + v_angular * dt)

        # ── 6. Growing pose covariance (dead-reckoning drift model) ───────────
        #
        #  Each metre of linear travel adds K_LINEAR_X to σ²_x (encoder slip).
        #  Yaw error leaks lateral uncertainty, so σ²_y grows faster.
        #  Each radian of angular travel adds K_ANGULAR_YAW to σ²_yaw.
        #  A small time-based term captures gyro-free yaw drift.
        #
        #  These are per-step increments; they accumulate over the trajectory.
        #  The EKF will still correct/reset them via loop-closure or IMU fusion.
        #
        dist = abs(v_linear)  * dt   # metres this step
        dang = abs(v_angular) * dt   # radians this step

        self._var_x   += K_LINEAR_X   * dist
        self._var_y   += K_LINEAR_Y   * dist + K_ANGULAR_YAW * dang
        self._var_yaw += K_ANGULAR_YAW * dang + K_DRIFT_YAW  * dt

        # ── 7. Build Odometry message ─────────────────────────────────────────
        q = yaw_to_quaternion(self._theta)   # normalised by construction

        odom = Odometry()
        odom.header.stamp    = stamp          # single stamp from step 1
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id  = self._base_frame

        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = q

        # Pose covariance 6×6 row-major [x, y, z, roll, pitch, yaw]
        # z / roll / pitch are unmeasured for a 2-D robot → LARGE
        odom.pose.covariance = [
            self._var_x, 0.0,          0.0,   0.0,   0.0,   0.0,
            0.0,          self._var_y,  0.0,   0.0,   0.0,   0.0,
            0.0,          0.0,          LARGE, 0.0,   0.0,   0.0,
            0.0,          0.0,          0.0,   LARGE, 0.0,   0.0,
            0.0,          0.0,          0.0,   0.0,   LARGE, 0.0,
            0.0,          0.0,          0.0,   0.0,   0.0,   self._var_yaw,
        ]

        odom.twist.twist.linear.x  = v_linear
        odom.twist.twist.linear.y  = 0.0
        odom.twist.twist.linear.z  = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = v_angular

        # Twist covariance 6×6 row-major [vx, vy, vz, ωx, ωy, ωz]
        # Fixed: represents per-cycle RPM measurement noise.
        # vy / vz / ωx / ωy → unmeasured, set LARGE so EKF ignores them.
        odom.twist.covariance = [
            VAR_VX, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,    LARGE, 0.0,   0.0,   0.0,   0.0,
            0.0,    0.0,   LARGE, 0.0,   0.0,   0.0,
            0.0,    0.0,   0.0,   LARGE, 0.0,   0.0,
            0.0,    0.0,   0.0,   0.0,   LARGE, 0.0,
            0.0,    0.0,   0.0,   0.0,   0.0,   VAR_WZ,
        ]

        self._odom_pub.publish(odom)

        # ── 8. TF broadcast: odom → base_footprint ───────────────────────────
        if self._tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp    = stamp        # same stamp as odom msg
            tf_msg.header.frame_id = self._odom_frame
            tf_msg.child_frame_id  = self._base_frame

            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation      = q    # same normalised quaternion

            self._tf_br.sendTransform(tf_msg)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # Under ros2 launch, SIGINT already shut the context down — calling
        # shutdown() again raises and makes launch report "process has died".
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
