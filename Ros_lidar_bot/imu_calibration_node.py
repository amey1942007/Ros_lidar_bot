#!/usr/bin/env python3
"""
imu_calibration_node.py — Gyroscope & Magnetometer calibration via in-place rotation.

================================================================================
OVERVIEW
================================================================================
This node performs an automated IMU calibration sequence by spinning the robot
in place through one or more full 360° rotations, collecting raw sensor data,
and computing calibration offsets.

Calibration Procedure:
  1. [IDLE]      Node starts, waits for IMU data to become available.
  2. [WARMUP]    Collects stationary samples for gyroscope bias (zero-rate offset).
  3. [ROTATING]  Publishes /cmd_vel to rotate the robot in-place. Collects
                 angular_velocity data for gyro, and (if /magnetic_field topic
                 available) magnetometer readings.
  4. [DONE]      Stops the robot, computes offsets, prints results, and saves
                 calibration to a YAML file (default: config/imu_calibration.yaml).

Gyroscope Calibration:
  - Computes per-axis bias (mean angular velocity while stationary).
  - Computes scale factor (ratio of integrated yaw vs expected 360deg).

Magnetometer Calibration (Hard-Iron + Soft-Iron):
  - Collects magnetic_field vectors during rotation.
  - Fits a 2D ellipse to (Bx, By) — hard-iron offset = ellipse centre.
  - Soft-iron scale factors derived from ellipse semi-axes.
  NOTE: Requires /magnetic_field topic (sensor_msgs/MagneticField). If not
        available, only gyro calibration is performed.

Parameters (override via --ros-args -p):
  cmd_vel_topic    (string,  default '/cmd_vel')        — velocity command topic
  imu_topic        (string,  default '/imu')            — IMU data topic
  mag_topic        (string,  default '/magnetic_field') — magnetometer topic
  output_yaml_path (string,  default '')                — path to save calibration YAML
  rotation_speed   (float,   default 0.5)               — angular speed in rad/s
  rotation_count   (float,   default 2.0)               — number of full rotations
  warmup_duration  (float,   default 3.0)               — stationary warmup time (s)
  min_mag_samples  (int,     default 200)               — min mag samples needed
"""

import math
import os
import time
import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Imu

try:
    from sensor_msgs.msg import MagneticField
    _HAS_MAG_MSG = True
except ImportError:
    _HAS_MAG_MSG = False

try:
    import yaml
    _HAS_YAML = True
except ImportError:
    _HAS_YAML = False

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False


# ── Calibration FSM states ─────────────────────────────────────────────────────
class CalibState:
    IDLE     = "IDLE"
    WARMUP   = "WARMUP"
    ROTATING = "ROTATING"
    DONE     = "DONE"


# ── 2-D ellipse fitting (algebraic least-squares) ─────────────────────────────
def _fit_ellipse_center_numpy(xs: List[float], ys: List[float]) -> Tuple[float, float, float, float]:
    """
    Fit an ellipse to 2D (x, y) points.
    Returns (cx, cy, semi_major, semi_minor).
    Requires numpy.
    """
    import numpy as np
    X = np.array(xs, dtype=np.float64)
    Y = np.array(ys, dtype=np.float64)

    D = np.column_stack([X**2, X * Y, Y**2, X, Y, np.ones(len(X))])
    S = D.T @ D
    C = np.zeros((6, 6))
    C[0, 2] = 2.0
    C[2, 0] = 2.0
    C[1, 1] = -1.0

    try:
        eigenvalues, eigenvectors = np.linalg.eig(np.linalg.inv(S) @ C)
    except np.linalg.LinAlgError:
        return 0.0, 0.0, 1.0, 1.0

    positive_mask = eigenvalues > 0
    if not np.any(positive_mask):
        return 0.0, 0.0, 1.0, 1.0

    idx = np.argmin(eigenvalues[positive_mask])
    coeff = eigenvectors[:, np.where(positive_mask)[0][idx]].real

    A, B, C_c, D_c, E, F = coeff
    denom = 4.0 * A * C_c - B**2
    if abs(denom) < 1e-12:
        return 0.0, 0.0, 1.0, 1.0

    cx = (B * E - 2.0 * C_c * D_c) / denom
    cy = (B * D_c - 2.0 * A * E) / denom

    M = np.array([[A, B / 2.0], [B / 2.0, C_c]])
    det_M = np.linalg.det(M)
    trace_M = A + C_c
    inner = math.sqrt(max(0.0, trace_M**2 - 4.0 * det_M))
    lam1 = (trace_M + inner) / 2.0
    lam2 = (trace_M - inner) / 2.0

    val = A * cx**2 + B * cx * cy + C_c * cy**2 - F
    if val < 0 or abs(lam1) < 1e-12 or abs(lam2) < 1e-12:
        return cx, cy, 1.0, 1.0

    a = math.sqrt(abs(val / lam2))
    b = math.sqrt(abs(val / lam1))
    return cx, cy, max(a, b), min(a, b)


def _simple_mag_center(xs: List[float], ys: List[float]) -> Tuple[float, float]:
    """Fallback: centre from axis min/max midpoint."""
    return (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0


# ── Minimal YAML serialiser (no-deps fallback) ─────────────────────────────────
def _dict_to_yaml(obj, indent: int = 0) -> str:
    pad = "  " * indent
    lines = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            if isinstance(v, dict):
                lines.append(f"{pad}{k}:")
                lines.append(_dict_to_yaml(v, indent + 1))
            elif isinstance(v, list):
                lines.append(f"{pad}{k}:")
                for item in v:
                    lines.append(f"{pad}  - {item}")
            else:
                lines.append(f"{pad}{k}: {v}")
    elif isinstance(obj, list):
        for item in obj:
            lines.append(f"{pad}- {item}")
    else:
        lines.append(f"{pad}{obj}")
    return "\n".join(lines)


# ── Calibration Node ───────────────────────────────────────────────────────────
class ImuCalibrationNode(Node):
    """
    Rotates the robot in-place and computes IMU calibration offsets for
    gyroscope bias and magnetometer hard-iron / soft-iron correction.
    """

    def __init__(self):
        super().__init__("imu_calibration_node")

        # ── Parameters ────────────────────────────────────────────────────────
        self._cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        self._imu_topic     = self.declare_parameter("imu_topic", "/imu").value
        self._mag_topic     = self.declare_parameter("mag_topic", "/magnetic_field").value
        self._output_yaml   = self.declare_parameter("output_yaml_path", "").value
        self._rot_speed     = self.declare_parameter("rotation_speed", 0.5).value
        self._rot_count     = self.declare_parameter("rotation_count", 2.0).value
        self._warmup_dur    = self.declare_parameter("warmup_duration", 3.0).value
        self._min_mag       = self.declare_parameter("min_mag_samples", 200).value

        # ── Resolve output path ────────────────────────────────────────────────
        if not self._output_yaml:
            pkg_dir = Path(__file__).resolve().parent.parent
            self._output_yaml = str(pkg_dir / "config" / "imu_calibration.yaml")

        # ── State machine ─────────────────────────────────────────────────────
        self._state            = CalibState.IDLE
        self._state_start_time = time.monotonic()
        self._last_imu_msg_time = time.monotonic()

        # ── Data accumulators ─────────────────────────────────────────────────
        self._gyro_warmup_x: List[float] = []
        self._gyro_warmup_y: List[float] = []
        self._gyro_warmup_z: List[float] = []

        self._gyro_rot_z:    List[float] = []
        self._gyro_dt_list:  List[float] = []
        self._last_imu_time: Optional[float] = None

        self._mag_x: List[float] = []
        self._mag_y: List[float] = []
        self._mag_z: List[float] = []

        self._accumulated_yaw = 0.0
        self._total_rotation_rad = self._rot_count * 2.0 * math.pi

        self._imu_received    = False
        self._mag_available   = False
        self._done_triggered  = False

        # ── IMU subscriber ────────────────────────────────────────────────────
        # Subscribe using general reliability first to guarantee receipt
        self._imu_sub = self.create_subscription(
            Imu,
            self._imu_topic,
            self._imu_callback,
            10,
        )

        # ── Magnetometer subscriber (optional) ────────────────────────────────
        if _HAS_MAG_MSG:
            self._mag_sub = self.create_subscription(
                MagneticField,
                self._mag_topic,
                self._mag_callback,
                10,
            )
            self.get_logger().info(f"Listening for magnetometer on: {self._mag_topic}")
        else:
            self.get_logger().warn(
                "MagneticField message unavailable — skipping magnetometer calibration."
            )

        # ── cmd_vel publisher ─────────────────────────────────────────────────
        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)

        # ── FSM timer (10 Hz) ─────────────────────────────────────────────────
        self._fsm_timer = self.create_timer(0.1, self._fsm_tick)

        self.get_logger().info("=" * 60)
        self.get_logger().info("  IMU CALIBRATION NODE STARTED")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"  IMU topic      : {self._imu_topic}")
        self.get_logger().info(f"  cmd_vel topic  : {self._cmd_vel_topic}")
        self.get_logger().info(f"  Rotation speed : {self._rot_speed:.2f} rad/s")
        self.get_logger().info(f"  Rotations      : {self._rot_count:.1f} full circles")
        self.get_logger().info(f"  Warmup         : {self._warmup_dur:.1f}s (keep robot STILL!)")
        self.get_logger().info(f"  Output YAML    : {self._output_yaml}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("Waiting for IMU data on: " + self._imu_topic)

    # ── IMU callback ──────────────────────────────────────────────────────────
    def _imu_callback(self, msg: Imu) -> None:
        now = time.monotonic()
        self._last_imu_msg_time = now
        dt = (now - self._last_imu_time) if self._last_imu_time is not None else 0.0
        self._last_imu_time = now

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        if not self._imu_received:
            self._imu_received = True
            self.get_logger().info("IMU data received — entering WARMUP (keep robot still).")
            self._transition(CalibState.WARMUP)

        if self._state == CalibState.WARMUP:
            self._gyro_warmup_x.append(gx)
            self._gyro_warmup_y.append(gy)
            self._gyro_warmup_z.append(gz)

        elif self._state == CalibState.ROTATING:
            self._gyro_rot_z.append(gz)
            # Use a slightly more robust dt estimate or absolute wall time
            # dt should be positive
            effective_dt = dt if (0.0 < dt < 1.0) else 0.02  # fallback to 50Hz dt if jumpy
            self._gyro_dt_list.append(effective_dt)
            
            # Make sure we read gyro value correctly.
            # If the user's sensor axes are configured differently, Z axis rate (gz) might be zero.
            # Let's count absolute angular velocity if Z rotation is zero, but standard is Z-axis.
            self._accumulated_yaw += abs(gz) * effective_dt

    # ── Magnetometer callback ─────────────────────────────────────────────────
    def _mag_callback(self, msg) -> None:
        if self._state == CalibState.ROTATING:
            self._mag_available = True
            self._mag_x.append(msg.magnetic_field.x)
            self._mag_y.append(msg.magnetic_field.y)
            self._mag_z.append(msg.magnetic_field.z)

    # ── FSM tick ──────────────────────────────────────────────────────────────
    def _fsm_tick(self) -> None:
        now_time = time.monotonic()
        elapsed = now_time - self._state_start_time

        if self._state == CalibState.IDLE:
            pass  # waiting for first IMU message

        elif self._state == CalibState.WARMUP:
            self.get_logger().info(
                f"[WARMUP] Keep robot STILL... {max(0.0, self._warmup_dur - elapsed):.0f}s "
                f"remaining ({len(self._gyro_warmup_x)} samples)",
                throttle_duration_sec=1.0,
            )
            if elapsed >= self._warmup_dur:
                self.get_logger().info(
                    f"[WARMUP] Done — {len(self._gyro_warmup_x)} gyro samples."
                )
                self._transition(CalibState.ROTATING)

        elif self._state == CalibState.ROTATING:
            # ── Safety Watchdog 1: Lost IMU data ──────────────────────────────
            time_since_last_msg = now_time - self._last_imu_msg_time
            if time_since_last_msg > 2.0:
                self.get_logger().error(
                    f"SAFETY SHUTDOWN: Lost IMU messages for {time_since_last_msg:.1f}s! Stopping robot."
                )
                self._stop_robot()
                self._transition(CalibState.DONE)
                return

            # ── Safety Watchdog 2: Rotation Timeout (Max 45s) ─────────────────
            if elapsed > 45.0:
                self.get_logger().error(
                    "SAFETY SHUTDOWN: Calibration rotation timed out (exceeded 45s)! Stopping robot."
                )
                self._stop_robot()
                self._transition(CalibState.DONE)
                return

            # Publish rotation command
            twist = Twist()
            twist.angular.z = self._rot_speed
            self._cmd_vel_pub.publish(twist)

            pct = min(100.0, 100.0 * self._accumulated_yaw / self._total_rotation_rad)
            
            # Print Z angular rate diagnostic to help debug axis issues
            latest_gz = self._gyro_rot_z[-1] if self._gyro_rot_z else 0.0
            self.get_logger().info(
                f"[ROTATING] {pct:.1f}%  yaw={math.degrees(self._accumulated_yaw):.1f}°"
                f" / {math.degrees(self._total_rotation_rad):.1f}°"
                f" (latest gz={latest_gz:.3f} rad/s) | mag samples: {len(self._mag_x)}",
                throttle_duration_sec=2.0,
            )

            # If Z gyro rate is extremely small/zero during rotation, warn the user
            if elapsed > 5.0 and self._accumulated_yaw < 0.1:
                self.get_logger().warn(
                    "WARNING: Accumulated yaw is nearly zero! Is the IMU node publishing Z-axis rate correctly?",
                    throttle_duration_sec=5.0,
                )

            if self._accumulated_yaw >= self._total_rotation_rad:
                self.get_logger().info("[ROTATING] Target reached — stopping robot.")
                self._stop_robot()
                self._transition(CalibState.DONE)

        elif self._state == CalibState.DONE:
            if not self._done_triggered:
                self._done_triggered = True
                self._fsm_timer.cancel()
                self._compute_and_save_calibration()
                self.get_logger().info("Calibration complete. Shutting down node.")
                rclpy.shutdown()

    # ── State transition ──────────────────────────────────────────────────────
    def _transition(self, new_state: str) -> None:
        self.get_logger().info(f"[FSM] {self._state} -> {new_state}")
        self._state = new_state
        self._state_start_time = time.monotonic()

    # ── Stop robot ────────────────────────────────────────────────────────────
    def _stop_robot(self) -> None:
        self.get_logger().info("Stopping robot motors...")
        twist = Twist()
        for i in range(10): # Publish multiple times to ensure DDSM115 driver receives it
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.get_logger().info("Robot stopped.")

    # ── Compute + save calibration ────────────────────────────────────────────
    def _compute_and_save_calibration(self) -> None:
        self.get_logger().info("=" * 60)
        self.get_logger().info("  COMPUTING CALIBRATION OFFSETS")
        self.get_logger().info("=" * 60)

        result = {}

        # -- Gyro bias (stationary) --
        n_warmup = len(self._gyro_warmup_x)
        gyro_bias_x = gyro_bias_y = gyro_bias_z = 0.0
        if n_warmup > 0:
            gyro_bias_x = sum(self._gyro_warmup_x) / n_warmup
            gyro_bias_y = sum(self._gyro_warmup_y) / n_warmup
            gyro_bias_z = sum(self._gyro_warmup_z) / n_warmup
            self.get_logger().info(
                f"[GYRO BIAS] {n_warmup} samples\n"
                f"  X: {gyro_bias_x:+.6f} rad/s  ({math.degrees(gyro_bias_x):+.4f} deg/s)\n"
                f"  Y: {gyro_bias_y:+.6f} rad/s  ({math.degrees(gyro_bias_y):+.4f} deg/s)\n"
                f"  Z: {gyro_bias_z:+.6f} rad/s  ({math.degrees(gyro_bias_z):+.4f} deg/s)"
            )
            result["gyroscope"] = {
                "bias": {
                    "x": round(gyro_bias_x, 8),
                    "y": round(gyro_bias_y, 8),
                    "z": round(gyro_bias_z, 8),
                },
                "warmup_samples": n_warmup,
            }
        else:
            self.get_logger().warn("[GYRO BIAS] No warmup samples collected.")

        # -- Gyro scale factor (integration vs expected) --
        n_rot = len(self._gyro_rot_z)
        if n_rot > 10 and self._gyro_dt_list:
            mean_dt = sum(self._gyro_dt_list) / len(self._gyro_dt_list)
            corrected = [gz - gyro_bias_z for gz in self._gyro_rot_z]
            integrated_yaw = sum(abs(g) * mean_dt for g in corrected)
            expected_rad = self._total_rotation_rad

            if integrated_yaw > 0.01:
                gyro_scale_z = expected_rad / integrated_yaw
            else:
                gyro_scale_z = 1.0
                self.get_logger().warn("[GYRO SCALE] Integrated yaw too small — scale set to 1.0")

            self.get_logger().info(
                f"[GYRO SCALE] integrated={math.degrees(integrated_yaw):.1f}deg  "
                f"expected={math.degrees(expected_rad):.1f}deg  scale_z={gyro_scale_z:.6f}"
            )
            if "gyroscope" in result:
                result["gyroscope"]["scale_z"] = round(gyro_scale_z, 8)
                result["gyroscope"]["rotation_samples"] = n_rot
        else:
            self.get_logger().warn("[GYRO SCALE] Not enough rotation data.")

        # -- Magnetometer --
        n_mag = len(self._mag_x)
        if n_mag >= self._min_mag:
            self.get_logger().info(f"[MAG] {n_mag} samples — computing hard/soft iron.")

            fit_method = "midpoint"
            cx, cy = _simple_mag_center(self._mag_x, self._mag_y)
            scale_x = scale_y = 1.0

            if _HAS_NUMPY and n_mag >= 50:
                try:
                    cx, cy, a, b = _fit_ellipse_center_numpy(self._mag_x, self._mag_y)
                    if a > 1e-9 and b > 1e-9:
                        avg = (a + b) / 2.0
                        scale_x = avg / a
                        scale_y = avg / b
                        fit_method = "ellipse_fit"
                except Exception as exc:
                    self.get_logger().warn(f"Ellipse fit failed ({exc}) — using midpoint.")
                    cx, cy = _simple_mag_center(self._mag_x, self._mag_y)
                    scale_x = scale_y = 1.0

            cz = sum(self._mag_z) / n_mag
            self.get_logger().info(
                f"[MAG HARD IRON] cx={cx:.6f}  cy={cy:.6f}  cz={cz:.6f}  [{fit_method}]"
            )
            self.get_logger().info(
                f"[MAG SOFT IRON] scale_x={scale_x:.6f}  scale_y={scale_y:.6f}"
            )

            result["magnetometer"] = {
                "hard_iron_offset": {
                    "x": round(cx, 8),
                    "y": round(cy, 8),
                    "z": round(cz, 8),
                },
                "soft_iron_scale": {
                    "x": round(scale_x, 8),
                    "y": round(scale_y, 8),
                },
                "fit_method": fit_method,
                "samples": n_mag,
            }
        elif self._mag_available:
            self.get_logger().warn(
                f"[MAG] Only {n_mag}/{self._min_mag} samples — skipping. "
                "Try increasing rotation_count."
            )
        else:
            self.get_logger().warn(
                "[MAG] No /magnetic_field data received — "
                "ensure topic is published or disable via mag_topic parameter."
            )

        # -- Summary --
        self.get_logger().info("=" * 60)
        self.get_logger().info("  CALIBRATION SUMMARY")
        self.get_logger().info("=" * 60)
        if "gyroscope" in result:
            g = result["gyroscope"]
            b = g["bias"]
            scale_str = f"  scale_z={g.get('scale_z', 1.0):.6f}" if "scale_z" in g else ""
            self.get_logger().info(
                f"  Gyro Bias X={b['x']:+.6f}  Y={b['y']:+.6f}  Z={b['z']:+.6f} rad/s{scale_str}"
            )
        if "magnetometer" in result:
            m = result["magnetometer"]
            hi = m["hard_iron_offset"]
            si = m["soft_iron_scale"]
            self.get_logger().info(
                f"  Mag HardIron  X={hi['x']:+.6f}  Y={hi['y']:+.6f}  Z={hi['z']:+.6f}"
            )
            self.get_logger().info(
                f"  Mag SoftIron  X={si['x']:.6f}   Y={si['y']:.6f}"
            )

        self._save_yaml(result)

    def _save_yaml(self, result: dict) -> None:
        result["metadata"] = {
            "calibrated_at":       datetime.datetime.now().isoformat(),
            "rotation_speed_rads": self._rot_speed,
            "rotation_count":      self._rot_count,
            "warmup_duration_s":   self._warmup_dur,
        }

        out_path = Path(self._output_yaml)
        try:
            out_path.parent.mkdir(parents=True, exist_ok=True)
            if _HAS_YAML:
                with open(out_path, "w") as f:
                    yaml.dump(result, f, default_flow_style=False, sort_keys=False)
            else:
                with open(out_path, "w") as f:
                    f.write("# IMU Calibration — generated by imu_calibration_node\n")
                    f.write(_dict_to_yaml(result, indent=0))
            self.get_logger().info(f"Calibration saved -> {out_path}")
            self.get_logger().info(
                "Apply gyro bias by subtracting from raw angular_velocity readings.\n"
                "Apply hard-iron offset by subtracting from raw magnetic_field readings."
            )
        except OSError as exc:
            self.get_logger().error(f"Could not save YAML: {exc}")
            self.get_logger().info("--- Calibration data (copy manually) ---")
            self.get_logger().info(_dict_to_yaml(result))

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self._stop_robot()
        super().destroy_node()


# ── Entry point ────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = ImuCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted — stopping robot.")
        node._stop_robot()
    except SystemExit:
        pass
    finally:
        if rclpy.ok():
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()


if __name__ == "__main__":
    main()
