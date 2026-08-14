#!/usr/bin/env python3
"""
launch_robot.launch.py — AMR4 full bringup launch file
========================================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Robot    : AMR4 mecanum 4WD with RPLidar A1

Node pipeline summary
---------------------
  joy_node       → /joy
  joy_teleop     → /cmd_vel
  safety_stop    : /cmd_vel → /cmd_vel_safe  (laser-based obstacle gate)
  amr4_driver    : /cmd_vel_safe → Arduino Mega (HDRIVE/DRIVE commands)
                   Arduino Mega telemetry → /odom_raw  (4-wheel RPMs)
  odom_node      : /odom_raw → /odom  (mecanum FK dead-reckoning odometry)
  imu_node       : /dev/ttyACM1 (BNO055 JSON) → /imu
  ekf_node       : /odom + /imu → /odometry/filtered  (remapped to /odom by EKF config)
  lidar_node     : /dev/ttyUSB0 (RPLidar A1 sensitivity mode) → /scan
  slam_toolbox   : /scan + /odom → map
  nav2           : map → /cmd_vel

Serial port assignments (typical — adjust if your system differs):
  /dev/ttyACM0 → Arduino Mega (DriveMaster.ino) — amr4_driver_node
  /dev/ttyACM1 → Arduino Mega (BNO055 IMU firmware) — imu_node
  /dev/ttyUSB0 → RPLidar A1 (USB-serial adapter) — lidar_node

Launch arguments
----------------
  verbose (bool, default false) — set true to see all node logs in terminal
  expect_frontier (bool, default false) — set true to wait for frontier_explorer
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)

    verbose = LaunchConfiguration("verbose").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    expect_frontier = LaunchConfiguration("expect_frontier").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    out = "screen" if verbose else "log"
    log_args = [] if verbose else ["--ros-args", "--log-level", "fatal"]
    nav2_log_level = "info" if verbose else "fatal"

    actions = []
    if not verbose:
        actions.append(SetEnvironmentVariable("RCUTILS_LOGGING_MIN_SEVERITY", "FATAL"))
        actions.append(SetEnvironmentVariable(
            "RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"
        ))

    # ── Web dashboard ─────────────────────────────────────────────────────────
    dashboard = Node(
        package=package_name,
        executable="robot_dashboard",
        name="robot_dashboard",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "port": 8080,
            "expect_frontier": expect_frontier,
        }],
    )

    # ── 1. Robot State Publisher ──────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "output": out,
        }.items(),
    )

    # ── 2. IMU Node (BNO055 via dedicated Arduino Mega UART) ──────────────────
    # This node opens /dev/ttyACM1 and reads BNO055 JSON lines from a separate
    # Arduino that runs the BNO055 firmware (NOT DriveMaster.ino).
    # It publishes /imu (sensor_msgs/Imu) for the EKF and HDRIVE heading latch.
    imu_node = Node(
        package=package_name,
        executable="imu_node",
        name="imu_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "serial_port":   "/dev/ttyACM1",
            "baud_rate":     500000,
            "output_topic":  "/imu",
            "frame_id":      "imu_link",
            "publish_rate":  50.0,
            "timeout":       0.1,
        }],
    )

    # ── 3. AMR4 Motor Driver Node (DriveMaster.ino via UART) ──────────────────
    # Receives /cmd_vel_safe, sends HDRIVE or DRIVE to the Arduino.
    # Reads Arduino telemetry and publishes /encoder (4-wheel RPMs).
    # /encoder → odom_node → /odom_raw → EKF → /odom
    driver_node = Node(
        package=package_name,
        executable="amr4_driver",
        name="amr4_driver_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "serial_port":      "/dev/ttyACM0",
            "baud_rate":        115200,
            "cmd_vel_topic":    "/cmd_vel_safe",
            "cmd_timeout":      0.5,
            "max_send_rate":    5.0,
            "omega_threshold":  0.05,
            "frame_id":         "base_footprint",
            "flush_rate":       1.0,
            # Raw wheel RPMs published here for odom_node
            "encoder_topic":    "/encoder",
        }],
    )

    # ── 4. Odometry Node ──────────────────────────────────────────────────────
    # Subscribes to /encoder (Float32MultiArray — 4 raw wheel RPMs from driver).
    # Runs mecanum forward kinematics to compute body velocity (vx, vy, omega).
    # Integrates pose (x, y, yaw) and publishes nav_msgs/Odometry on /odom_raw.
    # EKF then fuses /odom_raw + /imu and publishes final /odom.
    # broadcast_tf=False: EKF is the sole odom→base_footprint TF publisher.
    odom_node = Node(
        package=package_name,
        executable="odom_node",
        name="odom_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            # Chassis geometry — must match Config.h values
            "wheel_radius":   0.05,    # metres (WHEEL_RADIUS in Config.h)
            "chassis_l":      0.52,    # metres (CHASSIS_L  in Config.h)
            "chassis_w":      0.88,    # metres (CHASSIS_W  in Config.h)
            # Topics
            "encoder_topic":  "/encoder",    # raw RPMs from driver node
            "odom_topic":     "/odom_raw",   # FK odometry → EKF input
            "base_frame_id": "base_footprint",
            "odom_frame_id": "odom",
            # EKF publishes odom→base_footprint; odom_node must NOT also do it
            "broadcast_tf":   False,
            # Covariance — tune per robot (higher = trust EKF IMU fusion more)
            "pose_cov_x":      0.01,
            "pose_cov_y":      0.01,
            "pose_cov_yaw":    0.01,
            "twist_cov_vx":    0.005,   # tight — encoder FK vX is accurate
            "twist_cov_vy":    0.01,    # slightly looser — strafe FK has more slip
            "twist_cov_omega": 0.01,
        }],
    )

    # ── 5. LiDAR Node (RPLidar A1 via USB-serial, sensitivity mode) ──────────
    # Uses pyrplidar library with Express/Sensitivity mode (mode 1) for higher
    # point density than standard mode.
    # Requires: pip3 install pyrplidar --break-system-packages
    # Port: /dev/ttyUSB0 (USB-serial adapter from RPLidar A1 module)
    # Publishes /scan (sensor_msgs/LaserScan) in ROS CCW convention.
    lidar_node = Node(
        package=package_name,
        executable="lidar_node",
        name="lidar_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "serial_port":      "/dev/ttyUSB0",
            "serial_baud":      115200,
            "scan_topic":       "/scan",
            "frame_id":         "laser_frame",
            # RPLidar A1 reliable range: 0.15 m to 12.0 m
            "min_range":        0.15,
            "max_range":        12.0,
            # Motor PWM: 660 is nominal for A1 (600-700 range)
            "motor_pwm":        660,
            # True = Express/Sensitivity mode (higher density, recommended)
            # False = Standard mode (fallback)
            "sensitivity_mode": True,
            # 0.0 = publish every complete sweep (no throttle)
            "publish_rate":     0.0,
            "num_bins":         360,
        }],
    )

    # ── 6. Safety Stop Node ───────────────────────────────────────────────────
    # Scan-based velocity filter: /cmd_vel → /cmd_vel_safe
    # Stops the robot if obstacles are detected within min_safe_distance.
    safety_stop = Node(
        package=package_name,
        executable="safety_stop_node",
        name="safety_stop",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "min_safe_distance":     0.35,
            "ignore_below":          0.15,   # matches lidar min_range
            "front_opening_deg":     50.0,
            "rear_opening_deg":      50.0,
            "clear_margin":          0.10,
            "odom_raw_timeout_sec":  0.5,
        }],
    )

    # ── 6b. Gamepad teleop ────────────────────────────────────────────────────
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "device_id":       0,
            "deadzone":        0.05,
            "autorepeat_rate": 20.0,
        }],
    )

    joy_teleop = Node(
        package=package_name,
        executable="joy_teleop",
        name="joy_teleop",
        output=out,
        arguments=log_args,
    )

    # ── 7. EKF Node (fuses /odom_raw + /imu → /odom) ─────────────────────────
    # Fuses:
    #   /odom_raw — mecanum FK odometry (vX, vY, vYaw) from odom_node
    #   /imu      — BNO055 gyro (vYaw) from imu_node
    # Outputs /odometry/filtered remapped to /odom.
    # Is the SOLE publisher of odom→base_footprint TF (broadcast_tf=False in odom_node).
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output=out,
        arguments=log_args,
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    # ── 8. SLAM Toolbox ───────────────────────────────────────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": os.path.join(
                pkg_share, "config", "mapper_params_online_async.yaml"
            ),
            "use_sim_time": "false",
        }.items(),
    )

    # ── 9. Nav2 Navigation Stack ──────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time":  "false",
            "params_file":   os.path.join(pkg_share, "config", "nav2_params.yaml"),
            "log_level":     nav2_log_level,
        }.items(),
    )

    actions.extend([
        dashboard,

        # ── Stage 1 (T=0s): Hardware drivers + localization ───────────────────
        rsp,
        imu_node,        # /dev/ttyACM1 → /imu
        driver_node,     # /dev/ttyACM0 → /odom_raw, sends HDRIVE/DRIVE
        odom_node,       # /odom_raw → /odom (mecanum FK)
        lidar_node,      # /dev/ttyUSB0 → /scan (RPLidar A1 sensitivity mode)
        safety_stop,     # /cmd_vel → /cmd_vel_safe
        joy_node,
        joy_teleop,
        ekf_node,        # /odom + /imu → odom TF + fused pose

        # ── Stage 2 (T=5s): SLAM — needs /scan + odom TF ─────────────────────
        TimerAction(period=5.0, actions=[slam_toolbox]),

        # ── Stage 3 (T=8s): Nav2 — needs SLAM map + map→odom TF ──────────────
        TimerAction(period=8.0, actions=[nav2]),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="If true, all node logs go to the terminal.",
        ),
        DeclareLaunchArgument(
            "expect_frontier",
            default_value="false",
            description="If true, dashboard waits for frontier_explorer (autonomous).",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
