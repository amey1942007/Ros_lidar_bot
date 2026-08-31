#!/usr/bin/env python3
"""
launch_odom_test.launch.py — Minimal bringup for wheel/IMU odometry accuracy
=============================================================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Purpose  : Test /odom (EKF) with drive_distance — NO lidar, SLAM, or Nav2.

Starts only:
  rsp              → TF (base / imu / laser frames from URDF)
  amr4_driver      → /cmd_vel → Mega; publishes /encoder + /imu
  odom_node        → /encoder (counts + Arduino ms) → /odom_raw
  ekf_node         → /odom_raw + /imu → /odom (+ odom→base_footprint TF)
  joy + joy_teleop → optional manual positioning (left=vx/vy, right=rotate)

NOT started: lidar_node, slam_toolbox, Nav2, dashboard, frontier.

Usage
-----
  # Terminal 1 — odom stack
  ros2 launch Ros_lidar_bot launch_odom_test.launch.py
  # or with logs:
  ros2 launch Ros_lidar_bot launch_odom_test.launch.py verbose:=true

  # Terminal 2 — verify chain is live
  ros2 topic hz /encoder /imu /odom_raw /odom

  # Terminal 3 — distance accuracy test
  # Prefers the fused EKF /odom; falls back to /odom_raw if the EKF is down.
  ros2 run Ros_lidar_bot drive_distance

Mark a tape line on the floor, command e.g. X=1.0 Y=0.0, measure real travel
vs the node's reported error.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Critical EKF params inlined so /odom still works if ekf.yaml is stale on disk.
_EKF_INLINE_PARAMS = {
    "use_sim_time": False,
    "odom0": "/odom_raw",
    "imu0": "/imu",
    "publish_tf": True,
    "print_diagnostics": True,
    "frequency": 30.0,
    "two_d_mode": True,
    "world_frame": "odom",
    "odom_frame": "odom",
    "base_link_frame": "base_footprint",
    "odom0_config": [
        False, False, False, False, False, False,
        True, True, False, False, False, True,
        False, False, False,
    ],
    "imu0_config": [
        False, False, False, False, False, False,
        False, False, False, False, False, True,
        False, False, False,
    ],
}


def _launch_setup(context, *args, **kwargs):
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)

    verbose = LaunchConfiguration("verbose").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_joy = LaunchConfiguration("use_joy").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    use_ekf = LaunchConfiguration("use_ekf").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    out = "screen" if verbose else "log"
    log_args = [] if verbose else ["--ros-args", "--log-level", "fatal"]

    actions = []
    if not verbose:
        actions.append(SetEnvironmentVariable("RCUTILS_LOGGING_MIN_SEVERITY", "FATAL"))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "output": out,
        }.items(),
    )

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
            "cmd_vel_topic":    "/cmd_vel",
            "cmd_timeout":      0.5,
            "max_send_rate":    15.0,
            "omega_threshold":  0.05,
            "frame_id":         "base_footprint",
            # base_footprint avoids EKF waiting on imu_link TF for gyro-only fusion.
            "imu_frame_id":     "base_footprint",
            "flush_rate":       0.0,
            "encoder_topic":    "/encoder",
        }],
    )

    odom_node = Node(
        package=package_name,
        executable="odom_node",
        name="odom_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "wheel_radius":   0.05,
            "chassis_l":      0.52,
            "chassis_w":      0.63,
            "ppr1":           1300,
            "ppr2":            680,
            "ppr3":            400,
            "ppr4":            280,
            "encoder_topic":  "/encoder",
            "odom_topic":     "/odom_raw" if use_ekf else "/odom",
            "base_frame_id":  "base_footprint",
            "odom_frame_id":  "odom",
            "broadcast_tf":   not use_ekf,
            "pose_cov_x":      0.01,
            "pose_cov_y":      0.01,
            "pose_cov_yaw":    0.01,
            "twist_cov_vx":    0.005,
            "twist_cov_vy":    0.01,
            "twist_cov_omega": 0.01,
        }],
    )

    actions.extend([rsp, driver_node, odom_node])

    if use_ekf:
        ekf_node = Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=[_EKF_INLINE_PARAMS],
            remappings=[("/odometry/filtered", "/odom")],
        )
        actions.append(ekf_node)
        actions.append(LogInfo(
            msg="EKF enabled — /odom_raw + /imu → /odom. "
                "If /odom has no publisher: sudo apt install ros-humble-robot-localization"
        ))
    else:
        actions.append(LogInfo(
            msg="EKF disabled — odom_node publishes /odom + odom→base_footprint TF directly."
        ))

    if use_joy:
        actions.append(Node(
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
        ))
        actions.append(Node(
            package=package_name,
            executable="joy_teleop",
            name="joy_teleop",
            output=out,
            arguments=log_args,
            parameters=[{
                # Mecanum: left stick vx/vy, right stick rotate (BT Xbox RX=axis 2).
                "axis_linear": 1,
                "axis_strafe": 0,
                "axis_angular": 2,
                "axis_cam_tilt": -1,
                "ang_deadzone": 0.30,
                "stick_exclusive": True,
            }],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "verbose",
            default_value="true",
            description="Show node logs (default true for odom debugging).",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="true",
            description="Fuse /odom_raw + /imu with robot_localization EKF → /odom. "
                        "Set false if EKF is missing or /odom has no publisher.",
        ),
        DeclareLaunchArgument(
            "use_joy",
            default_value="true",
            description="Start joy_node + joy_teleop for manual positioning.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
