#!/usr/bin/env python3
"""
launch_odom_test.launch.py — Minimal bringup for wheel/IMU odometry accuracy
=============================================================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble
Purpose  : Test /odom (EKF) with drive_distance — NO lidar, SLAM, or Nav2.

Starts only:
  rsp              → TF (base / imu / laser frames from URDF)
  amr4_driver      → /cmd_vel_safe → Mega; publishes /encoder + /imu
  odom_node        → /encoder → /odom_raw
  ekf_node         → /odom_raw + /imu → /odom (+ odom→base_footprint TF)
  safety_stop      → /cmd_vel → /cmd_vel_safe (odom-stale guard; no laser without lidar)
  joy + joy_teleop → optional manual positioning

NOT started: lidar_node, slam_toolbox, Nav2, dashboard, frontier.

Usage
-----
  # Terminal 1 — odom stack
  ros2 launch Ros_lidar_bot launch_odom_test.launch.py
  # or with logs:
  ros2 launch Ros_lidar_bot launch_odom_test.launch.py verbose:=true

  # Terminal 2 — verify chain is live
  ros2 topic hz /encoder /imu /odom_raw /odom

  # Terminal 3 — distance accuracy test (uses EKF /odom)
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
    OpaqueFunction,
    SetEnvironmentVariable,
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
    use_joy = LaunchConfiguration("use_joy").perform(context).lower() in (
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
            "cmd_vel_topic":    "/cmd_vel_safe",
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
            "chassis_w":      0.88,
            "encoder_topic":  "/encoder",
            "odom_topic":     "/odom_raw",
            "base_frame_id":  "base_footprint",
            "odom_frame_id":  "odom",
            "broadcast_tf":   False,
            "pose_cov_x":      0.01,
            "pose_cov_y":      0.01,
            "pose_cov_yaw":    0.01,
            "twist_cov_vx":    0.005,
            "twist_cov_vy":    0.01,
            "twist_cov_omega": 0.01,
        }],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        # Always screen for odom bringup — silent EKF is hard to debug.
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {
                "use_sim_time": False,
                # Reinforce critical inputs in case YAML node-key missed.
                "odom0": "/odom_raw",
                "imu0": "/imu",
                "publish_tf": True,
                "print_diagnostics": True,
            },
        ],
        # Prefer unscoped names — more reliable remaps on Humble.
        remappings=[("odometry/filtered", "odom")],
    )

    # Keeps joy → /cmd_vel → /cmd_vel_safe. Without lidar, laser gate never
    # blocks; odom_raw-stale watchdog still protects if encoders drop.
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
            "ignore_below":          0.15,
            "front_opening_deg":     50.0,
            "rear_opening_deg":      50.0,
            "clear_margin":          0.10,
            "odom_raw_timeout_sec":  0.5,
        }],
    )

    actions.extend([rsp, driver_node, odom_node, ekf_node, safety_stop])

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
                "axis_angular": 3,
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
            "use_joy",
            default_value="true",
            description="Start joy_node + joy_teleop for manual positioning.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
