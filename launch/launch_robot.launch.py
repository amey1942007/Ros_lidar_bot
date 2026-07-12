#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)

    # ── Robot State Publisher ─────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    # ── Scan-based hardware safety stop ──────────────────────────────────────
    safety_stop = Node(
        package=package_name,
        executable="safety_stop_node.py",
        name="safety_stop",
        output="screen",
        parameters=[{
            "min_safe_distance": 0.25,
            "front_opening_deg": 180.0,
            "rear_opening_deg": 60.0,
        }],
    )

    # ── EKF: fuses wheel odometry + IMU → clean /odom ────────────────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("/odometry/filtered", "/odom")],
    )

    # ── SLAM Toolbox: builds map + publishes map→odom TF ─────────────────────
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

    # ── Nav2 Navigation Stack ─────────────────────────────────────────────────
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": os.path.join(pkg_share, "config", "nav2_params.yaml"),
        }.items(),
    )

    return LaunchDescription([
        # Stage 1: Robot state publisher & Safety stop
        rsp,
        safety_stop,
        
        # Stage 2: EKF (Fuses hardware odometry/IMU)
        ekf_node,
        
        # Stage 3: SLAM and Nav2 (Delayed to allow EKF to stabilize)
        TimerAction(period=3.0, actions=[slam_toolbox]),
        TimerAction(period=6.0, actions=[nav2]),
    ])
