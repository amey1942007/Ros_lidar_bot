#!/usr/bin/env python3
"""
autonomous_robot.launch.py – Hardware autonomous exploration launch.

Starts the hardware drivers, state estimation, SLAM, and Nav2 (via
launch_robot.launch.py), then launches the frontier explorer node.

Usage:
  ros2 launch Ros_lidar_bot autonomous_robot.launch.py
"""

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

    # ── 1. Base Hardware + SLAM + EKF ─────────────────────────────────────────
    # This brings up sensors, motor drivers, odometry, safety stop, and SLAM
    hardware_and_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "launch_robot.launch.py")
        )
    )

    # ── 2. Frontier Explorer ──────────────────────────────────────────────────
    # Starts after Nav2 is fully operational to command autonomous exploration.
    frontier_explorer = Node(
        package=package_name,
        executable="frontier_explorer_node",
        name="frontier_explorer",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "frontier_explorer.yaml")],
    )

    return LaunchDescription([
        hardware_and_mapping,
        
        # Nav2 begins at T=8s in launch_robot.launch.py. Give its action
        # servers 14 seconds to initialize before frontier exploration starts.
        TimerAction(period=22.0, actions=[frontier_explorer]),
    ])
