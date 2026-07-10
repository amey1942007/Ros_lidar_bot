#!/usr/bin/env python3
"""
autonomous_robot.launch.py – Hardware autonomous exploration launch.

Starts the hardware drivers, state estimation, SLAM (via launch_robot.launch.py),
brings up the Nav2 navigation stack configured for real hardware, and finally
launches the frontier explorer node.

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

    # ── 2. Nav2 Navigation Stack ──────────────────────────────────────────────
    # Needs to be started with use_sim_time=false for the physical robot.
    # Delayed slightly to allow SLAM to broadcast the initial /map
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

    # ── 3. Frontier Explorer ──────────────────────────────────────────────────
    # Starts after Nav2 is fully operational to command autonomous exploration.
    frontier_explorer = Node(
        package=package_name,
        executable="frontier_explorer_node.py",
        name="frontier_explorer",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "frontier_explorer.yaml")],
    )

    return LaunchDescription([
        hardware_and_mapping,
        
        # T=8s: Start Nav2 after SLAM has had time to build the initial map
        TimerAction(period=8.0, actions=[nav2]),
        
        # T=22s: Start the explorer 14s after Nav2 so the action servers are up
        TimerAction(period=10.0, actions=[frontier_explorer]),
    ])
