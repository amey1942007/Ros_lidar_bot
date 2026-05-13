#!/usr/bin/env python3
"""
Autonomous exploration launch.

Starts the full simulation + navigation stack (via launch_sim.launch.py) and
then, once Nav2 is operational, launches the frontier explorer node.

Usage:
  ros2 launch Ros_lidar_bot autonomous_exploration.launch.py
  ros2 launch Ros_lidar_bot autonomous_exploration.launch.py world:=warehouse.world
  ros2 launch Ros_lidar_bot autonomous_exploration.launch.py world:=office.world
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)
    world = LaunchConfiguration("world")

    # Full simulation + navigation (Gazebo + EKF + SLAM + Nav2)
    sim_and_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "launch_sim.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    # Frontier explorer — starts after Nav2 is fully operational.
    # Timeline: T=10s Nav2 starts, T=25s explorer starts (15s for Nav2 to warm up).
    frontier_explorer = Node(
        package=package_name,
        executable="frontier_explorer_node.py",
        name="frontier_explorer",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "frontier_explorer.yaml")],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="testing.world",
            description="World filename inside worlds/. Options: testing.world  warehouse.world  office.world",
        ),
        sim_and_nav,
        # T=18s Nav2 starts (from launch_sim), T=32s explorer starts (14s for Nav2 to warm up).
        TimerAction(period=32.0, actions=[frontier_explorer]),
    ])
