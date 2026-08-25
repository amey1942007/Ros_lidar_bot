#!/usr/bin/env python3
"""
amr4_teleop.launch.py — alias for amr4.launch.py

Old teleop launch had driver+joy only (no odom_node / EKF → no /odom).
This file now forwards to the full odom+teleop stack.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("Ros_lidar_bot")
    return LaunchDescription([
        DeclareLaunchArgument("verbose", default_value="true"),
        DeclareLaunchArgument("use_joy", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "amr4.launch.py")
            ),
            launch_arguments={
                "verbose": LaunchConfiguration("verbose"),
                "use_joy": LaunchConfiguration("use_joy"),
            }.items(),
        ),
    ])
