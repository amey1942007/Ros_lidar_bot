#!/usr/bin/env python3
"""
amr4.launch.py — AMR4 teleop + odometry bringup (no lidar / SLAM / Nav2)
========================================================================
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble

Starts:
  rsp, amr4_driver, odom_node, ekf → /odom, safety_stop, joy + joy_teleop

This replaces the old amr4_teleop.launch.py which only had driver+joy and
never started odom_node / EKF (so /odom was missing).

Usage
-----
  ros2 launch Ros_lidar_bot amr4.launch.py
  ros2 launch Ros_lidar_bot amr4.launch.py verbose:=true
  ros2 launch Ros_lidar_bot amr4_teleop.launch.py   # same stack (alias)

Then:
  ros2 topic hz /encoder /imu /odom_raw /odom
  ros2 run Ros_lidar_bot drive_distance
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("Ros_lidar_bot")
    return LaunchDescription([
        DeclareLaunchArgument("verbose", default_value="true"),
        DeclareLaunchArgument("use_joy", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, "launch", "launch_odom_test.launch.py")
            ),
            launch_arguments={
                "verbose": LaunchConfiguration("verbose"),
                "use_joy": LaunchConfiguration("use_joy"),
            }.items(),
        ),
    ])
