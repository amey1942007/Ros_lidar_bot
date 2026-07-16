#!/usr/bin/env python3
"""
autonomous_robot.launch.py – Hardware autonomous exploration launch.

Starts the hardware drivers, state estimation, SLAM, and Nav2 (via
launch_robot.launch.py), then launches the frontier explorer node.

Usage:
  ros2 launch Ros_lidar_bot autonomous_robot.launch.py
  ros2 launch Ros_lidar_bot autonomous_robot.launch.py verbose:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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
    out = "screen" if verbose else "log"
    log_args = [] if verbose else ["--ros-args", "--log-level", "warn"]

    hardware_and_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "launch_robot.launch.py")
        ),
        launch_arguments={
            "verbose": LaunchConfiguration("verbose"),
            "expect_frontier": "true",
        }.items(),
    )

    frontier_explorer = Node(
        package=package_name,
        executable="frontier_explorer_node",
        name="frontier_explorer",
        output=out,
        arguments=log_args,
        parameters=[os.path.join(pkg_share, "config", "frontier_explorer.yaml")],
    )

    return [
        hardware_and_mapping,
        # Nav2 begins at T=8s in launch_robot.launch.py. Give its action
        # servers 14 seconds to initialize before frontier exploration starts.
        TimerAction(period=22.0, actions=[frontier_explorer]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="If true, all node logs go to the terminal (old spammy mode).",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
