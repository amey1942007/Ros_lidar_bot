#!/usr/bin/env python3
"""
navigation_launch.py — ros_lidar_bot
Launches the Nav2 navigation stack for click-to-drive and autonomous navigation.

Usage (standalone):
  ros2 launch ros_lidar_bot navigation_launch.py

Typically included by launch_sim.launch.py automatically.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros_lidar_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ── Nav2 Bringup ─────────────────────────────────────────────────────────
    # Launches: bt_navigator, planner_server, controller_server,
    #           behavior_server, recoveries_server, waypoint_follower,
    #           local_costmap, global_costmap, lifecycle_manager
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            # We are using SLAM Toolbox for mapping, not a pre-built map.
            # Set use_composition to False to avoid lifecycle conflicts.
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        nav2_bringup,
    ])
