#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'Ros_lidar_bot'
    pkg_share = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='World to load in Gazebo'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup Nav2 stack'
    )

    explore_delay_arg = DeclareLaunchArgument(
        'explore_delay',
        default_value='8.0',
        description='Delay in seconds before starting explore_lite'
    )

    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    explore_params_file = os.path.join(pkg_share, 'config', 'explore_lite.yaml')

    sim_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam': 'false'
        }.items()
    )

    nav2_with_slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'slam': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': params_file,
            'use_composition': 'False'
        }.items()
    )

    explore = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[
            explore_params_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    delayed_explore = TimerAction(
        period=LaunchConfiguration('explore_delay'),
        actions=[explore]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        autostart_arg,
        explore_delay_arg,
        sim_only,
        nav2_with_slam,
        delayed_explore,
    ])
