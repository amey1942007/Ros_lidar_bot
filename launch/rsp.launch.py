#!/usr/bin/env python3

#author : Honey_IITD
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    output_mode = LaunchConfiguration('output')

    pkg_path = os.path.join(get_package_share_directory('Ros_lidar_bot'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output=output_mode,
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output=output_mode,
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/encoder']
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'output',
            default_value='log',
            description='screen or log — use log so bringup_status owns the terminal'),

        node_robot_state_publisher,
        node_joint_state_publisher
    ])
