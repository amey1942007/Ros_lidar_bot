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

    # Check if we have to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('Ros_lidar_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Create a joint_state_publisher node that subscribes to the real /encoder topic.
    # ⚠ driver_node publishes /encoder with BEST_EFFORT (SENSOR_DATA QoS preset).
    #   joint_state_publisher defaults to RELIABLE → QoS mismatch → no data received.
    #   Fix: override /encoder subscription QoS to BEST_EFFORT to match the publisher.
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/encoder']
        }],
        remappings=[],
        arguments=['--ros-args',
                   '--qos-profile', 'sensor_data',
                   '-r', '__node:=joint_state_publisher'],
    )

    # Launch
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher,
        node_joint_state_publisher
    ])
