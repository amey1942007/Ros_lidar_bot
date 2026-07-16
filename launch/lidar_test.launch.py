#!/usr/bin/env python3
"""
lidar_test.launch.py  –  Standalone LiDAR test for RPLidar S2E (Ethernet/UDP).

Uses Slamtec sllidar_ros2 (NOT the apt rplidar_ros with SDK 1.12, which is
serial-only and will try /dev/ttyUSB0 even when channel_type:=udp is set).

Install once:
    cd ~/Desktop/ros2_ws/src
    git clone https://github.com/Slamtec/sllidar_ros2.git
    cd ~/Desktop/ros2_ws
    colcon build --symlink-install --packages-select sllidar_ros2 Ros_lidar_bot
    source install/setup.bash

Usage (RPi eth0 must be 192.168.11.1/24; lidar at 192.168.11.2):
    ping -c 4 192.168.11.2
    ros2 launch Ros_lidar_bot lidar_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def _launch_setup(context, *args, **kwargs):
    package_name = 'Ros_lidar_bot'
    pkg_share = get_package_share_directory(package_name)

    channel_type = LaunchConfiguration('channel_type').perform(context)
    udp_ip = LaunchConfiguration('udp_ip').perform(context)
    udp_port = int(LaunchConfiguration('udp_port').perform(context))
    frame_id = LaunchConfiguration('frame_id').perform(context)
    inverted = LaunchConfiguration('inverted').perform(context).lower() in (
        '1', 'true', 'yes')
    angle_compensate = LaunchConfiguration('angle_compensate').perform(
        context).lower() in ('1', 'true', 'yes')
    scan_mode = LaunchConfiguration('scan_mode').perform(context)

    xacro_file = os.path.join(pkg_share, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': False,
        }]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                    '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                    '--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base_footprint',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                    '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                    '--frame-id', 'odom', '--child-frame-id', 'base_footprint'],
        output='screen',
    )

    # Official Slamtec S2E driver (UDP). Keep node name rplidar_node so
    # bringup_status / other tooling still find it.
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': channel_type,
            'udp_ip': udp_ip,
            'udp_port': udp_port,
            'frame_id': frame_id,
            'inverted': inverted,
            'angle_compensate': angle_compensate,
            'scan_mode': scan_mode,
        }],
    )

    return [
        rsp_node,
        jsp_node,
        static_tf_map_odom,
        static_tf_odom_base,
        lidar_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'channel_type', default_value='udp',
            description='Must be "udp" for S2E Ethernet'),
        DeclareLaunchArgument(
            'udp_ip', default_value='192.168.11.2',
            description='S2E factory IP'),
        DeclareLaunchArgument(
            'udp_port', default_value='8089',
            description='S2E factory UDP port'),
        DeclareLaunchArgument(
            'frame_id', default_value='laser_frame',
            description='Must match URDF laser link'),
        DeclareLaunchArgument(
            'inverted', default_value='false',
            description='Invert scan data'),
        DeclareLaunchArgument(
            'angle_compensate', default_value='true',
            description='Enable angle compensation'),
        DeclareLaunchArgument(
            'scan_mode', default_value='DenseBoost',
            description='DenseBoost = full rate; try Standard or Sensitivity if needed'),
        OpaqueFunction(function=_launch_setup),
    ])
