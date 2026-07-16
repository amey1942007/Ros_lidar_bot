#!/usr/bin/env python3
"""
lidar_test.launch.py  –  Standalone LiDAR test launch for RPLidar S2E (Ethernet/UDP) using official Slamtec rplidar_ros package.

Launches only:
  1. robot_state_publisher  (provides laser_frame TF from URDF)
  2. Static TF: map → odom (identity, for RViz2 fixed-frame compatibility)
  3. Static TF: odom → base_footprint (identity, robot parked at origin)
  4. rplidar_ros rplidar_composition (UDP to 192.168.11.2:8089, publishes /scan)

Usage (RPi eth0 must have a static IP on the lidar subnet, e.g. 192.168.11.1/24):
    # Build first:
    cd ~/Desktop/ros2_ws
    colcon build --symlink-install --packages-select Ros_lidar_bot
    source install/setup.bash

    # Launch:
    ros2 launch Ros_lidar_bot lidar_test.launch.py

    # Override lidar IP / scan mode if needed:
    ros2 launch Ros_lidar_bot lidar_test.launch.py udp_ip:=192.168.11.2 scan_mode:=Standard

    # In RViz2:
    #   Fixed Frame: map   (or laser_frame if you want minimal setup)
    #   Add → LaserScan → Topic: /scan
    #   Add → TF

TF tree published by this launch:
    map → odom → base_footprint → base_link → laser_frame
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    package_name = 'Ros_lidar_bot'
    pkg_share    = get_package_share_directory(package_name)

    # ── Launch Arguments (defaults = RPLidar S2E over Ethernet/UDP) ────────
    channel_type_arg = DeclareLaunchArgument(
        'channel_type', default_value='udp',
        description='Lidar channel: "udp" for S2E Ethernet, "serial" for USB lidars')

    udp_ip_arg = DeclareLaunchArgument(
        'udp_ip', default_value='192.168.11.2',
        description='S2E IP address (factory default 192.168.11.2; RPi eth0 '
                    'must be static on the same subnet, e.g. 192.168.11.1/24)')

    udp_port_arg = DeclareLaunchArgument(
        'udp_port', default_value='8089',
        description='S2E UDP port (factory default 8089)')

    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='laser_frame',
        description='Specifying frame_id of lidar (must match URDF link name)')

    inverted_arg = DeclareLaunchArgument(
        'inverted', default_value='false',
        description='Specifying whether or not to invert scan data')

    angle_compensate_arg = DeclareLaunchArgument(
        'angle_compensate', default_value='true',
        description='Specifying whether or not to enable angle_compensate of scan data')

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode', default_value='DenseBoost',
        description='Scan mode. "DenseBoost" = S2E full 32 kHz sample rate. '
                    'Use "Standard" if the driver logs "Failed to set scan mode".')

    # ── Robot State Publisher (provides laser_frame TF from URDF) ─────────
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

    # Joint state publisher (needed for continuous wheel joints in URDF)
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # ── Static TFs: map → odom → base_footprint ───────────────────────────
    # These place the robot at the world origin so RViz2 can display the scan
    # without needing the full EKF/SLAM stack.
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    static_tf_odom_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
    )

    # ── LiDAR Node (official Slamtec rplidar_ros package) ─────────────────
    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_node',
        output='screen',
        parameters=[{
            'channel_type': LaunchConfiguration('channel_type'),
            'udp_ip': LaunchConfiguration('udp_ip'),
            'udp_port': LaunchConfiguration('udp_port'),
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': LaunchConfiguration('inverted'),
            'angle_compensate': LaunchConfiguration('angle_compensate'),
            'scan_mode': LaunchConfiguration('scan_mode'),
        }],
    )

    return LaunchDescription([
        channel_type_arg,
        udp_ip_arg,
        udp_port_arg,
        frame_id_arg,
        inverted_arg,
        angle_compensate_arg,
        scan_mode_arg,
        rsp_node,
        jsp_node,
        static_tf_map_odom,
        static_tf_odom_base,
        lidar_node,
    ])
