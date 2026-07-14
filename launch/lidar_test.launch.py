#!/usr/bin/env python3
"""
lidar_test.launch.py  –  Standalone LiDAR test launch for RPLidar A1.

Launches only:
  1. robot_state_publisher  (provides laser_frame TF from URDF)
  2. Static TF: map → odom (identity, for RViz2 fixed-frame compatibility)
  3. Static TF: odom → base_footprint (identity, robot parked at origin)
  4. lidar_node             (reads /dev/ttyUSB0, publishes /scan)

Usage (on RPi or desktop with lidar plugged in via USB):
    # Build first:
    cd ~/Desktop/ros2_ws
    colcon build --symlink-install --packages-select Ros_lidar_bot
    source install/setup.bash

    # Launch:
    ros2 launch Ros_lidar_bot lidar_test.launch.py

    # Override port/baud if needed:
    ros2 launch Ros_lidar_bot lidar_test.launch.py serial_port:=/dev/ttyUSB0 serial_baud:=115200

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

    # ── Launch Arguments ─────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar A1 (e.g. /dev/ttyUSB0 or /dev/rplidar)')

    serial_baud_arg = DeclareLaunchArgument(
        'serial_baud', default_value='115200',
        description='Baud rate for RPLidar A1 (default 115200)')

    sensitivity_arg = DeclareLaunchArgument(
        'sensitivity_mode', default_value='false',
        description='Use express/sensitivity scan mode (true) or standard (false). '
                    'NOTE: RPLidar A1 firmware v1.29 does NOT support express — use false.')

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

    # ── LiDAR Node ────────────────────────────────────────────────────────
    lidar_node = Node(
        package=package_name,
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'serial_port':      LaunchConfiguration('serial_port'),
            'serial_baud':      LaunchConfiguration('serial_baud'),
            'scan_topic':       '/scan',
            'frame_id':         'laser_frame',
            'min_range':        0.15,
            'max_range':        12.0,
            'publish_rate':     10.0,
            'sensitivity_mode': LaunchConfiguration('sensitivity_mode'),
            'min_quality':      10,    # filter weak echoes; lower → more points, noisier
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        serial_baud_arg,
        sensitivity_arg,
        rsp_node,
        jsp_node,
        static_tf_map_odom,
        static_tf_odom_base,
        lidar_node,
    ])
