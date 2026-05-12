#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'Ros_lidar_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'testing.world'
            )
        }.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'my_bot',
            '-world', 'default',
            '-z', '0.1'
        ],
        output='screen'
    )

    slam_params_file = os.path.join(
        get_package_share_directory(package_name), 'config', 'mapper_params_online_async.yaml'
    )

    ekf_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': 'true'
        }.items()
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            # Add the IMU once we fix the URDF/World
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        remappings=[
            ('/odom', '/odom_raw') # This sends Gazebo's /odom to EKF's input
        ],
        output='screen'
    )

    # 2. EKF Node: Consumes /odom_raw + /imu -> Produces /odom
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odom')] # Clean filtered data
    )
    

    return LaunchDescription([
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        slam_toolbox,
        ekf_node
    ])
