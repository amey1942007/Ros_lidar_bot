#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'Ros_lidar_bot'
    pkg_share = get_package_share_directory(package_name)

    # ── World selector ──────────────────────────────────────────────────────
    # Available worlds:
    #   empty       — basic open world with a few obstacles (default)
    #   maze_world  — walled maze with corridors
    #   open_obstacles — wide open space with many scattered objects
    #   room_world  — multi-room building with doorways
    #   corridor_world — long main corridor with branching side corridors
    #
    # SLAM accuracy test worlds (20×20m arenas):
    #   circles  — spiral of cylinders, increasing radii
    #   boxes    — Tetris-shaped rectangles
    #   zigzag   — snaking corridor with wall teeth
    #   spiral   — spiral labyrinth (4 nested rings)
    #   star     — 5-pointed gold star
    #   cross    — plus-sign dividing 4 rooms
    #   arena    — octagonal arena with pillars
    #   grid     — 4×4 checkerboard of shapes
    #   scatter  — chaotic mix of shapes & angles
    #   castle   — castle with towers, gate, keep
    #
    # Usage: ros2 launch Ros_lidar_bot launch_sim.launch.py world:=star
    # ────────────────────────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description=(
            'World to load. Options: empty | maze_world | '
            'open_obstacles | room_world | corridor_world | '
            'circles | boxes | zigzag | spiral | star | '
            'cross | arena | grid | scatter | castle'
        )
    )

    def world_file(name):
        ext = '.world' if name == 'empty' else '.sdf'
        return os.path.join(pkg_share, 'worlds', name + ext)

    # Map world name → file path at Python time using LaunchConfiguration
    from launch.conditions import IfCondition
    from launch.substitutions import PythonExpression
    world_path = PythonExpression([
        '"' + os.path.join(pkg_share, 'worlds') + '/" + "'
        + '" + "',
        LaunchConfiguration('world'),
        '" + (".world" if "',
        LaunchConfiguration('world'),
        '" == "empty" else ".sdf")'
    ])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
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
            'gz_args': ['-r ', world_path]
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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/world/default/model/my_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/my_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # SLAM Toolbox — online async mapping with our custom params
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

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

    return LaunchDescription([
        world_arg,
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        slam_toolbox,
    ])
