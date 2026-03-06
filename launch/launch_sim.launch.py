#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'Ros_lidar_bot'
    pkg_share = get_package_share_directory(package_name)

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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Launch SLAM toolbox with simulation'
    )

    world_with_extension = LaunchConfiguration('world')

    # Build world path including extension based on world name
    from launch.substitutions import PythonExpression
    world_file = PythonExpression([
        '"', os.path.join(pkg_share, 'worlds'), '/" + "',
        world_with_extension,
        '" + (".world" if "',
        world_with_extension,
        '" == "empty" else ".sdf")'
    ])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
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
            'gz_args': ['-r ', world_file]
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
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/world/default/model/my_bot/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/my_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        slam_arg,
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        slam_toolbox,
    ])
