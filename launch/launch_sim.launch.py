#!/usr/bin/env python3

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def _first_available_package(*package_names):
    for name in package_names:
        try:
            get_package_share_directory(name)
            return name
        except PackageNotFoundError:
            continue
    raise PackageNotFoundError(
        'None of these simulation/bridge packages were found: ' + ', '.join(package_names)
    )


def generate_launch_description():
    package_name = 'Ros_lidar_bot'
    pkg_share = get_package_share_directory(package_name)

    sim_launch_pkg = _first_available_package('ros_ign_gazebo', 'ros_gz_sim')
    bridge_pkg = _first_available_package('ros_ign_bridge', 'ros_gz_bridge')
    spawn_pkg = _first_available_package('ros_ign_gazebo', 'ros_gz_sim')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description=(
            'World to load. Options: empty | maze_world | '
            'open_obstacles_world | room_world | corridor_world | '
            'circles | boxes | zigzag | spiral | star | '
            'cross | arena | grid | scatter | castle | office_world | warehouse_world'
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
        description='Enable SLAM (used by Nav2 bringup when nav2:=true)'
    )

    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='false',
        description='Launch Nav2 bringup stack'
    )

    explore_arg = DeclareLaunchArgument(
        'explore',
        default_value='false',
        description='Launch explore_lite frontier exploration (requires nav2:=true and slam:=true)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart Nav2 lifecycle nodes'
    )

    explore_delay_arg = DeclareLaunchArgument(
        'explore_delay',
        default_value='8.0',
        description='Delay in seconds before starting explore_lite'
    )

    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Path to Nav2 parameter file'
    )

    explore_params_arg = DeclareLaunchArgument(
        'explore_params_file',
        default_value=os.path.join(pkg_share, 'config', 'explore_lite.yaml'),
        description='Path to explore_lite parameter file'
    )

    world_file = PythonExpression([
        '"', os.path.join(pkg_share, 'worlds'), '/" + "',
        LaunchConfiguration('world'),
        '" + (".world" if "',
        LaunchConfiguration('world'),
        '" == "empty" else ".sdf")'
    ])

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    sim_launch_file = 'ign_gazebo.launch.py' if sim_launch_pkg == 'ros_ign_gazebo' else 'gz_sim.launch.py'
    sim_args_key = 'ign_args' if sim_launch_pkg == 'ros_ign_gazebo' else 'gz_args'

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory(sim_launch_pkg),
                'launch',
                sim_launch_file
            ])
        ),
        launch_arguments={sim_args_key: ['-r ', world_file]}.items()
    )

    spawn_entity = Node(
        package=spawn_pkg,
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
        package=bridge_pkg,
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

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'slam': LaunchConfiguration('slam'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_composition': 'False'
        }.items(),
        condition=IfCondition(LaunchConfiguration('nav2'))
    )

    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    standalone_slam_condition = IfCondition(PythonExpression([
        '"', LaunchConfiguration('slam'), '" == "true" and "',
        LaunchConfiguration('nav2'), '" == "false"'
    ]))

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('slam_toolbox'),
            '/launch/online_async_launch.py'
        ]),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
        condition=standalone_slam_condition
    )

    explore_condition = IfCondition(PythonExpression([
        '"', LaunchConfiguration('explore'), '" == "true" and "',
        LaunchConfiguration('nav2'), '" == "true" and "',
        LaunchConfiguration('slam'), '" == "true"'
    ]))

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[
            LaunchConfiguration('explore_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    delayed_explore = TimerAction(
        period=LaunchConfiguration('explore_delay'),
        actions=[explore_node],
        condition=explore_condition
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        slam_arg,
        nav2_arg,
        explore_arg,
        autostart_arg,
        explore_delay_arg,
        nav2_params_arg,
        explore_params_arg,
        rsp,
        gz_sim,
        spawn_entity,
        bridge,
        nav2_bringup,
        slam_toolbox,
        delayed_explore,
    ])
