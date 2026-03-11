#!/usr/bin/env python3
"""
Launch file — Ros_lidar_bot
Ignition Gazebo 6 (Fortress) + ROS2 Humble

Stack:
  1. robot_state_publisher  → publishes URDF TF tree (fixed joints + static frames)
  2. gz_sim (Ignition)      → physics + DiffDrive + GPU LiDAR
  3. spawn (create)         → spawns robot URDF into Ignition world
  4. ros_gz_bridge          → /clock, /odom, /cmd_vel, /tf, /scan, /joint_states
  5. slam_toolbox           → builds occupancy map from /scan

Usage:
  ros2 launch Ros_lidar_bot launch_sim.launch.py
  ros2 launch Ros_lidar_bot launch_sim.launch.py world:=maze_world

Available worlds: empty | maze_world | open_obstacles | room_world |
  corridor_world | circles | boxes | zigzag | spiral | star |
  cross | arena | grid | scatter | castle
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'Ros_lidar_bot'
    pkg_share = get_package_share_directory(package_name)

    # ── World selector ───────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description=(
            'World to load. Options: empty | maze_world | open_obstacles | '
            'room_world | corridor_world | circles | boxes | zigzag | spiral | '
            'star | cross | arena | grid | scatter | castle'
        )
    )

    # Build the world file path at launch time.
    # 'empty' → .world   |   everything else → .sdf
    worlds_dir = os.path.join(pkg_share, 'worlds')
    world_path = PythonExpression([
        '"' + worlds_dir + '/" + "',
        LaunchConfiguration('world'),
        '" + (".world" if "',
        LaunchConfiguration('world'),
        '" == "empty" else ".sdf")'
    ])

    # ── Robot State Publisher ────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # ── Ignition Gazebo ──────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': ['-r ', world_path]
        }.items()
    )

    # ── Spawn robot ──────────────────────────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name',  'my_bot',
            '-z',     '0.1',
        ],
        output='screen'
    )

    # ── ROS ↔ Ignition Bridge ────────────────────────────────────────────────
    # Syntax:
    #   /topic@ros_type[ign_type   → Ignition to ROS (unidirectional)
    #   /topic@ros_type]ign_type   → ROS to Ignition (unidirectional)
    #   /topic@ros_type[ign_type]  → bidirectional
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock — CRITICAL: keeps all ROS timestamps in sync with sim time
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',

            # Odometry — Ignition DiffDrive → ROS
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

            # Velocity commands — ROS teleop → Ignition DiffDrive
            # MUST use absolute topic '/cmd_vel' to match DiffDrive plugin topic
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',

            # TF tree — Ignition (odom→base_link, wheel TF) → ROS
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',

            # LiDAR scan — Ignition GPU Lidar → ROS (used by slam_toolbox)
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

            # Joint states — needed for robot_state_publisher to render
            # the robot model correctly in RViz (wheel rotation animation)
            # world name = "default" (set in all .world/.sdf files)
            # model name = "my_bot" (set in spawn_entity above)
            '/world/default/model/my_bot/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/world/default/model/my_bot/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    # ── SLAM Toolbox — online async ──────────────────────────────────────────
    slam_params_file = os.path.join(
        pkg_share, 'config', 'mapper_params_online_async.yaml'
    )
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
