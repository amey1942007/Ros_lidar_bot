#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)
    world = LaunchConfiguration("world")

    # ── Robot State Publisher ─────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    # ── Ignition Gazebo ───────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    # ── Gazebo → ROS 2 bridge ─────────────────────────────────────────────────
    # The argument keeps /cmd_vel so the Gazebo DiffDrive plugin receives on
    # its expected "cmd_vel" topic.  The Node-level remapping makes the bridge
    # subscribe to /cmd_vel_safe (safety node output) on the ROS side, so every
    # velocity command passes through the safety filter before reaching the robot.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        arguments=[
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            "/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
        ],
        remappings=[
            ("/odom", "/odom_raw"),
            ("/cmd_vel", "/cmd_vel_safe"),  # bridge reads from safety node output
        ],
        output="screen",
    )

    # ── Scan-based hardware safety stop ──────────────────────────────────────
    # Intercepts /cmd_vel from Nav2/teleop, checks the laser scan,
    # and blocks forward/reverse motion when an obstacle is < min_safe_distance.
    # Outputs safe commands on /cmd_vel_safe which the bridge forwards to Gazebo.
    safety_stop = Node(
        package=package_name,
        executable="safety_stop_node.py",
        name="safety_stop",
        output="screen",
        parameters=[{
            "min_safe_distance": 0.25,    # stop if obstacle within 25 cm
            "front_opening_deg": 120.0,   # forward safety arc (±60° from nose)
            "rear_opening_deg": 60.0,     # rear safety arc (±30° from tail)
        }],
    )

    # ── Spawn robot into Gazebo ───────────────────────────────────────────────
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", "my_bot",
            "-world", "default",
            "-z", "0.1",
        ],
        output="screen",
    )

    # ── EKF: fuses wheel odometry + IMU → clean /odom ────────────────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("/odometry/filtered", "/odom")],
    )

    # ── SLAM Toolbox: builds map + publishes map→odom TF ─────────────────────
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "slam_params_file": os.path.join(
                pkg_share, "config", "mapper_params_online_async.yaml"
            ),
            "use_sim_time": "true",
        }.items(),
    )

    # ── Nav2 Navigation Stack ─────────────────────────────────────────────────
    # Delayed 10 s so SLAM publishes at least one /map before costmaps start.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(pkg_share, "config", "nav2_params.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_share, "worlds", "testing.world"),
            description="Absolute path to Gazebo world SDF file",
        ),
        # Stage 1 (T=0): Gazebo + robot description + bridge + safety node
        rsp,
        gz_sim,
        bridge,
        safety_stop,
        # Stage 2 (T=3s): spawn robot once Gazebo is ready
        TimerAction(period=3.0, actions=[spawn_entity]),
        # Stage 3 (T=6s): start EKF + SLAM after robot is spawned and sensors publishing
        TimerAction(period=6.0, actions=[ekf_node, slam_toolbox]),
        # Stage 4 (T=16s): Nav2 after SLAM has had time to build initial map
        TimerAction(period=16.0, actions=[nav2]),
    ])
