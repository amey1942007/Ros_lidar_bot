#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={"use_sim_time": "false"}.items(),
    )

    # ── 2. IMU Node (BNO055 via Arduino Mega UART) ─────────────────────────────
    imu_node = Node(
        package=package_name,
        executable="imu_node",
        name="imu_node",
        output="screen",
        parameters=[{
            "serial_port": "/dev/ttyACM1",
            "baud_rate": 115200,
            "output_topic": "/imu",
            "frame_id": "imu_link",
            "publish_rate": 100.0,
            "timeout": 0.1,
        }],
    )

    # ── 3. Motor Driver Node (DDSM115 via UART) ──────────────────────────────
    driver_node = Node(
        package=package_name,
        executable="driver_node",
        name="driver_node",
        output="screen",
        parameters=[{
            "serial_port": "/dev/ttyACM0",
            "baud_rate": 115200,
        }],
    )

    # ── 4. Odometry Node ─────────────────────────────────────────────────────
    odom_node = Node(
        package=package_name,
        executable="odom_node",
        name="odom_node",
        output="screen",
        parameters=[{
            "broadcast_tf": False,   # EKF publishes odom→base_footprint — don't double-publish
        }],
    )

    # ── 5. LiDAR Node (RPLidar A1 via UART) ──────────────────────────────────
    lidar_node = Node(
        package=package_name,
        executable="lidar_node",
        name="lidar_node",
        output="screen",
    )

    # ── 6. Safety Stop Node ──────────────────────────────────────────────────
    safety_stop = Node(
        package=package_name,
        executable="safety_stop_node",
        name="safety_stop",
        output="screen",
        parameters=[{
            "min_safe_distance": 0.25,    # stop if obstacle within 25 cm
            "front_opening_deg": 180.0,
            "rear_opening_deg": 60.0,
        }],
    )

    # ── 7. EKF Node (Fuses Odom & IMU) ───────────────────────────────────────
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("/odometry/filtered", "/odom")],
    )

    # ── 8. SLAM Toolbox ──────────────────────────────────────────────────────
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
            "use_sim_time": "false",
        }.items(),
    )

    # ── 9. Semantic SLAM Node ────────────────────────────────────────────────
    # DISABLED: Missing model file "Vision Model/best.pt"
    # semantic_slam = Node(
    #     package=package_name,
    #     executable="semantic_slam_node.py",
    #     name="semantic_slam",
    #     output="screen",
    #     parameters=[{
    #         "model_path": os.path.join(pkg_share, "Vision Model", "best.pt"),
    #         "detect_hz": 1.0,
    #         "conf": 0.25,
    #         "startup_delay_sec": 15.0,  # Wait for SLAM to settle
    #         "max_depth": 4.0,
    #     }],
    # )

    # ── 10. YOLO-World Publisher Node ────────────────────────────────────────
    # Note: parses arguments via argparse, so we pass them via 'arguments' instead of 'parameters'.
    yolo_node = Node(
        package=package_name,
        executable="yolo",
        name="yolo_world_publisher",
        output="screen",
        arguments=[
            "--model", "yolov8s-world.pt",
            "--camera", "0",
            "--conf", "0.25",
            "--rate", "10.0",
        ],
    )

    return LaunchDescription([
        # Stage 1 (T=0): Base setup, sensors, and hardware drivers
        rsp,
        imu_node,
        driver_node,
        odom_node,
        lidar_node,
        safety_stop,

        # Stage 2 (T=3s): Start EKF and SLAM after sensors are online
        TimerAction(period=3.0, actions=[ekf_node, slam_toolbox]),

        # Stage 3 (T=8s): Start YOLO-World (semantic_slam disabled - missing model)
        TimerAction(period=8.0, actions=[yolo_node]),
    ])
