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
            "baud_rate": 500000,
            "output_topic": "/imu",
            "frame_id": "imu_link",
            "publish_rate": 50.0,
            "timeout": 0.1,
        }],
    )

    # ── 3. Motor Driver Node (DDSM115 via UART) ──────────────────────────────
    # Command path: Nav2 publishes /cmd_vel, safety_stop filters it into
    # /cmd_vel_safe, and this driver consumes only /cmd_vel_safe.
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
            # EKF (started at T=0) is the ONLY publisher of odom→base_footprint.
            # Two publishers of the same transform (odom_node raw pose vs EKF
            # fused pose) fight each other — TF flickers between two headings,
            # which showed up as the LiDAR scan rotating/jumping in RViz.
            "broadcast_tf": False,
        }],
    )

    # ── 5. LiDAR Node (RPLidar A1 via UART) ──────────────────────────────────
    # lidar_node = Node(
    #     package=package_name,
    #     executable="lidar_node",
    #     name="lidar_node",
    #     output="screen",
    #     parameters=[{
    #         "serial_port":      "/dev/ttyUSB0",  # RPLidar A1 USB-serial (CP2102). Adjust if different.
    #         "serial_baud":      115200,           # A1 default baud rate
    #         "scan_topic":       "/scan",
    #         "frame_id":         "laser_frame",   # Must match URDF link name
    #         "min_range":        0.15,             # Ignore returns closer than 15 cm
    #         "max_range":        12.0,             # A1 max range
    #         "publish_rate":     10.0,             # Hz — publish rate throttle
    #         "motor_pwm":        660,              # Motor PWM value (passed to set_motor_pwm)
    #         "sensitivity_mode": True,             # True = Express/Sensitivity (mode 1), False = Standard (mode 0)
    #                                               # Mode 1 gives higher point density on RPLidar A1.
    #                                               # Falls back to Standard automatically if unsupported.
    #     }],
    # )

    # ── 5. LiDAR Node (RPLidar A1 via UART — official Slamtec driver) ────────
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",   # ← changed from "rplidar_node"
        name="rplidar_node",                # this can stay whatever you want, it's just the node's name
        output="screen",
        # If startup fails (e.g. '80008000' scan-start timeout while the motor
        # spins up), restart instead of staying dead — no /scan means no /map.
        # 2s wasn't enough headroom: the motor has real mechanical spin-down
        # time, so a retry that fires before it's actually stopped just hits
        # the same failure again. 8s gives it room to settle first.
        respawn=True,
        respawn_delay=8.0,
        parameters=[{
            "channel_type":      "serial",
            "serial_port":       "/dev/ttyUSB0",
            "serial_baudrate":   115200,
            "frame_id":          "laser_frame",
            "inverted":          False,
            "angle_compensate":  True,
            # Standard (normal) mode — the A1 does NOT support "Sensitivity"
            # (A3/S-series only); requesting it made the driver exit with
            # "Failed to set scan mode". Standard skips the driver's mode
            # auto-negotiation with the device, unlike leaving this unset.
            "scan_mode":         "Standard",
        }],
    )

    # ── 6. Safety Stop Node ──────────────────────────────────────────────────
    # Keep this node in the command path: it subscribes to Nav2's /cmd_vel and
    # publishes filtered commands to /cmd_vel_safe for driver_node. Do not
    # remap driver_node to /cmd_vel, which would bypass the safety filter.
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

    # ── 9. Nav2 Navigation Stack ────────────────────────────────────────────
    # Nav2 publishes /cmd_vel. safety_stop below filters that into
    # /cmd_vel_safe, which is the driver's only velocity-command input.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": os.path.join(pkg_share, "config", "nav2_params.yaml"),
        }.items(),
    )

    # ── 10. Semantic SLAM Node ───────────────────────────────────────────────
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

    # ── 11. IMU Calibration Node ─────────────────────────────────────────────
    # Spins the robot ~2 rotations in place at startup to exercise the BNO055's
    # self-calibration and write offsets to config/imu_calibration.yaml.
    # NOTE: unlike the old staged design (which delayed EKF/SLAM by 40 s and
    # caused a startup TF gap), EKF and SLAM still start on their own schedule —
    # this node just runs alongside them. The robot WILL physically rotate
    # shortly after launch; make sure it has clearance.
    imu_calibration = Node(
        package=package_name,
        executable="imu_calibration_node",
        name="imu_calibration_node",
        output="screen",
    )

    # ── 12. Non-Safety Teleop Node (keyboard, bypasses safety stop) ───────────
    # DISABLED in launch: Keyboard teleop requires stdin/tty. It crashes when
    # launched without xterm. Run this manually in a separate terminal:
    # ros2 run Ros_lidar_bot non_safety_teleop
    #
    # non_safety_teleop = Node(
    #     package=package_name,
    #     executable="non_safety_teleop",
    #     name="non_safety_teleop_node",
    #     output="screen",
    # )

    # ── [DISABLED] YOLO-World Publisher Node — re-enable when needed ─────────
    # yolo_node = Node(
    #     package=package_name,
    #     executable="yolo",
    #     name="yolo_world_publisher",
    #     output="screen",
    #     arguments=[
    #         "--model", "yolov8s-world.pt",
    #         "--camera", "0",
    #         "--conf", "0.25",
    #         "--rate", "10.0",
    #     ],
    # )

    return LaunchDescription([
        # ── Stage 1 (T=0s): Hardware + EKF ─────────────────────────────────────
        # EKF starts immediately so odom→base_footprint exists as soon as the
        # first /odom_raw message arrives (~1 s). It is the sole TF publisher
        # for odom→base_footprint.
        rsp,
        imu_node,
        driver_node,
        odom_node,
        lidar_node,
        safety_stop,
        ekf_node,

        # ── Stage 2 (T=5s): SLAM ──────────────────────────────────────────────
        # Short delay so the odom→base_footprint TF and /scan are already
        # flowing when SLAM Toolbox processes its first scan.
        TimerAction(period=5.0, actions=[slam_toolbox]),

        # Nav2 starts after SLAM's map and map→odom transform are available.
        TimerAction(period=8.0, actions=[nav2]),

        # ── Stage 3 (T=8s): IMU calibration spin ─────────────────────────────
        # Starts after IMU/driver/EKF/SLAM are up so the spin is tracked by
        # odometry and SLAM rather than happening in a TF vacuum.
        TimerAction(period=8.0, actions=[imu_calibration]),

        # ── Teleop: run manually in a separate terminal (needs a tty) ─────────
        # ros2 run Ros_lidar_bot non_safety_teleop

        # ── [DISABLED] Semantic SLAM — re-enable when model is ready ──────────
        # TimerAction(period=50.0, actions=[semantic_slam]),

        # ── [DISABLED] YOLO-World — re-enable when needed ─────────────────────
        # TimerAction(period=50.0, actions=[yolo_node]),
    ])
