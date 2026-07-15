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
            # broadcast_tf=True so odom→base_footprint exists from T=0.
            # EKF starts at T=40s and publishes the same transform at 30 Hz;
            # robot_localization simply takes over — no TF conflict occurs.
            # Without this, there is NO odom→base_footprint TF for the first
            # 40 s, which causes the LiDAR scan and map to follow/rotate with
            # the robot in RViz (broken TF chain: map→odom→??→laser_frame).
            "broadcast_tf": True,
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
        parameters=[{
            "channel_type":      "serial",
            "serial_port":       "/dev/ttyUSB0",
            "serial_baudrate":   115200,
            "frame_id":          "laser_frame",
            "inverted":          False,
            "angle_compensate":  True,
            "scan_mode":         "Sensitivity",
        }],
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

    # ── 10. IMU Calibration Node ─────────────────────────────────────────────
    # Runs ONCE at startup: warmup (3s still) → rotate in-place (2×360°, ~25s)
    # → saves config/imu_calibration.yaml → shuts itself down automatically.
    #
    # Timing budget:
    #   warmup_duration  =  3 s
    #   rotation_count=2 at rotation_speed=0.5 rad/s → 2×2π/0.5 ≈ 25 s
    #   extra buffer                                            ≈  7 s
    #   ──────────────────────────────────────────────────────────────
    #   Total from node start                                  ≈ 35 s
    #
    # EKF + SLAM are therefore delayed to T=40s to guarantee calibration
    # is complete before the filter and mapper initialise.
    #
    # ⚠ IMPORTANT: Keep the robot STILL for the first 3 seconds after launch.
    #   Clear ~1 m of space around the robot before launching.
    imu_calibration_node = Node(
        package=package_name,
        executable="imu_calibration_node",
        name="imu_calibration_node",
        output="screen",
        parameters=[{
            "imu_topic":          "/imu",
            "cmd_vel_topic":      "/cmd_vel",    # bypasses safety stop during calib spin
            "rotation_speed":     0.5,           # rad/s — gentle spin
            "rotation_count":     2.0,           # 2 full rotations = 720°
            "warmup_duration":    3.0,           # seconds stationary at start
            "min_mag_samples":    200,           # minimum mag samples for hard-iron fit
            "output_yaml_path":   os.path.join(pkg_share, "config", "imu_calibration.yaml"),
        }],
    )

    # ── 11. Non-Safety Teleop Node (keyboard, bypasses safety stop) ───────────
    # Publishes directly to /cmd_vel_safe so the safety_stop filter is skipped.
    # Uses prefix="xterm -e" only if DISPLAY is set, otherwise runs directly.
    non_safety_teleop = Node(
        package=package_name,
        executable="non_safety_teleop",
        name="non_safety_teleop_node",
        output="screen",
        prefix="xterm -e" if os.environ.get("DISPLAY") else "",
    )

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
        # ── Stage 1 (T=0s): Hardware — sensors + drivers come up first ─────────
        rsp,
        imu_node,
        driver_node,
        odom_node,
        lidar_node,
        safety_stop,

        # ── Stage 2 (T=2s): IMU Calibration ───────────────────────────────────
        # Starts 2s after hardware to let serial connections settle.
        # Warmup 3s (KEEP BOT STILL) → rotates 720° → saves YAML → auto-exits.
        # Total duration ≈ 35s. EKF/SLAM start at T=40s to wait for it.
        TimerAction(period=2.0, actions=[imu_calibration_node]),

        # ── Stage 3 (T=40s): Localization + SLAM ─────────────────────────────
        # Starts after calibration is guaranteed complete (~35s after T=2s).
        # EKF fuses /odom_raw + /imu → publishes /odom.
        # SLAM Toolbox reads /scan + /odom TF → builds /map.
        TimerAction(period=40.0, actions=[ekf_node, slam_toolbox]),

        # ── Stage 4 (T=45s): Non-safety teleop for manual mapping drive ───────
        # 5s after EKF/SLAM so the filter has time to initialise before driving.
        TimerAction(period=45.0, actions=[non_safety_teleop]),

        # ── [DISABLED] Semantic SLAM — re-enable when model is ready ──────────
        # TimerAction(period=50.0, actions=[semantic_slam]),

        # ── [DISABLED] YOLO-World — re-enable when needed ─────────────────────
        # TimerAction(period=50.0, actions=[yolo_node]),
    ])

