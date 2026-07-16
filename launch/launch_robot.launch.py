#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)

    verbose = LaunchConfiguration("verbose").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )
    expect_frontier = LaunchConfiguration("expect_frontier").perform(context).lower() in (
        "1",
        "true",
        "yes",
    )

    out = "screen" if verbose else "log"
    log_args = [] if verbose else ["--ros-args", "--log-level", "warn"]
    nav2_log_level = "info" if verbose else "warn"

    actions = []
    if not verbose:
        # Quiet third-party / included launch INFO spam; status board uses print().
        actions.append(SetEnvironmentVariable("RCUTILS_LOGGING_MIN_SEVERITY", "WARN"))

    # ── Bringup status board (always on screen) ──────────────────────────────
    bringup_status = Node(
        package=package_name,
        executable="bringup_status",
        name="bringup_status",
        output="screen",
        parameters=[{
            "expect_frontier": expect_frontier,
            "update_hz": 1.0,
            "verbose_issues": True,
            "clear_screen": True,
        }],
    )

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
        output=out,
        arguments=log_args,
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
        output=out,
        arguments=log_args,
        parameters=[{
            "serial_port": "/dev/ttyACM0",
            "baud_rate": 115200,
            # 20 Hz encoder feedback → denser wheel odometry for the EKF.
            "poll_rate": 20.0,
        }],
    )

    # ── 4. Odometry Node ─────────────────────────────────────────────────────
    odom_node = Node(
        package=package_name,
        executable="odom_node",
        name="odom_node",
        output=out,
        arguments=log_args,
        parameters=[{
            # EKF (started at T=0) is the ONLY publisher of odom→base_footprint.
            # Two publishers of the same transform (odom_node raw pose vs EKF
            # fused pose) fight each other — TF flickers between two headings,
            # which showed up as the LiDAR scan rotating/jumping in RViz.
            "broadcast_tf": False,
        }],
    )

    # ── 5. LiDAR Node (RPLidar S2E via Ethernet/UDP — official Slamtec driver)
    #
    # The S2E streams over Ethernet (UDP), NOT USB-serial. This removes the
    # whole class of A1 failures we fought before (CP2102 dropouts, motor
    # spin-up '80008000' scan-start timeouts, USB bus contention with the two
    # Arduino/RS485 serial links).
    #
    # NETWORK SETUP REQUIRED on the RPi 5 (one-time):
    #   The S2E has a fixed default IP of 192.168.11.2 and talks UDP :8089.
    #   Give eth0 a static address on the same subnet, e.g. with nmcli:
    #     sudo nmcli con add type ethernet ifname eth0 con-name lidar \
    #          ipv4.method manual ipv4.addresses 192.168.11.1/24
    #   Verify with:  ping 192.168.11.2
    #
    # Requires ros-jazzy-rplidar-ros >= 2.1 (has udp/tcp channel support).
    lidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        name="rplidar_node",
        output=out,
        arguments=log_args,
        # Respawn if the driver ever exits (cable yank, power dip). UDP needs
        # no motor spin-down grace period like the A1 did, so 5 s is plenty.
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            "channel_type":      "udp",
            "udp_ip":            "192.168.11.2",   # S2E factory default
            "udp_port":          8089,             # S2E factory default
            "frame_id":          "laser_frame",
            "inverted":          False,
            "angle_compensate":  True,
            # DenseBoost = the S2E's full 32 kHz sample rate (~3200 pts/rev
            # at 10 Hz) — the "full power" mode. Falls back is NOT automatic:
            # if the driver logs "Failed to set scan mode", set "Standard".
            "scan_mode":         "DenseBoost",
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
        output=out,
        arguments=log_args,
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
        output=out,
        arguments=log_args,
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
            "log_level": nav2_log_level,
        }.items(),
    )

    actions.extend([
        bringup_status,
        # ── Stage 1 (T=0s): Hardware + EKF ─────────────────────────────────────
        rsp,
        imu_node,
        driver_node,
        odom_node,
        lidar_node,
        safety_stop,
        ekf_node,
        # ── Stage 2 (T=5s): SLAM ──────────────────────────────────────────────
        TimerAction(period=5.0, actions=[slam_toolbox]),
        # Nav2 starts after SLAM's map and map→odom transform are available.
        TimerAction(period=8.0, actions=[nav2]),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="If true, all node logs go to the terminal (old spammy mode).",
        ),
        DeclareLaunchArgument(
            "expect_frontier",
            default_value="false",
            description="If true, bringup_status waits for frontier_explorer (autonomous).",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
