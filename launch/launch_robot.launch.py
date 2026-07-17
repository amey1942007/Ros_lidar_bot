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
    # FATAL = only crashes; keeps bringup_status board alone on the TTY.
    log_args = [] if verbose else ["--ros-args", "--log-level", "fatal"]
    nav2_log_level = "info" if verbose else "fatal"

    actions = []
    if not verbose:
        # Kill ROS logger spam from included launches (slam/nav2/rsp) too.
        actions.append(SetEnvironmentVariable("RCUTILS_LOGGING_MIN_SEVERITY", "FATAL"))
        # Soften CycloneDDS discovery chatter when RViz joins from another host.
        actions.append(SetEnvironmentVariable("RCUTILS_CONSOLE_OUTPUT_FORMAT", "[{severity}] [{name}]: {message}"))

    # ── Web dashboard (replaces the terminal bringup_status board) ───────────
    # Serves the interactive GUI at http://<pi-ip>:8080 — open it in a
    # browser on the laptop (the Pi is headless over SSH, so no X11 needed).
    # Status board + live robot/lidar view + on-demand tools (IMU test,
    # IMU calibration, drive-distance) + click-to-send nav goals + E-STOP.
    # The old terminal board is still available:
    #   ros2 run Ros_lidar_bot bringup_status
    dashboard = Node(
        package=package_name,
        executable="robot_dashboard",
        name="robot_dashboard",
        output="screen",
        emulate_tty=True,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "port": 8080,
            "expect_frontier": expect_frontier,
        }],
    )

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "rsp.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "false",
            "output": out,
        }.items(),
    )

    # ── 2. IMU Node (BNO055 via Arduino Mega UART) ─────────────────────────────
    imu_node = Node(
        package=package_name,
        executable="imu_node",
        name="imu_node",
        output=out,
        arguments=log_args,
        # Serial nodes self-heal (retry/reconnect loops), but respawn covers
        # any crash path they can't recover from.
        respawn=True,
        respawn_delay=3.0,
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
    # Direct path: Nav2 / teleop → /cmd_vel → driver (safety_stop disabled).
    driver_node = Node(
        package=package_name,
        executable="driver_node",
        name="driver_node",
        output=out,
        arguments=log_args,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "serial_port": "/dev/ttyACM0",
            "baud_rate": 115200,
            # 20 Hz encoder feedback → denser wheel odometry for the EKF.
            "poll_rate": 20.0,
            # False: teleop W / ROS +X = physical forward on this chassis.
            "invert_drive": False,
            "cmd_vel_topic": "/cmd_vel",
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
    # Slamtec sllidar_ros2 (UDP). Do NOT use apt ros-jazzy-rplidar-ros SDK
    # 1.12 — that build is serial-only and ignores channel_type:=udp.
    # Install: git clone https://github.com/Slamtec/sllidar_ros2.git into src/
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
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
            # Standard (was DenseBoost): DenseBoost's ~3200 pts/rev cost the
            # RPi5 so much scan-match/raytrace CPU that map→odom ran ~0.6 s
            # stale (measured 2026-07-17) — Nav2 TF failures, map smear from
            # late corrections, and general stutter all traced back to it.
            # Standard still gives ~1000+ pts/rev at 10 Hz, plenty for indoor
            # SLAM, and the whole pipeline runs fresher. If the driver logs
            # "Failed to set scan mode", try "Sensitivity".
            "scan_mode":         "Standard",
        }],
    )

    # ── 6. Safety Stop Node — DISABLED ───────────────────────────────────────
    # User requested: driver takes /cmd_vel directly from Nav2 (no filter).
    # Re-enable by uncommenting the Node below and adding safety_stop to
    # actions.extend([...]), and set driver cmd_vel_topic back to /cmd_vel_safe.
    # safety_stop = Node(
    #     package=package_name,
    #     executable="safety_stop_node",
    #     name="safety_stop",
    #     output=out,
    #     arguments=log_args,
    #     parameters=[{
    #         "min_safe_distance": 0.40,
    #         "ignore_below": 0.28,
    #         "front_opening_deg": 90.0,
    #         "rear_opening_deg": 50.0,
    #     }],
    # )

    # ── 6b. Gamepad teleop (Bluetooth or USB controller) ─────────────────────
    # joy_node (SDL) reads the controller and publishes /joy; joy_teleop maps
    # it to /cmd_vel. Left stick = movement, RT/LT = linear speed ±,
    # RB/LB = angular speed ±. joy_teleop yields /cmd_vel to Nav2 whenever
    # the stick is centered, so both can coexist on the same topic.
    #
    # Bluetooth: pair once on the Pi with bluetoothctl (scan on / pair / trust
    # / connect) — "trust" makes it auto-reconnect on power-up. NOTE: over
    # Bluetooth many pads expose a DIFFERENT axis/button order than over a
    # USB dongle (e.g. Xbox BT without xpadneo puts triggers elsewhere).
    # If controls act wrong, check `ros2 topic echo /joy` and override the
    # axis_*/button_* parameters on joy_teleop below.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output=out,
        arguments=log_args,
        # Survive controller disconnect/reconnect (BT dropout or USB unplug).
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "device_id": 0,
            "deadzone": 0.05,
            # Keep /joy streaming while a stick is held so joy_teleop's
            # 0.5 s watchdog never fires mid-motion.
            "autorepeat_rate": 20.0,
        }],
    )

    joy_teleop = Node(
        package=package_name,
        executable="joy_teleop",
        name="joy_teleop",
        output=out,
        arguments=log_args,
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
    # Nav2 publishes /cmd_vel → driver_node (safety_stop bypassed).
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
        dashboard,
        # ── Stage 1 (T=0s): Hardware + EKF ─────────────────────────────────────
        rsp,
        imu_node,
        driver_node,
        odom_node,
        lidar_node,
        joy_node,
        joy_teleop,
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
