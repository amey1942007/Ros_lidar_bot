#!/usr/bin/env python3
"""
amr4_teleop.launch.py
=====================
AMR4 — Controller (joystick) drive launch file
Platform : Jetson Orin Nano · Ubuntu 22.04 · ROS 2 Humble

What this launch file starts
-----------------------------
  1. Robot State Publisher  (URDF → /tf)
  2. AMR4 Driver Node       (/cmd_vel → Serial → DriveMaster.ino)
  3. joy_node               (reads gamepad → /joy)
  4. joy_teleop_node        (/joy → /cmd_vel)

Architecture
------------
  [Gamepad]
      │
  joy_node ──► /joy
                │
         joy_teleop_node ──► /cmd_vel
                                │
                        amr4_driver_node ──► Serial ──► Arduino (DriveMaster.ino)
                                                              │
                                                     [4 mecanum wheels]

Navigation (Nav2 / SLAM) is NOT started here — this launch is
for controller-only driving.  Add nav2 later as a separate launch.

Usage
-----
  ros2 launch Ros_lidar_bot amr4_teleop.launch.py
  ros2 launch Ros_lidar_bot amr4_teleop.launch.py serial_port:=/dev/ttyACM1
  ros2 launch Ros_lidar_bot amr4_teleop.launch.py verbose:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ──────────────────────────────────────────────────────────────────────────────
def _launch_setup(context, *args, **kwargs):
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)

    verbose = LaunchConfiguration("verbose").perform(context).lower() in (
        "1", "true", "yes",
    )
    serial_port = LaunchConfiguration("serial_port").perform(context)
    lidar_ip    = LaunchConfiguration("lidar_ip").perform(context)
    scan_mode   = LaunchConfiguration("scan_mode").perform(context)

    out = "screen" if verbose else "log"

    # ── 1. Robot State Publisher ──────────────────────────────────────────────
    # Publishes the URDF TF tree so RViz / tf2 know the robot geometry.
    # Fill in the URDF placeholders (TODO_FILL) in slam.xacro / imu.xacro /
    # lidar.xacro before running — xacro will fail on un-substituted TODO_FILL
    # values until they are replaced with real numbers.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output=out,
        parameters=[{
            "use_sim_time": False,
            # Joint states for the 4 continuous wheel joints are published
            # by the driver node (future: once odom_node is added).
            # For now RSP uses /joint_states if available, otherwise holds last.
            "robot_description": _get_robot_description(pkg_share),
        }],
    )

    # ── 2. AMR4 Driver Node ───────────────────────────────────────────────────
    # Bridges /cmd_vel → "DRIVE,vy,vx,omega\n" → Arduino serial.
    # Also reads back Arduino telemetry and publishes /wheel_rpms and /imu/raw.
    driver_node = Node(
        package=package_name,
        executable="amr4_driver",
        name="amr4_driver_node",
        output=out,
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            # ── Serial connection ────────────────────────────────────────────
            # Change serial_port to match your Jetson USB port.
            # Use:  ls /dev/ttyACM*   or  ls /dev/serial/by-id/
            "serial_port":   serial_port,
            "baud_rate":     115200,    # must match BAUD_HOST in Config.h

            # ── Topic ────────────────────────────────────────────────────────
            # joy_teleop publishes directly to /cmd_vel for controller drive.
            # Change to /cmd_vel_safe when safety_stop_node is added.
            "cmd_vel_topic": "/cmd_vel",

            # ── Safety watchdog ──────────────────────────────────────────────
            # Send STOP if no cmd_vel arrives within this many seconds.
            # 0.5 s is safe for a controller — stick release → stop quickly.
            "cmd_timeout":   0.5,

            # ── IMU re-publish ───────────────────────────────────────────────
            # Publish BNO055 data from Arduino telemetry as sensor_msgs/Imu.
            "publish_imu":   True,
            "frame_id":      "imu_link",
        }],
    )

    # ── 3. joy_node ───────────────────────────────────────────────────────────
    # Reads the gamepad / controller and publishes /joy.
    # Supports USB HID and Bluetooth gamepads via SDL2.
    # If you have multiple controllers, set device_id accordingly.
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output=out,
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            "device_id":      0,       # 0 = first controller detected
            "deadzone":       0.05,    # ignore stick jitter below 5 %
            # Keep /joy publishing at 20 Hz while a stick is held,
            # so joy_teleop's watchdog never fires mid-motion.
            "autorepeat_rate": 20.0,
        }],
    )

    # ── 4. joy_teleop_node ────────────────────────────────────────────────────
    # Maps /joy axes/buttons → /cmd_vel (geometry_msgs/Twist).
    # This is the existing joy_teleop_node.py in this package.
    # Default mapping (see joy_teleop_node.py for full details):
    #   Left  stick Y-axis → linear.x  (forward/back)
    #   Left  stick X-axis → linear.y  (left/right strafe)
    #   Right stick X-axis → angular.z (rotate)
    #   Triggers (LT/RT)   → speed scale
    joy_teleop = Node(
        package=package_name,
        executable="joy_teleop",
        name="joy_teleop",
        output=out,
        respawn=True,
        respawn_delay=2.0,
    )

    # ── 5. RPLidar S2E (Ethernet / UDP) ─────────────────────────────────────
    # Uses sllidar_ros2 (Slamtec official source build — NOT apt rplidar_ros).
    # Jetson ethernet port must be on 192.168.11.1/24 (see README: nmcli setup).
    # Publishes /scan (sensor_msgs/LaserScan) on frame_id=laser_frame.
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='rplidar_node',
        output=out,
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'channel_type':     'udp',
            'udp_ip':           lidar_ip,
            'udp_port':         8089,
            'frame_id':         'laser_frame',   # must match lidar.xacro
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        scan_mode,
        }],
    )

    return [
        rsp,
        driver_node,
        joy_node,
        joy_teleop,
        lidar_node,
    ]


# ──────────────────────────────────────────────────────────────────────────────
def _get_robot_description(pkg_share: str) -> str:
    """
    Run xacro on the top-level URDF and return the resulting XML string.
    robot_state_publisher expects the expanded URDF, not a file path.
    """
    import subprocess
    xacro_file = os.path.join(pkg_share, "description", "robot.urdf.xacro")
    try:
        result = subprocess.run(
            ["xacro", xacro_file],
            capture_output=True, text=True, check=True,
        )
        return result.stdout
    except subprocess.CalledProcessError as exc:
        # If xacro fails (e.g. TODO_FILL not yet filled in) print a clear error.
        raise RuntimeError(
            f"xacro failed on {xacro_file}:\n{exc.stderr}\n"
            "Make sure you have replaced every TODO_FILL placeholder in the "
            "description/*.xacro files with real numbers."
        ) from exc


# ──────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "verbose",
            default_value="false",
            description="Set 'true' to print all node logs to the terminal.",
        ),
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="USB serial port of the Arduino (DriveMaster.ino).",
        ),
        DeclareLaunchArgument(
            "lidar_ip",
            default_value="192.168.11.2",
            description="RPLidar S2E IP address (factory default 192.168.11.2).",
        ),
        DeclareLaunchArgument(
            "scan_mode",
            default_value="DenseBoost",
            description="Scan mode: DenseBoost | Standard | Sensitivity.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
