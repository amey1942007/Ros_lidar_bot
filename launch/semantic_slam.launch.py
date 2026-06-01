#!/usr/bin/env python3
"""
Semantic SLAM launch — adds the object-detection node on top of the
existing simulation stack.

Usage:
  # Manual navigation + semantic mapping (drive yourself):
  ros2 launch Ros_lidar_bot semantic_slam.launch.py

  # Autonomous exploration + semantic mapping:
  ros2 launch Ros_lidar_bot semantic_slam.launch.py mode:=autonomous

  # With a different world:
  ros2 launch Ros_lidar_bot semantic_slam.launch.py world:=warehouse.world

The semantic_slam_node publishes /semantic_markers — add a MarkerArray
display in RViz2 pointing at that topic to see labelled 3D boxes on the map.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = "Ros_lidar_bot"
    pkg_share = get_package_share_directory(package_name)
    world = LaunchConfiguration("world")
    mode  = LaunchConfiguration("mode")   # "manual" or "autonomous"

    # ── Base simulation (Gazebo + SLAM + EKF + Nav2) ──────────────────────────
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "launch_sim.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    # ── Frontier explorer — only in autonomous mode ───────────────────────────
    frontier_explorer = Node(
        package=package_name,
        executable="frontier_explorer_node.py",
        name="frontier_explorer",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "frontier_explorer.yaml")],
        condition=IfCondition(
            # treat anything that starts with 'a' as autonomous
            LaunchConfiguration("mode")
        ),
    )

    # ── Semantic SLAM node ────────────────────────────────────────────────────
    semantic_slam = Node(
        package=package_name,
        executable="semantic_slam_node.py",
        name="semantic_slam",
        output="screen",
        parameters=[{
            "model_path": os.path.join(pkg_share, "Vision Model", "best.pt"),
            "detect_hz":  3.0,   # detections per second (lower = less CPU)
            "conf":       0.45,  # minimum YOLO confidence
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="testing.world",
            description="World file: testing.world / warehouse.world / office.world",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="manual",
            description="'manual' — drive yourself. 'autonomous' — frontier exploration.",
        ),
        sim,
        # Semantic detector starts when Nav2 is ready (same timing as frontier)
        TimerAction(period=32.0, actions=[semantic_slam]),
        # Frontier explorer starts only in autonomous mode
        TimerAction(period=32.0, actions=[frontier_explorer]),
    ])
