#!/usr/bin/env python3
"""
vision.launch.py — on-demand semantic vision pipeline.

  camera (OpenCV) → yolo (YOLO-World) → /yolo (String JSON)
  /yolo + /scan + TF map→base_link → semantic_slam → /semantic_markers

NOT part of the always-on bringup: started/stopped from the web dashboard's
"Semantic vision" toggle (or manually:
  ros2 launch Ros_lidar_bot vision.launch.py camera:=0 conf:=0.25).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="0",
                              description="OpenCV camera index"),
        DeclareLaunchArgument("conf", default_value="0.25",
                              description="YOLO confidence threshold"),
        # semantic_slam consumes detections at max 5 Hz (DETECT_HZ) — running
        # inference faster than that on the Pi is pure wasted CPU.
        DeclareLaunchArgument("rate", default_value="5.0",
                              description="Max YOLO inference FPS"),
        DeclareLaunchArgument("model", default_value="yolov8s-world.pt",
                              description="YOLO-World weights (auto-downloads)"),
        DeclareLaunchArgument("backend", default_value="auto",
                              description="capture backend: auto|picamera2|cv2 "
                                          "(RPi CSI cameras need picamera2)"),
        Node(
            package="Ros_lidar_bot",
            executable="yolo",
            name="yolo_world_publisher",
            output="screen",
            arguments=[
                "--camera", LaunchConfiguration("camera"),
                "--conf", LaunchConfiguration("conf"),
                "--rate", LaunchConfiguration("rate"),
                "--model", LaunchConfiguration("model"),
                "--backend", LaunchConfiguration("backend"),
            ],
        ),
        Node(
            package="Ros_lidar_bot",
            executable="semantic_slam_node",
            name="semantic_slam",
            output="screen",
        ),
    ])
