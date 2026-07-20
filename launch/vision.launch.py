#!/usr/bin/env python3
"""
vision.launch.py — on-demand semantic vision pipeline.

  camera (OpenCV) → yolo (YOLO-World) → /yolo (String JSON)
  /yolo + /scan + TF map→base_link → semantic_slam → /semantic_markers

NOT part of the always-on bringup: started/stopped from the web dashboard's
"Semantic vision" toggle (or manually:
  ros2 launch Ros_lidar_bot vision.launch.py camera:=0 conf:=0.25).
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Same cache path yolo.py defaults to -- kept identical so a manual `ros2 run`
# and a dashboard-launched vision.launch.py always share one downloaded copy.
DEFAULT_MODEL = os.path.expanduser("~/.cache/Ros_lidar_bot/yolov8s-world.pt")


def generate_launch_description():
    yolo = Node(
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
    )
    semantic = Node(
        package="Ros_lidar_bot",
        executable="semantic_slam_node",
        name="semantic_slam",
        output="screen",
    )
    # If the camera node dies (busy device, crash), tear down the whole
    # vision launch so the dashboard clears VISION ON instead of sitting
    # on a half-dead pipeline with stale markers and a blank feed.
    shutdown_on_yolo_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=yolo,
            on_exit=[EmitEvent(event=Shutdown(reason="yolo exited"))],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("camera", default_value="0",
                              description="camera index (Picamera2 camera_num or OpenCV index)"),
        DeclareLaunchArgument("conf", default_value="0.25",
                              description="YOLO confidence threshold"),
        # semantic_slam consumes detections at max 5 Hz (DETECT_HZ) — running
        # inference faster than that on the Pi is pure wasted CPU.
        DeclareLaunchArgument("rate", default_value="5.0",
                              description="Max YOLO inference FPS"),
        DeclareLaunchArgument("model", default_value=DEFAULT_MODEL,
                              description="YOLO-World weights path (auto-downloads on first run, cached after)"),
        DeclareLaunchArgument("backend", default_value="auto",
                              description="capture backend: auto|picamera2|cv2 "
                                          "(RPi CSI cameras need picamera2)"),
        yolo,
        semantic,
        shutdown_on_yolo_exit,
    ])
