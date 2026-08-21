#!/bin/bash

# Zenoh startup for RPi5 (Jazzy) - run this to start the Zenoh router + your launch file
# This starts the Zenoh hub that all team laptops connect to

set -e

echo "=== Zenoh Router Startup (RPi5 - Jazzy) ==="

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Enable Zenoh middleware
export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Suppress Zenoh debug output (optional, set to DEBUG for troubleshooting)
export RUST_LOG=info

# Start Zenoh router in background
echo "[1/2] Starting Zenoh router on 0.0.0.0:7447..."
ros2 run rmw_zenoh_cpp rmw_zenohd &
ROUTER_PID=$!
echo "      Router PID: $ROUTER_PID"

# Give router time to start
sleep 2

# Start your main launch file
echo "[2/2] Starting main launch file..."
ros2 launch Ros_lidar_bot launch_robot.launch.py

# Cleanup on exit
trap "kill $ROUTER_PID 2>/dev/null || true" EXIT

wait $ROUTER_PID
