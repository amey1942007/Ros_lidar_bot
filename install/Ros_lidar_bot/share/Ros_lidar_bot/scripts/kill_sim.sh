#!/bin/bash
# Run this before every relaunch: ros2_ws/src/Ros_lidar_bot/scripts/kill_sim.sh
# Kills all simulation-related processes cleanly.

echo "Stopping simulation processes..."

pkill -SIGTERM -f "slam_toolbox"    2>/dev/null
pkill -SIGTERM -f "ekf_node"        2>/dev/null
pkill -SIGTERM -f "frontier_explorer" 2>/dev/null
pkill -SIGTERM -f "nav2"            2>/dev/null
pkill -SIGTERM -f "bt_navigator"    2>/dev/null
pkill -SIGTERM -f "controller_server" 2>/dev/null
pkill -SIGTERM -f "planner_server"  2>/dev/null
pkill -SIGTERM -f "behavior_server" 2>/dev/null
pkill -SIGTERM -f "robot_state_publisher" 2>/dev/null
pkill -SIGTERM -f "parameter_bridge" 2>/dev/null

sleep 2

# Force-kill anything still alive
pkill -SIGKILL -f "ign gazebo"      2>/dev/null
pkill -SIGKILL -f "gz_sim"          2>/dev/null
pkill -SIGKILL -f "slam_toolbox"    2>/dev/null
pkill -SIGKILL -f "nav2"            2>/dev/null
pkill -SIGKILL -f "ekf_node"        2>/dev/null
pkill -SIGKILL -f "frontier_explorer" 2>/dev/null

sleep 1

# Clean up slam_toolbox serialized state so next run starts completely fresh
rm -f /tmp/slam_toolbox*
rm -f ~/.ros/slam_toolbox*
rm -f ~/slam_toolbox*

echo "Done. Safe to relaunch."
