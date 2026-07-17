#!/usr/bin/env bash
# kill_program.bash — Clean up all ROS 2 nodes, SLAM, RViz, and release serial ports.

echo "===================================================="
echo "🔴 Starting ROS 2 and Serial Port Cleanup..."
echo "===================================================="

# 1. Kill ROS 2 Launch and Node Processes
echo "Stopping ROS 2 daemon..."
ros2 daemon stop &>/dev/null || true

echo "Killing ROS 2 nodes and python processes..."
# Kill specific executables
PID_LIST=$(pgrep -f "driver_node|imu_node|odom_node|lidar_node|safety_stop|ekf_node|slam_toolbox|robot_state_publisher|joint_state_publisher|rviz2|xterm|teleop|joy_node")
if [ -n "$PID_LIST" ]; then
    echo "Found running nodes: $PID_LIST"
    kill -9 $PID_LIST &>/dev/null || true
fi

# Kill any python processes running Ros_lidar_bot scripts
pkill -9 -f "Ros_lidar_bot" || true
pkill -9 -f "ros2 launch" || true
pkill -9 -f "rviz2" || true
pkill -9 -f "xterm" || true
pkill -9 -f "async_slam_toolbox" || true

# 2. Free up Serial Ports (/dev/ttyACM* and /dev/ttyUSB*)
echo "Releasing USB/Serial ports..."
for dev in /dev/ttyACM* /dev/ttyUSB*; do
    if [ -e "$dev" ]; then
        echo "Freeing $dev..."
        # Use fuser to kill any processes holding the port open
        sudo fuser -k -9 "$dev" &>/dev/null || true
    fi
done

# 3. Clear ROS 2/FastDDS shared memory and transport locks if any
ipcs -m | awk '{print $2}' | xargs -r -n1 ipcrm -m &>/dev/null || true

echo "===================================================="
echo "✅ Cleanup complete!"
echo "If you unplug and replug your Arduino/Lidar, they"
echo "should now map back to /dev/ttyACM0, ACM1, and ttyUSB0."
echo "===================================================="
