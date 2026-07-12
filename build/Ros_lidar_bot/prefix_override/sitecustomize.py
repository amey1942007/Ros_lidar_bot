import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amey/Desktop/ros2_ws/src/Ros_lidar_bot/install/Ros_lidar_bot'
