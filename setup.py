from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Ros_lidar_bot'


def _amr_bridge_data_files():
    """Install vendored amr-bridge into share so launch can set PYTHONPATH."""
    entries = []
    root = 'amr-bridge'
    if not os.path.isdir(root):
        return entries
    for dirpath, _dirnames, filenames in os.walk(root):
        # Skip local venvs / caches if present
        parts = set(dirpath.split(os.sep))
        if '.venv' in parts or '__pycache__' in parts or '.git' in parts:
            continue
        files = [
            os.path.join(dirpath, f)
            for f in filenames
            if not f.endswith('.pyc') and f != '.env.pi'
        ]
        if files:
            entries.append((os.path.join('share', package_name, dirpath), files))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/description', glob('description/*.xacro')),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        *_amr_bridge_data_files(),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amey Chaudhari',
    maintainer_email='ameychaudhar19@gmail.com',
    description='Robot description package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'imu_node = Ros_lidar_bot.imu_node:main',
            'driver_node = Ros_lidar_bot.driver_node:main',
            'driver_control = Ros_lidar_bot.driver_control:main',
            'odom_node = Ros_lidar_bot.odom_node:main',
            'safety_stop_node = Ros_lidar_bot.safety_stop_node:main',
            'semantic_slam_node = Ros_lidar_bot.semantic_slam_node:main',
            'frontier_explorer_node = Ros_lidar_bot.frontier_explorer_node:main',
            'yolo = Ros_lidar_bot.yolo:main',
            'joy_teleop = Ros_lidar_bot.joy_teleop_node:main',
            'camera_servo = Ros_lidar_bot.camera_servo_node:main',
            'check_qos_mismatch = Ros_lidar_bot.check_qos_mismatch:main',
            'imu_test_node = Ros_lidar_bot.imu_test_node:main',
            'imu_calibration_node = Ros_lidar_bot.imu_calibration_node:main',
            'drive_distance = Ros_lidar_bot.drive_distance_node:main',
            'robot_dashboard = Ros_lidar_bot.robot_dashboard_node:main',
            'scan_min_range_filter = Ros_lidar_bot.scan_min_range_filter_node:main',
            'teleop_interface = Ros_lidar_bot.teleop_interface_node:main',
            'web_gamepad = Ros_lidar_bot.web_gamepad_node:main',
        ],
    },
    extras_require={
        'test': ['pytest'],
    },
)
