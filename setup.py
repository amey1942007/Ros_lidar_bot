from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'Ros_lidar_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/description', glob('description/*.xacro')),
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
            'lidar_node = Ros_lidar_bot.lidar_node:main',
            'safety_stop_node = Ros_lidar_bot.safety_stop_node:main',
            'semantic_slam_node = Ros_lidar_bot.semantic_slam_node:main',
            'frontier_explorer_node = Ros_lidar_bot.frontier_explorer_node:main',
            'yolo = Ros_lidar_bot.yolo:main',
            'teleop_node = Ros_lidar_bot.teleop_node:main',
            'non_safety_teleop = Ros_lidar_bot.non_safety_teleop_node:main',
            'check_qos_mismatch = Ros_lidar_bot.check_qos_mismatch:main',
            'imu_test_node = Ros_lidar_bot.imu_test_node:main',
            'imu_calibration_node = Ros_lidar_bot.imu_calibration_node:main',
        ],
    },
    extras_require={
        'test': ['pytest'],
    },
)
