from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_lidar_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'description'),
            glob('description/*')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*')),
    ],
    install_requires=['setuptools', 'scipy'],
    zip_safe=True,
    maintainer='honey',
    maintainer_email='honeykumar1806@gmail.com',
    description='Robot description package',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'teleop = ros_lidar_bot.teleop_node:main',
            'odom_check = ros_lidar_bot.odom_check:main',
            'frontier_explorer = ros_lidar_bot.frontier_explorer:main',
        ],
    },
)
