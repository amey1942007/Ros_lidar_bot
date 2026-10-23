from setuptools import find_packages
from setuptools import setup

setup(
    name='Ros_lidar_bot',
    version='0.0.0',
    packages=find_packages(
        include=('Ros_lidar_bot', 'Ros_lidar_bot.*')),
)
