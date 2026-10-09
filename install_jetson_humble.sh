#!/usr/bin/env bash
# =============================================================================
# Ros_lidar_bot — Setup Script for NVIDIA Jetson (ROS 2 Humble)
# =============================================================================
# Usage:
#   chmod +x install_jetson_humble.sh
#   ./install_jetson_humble.sh
# =============================================================================

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

log()     { echo -e "${GREEN}[OK]${NC} $1"; }
info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
section() { echo -e "\n${BLUE}=====================================================\n  $1\n=====================================================${NC}"; }

section "1. System Updates & Base Dependencies"
sudo apt-get update -y && sudo apt-get upgrade -y
sudo apt-get install -y \
    curl gnupg2 lsb-release build-essential cmake git wget \
    v4l-utils net-tools python3-pip python3-dev python3-serial \
    python3-numpy python3-scipy python3-tornado python3-jinja2

section "2. Adding ROS 2 Humble Apt Repository"
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -y

section "3. ROS 2 Humble & Navigation Stack"
sudo apt-get install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-rplidar-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-joy

section "4. Setting Permissions & Jetson Power Mode"
sudo usermod -aG dialout,tty $USER
if command -v nvpmodel &> /dev/null; then sudo nvpmodel -m 0 || true; fi
if command -v jetson_clocks &> /dev/null; then sudo jetson_clocks || true; fi

log "ROS 2 Humble setup completed for Jetson!"
