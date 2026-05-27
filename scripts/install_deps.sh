#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Full dependency installer for Ros_lidar_bot on Ubuntu 22.04 + ROS2 Humble
# Run once on a fresh machine:
#   chmod +x scripts/install_deps.sh && bash scripts/install_deps.sh
# ─────────────────────────────────────────────────────────────────────────────
set -e   # stop on any error

# ── 0. Sanity checks ─────────────────────────────────────────────────────────
if [[ "$(lsb_release -rs)" != "22.04" ]]; then
  echo "ERROR: This script is for Ubuntu 22.04 only."
  echo "       For Jazzy (Ubuntu 24.04 / RPi5) use the jazzy branch."
  exit 1
fi
echo "Ubuntu 22.04 confirmed."

# ── 1. System basics ─────────────────────────────────────────────────────────
sudo apt update
sudo apt install -y \
  curl gnupg2 lsb-release software-properties-common \
  build-essential git \
  python3 python3-pip python3-setuptools python3-wheel

# ── 2. ROS2 Humble ───────────────────────────────────────────────────────────
if ! dpkg -s ros-humble-ros-base &>/dev/null; then
  echo "Installing ROS2 Humble..."

  # Add ROS2 GPG key
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

  # Add ROS2 apt source
  echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  sudo apt update
  sudo apt install -y ros-humble-desktop
else
  echo "ROS2 Humble already installed — skipping."
fi

# ── 3. Colcon + rosdep (build tools) ─────────────────────────────────────────
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update

# ── 4. Gazebo Fortress + ROS-Gazebo bridge ────────────────────────────────────
# ros-humble-ros-gz-* pulls in Ignition Fortress automatically
if ! dpkg -s ros-humble-ros-gz-bridge &>/dev/null; then
  echo "Installing Gazebo Fortress + ROS bridge..."
  sudo apt install -y \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-interfaces
else
  echo "Gazebo bridge already installed — skipping."
fi

# ── 5. Navigation stack (Nav2) ────────────────────────────────────────────────
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-map-server

# ── 6. SLAM Toolbox ───────────────────────────────────────────────────────────
sudo apt install -y ros-humble-slam-toolbox

# ── 7. EKF (robot_localization) ───────────────────────────────────────────────
sudo apt install -y ros-humble-robot-localization

# ── 8. Robot description tools ───────────────────────────────────────────────
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher

# ── 9. RViz2 and diagnostics ─────────────────────────────────────────────────
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-rqt-image-view \
  ros-humble-tf2-tools

# ── 10. Python packages ───────────────────────────────────────────────────────
pip3 install -r "$(dirname "$0")/../requirements.txt"

# ── 11. Source ROS2 in ~/.bashrc if not already there ────────────────────────
SETUP_LINE="source /opt/ros/humble/setup.bash"
if ! grep -qF "$SETUP_LINE" ~/.bashrc; then
  echo "" >> ~/.bashrc
  echo "# ROS2 Humble" >> ~/.bashrc
  echo "$SETUP_LINE" >> ~/.bashrc
  echo "Added ROS2 source to ~/.bashrc"
fi

# ── 12. Done ─────────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo " All dependencies installed."
echo ""
echo " Next steps:"
echo "   1. Close and reopen your terminal  (or: source ~/.bashrc)"
echo "   2. cd ~/ros2_ws  (or wherever you cloned the repo)"
echo "   3. colcon build --packages-select Ros_lidar_bot --symlink-install"
echo "   4. source install/setup.bash"
echo "   5. ros2 launch Ros_lidar_bot launch_sim.launch.py"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
