#!/usr/bin/env bash
# =============================================================================
#  ROS 2 Jazzy + Ros_lidar_bot Full Installation Script
#  Target: Raspberry Pi 5 running Ubuntu 24.04 (Noble)
#  Author: Generated for Ros_lidar_bot project
#
#  Usage:
#    chmod +x install_rpi5_jazzy.sh
#    ./install_rpi5_jazzy.sh
#
#  What this script installs:
#    1. System prerequisites & locale
#    2. ROS 2 Jazzy (ros-base + key packages)
#    3. Nav2, SLAM Toolbox, robot_localization
#    4. ros2_control + diff_drive_controller
#    5. Python pip dependencies (pyrplidar, adafruit BNO055, pyserial, opencv)
#    6. Hardware peripheral permissions (I2C, UART, GPIO, dialout)
#    7. colcon + rosdep
#    8. Workspace build
# =============================================================================

set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

log()   { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }
section() { echo -e "\n${CYAN}============================================================${NC}"; \
            echo -e "${CYAN}  $*${NC}"; \
            echo -e "${CYAN}============================================================${NC}"; }

# ---------------------------------------------------------------------------
# 0. Sanity checks
# ---------------------------------------------------------------------------
section "0. Pre-flight checks"

# Must be run as a normal user (not root), but with sudo access
if [[ "$EUID" -eq 0 ]]; then
    error "Do NOT run as root. Run as your normal user (sudo access required)."
fi

# Verify Ubuntu 24.04
if ! grep -q "24.04" /etc/os-release 2>/dev/null; then
    warn "This script targets Ubuntu 24.04 (Noble). Detected OS:"
    cat /etc/os-release | grep PRETTY_NAME
    read -rp "Continue anyway? [y/N] " yn
    [[ "$yn" =~ ^[Yy]$ ]] || exit 1
fi

# Check internet connectivity
if ! ping -c 1 packages.ros.org &>/dev/null && ! ping -c 1 8.8.8.8 &>/dev/null; then
    error "No internet connection detected. Please connect and retry."
fi

log "Running as: $USER | OS: $(grep PRETTY_NAME /etc/os-release | cut -d= -f2 | tr -d '\"')"

# ---------------------------------------------------------------------------
# 1. Locale & system update
# ---------------------------------------------------------------------------
section "1. Locale & System Update"

sudo apt-get update -y
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
log "Locale configured."

sudo apt-get upgrade -y
log "System packages upgraded."

# ---------------------------------------------------------------------------
# 2. ROS 2 Jazzy repository setup
# ---------------------------------------------------------------------------
section "2. ROS 2 Jazzy Repository"

sudo apt-get install -y curl gnupg2 lsb-release software-properties-common

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt-get update -y
log "ROS 2 repository added."

# ---------------------------------------------------------------------------
# 3. ROS 2 Jazzy Core Installation
# ---------------------------------------------------------------------------
section "3. ROS 2 Jazzy Core"

# ros-base (no GUI tools, smaller footprint for RPi5)
sudo apt-get install -y ros-jazzy-ros-base

# Essential ROS tools
sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    python3-pip \
    python3-setuptools \
    python3-vcstool \
    ros-dev-tools

log "ROS 2 Jazzy core installed."

# ---------------------------------------------------------------------------
# 4. Navigation2 (Nav2) Stack
# ---------------------------------------------------------------------------
section "4. Nav2 Navigation Stack"

sudo apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-msgs \
    ros-jazzy-nav2-core \
    ros-jazzy-nav2-costmap-2d \
    ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner \
    ros-jazzy-nav2-behaviors \
    ros-jazzy-nav2-bt-navigator \
    ros-jazzy-nav2-lifecycle-manager \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-amcl \
    ros-jazzy-nav2-navfn-planner \
    ros-jazzy-nav2-regulated-pure-pursuit-controller \
    ros-jazzy-nav2-velocity-smoother \
    ros-jazzy-behaviortree-cpp-v3

log "Nav2 installed."

# ---------------------------------------------------------------------------
# 5. SLAM Toolbox
# ---------------------------------------------------------------------------
section "5. SLAM Toolbox"

sudo apt-get install -y \
    ros-jazzy-slam-toolbox

log "SLAM Toolbox installed."

# ---------------------------------------------------------------------------
# 6. Robot Localization (EKF)
# ---------------------------------------------------------------------------
section "6. Robot Localization (EKF)"

sudo apt-get install -y \
    ros-jazzy-robot-localization

log "robot_localization installed."

# ---------------------------------------------------------------------------
# 7. Robot State Publisher, TF2, xacro
# ---------------------------------------------------------------------------
section "7. Robot Description & TF2"

sudo apt-get install -y \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro \
    ros-jazzy-tf2 \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-tf2-geometry-msgs

log "TF2 and description packages installed."

# ---------------------------------------------------------------------------
# 8. ros2_control (for diff_drive_controller)
# ---------------------------------------------------------------------------
section "8. ros2_control"

sudo apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-diff-drive-controller \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-controller-manager

log "ros2_control installed."

# ---------------------------------------------------------------------------
# 9. Additional ROS packages
# ---------------------------------------------------------------------------
section "9. Additional ROS Packages"

sudo apt-get install -y \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-action-msgs \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-rviz2 \
    ros-jazzy-rqt \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-tf-tree \
    ros-jazzy-teleop-twist-keyboard

log "Additional ROS packages installed."

# ---------------------------------------------------------------------------
# 10. Python dependencies for Ros_lidar_bot nodes
# ---------------------------------------------------------------------------
section "10. Python Dependencies"

# System Python packages first
sudo apt-get install -y \
    python3-numpy \
    python3-opencv \
    python3-serial

# pip packages (use --break-system-packages for Ubuntu 24.04)
PIP_FLAGS="--break-system-packages"

log "Installing RPLidar driver (pyrplidar – supports Standard and Express/Sensitivity scan modes)..."
# The lidar_node imports 'from pyrplidar import PyRPlidar'. pyrplidar provides full RPLidar SDK
# support including Express/Sensitivity scan mode (mode 1) required for higher point density.
pip3 install $PIP_FLAGS pyrplidar

log "Installing pyserial (UART driver)..."
pip3 install $PIP_FLAGS pyserial

log "Installing OpenCV extras..."
pip3 install $PIP_FLAGS \
    opencv-python-headless \
    scikit-image \
    ultralytics

log "Python dependencies installed."

# ---------------------------------------------------------------------------
# 11. RPi5 Hardware Permissions & Peripheral Setup
# ---------------------------------------------------------------------------
section "11. RPi5 Hardware Permissions"

# Add user to required groups
log "Adding $USER to hardware groups..."
sudo usermod -a -G dialout,video "$USER"


# udev rules for serial/USB devices (RPLidar, DDSM115)
UDEV_RULES="/etc/udev/rules.d/99-ros-hardware.rules"
log "Creating udev rules at $UDEV_RULES..."
sudo tee "$UDEV_RULES" > /dev/null << 'UDEV_EOF'
# RPLidar A1 (CP210x USB-Serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", \
    SYMLINK+="rplidar", MODE="0666", GROUP="dialout"

# DDSM115 Motor Driver (FTDI or CH340 USB-Serial - adjust vendor/product as needed)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", \
    SYMLINK+="ddsm115", MODE="0666", GROUP="dialout"

# CH340 Serial (common on many Arduino/driver boards)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    SYMLINK+="ttyUSB_CH340", MODE="0666", GROUP="dialout"
UDEV_EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
log "udev rules applied."

# ---------------------------------------------------------------------------
# 12. UART Configuration (disable serial console, enable serial port)
# ---------------------------------------------------------------------------
section "12. UART Serial Port Configuration"

warn "UART configuration requires manual step on RPi5:"
echo ""
echo "  Run: sudo raspi-config"
echo "  Navigate: Interface Options → Serial Port"
echo "    → Login shell over serial: NO"
echo "    → Serial port hardware enabled: YES"
echo ""
echo "  Or run these commands to configure via cmdline.txt:"

# Disable console on serial0
if [[ -f "/boot/firmware/cmdline.txt" ]]; then
    CMDLINE=$(cat /boot/firmware/cmdline.txt)
    if echo "$CMDLINE" | grep -q "console=serial0"; then
        log "Removing serial console from cmdline.txt..."
        sudo sed -i 's/console=serial0,[0-9]* //g' /boot/firmware/cmdline.txt
        log "Serial console disabled. Hardware UART now free for DDSM115."
    else
        log "Serial console already not in cmdline.txt."
    fi
fi

# Enable UART in firmware config
if [[ -f "$FIRMWARE_CONFIG" ]]; then
    if ! grep -q "enable_uart=1" "$FIRMWARE_CONFIG"; then
        echo "enable_uart=1" | sudo tee -a "$FIRMWARE_CONFIG"
        log "UART enabled in firmware config."
    fi
fi

# ---------------------------------------------------------------------------
# 13. rosdep initialization
# ---------------------------------------------------------------------------
section "13. rosdep Setup"

if [[ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]]; then
    sudo rosdep init
fi
rosdep update
log "rosdep initialized and updated."

# ---------------------------------------------------------------------------
# 14. Shell environment setup
# ---------------------------------------------------------------------------
section "14. Shell Environment"

BASHRC="$HOME/.bashrc"
ROS_SOURCE="source /opt/ros/jazzy/setup.bash"
WS_SOURCE="source $HOME/Desktop/ros2_ws/install/setup.bash"
DOMAIN_ID="export ROS_DOMAIN_ID=0"
LOCALHOST="export ROS_LOCALHOST_ONLY=1"

add_if_missing() {
    local line="$1"
    local file="$2"
    if ! grep -qF "$line" "$file"; then
        echo "$line" >> "$file"
        log "Added to $file: $line"
    fi
}

add_if_missing "$ROS_SOURCE" "$BASHRC"
add_if_missing "# Ros_lidar_bot workspace" "$BASHRC"
add_if_missing "$WS_SOURCE" "$BASHRC"
add_if_missing "$DOMAIN_ID" "$BASHRC"
add_if_missing "$LOCALHOST" "$BASHRC"
add_if_missing "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" "$BASHRC"

log "ROS 2 environment added to ~/.bashrc"

# ---------------------------------------------------------------------------
# 15. Build the workspace
# ---------------------------------------------------------------------------
section "15. Building Ros_lidar_bot Workspace"

WS_DIR="$HOME/Desktop/ros2_ws"

if [[ ! -d "$WS_DIR/src/Ros_lidar_bot" ]]; then
    warn "Workspace not found at $WS_DIR. Skipping build."
else
    cd "$WS_DIR"
    source /opt/ros/jazzy/setup.bash

    log "Running rosdep install..."
    rosdep install --from-paths src --ignore-src -r -y || warn "Some rosdep deps may have failed (check manually)."

    log "Building with colcon..."
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        2>&1 | tee /tmp/colcon_build.log

    if [[ ${PIPESTATUS[0]} -eq 0 ]]; then
        log "Workspace built successfully!"
    else
        warn "Build had errors. Check /tmp/colcon_build.log"
    fi
fi

# ---------------------------------------------------------------------------
# 16. Optional: Real-time kernel (for lower latency)
# ---------------------------------------------------------------------------
section "16. Optional: Real-Time Kernel"

echo ""
echo "  For lower motor control latency, optionally install the RT kernel:"
echo "  sudo apt install linux-image-rt-rpi-2712"
echo "  (Reboot required. Only needed if experiencing control jitter.)"
echo ""

# ---------------------------------------------------------------------------
# 17. Post-install verification
# ---------------------------------------------------------------------------
section "17. Post-Install Verification"

log "Checking ROS 2 installation..."
source /opt/ros/jazzy/setup.bash

echo ""
echo "--- ROS 2 Version ---"
ros2 --version || warn "ros2 CLI not found in PATH"

echo ""
echo "--- Key Packages ---"
PACKAGES=(
    "ros-jazzy-navigation2"
    "ros-jazzy-slam-toolbox"
    "ros-jazzy-robot-localization"
    "ros-jazzy-ros2-control"
    "ros-jazzy-diff-drive-controller"
    "ros-jazzy-cv-bridge"
    "ros-jazzy-xacro"
)
for pkg in "${PACKAGES[@]}"; do
    if dpkg -l "$pkg" 2>/dev/null | grep -q "^ii"; then
        echo -e "  ${GREEN}✓${NC} $pkg"
    else
        echo -e "  ${RED}✗${NC} $pkg (NOT INSTALLED)"
    fi
done

echo ""
echo "--- Python Packages ---"
# Note: pyrplidar is imported as 'pyrplidar'
PY_PACKAGES=("pyrplidar" "serial" "cv2" "numpy" "ultralytics")
for pkg in "${PY_PACKAGES[@]}"; do
    if python3 -c "import $pkg" 2>/dev/null; then
        echo -e "  ${GREEN}✓${NC} python3: $pkg"
    else
        echo -e "  ${YELLOW}✗${NC} python3: $pkg (NOT installed — run: pip3 install $pkg --break-system-packages)"
    fi
done

echo ""
# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
section "Installation Complete!"

echo ""
echo -e "${GREEN}Next steps:${NC}"
echo ""
echo "  1. REBOOT the RPi5 for all hardware changes to take effect:"
echo "       sudo reboot"
echo ""
echo "  2. After reboot, source your workspace:"
echo "       source ~/Desktop/ros2_ws/install/setup.bash"
echo ""
echo "  3. Verify UART port (for DDSM115 motor driver):"
echo "       ls -la /dev/ttyAMA* /dev/ttyUSB*"
echo ""
echo "  4. Verify RPLidar port:"
echo "       ls -la /dev/rplidar  (if udev rule matched)"
echo "       ls -la /dev/ttyUSB0  (fallback)"
echo ""
echo "  5. Launch the robot:"
echo "       ros2 launch Ros_lidar_bot launch_robot.launch.py"
echo ""
echo "  6. In another terminal, start autonomous navigation:"
echo "       ros2 launch Ros_lidar_bot autonomous_robot.launch.py"
echo ""
echo -e "${YELLOW}Important:${NC} driver_node.py uses /dev/ttyAMA0 or /dev/ttyUSB0."
echo "  Update the port in driver_node.py if your DDSM115 appears at a different path."
echo ""
