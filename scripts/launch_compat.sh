#!/bin/bash
# launch_compat.sh — use this on machines where the GPU lidar shows no scan data.
#
# Problem: gpu_lidar in Ignition Gazebo requires ogre2 GPU rendering.
# On some machines (hybrid Intel/Nvidia laptops, systems without proper GPU
# drivers) ogre2 fails silently — the lidar transmits rays but detects nothing,
# so /scan is all max-range readings and SLAM never builds a map.
#
# Fix: LIBGL_ALWAYS_SOFTWARE=1 forces Mesa software (llvmpipe) rendering.
# This makes ogre2 work on ANY machine regardless of GPU drivers.
# It is slower than hardware rendering but perfectly fine for simulation.
#
# Usage:
#   bash scripts/launch_compat.sh                          # default testing.world
#   bash scripts/launch_compat.sh autonomous               # autonomous exploration
#   bash scripts/launch_compat.sh sim warehouse.world      # warehouse, manual mode
#   bash scripts/launch_compat.sh autonomous office.world  # office, autonomous

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source the workspace if not already sourced
if [ -z "$AMENT_PREFIX_PATH" ]; then
    # Try the standard ros2_ws layout first, then direct clone layout
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        source "$WORKSPACE_ROOT/install/setup.bash"
    elif [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
        source "$HOME/ros2_ws/install/setup.bash"
    else
        echo "ERROR: Could not find install/setup.bash. Run 'colcon build' first."
        exit 1
    fi
fi

# Force Mesa software rendering — makes ogre2 work without hardware GPU
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe

MODE="${1:-sim}"
WORLD="${2:-testing.world}"

echo "Launching with software rendering (LIBGL_ALWAYS_SOFTWARE=1)"
echo "Mode: $MODE  |  World: $WORLD"
echo ""

if [ "$MODE" = "autonomous" ]; then
    exec ros2 launch Ros_lidar_bot autonomous_exploration.launch.py world:="$WORLD"
else
    exec ros2 launch Ros_lidar_bot launch_sim.launch.py world:="$WORLD"
fi
