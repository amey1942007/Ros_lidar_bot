#!/bin/bash

# Zenoh setup for laptop (Humble) - run this once on each team member's laptop
# This configures ROS2 to connect to the Zenoh router on the RPi5

set -e

RPi5_TAILSCALE_IP="100.119.236.11"
ZENOH_ROUTER_PORT="7447"

echo "=== Zenoh Laptop Setup (ROS2 Humble) ==="
echo "Configuring to connect to RPi5 Zenoh router at $RPi5_TAILSCALE_IP:$ZENOH_ROUTER_PORT"

# Create ~/.ros/humble_zenoh_env.sh
mkdir -p ~/.ros

cat > ~/.ros/humble_zenoh_env.sh << 'EOF'
#!/bin/bash
# Source this before launching ROS2 nodes or rviz2

source /opt/ros/humble/setup.bash

export RMW_IMPLEMENTATION=rmw_zenoh_cpp

# Point to the RPi5 Zenoh router
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/100.119.236.11:7447"]'

echo "[Zenoh Config] Connected to RPi5 router at 100.119.236.11:7447"
EOF

chmod +x ~/.ros/humble_zenoh_env.sh

echo ""
echo "✓ Setup complete!"
echo ""
echo "Usage (before launching ROS2 nodes or rviz2):"
echo "  source ~/.ros/humble_zenoh_env.sh"
echo ""
echo "Or add to your ~/.bashrc or ~/.zshrc for automatic setup:"
echo "  echo 'source ~/.ros/humble_zenoh_env.sh' >> ~/.bashrc"
echo ""
echo "Then verify connection:"
echo "  source ~/.ros/humble_zenoh_env.sh"
echo "  ros2 topic list  # Should see RPi5 topics"
echo ""
