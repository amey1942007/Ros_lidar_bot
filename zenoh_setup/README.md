# Zenoh Network Setup for ROS2 Multi-Laptop Development

This setup allows 5-6 team members on Ubuntu laptops (ROS2 Humble) to subscribe to ROS2 topics running on an RPi5 (ROS2 Jazzy) over Tailscale, using Zenoh as the middleware.

## Architecture

```
RPi5 (Jazzy)
  ├─ Zenoh Router (hub)
  └─ Your ROS2 Launch File (publishes topics)
       ↓ TCP via Tailscale IP 100.119.236.11:7447
Laptop 1 (Humble) ─┐
Laptop 2 (Humble) ─┼─ All connect to Zenoh router
Laptop 3 (Humble) ─┤ Each sees same ROS2 topics
...               ─┘
```

## Setup Steps

### 1. RPi5 Setup (run once)

SSH into RPi5:
```bash
ssh pi@<rpi5-ip>
```

Navigate to your workspace and run the startup script:
```bash
cd ~/ros2_ws/src/Ros_lidar_bot/zenoh_setup
chmod +x rpi5_startup.sh
./rpi5_startup.sh
```

**Important:** Edit `rpi5_startup.sh` and replace `main.launch.py` with your actual launch file name before running.

The script will:
- Source ROS2 Jazzy
- Start the Zenoh router (listening on 0.0.0.0:7447)
- Launch your ROS2 nodes
- Keep running until you Ctrl+C

### 2. Laptop Setup (run once on each team member's laptop)

On each Ubuntu laptop with Humble:

```bash
cd ~/ros2_ws/src/Ros_lidar_bot/zenoh_setup
chmod +x laptop_setup.sh
./laptop_setup.sh
```

This creates `~/.ros/humble_zenoh_env.sh` which each team member needs to source.

### 3. Team Member Usage (daily)

Each time you want to use rviz2 or interact with RPi5 topics:

```bash
source ~/.ros/humble_zenoh_env.sh
ros2 topic list        # See RPi5 topics
rviz2                  # Full visualization
# or any other ros2 command
```

Or add to `~/.bashrc` or `~/.zshrc` for automatic setup:
```bash
echo 'source ~/.ros/humble_zenoh_env.sh' >> ~/.bashrc
source ~/.bashrc
```

## Troubleshooting

### Topics not showing up?
```bash
# Verify connection to router
source ~/.ros/humble_zenoh_env.sh
ros2 doctor
```

Check that:
- RPi5 Zenoh router is running (`./rpi5_startup.sh` still executing)
- Tailscale is connected on both RPi5 and laptop
- Firewall allows port 7447

### Check Zenoh router status on RPi5:
```bash
ps aux | grep rmw_zenohd
# Should show the router process running
```

### Enable debug logging:
On laptop or RPi5, before sourcing setup:
```bash
export RUST_LOG=debug
source ~/.ros/humble_zenoh_env.sh
```

## Notes

- **Cross-distro:** RPi5 runs Jazzy, laptops run Humble. Zenoh handles the wire-protocol differences transparently.
- **Scalable:** Works for 2 laptops or 10; just add more team members pointing to the same router IP.
- **No multicast needed:** Works fine over Tailscale; no local network broadcast required.
- **Network overhead:** Minimal; Zenoh is designed for low-latency pub/sub.
