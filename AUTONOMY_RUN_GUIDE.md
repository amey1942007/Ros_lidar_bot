# Autonomous Navigation + Frontier SLAM Guide

This package now supports **one unified launch file** for:
- Gazebo simulation
- Robot spawn + bridges
- SLAM Toolbox
- Nav2
- Explore Lite frontier exploration

## 1) Dependencies

Use ROS 2 Humble and install the standard dependencies:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge \
  ros-humble-xacro
```

### Explore Lite install

`explore_lite` may not always be available as an apt package in every setup.
If needed, clone and build it in your workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/robo-friends/m-explore-ros2.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 2) Build this package

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 3) Launch modes

## A) Simulation + SLAM only (manual teleop mapping)

```bash
ros2 launch Ros_lidar_bot launch_sim.launch.py world:=maze_world slam:=true nav2:=false explore:=false
```

## B) Full autonomous frontier exploration

```bash
ros2 launch Ros_lidar_bot launch_sim.launch.py world:=maze_world slam:=true nav2:=true explore:=true
```

This mode will:
1. Start Gazebo + robot + bridges.
2. Start Nav2 in **SLAM mode**.
3. Start `explore_lite` after a short delay (`explore_delay`, default `8.0s`).

## 4) Useful launch arguments

- `world:=<name>` choose world file in `worlds/`.
- `slam:=true|false` enable SLAM mode.
- `nav2:=true|false` enable Nav2 bringup.
- `explore:=true|false` start frontier exploration (works when `nav2:=true`).
- `explore_delay:=8.0` delay before explorer startup.
- `nav2_params_file:=<path>` custom Nav2 params YAML.
- `explore_params_file:=<path>` custom explore_lite params YAML.

## 5) Common troubleshooting

- If robot does not move, check `/cmd_vel` and Nav2 lifecycle status.
- If mapping drifts, reduce speed in Nav2 and ensure stable `/odom` timestamps.
- If exploration never starts, verify `explore:=true nav2:=true` and that `/global_costmap/costmap` exists.
- If `explore_lite` is missing, install from source as shown above.
