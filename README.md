# Ros_lidar_bot

A ROS 2 differential-drive robot with LiDAR, IMU, and camera sensors running in Ignition Gazebo. Supports real-time SLAM mapping, EKF-fused odometry, Nav2-based autonomous navigation, and frontier-driven autonomous exploration of unknown environments.

---

## Table of Contents

- [About](#about)
- [System Architecture](#system-architecture)
- [Robot Hardware](#robot-hardware)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [ROS 2 Topics](#ros-2-topics)
- [Configuration](#configuration)
- [Progress](#progress)
- [Common Issues](#common-issues)

---

## About

The robot is a custom differential-drive platform modelled from real hardware. The full simulation stack covers the complete sensor-to-autonomous-navigation pipeline:

- **Ignition Gazebo (Fortress)** physics simulation with a furnished 10×10 m indoor world
- **SLAM Toolbox** for real-time occupancy-grid mapping with loop closure
- **robot_localization EKF** that fuses wheel odometry and IMU into a clean `/odom`
- **Nav2** for global path planning, local trajectory following, and recovery behaviours
- **Frontier exploration** for fully autonomous map coverage without any pre-defined waypoints
- **Keyboard teleoperation** for manual driving

---

## System Architecture

```
Ignition Gazebo
  │
  ├─ DiffDrive plugin  ──→  /odom  ──→  [gz bridge]  ──→  /odom_raw
  ├─ GPU LiDAR         ──→  /scan  ──→  [gz bridge]  ──→  /scan
  ├─ IMU sensor        ──→  /imu   ──→  [gz bridge]  ──→  /imu
  ├─ Camera            ──→  /camera/image_raw  ──→  [gz bridge]
  └─ JointStatePublisher ─→ /joint_states ──→ [gz bridge]

[robot_state_publisher]  ←─ robot_description (URDF/Xacro)
    └─ TF: base_footprint → base_link → laser_frame / camera_link / imu / wheels

[ekf_filter_node]  ←─ /odom_raw + /imu
    └─ TF: odom → base_footprint
    └─ publishes: /odom (filtered)

[slam_toolbox]  ←─ /scan + TF(odom→base_footprint)
    └─ TF: map → odom
    └─ publishes: /map (OccupancyGrid, updated every 3 s)

[nav2 stack]  ←─ /map + /scan + /odom + TF tree
    ├─ planner_server   — NavFn A* global planner
    ├─ controller_server — Regulated Pure Pursuit local controller
    ├─ behavior_server  — spin / backup / wait recovery behaviours
    └─ bt_navigator     — navigate_to_pose action server → /cmd_vel

[frontier_explorer]  ←─ /map + TF(map→base_footprint)
    └─ sends goals to: navigate_to_pose action server
    └─ drives the robot toward unexplored frontiers until the map is complete
```

**Staged startup** (in `launch_sim.launch.py`):

| Time | What starts |
|---|---|
| T = 0 s | Gazebo, RSP, bridge |
| T = 3 s | Robot spawned into world |
| T = 6 s | EKF + SLAM Toolbox |
| T = 16 s | Nav2 stack |
| T = 30 s | Frontier explorer *(autonomous launch only)* |

---

## Robot Hardware

| Component | Specification |
|---|---|
| Base | Custom differential-drive chassis |
| Drive wheels | 2 × motorised, separation 339.5 mm, radius 81 mm |
| Casters | 2 × passive ball casters |
| LiDAR | 360° GPU LiDAR, range 0.3–12 m, 20 Hz, 360 samples/rev |
| IMU | 6-DOF IMU, 50 Hz |
| Camera | RGB 640×480, 30 Hz, 62° FOV, range 0.05–8 m |
| Footprint | ~500 mm × 360 mm |

---

## Project Structure

```
Ros_lidar_bot/
├── CMakeLists.txt
├── package.xml
├── setup.py
│
├── Ros_lidar_bot/
│   ├── frontier_explorer_node.py   # Autonomous frontier exploration node
│   └── teleop_node.py              # Keyboard teleoperation node
│
├── config/
│   ├── ekf.yaml                         # EKF sensor fusion parameters
│   ├── frontier_explorer.yaml           # Frontier explorer tuning
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox (mapping mode)
│   ├── mapper_params_localization.yaml  # SLAM Toolbox (localisation mode)
│   ├── mapper_params_lifelong.yaml
│   ├── mapper_params_offline.yaml
│   ├── mapper_params_online_sync.yaml
│   ├── nav2_params.yaml                 # Nav2 stack parameters
│   └── view_bot.rviz                    # RViz2 configuration
│
├── description/
│   ├── robot.urdf.xacro        # Top-level URDF entry point
│   ├── slam.xacro              # Base link, wheels, casters (geometry + inertials)
│   ├── slam.gazebo             # Gazebo material and friction properties
│   ├── robot_control.xacro    # DiffDrive + JointState Gazebo plugins
│   ├── lidar.xacro             # GPU LiDAR sensor
│   ├── camera.xacro            # RGB camera sensor
│   └── imu.xacro               # IMU sensor
│
├── launch/
│   ├── autonomous_exploration.launch.py  # Full auto: sim + nav2 + frontier explorer
│   ├── launch_sim.launch.py              # Sim + SLAM + EKF + Nav2 (manual / telop)
│   └── rsp.launch.py                     # Robot State Publisher only
│
├── meshes/
│   ├── base_link.stl
│   ├── wheel_urdf_1.stl
│   ├── wheel_urdf__1__1.stl
│   ├── ball_caster_wheel_1.stl
│   ├── ball_caster_wheel__1__1.stl
│   ├── lidar_urdf_1.stl
│   └── imu_for_urdf_1.stl
│
├── scripts/
│   └── kill_sim.sh             # Kill all simulation processes before relaunch
│
└── worlds/
    ├── testing.world           # 10×10 m furnished indoor environment
    ├── empty.world
    └── yoyo.sdf
```

---

## Prerequisites

| Dependency | Version | Install |
|---|---|---|
| Ubuntu | 22.04 LTS | — |
| ROS 2 | Humble | [docs.ros.org](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| Ignition Gazebo | Fortress | [gazebosim.org](https://gazebosim.org/docs/fortress/install_ubuntu) |
| SLAM Toolbox | Humble | `sudo apt install ros-humble-slam-toolbox` |
| Nav2 | Humble | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| robot_localization | Humble | `sudo apt install ros-humble-robot-localization` |
| ros_gz | Humble-Fortress | `sudo apt install ros-humble-ros-gz` |
| numpy | — | `pip3 install numpy` |

> Select **Fortress LTS** on the Gazebo install page and use binary installation.

---

## Installation

```bash
# 1. Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone
git clone https://github.com/amey1942007/Ros_lidar_bot.git

# 3. Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select Ros_lidar_bot --symlink-install

# 5. Source
source install/setup.bash
# Add permanently:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### Before every launch — kill leftover processes

Lingering processes from a previous run publish stale TF transforms and cause the robot to teleport. Always clean up first:

```bash
~/ros2_ws/src/Ros_lidar_bot/scripts/kill_sim.sh
```

---

### Manual navigation (teleoperation + SLAM)

```bash
ros2 launch Ros_lidar_bot launch_sim.launch.py
```

Starts Gazebo, bridge, RSP, EKF, SLAM Toolbox, and Nav2. Drive manually with the teleop node (new terminal):

```bash
ros2 run Ros_lidar_bot teleop_node.py
```

| Key | Action |
|-----|--------|
| `i` | Increase forward speed |
| `k` | Increase reverse speed |
| `j` | Turn left |
| `l` | Turn right |
| `Space` | Full stop |
| `Ctrl+C` | Quit |

You can also send goals directly from RViz2 using the **2D Goal Pose** button.

---

### Autonomous exploration

```bash
ros2 launch Ros_lidar_bot autonomous_exploration.launch.py
```

This single command starts everything — Gazebo, SLAM, EKF, Nav2, and the frontier explorer. The robot autonomously explores the entire environment, navigating to unexplored frontier regions until the map is complete.

The frontier explorer starts at **T = 30 s** after launch (Nav2 needs ~14 s to warm up after starting at T = 16 s).

---

### Visualise in RViz2

```bash
rviz2 -d ~/ros2_ws/src/Ros_lidar_bot/config/view_bot.rviz
```

| Display | Topic | Notes |
|---|---|---|
| Map | `/map` | SLAM occupancy grid |
| LaserScan | `/scan` | Live lidar hits |
| RobotModel | — | URDF via `/robot_description` |
| TF | — | Full transform tree |
| Image | `/camera/image_raw` | Camera feed |

Set **Fixed Frame** to `map`.

---

### View camera feed

**RViz2** — Add → By Topic → `/camera/image_raw` → Image

**rqt**:
```bash
ros2 run rqt_image_view rqt_image_view
```

**Gazebo GUI** — Plugins menu (⋮) → Image Display → select `/camera/image_raw`

---

### Save the finished map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Saves `my_map.pgm` and `my_map.yaml` for use in localisation mode.

---

## ROS 2 Topics

### Subscribed

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop |
| `/odom_raw` | `nav_msgs/Odometry` | Gazebo bridge (remapped `/odom`) |
| `/imu` | `sensor_msgs/Imu` | Gazebo bridge |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox (frontier explorer) |

### Published

| Topic | Type | Publisher |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge |
| `/odom` | `nav_msgs/Odometry` | EKF (filtered) |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo bridge |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo bridge |
| `/robot_description` | `std_msgs/String` | robot_state_publisher |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | RSP + EKF + SLAM |

### TF Tree

```
map
 └─ odom                (SLAM Toolbox)
     └─ base_footprint  (EKF)
         └─ base_link   (robot_state_publisher)
             ├─ laser_frame
             ├─ camera_link
             │   └─ camera_link_optical
             ├─ imu_for_urdf_1
             ├─ wheel_urdf_1        (right wheel)
             └─ wheel_urdf__1__1    (left wheel)
```

---

## Configuration

### EKF (`config/ekf.yaml`)
Fuses `/odom_raw` (wheel odometry) and `/imu` at 30 Hz in 2D mode. Publishes the `odom → base_footprint` TF and filtered `/odom`.

### SLAM Toolbox (`config/mapper_params_online_async.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| `resolution` | 0.05 m | Map cell size |
| `max_laser_range` | 12.0 m | Matches physical lidar range |
| `map_update_interval` | 3.0 s | Occupancy grid publish rate |
| `map_start_pose` | [0, 0, 0] | Map origin anchored to robot spawn position |
| `minimum_travel_distance` | 0.5 m | Distance between scan-match updates |
| `do_loop_closing` | true | Pose-graph loop closure for drift correction |

### Nav2 (`config/nav2_params.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| Planner | NavFn A* | `allow_unknown: true` for frontier goals |
| Controller | Regulated Pure Pursuit | `use_rotate_to_heading: true` |
| Linear velocity | 0.22 m/s | Target cruise speed |
| Local inflation radius | 0.55 m | Safety clearance around obstacles |
| Global inflation radius | 0.65 m | Routes planned well away from walls |
| `cost_scaling_factor` | 2.0 | Slow cost drop-off = robot stays far from walls |
| Recovery behaviours | spin, backup, wait | Triggered after 25 s without progress |

### Frontier Explorer (`config/frontier_explorer.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| `min_frontier_size` | 3 cells | Minimum cluster size to consider |
| `wall_safe_distance` | 0.6 m | Goals within this distance of the map boundary are rejected |
| `openness_weight` | 0.7 | Strongly prefers goals in open space over tight corners |
| `max_goal_distance` | 5.5 m | Maximum frontier search radius |
| `progress_timeout_sec` | 35 s | Longer than Nav2's 25 s recovery window |
| `no_frontier_done_ticks` | 10 | Declares exploration complete after 10 consecutive empty ticks |

---

## Progress

### Done
- [x] Differential-drive robot URDF with accurate inertials and collision meshes
- [x] GPU LiDAR, IMU, RGB camera sensors in simulation
- [x] Full Ignition Gazebo — ROS 2 bridge (sensors, clock, cmd_vel, joint states)
- [x] EKF odometry fusion (wheel odometry + IMU)
- [x] SLAM Toolbox online async mapping with loop closure
- [x] Nav2 stack (global planner, local controller, costmaps, recovery behaviours)
- [x] Frontier-based autonomous exploration node
- [x] Autonomous exploration launch file
- [x] Keyboard teleoperation node
- [x] 10×10 m furnished indoor test world
- [x] RViz2 configuration
- [x] Hardware-safe navigation (inflation radius, wall-safe boundary guard, openness filter)

### In Progress
- [ ] Hardware deployment and testing on the physical robot

### Planned
- [ ] Vision model integration — object detection on `/camera/image_raw` using a pre-trained model (YOLO / DETR)
- [ ] Semantic mapping — annotate the occupancy map with detected object classes and locations
- [ ] Semantic SLAM — persistent object-level map for downstream tasks (navigation to a chair, avoid humans, etc.)
- [ ] Real-hardware integration and field testing

---

## Common Issues

**Robot teleports between positions after relaunch**
→ A zombie process from the previous run is still publishing `map → odom`. Run `scripts/kill_sim.sh` before every relaunch.

**`worldToMap failed: mx,my: 198,X` errors**
→ A frontier goal landed at the map boundary (wall position). Increase `wall_safe_distance` in `frontier_explorer.yaml`.

**`No valid frontier` immediately at exploration start**
→ Either the map hasn't built yet (wait for T = 30 s) or `min_frontier_size` is too high for the current environment. Try reducing to 3.

**`Message Filter dropping message: frame 'odom'`**
→ Cosmetic RViz warning from a sim-clock reset. Does not affect navigation or SLAM.

**Robot collides with walls during exploration**
→ Increase `inflation_radius` in `nav2_params.yaml` and/or increase `wall_safe_distance` in `frontier_explorer.yaml`.
