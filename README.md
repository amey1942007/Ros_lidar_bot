# Ros_lidar_bot

A ROS 2 differential-drive robot with LiDAR, IMU, and camera sensors running in Ignition Gazebo. Supports real-time SLAM mapping, EKF-fused odometry, and keyboard teleoperation — with an autonomous frontier exploration pipeline in progress.

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

---

## About

The robot is a custom differential-drive platform modelled from real hardware. The simulation stack covers the full sensor-to-navigation pipeline:

- **Ignition Gazebo (Fortress)** physics simulation
- **SLAM Toolbox** for real-time occupancy-grid mapping
- **robot_localization EKF** that fuses wheel odometry + IMU into a clean `/odom`
- **Nav2** for global/local path planning and autonomous navigation
- **Keyboard teleoperation** for manual driving and map building

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
    └─ publishes TF: base_footprint → base_link → laser_frame
                                               → camera_link
                                               → imu_for_urdf_1
                                               → wheels

[ekf_filter_node]  ←─ /odom_raw + /imu
    └─ publishes TF: odom → base_footprint
    └─ publishes: /odom (filtered)

[slam_toolbox]  ←─ /scan + TF(odom→base_footprint)
    └─ publishes TF: map → odom
    └─ publishes: /map (OccupancyGrid)

[nav2 stack]  ←─ /map + /scan + /odom + TF tree
    └─ navigate_to_pose action server
    └─ publishes: /cmd_vel (velocity commands)
```

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
│   └── teleop_node.py          # Keyboard teleoperation node
│
├── config/
│   ├── ekf.yaml                # robot_localization EKF parameters
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox parameters
│   ├── mapper_params_localization.yaml  # SLAM localisation mode
│   ├── mapper_params_lifelong.yaml
│   ├── mapper_params_offline.yaml
│   ├── mapper_params_online_sync.yaml
│   └── view_bot.rviz           # RViz2 configuration
│
├── description/
│   ├── robot.urdf.xacro        # Top-level URDF entry point
│   ├── slam.xacro              # Base link, wheels, casters
│   ├── slam.gazebo             # Gazebo material/friction properties
│   ├── robot_control.xacro     # DiffDrive + JointState Gazebo plugins
│   ├── lidar.xacro             # GPU LiDAR sensor
│   ├── camera.xacro            # RGB camera sensor
│   └── imu.xacro               # IMU sensor
│
├── launch/
│   ├── launch_sim.launch.py    # Main simulation launch (Gazebo + SLAM + EKF)
│   └── rsp.launch.py           # Robot State Publisher only
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

> **Note:** Select **Fortress LTS** on the Gazebo install page and use binary installation.

---

## Installation

```bash
# 1. Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone the repository
git clone https://github.com/amey1942007/Ros_lidar_bot.git

# 3. Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select Ros_lidar_bot --symlink-install

# 5. Source the workspace
source install/setup.bash
```

Add to `~/.bashrc` so you don't have to source every session:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### Kill any leftover simulation processes before launching

If you previously ran the simulation, always clean up first to prevent zombie processes from causing TF conflicts and robot teleporting:

```bash
# Kill all simulation-related processes
pkill -SIGTERM -f "slam_toolbox"; pkill -SIGTERM -f "ekf_node"
pkill -SIGTERM -f "ign gazebo"; sleep 2
pkill -SIGKILL -f "ign gazebo"; pkill -SIGKILL -f "slam_toolbox"
```

---

### Launch the simulation

```bash
ros2 launch Ros_lidar_bot launch_sim.launch.py
```

This starts:
- Ignition Gazebo with `testing.world`
- Gazebo ↔ ROS 2 bridge (all sensor topics + clock)
- Robot State Publisher (URDF → TF tree)
- SLAM Toolbox (online async mapping)
- EKF node (odometry + IMU fusion)

---

### Keyboard teleoperation

In a **new terminal**:
```bash
source ~/ros2_ws/install/setup.bash
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

---

### Visualise in RViz2

```bash
rviz2 -d ~/ros2_ws/src/Ros_lidar_bot/config/view_bot.rviz
```

Key displays to add if not preset:

| Display | Topic / Frame |
|---|---|
| Map | `/map` |
| LaserScan | `/scan` |
| RobotModel | — (uses `/robot_description`) |
| TF | — |
| Image | `/camera/image_raw` |

Set **Fixed Frame** to `map`.

---

### View camera feed

**Option A — RViz2**: Add → By Topic → `/camera/image_raw` → Image

**Option B — rqt**:
```bash
ros2 run rqt_image_view rqt_image_view
```
Select `/camera/image_raw` from the dropdown.

**Option C — Gazebo GUI**: Plugins (⋮ menu) → Image Display → topic `/camera/image_raw`

---

### Save the map

After driving the robot to build a complete map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```
This saves `my_map.pgm` and `my_map.yaml`.

---

## ROS 2 Topics

### Subscribed

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop |
| `/odom_raw` | `nav_msgs/Odometry` | Gazebo bridge (remapped from `/odom`) |
| `/imu` | `sensor_msgs/Imu` | Gazebo bridge |

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
 └─ odom              (SLAM Toolbox)
     └─ base_footprint (EKF)
         └─ base_link  (robot_state_publisher)
             ├─ laser_frame
             ├─ camera_link
             │   └─ camera_link_optical
             ├─ imu_for_urdf_1
             ├─ wheel_urdf_1      (right wheel)
             └─ wheel_urdf__1__1  (left wheel)
```

---

## Configuration

### EKF (`config/ekf.yaml`)
Fuses wheel odometry (`/odom_raw`) and IMU (`/imu`) at 30 Hz. Runs in 2D mode — ignores Z, roll, pitch. Publishes the `odom → base_footprint` transform.

### SLAM Toolbox (`config/mapper_params_online_async.yaml`)
Online asynchronous SLAM. Key parameters:

| Parameter | Value | Notes |
|---|---|---|
| `resolution` | 0.05 m | Map cell size |
| `max_laser_range` | 20.0 m | Max range used for map building |
| `map_update_interval` | 5.0 s | How often the occupancy grid is published |
| `minimum_travel_distance` | 0.5 m | Robot must move this far before a new scan is processed |
| `do_loop_closing` | true | Corrects drift with loop closure |

---

## Progress

### Done
- [x] Differential-drive robot URDF with accurate inertials and collision meshes
- [x] GPU LiDAR, IMU, RGB camera sensors in simulation
- [x] Full Ignition Gazebo — ROS 2 bridge (sensors, clock, cmd_vel, joint states)
- [x] EKF odometry fusion (wheel odom + IMU)
- [x] SLAM Toolbox online async mapping
- [x] Keyboard teleoperation node
- [x] 10×10 m furnished indoor test world (desks, chairs, bookshelf, boxes)
- [x] RViz2 configuration

### In Progress
- [ ] Nav2 autonomous navigation (global + local planners, costmaps)
- [ ] Frontier-based autonomous exploration node
- [ ] Autonomous exploration launch file
- [ ] Hardware deployment and testing

### Planned
- [ ] Cartographer and G-Mapping SLAM comparison
- [ ] Semantic SLAM (object detection + map annotation)
- [ ] Real-hardware integration

---

## Common Issues

**Robot appears at wrong position in RViz after relaunch**
→ Zombie processes from the previous run are still publishing stale TF. Kill all simulation processes and relaunch.

**`worldToMap failed` errors in planner log**
→ A navigation goal landed at the map boundary. Increase inflation radius in `nav2_params.yaml` or add a wall-safe distance buffer in the frontier explorer.

**`No valid frontier` immediately at exploration start**
→ `min_frontier_size` is too high for the frontier clusters visible from the starting position. Reduce to 3–6 in `frontier_explorer.yaml`.

**`Message Filter dropping message: frame 'odom'`**
→ Cosmetic RViz warning caused by a sim-clock reset. Does not affect navigation or SLAM.
