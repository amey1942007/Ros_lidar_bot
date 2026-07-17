# Ros_lidar_bot

A ROS 2 differential-drive robot with LiDAR, IMU, camera, and semantic object detection. Runs in Ignition Gazebo with real-time SLAM, EKF odometry fusion, Nav2 autonomous navigation, frontier-based exploration, and YOLO World semantic mapping.

---

## Table of Contents

- [About](#about)
- [System Architecture](#system-architecture)
- [Semantic Detection Pipeline](#semantic-detection-pipeline)
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

Custom differential-drive robot platform targeting fully autonomous indoor operation. The stack covers:

- **Ignition Gazebo (Fortress)** physics simulation — indoor furnished worlds
- **SLAM Toolbox** — real-time occupancy-grid mapping with loop closure
- **robot_localization EKF** — fuses wheel odometry + IMU into clean `/odom`
- **Nav2** — global path planning, local trajectory, recovery behaviours
- **Frontier exploration** — fully autonomous map coverage, no pre-defined waypoints
- **Semantic mapping** — open-vocabulary YOLO World object detection → real-world (x, y) coordinates → RViz2 overlay
- **Gamepad teleoperation** — manual driving with a USB controller

---

## System Architecture

```
Ignition Gazebo
  │
  ├─ DiffDrive plugin   ──→ /odom       ──→ [gz bridge] ──→ /odom_raw
  ├─ GPU LiDAR          ──→ /scan       ──→ [gz bridge] ──→ /scan
  ├─ IMU sensor         ──→ /imu        ──→ [gz bridge] ──→ /imu
  ├─ Camera             ──→ /images_raw ──→ [gz bridge]
  └─ JointStatePublisher──→ /joint_states──→ [gz bridge]

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
    ├─ controller_server— Regulated Pure Pursuit local controller
    ├─ behavior_server  — spin / backup / wait recovery behaviours
    └─ bt_navigator     — navigate_to_pose action server → /cmd_vel

[frontier_explorer]  ←─ /map + TF(map→base_footprint)
    └─ sends goals to navigate_to_pose action server

[OpenCV + YOLO World publisher]  ←─ RPi Camera Module 3 (or Gazebo /images_raw)
    └─ runs YOLO World inference, publishes /yolo (JSON detections)

[semantic_slam_node]  ←─ /yolo + /scan + TF(map→base_link)
    └─ publishes: /semantic_markers (RViz2 MarkerArray with x,y map coordinates)
    └─ saves:     /tmp/semantic_map.json
```

**Staged startup** (`launch_sim.launch.py`):

| Time | What starts |
|------|-------------|
| T = 0 s | Gazebo, RSP, bridge |
| T = 3 s | Robot spawned into world |
| T = 6 s | EKF + SLAM Toolbox |
| T = 16 s | Nav2 stack |
| T = 32 s | Frontier explorer *(autonomous launch only)* |

The semantic node and OpenCV publisher are started **independently** in separate terminals.

---

## Semantic Detection Pipeline

The semantic pipeline runs as two cooperating nodes:

```
RPi Camera Module 3 (physical) or Gazebo /images_raw (simulation)
        │
        ▼
[opencv_yolo_publisher node]
  • Captures frames from the camera
  • Runs YOLO World open-vocabulary detection (classes configured in script)
  • Publishes /yolo — std_msgs/String — JSON array of detections
        │
        │  /yolo message format:
        │  [
        │    {"class": "chair",  "x": 320, "y": 240, "w": 100, "h": 150, "conf": 0.82},
        │    {"class": "person", "x": 120, "y": 200, "w": 60,  "h": 180, "conf": 0.91}
        │  ]
        │  x, y = bbox centre in pixels | w, h = bbox size in pixels
        ▼
[semantic_slam_node]
  • Subscribes to /yolo + /scan + TF (map → base_link)
  • For each detection:
      1. Compute horizontal angle from bbox centre pixel + camera FOV
      2. Look up LiDAR range at that angle  →  object depth (m)
      3. Project depth into base_link frame (x_base, y_base)
      4. Rotate to map frame using robot_yaw from TF
      5. Merge with existing object list (weighted average, 0.6 m radius)
  • Publishes after MIN_SIGHTINGS = 3 confirmations
        │
        ▼
/semantic_markers  —  visualization_msgs/MarkerArray
  • Coloured CUBE marker at (x, y) map position
  • TEXT_VIEW_FACING label: "chair (82%)"
  • Lifetime 3 s (refreshed on each detection cycle)

/tmp/semantic_map.json  —  persisted across node restarts
```

### Camera Constants (edit in `semantic_slam_node.py` if needed)

| Constant | Value | Notes |
|----------|-------|-------|
| `IMG_W` | 640 px | Frame width expected from publisher |
| `IMG_H` | 480 px | Frame height |
| `FOV_H` | 66° (1.152 rad) | RPi Camera Module 3 horizontal FOV |
| `FOCAL_LEN` | ~514 px | Derived: `IMG_W / (2 × tan(FOV_H/2))` |

---

## Robot Hardware

| Component | Specification |
|-----------|---------------|
| Base | Custom differential-drive chassis |
| Drive wheels | 2 × motorised, separation 339.5 mm, radius 81 mm |
| Casters | 2 × passive ball casters |
| LiDAR | 360° GPU LiDAR, range 0.3–12 m, 20 Hz, 360 samples/rev |
| IMU | 6-DOF IMU, 50 Hz |
| Camera | RPi Camera Module 3, RGB 640×480, 30 Hz, 66° FOV |
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
│   ├── __init__.py
│   ├── frontier_explorer_node.py   # Autonomous frontier exploration
│   ├── semantic_slam_node.py       # Semantic object localisation + MarkerArray
│   ├── safety_stop_node.py         # Emergency stop on proximity
│   └── joy_teleop_node.py          # Gamepad teleoperation (USB controller)
│
├── config/
│   ├── ekf.yaml                         # EKF sensor fusion parameters
│   ├── frontier_explorer.yaml           # Frontier explorer tuning
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox (mapping mode)
│   ├── mapper_params_localization.yaml  # SLAM Toolbox (localisation mode)
│   ├── nav2_params.yaml                 # Nav2 stack parameters
│   └── view_bot.rviz                    # RViz2 configuration
│
├── description/
│   ├── robot.urdf.xacro       # Top-level URDF entry point
│   ├── slam.xacro             # Base, wheels, casters
│   ├── slam.gazebo            # Gazebo material/friction
│   ├── robot_control.xacro   # DiffDrive + JointState plugins
│   ├── lidar.xacro            # GPU LiDAR sensor
│   ├── camera.xacro           # RGB camera sensor
│   └── imu.xacro              # IMU sensor
│
├── launch/
│   ├── autonomous_exploration.launch.py  # Sim + Nav2 + frontier explorer
│   ├── launch_sim.launch.py              # Sim + SLAM + EKF + Nav2
│   └── rsp.launch.py                     # Robot State Publisher only
│
├── meshes/
│   └── *.stl                  # Robot mesh files
│
├── scripts/
│   ├── install_deps.sh        # Install all dependencies
│   └── kill_sim.sh            # Kill all sim processes before relaunch
│
└── worlds/
    ├── testing.world           # 10×10 m furnished indoor world
    ├── home.world              # 3BHK residential (18×15 m)
    ├── room.world              # Mixed-use institutional space
    └── empty.world
```

---

## Prerequisites

| Dependency | Version | Install |
|------------|---------|---------|
| Ubuntu | 22.04 LTS | — |
| ROS 2 | Humble | [docs.ros.org](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| Ignition Gazebo | Fortress | [gazebosim.org](https://gazebosim.org/docs/fortress/install_ubuntu) |
| SLAM Toolbox | Humble | `sudo apt install ros-humble-slam-toolbox` |
| Nav2 | Humble | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| robot_localization | Humble | `sudo apt install ros-humble-robot-localization` |
| ros_gz | Humble-Fortress | `sudo apt install ros-humble-ros-gz` |
| ultralytics | ≥ 8.1 | `pip3 install ultralytics` |
| opencv-python | ≥ 4.8 | `pip3 install opencv-python` |
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

# 4. Install Python dependencies
pip3 install ultralytics opencv-python numpy

# 5. Build
colcon build --packages-select Ros_lidar_bot --symlink-install

# 6. Source
source install/setup.bash
# Add permanently:
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### Before every launch — kill leftover processes

```bash
~/ros2_ws/src/Ros_lidar_bot/scripts/kill_sim.sh
```

---

### Manual navigation (gamepad teleoperation + SLAM)

The hardware launch (`launch_robot.launch.py`) already starts `joy_node` +
`joy_teleop` for the USB controller dongle, so just plug in the dongle and
drive. Teleop yields `/cmd_vel` to Nav2 whenever the left stick is centered.

To run teleop standalone:

```bash
ros2 run joy joy_node &
ros2 run Ros_lidar_bot joy_teleop
```

| Control | Action |
|---------|--------|
| Left stick up/down | Forward / reverse (proportional) |
| Left stick left/right | Turn left / right (proportional) |
| `RT` | Increase linear speed (+0.05 m/s per press) |
| `LT` | Decrease linear speed (−0.05 m/s per press) |
| `RB` | Increase angular speed (+0.1 rad/s per press) |
| `LB` | Decrease angular speed (−0.1 rad/s per press) |
| Release stick | Stop (and yield `/cmd_vel` to Nav2) |

Axis/button indices are ROS parameters on `joy_teleop` — if your controller
maps differently, verify with `ros2 topic echo /joy` and override
`axis_linear`, `axis_angular`, `axis_rt`, `axis_lt`, `button_rb`, `button_lb`.

---

### Autonomous exploration

```bash
ros2 launch Ros_lidar_bot autonomous_exploration.launch.py
# Optional: world:=home.world  or  world:=room.world
```

Starts Gazebo, SLAM, EKF, Nav2, and the frontier explorer. The robot autonomously covers the entire environment. Explorer starts at **T = 32 s**.

---

### Semantic object detection

Run the two semantic nodes in **separate terminals** (after the simulation or real robot is up):

**Terminal 1 — OpenCV + YOLO World publisher:**
```bash
# (your opencv_yolo_publisher script)
python3 opencv_yolo_publisher.py
# Publishes /yolo with JSON detections from the RPi Cam 3
```

**Terminal 2 — Semantic SLAM node:**
```bash
ros2 run Ros_lidar_bot semantic_slam_node.py
```

Optional parameters:
```bash
ros2 run Ros_lidar_bot semantic_slam_node.py \
  --ros-args -p conf:=0.3 -p max_depth:=6.0
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `conf` | `0.25` | Minimum detection confidence to accept |
| `max_depth` | `8.0` | Maximum LiDAR depth (m) — ignore far-wall hits |

---

### View semantic markers in RViz2

```bash
rviz2 -d ~/ros2_ws/src/Ros_lidar_bot/config/view_bot.rviz
```

Add display: **MarkerArray** → topic `/semantic_markers`. Coloured 3D boxes and labels will appear at detected object positions in the map frame.

---

### Save the finished map

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

---

## ROS 2 Topics

### Subscribed

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/yolo` | `std_msgs/String` | semantic_slam_node | JSON array of YOLO World detections |
| `/scan` | `sensor_msgs/LaserScan` | semantic_slam_node, SLAM | LiDAR scan |
| `/odom_raw` | `nav_msgs/Odometry` | EKF | Raw wheel odometry from Gazebo |
| `/imu` | `sensor_msgs/Imu` | EKF | IMU data |
| `/map` | `nav_msgs/OccupancyGrid` | frontier_explorer, Nav2 | SLAM occupancy grid |
| `/cmd_vel` | `geometry_msgs/Twist` | Gazebo bridge | Velocity commands |

### Published

| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/semantic_markers` | `visualization_msgs/MarkerArray` | semantic_slam_node | Object positions in map frame (RViz2) |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge | Live LiDAR scan |
| `/odom` | `nav_msgs/Odometry` | EKF | Filtered odometry |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Occupancy grid |
| `/images_raw` | `sensor_msgs/Image` | Gazebo bridge | Camera feed |
| `/robot_description` | `std_msgs/String` | robot_state_publisher | URDF |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | RSP + EKF + SLAM | Transform tree |

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
Fuses `/odom_raw` + `/imu` at 30 Hz in 2D mode. Publishes `odom → base_footprint` TF and filtered `/odom`.

### SLAM Toolbox (`config/mapper_params_online_async.yaml`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `resolution` | 0.05 m | Map cell size |
| `max_laser_range` | 12.0 m | Matches LiDAR range |
| `map_update_interval` | 3.0 s | OccupancyGrid publish rate |
| `map_start_pose` | [0, 0, 0] | Map origin at robot spawn |
| `do_loop_closing` | true | Pose-graph loop closure |

### Nav2 (`config/nav2_params.yaml`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Planner | NavFn A* | `allow_unknown: true` |
| Controller | Regulated Pure Pursuit | `use_rotate_to_heading: true` |
| Linear velocity | 0.22 m/s | Cruise speed |
| Local inflation radius | 0.55 m | Safety clearance |
| Global inflation radius | 0.65 m | Route away from walls |
| Recovery behaviours | spin, backup, wait | After 25 s without progress |

### Frontier Explorer (`config/frontier_explorer.yaml`)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `min_frontier_size` | 3 cells | Minimum cluster to consider |
| `wall_safe_distance` | 0.6 m | Reject goals near map boundary |
| `openness_weight` | 0.7 | Prefer open space over tight corners |
| `max_goal_distance` | 5.5 m | Maximum frontier search radius |
| `progress_timeout_sec` | 35 s | Longer than Nav2 recovery window |
| `no_frontier_done_ticks` | 10 | Declare exploration complete |

### Semantic SLAM (ROS parameters)

| Parameter | Default | Notes |
|-----------|---------|-------|
| `conf` | `0.25` | Minimum detection confidence |
| `max_depth` | `8.0` | Ignore LiDAR hits beyond this (m) |
| `MERGE_RADIUS` | 0.60 m | (code constant) merge nearby detections |
| `MIN_SIGHTINGS` | 3 | (code constant) confirmations before publishing |
| `MIN_DEPTH` | 0.20 m | (code constant) ignore very close LiDAR returns |

---

## Progress

### Done
- [x] Differential-drive robot URDF with accurate inertials and collision meshes
- [x] GPU LiDAR, IMU, RGB camera sensors in simulation
- [x] Full Ignition Gazebo — ROS 2 bridge (sensors, clock, cmd\_vel, joint states)
- [x] EKF odometry fusion (wheel odometry + IMU)
- [x] SLAM Toolbox online async mapping with loop closure
- [x] Nav2 stack (global planner, local controller, costmaps, recovery behaviours)
- [x] Frontier-based autonomous exploration node
- [x] Autonomous exploration launch file
- [x] Gamepad teleoperation node (USB controller, joy-based)
- [x] Multiple indoor simulation worlds (testing, home 3BHK, room)
- [x] RViz2 configuration
- [x] Safety stop node (proximity-based emergency stop)
- [x] **Semantic SLAM node** — YOLO World detections → LiDAR-fused (x, y) → `/semantic_markers`
- [x] **Persistent semantic map** (`/tmp/semantic_map.json`)

### In Progress
- [ ] OpenCV + YOLO World publisher node (RPi Camera Module 3)
- [ ] Hardware deployment and testing on physical robot

### Planned
- [ ] Real-hardware integration and field testing
- [ ] Navigate-to-object commands using the semantic map
- [ ] Human avoidance using semantic labels

---

## Common Issues

**Robot teleports between positions after relaunch**
→ Zombie process still publishing stale TF. Run `scripts/kill_sim.sh` before every relaunch.

**`worldToMap failed: mx,my` errors**
→ Frontier goal landed at map boundary. Increase `wall_safe_distance` in `frontier_explorer.yaml`.

**No valid frontier at exploration start**
→ Map hasn't built yet (wait for T = 32 s) or `min_frontier_size` is too high. Try reducing to 3.

**`Message Filter dropping message: frame 'odom'`**
→ Cosmetic RViz warning from sim-clock reset. Does not affect navigation.

**`/semantic_markers` not appearing in RViz2**
→ Check (1) SLAM is running and TF map→base_link exists, (2) `/scan` is publishing, (3) the object has been seen ≥ 3 times. Use `ros2 topic echo /yolo` to verify the publisher is running.

**Semantic node reports wrong object positions**
→ Check `FOV_H` and `IMG_W` in `semantic_slam_node.py` match the actual camera used. For the RPi Camera Module 3 at 640×480, FOV_H = 66°.
