# Ros_lidar_bot

A ROS 2 (Jazzy) differential-drive robot for fully autonomous indoor mapping.
Real hardware: Raspberry Pi 5, DDSM115 hub motors over RS485, RPLidar S2E over
Ethernet, BNO055 IMU via Arduino Mega. Real-time SLAM, EKF sensor fusion, Nav2
navigation, frontier exploration, a scan-based safety stop, YOLO-World semantic
mapping, and a browser control dashboard served from the robot.

> **Branches**
> - **`main` (this branch — hardware):** runs the physical robot.
> - **[`Simulation`](../../tree/Simulation):** the Ignition Gazebo simulation
>   version of the same stack (furnished worlds, gz bridge, camera plugin).
>   Use it to develop and test without the robot.

---

## Table of Contents

- [System Architecture](#system-architecture)
- [Robot Hardware](#robot-hardware)
- [Web Dashboard](#web-dashboard)
- [Semantic Vision Pipeline](#semantic-vision-pipeline)
- [Project Structure](#project-structure)
- [Installation](#installation)
- [Usage](#usage)
- [ROS 2 Topics](#ros-2-topics)
- [Configuration](#configuration)
- [Common Issues](#common-issues)

---

## System Architecture

```
DDSM115 motors ←RS485/USB→ [driver_node] ──→ /encoder (JointState, 20 Hz)
                                │  ← subscribes /cmd_vel_safe
BNO055 ←UART→ Arduino Mega ←USB→ [imu_node] ──→ /imu (50 Hz)
RPLidar S2E ←Ethernet/UDP→ [sllidar_node] ──→ /scan (10 Hz, DenseBoost)

[odom_node]  ←─ /encoder ──→ /odom_raw (wheel odometry, no TF)

[ekf_filter_node]  ←─ /odom_raw + /imu
    └─ TF: odom → base_footprint          └─ publishes /odom (filtered)

[slam_toolbox]  ←─ /scan + TF
    └─ TF: map → odom                     └─ publishes /map

[nav2 stack]  ←─ /map + /scan + /odom ──→ /cmd_vel

[frontier_explorer]  ←─ /map + /safety_blocked + /manual_blacklist
    └─ sends NavigateToPose actions to Nav2 (autonomous exploration)

[joy_node + joy_teleop]  gamepad ──→ /cmd_vel  (overrides Nav2 while stick moves)

[safety_stop]  ←─ /cmd_vel + /scan
    └─ zeroes unsafe motion ──→ /cmd_vel_safe ──→ driver_node ──→ motors
    └─ publishes /safety_blocked
```

Every velocity command — Nav2, gamepad, dashboard, test tools — flows through
the safety stop before reaching the motors. Forward/reverse motion is blocked
when an obstacle is inside `min_safe_distance`; rotation is always preserved so
Nav2 recovery spins still work.

---

## Robot Hardware

| Component | Detail |
|---|---|
| Compute | Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy |
| Drive | 2× DDSM115 direct-drive hub motors (RS485, 10-byte CRC8/MAXIM packets) via USB-RS485 dongle on `/dev/ttyACM0` |
| Wheels | ⌀100.7 mm, wheel base 0.33 m, min reliable command ≈6 RPM |
| LiDAR | RPLidar S2E — **Ethernet/UDP** (`192.168.11.2:8089`), DenseBoost (~3200 pts/rev) |
| IMU | BNO055 on Arduino Mega, streamed over USB serial `/dev/ttyACM1` at 500000 baud |
| Camera (planned) | RPi Camera Module 3 for the semantic vision pipeline |
| Teleop | Any SDL gamepad (USB or Bluetooth) |

**LiDAR network setup (one-time):** give `eth0` a static IP on the lidar's
subnet:

```bash
sudo nmcli con add type ethernet ifname eth0 con-name lidar \
     ipv4.method manual ipv4.addresses 192.168.11.1/24
ping 192.168.11.2   # verify
```

The lidar driver is [Slamtec sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)
cloned into `src/` — the apt `rplidar-ros` package is serial-only and will not
work with the S2E.

---

## Web Dashboard

`launch_robot.launch.py` serves a self-contained browser GUI from the robot at
**`http://<robot-ip>:8080`** — no RViz or X11 needed on a headless Pi.

- **Live view** — robot model, lidar scan, motion trail, **live SLAM map**,
  planned path + current goal flag, frontier queue and blacklist zones,
  semantic object labels. Wheel-zoom canvas.
- **Click-to-navigate** — arm "Nav-goal mode", click the map to send a Nav2 goal.
- **No-go zones** — arm "🚫 No-go mode", press-and-drag to draw a circle the
  frontier explorer must avoid; pick duration (1/5/15 min or permanent).
- **System panel** — CPU, SoC temperature, RAM, disk, load, uptime, Pi
  throttling alarm.
- **💾 Save map** — writes `.pgm`/`.yaml` + a serialized pose-graph to `~/maps/`.
- **Tools** — IMU test, IMU calibration, drive-distance test; started on
  demand, output streamed to the console panel.
- **Semantic vision toggle** — starts/stops the camera + YOLO-World + semantic
  SLAM pipeline (see below).
- **E-STOP** — kills the running tool, cancels the nav goal, streams zero
  velocity.

---

## Semantic Vision Pipeline

Started on demand from the dashboard toggle (or manually:
`ros2 launch Ros_lidar_bot vision.launch.py`):

```
camera (OpenCV, 640×480) → yolo (YOLO-World, open vocabulary)
    → /yolo (String, JSON detections)
/yolo + /scan + TF map→base_link → semantic_slam
    → object (x, y) in map frame → /semantic_markers + /tmp/semantic_map.json
```

Object bearing comes from the bbox centre through the camera model
(66° HFOV), depth from the lidar ray at that bearing. Detections are merged
across sightings and only published after 3 confirmations. Labels appear in
RViz and on the dashboard map.

Requires `pip3 install ultralytics` and the `yolov8s-world.pt` weights
(auto-downloaded on first run).

---

## Project Structure

```
Ros_lidar_bot/
├── launch/
│   ├── launch_robot.launch.py       # hardware bringup (manual driving)
│   ├── autonomous_robot.launch.py   # bringup + frontier exploration
│   ├── vision.launch.py             # on-demand semantic vision (dashboard toggle)
│   ├── rsp.launch.py                # robot_state_publisher (URDF)
│   └── lidar_test.launch.py         # lidar-only smoke test
├── Ros_lidar_bot/
│   ├── driver_node.py               # DDSM115 RS485 driver — /cmd_vel_safe → motors, /encoder out
│   ├── odom_node.py                 # wheel odometry → /odom_raw
│   ├── imu_node.py                  # BNO055 serial reader → /imu
│   ├── safety_stop_node.py          # /cmd_vel → /cmd_vel_safe scan-based filter
│   ├── frontier_explorer_node.py    # autonomous exploration (Nav2 action client)
│   ├── robot_dashboard_node.py      # web GUI server (port 8080)
│   ├── joy_teleop_node.py           # gamepad → /cmd_vel
│   ├── yolo.py                      # YOLO-World camera detector → /yolo
│   ├── semantic_slam_node.py        # /yolo + lidar → object map
│   ├── imu_test_node.py / imu_calibration_node.py / drive_distance_node.py
│   └── bringup_status_node.py       # legacy terminal status board
├── config/
│   ├── ekf.yaml                     # robot_localization fusion (audited timestamps)
│   ├── nav2_params.yaml             # Nav2 stack tuning
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox
│   └── frontier_explorer.yaml       # exploration tuning
├── description/                     # URDF/Xacro
└── install_rpi5_jazzy.sh            # one-shot Pi setup script
```

---

## Installation

```bash
# 1. Workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 2. Clone (this repo + the S2E lidar driver)
git clone https://github.com/amey1942007/Ros_lidar_bot.git
git clone https://github.com/Slamtec/sllidar_ros2.git

# 3. ROS dependencies
sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
     ros-jazzy-nav2-bringup ros-jazzy-robot-localization ros-jazzy-joy

# 4. Python dependencies
pip3 install pyserial ultralytics opencv-python

# 5. Build & source
cd ~/ros2_ws && colcon build
source install/setup.bash   # add to ~/.bashrc
```

Or run `install_rpi5_jazzy.sh` on a fresh Pi.

---

## Usage

**Manual driving / mapping by hand:**

```bash
ros2 launch Ros_lidar_bot launch_robot.launch.py
```

**Fully autonomous exploration:**

```bash
ros2 launch Ros_lidar_bot autonomous_robot.launch.py
```

Then open `http://<robot-ip>:8080` in a browser. Node logs are suppressed to
keep the terminal clean — add `verbose:=true` to either launch to see them.

The gamepad always works: Nav2 owns `/cmd_vel` while the stick is centred,
the stick overrides while moved. Save the finished map from the dashboard.

---

## ROS 2 Topics

| Topic | Type | Producer → Consumer |
|---|---|---|
| `/cmd_vel` | Twist | Nav2 / teleop / dashboard → safety_stop |
| `/cmd_vel_safe` | Twist | safety_stop → driver_node |
| `/safety_blocked` | Bool | safety_stop → frontier_explorer / UI |
| `/encoder` | JointState | driver_node → odom_node |
| `/odom_raw` | Odometry | odom_node → EKF |
| `/odom` | Odometry | EKF (filtered) → Nav2 / UI |
| `/imu` | Imu | imu_node → EKF |
| `/scan` | LaserScan | sllidar → SLAM / Nav2 / safety_stop |
| `/map` | OccupancyGrid | slam_toolbox → Nav2 / frontier / dashboard |
| `/yolo` | String (JSON) | yolo → semantic_slam |
| `/semantic_markers` | MarkerArray | semantic_slam → RViz / dashboard |
| `/frontier_debug` | MarkerArray | frontier_explorer → RViz / dashboard |
| `/manual_blacklist` | String (JSON) | dashboard → frontier_explorer |

TF chain: `map → odom` (SLAM) `→ base_footprint` (EKF) `→ base_link → laser_frame / imu_link / wheels` (URDF).

---

## Configuration

The important, hardware-derived settings (change with care):

- **`ekf.yaml`** — wheel odometry is the primary sensor (vX, vYaw); gyro
  complements yaw. The full timestamp pipeline is documented in the file
  header.
- **`nav2_params.yaml`** — `transform_tolerance: 1.0` everywhere, needed
  because DenseBoost scan-matching runs ~0.6 s behind on the Pi.
- **DenseBoost** is intentional (full angular resolution). Do not downgrade
  the scan mode to buy CPU headroom without checking first.
- **`frontier_explorer.yaml`** — exploration scoring, blacklist radii and
  auto-calibration. Progress timeout must stay above Nav2's
  `movement_time_allowance`.
- **Serial ports** — motors `/dev/ttyACM0`, IMU `/dev/ttyACM1` (set in
  `launch_robot.launch.py`). If USB re-enumeration swaps them, check
  `ls -l /dev/serial/by-id/`.

---

## Common Issues

| Symptom | Cause / fix |
|---|---|
| Dashboard shows TF `map→odom` / `odom→base_footprint` MISSING | A stale previous launch is holding the serial ports (or port 8080). `pkill -9 -f "ros2 launch"; pkill -9 -f _node`, verify with `fuser -v /dev/ttyACM* 8080/tcp`, relaunch. |
| Driver up but robot won't move | Check `/cmd_vel_safe` is flowing (`ros2 topic hz /cmd_vel_safe`) — the driver only listens to the safety-filtered topic. For bench tests without safety_stop: `--ros-args -p cmd_vel_topic:=/cmd_vel`. |
| "SAFETY STOP: obstacle X m ahead" | Working as intended — something is inside `min_safe_distance` (0.40 m). The robot can still rotate away. |
| No `/scan` | Lidar Ethernet link: `ping 192.168.11.2`; check the static IP on `eth0`. |
| Serial nodes silent / no `/encoder` or `/imu` | ACM numbers swapped after replug — see `/dev/serial/by-id/`, update ports in the launch, or replug/reboot. |
| Robot plans but never drives | Chassis self-hits tripping the safety stop — `ignore_below` must stay ≥ the chassis radius (0.28 m). |
| Map smears during fast spins | Known S2E timestamp skew; keep rotation speeds moderate (see `ekf.yaml` header). |
| `worldToMap failed` errors during exploration | Frontier goal at the map boundary — raise `wall_safe_distance` in `frontier_explorer.yaml`. |
