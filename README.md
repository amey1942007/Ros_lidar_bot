# Ros_lidar_bot

Hardware bot: a differential-drive robot on a Raspberry Pi 5 (Ubuntu 24.04,
ROS 2 Jazzy). This is the `hardware` branch. It is not the AMR4 robot.

Bringup drives two DDSM115 hub motors, reads a BNO055 IMU and an RPLidar S2E,
fuses wheel odometry and IMU in an EKF, runs SLAM Toolbox and Nav2, and serves
a browser dashboard from the Pi. A scan-based safety stop sits in front of the
motors. Optional pieces, started only when you ask for them, are frontier
exploration, a pan/tilt camera head, YOLO-World semantic mapping, a web or
Bluetooth gamepad, and a cloud telemetry bridge.

---

## What runs

```
DDSM115 ←RS485→ driver_node  → /encoder
                         ← /cmd_vel_safe
BNO055 ←Arduino Mega→ imu_node → /imu
RPLidar S2E ←UDP→ sllidar_node → /scan_raw → scan_min_range_filter → /scan

odom_node ← /encoder → /odom_raw          (no TF)
ekf_node  ← /odom_raw + /imu → /odom and TF odom → base_footprint
slam_toolbox ← /scan → /map and TF map → odom
nav2 ← /map + /scan + /odom → /cmd_vel
safety_stop ← /cmd_vel + /scan + /emergency_stop → /cmd_vel_safe → driver_node
             also publishes /safety_blocked
```

Nav2, the gamepad, the keyboard teleop, the dashboard (nudge and
click-to-goal), drive-distance, and IMU calibration all publish `/cmd_vel`.
Frontier exploration sends Nav2 goals and does not publish velocity itself.
Only `safety_stop` publishes `/cmd_vel_safe`, and the driver subscribes to
that. Dashboard E-STOP latches `/emergency_stop` so the safety node zeros
the motor command even if another `/cmd_vel` publisher is still writing. Forward and reverse are blocked when a return inside the
front or rear opening is closer than `min_safe_distance` (0.35 m in
`launch_robot.launch.py`). Ranges closer than `ignore_below` (0.30 m) are
treated as the robot body. Rotation is not blocked.

`/scan` is not the raw lidar topic. The driver publishes `/scan_raw`;
`scan_min_range_filter` drops returns inside 0.30 m and republishes `/scan`.

---

## Hardware the launch file expects

| Piece | Where the launch file points |
|---|---|
| Compute | Raspberry Pi 5, ROS 2 Jazzy |
| Motors | DDSM115 on `/dev/ttyACM0`, 115200 baud, `invert_drive:=false` |
| IMU | BNO055 via Arduino Mega on `/dev/ttyACM1`, 500000 baud |
| Lidar | RPLidar S2E, UDP `192.168.11.2:8089`, frame `laser_frame`, scan mode `DenseBoost` |
| Camera head | Arduino Uno on `/dev/ttyUSB0`, 115200 baud. Sketch: `arduino/camera_head/camera_head.ino`. Pan is continuous-rotation; tilt is positional. Mounted upside-down (`upside_down:=true`). |
| Wheels | 100.7 mm diameter, 0.33 m wheel base (see `description/` and `config/ekf.yaml`) |

Give `eth0` an address on the lidar subnet once:

```bash
sudo nmcli con add type ethernet ifname eth0 con-name lidar \
     ipv4.method manual ipv4.addresses 192.168.11.1/24
ping 192.168.11.2
```

The S2E driver is [Slamtec sllidar_ros2](https://github.com/Slamtec/sllidar_ros2)
in the workspace `src/`. The apt `rplidar-ros` package is serial-only and
ignores `channel_type:=udp`.

If USB enumeration swaps the ACM ports, check `ls -l /dev/serial/by-id/` and
change the ports in `launch/launch_robot.launch.py`.

---

## Build and run

The bridge setup script expects this checkout at
`~/Desktop/ros2_ws/src/Ros_lidar_bot`. A normal colcon workspace works for
everything else if you source that workspace's `install/setup.bash`.

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/amey1942007/Ros_lidar_bot.git
git clone https://github.com/Slamtec/sllidar_ros2.git

sudo apt install ros-jazzy-slam-toolbox ros-jazzy-navigation2 \
     ros-jazzy-nav2-bringup ros-jazzy-robot-localization ros-jazzy-joy
pip3 install pyserial ultralytics opencv-python

cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

`install_rpi5_jazzy.sh` is the one-shot Pi installer for ROS 2 Jazzy.

Manual drive and mapping:

```bash
ros2 launch Ros_lidar_bot launch_robot.launch.py
```

Autonomous exploration (same bringup, then `frontier_explorer` at T=22 s,
after Nav2 which starts at T=8 s):

```bash
ros2 launch Ros_lidar_bot autonomous_robot.launch.py
```

Open `http://<pi>:8080`. Logs stay off the terminal unless you pass
`verbose:=true`.

Cloud bridge setup (venv, `/etc/amr-bridge.env`, launcher) is
[scripts/README.md](scripts/README.md). Do not put secrets in git.

---

## Joy teleop

`joy_teleop` turns `/joy` into `/cmd_vel` and `/camera_cmd`. While the stick
is centered it publishes one zero Twist and then stays quiet, so Nav2 can use
`/cmd_vel`. If `/joy` stops mid-motion it stops the robot.

`launch_robot.launch.py` starts only one source of `/joy`. `web_gamepad`
publishes zeros when its page goes stale, so it is not started next to a
physical `joy_node`.

`use_joystick` (default `auto`):

| Value | What starts |
|---|---|
| `1`, `true`, `yes` | `joy_node` + `joy_teleop` |
| `curses`, `keyboard`, `terminal` | terminal `teleop_interface` (no `/joy`) |
| `0`, `false`, `no` | `web_gamepad` + `joy_teleop`, even if a joystick exists |
| `auto` or anything else | `joy_node` if `/dev/input/js*` exists, otherwise `web_gamepad` |

### Web gamepad

With no joystick plugged in, bringup serves a virtual Xbox page on port 8765
(`host` `0.0.0.0`). Open `http://<pi>:8765` from a phone on the same network
or Tailscale. It publishes `/joy` at 30 Hz.

Left stick drives. Right stick pans and tilts the camera head. D-pad snaps the
head to a limit. RT/LT change linear speed, RB/LB change angular speed. B saves
a map, X toggles vision, Y starts IMU calibration, A starts or stops a waypoint
sequence. Those four actions are HTTP calls to the dashboard on port 8080.

### Bluetooth or USB pad (`joy_node`)

`joy_node` is the ROS `joy` package (SDL), `device_id` 0, so the pad must show
up as `/dev/input/js0`. USB pads usually do that as soon as they are plugged in.
For Bluetooth, pair once on the Pi:

```bash
bluetoothctl
power on
agent on
default-agent
scan on
# wait until the pad appears, then:
pair <MAC>
trust <MAC>
connect <MAC>
```

Quit `bluetoothctl`, check `ls /dev/input/js*`, then launch. `auto` picks
`joy_node` when a `js*` device exists. Force it with `use_joystick:=true`.

`joy_teleop` axis defaults match the web gamepad (`axis_linear` 1,
`axis_angular` 0, triggers on axes 4 and 5). A USB Xbox pad often puts the left
trigger on axis 2; override `axis_lt` if `ros2 topic echo /joy` disagrees.

### Keyboard

```bash
ros2 launch Ros_lidar_bot launch_robot.launch.py use_joystick:=keyboard
```

`teleop_interface` uses the terminal (i/,/j/l and the usual ROS keys, WASD for
the camera head). It needs a TTY.

---

## Launch files

Arguments below are the defaults in `launch/*.py`.

### `launch_robot.launch.py`

Hardware, EKF, SLAM (T=5 s), Nav2 (T=8 s), dashboard at T=0, and the cloud
bridge at T=5 s when `enable_amr_bridge` is true.

| Argument | Default | |
|---|---|---|
| `verbose` | `false` | Node logs on the terminal when true |
| `expect_frontier` | `false` | Dashboard expects `frontier_explorer` |
| `use_joystick` | `auto` | See the table above |
| `enable_amr_bridge` | `true` | Start `amr_telemetry_bridge` |
| `amr_bot_id` | `hardware-bot` | Passed through as the bridge `bot_id` |
| `amr_bridge_env` | `/etc/amr-bridge.env` | Secrets file for the bridge |
| `amr_bridge_python` | `auto` | `auto` uses `amr-bridge/.venv/bin/python` when that file is executable, otherwise `python3`. An explicit path is used as given |

```bash
ros2 launch Ros_lidar_bot launch_robot.launch.py \
  enable_amr_bridge:=false \
  amr_bot_id:=hardware-bot \
  amr_bridge_python:=auto
```

### `autonomous_robot.launch.py`

Includes `launch_robot.launch.py` with `expect_frontier:=true`, then starts
`frontier_explorer_node` at T=22 s.

| Argument | Default | |
|---|---|---|
| `verbose` | `false` | Node logs on the terminal when true |
| `enable_amr_bridge` | `true` | Passed through to `launch_robot` |

### `amr_bridge.launch.py`

Included by `launch_robot` (also runnable on its own). Same file is copied at
`amr-bridge/launch/amr_bridge.launch.py`.

| Argument | Default | |
|---|---|---|
| `bot_id` | `hardware-bot` | Overrides `AMR_BOT_ID` from the env file |
| `env_file` | `/etc/amr-bridge.env` | |
| `dashboard` | `http://127.0.0.1:8080` | Overrides `AMR_DASHBOARD_URL` |
| `gamepad` | `http://127.0.0.1:8765` | Overrides `AMR_GAMEPAD_URL`. `none`, `off`, or `-` unsets it so cloud joy uses the dashboard nudge instead |
| `python` | `auto` | Same rule as `amr_bridge_python` |

`launch_robot` passes `gamepad:=none` unless it actually started `web_gamepad`.

### `vision.launch.py`

Not part of bringup. The dashboard starts it, or:

```bash
ros2 launch Ros_lidar_bot vision.launch.py
```

| Argument | Default | |
|---|---|---|
| `camera` | `0` | Picamera2 camera number or OpenCV index |
| `conf` | `0.25` | YOLO confidence |
| `rate` | `5.0` | Max inference FPS |
| `model` | `~/.cache/Ros_lidar_bot/yolov8s-world.pt` | Downloaded on first run |
| `backend` | `auto` | `auto`, `picamera2`, or `cv2` |
| `flip` | `true` | Rotate the image 180° (camera mounted upside-down) |

If the YOLO process exits, this launch shuts down with it.

### `rsp.launch.py`

Included by bringup. `use_sim_time` default `false`, `output` default `log`.
`joint_state_publisher` merges `/encoder` and `/camera_joint_states`.

### `lidar_test.launch.py`

Lidar plus a static TF, no motors. `channel_type` `udp`, `udp_ip`
`192.168.11.2`, `udp_port` `8089`, `frame_id` `laser_frame`, `inverted`
`false`, `angle_compensate` `true`, `scan_mode` `DenseBoost`.

---

## Other pieces

**Dashboard** (`robot_dashboard`, port 8080). Map, scan, trail, semantic
labels, frontier queue, click-to-send a Nav2 goal, no-go circles for the
frontier explorer, CPU/RAM/temperature and Pi throttle flags, save map
(`.pgm`/`.yaml` and a pose-graph under `~/maps/`), and on-demand IMU test,
IMU calibration, and drive-distance. The vision toggle starts and stops
`vision.launch.py`. E-STOP latches `/emergency_stop` for 1.5 s so
`safety_stop` zeros `/cmd_vel_safe` ahead of Nav2 and teleop, publishes
zero `/cmd_vel`, cancels the nav goal, and kills the running tool. It
leaves the vision pipeline up. A nudge during that window is ignored.

**SLAM.** `slam_toolbox` `online_async_launch.py` with
`config/mapper_params_online_async.yaml`, started 5 s after bringup.

**Nav2.** `nav2_bringup` `navigation_launch.py` with `config/nav2_params.yaml`,
started at 8 s. `transform_tolerance` is 1.0 s because DenseBoost scan matching
on the Pi lags by about 0.6 s.

**Frontier explorer.** Only `autonomous_robot.launch.py`. It subscribes to
`/map`, `/safety_blocked`, and `/manual_blacklist`. Parameters:
`config/frontier_explorer.yaml`.

**Camera head.** `camera_servo` follows `/camera_cmd` from `joy_teleop` or
the keyboard teleop.

**YOLO / semantic map.** `yolo` publishes JSON on `/yolo`. `semantic_slam_node`
projects detections with `/scan` and TF `map` → `base_link`, and publishes
`/semantic_markers` plus `/tmp/semantic_map.json`. Needs `ultralytics`.

**Safety stop.** Parameters in `launch_robot.launch.py`: `min_safe_distance`
0.35 m, `ignore_below` 0.30 m, front and rear openings 50°, `clear_margin`
0.10 m, `odom_raw_timeout_sec` 0.5 s.

**Cloud bridge.** Off by passing `enable_amr_bridge:=false`. On by default.
Setup, the env file, and rollback: [scripts/README.md](scripts/README.md).
The longer bridge notes are `amr-bridge/PI_SETUP.md` and
`amr-bridge/AGENT_SETUP_AND_INTEGRATION.md`. No secrets belong in this repo.

---

## Topics

| Topic | Type | Path |
|---|---|---|
| `/cmd_vel` | Twist | Nav2, teleop, dashboard, drive-distance, IMU calibration → safety_stop |
| `/cmd_vel_safe` | Twist | safety_stop only → driver_node |
| `/emergency_stop` | Bool | dashboard E-STOP → safety_stop (zeros `/cmd_vel_safe` while true) |
| `/safety_blocked` | Bool | safety_stop → frontier explorer |
| `/encoder` | JointState | driver_node → odom_node and joint_state_publisher |
| `/odom_raw` | Odometry | odom_node → EKF |
| `/odom` | Odometry | EKF → Nav2 and dashboard |
| `/imu` | Imu | imu_node → EKF |
| `/scan_raw` | LaserScan | sllidar_node → scan filter |
| `/scan` | LaserScan | scan filter → SLAM, Nav2, safety_stop |
| `/map` | OccupancyGrid | slam_toolbox → Nav2, frontier, dashboard |
| `/yolo` | String (JSON) | yolo → semantic_slam |
| `/semantic_markers` | MarkerArray | semantic_slam → dashboard |
| `/frontier_debug` | MarkerArray | frontier_explorer → dashboard |
| `/manual_blacklist` | String (JSON) | dashboard → frontier_explorer |
| `/joy` | Joy | joy_node or web_gamepad → joy_teleop |
| `/camera_cmd` | Twist | joy_teleop or keyboard teleop → camera_servo (`angular.z` pan, `angular.y` tilt) |
| `/camera_joint_states` | JointState | camera_servo → joint_state_publisher |

TF: `map` → `odom` (SLAM) → `base_footprint` (EKF) → `base_link` and the
lidar, IMU, and wheel frames (URDF).

---

## Config files

- `config/ekf.yaml` — wheel odometry is the main source; the gyro complements yaw.
- `config/nav2_params.yaml` — `transform_tolerance: 1.0` for the DenseBoost lag.
- `config/mapper_params_online_async.yaml` — SLAM Toolbox.
- `config/frontier_explorer.yaml` — exploration scoring and blacklist.

DenseBoost is the scan mode set in `launch_robot.launch.py`. It is the full
S2E resolution and costs CPU on the Pi.

---

## Common problems

| What you see | What it usually is |
|---|---|
| Dashboard TF `map`→`odom` or `odom`→`base_footprint` missing | A previous launch still holds the serial ports or port 8080. Stop it and launch again. |
| Driver is up, robot does not move | The driver only listens to `/cmd_vel_safe`. Check `ros2 topic hz /cmd_vel_safe`. |
| Safety stop says an obstacle is ahead | A return is inside 0.35 m in the front opening. The robot can still rotate. |
| No `/scan` | `ping 192.168.11.2` and check the `eth0` address. |
| No `/encoder` or `/imu` | `/dev/ttyACM0` and `/dev/ttyACM1` swapped. See `/dev/serial/by-id/`. |
| Web gamepad and a real pad fight | Do not run both. `auto` already picks one. `web_gamepad` is only the branch with no `js*` device, or `use_joystick:=false`. |
| Bridge did not start | `enable_amr_bridge` defaults to true, but a missing env file or a Python without `supabase` skips the bridge and leaves the robot up. See [scripts/README.md](scripts/README.md). |
