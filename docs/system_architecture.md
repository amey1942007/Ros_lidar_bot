# System Architecture — Topic, TF & Action Map

Complete reference for every component in the Ros_lidar_bot stack: what each node
subscribes to, what it publishes, and how they connect to each other.

---

## How to read this document

| Symbol | Meaning |
|--------|---------|
| `→ publishes` | Node puts data onto this topic |
| `← subscribes` | Node reads from this topic |
| `⟷ action` | Request / response between two nodes |

---

## 1. Ignition Gazebo (simulation engine)

The physics simulator. Runs the robot model, sensors, and world.

**Publishes** (Ignition internal topics — picked up by the bridge):

| Ignition topic | Plugin that produces it |
|---|---|
| `odom` | DiffDrive plugin (wheel encoder odometry) |
| `scan` | GPU LiDAR plugin |
| `imu` | IMU plugin |
| `camera/image_raw` | Camera plugin |
| `camera/camera_info` | Camera plugin |
| `joint_states` | JointStatePublisher plugin |
| `clock` | Gazebo simulation clock |

**Subscribes**:

| Ignition topic | Who sends it |
|---|---|
| `cmd_vel` | ros_gz_bridge (velocity commands drive the DiffDrive plugin) |

---

## 2. ros_gz_bridge (parameter_bridge)

Translates between Ignition Gazebo topics and ROS 2 topics.

**Publishes to ROS 2**:

| ROS 2 topic | Source Ignition topic | Notes |
|---|---|---|
| `/odom_raw` | `odom` | Remapped: bridge calls it `/odom`, Node remapping renames it |
| `/scan` | `scan` | |
| `/imu` | `imu` | |
| `/camera/image_raw` | `camera/image_raw` | |
| `/camera/camera_info` | `camera/camera_info` | |
| `/joint_states` | `joint_states` | |
| `/clock` | `clock` | |

**Subscribes from ROS 2**:

| ROS 2 topic | Who publishes it | Notes |
|---|---|---|
| `/cmd_vel_safe` | safety_stop_node | Bridge argument says `/cmd_vel`; Node remapping redirects to `/cmd_vel_safe` so the safety filter always sits between Nav2 and the robot |

**Direction summary**:
```
Gazebo → ROS 2:  odom, scan, imu, camera/*, joint_states, clock
ROS 2 → Gazebo:  /cmd_vel_safe → Gazebo cmd_vel → DiffDrive plugin
```

---

## 3. robot_state_publisher (RSP)

Reads the URDF and current joint positions → publishes the full TF tree for every link.

**Subscribes**:

| Topic | Publisher |
|---|---|
| `/joint_states` | ros_gz_bridge |

**Publishes**:

| Topic / TF | Consumers |
|---|---|
| `/robot_description` | RViz2 (RobotModel display) |
| `/tf` — `base_footprint → base_link` | All nodes that need robot geometry |
| `/tf` — `base_link → laser_frame` | SLAM Toolbox, RViz2 |
| `/tf` — `base_link → camera_link → camera_link_optical` | RViz2 |
| `/tf` — `base_link → imu_for_urdf_1` | EKF (frame reference) |
| `/tf` — `base_link → wheel_urdf_1` | RViz2 |
| `/tf` — `base_link → wheel_urdf__1__1` | RViz2 |
| `/tf_static` | Fixed joints that never change |

---

## 4. ekf_filter_node (robot_localization EKF)

Fuses wheel odometry + IMU into clean, drift-reduced odometry.

**Subscribes**:

| Topic | Publisher | What it uses |
|---|---|---|
| `/odom_raw` | ros_gz_bridge | x, y position and yaw velocity from wheel encoders |
| `/imu` | ros_gz_bridge | Angular velocity + linear acceleration for orientation |

**Publishes**:

| Topic / TF | Consumers |
|---|---|
| `/odom` (remapped from `/odometry/filtered`) | controller_server, bt_navigator |
| `/tf` — `odom → base_footprint` | **Critical transform** — used by SLAM, Nav2, and frontier explorer to know where the robot is |

---

## 5. slam_toolbox

Builds the occupancy-grid map and anchors it to the world frame.

**Subscribes**:

| Topic | Publisher | What it uses |
|---|---|---|
| `/scan` | ros_gz_bridge | Laser hits to build and update the map |
| `/tf` | EKF + RSP | Needs `odom → base_footprint` to know the robot's pose at each scan time |

**Publishes**:

| Topic / TF | Consumers |
|---|---|
| `/map` (OccupancyGrid, every 3 s) | global_costmap, frontier_explorer, RViz2 |
| `/tf` — `map → odom` | **Root of the nav TF chain** — tells all nodes where the map frame sits relative to odometry |

**Cell values in the OccupancyGrid**:

| Value | Meaning |
|---|---|
| `0` | Free — laser passed through here, robot can drive here |
| `100` | Occupied — laser hit something (wall, furniture) |
| `-1` | Unknown — laser has never reached this cell |

---

## 6. Nav2 stack

### Internal call chain

```
bt_navigator
  ├── action client → planner_server   (compute_path_to_pose)
  ├── action client → controller_server (follow_path)
  └── action client → behavior_server  (spin / backup / wait)
```

---

### 6a. global_costmap

Inflated version of the SLAM map used for global path planning.

**Subscribes**:

| Topic | Plugin | Purpose |
|---|---|---|
| `/map` | `static_layer` | Base obstacle map from SLAM |
| `/scan` | `obstacle_layer` | Live obstacle updates (furniture moved, etc.) |
| `/tf` | — | Projects scan readings into map frame |

**Publishes**:

| Topic | Consumers |
|---|---|
| `/global_costmap/costmap` | planner_server, RViz2 |
| `/global_costmap/costmap_raw` | — |
| `/global_costmap/published_footprint` | RViz2 |

---

### 6b. planner_server (NavFn A*)

Computes a collision-free global path from robot position to goal.

**Subscribes**:

| Topic | Publisher |
|---|---|
| `/global_costmap/costmap` | global_costmap |
| `/tf` | EKF + SLAM (robot pose in map frame) |

**Publishes**:

| Topic | Consumers |
|---|---|
| `/plan` | controller_server, RViz2 |

**Action server**:

| Action | Called by |
|---|---|
| `compute_path_to_pose` | bt_navigator |

---

### 6c. local_costmap

Small rolling window around the robot for real-time obstacle avoidance.

**Subscribes**:

| Topic | Plugin | Purpose |
|---|---|---|
| `/scan` | `obstacle_layer` | Live laser data for obstacle marking |
| `/tf` | — | Robot pose to centre the rolling window |

**Publishes**:

| Topic | Consumers |
|---|---|
| `/local_costmap/costmap` | controller_server, RViz2 |
| `/local_costmap/costmap_raw` | behavior_server |
| `/local_costmap/published_footprint` | behavior_server, RViz2 |

---

### 6d. controller_server (Regulated Pure Pursuit)

Follows the global path while avoiding local obstacles in real time.

**Subscribes**:

| Topic | Publisher | Purpose |
|---|---|---|
| `/local_costmap/costmap_raw` | local_costmap | Collision checking ahead |
| `/plan` | planner_server | Path to follow |
| `/odom` | EKF | Current speed for velocity scaling |
| `/tf` | EKF + SLAM | Robot pose relative to path |

**Publishes**:

| Topic | Consumer |
|---|---|
| `/cmd_vel` | safety_stop_node |

**Action server**:

| Action | Called by |
|---|---|
| `follow_path` | bt_navigator |

---

### 6e. behavior_server (spin / backup / wait)

Recovery behaviours triggered when the robot is stuck.

**Subscribes**:

| Topic | Publisher |
|---|---|
| `/local_costmap/costmap_raw` | local_costmap |
| `/local_costmap/published_footprint` | local_costmap |
| `/tf` | EKF + SLAM |

**Publishes**:

| Topic | Consumer | Notes |
|---|---|---|
| `/cmd_vel` | safety_stop_node | Rotation / backup commands during recovery |

**Action servers**:

| Action | Called by |
|---|---|
| `spin` | bt_navigator |
| `backup` | bt_navigator |
| `wait` | bt_navigator |

---

### 6f. bt_navigator (Behavior Tree Navigator)

Top-level navigator that orchestrates planning, control, and recovery.

**Subscribes**:

| Topic | Publisher | Purpose |
|---|---|---|
| `/odom` | EKF | Checks if the robot has reached the goal |
| `/tf` | EKF + SLAM | Robot pose relative to goal |

**Action server** (entry point for all navigation requests):

| Action | Called by |
|---|---|
| `navigate_to_pose` | frontier_explorer_node |

**Action clients** (internal orchestration):

| Action | Server |
|---|---|
| `compute_path_to_pose` | planner_server |
| `follow_path` | controller_server |
| `spin` / `backup` / `wait` | behavior_server |

---

## 7. safety_stop_node (custom)

Hardware-critical velocity filter. The last gate before any velocity command
reaches the robot. Works identically in simulation and on real hardware.

**Subscribes**:

| Topic | Publisher | QoS |
|---|---|---|
| `/scan` | ros_gz_bridge | BEST_EFFORT |
| `/cmd_vel` | controller_server OR behavior_server OR teleop_node | RELIABLE |

**Publishes**:

| Topic | Consumer |
|---|---|
| `/cmd_vel_safe` | ros_gz_bridge |

**Safety logic**:

| Condition | Action |
|---|---|
| Obstacle < 0.25 m in front arc (±60°) and `linear.x > 0` | Set `linear.x = 0` — block forward motion |
| Obstacle < 0.25 m in rear arc (±30°) and `linear.x < 0` | Set `linear.x = 0` — block reverse motion |
| `angular.z` (rotation) | Always passes through — Nav2 spin recovery still works |
| Path is clear | All commands pass through unmodified |

---

## 8. frontier_explorer_node (custom)

Detects unexplored map regions and autonomously sends navigation goals until
the entire environment is mapped.

**Subscribes**:

| Topic | Publisher | Purpose |
|---|---|---|
| `/map` | slam_toolbox | Finds frontier cells (unknown cells adjacent to free cells) |
| `/tf` | EKF + SLAM | Looks up `map → base_footprint` to know robot's current position |

**Action client**:

| Action | Server | Notes |
|---|---|---|
| `navigate_to_pose` | bt_navigator | Sends frontier goal poses one at a time |

**No direct velocity publishing** — the frontier explorer only decides WHERE to go;
Nav2 handles all movement.

**Frontier cell definition**: an unknown cell (`-1`) that has at least one free (`0`)
neighbour. These appear at the edge of explored space where the laser scan hasn't reached yet.

---

## 9. teleop_node (custom)

Keyboard-controlled manual driving.

**Publishes**:

| Topic | Consumer |
|---|---|
| `/cmd_vel` | safety_stop_node |

**Subscribes**: nothing.

| Key | Action |
|---|---|
| `i` | Increase forward speed |
| `k` | Increase reverse speed |
| `j` | Turn left |
| `l` | Turn right |
| `Space` | Full stop |

---

## 10. RViz2 (visualisation only — no robot effect)

**Subscribes**:

| Topic | Publisher | Display |
|---|---|---|
| `/map` | slam_toolbox | Occupancy grid map |
| `/scan` | ros_gz_bridge | Laser scan hits (red dots) |
| `/robot_description` | RSP | 3D robot model |
| `/tf` + `/tf_static` | RSP + EKF + SLAM | All coordinate frames |
| `/camera/image_raw` | ros_gz_bridge | Camera feed panel |
| `/global_costmap/costmap` | global_costmap | Inflation around obstacles |
| `/local_costmap/costmap` | local_costmap | Local rolling window |
| `/plan` | planner_server | Planned path line |

---

## Complete data-flow diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│  IGNITION GAZEBO                                                     │
│  DiffDrive → odom ──┐   LiDAR → scan ──┐   IMU → imu ──┐          │
│  Camera → image ────┤   JointPub → js ─┤   Clock ───────┤          │
└────────────────────┼───────────────────┼────────────────┼──────────┘
                     ↓                   ↓                ↓
              ┌──────────────────────────────────────────────────┐
              │  ros_gz_bridge                                   │
              │  /odom_raw  /scan  /imu  /camera/*  /js  /clock │
              └──┬─────┬──────────────────────────────┬─────────┘
                 │     │                               │
         /odom_raw  /scan,/imu                    /joint_states
                 │     │                               │
                 ↓     ↓                               ↓
           ┌─────────┐  ┌────────────────────┐  ┌────────────────┐
           │   EKF   │  │   slam_toolbox     │  │      RSP       │
           └────┬────┘  └────────┬───────────┘  └───────┬────────┘
                │                │                       │
           /odom + /tf      /map + /tf              /tf + /robot_description
         (odom→base)     (map→odom)             (all joint TFs)
                │                │                       │
                └────────────────┴───────────────────────┘
                                 │ (TF tree: map→odom→base_footprint→base_link→sensors)
                                 │
              ┌──────────────────┴─────────────────────────────────┐
              │              Nav2 Stack                             │
              │                                                     │
              │  /scan+/map → global_costmap → /global_costmap/*   │
              │  /scan     → local_costmap  → /local_costmap/*     │
              │                                                     │
              │  frontier_explorer ──navigate_to_pose──→ bt_navigator
              │                                              │      │
              │                          compute_path_to_pose↓      │
              │                             planner_server          │
              │                             → /plan                 │
              │                                    ↓ follow_path    │
              │                          controller_server          │
              │                             → /cmd_vel             │
              │                                    │               │
              │                    spin/backup/wait↓               │
              │                          behavior_server           │
              │                             → /cmd_vel             │
              └──────────────────────────────────┬────────────────┘
                                                 │
                                            /cmd_vel
                                                 │
                                   ┌─────────────▼───────────────┐
                                   │      safety_stop_node        │
                                   │  (/scan < 0.25m → block)    │
                                   └─────────────┬───────────────┘
                                                 │
                                           /cmd_vel_safe
                                                 │
                                   ┌─────────────▼───────────────┐
                                   │       ros_gz_bridge          │
                                   │   (ROS /cmd_vel_safe         │
                                   │    → Gazebo cmd_vel)         │
                                   └─────────────┬───────────────┘
                                                 │
                                   ┌─────────────▼───────────────┐
                                   │    Gazebo DiffDrive plugin   │
                                   │       → robot moves          │
                                   └─────────────────────────────┘
```

---

## TF tree

```
map  (SLAM Toolbox publishes map→odom)
 └─ odom  (EKF publishes odom→base_footprint)
     └─ base_footprint  (RSP publishes everything below)
         └─ base_link
             ├─ laser_frame
             ├─ camera_link
             │   └─ camera_link_optical
             ├─ imu_for_urdf_1
             ├─ wheel_urdf_1        (right drive wheel)
             └─ wheel_urdf__1__1    (left drive wheel)
```

---

## Staged startup order

| Time | What starts | Why |
|---|---|---|
| T = 0 s | Gazebo, RSP, bridge, safety_stop | Foundation — simulation and safety filter must be first |
| T = 3 s | spawn_entity | Gazebo needs ~3 s to finish loading the world |
| T = 6 s | EKF, SLAM Toolbox | Robot is spawned; sensors now publishing |
| T = 16 s | Nav2 stack | SLAM has had 10 s to build an initial map for the costmaps |
| T = 30 s | frontier_explorer | Nav2 needs ~14 s to warm up after starting |
