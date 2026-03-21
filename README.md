<div align="center">
  <h1>🤖 Ros Lidar Bot 🤖</h1>
  <p><h3><i>Building cutting-edge ROS2 Humble + Gazebo Ignition kinematics, one node at a time.</i> 🚀</h3></p>
</div>

&nbsp;

> 🚨 **CRITICAL VERSION SPECS** 🚨
> Do not attempt to run this setup natively on other distributions! This repository contains code dynamically built **ONLY** for:
> * **ROS2 (Distro: Humble Hawkbill)** 🐢
> * **Gazebo Ignition 🚀** 
> 
> Trying to run this on ROS Noetic, Windows, or Gazebo Classic will fail due to deprecated XML plugins. Wait... are you re-forking this to melodic? Good luck with workspace rebuilding! 🎯

&nbsp;

## 🎯 What Is This Repository?

Looking for a fast, hassle-free way to simulate autonomous environments? Say no more! 

This repository ships with a comprehensive and dynamic **differential drive mobile robot**, equipped out of the box with an accurate 2D LiDAR scanner plugin. It is entirely setup for:
- 🗺️ **Simultaneous Localization and Mapping (SLAM)** built via SLAM Toolbox!
- 🦿 **Physics-Accurate URDF & Xacro Base Lines!**
- 🕹️ **Custom Nav2 Control Schemes!**
 
&nbsp;

## 🛠️ Installation & Building (Colcon)

Need a fresh workspace initialized or have an existing one? Just run these magic formulas and you are all set! 🪄

### 1️⃣ Build The Foundation
Setup your `src` workspace framework if you do not have one ready yet:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2️⃣ Clone Your Fleet
Assuming you are positioned snugly within that `~/ros2_ws/src` folder:
```bash
git clone <YOUR-URL-HERE> ros_lidar_bot
```

### 3️⃣ Construct & Compile 👷‍♂️ 
Build via `colcon` inside the workspace parent folder! (The symlink-install flag ensures dynamic script updating instead of reloading 1000 times!):
```bash
cd ~/ros2_ws 
colcon build --symlink-install
```

### 4️⃣ Source The Flow 🍃 
Once `build`, `log`, and `install` directories pop up, finalize it by sourcing your Linux path context variables:
```bash
source install/setup.bash
```

&nbsp;

## 🚀 Engine Ignition & Launch Setup

You're a couple terminal codes away from rendering physics!

### ⚙️ Main Robot Launch 
Activate the Gazebo Simulator and Robot State Publisher:
```bash
ros2 launch ros_lidar_bot launch_sim.launch.py
```

### 🌎 Testing In the Wild (Simulated Worlds)
We included more than just a boring empty floor. Want to stress test the collision dynamics? Feel free to swap your gazebos execution with any included `.sdf` file!
Included within `/worlds`:
- 🕳️ `empty.world`
- 📏 _Narrow local planning_ > `corridor_world.sdf`
- 🏁 _Endless dead ends_ > `maze_world.sdf`
- 🏢 _Desktops & chairs_ > `office_world.sdf`
- 📦 _The dreaded cage_ > `room_world.sdf`
- 💥 _Collision hazards_ > `open_obstacles_world.sdf`
- 🏗️ _Shelves on shelves_ > `warehouse_world.sdf`

### 🧪 SLAM Accuracy Test Worlds
10 creative worlds for benchmarking SLAM mapping accuracy. Each is a 20×20m arena — compare the SLAM map against the known layout to judge accuracy!

```bash
ros2 launch ros_lidar_bot launch_sim.launch.py world:=star
```

| World | What's Inside | Tests |
|---|---|---|
| `circles` | Spiral of cylinders (increasing radii) | Curved surface detection |
| `boxes` | Tetris-shaped rectangles | Straight edges & right angles |
| `zigzag` | Snaking corridor with wall teeth | Tight corridor navigation |
| `spiral` | 4-ring spiral labyrinth | Multi-turn mapping |
| `star` | 5-pointed gold star | Diagonal edge detection |
| `cross` | Plus-sign wall dividing 4 rooms | Room connectivity |
| `arena` | Octagonal boundary + 8 pillars | Non-rectangular boundaries |
| `grid` | 4×4 checkerboard of shapes | Repeating pattern handling |
| `scatter` | 14 random shapes & angles | Chaotic environment |
| `castle` | Castle with towers, gate, keep | Complex multi-feature scene |

### 🎮 Teleoperation Keyboard Control
Control the wheel velocities via WASD (Cmd_Vel overrides)! 🏎️
```bash
ros2 run ros_lidar_bot teleop_node
```

&nbsp;

## 📂 The Complete File Map 

Wondering how the puzzle fits together? Here's the layout and breakdown!

#### ⚙️ `config/`
* `mapper_params_online_async.yaml`: Fast real-time processing variables tuning our asynchronous SLAM mapping nodes.
* `nav2_params.yaml`: Brains of the Path Planner! Modifies the costmaps and DWB collision zones.
* `view_bot.rviz`: Custom Rviz plugin configurations for map boundaries, transforms, and Urdf.

#### 🤖 `description/`
* `robot_core.xacro` & `robot.urdf.xacro`: Structural parameters representing our chassis, links, wheels, weights, and joint rotations! 
* `lidar.xacro`: Gazebo Ray-tracing plugins connecting an invisible scanner sensor.
* `robot_control.xacro`: The `diff_drive` plugin that applies torque to your simulated wheels.
* `inertial_macros.xacro`: Standard geometrical physics math definitions.

#### 🚀 `launch/`
* `rsp.launch.py`: Broadcasts your entire models TF Tree parameters.
* `launch_sim.launch.py`: Coordinates the Gazebo node with your robot's model spawn vector.

#### 🐍 `ros_lidar_bot/`
* `teleop_node.py`: A native ROS python-node broadcasting twists keyboard interrupts.

#### 🌍 `worlds/`
* Multiple map `.sdf` parameters generated directly inside Gazebo!

&nbsp;

## �� The Future Roadmap (TODO)
We never stop updating! Some upcoming milestone prospects:
- 👾 **Computer Vision Upgrades:** Attaching a depth camera URDF layer + OpenCV filtering.
- 🏎️ **Hardware Overhaul:** Transplanting micro-ros nodes toward a physical Raspberry-Pi driven platform.
- 👯 **Fleet Operations:** Adding dynamic multi-namespace instances for simultaneous maze tracking.

<div align="center">
  <i>Happy coding and building! 🖥️✨</i>
</div>
