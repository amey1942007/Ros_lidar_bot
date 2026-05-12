# 🤖 SEMANTIC SLAM

> A ROS2-based autonomous differential drive robot equipped with LiDAR for obstacle detection and navigation in Gazebo simulation.

---

## 📌 Table of Contents

- [📖 About](#-about)
- [🎯 Goals](#-goals)
- [✅ Progress Checkpoints](#-progress-checkpoints)
- [🛠️ Tech Stack](#-tech-stack)
- [📂 Project Structure](#-project-structure)
- [⚙️ Installation](#️-installation)
- [▶️ Usage](#️-usage)
- [📡 ROS2 Topics](#-ros2-topics)
- [📸 Demo / Results](#-demo--results)
- [🔮 Future Improvements](#-future-improvements)
- [🤝 Contributing](#-contributing)
- [📜 License](#-license)

---

## 📖 About

The **SEMANTIC SLAM** project focuses on building and simulating a mobile robot capable of:

- Perceiving its environment using LiDAR sensors  
- Publishing scan data in ROS2  
- Performing differential drive motion control  
- Enabling autonomous navigation using the Nav2 stack  

This project serves as both a learning platform and a foundation for real-world robotics deployment.

---

## 🎯 Goals

### Primary Objectives

- Build a working LiDAR-enabled robot simulation  
- Implement obstacle detection and avoidance  
- Integrate ROS2 Navigation (Nav2) for autonomous movement
- testing Slam_toolbox , Cartographer , G-Mapping.
- Integrate ML to predict the environmental features.

### Learning Outcomes

- Understanding ROS2 nodes, topics, TF frames  
- Working with URDF/Xacro robot models  
- Using Gazebo plugins for sensors and motion  
- Practicing SLAM + Path Planning pipelines
- Different SLAM algorithms such as Pose-Graph optimisation etc.

---

## ✅ Progress Checkpoints

### Phase 1: Environment Setup

- [x] Create ROS2 workspace and repository  
- [x] Setup Gazebo Classic simulation environment  
- [x] Build differential drive base model  

---

### Phase 2: Robot Description (URDF/Xacro)

- [x] Add chassis and wheel links  
- [x] Configure joints properly  
- [x] Improve inertial and collision properties  

---

### Phase 3: LiDAR Integration

- [x] Add LiDAR sensor in URDF  
- [x] Verify `/scan` topic publishing  
- [ ] Tune sensor parameters for accuracy  

---

### Phase 4: Motion Control

- [x] Configure diff-drive Gazebo plugin  
- [x] Publish velocity commands via `/cmd_vel`  
- [x] Validate odometry output  

---

### Phase 5: Navigation & Autonomy

- [x] Integrate SLAM Toolbox
- [ ] Integrate Cartographer
- [ ] Integrate G-Mapping
- [ ] Configure Nav2 stack  
- [ ] Achieve goal-to-goal autonomous navigation  

---

### Phase 6: Train Model , introduce Semantic Slam

- [x] Run obstacle avoidance benchmarks  
- [ ] Improve stability and localization  
- [ ] Add documentation + demo videos  

---

### Phase 7: Implement on Hardware

- [ ] Run obstacle avoidance benchmarks  
- [ ] Improve stability and localization  
- [ ] Add documentation + demo videos  

---

## 🛠️ Tech Stack

| Category        | Tools Used |
|----------------|------------|
| Framework       | ROS2 Humble |
| Simulation      | Gazebo Fortress Ignition |
| Robot Model     | URDF / Xacro |
| Sensors         | LiDAR || IMU || Camera |
| Control         | Diff Drive Plugin |
| Navigation      | SLAM Toolbox + Nav2 (Planned) |
| Version Control | Git + GitHub |

---

## 📂 Project Structure

```bash
ros2_lidar_bot/
│── src/
│   └── lidar_bot_description/
│       ├── urdf/
│       ├── launch/
│       ├── config/
│       ├── worlds/
│       └── meshes/
│
│── README.md
│── package.xml
│── setup.py
```
---

## ⚙️ Installation

- ROS2 Humble (Go for Desktop Install) : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Gazebo Fortress Ignition : https://gazebosim.org/docs/fortress/install_ubuntu/
- Note : Make sure you have selected Fortress LTS in top right Corner , and Binary Installation.

