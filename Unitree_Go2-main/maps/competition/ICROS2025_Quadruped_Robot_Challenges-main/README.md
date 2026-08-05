# ICROS 2025 Quadruped Robot Challenge Simulation Map

<div style="display:flex;">
<div style="flex:50%; padding-right:10px; border-right: 1px solid #dcdde1">

**Package summary**

This repository provides a simulation map file for the ICROS 2025 QRC (Quadruped Robot Challenge).


**🏆 ICROS 2025 QRC Competition Results**
- **Grand Prize**: Korea Electronics Technology Institute Snobotics_go2 Team (1:52.41)

</div>
<div style="flex:40%; padding-left:10px;">

**Table of Contents**
- [Overview](#overview)
- [Related Links](#related-links)
- [Installation methods](#installation-methods)
    - [1. ROS2](#1-ros2)
    - [2. Dependencies](#2-dependencies)
    - [3. Gazebo](#3-gazebo)
    - [4. Usage](#4-usage)

</div>
</div>

---

## Overview

- A repository for ICROS 2025 Quadruped Robot Challenge simulation environment
- This repository provides a Gazebo world file for the QRC competition map
- Used for build, test, and deployment of quadruped robot algorithms

---

## Related Links

### 🏆 Competition Results and Videos
- **[Grand Prize Winner Video (KETI Snobotics_go2 Team)](https://www.youtube.com/watch?v=bHU_1xm5e2g)**
- **[Snobotics Labs Official Website](https://cypress-persimmon-6c5.notion.site/Home-1a9ee36fc61c8024951dd1021f555640)**
- **[Robot News Article: First Domestic Quadruped Robot Competition Completed](https://www.irobotnews.com/news/articleView.html?idxno=40871)**

### 📊 Competition Overview
- **Participating Teams**: 20 teams
- **Competition Format**: Autonomous navigation and remote control
- **Competition Period**: June 25-27, 2025
- **Venue**: Jeonbuk National University International Convention Center

---

## Installation methods

This tutorial provides a method to import the Gazebo world file for the ICROS 2025 QRC competition environment.

#### 1. ROS2

Tested on ROS2 Humble (Ubuntu 22.04). [ROS2 Install](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

#### 2. Dependencies

```bash
sudo apt-get install ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-urdf ros-$ROS_DISTRO-urdf-tutorial
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher
```

#### 3. Gazebo

```bash
sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-gazebo-ros-control
```

#### 4. Usage

Clone the package
```bash
cd $ROS_WORKSPACE/src
git clone https://github.com/snobotics/ICROS2025_Quadruped_Robot_Challenges.git
```

Build & install
```bash
cd ..
colcon build --packages-select ICROS2025_QRC_Simulation_Map
```

Source the workspace
```bash
source install/setup.bash
```

Launch the simulation environment
```bash
ros2 launch ICROS2025_QRC_Simulation_Map gazebo_world.launch.py
```

---

## Package Structure

```
ICROS2025_QRC_Simulation_Map/
├── launch/
│   └── gazebo_world.launch.py
├── worlds/
│   └── QRC_map.world
├── meshes/
│   ├── fullmap_ground_wall.obj
│   └── fullmap_ground_wall.mtl
├── CMakeLists.txt
├── package.xml
└── README.md
```
