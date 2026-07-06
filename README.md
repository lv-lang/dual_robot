# RDK X5-Based Heterogeneous Ackermann-Mecanum Dual-Robot Cooperative Dispatch and Perception System

This repository contains the source code for an RDK X5-based heterogeneous Ackermann-Mecanum dual-robot cooperative dispatch and perception system. The system is organized into three runtime workspaces:

- PC dispatch and Web App workspace
- mecanum robot controller workspace
- ackermann robot controller workspace

Chinese documentation: [README_cn.md](README_cn.md)

## Repository Layout

```text
pc/robot_ws/                 PC-side ROS 2 workspace and Web App source
mecanum/mecanum_ws/          mecanum robot controller ROS 2 workspace source
ackermann/ackermann_ws/      ackermann robot controller ROS 2 workspace source
```

Only source code and required configuration files are included. Generated build output, installed files, logs, caches, dependency directories, archives, and local runtime data are excluded.

## System Overview

The system uses two robots and one PC:

- `mecanum`: delivery robot using a mecanum chassis.
- `ackermann`: inspection robot using an ackermann chassis.
- `pc`: dispatch control plane, map display, Web App backend/frontend, event logs, task coordination, and RViz display.

The PC-side control plane starts from:

```bash
ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true
```

The Web App backend starts from:

```bash
ros2 launch robot_web robot_web.launch.py
```

Robot-side launch files are kept inside each controller workspace and should be built and sourced on the corresponding robot controller.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Node.js 18 or newer for the Web App
- Nav2, RViz2, Gazebo, and the robot-side driver dependencies required by each workspace

## Build the PC Workspace

```bash
cd pc/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Frontend development:

```bash
cd pc/robot_ws/apps/robot-control-pwa
npm install
VITE_ROBOT_WEB_TARGET=http://127.0.0.1:8000 npm run dev -- --host 0.0.0.0
```

## Build the mecanum Workspace

```bash
cd mecanum/mecanum_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Build the ackermann Workspace

```bash
cd ackermann/ackermann_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Notes

This repository is intended as a source release. It does not include `build/`, `install/`, `log/`, `node_modules/`, generated frontend `dist/`, rosbag data, local archives, or machine-specific caches.
