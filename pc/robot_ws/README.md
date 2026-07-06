# Dual-Robot Delivery and Inspection System

This project is a ROS 2 Humble based heterogeneous dual-robot delivery and inspection system for indoor competition demos and real-robot deployment. It includes a mecanum delivery robot, an ackermann inspection robot, a PC-side dispatch control plane, a Web App operator interface, map visualization, task dispatching, event logs, and interfaces for vision detection results.

Chinese documentation: [README_cn.md](README_cn.md)

## Features

- Distributed ROS 2 communication with isolated robot namespaces
- Delivery missions for the mecanum robot
- Inspection missions for the ackermann robot
- PC-side dispatching, resource locking, state aggregation, and system control
- Vue 3 PWA for task creation, system status, event logs, and live video display
- Real-robot map, pickup, delivery, and inspection point configuration
- Vision event integration for flame, smoke, and unstable cargo stacking detection
- Gazebo/Nav2 validation entry points for development regression testing

## Repository Layout

```text
apps/robot-control-pwa/     Web App frontend
src/robot_bringup/          PC and simulation launch orchestration
src/robot_dispatch/         Task dispatching and system state management
src/robot_interfaces/       Custom ROS msg/srv/action definitions
src/robot_mission/          Per-robot Mission Executor
src/robot_nav/              Nav2 parameters, maps, and navigation configs
src/robot_web/              App backend gateway
src/robot_tools/            Debugging, validation, and demo tools
src/robot_vision_bpu/       Vision detection node interface
src/robot_gazebo/           Gazebo simulation models and helper nodes
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Node.js 18 or newer
- Nav2, Gazebo, RViz2, and common ROS 2 desktop components

## Build the ROS 2 Workspace

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Start the Real-Robot PC Control Plane

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true
```

This launch starts the PC-side map service, dispatch layer, state aggregation, and RViz. It does not directly start the robot-side chassis, LiDAR, Nav2, or Mission Executor processes.

## Start the Web App Backend

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_web robot_web.launch.py
```

The backend listens on `http://0.0.0.0:8000` by default and serves `/api/*`, `/ws/status`, and frontend static files.

## Frontend Development

```bash
cd apps/robot-control-pwa
npm install
VITE_ROBOT_WEB_TARGET=http://127.0.0.1:8000 npm run dev -- --host 0.0.0.0
```

Build the frontend:

```bash
cd apps/robot-control-pwa
npm run build
```

## Gazebo/Nav2 Validation

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup dual_robot_gazebo_nav2.launch.py
```

## Validation Commands

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select robot_web robot_tools robot_bringup
colcon test-result --verbose
```

```bash
cd apps/robot-control-pwa
npm run test
npm run build
```
