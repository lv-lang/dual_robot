# 双机器人比赛源码仓库

本仓库包含 ROS 2 Humble 异构双机器人配送巡检系统的三端源码：

- PC 端调度与 Web App 工作空间
- mecanum 麦克纳木车控制器工作空间
- ackermann 阿克曼车控制器工作空间

English documentation: [README.md](README.md)

## 仓库结构

```text
pc/robot_ws/                 PC 端 ROS 2 工作空间与 Web App 源码
mecanum/mecanum_ws/          mecanum 机器人控制器 ROS 2 工作空间源码
ackermann/ackermann_ws/      ackermann 机器人控制器 ROS 2 工作空间源码
```

仓库只保留源码和必要配置文件，不包含构建输出、安装目录、日志、缓存、依赖目录、压缩包和本地运行数据。

## 系统概述

系统由两台机器人和一台 PC 组成：

- `mecanum`：麦克纳木配送车。
- `ackermann`：阿克曼巡检车。
- `pc`：调度控制平面、地图显示、Web App 前后端、事件日志、任务协同和 RViz 显示。

PC 端实车控制平面启动入口：

```bash
ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true
```

Web App 后端启动入口：

```bash
ros2 launch robot_web robot_web.launch.py
```

两台机器人端的 launch 文件分别保留在各自控制器工作空间中，需要在对应控制器上编译和 source 后启动。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Web App 需要 Node.js 18 或更高版本
- Nav2、RViz2、Gazebo，以及各工作空间所需的机器人端驱动依赖

## 编译 PC 工作空间

```bash
cd pc/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

前端开发运行：

```bash
cd pc/robot_ws/apps/robot-control-pwa
npm install
VITE_ROBOT_WEB_TARGET=http://127.0.0.1:8000 npm run dev -- --host 0.0.0.0
```

## 编译 mecanum 工作空间

```bash
cd mecanum/mecanum_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 编译 ackermann 工作空间

```bash
cd ackermann/ackermann_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 说明

本仓库作为源码发布使用，不包含 `build/`、`install/`、`log/`、`node_modules/`、前端生成的 `dist/`、rosbag 数据、本地压缩包和机器缓存。

