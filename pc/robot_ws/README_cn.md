# 双机器人智能配送巡检系统

本项目是一个基于 ROS 2 Humble 的异构双车协同配送与巡检系统，面向室内赛场演示和实车部署。系统包含麦克纳木配送车、阿克曼巡检车、PC 端调度控制平面、Web App 操作界面、地图显示、任务调度、事件日志和视觉检测接入接口。

English documentation: [README.md](README.md)

## 主要功能

- 双车分布式 ROS 2 通信与命名空间隔离
- mecanum 麦克纳木车配送任务执行
- ackermann 阿克曼车巡检任务执行
- PC 端任务调度、资源锁、状态聚合和系统控制
- Vue 3 PWA 操作界面，支持任务创建、系统状态、事件日志和实时画面展示
- 实车地图、任务点、巡检点和配送点配置
- 视觉检测结果接入接口，用于火焰、烟雾和货物堆叠异常事件上报
- Gazebo/Nav2 仿真验证入口保留用于开发回归测试

## 代码结构

```text
apps/robot-control-pwa/     Web App 前端
src/robot_bringup/          PC 与仿真 launch 编排
src/robot_dispatch/         任务调度与系统状态管理
src/robot_interfaces/       自定义 msg/srv/action 接口
src/robot_mission/          单车 Mission Executor
src/robot_nav/              Nav2 参数、地图和导航配置
src/robot_web/              App 后端网关
src/robot_tools/            调试、验证和演示工具
src/robot_vision_bpu/       视觉检测节点接口
src/robot_gazebo/           Gazebo 仿真模型与辅助节点
src/yahboomcar_*            底盘、导航和 RViz 相关适配包
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Node.js 18 或更高版本
- Nav2、Gazebo、RViz2 等 ROS 2 常用组件

## 编译 ROS 2 工作空间

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 启动 PC 实车控制平面

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup real_robot_control_plane.launch.py launch_rviz:=true
```

该入口负责 PC 侧地图服务、调度、状态聚合和 RViz，不直接启动两台车上的底盘、雷达、Nav2 或 Mission Executor。

## 启动 Web App 后端

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_web robot_web.launch.py
```

默认后端监听 `http://0.0.0.0:8000`，并提供 `/api/*`、`/ws/status` 和前端静态资源服务。

## 前端开发运行

```bash
cd apps/robot-control-pwa
npm install
VITE_ROBOT_WEB_TARGET=http://127.0.0.1:8000 npm run dev -- --host 0.0.0.0
```

前端构建：

```bash
cd apps/robot-control-pwa
npm run build
```

## Gazebo/Nav2 验证入口

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch robot_bringup dual_robot_gazebo_nav2.launch.py
```

## 常用验证

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

