# external_dwa Integration Notes

This directory documents the external DWA source reviewed for the robot1 local
planner replacement. The active integration code lives in:

- `src/robot_planner/local_planner/external_dwa_adapter.py`

## Reviewed Source

- Local path: `/home/robot/local_planner/dwa_planner`
- Upstream repository named by the source README:
  `https://github.com/amslabtech/dwa_planner`
- License: BSD 3-Clause, copyright 2020 amsl

## Source Review

The reviewed source is not a pure algorithm library. It contains:

- ROS2/C++ package metadata: `package.xml`, `CMakeLists.txt`
- ROS2 node wrapper: `src/dwa_planner_node.cpp`
- Node-based planner implementation: `include/dwa_planner/dwa_planner.hpp`,
  `src/dwa_planner.cpp`, `src/parameters.cpp`
- Demo and launch files under `launch/`
- Generated API documentation under `docs/`

The upstream `DWAPlanner` class inherits `rclcpp::Node`, creates a publisher on
`/cmd_vel`, subscribes to global topics, and owns TF lookup logic. Those ROS
parts are not used in this project.

## Integrated Subset

Only the algorithmic shape was adapted:

- dynamic window sampling over `vx` and `wz`
- in-place heading adjustment before forward rollout
- trajectory rollout
- obstacle, path, target, heading, smoothness, and speed costs
- scan-to-obstacle conversion

The integrated Python module has no ROS2 publisher, no ROS2 subscription, and no
topic names. The project ROS2 adapter in `dwa_local_planner.py` remains the only
place that publishes `/robot1/cmd_vel_raw` and `/robot1/local_path`.

## Current Motion Model

The reviewed external DWA source supports differential-style velocity:

- `linear.x`
- `angular.z`

It does not support mecanum lateral velocity. The first integrated mode
therefore sets:

```yaml
enable_vy: false
```

In `external_dwa` mode, `Twist.linear.y` remains `0.0`. The previous
`simple_tracker` mode is retained as fallback.
