# robot_planner

`robot_planner` owns the robot1 planning stack.

Current main chain:

```text
/robot1/goal_pose
  -> astar_global_planner
  -> /robot1/global_path
  -> mpc_dwb_planner
  -> /robot1/cmd_vel
```

The package is now a C++ `ament_cmake` package following the non-RL navigation
structure from `/home/robot/rl_nav_ws`: A* global planning plus an MPC-style path
tracker. The first local planner migration stage keeps the rl_nav_ws diff-drive
model (`linear.x` and `angular.z`, `linear.y=0.0`) so the architecture can be
validated before reintroducing full mecanum `vx/vy/wz` control.

Archived Python implementations live under `python_legacy/` and are not part of
the default build or bringup path.
