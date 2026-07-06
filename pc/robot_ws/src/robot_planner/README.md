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

The package is a C++ `ament_cmake` package: A* global planning plus an MPC-style
path tracker. The first local planner migration stage keeps the diff-drive
model (`linear.x` and `angular.z`, `linear.y=0.0`) so the architecture can be
validated before reintroducing full mecanum `vx/vy/wz` control.
