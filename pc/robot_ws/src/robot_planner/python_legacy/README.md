# Python planner legacy archive

This directory contains the previous Python `robot_planner` implementation.
It is no longer the main build path after the C++ `ament_cmake` migration.

Current main path:

- `astar_global_planner`: C++ A* global planner.
- `mpc_dwb_planner`: C++ rl_nav_ws-style MPC path tracker with DWB-style scoring.

The archived code is kept for behavior comparison and rollback reference only.
It should not be started by the default robot1 bringup.
