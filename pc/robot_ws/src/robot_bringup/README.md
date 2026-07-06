# robot_bringup

`robot_bringup` contains launch orchestration only. It should not contain planner,
safety, perception, or mission business logic.

## Dual Robot Nav2 G0/G1 Validation

Build and source the tools after changing `robot_tools`:

```bash
colcon build --symlink-install --packages-select robot_tools
source install/setup.bash
```

Start the dual-robot Gazebo/Nav2 stack from another terminal:

```bash
ros2 launch robot_bringup dual_robot_gazebo_nav2.launch.py
```

Check the topic namespace contract after startup:

```bash
ros2 run robot_tools check_robot_namespaces
```

This check fails if global `/cmd_vel`, `/odom`, or `/scan` exists, if
`/robot1/map` or `/robot2/map` exists, if shared `/map` is missing, or if the
expected namespaced `/robot1` and `/robot2` scan/odom/cmd_vel topics are
missing. For one-robot debug launches, scope the check:

```bash
ros2 run robot_tools check_robot_namespaces --robots robot1
ros2 run robot_tools check_robot_namespaces --robots robot2
```

Send one manual NavigateToPose goal to each robot:

```bash
ros2 run robot_tools send_nav_goal \
  --robot robot1 --x -7.5 --y -3.8 --yaw 0.0 --timeout-sec 180

ros2 run robot_tools send_nav_goal \
  --robot robot2 --x -7.5 --y 2.5 --yaw 0.0 --timeout-sec 180
```

The command targets `/<robot>/navigate_to_pose`, uses the shared `map` frame by
default, and prints `SUCCEEDED`, `FAILED`, `CANCELED`, or the raw action state.

Run the full G1 route file:

```bash
ros2 run robot_tools send_g1_nav_goals --goal-timeout-sec 180
```

For RViz manual testing, keep the fixed frame on `map` and use the Nav2 goal
tool for the selected robot namespace. The goal frame must remain the shared
map frame; do not create `/robot1/map` or `/robot2/map`.

## Robot1 MPC-DWB Direct Bringup

`robot1_mpc_bringup.launch.py` starts the robot1 direct local-planner stack:

- Gazebo robot1 simulation
- C++ A* global planner
- C++ MPC-DWB local planner
- optional RViz

Compatibility aliases:

- `robot1_sim_bringup.launch.py`
- `robot1_dwa_lite_bringup.launch.py`

These aliases currently start the same MPC-DWB direct stack.

This launch intentionally does not start:

- `safety_supervisor`
- legacy DWA, `DWA_lite`, `simple_tracker`, `external_dwa`, `mecanum_omni`, or `mecanum_competition`
- robot2 runtime nodes
- `fleet_manager`

The MPC-DWB planner publishes `/robot1/cmd_vel` directly. The first migration
stage follows the `rl_nav_ws` MPC tracker model, so it outputs differential
drive-style commands:

- `linear.x`
- `angular.z`
- `linear.y = 0.0`

Full mecanum `vx/vy/wz` support is planned for a later stage after the C++
migration baseline is validated.

The default visual simulation pairs `pioneer_test_20x10.world` with
`gazebo_odom_map.yaml`. Do not use `robot1_mecanum_empty.world` with
`gazebo_odom_map.yaml` when checking RViz/Gazebo visual alignment.

```bash
ros2 launch robot_bringup robot1_mpc_bringup.launch.py \
  launch_rviz:=false gui:=false
```

For visual testing:

```bash
ros2 launch robot_bringup robot1_mpc_bringup.launch.py \
  launch_rviz:=true gui:=true
```

Publish a test goal:

```bash
ros2 topic pub --once /robot1/goal_pose geometry_msgs/msg/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: -8.5
    y: -2.5
    z: 0.0
  orientation:
    w: 1.0
"
```

Check the direct local-planner outputs:

```bash
ros2 topic info /robot1/cmd_vel -v
ros2 topic echo /robot1/cmd_vel
ros2 topic echo /robot1/local_path
ros2 topic echo /robot1/planner_state
ros2 topic list | grep -E "^/(scan|odom|cmd_vel)$"
```
