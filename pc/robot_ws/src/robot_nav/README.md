# robot_nav

`robot_nav` contains the Nav2 launch and tuning files used by the dual-robot
PC workspace.

This package is intentionally small. It keeps only the navigation path:

- `robot1_nav2_sim.launch.py` and `robot2_nav2_sim.launch.py` for Gazebo
  parameter regression.
- `mecanum_nav2_real.launch.py` and `mecanum_nav2_real.yaml` for the real
  mecanum Nav2 chain.
- `dual_robot_nav2.rviz` and `mecanum_ackermann_nav.rviz` for local display.

It does not include mapping, legacy board smoke-test launch files, or vendor
bringup packages.

## Gazebo Tuning

The sim launch starts Gazebo and Nav2. It uses the Gazebo world map by default.
When `nav2_bringup` is available, Nav2 map_server owns `/map` and AMCL owns
`map -> robot1/odom`. If `nav2_bringup` is missing from the current ROS
environment, the launch automatically falls back to Gazebo's static map and TF
publishers so RViz still loads the simulation view:

```bash
ros2 launch robot_nav robot1_nav2_sim.launch.py launch_rviz:=true gui:=true
```

Use `nav2_params_file:=...` to override the Nav2 parameter YAML. This avoids
colliding with Gazebo's own `params_file` launch argument.

Default sim map:

```text
robot_gazebo/maps/gazebo_odom_map.yaml
```

## Real Robot

Start the chassis/lidar bringup on the board side, then start the real Nav2
chain when this package is used as a local overlay:

```bash
ros2 launch robot_nav mecanum_nav2_real.launch.py
```

The real bringup must provide `/mecanum/scan`, `/mecanum/odom`, and
`/mecanum/cmd_vel`.

Default real map:

```text
robot_nav/maps/lv_home.yaml
```

## Topic And Frame Contract

- Nav2 real nodes run under `/mecanum`.
- `/map` remains the global map topic.
- Robot inputs use `/mecanum/scan` and `/mecanum/odom`.
- Final velocity output is `/mecanum/cmd_vel`.
- Frames are `map -> mecanum/odom -> mecanum/base_footprint`.
