# robot_nav

`robot_nav` contains the robot1 Nav2 launch and tuning files extracted from the
RDK X5 `mecanum_nav` package.

This package is intentionally small. It keeps only the navigation path:

- Nav2 DWB parameters derived from `dwa_nav_params_mecanum.yaml`.
- `robot1_nav2_sim.launch.py` for Gazebo parameter tuning.
- `robot1_nav2_real.launch.py` for RDK X5 Nav2 runtime.
- `robot1_nav2_rdk_legacy.launch.py` for first-pass RDK X5 testing against
  the existing `lv_ws` Yahboom hardware topics.
- `laser_bringup_launch.py` as a real-robot convenience launch only.
- `lv_home.yaml/.pgm` as the real-robot map.
- `robot1_nav2.rviz` for visual tuning.

It does not include Cartographer, RTAB-Map, GMapping, TEB, map saving, or other
Yahboom algorithm launch files.

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

Start the chassis/lidar bringup separately, then start Nav2:

```bash
ros2 launch robot_nav laser_bringup_launch.py
ros2 launch robot_nav robot1_nav2_real.launch.py
```

The real bringup must provide or remap the hardware topics to `/robot1/scan`,
`/robot1/odom`, and `/robot1/cmd_vel`.

## RDK X5 Legacy Hardware Smoke Test

For the first real-robot test on the existing RDK X5 `~/lv_ws`, do not modify
the `lv_ws` hardware sources. Start the Yahboom hardware bringup from `lv_ws`,
then run the compatibility Nav2 launch from the copied overlay workspace:

```bash
# RDK terminal 1: hardware
cd ~/lv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mecanum_nav laser_bringup_launch.py

# RDK terminal 2: Nav2 only
cd ~/robot_ws_pc
source /opt/ros/humble/setup.bash
source ~/lv_ws/install/setup.bash
source install/setup.bash
ros2 launch robot_nav robot1_nav2_rdk_legacy.launch.py launch_rviz:=false
```

The legacy launch intentionally uses the original Yahboom interface:

- `/scan`
- `/odom`
- `/cmd_vel`
- `map -> odom -> base_footprint`

This mode is only for the first board smoke test. The project-level robot1
contract remains `/robot1/scan`, `/robot1/odom`, `/robot1/cmd_vel`, and
`map -> robot1/odom -> robot1/base_footprint`.

Run RViz on the PC side with the matching config:

```bash
rviz2 -d src/robot_nav/rviz/robot1_nav2_rdk_legacy.rviz
```

Default real map:

```text
robot_nav/maps/lv_home.yaml
```

## Topic And Frame Contract

- Nav2 nodes run under `/robot1`.
- `/map` remains the global map topic.
- Robot inputs use `/robot1/scan` and `/robot1/odom`.
- Final velocity output is `/robot1/cmd_vel`.
- Frames are `map -> robot1/odom -> robot1/base_footprint`.
