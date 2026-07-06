# robot_gazebo

Gazebo Classic hardware-replacement assets for the current robot1 X3
mecanum simulation. This package does not launch planning, safety, mission, BPU
vision, or robot2 runtime nodes.

## Run

Build and source the workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

Launch robot1 in Gazebo:

```bash
ros2 launch robot_gazebo robot1_mecanum_world.launch.py gui:=false
```

The default world is `pioneer_test_20x10.world`, adapted from
`/home/robot/rl_nav_ws/src/simulation/pioneer_gazebo_nav/worlds`, with the
paired `gazebo_odom_map.yaml` and `gazebo_odom_map.pgm` published as
`/robot1/map`. The default spawn pose is `x:=-8.5 y:=-3.8 z:=0.0 yaw:=0.0`,
which is a free pose in that map/world pair.

The launch file spawns the existing X3 robot1 description with Gazebo plugins
and a Gazebo hardware adapter for:

- `/robot1/scan`
- `/robot1/odom`
- `/robot1/map`
- `/robot1/camera/color/image_raw`
- `/robot1/camera/depth/image_raw`
- `/robot1/camera/color/camera_info`
- `/robot1/cmd_vel`

The robot model uses Gazebo's `libgazebo_ros_planar_move.so` plugin. It
subscribes to `/robot1/cmd_vel`, moves the Gazebo `robot1` entity directly, and
publishes `/robot1/odom` plus the `robot1/odom -> robot1/base_footprint` TF.
The Gazebo laser plugin publishes `/robot1/scan_raw`; `scan_self_filter`
removes near-field body returns and republishes the planner/RViz scan as
`/robot1/scan`.

Manual drive test:

```bash
ros2 topic pub --rate 10 --times 20 /robot1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15, y: 0.05}, angular: {z: 0.2}}"
ros2 topic pub --once /robot1/cmd_vel geometry_msgs/msg/Twist "{}"
```

## Test

Check that the expected namespaced topics exist:

```bash
ros2 topic list | grep '^/robot1/'
```

Sample one message from the required hardware-replacement topics:

```bash
ros2 topic echo --once /robot1/scan
ros2 topic echo --once /robot1/odom
ros2 topic echo --once /robot1/map
ros2 topic echo --once /robot1/camera/color/image_raw
```

Check that forbidden global hardware topics were not created:

```bash
ros2 topic list | grep -E '^/(scan|odom|cmd_vel)$' || true
```

Override the default world or map when needed:

```bash
ros2 launch robot_gazebo robot1_mecanum_world.launch.py \
  world:=/path/to/world.world \
  map_yaml:=/path/to/map.yaml
```
