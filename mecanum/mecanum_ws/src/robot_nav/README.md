# robot_nav

`robot_nav` contains the Nav2 launch and tuning files used by the mecanum RDK
workspace.

This package is intentionally small. It keeps only the navigation path:

- `mecanum_nav2_real.launch.py` for the real mecanum Nav2 runtime.
- `mecanum_nav2_real.yaml` for tuned DWB parameters.
- `laser_bringup_launch.py` as the chassis/lidar convenience launch.

It does not include mapping, legacy board smoke-test launch files, or vendor
bringup packages.

## Real Robot

Start the chassis/lidar bringup separately, then start Nav2:

```bash
ros2 launch robot_nav laser_bringup_launch.py
ros2 launch robot_nav mecanum_nav2_real.launch.py
```

The real bringup must provide `/mecanum/scan`, `/mecanum/odom`, and
`/mecanum/cmd_vel`.

Default real map:

```text
robot_nav/maps/lv_home.yaml
```

## Topic And Frame Contract

- Nav2 nodes run under `/mecanum`.
- `/map` remains the global map topic.
- Robot inputs use `/mecanum/scan` and `/mecanum/odom`.
- Final velocity output is `/mecanum/cmd_vel`.
- Frames are `map -> mecanum/odom -> mecanum/base_footprint`.
