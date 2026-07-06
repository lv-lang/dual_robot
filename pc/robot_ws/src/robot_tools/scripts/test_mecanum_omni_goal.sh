#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
GOAL_X="${1:--8.5}"
GOAL_Y="${2:--2.5}"
GOAL_YAW="${3:-0.0}"
WAIT_SECONDS="${WAIT_SECONDS:-20}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

read -r QZ QW < <(python3 -c 'import math, sys; yaw=float(sys.argv[1]); print(math.sin(yaw / 2.0), math.cos(yaw / 2.0))' "${GOAL_YAW}")

echo "Publishing /robot1/goal_pose: x=${GOAL_X}, y=${GOAL_Y}, yaw=${GOAL_YAW}"
ros2 topic pub --once /robot1/goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: ${GOAL_X}, y: ${GOAL_Y}, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: ${QZ}, w: ${QW}}
  }
}"

echo "Waiting for /robot1/global_path"
timeout "${WAIT_SECONDS}" ros2 topic echo --once /robot1/global_path >/tmp/robot1_global_path_sample.yaml

echo "Waiting for /robot1/local_path"
timeout "${WAIT_SECONDS}" ros2 topic echo --once /robot1/local_path >/tmp/robot1_local_path_sample.yaml

echo "Waiting for /robot1/cmd_vel_raw"
timeout "${WAIT_SECONDS}" ros2 topic echo --once /robot1/cmd_vel_raw >/tmp/robot1_cmd_vel_raw_sample.yaml

echo "Waiting for /robot1/cmd_vel"
timeout "${WAIT_SECONDS}" ros2 topic echo --once /robot1/cmd_vel >/tmp/robot1_cmd_vel_sample.yaml

echo "Checking /robot1/cmd_vel publisher ownership"
ros2 topic info /robot1/cmd_vel -v | tee /tmp/robot1_cmd_vel_info.txt
if ! grep -q "Publisher count: 1" /tmp/robot1_cmd_vel_info.txt; then
  echo "ERROR: /robot1/cmd_vel must have exactly one publisher" >&2
  exit 1
fi
if ! grep -q "/robot1/safety_supervisor" /tmp/robot1_cmd_vel_info.txt; then
  echo "ERROR: /robot1/cmd_vel is not published by /robot1/safety_supervisor" >&2
  exit 1
fi

echo "Sampling /robot1/odom"
timeout "${WAIT_SECONDS}" ros2 topic echo --once /robot1/odom >/tmp/robot1_odom_sample.yaml

echo "robot1 mecanum omni goal smoke test passed"
