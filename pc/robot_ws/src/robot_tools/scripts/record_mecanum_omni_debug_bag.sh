#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)}"
OUTPUT_DIR="${1:-${WORKSPACE_DIR}/bags/robot1_mecanum_omni_$(date +%Y%m%d_%H%M%S)}"
DURATION_SECONDS="${DURATION_SECONDS:-60}"

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

mkdir -p "$(dirname "${OUTPUT_DIR}")"

echo "Recording robot1 mecanum omni debug bag to ${OUTPUT_DIR}"
echo "Duration: ${DURATION_SECONDS}s"

set +e
timeout "${DURATION_SECONDS}" ros2 bag record \
  -o "${OUTPUT_DIR}" \
  /robot1/goal_pose \
  /robot1/global_path \
  /robot1/local_path \
  /robot1/scan \
  /robot1/odom \
  /robot1/cmd_vel_raw \
  /robot1/cmd_vel \
  /robot1/safety_state \
  /robot1/planner_state \
  /robot1/vision/detections \
  /robot1/vision/target_pose \
  /robot1/vision/debug_image
status=$?
set -e

if [ "${status}" -ne 0 ] && [ "${status}" -ne 124 ]; then
  echo "ERROR: ros2 bag record failed with status ${status}" >&2
  exit "${status}"
fi

echo "Bag saved: ${OUTPUT_DIR}"
