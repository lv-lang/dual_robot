# local_planner_legacy

This directory archives the previous robot1 local planner implementations.
The files are kept for reference only and are not the default local planning
path after the DWA_lite direct rebuild.

Archived implementations include:

- external DWA adapter and reviewed upstream notes
- mecanum competition planner core and ROS adapter
- mecanum omni planner core and ROS adapter
- simple/older DWA local planner code
- shared historical helpers for trajectories, scan sectors, side passing, and smoothing
- historical unit tests under `tests_legacy/`, renamed with a `legacy_` prefix
  so the active `robot_planner` test suite no longer collects them.

Why this code moved here:

- Multiple historical planners made `local_planner/` difficult to reason about.
- The external DWA and mecanum competition planners did not become the stable
  competition baseline.
- The current rebuild needs a single DWA_lite path with clear vx/vy/wz sampling,
  local obstacle checks, and direct `/robot1/cmd_vel` output.

Rules for this archive:

- Do not launch these nodes in the default robot1 bringup.
- Do not treat this directory as the active local planner implementation.
- Use these files only as temporary reference material when rebuilding DWA_lite.
- If historical code is reused, copy the relevant algorithmic idea into the new
  `local_planner/` DWA_lite modules instead of importing this archive.
- Do not rename `tests_legacy/legacy_*.py` back to `test_*.py` unless the
  corresponding historical planner is intentionally restored as an active path.
