import math

from local_planner.planner_utils import Pose2D, Velocity2D
from local_planner.trajectory_rollout import rollout_mecanum


def test_rollout_uses_mecanum_forward_and_lateral_kinematics():
    forward = rollout_mecanum(Pose2D(0.0, 0.0, 0.0), Velocity2D(0.2, 0.0, 0.0), 1.0, 0.1)
    lateral = rollout_mecanum(Pose2D(0.0, 0.0, 0.0), Velocity2D(0.0, 0.2, 0.0), 1.0, 0.1)

    assert math.isclose(forward[-1].x, 0.2, abs_tol=1e-6)
    assert math.isclose(forward[-1].y, 0.0, abs_tol=1e-6)
    assert math.isclose(lateral[-1].x, 0.0, abs_tol=1e-6)
    assert math.isclose(lateral[-1].y, 0.2, abs_tol=1e-6)


def test_rollout_respects_yaw_and_wraps_angle():
    sideways_world = rollout_mecanum(
        Pose2D(0.0, 0.0, math.pi / 2.0),
        Velocity2D(0.2, 0.0, 0.0),
        1.0,
        0.1,
    )
    rotating = rollout_mecanum(
        Pose2D(0.0, 0.0, math.pi / 2.0),
        Velocity2D(0.0, 0.0, math.pi),
        1.0,
        0.1,
    )

    assert math.isclose(sideways_world[-1].x, 0.0, abs_tol=1e-6)
    assert sideways_world[-1].y > 0.19
    assert -math.pi <= rotating[-1].yaw <= math.pi
