import math

from local_planner.footprint_collision_checker import FootprintCollisionChecker, FootprintConfig
from local_planner.trajectory_sampler import Pose2D


def test_existing_collision_api_remains_compatible():
    checker = FootprintCollisionChecker(
        FootprintConfig(robot_length=0.24, robot_width=0.20, obstacle_margin=0.05)
    )

    assert checker.collides(Pose2D(0.0, 0.0, 0.0), [(0.10, 0.0)])
    assert not checker.collides(Pose2D(0.0, 0.0, 0.0), [(1.0, 0.0)])
    assert checker.trajectory_collides([Pose2D(0.0, 0.0, 0.0)], [(0.10, 0.0)])
    assert checker.trajectory_clearance([Pose2D(0.0, 0.0, 0.0)], []) == math.inf


def test_front_and_side_clearance_use_rectangular_footprint():
    checker = FootprintCollisionChecker(
        FootprintConfig(robot_length=0.24, robot_width=0.20, obstacle_margin=0.05)
    )
    pose = Pose2D(0.0, 0.0, 0.0)

    assert math.isclose(checker.front_clearance(pose, [(0.30, 0.0)]), 0.13, abs_tol=1e-6)
    assert math.isclose(checker.side_clearance(pose, [(0.0, 0.25)], "left"), 0.10, abs_tol=1e-6)
    assert math.isclose(checker.side_clearance(pose, [(0.0, -0.25)], "right"), 0.10, abs_tol=1e-6)


def test_rectangle_corners_and_body_frame_transform_rotate_with_pose():
    checker = FootprintCollisionChecker(FootprintConfig(robot_length=0.2, robot_width=0.2))
    pose = Pose2D(1.0, 2.0, math.pi / 2.0)

    body = checker.obstacles_in_body_frame(pose, [(1.0, 3.0)])
    assert math.isclose(body[0][0], 1.0, abs_tol=1e-6)
    assert math.isclose(body[0][1], 0.0, abs_tol=1e-6)

    corners = checker.rectangle_corners(Pose2D(0.0, 0.0, 0.0))
    assert len(corners) == 4
    assert corners[0][0] > 0.0
    assert corners[0][1] > 0.0
