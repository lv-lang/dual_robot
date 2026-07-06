import math

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.footprint_checker import DwaLiteFootprintChecker
from local_planner.planner_utils import Pose2D


def test_footprint_checker_detects_inflated_rectangle_collision():
    checker = DwaLiteFootprintChecker(
        DwaLiteParams(robot_length=0.24, robot_width=0.20, safety_margin=0.05)
    )

    assert checker.collides(Pose2D(0.0, 0.0, 0.0), [(0.10, 0.0)])
    assert not checker.collides(Pose2D(0.0, 0.0, 0.0), [(1.0, 0.0)])


def test_footprint_checker_tracks_trajectory_clearance():
    checker = DwaLiteFootprintChecker(
        DwaLiteParams(robot_length=0.24, robot_width=0.20, safety_margin=0.05)
    )
    trajectory = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.5, 0.0, 0.0)]

    assert checker.trajectory_collides(trajectory, [(0.5, 0.0)])
    assert math.isinf(checker.trajectory_clearance(trajectory, []))
    assert checker.trajectory_clearance(trajectory, [(1.0, 0.0)]) > 0.0
