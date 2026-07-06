import math

from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.footprint_checker import DwaLiteFootprintChecker
from local_planner.planner_utils import Pose2D, Trajectory2D, Velocity2D
from local_planner.scan_obstacle_model import ObstacleSummary
from local_planner.trajectory_scorer import DwaLiteTrajectoryScorer


def _summary(front_blocked=False, preferred_side="left"):
    return ObstacleSummary(
        front_clearance=0.4 if front_blocked else math.inf,
        left_clearance=math.inf,
        right_clearance=0.2,
        nearest_clearance=0.4,
        front_blocked=front_blocked,
        hard_stop=False,
        preferred_side=preferred_side,
    )


def test_scorer_rejects_colliding_trajectory():
    params = DwaLiteParams(safety_margin=0.05)
    scorer = DwaLiteTrajectoryScorer(params, DwaLiteFootprintChecker(params))
    trajectory = Trajectory2D(
        command=Velocity2D(0.0, 0.0, 0.0),
        poses=[Pose2D(0.0, 0.0, 0.0)],
    )

    score = scorer.score(
        trajectory,
        [(0.0, 0.0), (1.0, 0.0)],
        (1.0, 0.0),
        [(0.05, 0.0)],
        Velocity2D(0.0, 0.0, 0.0),
        _summary(),
    )

    assert not score.feasible
    assert score.rejection_reason == "footprint_collision"


def test_scorer_rejects_trajectory_below_minimum_obstacle_clearance():
    params = DwaLiteParams(obstacle_min_dist=0.24, safety_margin=0.06)
    scorer = DwaLiteTrajectoryScorer(params, DwaLiteFootprintChecker(params))
    trajectory = Trajectory2D(
        command=Velocity2D(0.1, 0.0, 0.0),
        poses=[Pose2D(0.0, 0.0, 0.0)],
    )

    score = scorer.score(
        trajectory,
        [(0.0, 0.0), (1.0, 0.0)],
        (1.0, 0.0),
        [(0.30, 0.0)],
        Velocity2D(0.0, 0.0, 0.0),
        _summary(),
    )

    assert not score.feasible
    assert score.rejection_reason == "clearance_below_min"


def test_scorer_prefers_lateral_motion_toward_clear_side_when_front_blocked():
    params = DwaLiteParams(
        path_weight=0.0,
        goal_weight=0.0,
        obstacle_weight=0.0,
        heading_weight=0.0,
        velocity_weight=0.0,
        smoothness_weight=0.0,
        lateral_weight=1.0,
    )
    scorer = DwaLiteTrajectoryScorer(params, DwaLiteFootprintChecker(params))
    poses = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0)]
    left = Trajectory2D(command=Velocity2D(0.0, 0.12, 0.0), poses=poses)
    right = Trajectory2D(command=Velocity2D(0.0, -0.12, 0.0), poses=poses)
    stop = Trajectory2D(command=Velocity2D(0.0, 0.0, 0.0), poses=poses)

    left_score = scorer.score(left, [], (1.0, 0.0), [], Velocity2D(0.0, 0.0, 0.0), _summary(True, "left"))
    right_score = scorer.score(right, [], (1.0, 0.0), [], Velocity2D(0.0, 0.0, 0.0), _summary(True, "left"))
    stop_score = scorer.score(stop, [], (1.0, 0.0), [], Velocity2D(0.0, 0.0, 0.0), _summary(True, "left"))

    assert left_score.feasible
    assert left_score.total_cost < right_score.total_cost
    assert left_score.total_cost < stop_score.total_cost
