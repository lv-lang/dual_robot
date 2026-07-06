import math

from local_planner.footprint_collision_checker import FootprintCollisionChecker, FootprintConfig
from local_planner.motion_primitive_sampler import MotionPrimitive
from local_planner.trajectory_sampler import Pose2D, Velocity2D
from local_planner.trajectory_scorer import (
    HardConstraintConfig,
    ScoringContext,
    ScoringWeights,
    TrajectoryScorer,
)


def _scorer(**overrides):
    config = HardConstraintConfig(**overrides)
    return TrajectoryScorer(
        ScoringWeights(),
        config,
        FootprintCollisionChecker(FootprintConfig(robot_length=0.24, robot_width=0.20)),
    )


def test_scorer_returns_full_cost_breakdown_for_feasible_trajectory():
    primitive = MotionPrimitive(
        name="track_center",
        command=Velocity2D(0.1, 0.0, 0.0),
        trajectory=[Pose2D(0.0, 0.0, 0.0), Pose2D(0.1, 0.0, 0.0)],
    )

    score = _scorer(max_acc_x=2.0, max_jerk_x=30.0).score(
        primitive,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_world=[],
        context=ScoringContext(current_velocity=Velocity2D(0.0, 0.0, 0.0)),
    )

    assert score.feasible
    assert score.rejection_reason == ""
    assert math.isfinite(score.progress_cost)
    assert math.isfinite(score.path_distance_cost)
    assert math.isfinite(score.lateral_target_cost)
    assert math.isfinite(score.obstacle_clearance_cost)
    assert math.isfinite(score.heading_cost)
    assert math.isfinite(score.smoothness_cost)
    assert math.isfinite(score.jerk_cost)
    assert math.isfinite(score.side_switch_cost)
    assert math.isfinite(score.unnecessary_rotation_cost)
    assert math.isfinite(score.goal_cost)
    assert math.isfinite(score.total_cost)


def test_scorer_rejects_footprint_collision_and_path_deviation():
    collision = MotionPrimitive(
        name="track_center",
        command=Velocity2D(0.0, 0.0, 0.0),
        trajectory=[Pose2D(0.0, 0.0, 0.0)],
    )
    score = _scorer().score(
        collision,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_world=[(0.05, 0.0)],
        context=ScoringContext(current_velocity=Velocity2D(0.0, 0.0, 0.0)),
    )
    assert not score.feasible
    assert score.rejection_reason == "footprint_collision"

    deviated = MotionPrimitive(
        name="hard_left",
        command=Velocity2D(0.0, 0.1, 0.0),
        trajectory=[Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.5, 0.0)],
    )
    score = _scorer(max_path_deviation=0.2, max_acc_y=2.0, max_jerk_y=30.0).score(
        deviated,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_world=[],
        context=ScoringContext(current_velocity=Velocity2D(0.0, 0.0, 0.0)),
    )
    assert not score.feasible
    assert score.rejection_reason == "path_deviation"


def test_scorer_rejects_front_stop_zone_and_side_clearance():
    forward = MotionPrimitive(
        name="track_center",
        command=Velocity2D(0.2, 0.0, 0.0),
        trajectory=[Pose2D(0.0, 0.0, 0.0), Pose2D(0.2, 0.0, 0.0)],
    )
    score = _scorer(max_acc_x=3.0, max_jerk_x=40.0).score(
        forward,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_world=[(0.45, 0.0)],
        context=ScoringContext(
            current_velocity=Velocity2D(0.0, 0.0, 0.0),
            front_blocked=True,
        ),
    )
    assert not score.feasible
    assert score.rejection_reason == "front_stop_zone"

    left = MotionPrimitive(
        name="hard_left",
        command=Velocity2D(0.0, 0.1, 0.0),
        trajectory=[Pose2D(0.0, 0.0, 0.0)],
    )
    score = _scorer(max_acc_y=3.0, max_jerk_y=40.0, min_side_clearance=0.05).score(
        left,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_world=[(0.0, 0.16)],
        context=ScoringContext(current_velocity=Velocity2D(0.0, 0.0, 0.0)),
    )
    assert not score.feasible
    assert score.rejection_reason == "side_clearance"


def test_scorer_penalizes_side_switch_and_unnecessary_rotation():
    left = MotionPrimitive(
        name="hard_left",
        command=Velocity2D(0.0, 0.1, 0.2),
        trajectory=[Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.1, 0.1)],
    )

    score = _scorer(max_acc_y=3.0, max_acc_theta=3.0, max_jerk_y=40.0, max_jerk_theta=60.0).score(
        left,
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_world=[],
        context=ScoringContext(
            current_velocity=Velocity2D(0.0, 0.0, 0.0),
            previous_side="right",
            allow_rotation=False,
        ),
    )

    assert score.feasible
    assert score.side_switch_cost == 1.0
    assert score.unnecessary_rotation_cost > 0.0
