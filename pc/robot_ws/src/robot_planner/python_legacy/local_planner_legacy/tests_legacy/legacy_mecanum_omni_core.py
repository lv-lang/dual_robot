import math

from local_planner.footprint_collision_checker import (
    FootprintCollisionChecker,
    FootprintConfig,
)
from local_planner.mecanum_omni_core import MecanumOmniConfig, MecanumOmniPlanner
from local_planner.mecanum_omni_planner_node import (
    Velocity2D as AdapterVelocity2D,
    choose_velocity_feedback,
)
from local_planner.obstacle_processor import scan_to_obstacle_points
from local_planner.trajectory_sampler import Pose2D, SamplingConfig, TrajectorySampler, Velocity2D


def test_sampler_samples_vx_vy_wz_and_rolls_out_mecanum_sideways_motion():
    sampler = TrajectorySampler(
        SamplingConfig(
            min_vx=-0.1,
            max_vx=0.1,
            min_vy=-0.2,
            max_vy=0.2,
            max_wz=0.5,
            vx_samples=3,
            vy_samples=3,
            wz_samples=3,
            sim_time=1.0,
            dt=0.5,
            use_dynamic_window=False,
        )
    )

    samples = list(sampler.sample(Velocity2D(0.0, 0.0, 0.0)))
    assert any(sample.vx > 0.0 for sample in samples)
    assert any(sample.vy > 0.0 for sample in samples)
    assert any(sample.wz > 0.0 for sample in samples)

    trajectory = sampler.rollout(Pose2D(0.0, 0.0, 0.0), Velocity2D(0.0, 0.2, 0.0))
    assert trajectory[-1].x == 0.0
    assert math.isclose(trajectory[-1].y, 0.2, abs_tol=1e-6)


def test_rectangular_footprint_detects_point_inside_and_allows_clear_path():
    checker = FootprintCollisionChecker(
        FootprintConfig(robot_length=0.24, robot_width=0.20, obstacle_margin=0.05)
    )

    assert checker.collides(Pose2D(0.0, 0.0, 0.0), [(0.10, 0.0)])
    assert not checker.collides(Pose2D(0.0, 0.0, 0.0), [(1.0, 0.0)])


def test_scan_processor_filters_invalid_ranges_and_returns_robot_frame_points():
    points = scan_to_obstacle_points(
        ranges=[float("inf"), 1.0, 3.0, 0.01],
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        range_min=0.05,
        range_max=2.0,
        obstacle_range=2.5,
    )

    assert len(points) == 1
    assert math.isclose(points[0][0], 0.0, abs_tol=1e-6)
    assert math.isclose(points[0][1], 1.0, abs_tol=1e-6)


def test_planner_returns_best_cmd_and_cost_breakdown_for_open_path():
    planner = MecanumOmniPlanner(
        MecanumOmniConfig(
            sampling=SamplingConfig(
                min_vx=0.0,
                max_vx=0.3,
                min_vy=-0.2,
                max_vy=0.2,
                max_wz=0.4,
                vx_samples=4,
                vy_samples=3,
                wz_samples=3,
                sim_time=0.8,
                dt=0.2,
                use_dynamic_window=False,
            ),
            target_velocity=0.25,
        )
    )

    result = planner.plan(
        pose=Pose2D(0.0, 0.0, 0.0),
        current_velocity=Velocity2D(0.0, 0.0, 0.0),
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
    )

    assert result.valid
    assert result.best_cmd.vx > 0.0
    assert result.best_trajectory[-1].x > 0.0
    assert result.best_cost is not None
    assert math.isfinite(result.best_cost.path_cost)
    assert math.isfinite(result.best_cost.goal_cost)
    assert math.isfinite(result.best_cost.obstacle_cost)
    assert math.isfinite(result.best_cost.heading_cost)
    assert math.isfinite(result.best_cost.velocity_cost)
    assert math.isfinite(result.best_cost.smoothness_cost)
    assert math.isfinite(result.best_cost.lateral_cost)
    assert math.isfinite(result.best_cost.oscillation_cost)


def test_planner_returns_zero_when_all_trajectories_collide():
    planner = MecanumOmniPlanner(
        MecanumOmniConfig(
            sampling=SamplingConfig(
                min_vx=0.0,
                max_vx=0.2,
                min_vy=0.0,
                max_vy=0.0,
                max_wz=0.0,
                vx_samples=3,
                vy_samples=1,
                wz_samples=1,
                sim_time=0.5,
                dt=0.1,
                use_dynamic_window=False,
            ),
            footprint=FootprintConfig(robot_length=0.24, robot_width=0.20, obstacle_margin=0.05),
        )
    )

    result = planner.plan(
        pose=Pose2D(0.0, 0.0, 0.0),
        current_velocity=Velocity2D(0.0, 0.0, 0.0),
        path_points=[(0.0, 0.0), (1.0, 0.0)],
        goal=(1.0, 0.0),
        obstacles_robot_frame=[(0.05, 0.0)],
    )

    assert not result.valid
    assert result.reason == "no_feasible_trajectory"
    assert result.best_cmd == Velocity2D(0.0, 0.0, 0.0)


def test_last_command_feedback_prevents_dynamic_window_from_restarting_at_zero():
    selected_velocity = choose_velocity_feedback(
        "last_command",
        odom_velocity=AdapterVelocity2D(0.0, 0.0, 0.0),
        last_command=AdapterVelocity2D(0.0, 0.12, 0.0),
    )
    assert selected_velocity.vy == 0.12

    planner = MecanumOmniPlanner(
        MecanumOmniConfig(
            sampling=SamplingConfig(
                min_vx=0.0,
                max_vx=0.25,
                min_vy=-0.16,
                max_vy=0.16,
                max_wz=0.70,
                acc_lim_x=0.35,
                acc_lim_y=0.30,
                acc_lim_theta=0.90,
                vx_samples=5,
                vy_samples=5,
                wz_samples=7,
                sim_time=1.4,
                dt=0.1,
                control_period=0.1,
                use_dynamic_window=True,
            ),
            target_velocity=0.22,
        )
    )

    result = planner.plan(
        pose=Pose2D(-8.5, -3.8, 0.0),
        current_velocity=Velocity2D(
            selected_velocity.vx,
            selected_velocity.vy,
            selected_velocity.wz,
        ),
        previous_command=Velocity2D(
            selected_velocity.vx,
            selected_velocity.vy,
            selected_velocity.wz,
        ),
        path_points=[(-8.5, -3.8), (-8.5, -3.4), (-8.5, -2.9), (-8.5, -2.5)],
        goal=(-8.5, -3.4),
    )

    assert result.valid
    assert result.best_cmd.vy >= 0.14
