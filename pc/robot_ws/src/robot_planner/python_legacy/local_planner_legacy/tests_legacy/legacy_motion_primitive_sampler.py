import math

from local_planner.motion_primitive_sampler import (
    MotionPrimitiveConfig,
    MotionPrimitiveSampler,
    PRIMITIVE_NAMES,
    PrimitiveContext,
)
from local_planner.trajectory_sampler import Pose2D, Velocity2D


def test_sampler_generates_required_mecanum_primitives():
    sampler = MotionPrimitiveSampler(
        MotionPrimitiveConfig(use_dynamic_window=False, sim_time=0.5, dt=0.25)
    )

    primitives = sampler.sample(
        Pose2D(0.0, 0.0, 0.0),
        Velocity2D(0.0, 0.0, 0.0),
        PrimitiveContext(heading_error=0.1, rejoin_direction=-0.2),
    )
    names = {primitive.name for primitive in primitives}

    assert set(PRIMITIVE_NAMES) == names
    assert any(p.command.vy > 0.0 for p in primitives if p.name == "hard_left")
    assert any(p.command.vy < 0.0 for p in primitives if p.name == "hard_right")
    assert any(p.command.vx == 0.0 and p.command.vy == 0.0 for p in primitives if p.name == "slow_stop")


def test_rollout_uses_mecanum_body_frame_velocity():
    sampler = MotionPrimitiveSampler(
        MotionPrimitiveConfig(use_dynamic_window=False, sim_time=1.0, dt=0.5)
    )

    sideways = sampler.rollout(Pose2D(0.0, 0.0, 0.0), Velocity2D(0.0, 0.2, 0.0))
    assert sideways[-1].x == 0.0
    assert math.isclose(sideways[-1].y, 0.2, abs_tol=1e-6)

    rotated = sampler.rollout(Pose2D(0.0, 0.0, math.pi / 2.0), Velocity2D(0.2, 0.0, 0.0))
    assert math.isclose(rotated[-1].x, 0.0, abs_tol=1e-6)
    assert math.isclose(rotated[-1].y, 0.2, abs_tol=1e-6)


def test_dynamic_window_limits_command_delta():
    sampler = MotionPrimitiveSampler(
        MotionPrimitiveConfig(
            acc_lim_x=0.4,
            acc_lim_y=0.4,
            acc_lim_theta=1.0,
            control_period=0.1,
            use_dynamic_window=True,
        )
    )

    primitives = sampler.sample(Pose2D(0.0, 0.0, 0.0), Velocity2D(0.0, 0.0, 0.0))

    assert max(abs(p.command.vx) for p in primitives) <= 0.04
    assert max(abs(p.command.vy) for p in primitives) <= 0.04
    assert max(abs(p.command.wz) for p in primitives) <= 0.10
