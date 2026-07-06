import inspect
import math

from local_planner.command_smoother import CommandSmoother, CommandSmootherConfig
from local_planner.trajectory_sampler import Velocity2D


def test_smoother_clamps_vx_vy_wz_to_configured_limits():
    smoother = CommandSmoother(
        CommandSmootherConfig(
            min_vx=-0.10,
            max_vx=0.30,
            min_vy=-0.20,
            max_vy=0.20,
            max_wz=0.70,
            acc_lim_x=10.0,
            acc_lim_y=10.0,
            acc_lim_theta=10.0,
            jerk_lim_x=100.0,
            jerk_lim_y=100.0,
            jerk_lim_theta=100.0,
        )
    )

    command = smoother.smooth(Velocity2D(1.0, -1.0, 2.0), dt=0.1)

    assert command == Velocity2D(0.30, -0.20, 0.70)


def test_smoother_limits_acceleration_per_axis():
    smoother = CommandSmoother(
        CommandSmootherConfig(
            max_vx=1.0,
            max_vy=1.0,
            max_wz=2.0,
            acc_lim_x=0.40,
            acc_lim_y=0.30,
            acc_lim_theta=1.00,
            jerk_lim_x=100.0,
            jerk_lim_y=100.0,
            jerk_lim_theta=100.0,
        )
    )

    command = smoother.smooth(Velocity2D(1.0, 1.0, 2.0), dt=0.1)

    assert math.isclose(command.vx, 0.04)
    assert math.isclose(command.vy, 0.03)
    assert math.isclose(command.wz, 0.10)


def test_smoother_limits_jerk_before_reaching_acceleration_limit():
    smoother = CommandSmoother(
        CommandSmootherConfig(
            max_vx=1.0,
            max_vy=1.0,
            max_wz=2.0,
            acc_lim_x=1.0,
            acc_lim_y=1.0,
            acc_lim_theta=2.0,
            jerk_lim_x=2.0,
            jerk_lim_y=1.0,
            jerk_lim_theta=4.0,
        )
    )

    command = smoother.smooth(Velocity2D(1.0, 1.0, 2.0), dt=0.1)

    assert math.isclose(command.vx, 0.02)
    assert math.isclose(command.vy, 0.01)
    assert math.isclose(command.wz, 0.04)
    assert math.isclose(smoother.last_acceleration.vx, 0.20)
    assert math.isclose(smoother.last_acceleration.vy, 0.10)
    assert math.isclose(smoother.last_acceleration.wz, 0.40)


def test_smoother_applies_vy_and_wz_deadbands():
    smoother = CommandSmoother(
        CommandSmootherConfig(
            min_vx=-1.0,
            max_vx=1.0,
            max_vy=1.0,
            max_wz=1.0,
            acc_lim_x=10.0,
            acc_lim_y=10.0,
            acc_lim_theta=10.0,
            jerk_lim_x=100.0,
            jerk_lim_y=100.0,
            jerk_lim_theta=100.0,
            vy_deadband=0.03,
            wz_deadband=0.05,
        )
    )

    command = smoother.smooth(Velocity2D(0.2, 0.02, -0.04), dt=0.1)

    assert command == Velocity2D(0.2, 0.0, 0.0)


def test_smoother_low_pass_filters_limited_command_against_last_command():
    smoother = CommandSmoother(
        CommandSmootherConfig(
            min_vx=-1.0,
            max_vx=1.0,
            min_vy=-1.0,
            max_vy=1.0,
            max_wz=1.0,
            acc_lim_x=10.0,
            acc_lim_y=10.0,
            acc_lim_theta=10.0,
            jerk_lim_x=100.0,
            jerk_lim_y=100.0,
            jerk_lim_theta=100.0,
            low_pass_alpha=0.25,
        ),
        initial_command=Velocity2D(0.4, -0.4, 0.4),
    )

    command = smoother.smooth(Velocity2D(1.0, 0.4, -0.4), dt=0.1)

    assert math.isclose(command.vx, 0.55)
    assert math.isclose(command.vy, -0.20)
    assert math.isclose(command.wz, 0.20)


def test_smoother_reuses_velocity2d_and_has_no_ros2_side_effects():
    command = CommandSmoother().smooth(Velocity2D(0.1, 0.0, 0.0), dt=0.1)

    assert isinstance(command, Velocity2D)
    source = inspect.getsource(CommandSmoother)
    assert "rclpy" not in source
    assert "create_publisher" not in source
    assert "create_subscription" not in source
