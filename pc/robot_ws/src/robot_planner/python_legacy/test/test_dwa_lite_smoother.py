from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import Velocity2D
from local_planner.velocity_smoother import DwaLiteVelocitySmoother


def test_smoother_limits_velocity_acceleration_and_jerk():
    params = DwaLiteParams(
        max_vx=0.3,
        max_vy=0.2,
        max_wz=0.8,
        acc_lim_x=0.2,
        acc_lim_y=0.2,
        acc_lim_theta=0.4,
        jerk_lim_x=10.0,
        jerk_lim_y=10.0,
        jerk_lim_theta=10.0,
        control_period=0.1,
        cmd_filter_alpha=1.0,
        vx_deadband=0.0,
        vy_deadband=0.0,
        wz_deadband=0.0,
    )
    smoother = DwaLiteVelocitySmoother(params)

    cmd = smoother.smooth(Velocity2D(1.0, 1.0, 2.0))

    assert 0.0 < cmd.vx <= 0.0200001
    assert 0.0 < cmd.vy <= 0.0200001
    assert 0.0 < cmd.wz <= 0.0400001


def test_smoother_applies_low_pass_filter_and_deadband():
    params = DwaLiteParams(
        max_vx=1.0,
        acc_lim_x=10.0,
        jerk_lim_x=100.0,
        control_period=0.1,
        cmd_filter_alpha=0.5,
        vx_deadband=0.06,
        vy_deadband=0.06,
        wz_deadband=0.06,
    )
    smoother = DwaLiteVelocitySmoother(params)

    cmd = smoother.smooth(Velocity2D(0.05, 0.01, 0.01))

    assert cmd.vx == 0.0
    assert cmd.vy == 0.0
    assert cmd.wz == 0.0


def test_smoother_does_not_deadband_active_ramp_to_nonzero_target():
    params = DwaLiteParams(
        max_vx=0.32,
        acc_lim_x=0.45,
        jerk_lim_x=1.2,
        control_period=1.0 / 15.0,
        cmd_filter_alpha=0.55,
        vx_deadband=0.01,
        vy_deadband=0.01,
        wz_deadband=0.02,
    )
    smoother = DwaLiteVelocitySmoother(params)

    first = smoother.smooth(Velocity2D(0.24, 0.0, 0.0))
    second = smoother.smooth(Velocity2D(0.24, 0.0, 0.0))

    assert first.vx > 0.0
    assert second.vx > first.vx
