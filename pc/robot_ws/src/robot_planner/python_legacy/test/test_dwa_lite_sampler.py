from local_planner.dwa_lite_params import DwaLiteParams
from local_planner.planner_utils import Velocity2D
from local_planner.trajectory_sampler import DwaLiteTrajectorySampler


def test_sampler_samples_vx_vy_wz_with_dynamic_window_limits():
    params = DwaLiteParams(
        min_vx=-0.2,
        max_vx=0.4,
        min_vy=-0.3,
        max_vy=0.3,
        min_wz=-1.0,
        max_wz=1.0,
        acc_lim_x=0.5,
        acc_lim_y=0.4,
        acc_lim_theta=2.0,
        control_period=0.2,
        vx_samples=3,
        vy_samples=3,
        wz_samples=3,
        use_dynamic_window=True,
    )

    samples = list(DwaLiteTrajectorySampler(params).sample(Velocity2D(0.1, 0.0, 0.2)))

    assert samples
    assert all(0.0 <= sample.vx <= 0.2 for sample in samples)
    assert all(-0.08 <= sample.vy <= 0.08 for sample in samples)
    assert all(-0.2 <= sample.wz <= 0.6 for sample in samples)
    assert any(sample.vy > 0.0 for sample in samples)
    assert any(sample.vy < 0.0 for sample in samples)


def test_sampler_can_use_full_velocity_window_for_offline_scoring():
    params = DwaLiteParams(
        min_vx=0.0,
        max_vx=0.2,
        min_vy=-0.2,
        max_vy=0.2,
        min_wz=-0.4,
        max_wz=0.4,
        vx_samples=3,
        vy_samples=3,
        wz_samples=3,
        use_dynamic_window=False,
    )

    samples = list(DwaLiteTrajectorySampler(params).sample(Velocity2D(0.0, 0.0, 0.0)))

    assert len(samples) == 27
    assert any(sample.vx == 0.2 for sample in samples)
    assert any(sample.vy == -0.2 for sample in samples)
    assert any(sample.wz == 0.4 for sample in samples)
