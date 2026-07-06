from local_planner.scan_obstacle_model import downsample_obstacle_points


def test_downsample_obstacle_points_caps_count_and_keeps_nearest_points():
    points = [(float(index), 0.0) for index in range(100)]

    sampled = downsample_obstacle_points(points, 12)

    assert len(sampled) <= 12
    assert (0.0, 0.0) in sampled
    assert (1.0, 0.0) in sampled


def test_downsample_obstacle_points_handles_disabled_cap():
    assert downsample_obstacle_points([(1.0, 0.0), (2.0, 0.0)], 0) == []
