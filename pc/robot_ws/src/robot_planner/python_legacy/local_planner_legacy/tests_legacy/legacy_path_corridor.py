import math

import pytest

from local_planner.path_corridor import (
    PathCorridor,
    cross_track_error,
    is_rejoined,
    lookahead_point,
    nearest_path_index,
    path_tangent_heading,
)
from local_planner.trajectory_sampler import Pose2D


def test_nearest_path_index_starts_from_progress_index_to_prevent_backtracking():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]

    assert nearest_path_index(Pose2D(1.1, 0.0, 0.0), path) == 1
    assert nearest_path_index(Pose2D(1.1, 0.0, 0.0), path, start_index=2) == 2


def test_lookahead_interpolates_along_path_distance_and_clamps_at_goal():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

    index, point = lookahead_point(path, start_index=0, lookahead_distance=1.5)
    assert index == 2
    assert point == (1.0, 0.5)

    index, point = lookahead_point(path, start_index=1, lookahead_distance=5.0)
    assert index == 2
    assert point == (1.0, 1.0)


def test_cross_track_error_is_signed_positive_on_left_side_of_path_tangent():
    path = [(0.0, 0.0), (2.0, 0.0)]

    left_pose = Pose2D(0.5, 0.3, 0.0)
    right_pose = Pose2D(0.5, -0.2, 0.0)

    assert math.isclose(cross_track_error(left_pose, path, 0, signed=True), 0.3)
    assert math.isclose(cross_track_error(right_pose, path, 0, signed=True), -0.2)
    assert math.isclose(cross_track_error(right_pose, path, 0, signed=False), 0.2)


def test_path_tangent_heading_uses_forward_segment_then_previous_at_final_point():
    east_then_north = [(0.0, 0.0), (1.0, 0.0), (1.0, 2.0)]

    assert math.isclose(path_tangent_heading(east_then_north, 0), 0.0, abs_tol=1e-9)
    assert math.isclose(
        path_tangent_heading(east_then_north, 1),
        math.pi / 2.0,
        abs_tol=1e-9,
    )
    assert math.isclose(
        path_tangent_heading(east_then_north, 2),
        math.pi / 2.0,
        abs_tol=1e-9,
    )


def test_path_corridor_progress_index_is_monotonic_and_rejoin_uses_tolerance():
    corridor = PathCorridor(
        [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
        lookahead_distance=0.75,
        rejoin_tolerance=0.2,
    )

    state = corridor.update(Pose2D(2.05, 0.1, 0.0))
    assert state.nearest_index == 2
    assert state.rejoined
    assert state.lookahead_index == 3
    assert state.lookahead_point == (2.75, 0.0)

    state = corridor.update(Pose2D(0.9, 0.1, 0.0))
    assert state.nearest_index == 2
    assert corridor.progress_index == 2

    state = corridor.update(Pose2D(2.2, 0.35, 0.0))
    assert not state.rejoined
    assert math.isclose(state.abs_cross_track_error, 0.35)
    assert not is_rejoined(Pose2D(2.2, 0.35, 0.0), corridor.path, state.nearest_index, 0.2)


def test_empty_or_non_finite_paths_are_rejected():
    with pytest.raises(ValueError):
        PathCorridor([])

    with pytest.raises(ValueError):
        nearest_path_index(Pose2D(0.0, 0.0, 0.0), [(0.0, float("nan"))])
