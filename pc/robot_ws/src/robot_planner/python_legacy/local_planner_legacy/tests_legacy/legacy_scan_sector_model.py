import math

from local_planner.scan_sector_model import ScanSectorConfig, summarize_scan_sectors


def test_scan_sector_model_partitions_robot_frame_points():
    summary = summarize_scan_sectors(
        [
            (0.30, 0.00),
            (0.40, 0.30),
            (0.50, -0.32),
            (0.00, 0.42),
            (0.00, -0.44),
            (float("inf"), 0.0),
        ],
        ScanSectorConfig(robot_width=0.20, obstacle_margin=0.05),
    )

    assert summary.sectors["front"] == ((0.30, 0.00),)
    assert summary.sectors["front_left"] == ((0.40, 0.30),)
    assert summary.sectors["front_right"] == ((0.50, -0.32),)
    assert summary.sectors["left"] == ((0.00, 0.42),)
    assert summary.sectors["right"] == ((0.00, -0.44),)


def test_scan_sector_model_reports_front_left_right_clearance():
    summary = summarize_scan_sectors(
        [(0.55, 0.01), (0.0, 0.38), (0.0, -0.52)],
        ScanSectorConfig(
            robot_width=0.20,
            obstacle_margin=0.05,
            front_check_distance=0.80,
            side_check_distance=0.70,
        ),
    )

    assert math.isclose(summary.front_clearance, 0.55)
    assert math.isclose(summary.left_clearance, 0.23)
    assert math.isclose(summary.right_clearance, 0.37)


def test_scan_sector_model_uses_configured_max_clearance_when_sector_empty():
    summary = summarize_scan_sectors(
        [],
        ScanSectorConfig(front_check_distance=0.90, side_check_distance=0.65),
    )

    assert summary.front_clearance == 0.90
    assert summary.left_clearance == 0.65
    assert summary.right_clearance == 0.65
