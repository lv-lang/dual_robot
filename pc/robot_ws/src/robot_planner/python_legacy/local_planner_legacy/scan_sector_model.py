#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Sequence, Tuple


Point2D = Tuple[float, float]
SECTOR_NAMES = ("front", "front_left", "front_right", "left", "right")


@dataclass(frozen=True)
class ScanSectorConfig:
    robot_width: float = 0.20
    obstacle_margin: float = 0.05
    front_check_distance: float = 0.80
    bypass_check_distance: float = 1.00
    side_check_distance: float = 0.70
    side_forward_window: float = 0.35


@dataclass(frozen=True)
class ScanSectorSummary:
    sectors: Mapping[str, Tuple[Point2D, ...]]
    front_clearance: float
    left_clearance: float
    right_clearance: float


class ScanSectorModel:
    """Robot-frame scan sector model for robot1 mecanum side-pass planning.

    Points are plain ``(x, y)`` tuples in robot frame. Positive ``x`` is forward
    and positive ``y`` is left. This module is intentionally ROS-free.
    """

    def __init__(self, config: ScanSectorConfig = ScanSectorConfig()) -> None:
        self.config = config

    def summarize(self, points_robot_frame: Sequence[Point2D]) -> ScanSectorSummary:
        sectors: Dict[str, List[Point2D]] = {name: [] for name in SECTOR_NAMES}
        half_width = self.config.robot_width * 0.5 + self.config.obstacle_margin

        front_clearance = self.config.front_check_distance
        left_clearance = self.config.side_check_distance
        right_clearance = self.config.side_check_distance

        for point in points_robot_frame:
            if not _finite_point(point):
                continue

            x, y = point
            if 0.0 <= x <= self.config.front_check_distance and abs(y) <= half_width:
                sectors["front"].append(point)
                front_clearance = min(front_clearance, max(0.0, x))

            if (
                self.config.side_forward_window < x <= self.config.bypass_check_distance
                and y > half_width
            ):
                sectors["front_left"].append(point)

            if (
                self.config.side_forward_window < x <= self.config.bypass_check_distance
                and y < -half_width
            ):
                sectors["front_right"].append(point)

            if (
                abs(x) <= self.config.side_forward_window
                and half_width < y <= self.config.side_check_distance + half_width
            ):
                sectors["left"].append(point)
                left_clearance = min(left_clearance, max(0.0, y - half_width))

            if (
                abs(x) <= self.config.side_forward_window
                and -self.config.side_check_distance - half_width <= y < -half_width
            ):
                sectors["right"].append(point)
                right_clearance = min(right_clearance, max(0.0, -y - half_width))

        return ScanSectorSummary(
            sectors={name: tuple(values) for name, values in sectors.items()},
            front_clearance=front_clearance,
            left_clearance=left_clearance,
            right_clearance=right_clearance,
        )


def summarize_scan_sectors(
    points_robot_frame: Sequence[Point2D],
    config: ScanSectorConfig = ScanSectorConfig(),
) -> ScanSectorSummary:
    return ScanSectorModel(config).summarize(points_robot_frame)


def _finite_point(point: Point2D) -> bool:
    return len(point) == 2 and math.isfinite(point[0]) and math.isfinite(point[1])
