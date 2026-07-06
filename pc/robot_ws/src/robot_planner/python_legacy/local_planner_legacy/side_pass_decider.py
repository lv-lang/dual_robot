#!/usr/bin/env python3

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from local_planner.scan_sector_model import ScanSectorSummary


class SidePassAction(str, Enum):
    TRACK_PATH = "TRACK_PATH"
    SIDESTEP_LEFT = "SIDESTEP_LEFT"
    SIDESTEP_RIGHT = "SIDESTEP_RIGHT"
    BLOCKED_STOP = "BLOCKED_STOP"


class BypassSide(str, Enum):
    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class SidePassConfig:
    front_blocked_distance: float = 0.45
    side_clearance_required: float = 0.18
    side_hysteresis: float = 0.08
    side_switch_cooldown: int = 3
    preferred_bypass_side: BypassSide = BypassSide.LEFT


@dataclass(frozen=True)
class SidePassDecision:
    action: SidePassAction
    bypass_side: Optional[BypassSide]
    reason: str
    front_clearance: float
    left_clearance: float
    right_clearance: float
    cooldown_remaining: int


class SidePassDecider:
    """Stateful side-pass selector for a robot-frame scan sector summary."""

    def __init__(self, config: SidePassConfig = SidePassConfig()) -> None:
        self.config = _normalize_config(config)
        self._last_side: Optional[BypassSide] = None
        self._cooldown_remaining = 0

    @property
    def last_side(self) -> Optional[BypassSide]:
        return self._last_side

    @property
    def cooldown_remaining(self) -> int:
        return self._cooldown_remaining

    def reset(self) -> None:
        self._last_side = None
        self._cooldown_remaining = 0

    def decide(self, scan: ScanSectorSummary) -> SidePassDecision:
        cooldown_active = self._cooldown_remaining > 0
        front_blocked = scan.front_clearance < self.config.front_blocked_distance

        if not front_blocked:
            self._tick_cooldown()
            return SidePassDecision(
                action=SidePassAction.TRACK_PATH,
                bypass_side=self._last_side,
                reason="front_clear",
                front_clearance=scan.front_clearance,
                left_clearance=scan.left_clearance,
                right_clearance=scan.right_clearance,
                cooldown_remaining=self._cooldown_remaining,
            )

        left_open = scan.left_clearance >= self.config.side_clearance_required
        right_open = scan.right_clearance >= self.config.side_clearance_required
        if not left_open and not right_open:
            self._tick_cooldown()
            return SidePassDecision(
                action=SidePassAction.BLOCKED_STOP,
                bypass_side=self._last_side,
                reason="front_and_sides_blocked",
                front_clearance=scan.front_clearance,
                left_clearance=scan.left_clearance,
                right_clearance=scan.right_clearance,
                cooldown_remaining=self._cooldown_remaining,
            )

        selected, reason = self._select_side(
            left_open=left_open,
            right_open=right_open,
            left_clearance=scan.left_clearance,
            right_clearance=scan.right_clearance,
            cooldown_active=cooldown_active,
        )
        switched = self._last_side is not None and selected != self._last_side
        self._last_side = selected
        if switched:
            self._cooldown_remaining = self.config.side_switch_cooldown
        else:
            self._tick_cooldown()

        return SidePassDecision(
            action=(
                SidePassAction.SIDESTEP_LEFT
                if selected == BypassSide.LEFT
                else SidePassAction.SIDESTEP_RIGHT
            ),
            bypass_side=selected,
            reason=reason,
            front_clearance=scan.front_clearance,
            left_clearance=scan.left_clearance,
            right_clearance=scan.right_clearance,
            cooldown_remaining=self._cooldown_remaining,
        )

    def _select_side(
        self,
        left_open: bool,
        right_open: bool,
        left_clearance: float,
        right_clearance: float,
        cooldown_active: bool,
    ) -> tuple[BypassSide, str]:
        if self._last_side == BypassSide.LEFT and left_open:
            if (
                right_open
                and right_clearance > left_clearance + self.config.side_hysteresis
                and not cooldown_active
            ):
                return BypassSide.RIGHT, "right_clearance_exceeds_hysteresis"
            return BypassSide.LEFT, "keep_left_hysteresis"

        if self._last_side == BypassSide.RIGHT and right_open:
            if (
                left_open
                and left_clearance > right_clearance + self.config.side_hysteresis
                and not cooldown_active
            ):
                return BypassSide.LEFT, "left_clearance_exceeds_hysteresis"
            return BypassSide.RIGHT, "keep_right_hysteresis"

        if left_open and not right_open:
            return BypassSide.LEFT, "left_only_open"
        if right_open and not left_open:
            return BypassSide.RIGHT, "right_only_open"

        delta = left_clearance - right_clearance
        if delta > self.config.side_hysteresis:
            return BypassSide.LEFT, "left_clearance_larger"
        if delta < -self.config.side_hysteresis:
            return BypassSide.RIGHT, "right_clearance_larger"
        return self.config.preferred_bypass_side, "preferred_bypass_side"

    def _tick_cooldown(self) -> None:
        self._cooldown_remaining = max(0, self._cooldown_remaining - 1)


def _normalize_config(config: SidePassConfig) -> SidePassConfig:
    preferred = config.preferred_bypass_side
    if isinstance(preferred, str):
        preferred = BypassSide(preferred.lower())
    return SidePassConfig(
        front_blocked_distance=config.front_blocked_distance,
        side_clearance_required=config.side_clearance_required,
        side_hysteresis=config.side_hysteresis,
        side_switch_cooldown=max(0, int(config.side_switch_cooldown)),
        preferred_bypass_side=preferred,
    )
