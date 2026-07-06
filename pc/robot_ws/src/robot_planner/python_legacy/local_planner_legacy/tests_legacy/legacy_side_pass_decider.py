from local_planner.scan_sector_model import ScanSectorSummary
from local_planner.side_pass_decider import (
    BypassSide,
    SidePassAction,
    SidePassConfig,
    SidePassDecider,
)


def _summary(front: float, left: float, right: float) -> ScanSectorSummary:
    return ScanSectorSummary(
        sectors={"front": (), "front_left": (), "front_right": (), "left": (), "right": ()},
        front_clearance=front,
        left_clearance=left,
        right_clearance=right,
    )


def test_side_pass_tracks_path_when_front_is_clear():
    decider = SidePassDecider(SidePassConfig(front_blocked_distance=0.45))

    decision = decider.decide(_summary(front=0.70, left=0.20, right=0.50))

    assert decision.action == SidePassAction.TRACK_PATH
    assert decision.bypass_side is None
    assert decision.reason == "front_clear"


def test_side_pass_uses_preferred_side_when_both_sides_are_similar():
    decider = SidePassDecider(
        SidePassConfig(
            preferred_bypass_side=BypassSide.RIGHT,
            side_hysteresis=0.10,
            side_clearance_required=0.18,
        )
    )

    decision = decider.decide(_summary(front=0.20, left=0.46, right=0.42))

    assert decision.action == SidePassAction.SIDESTEP_RIGHT
    assert decision.bypass_side == BypassSide.RIGHT
    assert decision.reason == "preferred_bypass_side"


def test_side_pass_selects_side_with_more_clearance_when_difference_is_large():
    decider = SidePassDecider(SidePassConfig(side_hysteresis=0.05))

    decision = decider.decide(_summary(front=0.20, left=0.22, right=0.50))

    assert decision.action == SidePassAction.SIDESTEP_RIGHT
    assert decision.bypass_side == BypassSide.RIGHT
    assert decision.reason == "right_clearance_larger"


def test_side_pass_hysteresis_keeps_previous_side_for_small_clearance_change():
    decider = SidePassDecider(
        SidePassConfig(
            side_hysteresis=0.10,
            side_clearance_required=0.18,
            preferred_bypass_side=BypassSide.LEFT,
        )
    )

    first = decider.decide(_summary(front=0.20, left=0.50, right=0.30))
    second = decider.decide(_summary(front=0.20, left=0.44, right=0.50))

    assert first.bypass_side == BypassSide.LEFT
    assert second.action == SidePassAction.SIDESTEP_LEFT
    assert second.reason == "keep_left_hysteresis"


def test_side_switch_cooldown_blocks_switch_until_it_expires():
    decider = SidePassDecider(
        SidePassConfig(
            side_hysteresis=0.05,
            side_switch_cooldown=2,
            preferred_bypass_side=BypassSide.LEFT,
        )
    )

    first = decider.decide(_summary(front=0.20, left=0.55, right=0.20))
    switched = decider.decide(_summary(front=0.20, left=0.19, right=0.60))
    held = decider.decide(_summary(front=0.20, left=0.30, right=0.70))
    after_cooldown = decider.decide(_summary(front=0.20, left=0.30, right=0.70))

    assert first.bypass_side == BypassSide.LEFT
    assert switched.bypass_side == BypassSide.RIGHT
    assert switched.cooldown_remaining == 2
    assert held.bypass_side == BypassSide.RIGHT
    assert held.reason == "keep_right_hysteresis"
    assert after_cooldown.bypass_side == BypassSide.RIGHT
    assert after_cooldown.cooldown_remaining == 0


def test_side_pass_blocks_when_front_and_both_sides_are_closed():
    decider = SidePassDecider(SidePassConfig(side_clearance_required=0.20))

    decision = decider.decide(_summary(front=0.10, left=0.12, right=0.19))

    assert decision.action == SidePassAction.BLOCKED_STOP
    assert decision.bypass_side is None
    assert decision.reason == "front_and_sides_blocked"
