"""Tests for the IR-tracking-sensor Task-1 route plan (``carbot.ir_route_plan``).

Distances come from ``docs/task1-single-source-of-truth.md`` (2026-08-19 revision):
Phase 11 is retired, and each lap after the first starts at Phase 2, reached by
crossing the T junction straight through instead of turning into the start-box stem.
"""

from carbot.ir_route_plan import (
    StepKind,
    task1_route_for_laps,
    task1_route_two_laps,
    total_distance_m,
)


def test_one_lap_plan_stops_at_the_t_junction():
    plan = task1_route_for_laps(1)
    assert plan.steps[-1].kind == StepKind.STOP
    assert plan.steps[-1].label == "T junction: halt, task complete"


def test_one_lap_plan_has_no_phase_11():
    plan = task1_route_for_laps(1)
    labels = [s.label for s in plan]
    assert not any("Phase 11" in label or "phase 11" in label.lower() for label in labels)


def test_two_lap_plan_crosses_between_laps_and_stops_at_the_end():
    plan = task1_route_two_laps()
    kinds = [s.kind for s in plan]
    # Phase 1 stem, T turn, then lap 1 (Phase 2..10 + CROSS), then lap 2 (Phase 2..10 + STOP).
    assert kinds == [
        StepKind.STRAIGHT,  # Phase 1 stem
        StepKind.TURN_RIGHT,  # T junction (start stem)
        StepKind.STRAIGHT,  # lap 1 Phase 2 east
        StepKind.TURN_LEFT,  # lap 1 ARC 1
        StepKind.STRAIGHT,  # lap 1 Phase 4 north
        StepKind.TURN_LEFT,  # lap 1 ARC 2
        StepKind.STRAIGHT,  # lap 1 Phase 6 west
        StepKind.TURN_LEFT,  # lap 1 ARC 3
        StepKind.STRAIGHT,  # lap 1 Phase 8 entry
        StepKind.ROUNDABOUT,  # lap 1 Phase 9 roundabout
        StepKind.TURN_RIGHT,  # lap 1 roundabout exit
        StepKind.STRAIGHT,  # lap 1 Phase 10 return
        StepKind.CROSS,  # lap 1 -> lap 2, straight through the T junction
        StepKind.STRAIGHT,  # lap 2 Phase 2 east
        StepKind.TURN_LEFT,  # lap 2 ARC 1
        StepKind.STRAIGHT,  # lap 2 Phase 4 north
        StepKind.TURN_LEFT,  # lap 2 ARC 2
        StepKind.STRAIGHT,  # lap 2 Phase 6 west
        StepKind.TURN_LEFT,  # lap 2 ARC 3
        StepKind.STRAIGHT,  # lap 2 Phase 8 entry
        StepKind.ROUNDABOUT,  # lap 2 Phase 9 roundabout
        StepKind.TURN_RIGHT,  # lap 2 roundabout exit
        StepKind.STRAIGHT,  # lap 2 Phase 10 return
        StepKind.STOP,  # lap 2 -> halt, task complete
    ]


def test_cross_and_stop_steps_have_no_distance_or_angle():
    plan = task1_route_two_laps()
    junction_steps = [s for s in plan if s.kind in (StepKind.CROSS, StepKind.STOP)]
    assert len(junction_steps) == 2
    for step in junction_steps:
        assert step.distance_m == 0.0
        assert step.angle_deg == 0.0


def test_phase_distances_match_the_single_source_of_truth():
    plan = task1_route_two_laps()
    straights = [s.distance_m for s in plan if s.kind == StepKind.STRAIGHT]
    # Phase 1 stem once, then Phase 2/4/6/8/10 repeated for each of the 2 laps.
    per_lap = [0.160, 0.192, 0.585, 0.075, 0.230]
    assert straights == [0.10, *per_lap, *per_lap]


def test_roundabout_step_matches_the_single_source_of_truth():
    plan = task1_route_two_laps()
    roundabouts = [s for s in plan if s.kind == StepKind.ROUNDABOUT]
    assert len(roundabouts) == 2
    for step in roundabouts:
        assert step.distance_m == 0.848
        assert step.angle_deg == 270.0


def test_turns_are_90_deg():
    plan = task1_route_two_laps()
    for s in plan:
        if s.kind in (StepKind.TURN_LEFT, StepKind.TURN_RIGHT):
            assert s.angle_deg == 90.0


def test_three_lap_plan_crosses_twice_and_stops_once():
    plan = task1_route_for_laps(3)
    kinds = [s.kind for s in plan]
    assert kinds.count(StepKind.CROSS) == 2
    assert kinds.count(StepKind.STOP) == 1
    assert kinds[-1] == StepKind.STOP


def test_laps_must_be_positive():
    import pytest

    with pytest.raises(ValueError, match="laps must be at least 1"):
        task1_route_for_laps(0)


def test_total_distance_is_two_full_laps_plus_the_stem():
    plan = task1_route_two_laps()
    total = total_distance_m(plan)
    # stem 0.10 + 2 * (0.160+0.192+0.585+0.075+0.230+0.848 roundabout)
    per_lap_total = 0.160 + 0.192 + 0.585 + 0.075 + 0.230 + 0.848
    assert total == round(0.10 + 2 * per_lap_total, 10)
