"""Tests for the Task-1 route plan (``carbot.route_plan``)."""

from carbot.route_plan import MAP_SCALE, StepKind, task1_route, total_distance_m


def test_task1_route_has_14_steps():
    plan = task1_route()
    assert len(plan) == 14
    assert plan.name == "task1"


def test_task1_route_kind_sequence():
    plan = task1_route()
    kinds = [s.kind for s in plan]
    assert kinds == [
        StepKind.STRAIGHT,
        StepKind.TURN_RIGHT,
        StepKind.STRAIGHT,
        StepKind.TURN_LEFT,
        StepKind.STRAIGHT,
        StepKind.TURN_LEFT,
        StepKind.STRAIGHT,
        StepKind.TURN_LEFT,
        StepKind.STRAIGHT,
        StepKind.ROUNDABOUT,
        StepKind.TURN_RIGHT,
        StepKind.STRAIGHT,
        StepKind.TURN_RIGHT,
        StepKind.STRAIGHT,
    ]


def test_task1_route_distances():
    plan = task1_route()
    straights = [s.distance_m for s in plan if s.kind == StepKind.STRAIGHT]
    assert straights == [
        round(d * MAP_SCALE, 4) for d in [0.10, 0.16, 0.19, 0.585, 0.075, 0.23, 0.10]
    ]
    lap = next(s for s in plan if s.kind == StepKind.ROUNDABOUT)
    assert lap.distance_m == round(0.848 * MAP_SCALE, 4)
    assert lap.angle_deg == 270.0


def test_task1_route_total():
    plan = task1_route()
    total = total_distance_m(plan)
    assert 1.8 < total < 2.1  # straight + roundabout arc, scaled by MAP_SCALE


def test_task1_route_turns_are_90_deg():
    plan = task1_route()
    for s in plan:
        if s.kind in (StepKind.TURN_LEFT, StepKind.TURN_RIGHT):
            assert s.angle_deg == 90.0
