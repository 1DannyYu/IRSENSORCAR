"""The junction sequence, its ordered approach sequences, and the distance gate.

These encode three real-track failures in order: the 2026-08-19 run (six junction events, six
right turns, none of the straight-through crossings the lap needs -- see `carbot.ir_route` for
the diagnosis), the 2026-08-20 dwell-timer fixes, and the 2026-08-20 rewrite to ordered signal
sequences from direct real-track tracing (see the module docstring's "third pass" section and
docs/progress/2026-08-20-map1-junction-signal-sequences.md).
"""

from __future__ import annotations

import math

import pytest

from carbot.ir_route import (
    TASK1_CORNER_WINDOWS,
    TASK1_LOOP_ONLY,
    TASK1_ROUTE,
    TURN_COMPLETE_READING,
    JunctionAction,
    JunctionSequencer,
    RouteJunction,
    RoutePlan,
    SequenceStep,
    task1_route_for_laps,
)


def test_task1_lap_is_two_turns_and_one_crossing():
    actions = [j.action for j in TASK1_ROUTE.loop]
    assert actions == [
        JunctionAction.TURN_RIGHT,
        JunctionAction.TURN_RIGHT,
        JunctionAction.CROSS,
    ]


def test_the_t_junction_is_crossed_not_turned():
    """The failure being fixed: the return T was turned into, landing back in the start box."""
    t_junction = TASK1_ROUTE.loop[-1]
    assert t_junction.name == "T junction"
    assert t_junction.action is JunctionAction.CROSS
    assert t_junction.turn_direction == 0


def test_the_first_junction_is_turned_even_though_the_looping_one_is_crossed():
    """Same physical T, opposite action — which is exactly why the reading cannot decide."""
    assert TASK1_ROUTE.prologue[0].action is JunctionAction.TURN_RIGHT
    assert TASK1_ROUTE.loop[-1].action is JunctionAction.CROSS


def test_plan_runs_the_prologue_once_then_cycles():
    names = [TASK1_ROUTE.at(i).name for i in range(8)]
    assert names == [
        "start stem T junction",
        "roundabout entry",
        "roundabout exit",
        "T junction",
        "roundabout entry",
        "roundabout exit",
        "T junction",
        "roundabout entry",
    ]


def test_plan_rejects_a_negative_index():
    with pytest.raises(ValueError, match="non-negative"):
        TASK1_ROUTE.at(-1)


def test_plan_needs_a_loop():
    with pytest.raises(ValueError, match="at least one junction"):
        RoutePlan(prologue=(), loop=())


def test_sequencer_walks_the_route_in_order():
    seq = JunctionSequencer()
    seq.travel(500.0)
    assert seq.accept().name == "start stem T junction"
    seq.travel(500.0)
    assert seq.accept().name == "roundabout entry"
    seq.travel(500.0)
    assert seq.accept().name == "roundabout exit"
    seq.travel(500.0)
    assert seq.accept().name == "T junction"
    seq.travel(500.0)
    assert seq.accept().name == "roundabout entry"


def test_gate_blocks_a_junction_read_again_too_soon():
    """The roundabout fired four sustained crossbars in 18s; the gate is what discards them."""
    seq = JunctionSequencer()
    seq.travel(100.0)
    seq.accept()  # prologue T
    seq.travel(5.0)
    assert seq.shortfall_cm() == pytest.approx(55.0)  # roundabout entry gate is 60cm


def test_gate_opens_once_the_distance_is_covered():
    seq = JunctionSequencer()
    seq.accept()
    seq.travel(60.0)
    assert seq.shortfall_cm() == 0.0


def test_accepting_restarts_the_distance_measurement():
    seq = JunctionSequencer()
    seq.travel(999.0)
    seq.accept()
    assert seq.cm_since_previous == 0.0


def test_exit_to_t_gate_fits_between_the_real_spacings():
    """23cm from the roundabout exit to the T, ~150cm from the T to the next entry.

    The gate has to clear the short hop without also clearing a re-read, which is only
    possible because the two spacings differ by more than 6x.
    """
    exit_to_t = TASK1_ROUTE.loop[2].min_cm_since_previous
    t_to_entry = TASK1_ROUTE.loop[0].min_cm_since_previous
    assert exit_to_t < 23.0
    assert t_to_entry < 150.0
    assert t_to_entry > 23.0  # a stray reading on the 23cm hop cannot pass as an entry


def test_travel_rejects_reverse():
    with pytest.raises(ValueError, match="non-negative"):
        JunctionSequencer().travel(-1.0)


def test_turn_direction_maps_to_the_wheel_speed_convention():
    assert RouteJunction("x", JunctionAction.TURN_RIGHT, 0.0, approach=()).turn_direction == 1
    assert RouteJunction("x", JunctionAction.TURN_LEFT, 0.0, approach=()).turn_direction == -1
    assert RouteJunction("x", JunctionAction.CROSS, 0.0, approach=()).turn_direction == 0


def test_loop_only_route_starts_at_the_roundabout_entry():
    """`--start-on-loop`: the car is placed on the east-west line, past the stem."""
    assert TASK1_LOOP_ONLY.prologue == ()
    assert TASK1_LOOP_ONLY.at(0).name == "roundabout entry"
    assert TASK1_LOOP_ONLY.loop == TASK1_ROUTE.loop


def test_two_lap_route_crosses_the_first_t_and_stops_at_the_second():
    """The behaviour asked for on the track: lap 1 crosses phase 10, lap 2 halts there."""
    plan = task1_route_for_laps(2)
    walked = [plan.at(i) for i in range(7)]
    assert [j.action for j in walked] == [
        JunctionAction.TURN_RIGHT,  # start-box stem T
        JunctionAction.TURN_RIGHT,  # lap 1 roundabout entry
        JunctionAction.TURN_RIGHT,  # lap 1 roundabout exit
        JunctionAction.CROSS,  # lap 1 phase 10 T junction — straight through
        JunctionAction.TURN_RIGHT,  # lap 2 roundabout entry
        JunctionAction.TURN_RIGHT,  # lap 2 roundabout exit
        JunctionAction.STOP,  # lap 2 phase 10 T junction — halt
    ]


def test_the_stop_stays_put_once_reached():
    plan = task1_route_for_laps(2)
    assert plan.at(7).action is JunctionAction.STOP
    assert plan.at(99).action is JunctionAction.STOP


def test_one_lap_route_stops_at_the_first_t_junction():
    plan = task1_route_for_laps(1)
    assert [j.action for j in (plan.at(0), plan.at(1), plan.at(2), plan.at(3))] == [
        JunctionAction.TURN_RIGHT,
        JunctionAction.TURN_RIGHT,
        JunctionAction.TURN_RIGHT,
        JunctionAction.STOP,
    ]


def test_lap_route_started_on_the_loop_skips_the_stem():
    plan = task1_route_for_laps(2, start_on_loop=True)
    assert plan.at(0).name == "roundabout entry"
    assert [plan.at(i).action for i in range(6)] == [
        JunctionAction.TURN_RIGHT,
        JunctionAction.TURN_RIGHT,
        JunctionAction.CROSS,
        JunctionAction.TURN_RIGHT,
        JunctionAction.TURN_RIGHT,
        JunctionAction.STOP,
    ]


def test_lap_count_must_be_positive():
    with pytest.raises(ValueError):
        task1_route_for_laps(0)


def test_the_stop_keeps_the_t_junction_distance_gate():
    plan = task1_route_for_laps(2)
    assert plan.at(6).min_cm_since_previous == TASK1_ROUTE.loop[-1].min_cm_since_previous


def test_start_stem_t_junction_has_a_distance_gate():
    """2026-08-20 correction: this used to be 0.0 with the reasoning "no previous junction to
    mistake it for" -- true, but that only rules out re-reading a junction, not a spurious
    trigger before the car has gone anywhere (departure-box print noise, a motor-start
    electrical transient). Photo evidence put the sensor ~6cm from the T at rest; half of
    that, matching every other gate's "half the real distance" rule."""
    assert TASK1_ROUTE.prologue[0].min_cm_since_previous == 3.0


# --------------------------------------------------- 2026-08-20 third pass: signal sequences
#
# Direct real-track tracing (operator watching the per-frame P1..P4 log) showed every real
# junction produces an ORDERED sequence of readings, several of which are not junction-shaped
# in isolation at all -- see the carbot.ir_route module docstring's "third pass" section.

START_T = TASK1_ROUTE.prologue[0]
ROUNDABOUT_ENTRY = TASK1_ROUTE.loop[0]
ROUNDABOUT_EXIT = TASK1_ROUTE.loop[1]
T_JUNCTION = TASK1_ROUTE.loop[2]


def test_start_stem_t_approach_is_crossbar_then_a_confirmed_clear():
    """2026-08-20, eighth pass: back to two steps (1111 then 0000) now that
    IRLineNav._approach_step ignores stray blips mid-accumulation instead of resetting on
    them -- sustained 1111 for 0.15-0.19s (1.5-1.9cm at 10cm/s), then a confirmed 0000 for
    more than 0.05s (0.5cm), before committing to creep+turn."""
    assert [s.bits for s in START_T.approach] == [(1, 1, 1, 1), (0, 0, 0, 0)]
    assert START_T.approach[0].min_cm == 1.7
    assert START_T.approach[1].min_cm == 0.5
    assert START_T.creep_cm == 8.5
    assert START_T.turn_deg == 90.0


def test_roundabout_entry_approach_has_the_1001_shoulder():
    assert [s.bits for s in ROUNDABOUT_ENTRY.approach] == [
        (1, 1, 1, 1),
        (1, 0, 0, 1),
        (0, 0, 0, 0),
    ]
    assert ROUNDABOUT_ENTRY.approach[0].min_cm == pytest.approx(1.65)
    assert ROUNDABOUT_ENTRY.approach[1].min_cm == pytest.approx(0.2)
    assert ROUNDABOUT_ENTRY.creep_cm == 8.0
    assert ROUNDABOUT_ENTRY.turn_deg == pytest.approx(42.5)


def test_roundabout_entry_has_an_arc_length_trigger():
    """2026-08-20, twelfth pass: Phase 8 barely exists as a straight -- ARC 3 blends directly
    into the roundabout, so the crossbar above is a fallback, not the primary trigger. 20cm is
    well past any single arc's real length (~12cm each)."""
    assert ROUNDABOUT_ENTRY.arc_trigger_cm == pytest.approx(20.0)


def test_roundabout_exit_approach_is_the_four_step_sweep():
    """0101 is Kind.NOISE and 0100/0110 are ordinary DRIFT/ON_LINE under
    carbot.ir_geometry -- only the order carries the signal, which is exactly why the old
    single-reading confirm_signatures design could never represent this junction."""
    assert [s.bits for s in ROUNDABOUT_EXIT.approach] == [
        (0, 1, 1, 1),
        (0, 1, 0, 1),
        (0, 1, 0, 0),
        (0, 1, 1, 0),
    ]
    assert ROUNDABOUT_EXIT.creep_cm == 6.5
    assert ROUNDABOUT_EXIT.turn_deg == 90.0


def test_t_junction_approach_is_asymmetric_not_the_stems_crossbar():
    """The lap-crossing T reads 0111 (asymmetric, approached off-centre from Phase 10), not
    the start stem's symmetric 1111 -- same physical junction, different approach angle."""
    assert [s.bits for s in T_JUNCTION.approach] == [(0, 1, 1, 1), (0, 1, 1, 0)]
    assert T_JUNCTION.approach[0].min_cm == 2.0


def test_t_junction_has_no_turn():
    """(g/h) Reaching the approach sequence's last step (0110) IS arrival -- no creep, no
    turn, unlike every other junction in the route."""
    assert T_JUNCTION.creep_cm == 0.0
    assert T_JUNCTION.turn_deg == 0.0


def test_turn_complete_reading_is_ordinary_centred_on_line():
    """The closed-loop turn's stop condition is literally the same reading normal FOLLOW
    treats as "centred, drive straight" -- see IRLineNav._turn_step for why checking this one
    specific reading (not "any channel visible") avoids the original false-early-exit bug."""
    assert TURN_COMPLETE_READING == (0, 1, 1, 0)


def test_final_t_junction_reuses_the_t_junctions_approach_sequence():
    plan = task1_route_for_laps(2)
    stop = plan.at(6)
    assert stop.action is JunctionAction.STOP
    assert stop.approach == T_JUNCTION.approach


def test_sequence_step_default_min_cm_is_zero():
    assert SequenceStep((1, 1, 1, 1)).min_cm == 0.0


# --------------------------------------------------------------- corner windows


def test_roundabout_traversal_window_spans_the_measured_270_degree_arc():
    """2026-08-20, eleventh pass: 33.5cm inner black-line diameter -> pi*33.5 = 105.24cm
    circumference, 270/360 of that = 78.93cm -- the window sits inside that with margin at
    both ends for the entry turn's settling and the exit approach's own crossbar detection."""
    window = next(w for w in TASK1_CORNER_WINDOWS if w.name == "roundabout traversal")
    assert window.while_pending == "roundabout exit"
    arc_cm = math.pi * 33.5 * 270 / 360
    assert window.start_cm > 0
    assert window.end_cm < arc_cm
    assert window.end_cm - window.start_cm == pytest.approx(70.0)
