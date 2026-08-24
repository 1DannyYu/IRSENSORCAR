"""Tests for the Phase 2 per-tick decision logic in full_run.py.

`decide_step` is a pure function extracted from the script's main loop (no sensor, car, or
GPIO involved), so it is exercised directly with synthetic `IRLineReading`s. Unlike example
49, a P0111 reading past the time gate must also be *held* past `END_MARKER_HOLD_S` before it
stops the run - a brief flicker doesn't count.
"""

from pathlib import Path
from runpy import run_path

from carbot.ir_geometry import classify
from carbot.ir_line_nav import IRLineReading
from carbot.ir_modes import ROUNDABOUT_P1001_HOLD_S

SCRIPT = Path(__file__).parents[1] / "full_run.py"
NAMESPACE = run_path(str(SCRIPT))
LoopState = NAMESPACE["LoopState"]
StopAction = NAMESPACE["StopAction"]
ExitModeAction = NAMESPACE["ExitModeAction"]
DriveAction = NAMESPACE["DriveAction"]
decide_step = NAMESPACE["decide_step"]
END_MARKER_AFTER_S = NAMESPACE["END_MARKER_AFTER_S"]
END_MARKER_HOLD_S = NAMESPACE["END_MARKER_HOLD_S"]
END_MARKER_FORWARD_S = NAMESPACE["END_MARKER_FORWARD_S"]

P1001_FORWARD_S = 0.62
P1001_TURN_S = 1.67


def reading_for(physical: tuple[int, int, int, int]) -> IRLineReading:
    return IRLineReading(
        channels=physical,
        physical=physical,
        state=classify(physical, physical=True),
        visible=any(physical),
    )


def step(reading_bits, *, elapsed: float, now: float, state, speed: int = 150):
    return decide_step(
        reading_for(reading_bits),
        elapsed=elapsed,
        now=now,
        speed=speed,
        state=state,
        p1001_forward_s=P1001_FORWARD_S,
        p1001_turn_s=P1001_TURN_S,
    )


def test_centred_line_drives_forward_and_is_not_a_special_case():
    state = LoopState()

    action = step((0, 1, 1, 0), elapsed=1.0, now=1.0, state=state)

    assert isinstance(action, DriveAction)
    assert action.command.left == action.command.right == 150
    assert state.previous_command is action.command
    assert state.previous_localising == (0, 1, 1, 0)


def test_p1001_below_hold_threshold_keeps_driving():
    state = LoopState()

    action = step((1, 0, 0, 1), elapsed=1.0, now=1.0, state=state)

    assert isinstance(action, DriveAction)
    assert state.p1001_since == 1.0
    assert not state.exit_done


def test_sustained_p1001_triggers_exit_mode_once():
    state = LoopState()
    step((1, 0, 0, 1), elapsed=1.0, now=1.0, state=state)

    action = step((1, 0, 0, 1), elapsed=1.2, now=1.2, state=state)

    assert action == ExitModeAction(P1001_FORWARD_S, P1001_TURN_S)
    assert state.exit_done
    assert state.p1001_since is None
    assert state.previous_command is None

    # A later sustained P1001 must not re-trigger exit mode.
    step((1, 0, 0, 1), elapsed=5.0, now=5.0, state=state)
    later = step((1, 0, 0, 1), elapsed=5.5, now=5.5, state=state)
    assert isinstance(later, DriveAction)


def test_p1001_interrupted_before_hold_resets_the_timer():
    state = LoopState()
    step((1, 0, 0, 1), elapsed=1.0, now=1.0, state=state)
    step((0, 1, 1, 0), elapsed=1.05, now=1.05, state=state)  # briefly back on-line
    assert state.p1001_since is None

    action = step((1, 0, 0, 1), elapsed=1.06, now=1.06, state=state)

    assert isinstance(action, DriveAction)
    assert state.p1001_since == 1.06


def test_p0111_before_the_time_gate_is_not_an_end_marker():
    state = LoopState()

    action = step((0, 1, 1, 1), elapsed=END_MARKER_AFTER_S - 0.1, now=10.0, state=state)

    assert isinstance(action, DriveAction)
    assert state.end_marker_since is None


def test_p0111_after_the_time_gate_starts_the_hold_but_does_not_stop_yet():
    state = LoopState()

    action = step((0, 1, 1, 1), elapsed=END_MARKER_AFTER_S, now=100.0, state=state)

    # A single tick isn't a hold: END_MARKER_HOLD_S hasn't elapsed yet.
    assert isinstance(action, DriveAction)
    assert state.end_marker_since == 100.0


def test_p0111_held_past_the_hold_threshold_stops():
    state = LoopState()
    step((0, 1, 1, 1), elapsed=END_MARKER_AFTER_S, now=100.0, state=state)

    action = step(
        (0, 1, 1, 1),
        elapsed=END_MARKER_AFTER_S + 0.3,
        now=100.0 + END_MARKER_HOLD_S + 0.1,
        state=state,
    )

    assert action == StopAction(END_MARKER_FORWARD_S)


def test_p0111_flicker_before_the_hold_resets_and_does_not_stop():
    state = LoopState()
    step((0, 1, 1, 1), elapsed=END_MARKER_AFTER_S, now=100.0, state=state)
    step((0, 1, 1, 0), elapsed=END_MARKER_AFTER_S + 0.05, now=100.05, state=state)
    assert state.end_marker_since is None

    action = step(
        (0, 1, 1, 1),
        elapsed=END_MARKER_AFTER_S + 0.35,
        now=100.05 + END_MARKER_HOLD_S + 0.1,
        state=state,
    )

    # The new hold only just started on this tick, so it hasn't been held
    # past END_MARKER_HOLD_S yet.
    assert isinstance(action, DriveAction)
    assert state.end_marker_since == 100.05 + END_MARKER_HOLD_S + 0.1


def test_end_marker_stop_takes_priority_over_a_stale_unresolved_p1001_latch():
    """A single tick can only report one physical reading, so a P1001 latch from an earlier,
    incomplete hold must not survive into a later P0111 hold and block the stop path."""
    state = LoopState()
    step((1, 0, 0, 1), elapsed=END_MARKER_AFTER_S - 1.0, now=39.0, state=state)
    assert state.p1001_since == 39.0

    step((0, 1, 1, 1), elapsed=END_MARKER_AFTER_S, now=40.0, state=state)
    action = step(
        (0, 1, 1, 1),
        elapsed=END_MARKER_AFTER_S + 0.3,
        now=40.0 + END_MARKER_HOLD_S + 0.1,
        state=state,
    )

    assert action == StopAction(END_MARKER_FORWARD_S)
    assert state.p1001_since is None


def test_p1001_hold_still_fires_before_the_end_marker_time_gate():
    state = LoopState()
    step((1, 0, 0, 1), elapsed=5.0, now=5.0, state=state)

    action = step((1, 0, 0, 1), elapsed=5.2, now=5.0 + ROUNDABOUT_P1001_HOLD_S + 0.1, state=state)

    assert action == ExitModeAction(P1001_FORWARD_S, P1001_TURN_S)
