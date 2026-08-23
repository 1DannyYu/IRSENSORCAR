"""Tests for the Phase 2 per-tick decision logic in example 49.

`decide_step` is a pure function extracted from the script's main loop (no sensor, car, or
GPIO involved), so it is exercised directly with synthetic `IRLineReading`s.
"""

from pathlib import Path
from runpy import run_path

from carbot.ir_geometry import classify
from carbot.ir_line_nav import IRLineReading
from carbot.ir_modes import ROUNDABOUT_P1001_HOLD_S

SCRIPT = Path(__file__).parents[1] / "examples" / "49_ir_phase1_to_phase2_then_original_trace.py"
NAMESPACE = run_path(str(SCRIPT))
LoopState = NAMESPACE["LoopState"]
StopAction = NAMESPACE["StopAction"]
ExitModeAction = NAMESPACE["ExitModeAction"]
DriveAction = NAMESPACE["DriveAction"]
decide_step = NAMESPACE["decide_step"]
P0111_STOP_START_S = NAMESPACE["P0111_STOP_START_S"]
P0111_STOP_FORWARD_S = NAMESPACE["P0111_STOP_FORWARD_S"]

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
    assert not state.exit_action_done


def test_sustained_p1001_triggers_exit_mode_once():
    state = LoopState()
    step((1, 0, 0, 1), elapsed=1.0, now=1.0, state=state)

    action = step((1, 0, 0, 1), elapsed=1.2, now=1.2, state=state)

    assert action == ExitModeAction(P1001_FORWARD_S, P1001_TURN_S)
    assert state.exit_action_done
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


def test_p0111_before_the_time_gate_does_not_stop():
    state = LoopState()

    action = step((0, 1, 1, 1), elapsed=P0111_STOP_START_S - 0.1, now=10.0, state=state)

    assert isinstance(action, DriveAction)


def test_p0111_after_the_time_gate_stops():
    state = LoopState()

    action = step((0, 1, 1, 1), elapsed=P0111_STOP_START_S, now=100.0, state=state)

    assert action == StopAction(P0111_STOP_FORWARD_S)


def test_p0111_stop_fires_despite_a_stale_unresolved_p1001_latch():
    """A single tick can only report one physical reading, so a P1001 latch from an earlier,
    incomplete hold must not survive into a later P0111 tick and block the stop path."""
    state = LoopState()
    step((1, 0, 0, 1), elapsed=P0111_STOP_START_S - 1.0, now=39.0, state=state)
    assert state.p1001_since == 39.0

    action = step(
        (0, 1, 1, 1),
        elapsed=P0111_STOP_START_S,
        now=39.0 + ROUNDABOUT_P1001_HOLD_S + 1.0,
        state=state,
    )

    assert action == StopAction(P0111_STOP_FORWARD_S)
    assert state.p1001_since is None
