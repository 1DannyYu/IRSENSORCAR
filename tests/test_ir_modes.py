from carbot.ir_geometry import classify
from carbot.ir_modes import (
    CIRCLE_MODE_START_S,
    CircleModeState,
    DriveMode,
    auto_tracing_command,
    enter_roundabout_command,
    line_search_required,
    phase1_to_phase2_timing,
    roundabout_entry_turn_s,
    search_sweep_turn_s,
)


def test_auto_tracing_never_emits_right_motion() -> None:
    for bits in ((0, 0, 0, 1), (0, 0, 1, 0), (0, 1, 1, 0), (1, 1, 1, 0)):
        command = auto_tracing_command(classify(bits, physical=True), speed=150)
        assert command.left >= 0
        assert command.right >= 0


def test_auto_tracing_turns_left_for_left_drift() -> None:
    command = auto_tracing_command(classify((0, 1, 0, 0), physical=True), speed=150)
    assert command.left == 0
    assert command.left < command.right


def test_left_curve_junction_turns_left_instead_of_holding_straight() -> None:
    command = auto_tracing_command(classify((1, 1, 1, 0), physical=True), speed=150)
    assert command.left == 0
    assert command.right == 150


def test_p1000_is_a_hard_left_pivot() -> None:
    state = classify((1, 0, 0, 0), physical=True)
    command = auto_tracing_command(state, speed=150)
    assert state.inner_ratio == -1.0
    assert (command.left, command.right) == (-150, 150)


def test_circle_mode_splits_entry_auto_trace_and_exit() -> None:
    state = CircleModeState()
    assert state.observe(elapsed_s=CIRCLE_MODE_START_S - 0.1, bits=(1, 1, 1, 1)) is None
    assert state.observe(elapsed_s=CIRCLE_MODE_START_S, bits=(1, 1, 1, 1)) == "enter"
    assert state.phase.value == "inside-roundabout"
    for bits in ((0, 1, 1, 1), (0, 1, 0, 1), (0, 1, 0, 0)):
        assert state.observe(elapsed_s=30.0, bits=bits) is None
    assert state.observe(elapsed_s=30.0, bits=(0, 1, 1, 0)) == "exit"
    assert state.phase.value == "exited-roundabout"


def test_circle_mode_enters_when_both_entry_readings_arrive_within_one_second() -> None:
    state = CircleModeState()
    assert state.observe(elapsed_s=24.5, bits=(1, 1, 1, 0)) is None
    assert state.observe(elapsed_s=24.8, bits=(1, 1, 1, 1)) == "enter"


def test_enter_roundabout_turns_right_for_every_state() -> None:
    for bits in [(a, b, c, d) for a in (0, 1) for b in (0, 1) for c in (0, 1) for d in (0, 1)]:
        command = enter_roundabout_command(classify(bits, physical=True), speed=150)
        assert (command.left, command.right) == (150, -150)


def test_phase1_to_phase2_is_17cm_then_90deg() -> None:
    forward_s, turn_s = phase1_to_phase2_timing()
    assert forward_s == 2.1
    assert turn_s > 2.0


def test_chained_mode_is_available() -> None:
    assert DriveMode.CHAINED.value == "chained"


def test_roundabout_entry_turn_is_shorter_than_phase1_turn() -> None:
    _, phase1_turn_s = phase1_to_phase2_timing()
    assert 1.4 < roundabout_entry_turn_s() < phase1_turn_s


def test_search_starts_only_for_genuine_p0000_line_loss() -> None:
    assert line_search_required(classify((0, 0, 0, 0), physical=True), None)
    assert line_search_required(classify((0, 0, 0, 0), physical=True), (1, 0, 0, 0))
    assert not line_search_required(classify((0, 0, 0, 0), physical=True), (0, 1, 0, 0))


def test_search_sweeps_use_5_20_45_degree_progression() -> None:
    assert search_sweep_turn_s(5.0) < search_sweep_turn_s(20.0)
    assert search_sweep_turn_s(20.0) < search_sweep_turn_s(45.0)
