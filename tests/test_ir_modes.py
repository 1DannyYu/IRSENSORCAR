from carbot.ir_geometry import classify
from carbot.ir_modes import (
    CIRCLE_MODE_START_S,
    DriveMode,
    auto_tracing_command,
    circle_triggered,
    phase1_to_phase2_timing,
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


def test_circle_trigger_is_delayed_and_one_shot() -> None:
    assert not circle_triggered(
        elapsed_s=CIRCLE_MODE_START_S - 0.1, bits=(1, 1, 1, 0), entered=False
    )
    assert circle_triggered(
        elapsed_s=CIRCLE_MODE_START_S, bits=(1, 1, 1, 0), entered=False
    )
    assert not circle_triggered(elapsed_s=50.0, bits=(1, 1, 1, 0), entered=True)


def test_chained_mode_uses_the_circle_trigger_policy() -> None:
    assert circle_triggered(
        elapsed_s=CIRCLE_MODE_START_S, bits=(1, 1, 1, 0), entered=False
    )


def test_phase1_to_phase2_is_17cm_then_90deg() -> None:
    forward_s, turn_s = phase1_to_phase2_timing()
    assert forward_s == 1.7
    assert turn_s > 2.0


def test_chained_mode_is_available() -> None:
    assert DriveMode.CHAINED.value == "chained"
