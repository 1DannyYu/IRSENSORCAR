"""Regression tests for example 39's sensor-blind Phase 1 manoeuvre."""

from pathlib import Path
from runpy import run_path

import pytest

SCRIPT = Path(__file__).parents[2] / "examples" / "other" / "39_map1_ir_line_follow.py"
RUN_HARDCODED_PHASE1 = run_path(str(SCRIPT))["run_hardcoded_phase1"]
PHASE2_ACQUISITION_COMMAND = run_path(str(SCRIPT))["phase2_acquisition_command"]
PHASE3_LEAD_IN_TRANSITION = run_path(str(SCRIPT))["phase3_lead_in_transition"]
PHASE3_COMPLETION_GATE = run_path(str(SCRIPT))["Phase3CompletionGate"]


class FakeCar:
    def __init__(self) -> None:
        self.moves: list[tuple[float, int, int]] = []

    def move_for(self, seconds: float, left: int, right: int, *, on_command=None) -> int:
        self.moves.append((seconds, left, right))
        if on_command is not None:
            on_command(1, 0.0, left, right)
        return 1


class FailIfUsedNav:
    def step(self, _reading, _dt):
        raise AssertionError("normal pendulum search should not replace a known line direction")


def test_phase1_is_exactly_forward_then_stationary_right_spin() -> None:
    car = FakeCar()
    sleeps: list[float] = []
    logs: list[str] = []

    forward_s, turn_s = RUN_HARDCODED_PHASE1(
        car,
        speed=150,
        spin_rate_deg_per_s=42.0,
        spin_dead_time_s=0.41,
        sleep=sleeps.append,
        log=logs.append,
    )

    assert forward_s == pytest.approx(2.2)
    assert turn_s == pytest.approx(0.41 + 95.0 / 42.0)
    assert car.moves == [
        (pytest.approx(forward_s), 150, 150),
        (pytest.approx(turn_s), 150, -150),
    ]
    assert sleeps == [0.2, 0.2]
    assert any("SENSORS DISABLED" in line for line in logs)
    assert any("FORWARD COMMAND #01" in line for line in logs)
    assert any("FORWARD COMPLETE" in line for line in logs)
    assert any("RIGHT SPIN COMPLETE" in line for line in logs)


def test_phase1_dry_run_reports_plan_without_sleeping() -> None:
    sleeps: list[float] = []
    logs: list[str] = []

    RUN_HARDCODED_PHASE1(
        None,
        speed=150,
        spin_rate_deg_per_s=42.0,
        spin_dead_time_s=0.41,
        sleep=sleeps.append,
        log=logs.append,
    )

    assert sleeps == []
    assert logs[-1] == "DRY RUN; no motor commands issued"


def test_phase1_forward_calibration_can_be_overridden_without_changing_spin() -> None:
    car = FakeCar()

    forward_s, turn_s = RUN_HARDCODED_PHASE1(
        car,
        speed=150,
        forward_speed=220,
        forward_s=1.4,
        spin_rate_deg_per_s=39.7,
        spin_dead_time_s=0.41,
        sleep=lambda _seconds: None,
        log=lambda _message: None,
    )

    assert forward_s == pytest.approx(1.4)
    assert car.moves == [
        (pytest.approx(1.4), 220, 220),
        (pytest.approx(turn_s), 150, -150),
    ]


def test_phase1_logs_sensor_bits_as_observation_only() -> None:
    car = FakeCar()
    logs: list[str] = []

    RUN_HARDCODED_PHASE1(
        car,
        speed=150,
        spin_rate_deg_per_s=39.7,
        spin_dead_time_s=0.41,
        sleep=lambda _seconds: None,
        log=logs.append,
        observe_bits=lambda: (0, 0, 0, 0),
    )

    assert any("passive IR=P0000 (OBSERVE ONLY)" in line for line in logs)
    assert car.moves[0][1:] == (150, 150)


@pytest.mark.parametrize(
    ("spin_rate", "dead_time"),
    [(0.0, 0.41), (42.0, -0.01)],
)
def test_phase1_rejects_invalid_timing_inputs(spin_rate: float, dead_time: float) -> None:
    with pytest.raises(ValueError):
        RUN_HARDCODED_PHASE1(
            FakeCar(),
            speed=150,
            spin_rate_deg_per_s=spin_rate,
            spin_dead_time_s=dead_time,
        )


def test_phase2_acquisition_pivots_toward_visible_right_line_through_p0000() -> None:
    from carbot.ir_line_nav import make_reading

    nav = FailIfUsedNav()
    right = make_reading((0, 0, 0, 1))  # physical P0001 after channel reordering
    left, right_pwm, direction, reason = PHASE2_ACQUISITION_COMMAND(right, nav, 0.01, 150, 0)
    assert (left, right_pwm, direction) == (150, -150, 1)
    assert "pivoting right" in reason

    blank = make_reading((0, 0, 0, 0))
    left, right_pwm, direction, reason = PHASE2_ACQUISITION_COMMAND(
        blank, nav, 0.01, 150, direction
    )
    assert (left, right_pwm, direction) == (150, -150, 1)
    assert "continuing right" in reason


def test_phase2_acquisition_stops_when_line_is_centred() -> None:
    from carbot.ir_line_nav import make_reading

    centred = make_reading((1, 0, 1, 0))  # physical P0110 after channel reordering
    assert PHASE2_ACQUISITION_COMMAND(centred, FailIfUsedNav(), 0.01, 150, 1)[:3] == (0, 0, 1)


def test_phase3_lead_in_confirms_measured_leftward_arc_entry_sequence() -> None:
    stage, detected, _ = PHASE3_LEAD_IN_TRANSITION(0, (0, 1, 0, 0))
    assert (stage, detected) == (1, False)
    stage, detected, _ = PHASE3_LEAD_IN_TRANSITION(stage, (0, 0, 0, 0))
    assert (stage, detected) == (2, False)
    stage, detected, reason = PHASE3_LEAD_IN_TRANSITION(stage, (1, 0, 0, 0))
    assert (stage, detected) == (0, True)
    assert "confirmed" in reason


def test_phase3_lead_in_rejects_centred_and_right_side_paper_noise() -> None:
    stage = 0
    for physical in ((0, 1, 1, 0), (0, 0, 1, 0), (0, 0, 0, 1), (0, 1, 1, 1)):
        stage, detected, _ = PHASE3_LEAD_IN_TRANSITION(stage, physical)
        assert (stage, detected) == (0, False)


def test_phase3_lead_in_accepts_direct_left_pair_curve_shape() -> None:
    assert PHASE3_LEAD_IN_TRANSITION(0, (1, 1, 0, 0))[1] is True
    assert PHASE3_LEAD_IN_TRANSITION(0, (1, 1, 1, 0))[1] is True


def test_phase3_completion_requires_turn_before_centred_exit_confirmation() -> None:
    gate = PHASE3_COMPLETION_GATE(exit_confirm_s=0.8, phase4_proof_s=2.0)
    assert gate.update((0, 1, 1, 0), "on_line", "follow", 1.0) is None
    assert gate.mode == "arc"

    assert gate.update((1, 0, 0, 0), "drift", "follow", 0.1) is None
    assert gate.arc_turn_observed is True
    assert gate.update((0, 1, 1, 0), "on_line", "follow", 0.4) is None
    assert gate.update((0, 1, 1, 0), "on_line", "follow", 0.4) == "phase4"


def test_phase3_default_exit_gate_accepts_the_observed_half_second_p0110_window() -> None:
    gate = PHASE3_COMPLETION_GATE()
    gate.update((1, 0, 0, 0), "drift", "follow", 0.1)
    for _ in range(4):
        assert gate.update((0, 1, 1, 0), "on_line", "follow", 0.1) is None
    assert gate.update((0, 1, 1, 0), "on_line", "follow", 0.1) == "phase4"


def test_phase3_completion_requires_stable_phase4_following() -> None:
    gate = PHASE3_COMPLETION_GATE(exit_confirm_s=0.1, phase4_proof_s=2.0)
    gate.update((0, 1, 0, 0), "drift", "follow", 0.1)
    assert gate.update((0, 1, 1, 0), "on_line", "follow", 0.1) == "phase4"

    assert gate.update((0, 1, 1, 0), "on_line", "follow", 1.0) is None
    assert gate.update((0, 0, 0, 0), "ambiguous", "search", 0.1) is None
    assert gate.phase4_valid_elapsed == 0.0
    assert gate.update((0, 1, 1, 0), "on_line", "follow", 1.0) is None
    assert gate.update((0, 1, 0, 0), "drift", "follow", 1.0) == "complete"
