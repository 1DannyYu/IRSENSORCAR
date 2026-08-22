"""Pure-software tests for Map 1's explicit ten-phase model."""

import pytest

from carbot.map1_phases import (
    ARC_TEST_DISTANCE_CREDIT_SCALE,
    ARC_TEST_OFF_TRACK_DWELL_S,
    ARC_TEST_REVERSE_REPLAY_WINDOW_S,
    MAP1_PHASES,
    PHASE1_FORWARD_PWM,
    PHASE1_FORWARD_S,
    PHASE1_RIGHT_TURN_COMPENSATION_DEG,
    PHASE1_RIGHT_TURN_DEG,
    PHASE1_RIGHT_TURN_PULSE_DEG,
    PHASE1_SPIN_DEAD_TIME_S,
    PHASE1_SPIN_RATE_DEG_PER_S,
    PROVISIONAL_FORWARD_SPEED_CM_S,
    Map1PhaseKind,
    Map1PhaseProgress,
    estimate_forward_distance_cm,
    map1_phase,
    phase_start_cm,
)


def test_phase_table_has_exactly_ten_ordered_phases() -> None:
    assert [phase.number for phase in MAP1_PHASES] == list(range(1, 11))
    assert [phase.distance_cm for phase in MAP1_PHASES[:8]] == [
        16.0,
        15.5,
        12.0,
        18.0,
        12.0,
        47.0,
        12.0,
        7.5,
    ]
    assert map1_phase(9).kind is Map1PhaseKind.ROUNDABOUT
    assert map1_phase(10).distance_cm == pytest.approx(21.5)


def test_all_three_bounded_arcs_are_declared_left_only() -> None:
    assert [map1_phase(number).steering_direction_limit for number in (3, 5, 7)] == [-1, -1, -1]
    assert all(
        phase.steering_direction_limit == 0
        for phase in MAP1_PHASES
        if phase.number not in (3, 5, 7)
    )


def test_phase1_timing_is_the_current_provisional_speed_source() -> None:
    assert PHASE1_FORWARD_S == pytest.approx(2.2)
    assert PHASE1_FORWARD_PWM == 150
    assert PROVISIONAL_FORWARD_SPEED_CM_S == pytest.approx(16.0 / 2.2)
    assert PHASE1_RIGHT_TURN_DEG == pytest.approx(90.0)
    assert PHASE1_RIGHT_TURN_COMPENSATION_DEG == pytest.approx(5.0)
    assert PHASE1_RIGHT_TURN_PULSE_DEG == pytest.approx(95.0)
    assert PHASE1_SPIN_RATE_DEG_PER_S == pytest.approx(39.7)
    assert PHASE1_SPIN_DEAD_TIME_S == pytest.approx(0.41)
    assert ARC_TEST_DISTANCE_CREDIT_SCALE == pytest.approx(0.25)
    assert ARC_TEST_OFF_TRACK_DWELL_S == pytest.approx(0.3)
    assert ARC_TEST_REVERSE_REPLAY_WINDOW_S == pytest.approx(3.0)


def test_phase_lookup_rejects_out_of_range_numbers() -> None:
    with pytest.raises(ValueError):
        map1_phase(0)
    with pytest.raises(ValueError):
        map1_phase(11)


def test_command_distance_distinguishes_forward_slow_turn_spin_and_reverse() -> None:
    common = {"dt": 1.0, "reference_pwm": 150, "reference_speed_cm_s": 10.0}
    assert estimate_forward_distance_cm(left=150, right=150, **common) == pytest.approx(10.0)
    assert estimate_forward_distance_cm(left=90, right=90, **common) == pytest.approx(6.0)
    assert estimate_forward_distance_cm(left=30, right=90, **common) == pytest.approx(4.0)
    assert estimate_forward_distance_cm(left=150, right=-150, **common) == 0.0
    assert estimate_forward_distance_cm(left=-150, right=-150, **common) == 0.0


def test_progress_can_cross_multiple_phase_boundaries_without_losing_remainder() -> None:
    progress = Map1PhaseProgress(start_phase=2)
    transitions = progress.advance_cm(15.5 + 12.0 + 2.5)

    assert [(event.completed.number, event.current.number) for event in transitions] == [
        (2, 3),
        (3, 4),
    ]
    assert progress.current == map1_phase(4)
    assert progress.phase_cm == pytest.approx(2.5)
    assert progress.total_cm == pytest.approx(phase_start_cm(4) + 2.5)


def test_progress_observes_actual_command_scale() -> None:
    progress = Map1PhaseProgress(start_phase=2)
    progress.observe_command(
        dt=1.0,
        left=90,
        right=90,
        reference_pwm=150,
        reference_speed_cm_s=10.0,
    )
    assert progress.phase_cm == pytest.approx(6.0)
