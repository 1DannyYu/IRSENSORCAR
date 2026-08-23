from __future__ import annotations

import itertools

import pytest

from carbot.ir_geometry import (
    BLIND_BAND_CM,
    DETECTION_LIMIT_CM,
    LINE_WIDTH_CM,
    PHYSICAL_ORDER,
    SENSOR_POSITIONS_CM,
    STATE_TABLE,
    Kind,
    classify,
    resolve_blind,
    to_physical,
    wheel_speeds,
)


def test_physical_order_is_out2_out1_out3_out4():
    # Measured 2026-08-19: sweeping a card left to right tripped the channels
    # in the order Out2, Out1, Out3, Out4.
    assert PHYSICAL_ORDER == (1, 0, 2, 3)


def test_to_physical_reorders_out_channels():
    # (Out1, Out2, Out3, Out4) = (a, b, c, d) -> physical (b, a, c, d)
    assert to_physical(("a", "b", "c", "d")) == ("b", "a", "c", "d")


def test_to_physical_rejects_wrong_width():
    with pytest.raises(ValueError):
        to_physical((1, 0, 1))


def test_centred_line_lights_the_middle_pair_in_out_order():
    # A centred line puts black on P2 and P3, which are Out1 and Out3.
    reading = (1, 0, 1, 0)  # Out1..Out4
    assert classify(reading).kind is Kind.ON_LINE


def test_table_is_total_over_all_sixteen_readings():
    assert set(STATE_TABLE) == set(itertools.product((0, 1), repeat=4))


def test_every_reading_classifies():
    for bits in itertools.product((0, 1), repeat=4):
        assert classify(bits, physical=True).bits == bits


def test_classify_rejects_non_binary():
    with pytest.raises(ValueError):
        classify((0, 1, 2, 0), physical=True)


@pytest.mark.parametrize(
    ("bits", "kind"),
    [
        ((0, 1, 1, 0), Kind.ON_LINE),
        ((0, 0, 1, 0), Kind.DRIFT),
        ((0, 1, 0, 0), Kind.DRIFT),
        ((0, 0, 0, 1), Kind.DRIFT),
        ((1, 0, 0, 0), Kind.DRIFT),
        ((0, 0, 0, 0), Kind.AMBIGUOUS),
        ((1, 1, 1, 1), Kind.JUNCTION),
        ((0, 1, 1, 1), Kind.JUNCTION),
        ((1, 1, 1, 0), Kind.JUNCTION),
        ((0, 0, 1, 1), Kind.JUNCTION),
        ((1, 1, 0, 0), Kind.JUNCTION),
        ((1, 0, 1, 0), Kind.NOISE),
        ((0, 1, 0, 1), Kind.NOISE),
        ((1, 0, 0, 1), Kind.NOISE),
        ((1, 0, 1, 1), Kind.NOISE),
        ((1, 1, 0, 1), Kind.NOISE),
    ],
)
def test_kind_of_each_reading(bits, kind):
    assert classify(bits, physical=True).kind is kind


def test_noise_states_are_exactly_the_non_contiguous_ones():
    def contiguous(bits):
        lit = [i for i, b in enumerate(bits) if b]
        return not lit or lit == list(range(lit[0], lit[-1] + 1))

    for bits, state in STATE_TABLE.items():
        assert (state.kind is Kind.NOISE) == (not contiguous(bits)), bits


def test_drift_directions_mirror():
    left = classify((0, 1, 0, 0), physical=True)
    right = classify((0, 0, 1, 0), physical=True)
    assert left.direction == -1
    assert right.direction == +1
    assert left.offset_cm == -right.offset_cm
    assert left.inner_ratio == right.inner_ratio


def test_outer_sensors_demand_a_harder_correction_than_inner():
    inner = classify((0, 0, 1, 0), physical=True)
    outer = classify((0, 0, 0, 1), physical=True)
    assert outer.inner_ratio < inner.inner_ratio


def test_centred_state_drives_straight():
    state = classify((0, 1, 1, 0), physical=True)
    assert state.direction == 0
    assert wheel_speeds(150, state.direction, state.inner_ratio) == (150, 150)


def test_noise_states_never_localise_the_line():
    for state in STATE_TABLE.values():
        if state.kind is Kind.NOISE:
            assert state.offset_cm is None
            assert state.direction == 0


# ---------------------------------------------------------------- 0000 logic


def test_blind_band_is_gap_minus_line_width():
    assert BLIND_BAND_CM == pytest.approx(0.8)
    assert LINE_WIDTH_CM == 2.0


def test_zero_reading_after_inner_sensor_is_the_blind_band_not_a_loss():
    verdict, offset = resolve_blind((0, 0, 1, 0))
    assert verdict == "blind"
    assert offset > 0  # line is right of centre, steer right

    verdict, offset = resolve_blind((0, 1, 0, 0))
    assert verdict == "blind"
    assert offset < 0


def test_zero_reading_after_outer_sensor_is_a_real_loss():
    assert resolve_blind((0, 0, 0, 1))[0] == "lost"
    assert resolve_blind((1, 0, 0, 0))[0] == "lost"


def test_zero_reading_straight_from_centred_is_treated_as_undulation():
    # The line cannot skip the 0010/0100 windows, so this is the paper lifting
    # every channel out of range — which reads as black on this hardware.
    assert resolve_blind((0, 1, 1, 0))[0] == "hold"


def test_zero_reading_with_no_history_is_a_loss():
    assert resolve_blind(None)[0] == "lost"


def test_blind_offset_sits_between_the_inner_and_outer_windows():
    _, offset = resolve_blind((0, 0, 1, 0))
    inner = classify((0, 0, 1, 0), physical=True).offset_cm
    outer = classify((0, 0, 0, 1), physical=True).offset_cm
    assert inner < offset < outer


# ------------------------------------------------------------------ geometry


def test_sensor_positions_match_the_measured_spacings():
    p1, p2, p3, p4 = SENSOR_POSITIONS_CM
    assert p2 - p1 == pytest.approx(2.8)
    assert p3 - p2 == pytest.approx(0.8)
    assert p4 - p3 == pytest.approx(2.8)
    assert p4 - p1 == pytest.approx(6.4)


def test_positions_are_symmetric_about_the_bar_centre():
    p1, p2, p3, p4 = SENSOR_POSITIONS_CM
    assert p1 == -p4
    assert p2 == -p3


def test_detection_limit_is_the_outer_sensor_plus_half_the_line():
    assert DETECTION_LIMIT_CM == pytest.approx(SENSOR_POSITIONS_CM[3] + LINE_WIDTH_CM / 2)


def test_a_two_cm_line_cannot_span_an_outer_gap():
    p1, p2, _, _ = SENSOR_POSITIONS_CM
    assert (p2 - p1) > LINE_WIDTH_CM  # which is why 1100 and 0011 are junctions


# -------------------------------------------------------------- wheel speeds


def test_wheel_speeds_slow_the_inside_wheel():
    assert wheel_speeds(150, +1, 0.4) == (150, 60)
    assert wheel_speeds(150, -1, 0.4) == (60, 150)
    assert wheel_speeds(150, 0, 0.4) == (150, 150)


def test_wheel_speeds_at_the_agreed_ladder():
    assert wheel_speeds(150, +1, classify((0, 0, 1, 0), physical=True).inner_ratio)[1] == 110
    assert wheel_speeds(150, +1, classify((0, 0, 0, 1), physical=True).inner_ratio)[1] == 20
