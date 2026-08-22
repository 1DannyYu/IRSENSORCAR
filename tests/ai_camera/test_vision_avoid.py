"""Tests for the visual + sonar obstacle fusion, with synthetic detections.

No camera and no car: the point of `carbot.vision_avoid` is that the patrol's
stop/turn decision is decidable from plain data. Frame size is the verified
IMX500 preview stream, 640x480.
"""

from __future__ import annotations

import pytest

from carbot.vision_avoid import (
    Detection,
    ObstaclePolicy,
    blocking_detections,
    fuse,
    is_blocking,
)

FRAME = (640, 480)
FW, FH = FRAME

# Class indices in the network's own 90-entry COCO-91 space, as reported by the
# SSD mobilenetv2 .rpk on this Pi. The 80-class COCO list puts "toilet" at 61.
CHAIR = 61
DINING_TABLE = 66
CLOCK = 84


def chair_ahead(confidence: float = 0.38) -> Detection:
    """The verified hardware case: chair box centred, low, and large."""
    return Detection(
        category=CHAIR, confidence=confidence, x=200, y=200, width=240, height=200, name="chair"
    )


# ------------------------------------------------------------------ Detection


def test_detection_rejects_a_corner_style_box():
    """Pitfall 7: convert_inference_coords returns (x, y, w, h), not corners."""
    with pytest.raises(ValueError, match="positive width and height"):
        Detection(category=CHAIR, confidence=0.4, x=147, y=245, width=14 - 147, height=185 - 245)


def test_detection_allows_a_box_clipped_past_the_frame_edge():
    d = Detection(category=CHAIR, confidence=0.4, x=-30, y=-10, width=120, height=140, name="chair")
    assert d.center_x == 30
    assert d.bottom == 130


def test_detection_geometry_and_area():
    d = Detection(category=CHAIR, confidence=0.4, x=100, y=100, width=200, height=100, name="chair")
    assert d.center_x == 200
    assert d.bottom == 200
    assert d.area_fraction(FW, FH) == pytest.approx((200 * 100) / (640 * 480))


def test_detection_label_falls_back_to_the_raw_index():
    """No label table here: an unnamed detection reports its index, never a guess."""
    assert chair_ahead().label() == "chair"
    assert Detection(category=61, confidence=0.4, x=0, y=0, width=10, height=10).label() == "61"


def test_area_fraction_rejects_a_degenerate_frame():
    with pytest.raises(ValueError):
        chair_ahead().area_fraction(0, 480)


# ----------------------------------------------------------------- is_blocking


def test_central_low_large_detection_blocks():
    assert is_blocking(chair_ahead(), FW, FH) is True


def test_low_confidence_detection_does_not_block():
    """Pitfall 8 in reverse: 0.30 must keep the 0.32-0.44 chairs, drop noise below it."""
    assert is_blocking(chair_ahead(confidence=0.29), FW, FH) is False
    assert is_blocking(chair_ahead(confidence=0.32), FW, FH) is True


def test_confidence_exactly_at_the_threshold_blocks():
    assert is_blocking(chair_ahead(confidence=0.30), FW, FH) is True


def test_off_centre_detection_does_not_block():
    """A box the car will drive past, not into: centre_x far from frame centre."""
    off = Detection(
        category=CHAIR, confidence=0.5, x=560, y=200, width=240, height=200, name="chair"
    )
    assert off.center_x == 680
    assert is_blocking(off, FW, FH) is False


def test_high_detection_does_not_block():
    """A wall clock is large and central but its box bottom is high in frame."""
    high = Detection(
        category=CLOCK, confidence=0.6, x=200, y=0, width=240, height=200, name="clock"
    )
    assert high.bottom == 200  # 0.42 of 480, above the 0.45 line
    assert is_blocking(high, FW, FH) is False


def test_small_detection_does_not_block():
    """Central and low, but too small a share of the frame to be in the way."""
    small = Detection(
        category=CHAIR, confidence=0.6, x=300, y=400, width=40, height=40, name="chair"
    )
    assert small.area_fraction(FW, FH) < 0.06
    assert is_blocking(small, FW, FH) is False


def test_policy_thresholds_are_honoured():
    small = Detection(
        category=CHAIR, confidence=0.6, x=300, y=400, width=40, height=40, name="chair"
    )
    permissive = ObstaclePolicy(min_area_fraction=0.001)
    assert is_blocking(small, FW, FH, permissive) is True


def test_is_blocking_rejects_a_degenerate_frame():
    with pytest.raises(ValueError):
        is_blocking(chair_ahead(), 0, 480)


def test_blocking_detections_keeps_input_order_and_filters():
    small = Detection(
        category=CHAIR, confidence=0.6, x=300, y=400, width=40, height=40, name="chair"
    )
    table = Detection(
        category=DINING_TABLE,
        confidence=0.32,
        x=180,
        y=230,
        width=280,
        height=180,
        name="dining table",
    )
    result = blocking_detections([small, chair_ahead(), table], FW, FH)
    assert result == (chair_ahead(), table)


# ------------------------------------------------------------------------ fuse


def test_clear_sonar_and_no_detections_is_clear():
    verdict = fuse(60.0, [], FRAME)
    assert verdict.blocked is False
    assert verdict.blocking == ()
    assert verdict.sonar_cm == 60.0
    assert "clear" in verdict.reason


def test_sonar_none_blocks_even_with_no_detections():
    """Pitfall 3: the HC-SR04 is blind below ~20 cm; None is never free space."""
    verdict = fuse(None, [], FRAME)
    assert verdict.blocked is True
    assert verdict.sonar_cm is None
    assert "no sonar reading" in verdict.reason


def test_near_sonar_blocks():
    verdict = fuse(25.0, [], FRAME)
    assert verdict.blocked is True
    assert "25 cm" in verdict.reason


def test_sonar_exactly_at_the_stop_distance_is_clear():
    assert fuse(30.0, [], FRAME).blocked is False
    assert fuse(29.9, [], FRAME).blocked is True


def test_vision_blocks_when_sonar_reads_clear():
    """The whole reason this module exists: the sonar sees under the chair."""
    verdict = fuse(85.0, [chair_ahead()], FRAME)
    assert verdict.blocked is True
    assert verdict.blocking == (chair_ahead(),)
    assert "chair 0.38" in verdict.reason
    assert "sonar 85 cm" in verdict.reason


def test_harmless_detection_leaves_the_path_clear():
    high = Detection(
        category=CLOCK, confidence=0.6, x=200, y=0, width=240, height=200, name="clock"
    )
    verdict = fuse(85.0, [high], FRAME)
    assert verdict.blocked is False
    assert verdict.blocking == ()


def test_sonar_none_and_vision_both_reported():
    verdict = fuse(None, [chair_ahead()], FRAME)
    assert verdict.blocked is True
    assert "no sonar reading" in verdict.reason
    assert "chair 0.38" in verdict.reason


def test_near_sonar_and_vision_both_reported():
    verdict = fuse(12.0, [chair_ahead()], FRAME)
    assert verdict.blocked is True
    assert "12 cm" in verdict.reason
    assert "chair 0.38" in verdict.reason


def test_verified_hardware_case_chair_plus_dining_table():
    """The pair the Pi actually reported when a chair and table sat ahead."""
    table = Detection(
        category=DINING_TABLE,
        confidence=0.32,
        x=180,
        y=230,
        width=280,
        height=180,
        name="dining table",
    )
    verdict = fuse(90.0, [chair_ahead(), table], FRAME)
    assert verdict.blocked is True
    assert len(verdict.blocking) == 2
    assert "chair 0.38" in verdict.reason
    assert "dining table 0.32" in verdict.reason


def test_custom_policy_reaches_the_fusion_reason():
    verdict = fuse(40.0, [], FRAME, ObstaclePolicy(sonar_stop_cm=50.0))
    assert verdict.blocked is True
    assert "40 cm < 50 cm" in verdict.reason


# -------------------------------------------------------------------- policy


@pytest.mark.parametrize(
    "kwargs",
    [
        {"confidence_threshold": 0.0},
        {"confidence_threshold": 1.5},
        {"center_x_fraction": 0.0},
        {"min_bottom_fraction": 1.5},
        {"min_area_fraction": -0.1},
        {"sonar_stop_cm": 0.0},
    ],
)
def test_policy_rejects_out_of_range_values(kwargs):
    with pytest.raises(ValueError):
        ObstaclePolicy(**kwargs)
