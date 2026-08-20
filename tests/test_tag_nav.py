"""Tests for the AprilTag-supervised navigation layer (tag_nav.py)."""

from __future__ import annotations

import pytest

from carbot.line_follow import LineReading
from carbot.line_nav import LineNav, NavPolicy, NavState
from carbot.tag_nav import DEPARTURE_ZONE, TagNavPolicy, TagSupervisedNav


def drive_to_follow(nav):
    """Departure confirm -> heading aligned (90 deg) -> stem seen 0.3 s."""
    nav.step(vertical_reading(), dt=0.1, localization=FakeLoc(*DEPARTURE_ZONE, heading=90.0))
    assert nav.state == "DEPART"
    for _ in range(6):
        nav.step(vertical_reading(), dt=0.1, localization=FakeLoc(*DEPARTURE_ZONE, heading=90.0))
    assert nav.state == "FOLLOW"
    return nav


class FakeLoc:
    def __init__(self, x: float, y: float, heading: float = 90.0):
        self.x_m = x
        self.y_m = y
        self.heading_deg = heading


def vertical_reading(error_fraction: float = 0.0) -> LineReading:
    return LineReading(
        visible=True,
        error_px=error_fraction * 320,
        error_fraction=error_fraction,
        centroid_x=320 + error_fraction * 320,
        line_width_px=10.0,
        dark_fraction=0.02,
        tracked_rows=12,
        roi=(48, 326, 0, 640),
        centroid_y=200.0,
        axis="vertical",
    )


def near_t_bar() -> LineReading:
    """A near cross-bar low in the ROI — the vision layer wants to spin."""
    return LineReading(
        visible=True,
        error_px=0.0,
        error_fraction=0.0,
        centroid_x=320.0,
        line_width_px=120.0,
        dark_fraction=0.02,
        tracked_rows=6,
        roi=(48, 326, 0, 640),
        centroid_y=296.0,  # frac (296-48)/(326-48) ≈ 0.89 > t_min_roi_y_fraction
        axis="horizontal",
    )


def make_nav() -> TagSupervisedNav:
    return TagSupervisedNav(
        nav=LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=0.0)),
        policy=TagNavPolicy(),
    )


def test_departure_requires_position_fix_in_zone():
    nav = make_nav()
    # No localization yet: hold (vision alone drifted off course 2026-08-17).
    cmd = nav.step(vertical_reading(), dt=0.1, localization=None)
    assert cmd.action == "hold"
    assert cmd.left == cmd.right == 0
    assert nav.state == "STARTING"
    # Localization outside the (wide) departure window: still starting.
    cmd = nav.step(vertical_reading(), dt=0.1, localization=FakeLoc(0.20, 0.05))
    assert cmd.action == "hold"
    assert nav.state == "STARTING"
    # Inside the zone: depart phase starts (align + blind creep).
    cmd = nav.step(vertical_reading(), dt=0.1, localization=FakeLoc(*DEPARTURE_ZONE))
    assert "departure confirmed" in cmd.reason
    assert nav.state == "DEPART"


def test_off_track_position_stops_the_car():
    nav = drive_to_follow(make_nav())
    cmd = nav.step(vertical_reading(), dt=0.1, localization=FakeLoc(0.45, 0.30))
    assert cmd.action == "off-track"
    assert cmd.left == cmd.right == 0
    assert nav.state == "OFF_TRACK"
    # Stays held.
    cmd = nav.step(vertical_reading(), dt=0.1, localization=FakeLoc(0.44, 0.35))
    assert cmd.left == cmd.right == 0


def test_turn_vetoed_until_position_reaches_t():
    nav = drive_to_follow(make_nav())
    # Vision sees the near T bar and wants to spin, but position is still
    # below the T — the spin must be vetoed (2026-08-17 early-turn failure).
    cmd = nav.step(near_t_bar(), dt=0.1, localization=FakeLoc(0.590, 0.19))
    assert "vetoed" in cmd.reason
    assert nav.nav.state is NavState.FOLLOW
    assert cmd.left == cmd.right == 200  # keeps driving straight


def test_turn_allowed_when_position_at_t():
    nav = drive_to_follow(make_nav())
    cmd = nav.step(near_t_bar(), dt=0.1, localization=FakeLoc(0.590, 0.25))
    assert "vetoed" not in cmd.reason
    assert nav.state == "TURN"
    assert cmd.left < 0 and cmd.right > 0  # the vision spin passes through


def test_policy_validation():
    with pytest.raises(ValueError):
        TagNavPolicy(position_corridor_m=0.0)
    with pytest.raises(ValueError):
        TagNavPolicy(depart_confirm_x_lo=0.7, depart_confirm_x_hi=0.5)


def test_stale_position_keeps_driving_on_vision_after_departure():
    nav = drive_to_follow(make_nav())
    # After departure, frames without a tag fix keep driving on vision.
    cmd = nav.step(vertical_reading(), dt=0.1, localization=None)
    assert cmd.action == "follow"
    assert cmd.left == cmd.right == 200
    # Even when the position goes stale, the black-line layer keeps driving.
    nav._last_pos_age = nav._policy.stale_position_s + 0.1
    cmd = nav.step(vertical_reading(), dt=0.1, localization=None)
    assert cmd.action == "follow"
    assert cmd.left == cmd.right == 200


def test_turn_is_fixed_duration_spin_then_follows():
    nav = drive_to_follow(make_nav())
    # At the T with right-turn evidence -> TURN state, spin command.
    cmd = nav.step(near_t_bar(), dt=0.1, localization=FakeLoc(0.590, 0.25))
    assert nav.state == "TURN"
    assert cmd.left < 0 and cmd.right > 0
    # Spin continues until the fixed 90 deg duration elapses.
    steps = 0
    while nav.state == "TURN" and steps < 100:
        cmd = nav.step(near_t_bar(), dt=0.1, localization=FakeLoc(0.590, 0.25))
        steps += 1
        if nav.state == "TURN":
            assert cmd.left < 0 and cmd.right > 0, f"still spinning at step {steps}"
    # After the duration: back to FOLLOW, straight command.
    assert nav.state == "FOLLOW"
    assert cmd.left == cmd.right > 0
    assert "Phase 2" in cmd.reason
    # The 90 deg spin at speed 200 is ~1.7 s -> ~17 steps.
    assert steps == 17
    # Cooldown: right-turn evidence right after the turn must not re-spin.
    cmd = nav.step(near_t_bar(), dt=0.1, localization=FakeLoc(0.590, 0.25))
    assert nav.state == "FOLLOW"
    assert cmd.left == cmd.right > 0
    assert "cooldown" in cmd.reason
