"""Tests for the line-navigation state machine, with synthetic readings.

No camera and no car: :class:`carbot.line_nav.LineNav` decides from plain
:class:`~carbot.line_follow.LineReading` values. Each test feeds a scripted
sequence of readings and checks the resulting wheel commands and state
transitions. The roundabout timings use the verified spin rate anchor (53.5
deg/s at speed 200 => one lap ~6.7 s), so a 6.5 s ``roundabout_loop_min_s``
only releases the exit fork after a real lap's worth of time.
"""

from __future__ import annotations

import pytest

from carbot.line_follow import LineReading
from carbot.line_nav import LineNav, NavPolicy, NavState, steer_command

ROI = (48, 326, 0, 640)


def near_t_bar(**kwargs) -> LineReading:
    """A crossing low in the ROI — wheels have reached the T (frac > 0.85)."""
    defaults = {
        "error_fraction": 0.0,
        "axis": "horizontal",
        "line_width": 120,
        "centroid_y": 296.0,  # (296-48)/(326-48) ≈ 0.89, above t_min_roi_y_fraction
    }
    defaults.update(kwargs)
    return line_reading(**defaults)


def line_reading(
    visible: bool = True,
    error_fraction: float = 0.0,
    junction: bool = False,
    branch_count: int = 1,
    line_width: float = 20.0,
    candidate_centroids: tuple[float, ...] | None = None,
    axis: str = "vertical",
    centroid_y: float | None = None,
    ground_u_px: float | None = None,
) -> LineReading:
    centroid = 320 + (error_fraction or 0) * 320
    if junction and branch_count < 2:
        branch_count = 2
    branches = (centroid,) + tuple(centroid + 80 * (i + 1) for i in range(branch_count - 1))
    return LineReading(
        visible=visible,
        error_px=error_fraction * 320 if error_fraction is not None else None,
        error_fraction=error_fraction,
        centroid_x=centroid,
        line_width_px=line_width,
        dark_fraction=0.02,
        tracked_rows=100,
        roi=ROI,
        centroid_y=centroid_y,
        branch_count=branch_count,
        branch_centroids=branches,
        junction=junction,
        candidate_centroids=candidate_centroids or branches,
        axis=axis,
        ground_u_px=ground_u_px,
    )


# ------------------------------------------------------------- steering


def test_ground_view_reading_is_not_recentred_by_the_raw_pixel_offset():
    """`expected_center_fraction` corrects a raw-pixel camera quirk that
    does not apply to ground-view readings — applying it anyway corrupted an
    already-correct near-zero BEV error into a large false one and veered
    the car right from frame one on every 2026-08-16 run (a reading with
    `centroid_x` at the raw-frame centre reads `error_fraction=-0.01` there,
    but `expected_center_fraction=0.46` used to turn that into `+0.27`)."""
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.46))
    reading = line_reading(error_fraction=-0.01, ground_u_px=150.0)
    cmd = nav.step(reading, dt=0.1)
    assert cmd.left == cmd.right == 200


def test_line_right_of_centre_slows_the_right_wheel():
    cmd = steer_command(line_reading(error_fraction=0.5), NavPolicy(speed=200))
    assert cmd.action == "follow"
    assert cmd.left == 200
    assert cmd.right < 200
    assert cmd.left > cmd.right


def test_line_left_of_centre_slows_the_left_wheel():
    cmd = steer_command(line_reading(error_fraction=-0.5), NavPolicy(speed=200))
    assert cmd.left < cmd.right
    assert cmd.right == 200


def test_centred_line_drives_straight():
    cmd = steer_command(line_reading(error_fraction=0.0), NavPolicy(speed=200))
    assert cmd.left == cmd.right == 200


def test_small_error_inside_deadband_drives_straight():
    cmd = steer_command(
        line_reading(error_fraction=0.06), NavPolicy(speed=200, steer_deadband=0.10)
    )
    assert cmd.left == cmd.right == 200


def test_turn_is_bounded_by_min_ratio():
    cmd = steer_command(line_reading(error_fraction=1.0), NavPolicy(speed=200, turn_gain=1.0))
    assert cmd.right == round(200 * NavPolicy().min_ratio)


def test_invisible_line_steers_nowhere():
    cmd = steer_command(line_reading(visible=False), NavPolicy(speed=200))
    assert cmd.left == cmd.right == 0


# ----------------------------------------------------------------- states


def test_follows_while_line_visible():
    nav = LineNav(NavPolicy(speed=200))
    cmd = nav.step(line_reading(error_fraction=0.2), dt=1.0 / 15)
    assert nav.state is NavState.FOLLOW
    assert cmd.action == "follow"


def test_lost_line_search_after_timeout():
    nav = LineNav(NavPolicy(speed=200, search_timeout_s=2.0))
    nav.step(line_reading(visible=False), dt=0.5)
    nav.step(line_reading(visible=False), dt=0.5)
    nav.step(line_reading(visible=False), dt=0.5)
    cmd = nav.step(line_reading(visible=False), dt=0.5)  # 2.0 s elapsed -> SEARCH
    assert nav.state is NavState.SEARCH
    assert cmd.action == "search"
    assert cmd.left != 0 or cmd.right != 0


def test_blind_creep_drives_straight_while_line_lost():
    """A forward-tilted low camera has a blind cone right under/just ahead
    of the wheels; a confidently-centred run can lose the line completely
    there while the chassis is still squarely on the route (2026-08-16).
    Sitting stopped can never recover from that — creep straight for the
    configured window before falling back to the stop-and-search default."""
    nav = LineNav(NavPolicy(speed=200, blind_creep_s=1.0, search_timeout_s=5.0))
    cmd = nav.step(line_reading(visible=False), dt=0.5)
    assert cmd.left == cmd.right == 200
    assert "blind creep" in cmd.reason
    assert nav.state is NavState.FOLLOW


def test_blind_creep_falls_back_to_stop_after_its_window():
    nav = LineNav(NavPolicy(speed=200, blind_creep_s=0.5, search_timeout_s=5.0))
    nav.step(line_reading(visible=False), dt=0.3)
    cmd = nav.step(line_reading(visible=False), dt=0.3)  # 0.6s > 0.5s window
    assert cmd.left == cmd.right == 0
    assert "waiting to search" in cmd.reason


def test_blind_creep_resets_once_the_line_reappears():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, blind_creep_s=1.0))
    nav.step(line_reading(visible=False), dt=0.4)
    nav.step(line_reading(error_fraction=0.0), dt=0.1)  # line reappears
    cmd = nav.step(line_reading(visible=False), dt=0.4)  # a later, separate gap
    assert cmd.left == cmd.right == 200
    assert "blind creep" in cmd.reason


def test_search_holds_still_when_sweep_disabled():
    """If search_sweep_deg is 0, search holds still."""
    nav = LineNav(NavPolicy(speed=200, search_timeout_s=0.0, search_sweep_deg=0.0))
    first = nav.step(line_reading(visible=False), dt=0.1)
    second = nav.step(line_reading(visible=False), dt=0.1)
    assert first.left == first.right == 0
    assert second.left == second.right == 0


def test_search_visual_sweep_oscillates_and_reacquires():
    """SEARCH state oscillates wheels in small steps to scan for lost line, reacquiring when centered."""
    nav = LineNav(
        NavPolicy(
            speed=200,
            search_timeout_s=0.0,
            search_sweep_deg=20.0,
            search_spin_speed_ratio=0.75,
            search_give_up_s=3.0,
            reacquire_error=0.30,
        )
    )
    # Step into SEARCH: first step spins left (-150, 150)
    cmd1 = nav.step(line_reading(visible=False), dt=0.1)
    assert nav.state is NavState.SEARCH
    assert cmd1.action == "search"
    assert cmd1.left == -150
    assert cmd1.right == 150
    assert "visual sweep left" in cmd1.reason

    # Mid-search: centered line appears -> reacquires FOLLOW
    cmd2 = nav.step(line_reading(error_fraction=0.10), dt=0.1)
    assert nav.state is NavState.FOLLOW
    assert cmd2.action == "follow"


def test_reacquired_line_returns_to_follow():
    nav = LineNav(NavPolicy(speed=200, search_timeout_s=1.0, reacquire_error=0.40))
    nav.step(line_reading(visible=False), dt=1.1)
    assert nav.state is NavState.SEARCH
    cmd = nav.step(line_reading(error_fraction=-0.3), dt=0.1)
    assert nav.state is NavState.FOLLOW
    assert cmd.action == "follow"


def test_persistent_junction_enters_roundabout():
    nav = LineNav(NavPolicy(speed=200, junction_min_s=1.0, enable_roundabout=True))
    nav.step(line_reading(error_fraction=0.0), dt=0.5)  # baseline width 20
    nav.step(line_reading(error_fraction=0.0, junction=True, line_width=50), dt=0.5)
    assert nav.state is NavState.FOLLOW  # under junction_min_s
    nav.step(line_reading(error_fraction=0.0, junction=True, line_width=50), dt=0.5)
    assert nav.state is NavState.ROUNDABOUT


def test_junction_without_width_jump_stays_follow():
    """Environment shadows fork the reading but keep the line narrow: no roundabout."""
    nav = LineNav(NavPolicy(speed=200, junction_min_s=0.5))
    nav.step(line_reading(error_fraction=0.0), dt=0.5)  # baseline 20
    nav.step(line_reading(error_fraction=0.0, junction=True, line_width=22), dt=0.5)
    nav.step(line_reading(error_fraction=0.0, junction=True, line_width=22), dt=0.5)
    assert nav.state is NavState.FOLLOW


def test_roundabout_stays_until_lap_time_elapsed():
    nav = LineNav(
        NavPolicy(speed=200, junction_min_s=0.1, roundabout_loop_min_s=6.5, enable_roundabout=True)
    )
    nav.step(line_reading(error_fraction=0.1), dt=0.1)  # baseline
    for _ in range(5):
        nav.step(line_reading(error_fraction=0.1, junction=True, line_width=50), dt=0.1)
    assert nav.state is NavState.ROUNDABOUT
    # 3 s in, no exit fork yet
    for _ in range(25):
        cmd = nav.step(line_reading(error_fraction=0.1), dt=0.1)
    assert nav.state is NavState.ROUNDABOUT
    assert cmd.reason.startswith("roundabout:")


def test_roundabout_exits_after_lap_and_fork():
    nav = LineNav(
        NavPolicy(speed=200, junction_min_s=0.1, roundabout_loop_min_s=2.0, enable_roundabout=True)
    )
    nav.step(line_reading(error_fraction=0.1), dt=0.1)  # baseline
    for _ in range(5):
        nav.step(line_reading(error_fraction=0.1, junction=True, line_width=50), dt=0.1)
    # drive the lap: 2.0 s with no junction
    for _ in range(20):
        nav.step(line_reading(error_fraction=0.1), dt=0.1)
    assert nav.state is NavState.ROUNDABOUT
    # exit fork after the lap minimum
    cmd = nav.step(line_reading(error_fraction=0.1, junction=True, line_width=50), dt=0.1)
    assert nav.state is NavState.FOLLOW
    assert "roundabout exit" in cmd.reason


def test_roundabout_does_not_exit_before_lap_minimum():
    nav = LineNav(
        NavPolicy(speed=200, junction_min_s=0.1, roundabout_loop_min_s=10.0, enable_roundabout=True)
    )
    nav.step(line_reading(error_fraction=0.1), dt=0.1)  # baseline
    for _ in range(5):
        nav.step(line_reading(error_fraction=0.1, junction=True, line_width=50), dt=0.1)
    for _ in range(20):
        nav.step(line_reading(error_fraction=0.1), dt=0.1)
    nav.step(line_reading(error_fraction=0.1, junction=True, line_width=50), dt=0.1)
    assert nav.state is NavState.ROUNDABOUT


def test_lookahead_target_is_not_overridden_by_stale_lock():
    """A centred path must steer straight even if the previous frame was offset."""
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, max_error_jump=1.5))
    nav.step(line_reading(error_fraction=-1.0, candidate_centroids=(0.0, 320.0)), dt=0.1)
    cmd = nav.step(line_reading(error_fraction=0.0, candidate_centroids=(0.0, 320.0)), dt=0.1)
    assert cmd.action == "follow"
    assert cmd.left == cmd.right == 200


def test_horizontal_stroke_spins_to_align():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=0.0))
    cmd = nav.step(near_t_bar(), dt=0.1)
    assert cmd.action == "search"
    assert cmd.left == -200
    assert cmd.right == 200


def test_far_thin_crossing_does_not_spin():
    """Forward camera sees the T while wheels are still on the stem."""
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=0.0))
    cmd = nav.step(
        line_reading(error_fraction=0.0, axis="horizontal", line_width=18, centroid_y=120.0),
        dt=0.1,
    )
    assert cmd.left == cmd.right == 200
    assert "spin" not in cmd.reason
    assert "straight" in cmd.reason


def test_off_center_bar_does_not_start_a_t_turn():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5))
    cmd = nav.step(line_reading(error_fraction=-0.33, axis="horizontal", line_width=40), dt=0.1)
    assert cmd.right != -200
    assert "spin" not in cmd.reason


def test_later_crossbar_does_not_restart_t_turn_after_follow():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5))
    nav.step(line_reading(error_fraction=-0.12, axis="vertical", line_width=100), dt=0.2)
    nav.step(line_reading(error_fraction=-0.06, axis="vertical", line_width=100), dt=0.2)
    cmd = nav.step(near_t_bar(line_width=46), dt=0.1)
    assert "spin" not in cmd.reason


def test_horizontal_stroke_stops_after_a_right_angle():
    # Vision-guided spin: the hard ceiling is 2x the nominal 90-degree time
    # (53.5 deg/s -> 1.68 s, ceiling ~3.36 s). After the ceiling the car
    # stops spinning even if no vertical line ever confirmed alignment.
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=0.0))
    cmd = None
    for _ in range(40):
        cmd = nav.step(near_t_bar(), dt=0.1)
    assert cmd is not None
    assert cmd.left == cmd.right == 200
    assert "spin" not in cmd.reason


def test_turn_completes_early_when_aligned():
    # The spin finishes as soon as the camera sees the next vertical line
    # centred and thick (>=0.7 of the nominal spin already elapsed).
    nav = LineNav(
        NavPolicy(
            speed=200,
            expected_center_fraction=0.5,
            right_turn_after_s=0.0,
            spin_deg_per_s_at_200=90.0,
        )
    )
    for _ in range(8):
        nav.step(near_t_bar(), dt=0.1)  # 0.8 s of spin
    cmd = nav.step(line_reading(error_fraction=0.0, axis="vertical", line_width=120), dt=0.1)
    assert "spin" not in cmd.reason
    assert cmd.left == cmd.right == 200


def test_start_box_keeps_straight_until_the_t():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=2.5))
    cmd = nav.step(near_t_bar(), dt=0.1)
    assert cmd.left == cmd.right == 200
    assert "straight" in cmd.reason or "keep straight" in cmd.reason


def test_error_jump_stops_then_searches():
    nav = LineNav(
        NavPolicy(
            speed=200,
            expected_center_fraction=0.5,
            max_error_jump=0.35,
            jump_search_s=0.3,
        )
    )
    first = nav.step(line_reading(error_fraction=0.0), dt=0.1)
    assert first.left == first.right == 200
    stopped = nav.step(line_reading(error_fraction=-0.9), dt=0.1)
    assert stopped.left == stopped.right == 0
    assert "jump: stop" in stopped.reason
    assert nav.state is NavState.FOLLOW
    searching = nav.step(line_reading(error_fraction=-0.9), dt=0.3)
    assert nav.state is NavState.SEARCH
    assert searching.action == "search"
    assert searching.left != 0 or searching.right != 0


def test_search_ignores_far_edge_lock():
    nav = LineNav(
        NavPolicy(speed=200, search_timeout_s=0.0, reacquire_error=0.40, search_sweep_deg=0.0)
    )
    nav.step(line_reading(visible=False), dt=0.1)
    assert nav.state is NavState.SEARCH
    cmd = nav.step(line_reading(error_fraction=-0.9), dt=0.1)
    assert nav.state is NavState.SEARCH
    assert cmd.left == cmd.right == 0


def test_search_give_up_stops():
    nav = LineNav(NavPolicy(speed=200, search_timeout_s=0.0, search_give_up_s=0.5))
    nav.step(line_reading(visible=False), dt=0.1)
    nav.step(line_reading(visible=False), dt=0.3)
    cmd = nav.step(line_reading(visible=False), dt=0.3)
    assert cmd.left == cmd.right == 0
    assert "give up" in cmd.reason


def test_junction_prefers_the_right_branch():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=0.0))
    nav.step(line_reading(error_fraction=0.0, line_width=20), dt=0.1)
    cmd = nav.step(
        line_reading(error_fraction=0.0, junction=True, branch_count=2, line_width=50),
        dt=0.1,
    )
    assert cmd.action == "follow"
    assert cmd.left == 200
    assert cmd.right < 200


def test_false_junction_does_not_retarget():
    """Forward camera always sees far forks; only a widened local line is a T."""
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5))
    nav.step(line_reading(error_fraction=0.0, line_width=20), dt=0.1)
    cmd = nav.step(
        line_reading(error_fraction=0.0, junction=True, branch_count=2, line_width=22),
        dt=0.1,
    )
    assert cmd.left == cmd.right == 200


def test_fat_junction_does_not_poison_line_width():
    nav = LineNav(NavPolicy(speed=200, min_width_ratio=0.5))
    nav.step(line_reading(error_fraction=0.0, line_width=130), dt=0.1)
    nav.step(line_reading(error_fraction=0.0, junction=True, line_width=350), dt=0.1)
    cmd = nav.step(line_reading(error_fraction=0.05, line_width=128), dt=0.1)
    assert cmd.left > 0 and cmd.right > 0
    assert "thin" not in cmd.reason


def test_rightward_junction_jump_is_followed():
    nav = LineNav(
        NavPolicy(
            speed=200, expected_center_fraction=0.5, max_error_jump=0.35, right_turn_after_s=0.0
        )
    )
    nav.step(line_reading(error_fraction=0.0), dt=0.1)
    cmd = nav.step(line_reading(error_fraction=0.45, junction=True), dt=0.1)
    assert cmd.left == 200
    assert cmd.right < cmd.left
    assert "jump" not in cmd.reason


def test_thin_lock_is_held_not_followed():
    nav = LineNav(NavPolicy(speed=200, min_width_ratio=0.5))
    nav.step(line_reading(error_fraction=0.0, line_width=150), dt=0.1)
    cmd = nav.step(line_reading(error_fraction=0.3, line_width=60), dt=0.1)
    assert cmd.left == cmd.right == 0
    assert "thin" in cmd.reason


def test_centered_narrower_line_is_still_followed():
    nav = LineNav(NavPolicy(speed=200, min_width_ratio=0.5, steer_deadband=0.10))
    nav.step(line_reading(error_fraction=0.0, line_width=120), dt=0.1)
    cmd = nav.step(line_reading(error_fraction=0.0, line_width=54), dt=0.1)
    assert cmd.left == cmd.right == 200
    assert "thin" not in cmd.reason


def test_t_turn_keeps_spinning_through_vertical_flicker():
    nav = LineNav(NavPolicy(speed=200, expected_center_fraction=0.5, right_turn_after_s=0.0))
    nav.step(near_t_bar(), dt=0.1)
    cmd = nav.step(line_reading(error_fraction=0.14, axis="vertical", line_width=113), dt=0.1)
    assert cmd.left == -200
    assert cmd.right == 200
    assert "spin" in cmd.reason


def test_t_turn_does_not_spin_a_second_time():
    nav = LineNav(
        NavPolicy(
            speed=200,
            expected_center_fraction=0.5,
            spin_deg_per_s_at_200=90.0,
            right_turn_after_s=0.0,
        )
    )
    for _ in range(12):
        nav.step(near_t_bar(), dt=0.1)
    follow = nav.step(line_reading(error_fraction=0.0, axis="vertical", line_width=120), dt=0.1)
    assert follow.left == follow.right == 200
    later = nav.step(near_t_bar(), dt=0.1)
    assert later.left == later.right == 200
    assert "spin" not in later.reason


def test_first_right_spins_then_resumes_follow():
    nav = LineNav(
        NavPolicy(
            speed=200,
            first_right_s=0.3,
            first_right_deg=90.0,
            spin_deg_per_s_at_200=90.0,
        )
    )
    nav.step(line_reading(error_fraction=0.0), dt=0.2)
    cmd = nav.step(line_reading(error_fraction=0.0), dt=0.2)
    assert nav.state is NavState.RIGHT_TURN
    assert cmd.left == -200
    assert cmd.right == 200
    nav.step(line_reading(error_fraction=0.0), dt=0.5)
    cmd = nav.step(line_reading(error_fraction=0.0), dt=0.6)
    assert nav.state is NavState.FOLLOW
    assert cmd.left == cmd.right == 200


def test_nav_rejects_negative_dt():
    nav = LineNav()
    with pytest.raises(ValueError):
        nav.step(line_reading(), dt=-1.0)


def test_policy_rejects_invalid_values():
    with pytest.raises(ValueError):
        NavPolicy(speed=1001)
    with pytest.raises(ValueError):
        NavPolicy(min_ratio=0.9, max_ratio=0.1)
    with pytest.raises(ValueError):
        NavPolicy(roundabout_loop_min_s=0)
