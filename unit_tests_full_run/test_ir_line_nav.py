"""Tests for the IR line-nav state machine, with synthetic sensor readings.

No hardware: :class:`carbot.ir_line_nav.IRLineNav` decides from plain
:class:`~carbot.ir_line_nav.IRLineReading` values built directly (the same
pattern as ``test_line_nav.py``). Covers proportional follow, junction approach-sequence
matching, the closed-loop turn, and the line-recovery search: on a lost line the car pivots
in place (never creeps forward), sweeping ``search_sweep_min_deg`` left then back through
centre to the same angle right; if that finds nothing the angle steps up by
``search_sweep_step_deg`` (5 -> 10 -> 15 -> ... -> ``search_sweep_deg``) until the line is
seen again.
"""

from __future__ import annotations

import pytest

from carbot.ir_line_nav import IRLineNav, IRNavPolicy, IRNavState, make_reading
from carbot.ir_route import (
    TASK1_LOOP_ONLY,
    TASK1_ROUTE,
    CornerWindow,
    JunctionAction,
    RouteJunction,
    RoutePlan,
    SequenceStep,
)

# Raw Out1..Out4 tuples -> physical P1..P4 after to_physical (PHYSICAL_ORDER swaps 0,1).
CENTRED = (1, 0, 1, 0)  # physical 0110 — P2+P3, the only two-sensor line reading
CROSSBAR = (1, 1, 1, 1)  # physical 1111 — symmetric, the start stem / roundabout entry
GAP = (0, 0, 0, 0)  # physical 0000 — blind band, a real loss, or a T/entry approach trigger
DRIFT_RIGHT = (0, 0, 1, 0)  # physical 0010 — P3 only
DRIFT_LEFT = (1, 0, 0, 0)  # physical 0100 — P2 only (also the roundabout exit's 3rd step)
FAR_RIGHT = (0, 0, 0, 1)  # physical 0001 — P4 only
FAR_LEFT = (0, 1, 0, 0)  # physical 1000 — P1 only
RIGHT_BRANCH_0111 = (1, 0, 1, 1)  # physical 0111 — T-junction/roundabout-exit approach start
ROUNDABOUT_ENTRY_SHOULDER = (0, 1, 0, 1)  # physical 1001 — roundabout entry's middle step
ROUNDABOUT_EXIT_NOISE = (1, 0, 0, 1)  # physical 0101 — roundabout exit's 2nd step, Kind.NOISE


def default_nav(**policy_kwargs) -> IRLineNav:
    return IRLineNav(IRNavPolicy(**policy_kwargs))


# ------------------------------------------------------------- follow


def test_follow_centered_drives_straight():
    nav = default_nav()
    cmd = nav.step(make_reading(CENTRED), dt=0.01)
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right == 150


def test_follow_steers_toward_the_line():
    # Only P3 lit -> the line sits right of the bar centre, so the car has
    # drifted left and must steer right: slow the right wheel.
    # 2026-08-20, fifth pass: the start stem (default_nav's initial pending) now holds on
    # DRIFT instead of steering (see IRLineNav._follow_step) -- start on the loop instead so
    # this exercises the generic DRIFT steering math it is actually testing.
    nav = default_nav(route=TASK1_LOOP_ONLY)
    cmd = nav.step(make_reading(DRIFT_RIGHT), dt=0.01)
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.right < cmd.left == 150


def test_follow_steers_the_mirror_way_for_the_mirror_reading():
    nav = default_nav(route=TASK1_LOOP_ONLY)
    right = nav.step(make_reading(DRIFT_RIGHT), dt=0.01)
    nav = default_nav(route=TASK1_LOOP_ONLY)
    left = nav.step(make_reading(DRIFT_LEFT), dt=0.01)
    assert (right.left, right.right) == (left.right, left.left)


def test_outer_sensor_demands_a_harder_correction():
    nav = default_nav(route=TASK1_LOOP_ONLY)
    slight = nav.step(make_reading(DRIFT_RIGHT), dt=0.01)
    nav = default_nav(route=TASK1_LOOP_ONLY)
    hard = nav.step(make_reading(FAR_RIGHT), dt=0.01)
    assert hard.right < slight.right


def test_the_start_stem_holds_on_drift_instead_of_steering():
    """2026-08-20, fifth pass, real-track: a lone P1000 (far left, outer sensor only)
    partway through the dead-straight start stem steered the car hard left. The stem is
    print-straight end to end, so any DRIFT reading not part of the T's approach sequence is
    the docstring's "0001/1000 just before the post-crossbar 0000" transitional artifact, not
    genuine physical drift -- hold instead."""
    nav = default_nav()  # default route's initial pending is the start stem T
    cmd = nav.step(make_reading(FAR_LEFT), dt=0.01)
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right == 150


def test_line_lost_enters_search_with_left_sweep():
    nav = default_nav()
    cmd = nav.step(make_reading(GAP), dt=0.1)
    assert cmd.state is IRNavState.SEARCH
    assert cmd.left == -150 and cmd.right == 150  # spinning left
    assert "pivot left" in cmd.reason


# --------------------------------------------------------- junction approach sequences
#
# 2026-08-20 third pass: every real junction produces an ORDERED sequence of readings, not
# one sustained reading. `_approach_step` tracks progress through
# `RouteJunction.approach`; only the last step completing counts as arrival.


def _single_junction_nav(junction: RouteJunction, **policy_kwargs) -> IRLineNav:
    """A nav whose only pending junction is `junction` (0cm gate), so approach-sequence
    behaviour can be checked in isolation."""
    plan = RoutePlan(prologue=(), loop=(junction,))
    return default_nav(route=plan, **policy_kwargs)


def test_approach_step_holds_while_the_first_steps_persistence_is_unmet():
    junction = RouteJunction(
        "x",
        JunctionAction.TURN_RIGHT,
        0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=2.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0,
        turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # 1cm < 2cm required
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right == 150  # held centred (no history yet -> centred default)
    assert "approaching x, step 1/2" in cmd.reason
    assert "1.00/2.00cm" in cmd.reason


def test_approach_step_does_not_steer_on_the_confirming_readings_offset():
    """The whole point of holding instead of steering: RIGHT_BRANCH_0111 has a real offset
    under STATE_TABLE, but during approach that offset must never reach the wheels."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((0, 1, 1, 1), min_cm=2.0), SequenceStep((0, 1, 1, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CENTRED), 0.01)  # steady last-good command
    cmd = nav.step(make_reading(RIGHT_BRANCH_0111), 0.1)  # 1cm < 2cm required
    assert cmd.left == cmd.right == 150
    assert "holding previous" in cmd.reason


def test_approach_step_advances_past_a_satisfied_step():
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    # 1cm satisfies step 1's min_cm on this very call, which advances immediately -- the
    # returned reason already reflects step 2, not step 1.
    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert "step 2/2" in cmd.reason


def test_approach_step_completes_and_commits_a_turn():
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # step 1 satisfied
    cmd = nav.step(make_reading(GAP), 0.1)  # step 2 (0000) -> arrival
    assert cmd.state is IRNavState.JUNCTION_CREEP
    assert nav.last_junction == "x"


def test_any_reading_is_ignored_during_the_blind_window_not_just_0000_or_junction():
    """2026-08-20, seventh pass: the 0000/Kind.JUNCTION carve-out wasn't enough -- any
    misclassified frame mid-crossbar could still interrupt the accumulation. Once a step has
    genuinely started and more than 0.1s remains before its min_cm is satisfied, every reading
    is ignored outright (not just 0000/junction-shaped ones) and elapsed distance just
    accumulates -- including an ordinary CENTRED (0110) reading, which would otherwise read as
    "back on the line, not approaching a junction at all"."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=3.0),),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.15)  # 1.5/3.0cm, 0.15s of 1.5cm remaining -- still blind
    cmd = nav.step(make_reading(CENTRED), 0.05)  # 0.5cm more; would normally mean "centred"
    assert cmd.state is IRNavState.FOLLOW
    assert "blind window" in cmd.reason
    assert "2.00/3.00cm" in cmd.reason


def test_a_stray_0000_blip_mid_crossbar_does_not_reset_the_approach():
    """2026-08-20, sixth pass, real-track: the start T's single-step crossbar accumulation
    was reset by a stray 0000 flicker mid-crossbar, so it kept restarting and never reached
    its min_cm -- the car oscillated at the junction instead of committing to the turn. A
    0000 that is not this junction's own expected next step must be ignored, not a reset."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.5),),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # 1.0/1.5cm
    blip = nav.step(make_reading(GAP), 0.02)  # stray flicker, not this junction's next step
    assert blip.state is IRNavState.FOLLOW
    assert "holding previous" in blip.reason
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # resumes from 1.0cm, not reset to 0
    assert cmd.state is IRNavState.JUNCTION_CREEP


def test_a_reading_matching_neither_step_resets_and_falls_through():
    """A stray frame that matches neither the tracked step nor the next one resets progress
    to step 0 and is handled as ordinary FOLLOW (not held as noise)."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # step 1 satisfied -> now tracking step 2 (0000)
    cmd = nav.step(make_reading(CENTRED), 0.1)  # unrelated: ordinary centred line
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.reason == "centred"
    # And the sequence really did reset: CROSSBAR again must re-satisfy step 1 from scratch.
    cmd = nav.step(make_reading(CROSSBAR), 0.5)  # comfortably >= 1cm
    assert "step 2/2" in cmd.reason


def test_a_skewed_approach_reading_before_the_first_step_holds():
    """2026-08-20 real-track observation: 0111/1110 commonly appear just BEFORE a symmetric
    1111 (and 0001/1000 just before the post-crossbar 0000) as the car approaches a real
    crossbar from a skewed angle -- an ordinary, expected transitional reading, not noise.

    2026-08-20, fourth pass: a real two-lap run put a lone P1110 partway through the
    dead-straight start stem, and the old "not started -> steer like a curve" rule turned
    that into a hard left turn. STATE_TABLE's offset for Kind.JUNCTION entries was only ever
    assigned by analogy and never validated for direction -- it must hold instead, whether or
    not any approach progress has been made yet."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=2.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    # 1110 ("branch or curve on the left") is Kind.JUNCTION but not this junction's step 0 --
    # no approach progress has been made yet, but it must still hold, not steer.
    cmd = nav.step(make_reading((1, 1, 1, 0)), 0.1)
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right  # held at the initial straight command, not steered


def test_bounded_curve_mode_steers_on_three_sensor_curve_patterns() -> None:
    """Bounded curve tests may use a three-sensor pattern's directional offset."""
    policy = {
        "route": TASK1_LOOP_ONLY,
        "junction_detection_enabled": False,
        "curve_pattern_steering_enabled": True,
    }
    right = default_nav(**policy).step(make_reading(RIGHT_BRANCH_0111), 0.01)
    left = default_nav(**policy).step(make_reading((1, 1, 1, 0)), 0.01)  # physical P1110

    assert (right.left, right.right) == (150, 60)
    assert (left.left, left.right) == (60, 150)
    assert "bounded curve mode" in right.reason
    assert "bounded curve mode" in left.reason


def test_bounded_left_curve_ignores_every_right_side_pattern() -> None:
    """Raised paper produced false black on ARC 1's right side during the real-track run.

    ARC 1 is a known left turn, so neither ordinary right DRIFT nor a junction-shaped
    right pattern may overwrite the last safe left command or localising history.
    """
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        junction_detection_enabled=False,
        curve_pattern_steering_enabled=True,
        steering_direction_limit=-1,
    )
    safe_left = nav.step(make_reading(DRIFT_LEFT), 0.01)
    assert safe_left.left < safe_left.right
    centred = nav.step(make_reading(CENTRED), 0.01)
    assert (centred.left, centred.right) == (safe_left.left, safe_left.right)
    assert "maintaining left curvature" in centred.reason

    for raw in (DRIFT_RIGHT, FAR_RIGHT, (0, 0, 1, 1), RIGHT_BRANCH_0111):
        ignored = nav.step(make_reading(raw), 0.01)
        assert (ignored.left, ignored.right) == (safe_left.left, safe_left.right)
        assert "opposite-side noise ignored" in ignored.reason
        assert "maintaining last permitted curve correction" in ignored.reason

    assert nav._last_localising == (0, 1, 1, 0)


def test_bounded_left_curve_keeps_turning_while_sensor_is_centred() -> None:
    """P0110 means centred, not straight, when the mapped segment itself is an ARC."""
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        junction_detection_enabled=False,
        curve_pattern_steering_enabled=True,
        steering_direction_limit=-1,
    )
    centred = nav.step(make_reading(CENTRED), 0.01)
    assert (centred.left, centred.right) == (110, 150)
    assert "maintaining left curvature" in centred.reason


def test_bounded_left_curve_uses_gentle_fallback_if_first_reading_is_forbidden() -> None:
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        junction_detection_enabled=False,
        curve_pattern_steering_enabled=True,
        steering_direction_limit=-1,
    )
    ignored = nav.step(make_reading(RIGHT_BRANCH_0111), 0.01)
    assert (ignored.left, ignored.right) == (110, 150)
    assert "gentle curve-side fallback" in ignored.reason


def test_bounded_left_curve_search_never_issues_a_right_pivot() -> None:
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        junction_detection_enabled=False,
        curve_pattern_steering_enabled=True,
        steering_direction_limit=-1,
        search_sweep_deg=5.0,
        search_sweep_min_deg=5.0,
        off_track_dwell_s=100.0,
    )
    nav.step(make_reading(FAR_LEFT), 0.01)
    searching = nav.step(make_reading(GAP), 0.01)
    assert searching.state is IRNavState.SEARCH
    assert (searching.left, searching.right) == (-150, 150)
    assert "one-way pivot left" in searching.reason

    # A forbidden right-side hit is ignored even while searching; it cannot end SEARCH or
    # produce an L150/R-150 right pivot.
    still_searching = nav.step(make_reading(FAR_RIGHT), 0.1)
    assert still_searching.state is IRNavState.SEARCH
    assert (still_searching.left, still_searching.right) == (-150, 150)
    assert "opposite-side noise ignored" in still_searching.reason

    stopped = nav.step(make_reading(GAP), 1.0)
    assert stopped.state is IRNavState.FAILED
    assert (stopped.left, stopped.right) == (0, 0)
    assert "one-way search left reached 5deg" in stopped.reason


def test_bounded_curve_mode_holds_symmetric_crossbar_without_guessing() -> None:
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        junction_detection_enabled=False,
        curve_pattern_steering_enabled=True,
    )
    previous = nav.step(make_reading(DRIFT_RIGHT), 0.01)
    crossbar = nav.step(make_reading(CROSSBAR), 0.01)
    assert (crossbar.left, crossbar.right) == (previous.left, previous.right)
    assert "holding previous" in crossbar.reason


def test_a_junction_shaped_reading_that_breaks_a_started_sequence_holds():
    """Unlike the skewed-approach case above, a reading that interrupts a sequence already
    partway matched is close to a real junction and about to commit to an action -- hold
    instead of steering hard on an offset that risks throwing off an approach already mostly
    confirmed (2026-08-20 real-track regression, distinct from the case above)."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=2.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # step 0 (1111) partway satisfied: index/cm > 0
    # An unrelated junction-shaped reading now breaks the in-progress sequence.
    cmd = nav.step(make_reading((1, 1, 1, 0)), 0.1)
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right  # held, not steered left on the -1.6cm offset
    assert "broke x's approach mid-sequence" in cmd.reason


def test_a_lingering_noise_echo_of_a_just_matched_step_does_not_reset_progress():
    """2026-08-20, tenth pass, real-track: the roundabout entry's approach twice got within
    one step of completing and was wiped both times by a lingering Kind.NOISE echo of the
    step just matched (the sensor takes a frame or two to actually clear once the index has
    already moved on to the next step). Kind.NOISE must hold without resetting, same as
    Kind.JUNCTION -- only a genuine ON_LINE/DRIFT reading is real evidence of leaving the
    junction area."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=2.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # step 0 (1111) partway satisfied: 1.0/2.0cm
    noise = (1, 1, 0, 1)  # physical 1101, Kind.NOISE ("P3 dropped out"), not the current
    # nor next step's target and not ON_LINE/DRIFT.
    cmd = nav.step(make_reading(noise), 0.1)
    assert cmd.state is IRNavState.FOLLOW
    assert "broke x's approach mid-sequence" in cmd.reason
    # Progress was preserved (not reset to 0): the crossbar only needs 1.0cm more, not 2.0.
    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert "step 2/2" in cmd.reason


def test_a_single_frame_drift_blip_does_not_reset_near_complete_progress():
    """2026-08-20, eleventh pass, real-track: the roundabout entry's approach reached step
    2/3 (the 1001 shoulder) and was reset by a single frame of genuine Kind.DRIFT (0001, "far
    right") sitting between the shoulder and the step's own 0000 -- a real single-line-shaped
    reading, so it must be sustained (approach_break_confirm_s) before it is trusted enough to
    reset, at a realistic ~100Hz dt. A reading that goes back to matching/holding resets the
    confirmation clock rather than carrying it over."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=5.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # step 0 satisfied -> step 1/2
    blip = nav.step(make_reading(FAR_RIGHT), 0.01)  # one frame of genuine DRIFT
    assert blip.state is IRNavState.FOLLOW
    assert "possible break" in blip.reason
    # A junction-shaped blip in between resets the confirmation clock, not the approach itself.
    nav.step(make_reading(RIGHT_BRANCH_0111), 0.01)
    cmd = nav.step(make_reading(FAR_RIGHT), 0.01)  # confirming again, only 0.01s in
    assert "possible break" in cmd.reason
    for _ in range(10):  # sustained DRIFT for up to 10 * 0.01s = 0.1s > approach_break_confirm_s
        cmd = nav.step(make_reading(FAR_RIGHT), 0.01)
    # Progress really was reset this time: the crossbar needs the full 1.0cm again, not 0.
    cmd = nav.step(make_reading(CROSSBAR), 0.05)
    assert "0.28/1.00cm" in cmd.reason  # prior hard-correction command advances more slowly


def test_roundabout_entry_shoulder_1001_is_part_of_the_sequence_not_noise():
    """1001 is Kind.NOISE under carbot.ir_geometry, but as the roundabout entry's own 2nd
    approach step it must advance the sequence, not get held as generic noise."""
    junction = RouteJunction(
        "roundabout entry", JunctionAction.TURN_RIGHT, 0.0,
        approach=(
            SequenceStep((1, 1, 1, 1), min_cm=1.0),
            SequenceStep((1, 0, 0, 1), min_cm=0.1),
            SequenceStep((0, 0, 0, 0)),
        ),
        creep_cm=8.0, turn_deg=42.5,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # step 1
    cmd = nav.step(make_reading(ROUNDABOUT_ENTRY_SHOULDER), 0.2)  # step 2, 2cm >= 0.1cm
    assert "step 3/3" in cmd.reason
    cmd = nav.step(make_reading(GAP), 0.1)  # step 3 -> arrival
    assert cmd.state is IRNavState.JUNCTION_CREEP


def test_arc_trigger_cm_opens_signal_matching_but_does_not_accept_by_itself():
    """Arc distance is only supporting evidence; it must never perform a junction action on an
    unrelated reading.  The 2026-08-21 log previously accepted the roundabout on P0001."""
    junction = RouteJunction(
        "roundabout entry", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.0),),
        creep_cm=8.0, turn_deg=42.5,
        arc_trigger_cm=20.0,
    )
    nav = _single_junction_nav(junction)
    nav._phase_mode = "arc"
    nav._arc_cm = 20.0

    cmd = nav.step(make_reading(FAR_RIGHT), 0.1)
    assert cmd.state is IRNavState.FOLLOW
    assert nav.junctions_seen == 0

    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert cmd.state is IRNavState.FOLLOW  # prior hard correction earns only partial distance
    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert cmd.state is IRNavState.JUNCTION_CREEP


def test_arc_trigger_can_open_matching_when_transition_count_is_unmet():
    junction = RouteJunction(
        "roundabout entry", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=1.0),),
        creep_cm=8.0, turn_deg=42.5,
        arc_trigger_cm=20.0,
        min_phase_transitions=6,
    )
    nav = _single_junction_nav(junction)
    nav._phase_mode = "arc"
    nav._arc_cm = 20.0
    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert cmd.state is IRNavState.JUNCTION_CREEP
    assert nav._phase_transitions < 6


def test_roundabout_exit_four_step_sweep_including_noise_classified_reading():
    """0101 is Kind.NOISE, 0100/0110 are ordinary DRIFT/ON_LINE -- none look like a junction
    in isolation, only the order matters."""
    junction = RouteJunction(
        "roundabout exit", JunctionAction.TURN_RIGHT, 0.0,
        approach=(
            SequenceStep((0, 1, 1, 1)),
            SequenceStep((0, 1, 0, 1)),
            SequenceStep((0, 1, 0, 0)),
            SequenceStep((0, 1, 1, 0)),
        ),
        creep_cm=6.5, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(RIGHT_BRANCH_0111), 0.1)
    nav.step(make_reading(ROUNDABOUT_EXIT_NOISE), 0.1)
    nav.step(make_reading(DRIFT_LEFT), 0.1)
    cmd = nav.step(make_reading(CENTRED), 0.1)  # 0110 -> arrival
    assert cmd.state is IRNavState.JUNCTION_CREEP
    assert nav.last_junction == "roundabout exit"


def test_cross_action_needs_no_creep_or_turn():
    """(g) Reaching the last approach step IS arrival -- no JUNCTION_CREEP/JUNCTION_TURN."""
    junction = RouteJunction(
        "T junction", JunctionAction.CROSS, 0.0,
        approach=(SequenceStep((0, 1, 1, 1), min_cm=1.0), SequenceStep((0, 1, 1, 0))),
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(RIGHT_BRANCH_0111), 0.2)  # satisfies 1cm
    cmd = nav.step(make_reading(CENTRED), 0.1)  # 0110 -> CROSS fires immediately
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right
    assert "crossing T junction straight through" in cmd.reason


def test_stop_action_halts_the_wheels():
    junction = RouteJunction(
        "final T junction", JunctionAction.STOP, 0.0,
        approach=(SequenceStep((0, 1, 1, 1), min_cm=1.0), SequenceStep((0, 1, 1, 0))),
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(RIGHT_BRANCH_0111), 0.2)
    cmd = nav.step(make_reading(CENTRED), 0.1)
    assert cmd.state is IRNavState.STOPPED
    assert cmd.left == cmd.right == 0


# ------------------------------------------------------------- creep + closed-loop turn


def _turning_junction(creep_cm=4.0, turn_deg=90.0) -> RouteJunction:
    return RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=0.0),),
        creep_cm=creep_cm, turn_deg=turn_deg,
    )


def test_junction_commits_creep_then_closed_loop_turn():
    nav = _single_junction_nav(_turning_junction(creep_cm=4.0))  # 4cm @ 10cm/s = 0.4s
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # approach completes on the very first frame
    assert cmd.state is IRNavState.JUNCTION_CREEP
    assert cmd.left == cmd.right == 150
    cmd = nav.step(make_reading(CROSSBAR), 0.5)  # 0.5s >= 0.4 -> pivot
    assert cmd.state is IRNavState.JUNCTION_TURN
    assert cmd.left > 0 and cmd.right < 0  # default right turn
    assert "watching for 0110" in cmd.reason
    cmd = nav.step(make_reading(CENTRED), 3.0)  # 0110 reached -> turn ends
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right == 150
    assert "line reacquired (0110)" in cmd.reason


def test_turn_ignores_readings_other_than_0110():
    """Mid-turn, the crossbar itself and other transitional readings must not end the turn --
    only TURN_COMPLETE_READING (0110) does."""
    nav = _single_junction_nav(_turning_junction(creep_cm=1.0))
    nav.step(make_reading(CROSSBAR), 0.1)  # arrival
    nav.step(make_reading(CROSSBAR), 0.2)  # creep done -> JUNCTION_TURN
    for reading in (CROSSBAR, RIGHT_BRANCH_0111, ROUNDABOUT_EXIT_NOISE, DRIFT_LEFT, FAR_LEFT):
        cmd = nav.step(make_reading(reading), 0.1)
        assert cmd.state is IRNavState.JUNCTION_TURN
    cmd = nav.step(make_reading(CENTRED), 0.1)  # only 0110 ends it
    assert cmd.state is IRNavState.FOLLOW


def test_turn_requires_the_minimum_spin_dead_time_before_trusting_0110():
    """A 0110 in the very first instant (e.g. residual alignment right as the pivot starts)
    must not be trusted -- spin_dead_time_s is the same "motor hasn't really moved yet"
    floor the spin calibration itself uses."""
    nav = _single_junction_nav(
        _turning_junction(creep_cm=1.0), spin_dead_time_s=0.5, spin_rate_deg_per_s=42.0
    )
    nav.step(make_reading(CROSSBAR), 0.1)
    nav.step(make_reading(CROSSBAR), 0.2)  # -> JUNCTION_TURN
    cmd = nav.step(make_reading(CENTRED), 0.1)  # 0110 immediately, but < 0.5s dead time
    assert cmd.state is IRNavState.JUNCTION_TURN
    cmd = nav.step(make_reading(CENTRED), 0.5)  # now past dead time, still reading 0110
    assert cmd.state is IRNavState.FOLLOW


def test_turn_requires_0110_to_be_sustained_not_a_single_coincidental_frame():
    """2026-08-20, ninth pass, real-track: a single 0110 frame mid-spin ended a real turn at
    ~28 degrees of a nominal 90 -- some other feature the sensor swept past briefly happened
    to read the same bits. At a realistic ~100Hz dt, one frame is nowhere near
    turn_confirm_s (0.08s default); it must keep turning, and a reading that changes again
    resets the confirmation clock rather than carrying it over."""
    nav = _single_junction_nav(_turning_junction(creep_cm=1.0))
    nav.step(make_reading(CROSSBAR), 0.1)
    nav.step(make_reading(CROSSBAR), 0.2)  # -> JUNCTION_TURN
    nav.step(make_reading(FAR_LEFT), 0.5)  # past spin_dead_time_s (0.41 default)
    cmd = nav.step(make_reading(CENTRED), 0.01)  # one coincidental 0110 frame
    assert cmd.state is IRNavState.JUNCTION_TURN
    cmd = nav.step(make_reading(FAR_LEFT), 0.01)  # back to spinning -- resets the confirm clock
    assert cmd.state is IRNavState.JUNCTION_TURN
    cmd = nav.step(make_reading(CENTRED), 0.01)
    assert cmd.state is IRNavState.JUNCTION_TURN  # only 0.01s into confirming again
    for _ in range(20):  # sustained 0110 for up to 20 * 0.01s = 0.2s > turn_confirm_s
        cmd = nav.step(make_reading(CENTRED), 0.01)
        if cmd.state is IRNavState.FOLLOW:
            break
    assert cmd.state is IRNavState.FOLLOW
    assert "line reacquired (0110)" in cmd.reason


def test_turn_times_out_if_0110_never_returns():
    """A chassis fault or misalignment that never reproduces 0110 must not spin forever."""
    nav = _single_junction_nav(
        _turning_junction(creep_cm=1.0, turn_deg=90.0),
        spin_rate_deg_per_s=42.0,
        spin_dead_time_s=0.41,
        turn_timeout_scale=2.0,
    )
    nav.step(make_reading(CROSSBAR), 0.1)
    nav.step(make_reading(CROSSBAR), 0.2)  # -> JUNCTION_TURN
    timeout_s = nav.policy.turn_timeout_s(90.0)
    cmd = nav.step(make_reading(FAR_LEFT), timeout_s + 1.0)  # never reads 0110
    assert cmd.state is IRNavState.FAILED
    assert cmd.left == cmd.right == 0
    assert "failed" in cmd.reason
    assert "0110 never seen" in cmd.reason
    latched = nav.step(make_reading(CENTRED), 0.1)
    assert latched.state is IRNavState.FAILED
    assert latched.left == latched.right == 0


def test_creep_duration_is_the_junctions_own_creep_cm_over_speed():
    nav = _single_junction_nav(_turning_junction(creep_cm=9.5))
    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert cmd.state is IRNavState.JUNCTION_CREEP
    cmd = nav.step(make_reading(CROSSBAR), 0.9)  # 0.9s < 0.95s -> still creeping
    assert cmd.state is IRNavState.JUNCTION_CREEP
    assert "creeping 9.5cm" in cmd.reason
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # 1.0s >= 0.95s -> pivot
    assert cmd.state is IRNavState.JUNCTION_TURN


def test_turn_timeout_scales_with_expected_angle():
    policy = IRNavPolicy(spin_rate_deg_per_s=42.0, spin_dead_time_s=0.41, turn_timeout_scale=2.0)
    assert policy.turn_timeout_s(90.0) == pytest.approx(2.0 * (0.41 + 90.0 / 42.0))
    assert policy.turn_timeout_s(42.5) == pytest.approx(2.0 * (0.41 + 42.5 / 42.0))


def test_post_turn_0000_starts_a_search_not_a_stale_pre_turn_blind_band():
    """The turn's real angle is not otherwise verified, so a 0000 right after landing must
    not be resolved with the line position from before the turn: that geometry belongs to
    the old heading and says nothing about where the line is on the new one."""
    nav = _single_junction_nav(_turning_junction(creep_cm=1.0))
    # DRIFT_RIGHT is in ir_geometry.BLIND_AFTER_RIGHT -- if this survived the turn, the
    # post-turn 0000 below would be misread as "blind band, keep going" instead of "lost".
    nav.step(make_reading(DRIFT_RIGHT), 0.01)
    nav.step(make_reading(CROSSBAR), 0.1)
    nav.step(make_reading(CROSSBAR), 0.2)  # -> JUNCTION_TURN
    cmd = nav.step(make_reading(CENTRED), 5.0)  # 0110 -> turn done
    assert cmd.state is IRNavState.FOLLOW
    cmd = nav.step(make_reading(GAP), dt=0.1)  # 0000 immediately after landing
    assert cmd.state is IRNavState.SEARCH
    assert "pivot left" in cmd.reason


def test_line_lost_after_junction_turn_enters_search():
    """The reported failure: after the T-junction the car faces the ~2.4cm
    gap between the sensor pairs and reads nothing — it must search, not stop."""
    nav = _single_junction_nav(_turning_junction(creep_cm=1.0))
    nav.step(make_reading(CROSSBAR), 0.1)
    nav.step(make_reading(CROSSBAR), 0.2)  # -> JUNCTION_TURN
    cmd = nav.step(make_reading(CENTRED), 5.0)  # 0110 -> turn done
    assert cmd.state is IRNavState.FOLLOW
    cmd = nav.step(make_reading(GAP), dt=0.1)  # gap under the bar
    assert cmd.state is IRNavState.SEARCH
    assert cmd.left < 0 < cmd.right


def test_a_pivot_does_not_count_toward_the_next_gate():
    """A spin covers no ground, so feeding it to the odometer would open the gate early."""
    nav = _single_junction_nav(_turning_junction(creep_cm=1.0))
    nav.step(make_reading(CROSSBAR), 0.1)
    nav.step(make_reading(CROSSBAR), 0.2)  # -> JUNCTION_TURN
    before = nav.junctions.cm_since_previous
    while nav.state is IRNavState.JUNCTION_TURN:
        nav.step(make_reading(FAR_LEFT), 0.01)
    assert nav.junctions.cm_since_previous == pytest.approx(before)


# ------------------------------------------------------------- search


def test_search_pivots_in_place_at_min_angle_then_steps_up():
    """Line recovery is a strictly in-place pendulum: pivot left by `search_sweep_min_deg`
    (5 deg), pivot back through centre to the same angle right, then -- if still no line --
    step the angle up to the next level (5 -> 10 deg). No forward creep on any step."""
    nav = default_nav(
        search_sweep_min_deg=5.0,
        search_sweep_step_deg=5.0,
        search_sweep_deg=10.0,
        off_track_dwell_s=100.0,  # disable off-track/reverse divergence; this test is the search only
    )
    gap = make_reading(GAP)

    # Enter search -> first in-place left pivot at the min angle (5 deg).
    cmd = nav.step(gap, 0.05)
    assert cmd.state is IRNavState.SEARCH
    assert "pivot left 5deg" in cmd.reason
    assert (cmd.left, cmd.right) == (-150, 150)  # in place, no forward

    # Overwhelm the left pivot duration (0.41 + 5/42.0 = 0.529s) -> pivot right (2x angle).
    cmd = nav.step(gap, 1.0)
    assert "pivot right 10deg" in cmd.reason
    assert (cmd.left, cmd.right) == (150, -150)  # in place

    # Overwhelm the right pivot (0.41 + 10/42.0 = 0.648s) -> angle steps up to 10 deg,
    # search restarts its pendulum at the bigger angle.
    cmd = nav.step(gap, 1.0)
    assert "pivot left 10deg" in cmd.reason
    assert (cmd.left, cmd.right) == (-150, 150)


def test_search_at_ceiling_backtracks_then_turns_around():
    """Once the pendulum reaches `search_sweep_deg` and a full pair still finds no line, the
    car backs up 5cm and re-probes from the min angle; after `search_retreat_count` retreats
    it pivots 180 deg instead. It never stops on its own (give-up aside)."""
    # min == max == 15 so the *first* pair is already at the ceiling -> immediate recovery.
    nav = default_nav(
        search_sweep_min_deg=15.0,
        search_sweep_step_deg=5.0,
        search_sweep_deg=15.0,
        search_retreat_cm=5.0,
        search_retreat_count=1,
        off_track_dwell_s=100.0,
    )
    gap = make_reading(GAP)
    nav.step(gap, 0.05)  # in search, pivot left 15 deg
    nav.step(gap, 1.0)  # left 15 done -> right (30 deg)

    # Right 30 done -> still no line -> BACKTRACK (reverse 5cm).
    cmd = nav.step(gap, 2.0)
    assert "backtrack" in cmd.reason
    assert (cmd.left, cmd.right) == (-150, -150)  # reverse on the spot
    assert nav._search_retreats == 0  # not yet counted until the backtrack finishes

    # Backtrack (5cm @ 10cm/s = 0.5s) finishes -> re-probe from the min angle.
    cmd = nav.step(gap, 1.0)
    assert "pivot left 15deg" in cmd.reason
    assert nav._search_retreats == 1

    # Second ceiling pair, right 30 done -> retreats used up -> TURN_AROUND (180 deg).
    nav.step(gap, 1.0)  # left 15
    cmd = nav.step(gap, 2.0)  # right 30 done -> turn around
    assert "turn around" in cmd.reason
    assert (cmd.left, cmd.right) == (150, -150)  # in-place 180

    # Turn around (0.41 + 180/42 = 4.696s) not done yet -> still turning.
    cmd = nav.step(gap, 1.0)
    assert "turn around" in cmd.reason

    # Turn around finishes -> re-probe from the min angle again.
    cmd = nav.step(gap, 5.0)
    assert "pivot left 15deg" in cmd.reason


def test_search_reacquires_line_and_resumes_follow():
    nav = default_nav()
    nav.step(make_reading(GAP), 0.1)  # into search
    cmd = nav.step(make_reading(CENTRED), 0.1)  # line back under bar
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right == 150  # centred: straight, no correction


def test_search_reacquired_junction_approach_is_still_tracked():
    """Reacquiring the line as the pending junction's first approach reading must be treated
    as the start of that sequence, not just plain follow — the search delegates back to
    follow, which runs _approach_step first."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=5.0), SequenceStep((0, 0, 0, 0))),
        creep_cm=1.0, turn_deg=90.0,
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(GAP), 0.1)  # into search
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # reacquired as the approach's first step
    assert cmd.state is IRNavState.FOLLOW
    assert "approaching x, step 1/2" in cmd.reason


def test_search_gives_up_and_stops():
    nav = default_nav(search_give_up_s=1.0)
    gap = make_reading(GAP)
    nav.step(gap, 0.5)  # enters search (transition itself takes no search time)
    nav.step(gap, 0.5)  # finishes left sweep -> sweep right
    cmd = nav.step(gap, 0.5)  # 1.0s of searching >= give-up -> stop
    assert cmd.left == cmd.right == 0
    assert cmd.state is IRNavState.FAILED
    assert "search failed" in cmd.reason


def test_search_zero_give_up_never_stops():
    # off_track_dwell_s disabled (huge) -- this test runs GAP for 25s straight to check the
    # give-up=0 case specifically; see test_off_track_* for the 2026-08-20 feature.
    nav = default_nav(search_give_up_s=0.0, off_track_dwell_s=100.0)
    gap = make_reading(GAP)
    for _ in range(50):
        cmd = nav.step(gap, 0.5)
        assert cmd.state is IRNavState.SEARCH
    assert not (cmd.left == cmd.right == 0)


# --------------------------------------------------- 2026-08-20 phase tracker (straight/arc)
#
# Distinguishes "on a straight phase" (sustained 0110) from "on an arc" (a repeating
# non-0110 correction) -- see tasks/ir-sensor-tracking/phase-tracking-and-junction-detection-
# plan.md. Gates RouteJunction.min_phase_transitions/.min_arc_cm (see below).


def test_phase_tracker_accumulates_straight_cm_on_sustained_0110():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.0)  # issue the command active over the next interval
    nav.step(make_reading(CENTRED), 0.5)  # 5cm
    nav.step(make_reading(CENTRED), 0.3)  # +3cm
    assert nav._phase_mode == "straight"
    assert nav._straight_cm == pytest.approx(8.0)
    assert nav._phase_transitions == 0


def test_distance_credit_scales_with_the_actual_corner_command() -> None:
    window = CornerWindow("slow", "roundabout entry", 0.0, 100.0, speed_scale=0.6)
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        corner_windows=(window,),
        speed=150,
        forward_speed_cm_per_s=10.0,
        forward_speed_reference_pwm=150,
    )
    first = nav.step(make_reading(CENTRED), 0.0)
    assert first.left == first.right == 90
    nav.step(make_reading(CENTRED), 1.0)
    assert nav.junctions.cm_since_previous == pytest.approx(6.0)
    assert nav._straight_cm == pytest.approx(6.0)


def test_phase_tracker_ignores_a_correction_blip_under_the_dwell():
    """A non-0110 reading shorter than phase_transition_dwell_s (0.8s default) is noise, not
    a real arc-correction event -- must not flip the mode, and _straight_cm keeps
    accumulating straight through it."""
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.0)
    nav.step(make_reading(CENTRED), 0.5)
    nav.step(make_reading(DRIFT_LEFT), 0.3)  # 0100, only 0.3s < 0.8s dwell
    nav.step(make_reading(CENTRED), 0.2)
    assert nav._phase_mode == "straight"
    assert nav._phase_transitions == 0
    assert nav._straight_cm == pytest.approx(10.0)  # (0.5+0.3+0.2)s * 10cm/s, all counted


def test_phase_tracker_confirms_arc_after_a_sustained_correction():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.0)
    nav.step(make_reading(CENTRED), 0.5)  # _straight_cm = 5.0
    nav.step(make_reading(DRIFT_LEFT), 0.9)  # >= 0.8s dwell -> confirmed arc
    assert nav._phase_mode == "arc"
    assert nav._phase_transitions == 1
    assert nav._straight_cm == pytest.approx(14.0)  # includes the flip-confirming frame
    assert nav._arc_cm == 0.0  # just reset on entering arc mode


def test_sustained_0000_never_counts_as_arc_evidence() -> None:
    """A line loss must not become the evidence that auto-enables a later route junction."""
    nav = default_nav(route=TASK1_LOOP_ONLY, off_track_dwell_s=100.0)
    nav.step(make_reading(CENTRED), 0.0)
    for _ in range(20):
        nav.step(make_reading(GAP), 0.1)
    assert nav._phase_mode == "straight"
    assert nav._phase_transitions == 0
    assert nav._arc_cm == 0.0


def test_phase_tracker_confirms_return_to_straight():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.5)
    nav.step(make_reading(DRIFT_LEFT), 0.9)  # -> arc, transitions=1
    nav.step(make_reading(CENTRED), 0.9)  # sustained 0110 >= 0.8s -> back to straight
    assert nav._phase_mode == "straight"
    assert nav._phase_transitions == 2
    assert nav._straight_cm == 0.0  # just reset on entering straight mode


def test_phase_tracker_resets_when_a_junction_is_accepted():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.5)
    nav.step(make_reading(DRIFT_LEFT), 0.9)  # -> arc, transitions=1
    _reach_next_junction(nav)  # start stem T accepted
    assert nav._phase_mode == "straight"
    assert nav._phase_transitions == 0
    assert nav._straight_cm == 0.0
    assert nav._arc_cm == 0.0


# ------------------------------------------- 2026-08-20 junction preconditions (phase tracker)


def test_roundabout_entry_approach_is_gated_by_phase_transitions():
    """Distance gate alone isn't enough for the roundabout entry -- see
    carbot.ir_route.RouteJunction.min_phase_transitions."""
    from carbot.ir_route import TASK1_LOOP_ONLY

    nav = default_nav(route=TASK1_LOOP_ONLY)  # pending: roundabout entry, min_phase_transitions=6
    assert nav.junctions.pending.min_phase_transitions == 6
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # precondition unmet: not even attempted
    assert "approaching roundabout entry" not in cmd.reason
    nav._phase_transitions = 6  # precondition satisfied directly (see test_phase_tracker_*)
    cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert "approaching roundabout entry" in cmd.reason


def test_roundabout_exit_approach_is_gated_by_arc_cm():
    """Phase 9 (the roundabout traversal) is one continuous curve -- gated on accumulated
    _arc_cm, not a mode-flip count. See carbot.ir_route.RouteJunction.min_arc_cm."""
    plan = RoutePlan(prologue=(), loop=(TASK1_ROUTE.loop[1],))  # pending: roundabout exit
    nav = default_nav(route=plan)
    assert nav.junctions.pending.min_arc_cm == 68.0
    cmd = nav.step(make_reading(RIGHT_BRANCH_0111), 0.1)  # precondition unmet
    assert "approaching roundabout exit" not in cmd.reason
    nav._arc_cm = 68.0  # precondition satisfied directly
    cmd = nav.step(make_reading(RIGHT_BRANCH_0111), 0.1)
    assert "approaching roundabout exit" in cmd.reason


def test_start_t_and_t_junction_have_no_phase_precondition():
    assert TASK1_ROUTE.prologue[0].min_phase_transitions == 0
    assert TASK1_ROUTE.prologue[0].min_arc_cm == 0.0
    assert TASK1_ROUTE.loop[2].min_phase_transitions == 0
    assert TASK1_ROUTE.loop[2].min_arc_cm == 0.0


# --------------------------------------------------- 2026-08-20 off-track recovery (reverse)


def test_off_track_timer_ignores_readings_under_the_dwell():
    nav = default_nav()
    for _ in range(19):  # 1.9s of continuous 0000, just under the 2.0s default dwell
        nav.step(make_reading(GAP), 0.1)
    assert nav.state is IRNavState.SEARCH  # ordinary lost-line search, not reverse yet


def test_sustained_0000_triggers_reverse_after_the_dwell():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.5)  # give it a command history to replay
    for _ in range(21):  # 2.1s of continuous 0000
        cmd = nav.step(make_reading(GAP), 0.1)
    assert nav.state is IRNavState.REVERSE
    assert "reverse-replay" in cmd.reason


def test_sustained_1111_off_paper_also_triggers_reverse():
    """A junction's own 1111 hold is well under 1s -- 2s sustained can only be carpet beyond
    the paper's edge, never a real junction. See docs/hardware/ir-tracing-sensor.md.

    2026-08-20, sixth pass: the start stem's approach (default_nav's initial pending) is now a
    single 1.5cm step, so sustained 1111 completes it almost immediately instead of ever
    idling -- start on the loop instead, where the pending roundabout entry's approach still
    has two more steps (1001, then 0000) that pure 1111 never satisfies."""
    nav = default_nav(route=TASK1_LOOP_ONLY)
    for _ in range(21):
        nav.step(make_reading(CROSSBAR), 0.1)
    assert nav.state is IRNavState.REVERSE


def test_switching_between_0000_and_1111_restarts_the_off_track_clock():
    nav = default_nav()
    for _ in range(15):
        nav.step(make_reading(GAP), 0.1)
    cmd = nav.step(make_reading(CROSSBAR), 0.1)  # switched reading -- clock restarts
    assert cmd.state is not IRNavState.REVERSE
    for _ in range(15):  # only 1.6s of continuous 1111 since the switch -- not enough yet
        cmd = nav.step(make_reading(CROSSBAR), 0.1)
    assert cmd.state is not IRNavState.REVERSE


def test_reverse_replay_sign_flips_the_history_newest_first():
    """Directly seeds the history and enters REVERSE, isolating the queue-ordering mechanism
    from what real intervening SEARCH/blind-band commands would organically produce during a
    genuine 2s off-track period (those get recorded too -- this just checks the replay itself
    plays newest-first, sign-flipped)."""
    nav = default_nav()
    nav._command_history.append((150, 110, 0.2))  # older
    nav._command_history.append((110, 150, 0.2))  # newest
    nav._command_history_s = 0.4
    nav._enter_reverse()
    cmd = nav._reverse_step(make_reading(GAP), 0.05)
    assert nav.state is IRNavState.REVERSE
    assert (cmd.left, cmd.right) == (-110, -150)


def test_search_pivots_are_never_added_to_reverse_history() -> None:
    nav = default_nav(route=TASK1_LOOP_ONLY, off_track_dwell_s=100.0)
    nav.step(make_reading(CENTRED), 0.1)
    nav.step(make_reading(FAR_RIGHT), 0.1)  # outer sensor makes the following 0000 a real loss
    before = list(nav._command_history)
    for _ in range(10):
        cmd = nav.step(make_reading(GAP), 0.1)
        assert cmd.state is IRNavState.SEARCH
    assert list(nav._command_history) == before


def test_reverse_replay_stops_early_on_reacquiring_a_real_signal():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.5)
    for _ in range(21):
        nav.step(make_reading(GAP), 0.1)
    assert nav.state is IRNavState.REVERSE
    cmd = nav.step(make_reading(CENTRED), 0.01)  # reacquired -- stop replaying immediately
    assert cmd.state is IRNavState.FOLLOW
    assert cmd.left == cmd.right == 150


def test_bounded_left_curve_reverse_ignores_false_right_reacquisition() -> None:
    nav = default_nav(
        route=TASK1_LOOP_ONLY,
        junction_detection_enabled=False,
        curve_pattern_steering_enabled=True,
        steering_direction_limit=-1,
    )
    nav._command_history.extend([(110, 150, 0.2)] * 10)
    nav._command_history_s = 2.0
    nav._enter_reverse()

    for raw in (FAR_RIGHT, RIGHT_BRANCH_0111, ROUNDABOUT_EXIT_NOISE):
        cmd = nav.step(make_reading(raw), 0.01)
        assert cmd.state is IRNavState.REVERSE
        assert (cmd.left, cmd.right) == (-110, -150)

    reacquired = nav.step(make_reading(CENTRED), 0.01)
    assert reacquired.state is IRNavState.FOLLOW
    assert (reacquired.left, reacquired.right) == (110, 150)
    assert "maintaining left curvature" in reacquired.reason


def test_reverse_replay_falls_through_to_search_once_exhausted():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.1)  # a thin history -- one short command
    for _ in range(21):
        nav.step(make_reading(GAP), 0.1)
    assert nav.state is IRNavState.REVERSE
    cmd = None
    for _ in range(50):  # replay the (short) queue out with continued 0000
        cmd = nav.step(make_reading(GAP), 0.1)
        if cmd.state is IRNavState.SEARCH:
            break
    assert cmd.state is IRNavState.SEARCH


def test_reverse_replay_is_excluded_from_distance_credit():
    nav = default_nav()
    nav.step(make_reading(CENTRED), 0.5)
    for _ in range(21):
        nav.step(make_reading(GAP), 0.1)
    assert nav.state is IRNavState.REVERSE
    before = nav.junctions.cm_since_previous
    nav.step(make_reading(GAP), 0.1)  # still replaying
    assert nav.junctions.cm_since_previous == pytest.approx(before)


def test_off_track_does_not_trigger_mid_creep_or_turn():
    """JUNCTION_CREEP/JUNCTION_TURN are short, deliberately sensor-blind manoeuvres with their
    own timeout -- the off-track check is skipped for them (see IRLineNav.step)."""
    junction = RouteJunction(
        "x", JunctionAction.TURN_RIGHT, 0.0,
        approach=(SequenceStep((1, 1, 1, 1), min_cm=0.0),),
        creep_cm=50.0, turn_deg=90.0,  # long creep, comfortably over the 2s off-track dwell
    )
    nav = _single_junction_nav(junction)
    nav.step(make_reading(CROSSBAR), 0.1)  # arrival -> JUNCTION_CREEP
    assert nav.state is IRNavState.JUNCTION_CREEP
    for _ in range(25):  # 2.5s of blind creep, sensor irrelevant but still reads GAP
        cmd = nav.step(make_reading(GAP), 0.1)
    assert cmd.state is IRNavState.JUNCTION_CREEP  # not diverted into REVERSE


# ------------------------------------------------------------- policy


@pytest.mark.parametrize(
    "kwargs",
    [
        {"speed": -1},
        {"speed": 1001},
        {"turn_gain": 0.0},
        {"deadband": 1.0},
        {"turn_direction": 0},
        {"spin_rate_deg_per_s": 0.0},
        {"spin_dead_time_s": -1.0},
        {"turn_timeout_scale": 0.0},
        {"forward_speed_cm_per_s": 0.0},
        {"search_sweep_deg": -1.0},
        {"search_creep_step_s": -0.1},
        {"search_creep_speed_ratio": 0.0},
        {"search_creep_speed_ratio": 1.5},
        {"search_creep_steps_per_cycle": 0},
        {"search_give_up_s": -1.0},
        {"steering_direction_limit": 2},
    ],
)
def test_invalid_policy_rejected(kwargs):
    with pytest.raises(ValueError):
        IRNavPolicy(**kwargs)


def test_sweep_duration_uses_calibrated_spin_model():
    policy = IRNavPolicy()
    # 0.41s dead time + 10 deg / 42.0 deg/s
    assert policy.sweep_duration(10.0) == pytest.approx(0.41 + 10.0 / 42.0)
    assert policy.sweep_duration(20.0) == pytest.approx(0.41 + 20.0 / 42.0)


# ------------------------------------------------- route-driven junctions (integration)
#
# End-to-end with the real TASK1_ROUTE data, not a synthetic minimal junction.

RIGHT_BRANCH = RIGHT_BRANCH_0111  # physical 0111 — the roundabout exit and the T junction


def _drive(nav: IRLineNav, channels, seconds: float, dt: float = 0.01):
    """Hold one reading for a while, returning every command produced."""
    return [nav.step(make_reading(channels), dt) for _ in range(int(seconds / dt))]


def _settle(nav: IRLineNav):
    """Run the creep and pivot out to completion (or a STOP action's halt)."""
    for _ in range(4000):
        if nav.state in (IRNavState.FOLLOW, IRNavState.STOPPED):
            return
        nav.step(make_reading(CENTRED), 0.01)
    raise AssertionError("junction never finished")


def _reach_next_junction(nav: IRLineNav) -> None:
    """Drive far enough to clear the next distance gate, then walk its real approach
    sequence to completion (always feeding whatever step `_approach_index` is currently
    tracking, so persistence requirements are satisfied step by step in order), then settle
    out any creep/turn.

    Directly satisfies the pending junction's phase-tracker precondition (see
    carbot.ir_route.RouteJunction.min_phase_transitions/.min_arc_cm) rather than actually
    driving the straight/arc pattern that would earn it -- this helper is testing junction
    arrival, not the phase tracker itself (see test_phase_tracker_* for that).
    """
    gate = nav.junctions.pending.min_cm_since_previous
    seconds = gate / nav.policy.forward_speed_cm_per_s + 1.0
    for _ in range(int(seconds / 0.01) + 10):
        if nav.state is IRNavState.STOPPED:
            return
        nav.step(make_reading(CENTRED), dt=0.01)
    pending = nav.junctions.pending
    nav._phase_transitions = pending.min_phase_transitions
    nav._arc_cm = pending.min_arc_cm
    approach = pending.approach
    seen_before = nav.junctions_seen
    for _ in range(4000):
        if nav.junctions_seen > seen_before or nav.state is IRNavState.STOPPED:
            break
        raw = _raw_for(approach[nav._approach_index].bits)
        nav.step(make_reading(raw), dt=0.01)
    else:
        raise AssertionError("approach sequence never completed")
    _settle(nav)


def _raw_for(physical: tuple[int, int, int, int]) -> tuple[int, int, int, int]:
    """Invert to_physical's PHYSICAL_ORDER swap (0<->1) to get a raw Out-order tuple that
    decodes to `physical`."""
    p1, p2, p3, p4 = physical
    return (p2, p1, p3, p4)


def test_first_junction_out_of_the_start_box_turns_right():
    nav = default_nav()
    _reach_next_junction(nav)
    assert nav.last_junction == "start stem T junction"
    assert nav.junctions_seen == 1


def test_the_returning_t_junction_is_crossed_not_turned():
    nav = default_nav()
    for _ in range(3):  # prologue T, roundabout entry, roundabout exit
        _reach_next_junction(nav)
    assert nav.last_junction == "roundabout exit"
    assert nav.state is IRNavState.FOLLOW
    _reach_next_junction(nav)  # the lap-crossing T: CROSS, no pivot
    assert nav.last_junction == "T junction"
    assert nav.state is IRNavState.FOLLOW


def test_a_junction_read_again_immediately_is_rejected():
    """After the start-stem T, immediately walking the roundabout entry's full approach
    sequence (without covering its 60cm gate) must be rejected, not accepted."""
    nav = default_nav()
    _reach_next_junction(nav)  # start stem T, accepted
    assert nav.last_junction == "start stem T junction"
    pending = nav.junctions.pending  # roundabout entry
    nav._phase_transitions = pending.min_phase_transitions  # satisfy the precondition directly
    nav._arc_cm = pending.min_arc_cm
    approach = pending.approach  # its sequence, too soon for the *distance* gate specifically
    cmd = None
    for _ in range(2000):
        raw = _raw_for(approach[nav._approach_index].bits)
        cmd = nav.step(make_reading(raw), dt=0.01)
        if nav.junctions_rejected > 0:
            break
    assert nav.junctions_rejected > 0
    assert nav.junctions_seen == 1
    assert cmd.left == cmd.right
    assert "short of the" in cmd.reason


def test_a_stop_junction_halts_the_wheels():
    from carbot.ir_route import task1_route_for_laps

    nav = default_nav(route=task1_route_for_laps(1))
    for _ in range(4):
        _reach_next_junction(nav)
    assert nav.state is IRNavState.STOPPED
    assert nav.last_junction == "final T junction"


def test_the_stop_is_latched_against_further_readings():
    from carbot.ir_route import task1_route_for_laps

    nav = default_nav(route=task1_route_for_laps(1))
    for _ in range(4):
        _reach_next_junction(nav)
    assert nav.state is IRNavState.STOPPED
    for reading in (CENTRED, CROSSBAR, DRIFT_RIGHT, GAP):
        cmd = nav.step(make_reading(reading), dt=0.01)
        assert cmd.left == 0 and cmd.right == 0
        assert cmd.state is IRNavState.STOPPED


# ------------------------------------------------- gate rejection keeps steering

SKEW_LEFT = (1, 1, 0, 0)  # physical 1100 — left pair, a curve read at a shallow angle
SKEW_RIGHT = (0, 0, 1, 1)  # physical 0011 — right pair


def _gated_directional_nav() -> IRLineNav:
    """A nav whose only junction has a 60cm gate and an approach that completes in a single,
    directional reading -- isolates the gate-rejection path (which must keep steering, see
    the 2026-08-19 regression below) from the mid-sequence-interrupt path above (which must
    not, since none of the real Task-1 junctions' final steps carry directional offset:
    0000/0110 both resolve to direction 0)."""
    junction = RouteJunction("x", JunctionAction.TURN_RIGHT, 60.0, approach=(SequenceStep(SKEW_LEFT),))
    return _single_junction_nav(junction)


def test_a_gated_out_junction_still_steers_toward_the_line():
    """Regression: holding straight here drove the car off the paper on 2026-08-19.

    The gate rejecting a reading means "not the junction the route wants", not "ignore
    where the line is" — the approach sequence completing early still has to be steered on
    if the gate then rejects it.
    """
    nav = _gated_directional_nav()
    cmd = nav.step(make_reading(SKEW_LEFT), 0.01)  # completes the approach; gate rejects it
    assert nav.junctions_rejected > 0
    assert cmd.left < cmd.right, "1100 means the line is left; the left wheel must slow"


def test_a_gated_out_junction_steers_the_other_way_too():
    junction = RouteJunction("x", JunctionAction.TURN_RIGHT, 60.0, approach=(SequenceStep(SKEW_RIGHT),))
    nav = _single_junction_nav(junction)
    cmd = nav.step(make_reading(SKEW_RIGHT), 0.01)
    assert nav.junctions_rejected > 0
    assert cmd.right < cmd.left, "0011 means the line is right; the right wheel must slow"


def test_a_gated_out_junction_does_not_advance_the_route():
    nav = _gated_directional_nav()
    pending_before = nav.junctions.pending.name
    nav.step(make_reading(SKEW_LEFT), 0.01)
    assert nav.junctions.pending.name == pending_before
    assert nav.junctions_seen == 0
    assert nav.junctions_rejected > 0
