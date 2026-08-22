"""IR sensor line following — 4-channel direct steering without camera.

The sensor's physical layout, the meaning of all 16 readings, and the geometry
that produces them live in :mod:`carbot.ir_geometry`. This module is only the
navigation state machine on top of it.

**The physical channel order was wrong here until 2026-08-19.** This docstring
previously recorded the bar as ``Out4 Out3 Out1 Out2`` left to right, read off
the potentiometer silkscreen. A card swept across the bar tripped the channels
in the order ``Out2 Out1 Out3 Out4`` instead — the leading and trailing edges of
the card agreed independently, and the operator separately confirmed ``Out4`` is
the rightmost sensor. The old order is the exact mirror of the truth, so every
steering correction was being applied to the wrong side.

Two other numbers here were also wrong: the bar spans **64 mm**, not the ~10 mm
once recorded, and the outer gap is **2.8 cm**, not 2.4 cm. What matters for
recovery is not the gap but ``gap - line width = 0.8 cm``: the band of line
positions no channel can see.

Steering comes from :data:`carbot.ir_geometry.STATE_TABLE`, which is total over
all 16 readings and splits them three ways — readings a single 2 cm line can
produce (steer on these), readings needing a second dark feature (junctions and
badly skewed passes over a curve), and non-contiguous readings that one line
cannot produce at all (hold the previous command, never steer).

``0000`` is deliberately not "line lost". Inside the 0.8 cm blind band the car
is squarely on the line and sees nothing, so the previous reading decides:
the line can only leave the bar past an *outer* sensor, making ``0000`` after
``0010``/``0100`` a blind band and ``0000`` after ``0001``/``1000`` a real loss.

Line-recovery search (SEARCH state): on a genuine loss the car probes rather
than stopping — a strictly in-place pendulum: pivot `search_sweep_min_deg` (5 deg) left,
then back through centre to the same angle right (watching for the line throughout), never
creeping forward. If a full pair finds no line, the angle steps up by
`search_sweep_step_deg` (5 deg) toward `search_sweep_deg` (45 deg), then holds there until
the line is seen again or until `search_give_up_s`.

Junctions are sequenced by :mod:`carbot.ir_route`, not by the reading — the reading only
says *that* a junction feature is under the bar, matched against an ordered signal sequence
specific to each junction (:class:`carbot.ir_route.SequenceStep`); which junction it is, and
whether to turn or cross, comes from the route plan plus a distance gate. See the
:mod:`carbot.ir_route` module docstring for the two earlier designs (a shared boolean, then a
single-reading dwell timer) this replaced and why each one broke on real track data.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from typing import TYPE_CHECKING

from carbot.ir_geometry import (
    DETECTION_LIMIT_CM,
    IRState,
    Kind,
    classify,
    resolve_blind,
    to_physical,
    wheel_speeds,
)
from carbot.ir_route import (
    TASK1_CORNER_WINDOWS,
    TASK1_ROUTE,
    TURN_COMPLETE_READING,
    CornerWindow,
    JunctionAction,
    JunctionSequencer,
    RouteJunction,
    RoutePlan,
)
from carbot.map1_phases import estimate_forward_distance_cm

if TYPE_CHECKING:
    from carbot.ir_tracing import IRTracingSensor


@dataclass(frozen=True)
class IRLineReading:
    """One cycle of 4-channel IR sensor data, already classified."""

    channels: tuple[int, int, int, int]  # (Out1..Out4) as the driver reports them
    physical: tuple[int, int, int, int]  # (P1..P4) left to right, 1 = black
    state: IRState  # entry from carbot.ir_geometry.STATE_TABLE
    visible: bool  # any channel sees black

    @property
    def summary(self) -> str:
        bits = "".join(str(b) for b in self.physical)
        return f"{bits} {self.state.label}"

    @property
    def error_fraction(self) -> float:
        """Offset normalised to [-1, 1] against the detection limit, for logs."""
        if self.state.offset_cm is None:
            return 0.0
        return max(-1.0, min(1.0, self.state.offset_cm / DETECTION_LIMIT_CM))


def make_reading(channels: tuple[int, int, int, int]) -> IRLineReading:
    """Classify an ``Out1..Out4`` reading without touching hardware."""
    physical = to_physical(channels)
    return IRLineReading(
        channels=tuple(channels),  # type: ignore[arg-type]
        physical=physical,
        state=classify(physical, physical=True),
        visible=any(physical),
    )


def detect_ir_line(sensor: IRTracingSensor, speed: int = 200) -> IRLineReading:
    """Read the four channels and classify them. ``speed`` is unused, kept for
    call-site compatibility with the example scripts."""
    return make_reading(sensor.read())  # type: ignore[arg-type]


class IRNavState(Enum):
    """Where the car is in the scripted-route plan."""

    FOLLOW = "follow"  # proportional steering on the line (includes matching a junction's
    # approach sequence -- see IRLineNav._approach_step)
    JUNCTION_CREEP = "junction_creep"  # committed to the junction; blind creep before pivoting
    JUNCTION_TURN = "junction_turn"  # spinning right, closed-loop until 0110 or a timeout
    SEARCH = "search"  # line lost; sweep ±search_sweep_deg, then creep forward step by step
    REVERSE = "reverse"  # off-track (0000/1111 sustained); replaying recent commands backward
    STOPPED = "stopped"  # the route's planned laps are done; latched, wheels held at zero
    FAILED = "failed"  # unsafe/uncertain recovery outcome; latched, wheels held at zero


class IRSearchPhase(Enum):
    """Sub-phases of the line-recovery search.

    Line recovery is a strictly in-place pendulum: pivot left to the current
    sweep angle, then pivot back through centre to the same angle on the right.
    All probing rotates the car on the spot -- it never creeps forward while
    searching. If a complete left+right pair finds no line, the sweep angle is
    bumped up one step (see ``search_sweep_min_deg``/``search_sweep_step_deg``/
    ``search_sweep_deg``) and the pair repeats, until ``search_give_up_s``.
    """

    SWEEP_LEFT = "sweep_left"  # pivot left to the current sweep angle
    SWEEP_RIGHT = "sweep_right"  # pivot back through centre to the same angle on the right
    BACKTRACK = "backtrack"  # after ceiling sweeps find nothing: reverse 5 cm, then re-probe
    TURN_AROUND = "turn_around"  # after retreats run out: pivot 180 deg, then re-probe


@dataclass(frozen=True)
class IRNavPolicy:
    """Tunables for :class:`IRLineNav`.

    The Task-1 route is a fixed, known path (see :mod:`carbot.ir_route`), not a maze to be
    explored, so a junction does not need to be *classified* left/right by sensor pattern —
    the route already knows the action. What the sensor does need to do, per junction, is
    recognise its own **ordered signal sequence** (``RouteJunction.approach``, from real-track
    tracing 2026-08-20) — most of these readings are not even junction-shaped in isolation
    (e.g. the roundabout exit's sequence includes ``0101``, which
    :data:`carbot.ir_geometry.STATE_TABLE` classifies as noise, and ends on ``0110``, ordinary
    centred FOLLOW); only the *order* they arrive in is the real signal.
    """

    # ------------------------------------------------------------------
    # TUNING GUIDE — symptom observed on the real car -> field to change.
    # Change ONE field at a time and re-test; several of these interact so isolate which one
    # is wrong. Per-junction approach sequences, creep distances, and turn magnitudes live in
    # carbot.ir_route (RouteJunction.approach/.creep_cm/.turn_deg), not here — this table only
    # covers the policy-wide knobs below.
    #
    #   Symptom                                    -> Field to adjust
    #   ------------------------------------------------------------------
    #   Never detects a junction, drives straight    -> a RouteJunction.approach step's min_cm
    #   through onto blank paper                        in ir_route.py, v (persistence
    #                                                     requirement outlasting the real hold)
    #   Falsely "arrives" at a junction mid-line      -> that step's min_cm ^, or re-check the
    #                                                     approach sequence against a fresh
    #                                                     real-track log (see examples/39's
    #                                                     per-frame P1..P4 log)
    #   Turn stops short of the new heading            -> shouldn't happen -- the turn is
    #                                                     closed-loop on 0110 now, not timed.
    #                                                     If it does, spin_rate_deg_per_s/
    #                                                     spin_dead_time_s are themselves off;
    #                                                     re-run examples/other/41_motor_spin_angle_sweep.py
    #   Turn never ends, runs to the timeout           -> turn_timeout_scale ^ (if a slow but
    #     ("... timeout, 0110 never seen" in the log)      genuine turn just needs longer), OR
    #                                                     check wheel/axle alignment -- a
    #                                                     chassis fault can stop the car from
    #                                                     ever reacquiring 0110 at all (see
    #                                                     docs/progress/2026-08-20-map1-spin-
    #                                                     recalibration-carpet.md for the kind
    #                                                     of fault to look for)
    #   Wheel speeds during FOLLOW oscillate/snake     -> turn_gain v
    #   Car drifts off-centre before correcting         -> turn_gain ^
    #   Car "hunts" on an already-centred line          -> deadband ^
    #   Car ignores a real small offset                 -> deadband v
    #   After a turn the line is found by the sweep     -> search_sweep_deg ^
    #     but only just (edge of the probe arc)             (probe wider; the
    #                                                      gap-between-pairs
    #                                                      dead zone is ~2.4cm)
    #   Line is straight ahead past the gap and the     -> search_creep_step_s v
    #     creep blows past it without the sensor             or search_creep_speed_ratio v
    #     ever reading black                                (shorter/slower steps)
    #   Search creeps too far before sweeping again     -> search_creep_steps_per_cycle v
    #   Search never stops; car wanders off the map     -> search_give_up_s v (0 = never give up)
    # ------------------------------------------------------------------

    speed: int = 150
    # Proportional steering strength while FOLLOWing. Higher = the inside
    # wheel slows down more for the same line offset (sharper correction).
    # Too high -> oscillates/snakes; too low -> drifts off before correcting.
    turn_gain: float = 2.0
    # |error_fraction| below this counts as "on line" -> drive straight,
    # no correction. Too high -> ignores real small offsets; too low ->
    # constantly makes tiny corrections even when already centred.
    deadband: float = 0.15
    # +1 = right turn, -1 = left turn. Only a fallback used before the first junction commits
    # (every real junction sets its own direction from the route -- see
    # carbot.ir_route.RouteJunction.turn_direction) -- Task-1 never actually reaches this.
    turn_direction: int = 1
    # Measured directly on the Task-1 map paper at speed=150, on carpet
    # underneath the paper (verified 2026-08-20, examples/other/41_motor_spin_angle_sweep.py,
    # 5-point sweep 2-10s, all 5 confirmed a true in-place pivot -- no chassis
    # drift -- before being recorded; linear fit angle = rate*(duration -
    # dead_time)): rate 42.0 deg/s, dead_time 0.41s. See
    # docs/progress/2026-08-20-map1-spin-recalibration-carpet.md for the full
    # sweep data, including an earlier same-day attempt discarded because a
    # chassis/wheel issue was making the "pivot" drift sideways mid-spin.
    # Supersedes the 2026-08-18 reading (40.5 deg/s, 0.2s dead_time) taken on
    # the same paper -- the dead-time roughly doubled, most likely the surface
    # underneath (carpet vs. whatever the 08-18 measurement sat on) or
    # mechanical wear/realignment from the chassis issue above, not a fixed
    # property of the car. NOT extrapolated from the camera-based calibration
    # (examples/ai_camera/23_cam_spin_rate_check.py, measured on a different, textured
    # surface elsewhere in the room) — friction differs by surface, so that
    # number does not transfer here. These two constants are only valid at
    # `speed=150` on this paper, on this surface; re-run the sweep before
    # trusting them at a different speed, print, or underlying floor.
    spin_rate_deg_per_s: float = 42.0
    spin_dead_time_s: float = 0.41
    # Closed-loop junction turns (2026-08-20, see IRLineNav._turn_step) end on
    # TURN_COMPLETE_READING (0110), not a fixed duration, but still need a safety ceiling in
    # case that reading never comes back (misalignment, a genuine sensor fault) -- without
    # one a lost car here would spin forever. turn_timeout_s() multiplies the nominal timed
    # duration for a junction's expected turn_deg (RouteJunction.turn_deg) by this scale.
    turn_timeout_scale: float = 2.0
    # 2026-08-20, ninth pass, real-track: a single 0110 frame mid-spin ended the start T's
    # turn at ~28 degrees of a nominal 90 -- some other feature swept past briefly happened to
    # read the same bits. TURN_COMPLETE_READING must now be sustained this long before the
    # turn is considered done -- see IRLineNav._turn_step.
    turn_confirm_s: float = 0.08
    # 2026-08-20, eleventh pass, real-track: the roundabout entry's approach was reset by a
    # single frame of genuine Kind.ON_LINE/DRIFT (real evidence the car left the junction area,
    # per the tenth-pass fix) sitting between the 1001 shoulder and the step's own 0000 -- one
    # frame of it is as likely to be the same transitional shoulder as a real reacquisition.
    # Require it sustained this long before trusting it enough to reset -- see
    # IRLineNav._approach_step.
    approach_break_confirm_s: float = 0.05
    # Forward speed used to convert a RouteJunction's per-junction creep_cm (see
    # carbot.ir_route) into a drive duration. Floor reference was 11.7 cm/s at speed=200 (see
    # docs/progress/2026-08-14-travel-speed-and-coverage.md). Hardware runners must pass their
    # measured surface-specific value explicitly; this generic policy default remains 10cm/s
    # for backwards-compatible library behavior and synthetic tests.
    forward_speed_cm_per_s: float = 10.0
    forward_speed_reference_pwm: int = 150
    # ------------------------------------------------------------------
    # LINE-RECOVERY SEARCH — what to do when no channel sees black.
    #
    # The sensor bar spans ~10mm but the two pairs (Out4+Out3 left,
    # Out1+Out2 right) are separated by a ~2.4cm dead zone, and the
    # Task-1 route line is only ~2cm wide — so after a junction turn the
    # car can point straight into the gap and read (0,0,0,0) even though
    # the line is still just ahead. Stopping can never recover from that,
    # so on a lost line the car probes:
    #   1. sweep `search_sweep_deg` to the left,
    #   2. sweep back through centre to `search_sweep_deg` right
    #      (2x the left-sweep rotation, watching for the line the whole
    #      way — see `IRLineNav._search_step`),
    #   3. if still nothing, creep forward in `search_creep_step_s` steps
    #      at `search_creep_speed_ratio * speed` to bring the line under
    #      the bar, repeating steps 1-3 after `search_creep_steps_per_cycle`
    #      steps.
    # Any channel reading black at any moment ends the search and resumes
    # normal follow steering. Sweep timings use the same calibrated
    # spin model as the junction turn (`spin_rate_deg_per_s` +
    # `spin_dead_time_s`), which was measured at speed=150.
    # ------------------------------------------------------------------
    # Maximum in-place pendulum sweep angle (odd-multiple stepping only reaches
    # the even steps; this is the ceiling). The search starts at
    # `search_sweep_min_deg` (5 deg) and, if a full left+right pendulum pair finds no
    # line, increases by `search_sweep_step_deg` (5 deg) up to this ceiling -- i.e.
    # 5 -> 10 -> 15 -> ... -> 45 deg -- then stays at the ceiling until give-up.
    # 2026-08-20: bumped from 10.0 -- ARC 1 -> Phase 4 real-track runs went off-track and
    # stayed lost, consistent with the corner's uneven surface (paper not flat over carpet
    # there) throwing the sensor readings off enough that a 10 deg probe missed the line's
    # edge.
    search_sweep_deg: float = 45.0
    # Start angle of the in-place pendulum sweep (degrees).
    search_sweep_min_deg: float = 5.0
    # How many degrees each pendulum sweep angle steps up (5 -> 10 -> 15 ...).
    search_sweep_step_deg: float = 5.0
    # The forward creep phase was removed (2026-08-20+); search is in-place only. These
    # fields are kept for back-compat with the CLI/args plumbing and are no longer used by
    # the search -- a lost line never drives forward.
    search_creep_step_s: float = 0.3
    # Creep speed as a fraction of `speed` (unused by the in-place search).
    search_creep_speed_ratio: float = 0.5
    # Creep steps per search cycle (unused by the in-place search).
    search_creep_steps_per_cycle: int = 4
    # Stop the car after this many total seconds of searching.
    # 0 disables the timeout (not recommended -- the car will wander).
    search_give_up_s: float = 30.0
    # After a full pair at the ceiling angle (`search_sweep_deg`) still finds no line, the
    # car backs up `search_retreat_cm` (reverse on the spot) then re-probes from the min
    # angle. This happens up to `search_retreat_count` times; if the line is still not found
    # after those retreats, the car pivots 180 deg in place and re-probes again.
    search_retreat_cm: float = 5.0
    search_retreat_count: int = 2
    # The junction sequence for the lap. `turn_direction` above is only the fallback used
    # before the first junction is reached; each junction carries its own direction.
    route: RoutePlan = TASK1_ROUTE
    # Independent phase tests follow only the local line and stop on their own bounded distance;
    # they must not accidentally execute a full-route junction action.
    junction_detection_enabled: bool = True
    # A tight printed curve can cover three adjacent sensors and therefore looks like a junction
    # to the geometry table (0111/1110). Full-route and straight-phase control must hold on those
    # patterns until a route sequence confirms a junction. A bounded independent ARC/roundabout
    # test, however, has no junction action and must use the pattern's left/right offset to remain
    # on the curve instead of holding a stale straight command.
    curve_pattern_steering_enabled: bool = False
    # Optional directional guard for a bounded curve whose geometry is known in advance:
    # -1 permits left corrections only, +1 permits right corrections only, and 0 permits both.
    # Opposite-side readings are treated as paper/height noise and never replace the last
    # localising history. A lost-line search under this guard probes only in the permitted
    # direction and stops at the angle ceiling instead of issuing the opposite pivot.
    steering_direction_limit: int = 0
    # Stretches of continuous curve too tight for the steady-state gains above -- see
    # carbot.ir_route.CornerWindow. Applied only while FOLLOWing; never turns off line
    # tracking, only drives slower and corrects harder for that stretch.
    corner_windows: tuple[CornerWindow, ...] = TASK1_CORNER_WINDOWS
    # ------------------------------------------------------------------
    # PHASE TRACKER — 2026-08-20, see tasks/ir-sensor-tracking/
    # phase-tracking-and-junction-detection-plan.md. Distinguishes "on a straight phase"
    # (sustained 0110) from "on an arc" (a repeating 0110/0100-style correction rhythm), to
    # gate RouteJunction.min_phase_transitions/.min_arc_cm preconditions -- e.g. the
    # roundabout entry's approach sequence is not even attempted until the tracker confirms
    # Phase 6 and ARC 3 are actually done, not just "the distance gate is open".
    # ------------------------------------------------------------------
    # A non-"0110" reading must persist this long before it counts as a real correction event
    # (flips straight<->arc mode) -- shorter blips are noise (paper texture, a single-frame
    # misread) and must not flip the mode. Deliberately longer than the old junction_min_s
    # (0.15s, filtered single-sample paper-fold noise for a *sustained crossbar* check) --
    # this filters shorter single-frame misreads without confusing them for the genuine,
    # repeating arc-correction rhythm.
    phase_transition_dwell_s: float = 0.8
    # ------------------------------------------------------------------
    # OFF-TRACK RECOVERY — 2026-08-20, see the planning doc above.
    #
    # A junction's own 1111 hold is well under 1s (1.65-1.9cm at ~10cm/s); 1111 sustained for
    # a full off_track_dwell_s cannot be a junction, only carpet beyond the paper's edge (see
    # docs/hardware/ir-tracing-sensor.md -- no return reads the same as black). 0000 sustained
    # that long means blank paper, off any line. Either one triggers reverse-replay: pop the
    # last reverse_replay_window_s of actually-commanded (left, right, dt) history and re-issue
    # each entry sign-flipped, newest first -- retracing the real path back, not a freshly
    # guessed reverse manoeuvre. Falls through to the existing sweep SEARCH only if the full
    # replay finishes and the sensor still reads 0000/1111.
    # ------------------------------------------------------------------
    off_track_dwell_s: float = 2.0
    reverse_replay_window_s: float = 2.0

    def __post_init__(self) -> None:
        if not 0 <= self.speed <= 1000:
            raise ValueError("speed must be in [0, 1000]")
        if self.turn_gain <= 0:
            raise ValueError("turn_gain must be positive")
        if not 0.0 <= self.deadband < 1.0:
            raise ValueError("deadband must be in [0, 1)")
        if self.turn_direction not in (1, -1):
            raise ValueError("turn_direction must be 1 (right) or -1 (left)")
        if self.steering_direction_limit not in (-1, 0, 1):
            raise ValueError("steering_direction_limit must be -1, 0, or 1")
        if self.spin_rate_deg_per_s <= 0:
            raise ValueError("spin_rate_deg_per_s must be positive")
        if self.spin_dead_time_s < 0:
            raise ValueError("spin_dead_time_s must be non-negative")
        if self.turn_timeout_scale <= 0:
            raise ValueError("turn_timeout_scale must be positive")
        if self.turn_confirm_s < 0:
            raise ValueError("turn_confirm_s must be non-negative")
        if self.approach_break_confirm_s < 0:
            raise ValueError("approach_break_confirm_s must be non-negative")
        if self.forward_speed_cm_per_s <= 0:
            raise ValueError("forward_speed_cm_per_s must be positive")
        if self.forward_speed_reference_pwm <= 0:
            raise ValueError("forward_speed_reference_pwm must be positive")
        if self.search_sweep_deg < 0:
            raise ValueError("search_sweep_deg must be non-negative")
        if self.search_sweep_min_deg < 0:
            raise ValueError("search_sweep_min_deg must be non-negative")
        if self.search_sweep_step_deg <= 0:
            raise ValueError("search_sweep_step_deg must be positive")
        if self.search_sweep_min_deg > self.search_sweep_deg:
            raise ValueError("search_sweep_min_deg must be <= search_sweep_deg")
        if self.search_retreat_cm < 0:
            raise ValueError("search_retreat_cm must be non-negative")
        if self.search_retreat_count < 0:
            raise ValueError("search_retreat_count must be non-negative")
        if self.search_creep_step_s < 0:
            raise ValueError("search_creep_step_s must be non-negative")
        if not 0.0 < self.search_creep_speed_ratio <= 1.0:
            raise ValueError("search_creep_speed_ratio must be in (0, 1]")
        if self.search_creep_steps_per_cycle < 1:
            raise ValueError("search_creep_steps_per_cycle must be >= 1")
        if self.search_give_up_s < 0:
            raise ValueError("search_give_up_s must be non-negative")
        if self.phase_transition_dwell_s < 0:
            raise ValueError("phase_transition_dwell_s must be non-negative")
        if self.off_track_dwell_s <= 0:
            raise ValueError("off_track_dwell_s must be positive")
        if self.reverse_replay_window_s <= 0:
            raise ValueError("reverse_replay_window_s must be positive")

    def turn_timeout_s(self, turn_deg: float) -> float:
        """Safety ceiling for a closed-loop junction turn (see `IRLineNav._turn_step`) --
        `turn_timeout_scale` times the nominal timed duration for `turn_deg`, generous enough
        that a turn genuinely slower than calibrated still gets to finish."""
        return self.turn_timeout_scale * (
            self.spin_dead_time_s + turn_deg / self.spin_rate_deg_per_s
        )

    def sweep_duration(self, deg: float) -> float:
        """Time to spin ``deg`` degrees, using the same calibrated spin model
        as the junction turn (rate 42.0 deg/s, dead time 0.41s at speed 150)."""
        return self.spin_dead_time_s + deg / self.spin_rate_deg_per_s


@dataclass(frozen=True)
class IRNavCommand:
    """One step of wheel speeds plus why, for the log."""

    left: int
    right: int
    reason: str
    state: IRNavState


class IRLineNav:
    """Tracks state across cycles: follow the line, matching a junction's ordered approach
    sequence as it comes into range, then creep + a closed-loop turn (or an immediate
    cross/stop) once that sequence completes.

    Call :meth:`step` once per IR read with the cycle-to-cycle ``dt`` in seconds. A junction
    is *detected* by matching its specific ``RouteJunction.approach`` sequence (see
    :mod:`carbot.ir_route`) but not *classified* by the reading alone — the turn direction,
    creep distance, and action all come from the pre-known route.
    """

    def __init__(self, policy: IRNavPolicy | None = None) -> None:
        self.policy = policy or IRNavPolicy()
        self.state = IRNavState.FOLLOW
        #: Progress through the pending junction's approach sequence -- see `_approach_step`.
        self._approach_index = 0
        self._approach_cm = 0.0
        self._approach_break_elapsed = 0.0
        self._creep_elapsed = 0.0
        self._creep_target_cm = 0.0  # set by _commit_junction before JUNCTION_CREEP is entered
        self._turn_elapsed = 0.0
        self._turn_confirm_elapsed = 0.0
        self._turn_target_deg = 0.0  # set by _commit_junction before JUNCTION_TURN is entered
        self._search_phase = IRSearchPhase.SWEEP_LEFT
        self._search_elapsed = 0.0  # time in the current search sub-phase
        self._search_total = 0.0  # total time spent searching
        self._search_sweep_deg = self.policy.search_sweep_min_deg  # current pendulum angle (deg)
        self._search_creep_steps = 0  # kept for back-compat; unused by in-place search
        self._search_retreats = 0  # how many 5cm retreats have been made this search
        self._last_command: IRNavCommand | None = None
        self._last_permitted_curve_command: IRNavCommand | None = None
        self._issued_command: IRNavCommand | None = None
        self._progress_cm_this_step = 0.0
        self._last_localising: tuple[int, int, int, int] | None = (0, 1, 1, 0)
        #: Which junction the route expects next, and how far since the last one. The action
        #: comes from here rather than from the reading — see `carbot.ir_route`.
        self.junctions = JunctionSequencer(self.policy.route)
        #: Set while the car is inside the scripted turn, so the distance gate is not fed by
        #: a pivot that covers no ground.
        self._turn_direction = self.policy.turn_direction
        self.junctions_seen = 0
        self.last_junction: str | None = None
        self.junctions_rejected = 0
        self.noise_frames = 0
        #: True from a crossed junction until the bar clears it, so the same dark feature is
        #: not re-detected and does not steer the car onto the branch it just declined.
        self._crossing = False
        #: Straight/arc phase tracker (see IRNavPolicy.phase_transition_dwell_s) -- reset
        #: whenever a junction is accepted, since each leg starts a fresh straight/arc
        #: sequence. "straight" is the default starting assumption for every leg.
        self._phase_mode = "straight"
        self._straight_cm = 0.0
        self._arc_cm = 0.0
        self._phase_transitions = 0
        self._phase_candidate_elapsed = 0.0
        #: Off-track recovery (see IRNavPolicy.off_track_dwell_s/.reverse_replay_window_s).
        self._off_track_reading: tuple[int, int, int, int] | None = None
        self._off_track_elapsed = 0.0
        self._command_history: deque[tuple[int, int, float]] = deque()
        self._command_history_s = 0.0
        self._reverse_queue: deque[tuple[int, int, float]] = deque()
        self._reverse_elapsed_in_segment = 0.0

    def step(self, reading: IRLineReading, dt: float) -> IRNavCommand:
        if dt < 0:
            raise ValueError("dt must be non-negative")
        # Latched: neither a completed route nor a safety failure can be restarted by a stray
        # sensor reading.
        if self.state is IRNavState.STOPPED:
            return self._halt("route complete")
        if self.state is IRNavState.FAILED:
            return self._fail("safety stop remains latched")

        # Credit the command that was actually active during the interval which just elapsed.
        # The old implementation credited full-speed distance from state alone, so a PWM-90
        # corner correction accumulated the same centimetres as PWM 150 and even a heavily
        # asymmetric steering command advanced at full speed.
        self._progress_cm_this_step = 0.0
        if self._issued_command is not None and self._issued_command.state in (
            IRNavState.FOLLOW,
            IRNavState.JUNCTION_CREEP,
        ):
            progress_cm = estimate_forward_distance_cm(
                dt=dt,
                left=self._issued_command.left,
                right=self._issued_command.right,
                reference_pwm=self.policy.forward_speed_reference_pwm,
                reference_speed_cm_s=self.policy.forward_speed_cm_per_s,
            )
            self._progress_cm_this_step = progress_cm
            self.junctions.travel(progress_cm)
            if self._issued_command.state is IRNavState.FOLLOW and self.state is IRNavState.FOLLOW:
                self._update_phase_tracker(reading, dt, progress_cm)

        # Off-track recovery (2026-08-20): checked before anything else, on the raw reading,
        # regardless of current state -- except REVERSE itself (can't re-trigger mid-replay)
        # and the blind junction manoeuvres (JUNCTION_CREEP/JUNCTION_TURN are short, deliberately
        # sensor-blind, and already have their own timeout; a 2s dwell rarely applies there and
        # interrupting one mid-manoeuvre with a reverse-replay is more likely to make things
        # worse than better). See IRNavPolicy.off_track_dwell_s.
        if self.state in (IRNavState.FOLLOW, IRNavState.SEARCH):
            self._update_off_track_timer(reading, dt)
            if self._off_track_elapsed >= self.policy.off_track_dwell_s and self._command_history:
                self._enter_reverse()

        replaying = self.state is IRNavState.REVERSE
        if replaying:
            cmd = self._reverse_step(reading, dt)
        elif self.state is IRNavState.JUNCTION_TURN:
            cmd = self._turn_step(reading, dt)
        elif self.state is IRNavState.JUNCTION_CREEP:
            cmd = self._creep_step(reading, dt)
        elif self.state is IRNavState.SEARCH:
            cmd = self._search_step(reading, dt)
        else:
            cmd = self._follow_step(reading, dt)

        # Reverse history contains forward-progress commands only. Recording SEARCH pivots made
        # reverse-replay undo the search pendulum instead of retracing the path that lost the line.
        if not replaying and cmd.state in (IRNavState.FOLLOW, IRNavState.JUNCTION_CREEP):
            self._record_command(cmd, dt)
        self._issued_command = cmd
        return cmd

    def _halt(self, note: str) -> IRNavCommand:
        cmd = IRNavCommand(0, 0, note, IRNavState.STOPPED)
        self._last_command = cmd
        return cmd

    def _fail(self, note: str) -> IRNavCommand:
        self.state = IRNavState.FAILED
        cmd = IRNavCommand(0, 0, note, IRNavState.FAILED)
        self._last_command = cmd
        return cmd

    # ------------------------------------------------------------ off-track recovery
    def _update_off_track_timer(self, reading: IRLineReading, dt: float) -> None:
        """Track how long the *same* off-track reading (0000 or 1111) has been continuous.

        Keyed to a specific reading, not "any qualifying one" -- switching between 0000 and
        1111 mid-window (e.g. clipping the paper edge) does not represent 2s of sitting still
        off-track, so it restarts the clock rather than carrying it over.
        """
        off_track_bits = ((0, 0, 0, 0), (1, 1, 1, 1))
        if reading.physical in off_track_bits and reading.physical == self._off_track_reading:
            self._off_track_elapsed += dt
        elif reading.physical in off_track_bits:
            self._off_track_reading = reading.physical
            self._off_track_elapsed = dt
        else:
            self._off_track_reading = None
            self._off_track_elapsed = 0.0

    def _record_command(self, cmd: IRNavCommand, dt: float) -> None:
        """Append to the rolling command history used by reverse-replay, trimming anything
        older than the replay window needs (kept with a small margin, not trimmed exactly to
        the window, so a slightly-late off-track trigger still has the full window available).
        """
        self._command_history.append((cmd.left, cmd.right, dt))
        self._command_history_s += dt
        margin_s = self.policy.reverse_replay_window_s + 1.0
        while self._command_history_s > margin_s and self._command_history:
            _, _, old_dt = self._command_history.popleft()
            self._command_history_s -= old_dt

    def _enter_reverse(self) -> None:
        """Off-track confirmed: snapshot the command history (newest-first) and start
        replaying it sign-flipped. The history is consumed by this snapshot; a fresh one
        builds up again once normal driving resumes."""
        self.state = IRNavState.REVERSE
        self._reverse_queue = deque(reversed(self._command_history))
        self._command_history.clear()
        self._command_history_s = 0.0
        self._reverse_elapsed_in_segment = 0.0
        self._off_track_reading = None
        self._off_track_elapsed = 0.0

    def _reverse_step(self, reading: IRLineReading, dt: float) -> IRNavCommand:
        """Replay the pre-off-track command history backward, one recorded segment at a time.

        Ends early when a valid line reading reappears and resumes FOLLOW. On a direction-limited
        curve, opposite-side and impossible NOISE readings do not count as reacquisition; raised
        paper near the edge produced exactly those false exits during the 06:51 ARC 1 run. If the
        whole window replays with no valid reacquisition, falls through to the existing SEARCH.
        """
        off_track = reading.physical in ((0, 0, 0, 0), (1, 1, 1, 1))
        valid_reacquisition = not off_track
        if valid_reacquisition and self.policy.steering_direction_limit:
            valid_reacquisition = (
                reading.state.kind is not Kind.NOISE
                and not self._direction_is_blocked(reading.state)
                and (
                    reading.state.kind in (Kind.ON_LINE, Kind.DRIFT)
                    or (
                        self.policy.curve_pattern_steering_enabled
                        and reading.state.kind is Kind.JUNCTION
                        and reading.state.direction == self.policy.steering_direction_limit
                    )
                )
            )
        if valid_reacquisition:
            self.state = IRNavState.FOLLOW
            self._last_localising = None
            return self._follow_step(reading, 0.0)

        if not self._reverse_queue:
            self._enter_search(preserve_total=True)
            return self._search_step(reading, 0.0)

        left, right, seg_dt = self._reverse_queue[0]
        self._reverse_elapsed_in_segment += dt
        if self._reverse_elapsed_in_segment >= seg_dt:
            self._reverse_queue.popleft()
            self._reverse_elapsed_in_segment = 0.0
        cmd = IRNavCommand(
            -left,
            -right,
            f"reverse-replay: retracing off-track path, {len(self._reverse_queue)} steps left",
            IRNavState.REVERSE,
        )
        self._last_command = cmd
        return cmd

    # ------------------------------------------------------------ phase tracker
    def _update_phase_tracker(self, reading: IRLineReading, dt: float, progress_cm: float) -> None:
        """(b/c/d) Distinguish "on a straight phase" from "on an arc" by the correction
        rhythm: sustained `0110` means straight, a recurring non-`0110` correction means an
        arc. See IRNavPolicy.phase_transition_dwell_s and the 2026-08-20 planning doc. Only
        called for readings that fell through _approach_step (i.e. not part of an active
        junction approach) -- see _follow_step.
        """
        if self._phase_mode == "straight":
            self._straight_cm += progress_cm
        else:
            self._arc_cm += progress_cm

        # 0000, junction-shaped, and impossible/noise readings are not evidence of an arc.
        # The old "anything except 0110 means arc" rule turned a sustained line loss into a
        # phase transition and eventually auto-accepted the roundabout entry.
        if reading.state.kind not in (Kind.ON_LINE, Kind.DRIFT):
            self._phase_candidate_elapsed = 0.0
            return

        is_straight = reading.physical == (0, 1, 1, 0)

        opposes_current_mode = is_straight if self._phase_mode == "arc" else not is_straight
        if not opposes_current_mode:
            self._phase_candidate_elapsed = 0.0
            return
        self._phase_candidate_elapsed += dt
        if self._phase_candidate_elapsed < self.policy.phase_transition_dwell_s:
            return
        # Confirmed: flip mode, count it, and start the new mode's accumulator fresh.
        self._phase_transitions += 1
        self._phase_candidate_elapsed = 0.0
        if self._phase_mode == "straight":
            self._phase_mode = "arc"
            self._arc_cm = 0.0
        else:
            self._phase_mode = "straight"
            self._straight_cm = 0.0

    def _reset_phase_tracker(self) -> None:
        """Called whenever a junction is accepted -- each leg starts a fresh straight/arc
        sequence, "straight" by default."""
        self._phase_mode = "straight"
        self._straight_cm = 0.0
        self._arc_cm = 0.0
        self._phase_transitions = 0
        self._phase_candidate_elapsed = 0.0

    def _phase_precondition_met(self, pending: RouteJunction) -> bool:
        """Whether the phase tracker confirms enough of the preceding leg is done to even
        start matching `pending`'s approach sequence -- see
        carbot.ir_route.RouteJunction.min_phase_transitions/.min_arc_cm."""
        return (
            self._phase_transitions >= pending.min_phase_transitions
            and self._arc_cm >= pending.min_arc_cm
        )

    def _active_corner_window(self) -> CornerWindow | None:
        """Return the active ARC 1/2/3 corner window, if any.

        These corners are not junctions. ``window.while_pending`` remains
        ``roundabout entry`` through all three; this method only checks whether the
        estimated position falls inside a configured corner interval.
        """
        pending = self.junctions.pending
        cm = self.junctions.cm_since_previous
        for window in self.policy.corner_windows:
            if pending.name == window.while_pending and window.start_cm <= cm <= window.end_cm:
                return window
        return None

    def _steer(self, state: IRState, note: str) -> IRNavCommand:
        speed = self.policy.speed
        inner_ratio = state.inner_ratio
        window = self._active_corner_window()
        if window is not None:
            speed = round(speed * window.speed_scale)
            inner_ratio = max(0.0, min(1.0, inner_ratio * window.inner_ratio_scale))
            note = f"{note} [{window.name} window]"
        left, right = wheel_speeds(speed, state.direction, inner_ratio)
        cmd = IRNavCommand(left, right, note, IRNavState.FOLLOW)
        self._last_command = cmd
        if (
            self.policy.steering_direction_limit
            and state.direction == self.policy.steering_direction_limit
        ):
            # Keep this across centred readings. If raised paper later creates a false
            # opposite-side hit, holding the most recent command would otherwise hold the
            # centred L=R command and send the chassis straight out of the known curve.
            self._last_permitted_curve_command = cmd
        return cmd

    def _direction_is_blocked(self, state: IRState) -> bool:
        """Whether a known one-way curve forbids this reading's steering direction."""
        limit = self.policy.steering_direction_limit
        return limit != 0 and state.direction != 0 and state.direction != limit

    def _permitted_curve_side_state(self) -> IRState:
        """Return the slight DRIFT state on a one-way curve's permitted side."""
        bits = (0, 1, 0, 0) if self.policy.steering_direction_limit < 0 else (0, 0, 1, 0)
        return classify(bits, physical=True)

    def _hold_permitted_curve_direction(self, base_reason: str) -> IRNavCommand:
        """Ignore opposite-side noise without falling back to unsafe straight travel.

        A bounded one-way ARC keeps its most recent permitted directional correction across
        centred frames. If no such correction exists yet, use the slight DRIFT state on the
        known curve side as a gentle fallback (L110/R150 at the current Phase 3/5/7 tuning).
        """
        if self._last_permitted_curve_command is not None:
            previous = self._last_permitted_curve_command
            cmd = IRNavCommand(
                previous.left,
                previous.right,
                f"{base_reason}: maintaining last permitted curve correction",
                IRNavState.FOLLOW,
            )
            self._last_command = cmd
            return cmd

        fallback = self._permitted_curve_side_state()
        return self._steer(fallback, f"{base_reason}: gentle curve-side fallback")

    def _hold(self, base_reason: str) -> IRNavCommand:
        """Keep driving the last steady command instead of correcting on this reading.

        Used for genuine noise (impossible from one line) and while still mid-way through a
        junction's approach sequence: neither should feed the generic offset-based
        correction, which is only valid for a single straight line under the bar.
        """
        if self._last_command is not None:
            return IRNavCommand(
                self._last_command.left,
                self._last_command.right,
                f"{base_reason}: holding previous",
                IRNavState.FOLLOW,
            )
        return self._steer(classify((0, 1, 1, 0), physical=True), f"{base_reason}: no history")

    def _approach_step(
        self, pending: RouteJunction, reading: IRLineReading, dt: float
    ) -> IRNavCommand | None:
        """(a/e/f/g/h) Advance `pending`'s ordered approach sequence
        (`carbot.ir_route.RouteJunction.approach`) if `reading` matches the step currently
        being tracked, or the next one (a fast transition).

        Returns a hold command while still mid-sequence, the arrival command
        (`_reach_junction`) once the last step completes, or ``None`` if this reading is not
        part of the sequence at all — the caller falls through to normal steering, except for
        `Kind.JUNCTION` readings, which the caller (`_follow_step`) always holds on rather
        than steers (see its 2026-08-20 fourth-pass note): a junction-shaped reading proved,
        twice now on real track, unreliable enough that it must never carry its own action —
        only a phase-tracker input.

        A reading that matches neither the current nor the next expected step is handled by
        whether any progress has actually been made yet (``started``: this step's own
        ``min_cm`` partly satisfied, or already past step 0):

        * **Not started** (index 0, no distance accumulated) — falls through to `_follow_step`,
          which holds on `Kind.JUNCTION` and steers normally on anything else (an ordinary
          curve reading is not junction-shaped and should still steer).
        * **Started** — real progress has been made on a specific junction's sequence, so
          anything that is not a genuine single-line reading (`Kind.ON_LINE`/`Kind.DRIFT`)
          holds *without* resetting that progress (2026-08-20, tenth pass: a lingering
          `Kind.NOISE` echo of the step just matched wiped the roundabout entry's approach
          twice, both times one step short of completing); only `Kind.ON_LINE`/`Kind.DRIFT` --
          real evidence the car is back on ordinary line, not still near the junction --
          resets progress and falls through (2026-08-20 real-track regression: a bare
          non-junction mid-sequence blip steered the car off course before it ever reached the
          actual junction).
        """
        arc_gate_met = (
            pending.arc_trigger_cm > 0
            and self._phase_mode == "arc"
            and self._arc_cm >= pending.arc_trigger_cm
        )
        if not self._phase_precondition_met(pending) and not arc_gate_met:
            # The distance gate alone isn't enough for e/f -- see
            # carbot.ir_route.RouteJunction.min_phase_transitions/.min_arc_cm. Don't even try
            # matching this junction's approach until the phase tracker agrees the preceding
            # leg is actually done; a coincidental early reading otherwise risks the same
            # premature-match class of bug the distance gate exists to prevent.
            return None
        approach = pending.approach
        index = self._approach_index
        step = approach[index]
        started = index > 0 or self._approach_cm > 0
        speed = self.policy.forward_speed_cm_per_s
        remaining_s = (step.min_cm - self._approach_cm) / speed if speed > 0 else 0.0
        # 2026-08-20, seventh pass, real-track: even holding-not-resetting on a stray 0000/
        # junction blip (below) still means every intervening frame's *classification* has to
        # be right, and it was not -- the crossbar accumulation kept getting interrupted at
        # the resolution limit. Once a step has genuinely started, ignore what the sensor says
        # entirely and just accumulate elapsed distance, until the last 0.1s of this step's
        # window -- close enough to completion that a real reading is worth checking again.
        blind = started and remaining_s > 0.1
        if reading.physical == step.bits or blind:
            self._approach_break_elapsed = 0.0
            # Use the command that was physically active over this sensor interval. On the very
            # first call there is no previously issued command, so keep the historical fallback
            # for library callers that begin with a car already moving.
            interval_cm = (
                self._progress_cm_this_step if self._issued_command is not None else dt * speed
            )
            self._approach_cm += interval_cm
            note = reading.summary if reading.physical == step.bits else "ignored, blind window"
            if self._approach_cm < step.min_cm:
                return self._hold(
                    f"approaching {pending.name}, step {index + 1}/{len(approach)} "
                    f"({note}) {self._approach_cm:.2f}/{step.min_cm:.2f}cm"
                )
            if index == len(approach) - 1:
                return self._reach_junction(reading)
            self._approach_index += 1
            self._approach_cm = 0.0
            return self._hold(
                f"approaching {pending.name}, step {self._approach_index + 1}/"
                f"{len(approach)} ({note})"
            )
        if started and index + 1 < len(approach) and reading.physical == approach[index + 1].bits:
            # Fast transition: real progress had already been made (step 0 at least partly
            # matched), and the sensor jumped straight to the next step's reading without a
            # frame catching the one in between. NOT applied from a completely fresh state
            # (started False) -- otherwise an ordinary 0000 (the blind band, or a genuine
            # line loss, common everywhere) would instantly "complete" any junction whose
            # last approach step happens to be 0000, with zero persistence ever checked.
            self._approach_index += 1
            self._approach_cm = 0.0
            self._approach_break_elapsed = 0.0
            return self._approach_step(pending, reading, dt)

        if started and reading.state.kind not in (Kind.ON_LINE, Kind.DRIFT):
            # 2026-08-20, sixth pass, real-track: a stray 0000 blip mid-crossbar (sensor
            # flicker right at the sensor's own resolution limit, not the sequence's own
            # expected 0000 step -- that already matched or fast-transitioned above and never
            # reaches here) was resetting the start-T's 1111 accumulation on every flicker,
            # so it never reached its min_cm and the car oscillated at the junction instead of
            # committing to the turn.
            # 2026-08-20, tenth pass, real-track: the JUNCTION-or-0000 carve-out was still too
            # narrow -- the roundabout entry's approach twice got within one step of
            # completing (1.61/1.65cm of the crossbar; 0.10/0.20cm into the 1001 shoulder) and
            # was wiped both times by a *lingering* Kind.NOISE echo of the step just matched
            # (a JUNCTION/DRIFT reading takes a frame or two to actually clear once the step
            # it belongs to has already been counted and the index has moved on). Only a
            # genuine single-line reading (ON_LINE/DRIFT) is real evidence the car is back on
            # ordinary line and should reset progress; anything else near a junction --
            # JUNCTION, NOISE, or the blind 0000/1111 band -- holds instead.
            self._approach_break_elapsed = 0.0
            return self._hold(f"broke {pending.name}'s approach mid-sequence ({reading.summary})")
        if started:
            # 2026-08-20, eleventh pass, real-track: the roundabout entry's approach reached
            # step 2/3 (1001 shoulder) and was then reset by a single frame of Kind.DRIFT
            # (0001, "far right") between the shoulder and the step's own genuine 0000 -- a
            # real single-line-shaped reading, so the ON_LINE/DRIFT carve-out above correctly
            # let it through, but one frame of it is exactly as likely to be part of the same
            # transitional shoulder as a fresh line reacquisition. Require it to be sustained
            # for approach_break_confirm_s before trusting it enough to reset -- same
            # "confirm, don't act on one frame" pattern as turn_confirm_s.
            self._approach_break_elapsed += dt
            if self._approach_break_elapsed < self.policy.approach_break_confirm_s:
                return self._hold(
                    f"possible break of {pending.name}'s approach, confirming "
                    f"({reading.summary}) {self._approach_break_elapsed:.2f}/"
                    f"{self.policy.approach_break_confirm_s:.2f}s"
                )
            self._approach_index = 0
            self._approach_cm = 0.0
            self._approach_break_elapsed = 0.0
        return None

    def _commit_junction(
        self, label: str, direction: int, creep_cm: float, turn_deg: float, reading: IRLineReading
    ) -> IRNavCommand:
        """Run the turn shared by the start T, roundabout entry, and roundabout exit.

        After the approach sequence is confirmed, creep forward to put the axle on the
        junction centre, then use the closed-loop turn in :meth:`_turn_step`.
        """
        self.junctions_seen += 1
        self.last_junction = label
        self._turn_direction = direction
        self._creep_target_cm = creep_cm
        self._turn_target_deg = turn_deg
        self.state = IRNavState.JUNCTION_CREEP
        self._creep_elapsed = 0.0
        return self._creep_step(reading, 0.0)

    def _reach_junction(self, reading: IRLineReading) -> IRNavCommand:
        """This junction's approach sequence has completed. The route, not the reading, says
        what to do.

        The distance gate comes first: a junction that turns up well before the route expects
        the next one is the junction just handled being read a second time, or a curve taken at
        a shallow enough angle to coincidentally match part of a sequence. Acting on it
        desynchronises the lap.
        """
        state = reading.state
        shortfall = self.junctions.shortfall_cm()
        pending = self.junctions.pending
        if shortfall > 0:
            self.junctions_rejected += 1
            self._approach_index = 0
            self._approach_cm = 0.0
            self._approach_break_elapsed = 0.0
            # Rejected means "not the junction the route is waiting for", not "no information".
            # What produces these is a curve lighting extra channels, and the state table's
            # offset for them is that curve's direction. Steering must keep running on it:
            # the 2026-08-19 two-lap run held straight here instead and drove off the paper
            # in phase 2, with the line already hard left and `1100` rejected by the gate.
            return self._steer(
                state,
                f"junction ignored, {shortfall:.0f}cm short of the {pending.name} gate; "
                f"steering on {state.label}",
            )

        junction = self.junctions.accept()
        self._approach_index = 0
        self._approach_cm = 0.0
        self._approach_break_elapsed = 0.0
        self._reset_phase_tracker()
        if junction.action is JunctionAction.STOP:
            # (h) Final lap: car stops centred on the T junction, task complete.
            self.junctions_seen += 1
            self.last_junction = junction.name
            self.state = IRNavState.STOPPED
            return self._halt(f"route complete at {junction.name}")
        if junction.action is JunctionAction.CROSS:
            # (g) Lap 2+: no turn, straight through into the next lap's Phase 2. The approach
            # sequence's last step (0110) already means the car is centred on the new line,
            # so there is nothing further to creep or turn through.
            self.junctions_seen += 1
            self.last_junction = junction.name
            self._crossing = True
            return self._steer(
                classify((0, 1, 1, 0), physical=True),
                f"crossing {junction.name} straight through",
            )
        return self._commit_junction(
            junction.name, junction.turn_direction, junction.creep_cm, junction.turn_deg, reading
        )

    def _follow_step(self, reading: IRLineReading, dt: float) -> IRNavCommand:
        state = reading.state

        if self._crossing:
            if state.kind is Kind.JUNCTION:
                # Still driving over the junction just crossed. Steering on this reading would
                # pull the car onto the branch it decided not to take.
                return self._steer(
                    classify((0, 1, 1, 0), physical=True),
                    f"still over {self.last_junction}, holding straight",
                )
            self._crossing = False

        pending = self.junctions.pending
        if self.policy.junction_detection_enabled:
            approached = self._approach_step(pending, reading, dt)
            if approached is not None:
                return approached
        if state.kind is Kind.JUNCTION:
            if self.policy.curve_pattern_steering_enabled and state.direction:
                if self._direction_is_blocked(state):
                    self.noise_frames += 1
                    return self._hold_permitted_curve_direction(
                        f"bounded curve mode: opposite-side noise ignored ({state.label})"
                    )
                return self._steer(
                    state,
                    f"bounded curve mode: steering on {state.label}",
                )
            # 2026-08-20, fourth pass, real-track: a lone P1110 partway through the dead-
            # straight start stem still steered the car left (`_approach_step` returning
            # None here used to fall through to the generic offset steer below). Per the
            # operator's direct tracing, a junction-shaped reading that is not part of a
            # confirmed approach sequence must never carry its own action -- STATE_TABLE's
            # offset for Kind.JUNCTION entries was only ever assigned by analogy and never
            # validated for direction. It is only a phase-tracker input (fed above); hold
            # the previous command instead of steering on it.
            return self._hold(f"junction-shaped reading, no confirmed approach ({state.label})")

        if state.kind is Kind.DRIFT and pending is TASK1_ROUTE.prologue[0]:
            # 2026-08-20, fifth pass, real-track: P1000 (outer sensor only, the largest DRIFT
            # offset) mid-stem also steered the car hard left, same failure as the JUNCTION
            # case above. The start stem is print-straight end to end -- unlike every other
            # straight leg, it never needs a real DRIFT correction, so any DRIFT reading here
            # that isn't part of the approach sequence is the "0001/1000 just before the
            # post-crossbar 0000" transitional artifact this module's docstring already
            # documents, not genuine physical drift. Every other straight leg (Phase 2/4/6/8/
            # 10) keeps live DRIFT correction below -- gating those too disabled the basic
            # steer-toward-the-line behaviour outright (confirmed against
            # test_follow_steers_toward_the_line).
            return self._hold(f"start stem, drift reading not part of the approach ({state.label})")

        if state.kind is Kind.NOISE:
            # Non-contiguous black: one 2 cm line cannot produce it, so it is
            # undulation, a mis-tuned pot, or a second feature. Never steer.
            self.noise_frames += 1
            return self._hold(f"noise {state.label}")

        if state.kind is Kind.AMBIGUOUS:
            verdict, offset = resolve_blind(self._last_localising)
            if verdict == "blind":
                blind = IRState(state.bits, Kind.DRIFT, offset, state.inner_ratio, "blind band")
                return self._steer(blind, f"blind band, line {'right' if offset > 0 else 'left'}")
            if verdict == "hold" and self._last_command is not None:
                self.noise_frames += 1
                return IRNavCommand(
                    self._last_command.left,
                    self._last_command.right,
                    "all dark straight from centred: undulation, holding previous",
                    IRNavState.FOLLOW,
                )
            self._enter_search()
            return self._search_step(reading, 0.0)

        # ON_LINE or DRIFT — the readings a single line can produce.
        if self._direction_is_blocked(state):
            # Do not store this as `_last_localising`: a following P0000 must be resolved
            # from the last geometrically valid line position, not from a raised-paper
            # false black on the forbidden side.
            self.noise_frames += 1
            return self._hold_permitted_curve_direction(
                f"opposite-side noise ignored ({state.label})"
            )
        self._last_localising = state.bits
        if state.kind is Kind.ON_LINE:
            if self.policy.steering_direction_limit:
                # P0110 only says the sensor bar is centred over the black line. On a known
                # ARC, equal wheel speeds follow the tangent and leave the curve; maintain a
                # mild feed-forward turn even while centred. Straight phases have limit=0
                # and retain ordinary L=R centred behavior.
                curve_state = self._permitted_curve_side_state()
                return self._steer(
                    curve_state,
                    f"centred on bounded curve: maintaining "
                    f"{'left' if curve_state.direction < 0 else 'right'} curvature",
                )
            return self._steer(state, "centred")
        return self._steer(state, f"{state.label}, offset {state.offset_cm:+.1f}cm")

    def _creep_step(self, reading: IRLineReading, dt: float) -> IRNavCommand:
        """Blind straight creep — ignores the sensor, only elapsed time matters.

        Moves the wheel axle (not just the forward-mounted sensor) over the
        junction centre before the turn starts, for `_creep_target_cm` (set per-junction by
        `_commit_junction` from `RouteJunction.creep_cm`) at `forward_speed_cm_per_s`. See
        `_follow_step` for why this must not react to sensor readings mid-creep.
        """
        self._creep_elapsed += dt
        duration = self._creep_target_cm / self.policy.forward_speed_cm_per_s
        if self._creep_elapsed >= duration:
            self.state = IRNavState.JUNCTION_TURN
            self._turn_elapsed = 0.0
            self._turn_confirm_elapsed = 0.0
            return self._turn_step(reading, 0.0)
        return IRNavCommand(
            self.policy.speed,
            self.policy.speed,
            f"junction confirmed; creeping {self._creep_target_cm:.1f}cm "
            f"to centre: {self._creep_elapsed:.2f}/{duration:.2f}s",
            IRNavState.JUNCTION_CREEP,
        )

    def _turn_step(self, reading: IRLineReading, dt: float) -> IRNavCommand:
        """Closed-loop spin: keep turning until the sensor reads `TURN_COMPLETE_READING`
        (0110), not a pure timed spin.

        2026-08-18's original design deliberately ignored the sensor mid-turn ("never exit
        early on line reacquired") because the junction crossbar itself reads black while the
        car pivots on top of it, so checking for *any visible channel* fired almost
        immediately (0.30s into a 2.24s nominal 90° turn, ~12°) — the crossbar, not the new
        line, tripped it. This does not reintroduce that bug: real-track tracing (2026-08-20)
        showed the turn ending in one specific, late-arriving reading (0110, ordinary centred
        FOLLOW) reached only once the car has swept far enough that the outer sensors have
        cleared the old crossbar/curve entirely — checking for that ONE reading, not "any
        channel visible", is what makes closing the loop here safe.

        Three guards: `spin_dead_time_s` as a minimum elapsed time before a 0110 read is
        trusted (the same "motor hasn't really started moving yet" floor the spin calibration
        itself uses — guards against a coincidental 0110 in the very first instant),
        `turn_confirm_s` as a minimum *sustained* 0110 before it counts (2026-08-20, ninth
        pass, real-track: a single 0110 frame mid-spin ended the start T's turn at ~0.67s of a
        nominal 2.24s 90° turn -- ~28°, nowhere near a real crossing -- because some other
        feature the sensor swept past briefly happened to read the same bits; sustaining it
        rules out a one-frame coincidence), and `turn_timeout_s` as a ceiling in case 0110
        never comes back at all (misalignment, a genuine sensor fault) — without it a lost car
        here would spin forever.
        """
        self._turn_elapsed += dt
        timeout_s = self.policy.turn_timeout_s(self._turn_target_deg)
        if (
            self._turn_elapsed >= self.policy.spin_dead_time_s
            and reading.physical == TURN_COMPLETE_READING
        ):
            self._turn_confirm_elapsed += dt
        else:
            self._turn_confirm_elapsed = 0.0
        if self._turn_confirm_elapsed >= self.policy.turn_confirm_s:
            self.state = IRNavState.FOLLOW
            # The pivot's real angle is not otherwise verified, so a 0000 immediately after
            # this must not be resolved with the line position from before the turn: that
            # geometry belongs to the old heading. See the 2026-08-20 fix this line preserves.
            self._last_localising = None
            return IRNavCommand(
                self.policy.speed,
                self.policy.speed,
                "junction turn done: line reacquired (0110)",
                IRNavState.FOLLOW,
            )
        if self._turn_elapsed >= timeout_s:
            self._last_localising = None
            return self._fail(
                f"junction turn failed after {timeout_s:.2f}s: 0110 never seen; "
                "operator intervention required"
            )

        speed = self.policy.speed
        if self._turn_direction > 0:
            left, right = speed, -speed
        else:
            left, right = -speed, speed
        return IRNavCommand(
            left,
            right,
            f"junction turn {'right' if self._turn_direction > 0 else 'left'}: "
            f"watching for 0110, {self._turn_elapsed:.2f}/{timeout_s:.2f}s timeout",
            IRNavState.JUNCTION_TURN,
        )

    # ------------------------------------------------------------ SEARCH
    def _enter_search(self, *, preserve_total: bool = False) -> None:
        """Start (or restart) the line-recovery search from the current heading."""
        self.state = IRNavState.SEARCH
        self._search_phase = IRSearchPhase.SWEEP_LEFT
        self._search_elapsed = 0.0
        if not preserve_total:
            self._search_total = 0.0
        self._search_sweep_deg = self.policy.search_sweep_min_deg
        self._search_creep_steps = 0
        self._search_retreats = 0

    def _begin_search_phase(self, phase: IRSearchPhase) -> None:
        self._search_phase = phase
        self._search_elapsed = 0.0

    def _search_step(self, reading: IRLineReading, dt: float) -> IRNavCommand:
        """Recovery for a lost line: in-place pendulum only, angle steps up.

        The car pivots on the spot -- it never creeps forward. Each sweep level is a full
        pendulum pair: pivot left by ``_search_sweep_deg``, then pivot back through centre
        to the same angle on the right. If neither swing finds a line, the angle steps up
        by ``search_sweep_step_deg`` (5 deg), from ``search_sweep_min_deg`` (5 deg) up to
        ``search_sweep_deg`` (45 deg), then holds at the ceiling until ``search_give_up_s``.

        The sensor is checked every cycle, so the moment any channel sees black the search
        ends and normal follow resumes. Delegating back into `_follow_step` means a
        reacquired reading that is really the start of a junction's approach sequence is
        still handled as one. Sweep timing uses the same calibrated spin model as the
        junction turn; the right swing rotates 2x the left-swing angle so the bar is probed
        on both sides of the heading and the car ends up back near centre.
        """
        # On a known one-way curve, a visible reading on the forbidden side is a paper-height
        # artefact, not a reacquisition. Keep probing in the permitted direction.
        blocked_visible = reading.visible and self._direction_is_blocked(reading.state)
        if reading.visible and not blocked_visible:
            self.state = IRNavState.FOLLOW
            return self._follow_step(reading, 0.0)

        self._search_total += dt
        if self.policy.search_give_up_s > 0 and self._search_total >= self.policy.search_give_up_s:
            return self._fail(f"search failed after {self._search_total:.1f}s")

        self._search_elapsed += dt
        speed = self.policy.speed
        sweep_deg = self._search_sweep_deg

        if self.policy.steering_direction_limit:
            direction = self.policy.steering_direction_limit
            direction_name = "left" if direction < 0 else "right"
            duration = self.policy.sweep_duration(self.policy.search_sweep_deg)
            if self._search_elapsed >= duration:
                return self._fail(
                    f"one-way search {direction_name} reached "
                    f"{self.policy.search_sweep_deg:.0f}deg without reacquiring the line"
                )
            left, right = (-speed, speed) if direction < 0 else (speed, -speed)
            ignored = "; opposite-side noise ignored" if blocked_visible else ""
            return IRNavCommand(
                left,
                right,
                f"search: one-way pivot {direction_name} up to "
                f"{self.policy.search_sweep_deg:.0f}deg "
                f"{self._search_elapsed:.2f}/{duration:.2f}s{ignored}",
                IRNavState.SEARCH,
            )

        if self._search_phase is IRSearchPhase.BACKTRACK:
            # Reverse `search_retreat_cm` on the spot, then re-probe from the min angle.
            retreat_s = self.policy.search_retreat_cm / self.policy.forward_speed_cm_per_s
            if self._search_elapsed >= retreat_s:
                self._search_retreats += 1
                self._search_sweep_deg = self.policy.search_sweep_min_deg
                self._begin_search_phase(IRSearchPhase.SWEEP_LEFT)
                return self._search_step(reading, 0.0)
            return IRNavCommand(
                -speed,
                -speed,
                f"search: backtrack {self.policy.search_retreat_cm:.0f}cm "
                f"{self._search_elapsed:.2f}/{retreat_s:.2f}s",
                IRNavState.SEARCH,
            )

        if self._search_phase is IRSearchPhase.TURN_AROUND:
            # Pivot 180 deg in place, then re-probe from the min angle.
            turn180_s = self.policy.sweep_duration(180.0)
            if self._search_elapsed >= turn180_s:
                self._search_sweep_deg = self.policy.search_sweep_min_deg
                self._begin_search_phase(IRSearchPhase.SWEEP_LEFT)
                return self._search_step(reading, 0.0)
            return IRNavCommand(
                speed,
                -speed,
                f"search: turn around 180deg {self._search_elapsed:.2f}/{turn180_s:.2f}s",
                IRNavState.SEARCH,
            )

        if self._search_phase is IRSearchPhase.SWEEP_LEFT:
            duration = self.policy.sweep_duration(sweep_deg)
            if self._search_elapsed >= duration:
                self._begin_search_phase(IRSearchPhase.SWEEP_RIGHT)
                return self._search_step(reading, 0.0)
            return IRNavCommand(
                -speed,
                speed,
                f"search: pivot left {sweep_deg:.0f}deg {self._search_elapsed:.2f}/{duration:.2f}s",
                IRNavState.SEARCH,
            )

        # SWEEP_RIGHT: from left-sweep heading back through centre to the same angle right --
        # 2x the left-swing angle. On completion, either step the pendulum angle up and
        # repeat, or -- once at the ceiling -- fall into the retreat / turn-around recovery.
        duration = self.policy.sweep_duration(2.0 * sweep_deg)
        if self._search_elapsed >= duration:
            if sweep_deg >= self.policy.search_sweep_deg:
                # A full ceiling pendulum found nothing. Back up (or turn around once the
                # retreats are used up), then re-probe from the min angle.
                next_phase = (
                    IRSearchPhase.BACKTRACK
                    if self._search_retreats < self.policy.search_retreat_count
                    else IRSearchPhase.TURN_AROUND
                )
                if next_phase is IRSearchPhase.TURN_AROUND:
                    self._search_retreats = 0  # fresh search after the 180
                self._begin_search_phase(next_phase)
                return self._search_step(reading, 0.0)
            self._advance_search_sweep_deg()
            self._begin_search_phase(IRSearchPhase.SWEEP_LEFT)
            return self._search_step(reading, 0.0)
        return IRNavCommand(
            speed,
            -speed,
            f"search: pivot right {2.0 * sweep_deg:.0f}deg "
            f"{self._search_elapsed:.2f}/{duration:.2f}s",
            IRNavState.SEARCH,
        )

    def _advance_search_sweep_deg(self) -> None:
        """Bump the pendulum angle up one step, saturating at ``search_sweep_deg``."""
        self._search_sweep_deg = min(
            self.policy.search_sweep_deg,
            self._search_sweep_deg + self.policy.search_sweep_step_deg,
        )
