"""AprilTag-supervised black-line navigation for the Task-1 map.

The chassis has no encoders and no IMU, and the AprilTag pose estimates are
per-tag: position is trustworthy after median filtering, but heading is not
(different visible tags disagree by tens of degrees because of print/paste
misalignment — verified 2026-08-17). So heading/turning cannot be trusted to
tags, and near-field black-line vision is blocked by the camera's ~17 cm blind
zone (measured 2026-08-17: camera 28 cm above the floor, ~9.5 cm ahead of the
chassis centre; the ground-view window starts at the calibration target's near
edge).

:class:`TagSupervisedNav` therefore uses each channel for what it is good at:

* **black-line vision** (``carbot.line_nav.LineNav``) steers the car inside a
  segment — a stable 15 mm stroke in the bird's-eye window at ~0.2-0.3 m is
  followed proportionally, and a near T cross-bar triggers the right spin
  (the visual T detection is what worked on 2026-08-17 after the camera
  re-tilt).
* **AprilTag position** (median-filtered) supervises the plan: confirm the
  car is in the departure zone before starting, veto an early visual T-turn
  until the tracked position has actually reached the T, and stop with a
  report when the position drifts off the planned corridor (the 2026-08-17
  runs drifted west because the visual "line" it locked was not always the
  stem).

The planned route is the Task-1 lap in map metres (map frame: SW origin,
x east, y north — see ``scratch/landmarks/task1-tag-map.json``): departure
zone centre ≈ (0.59, 0.08), stem is the vertical segment x=0.59 from
y=0.08 to the T at (0.59, 0.248), then right onto Phase 2 east
(y=0.248, x increasing).

This module is pure: no camera, no car, no I2C. The caller feeds one
``LineReading`` and one optional ``LandmarkLocalization`` per frame and gets
back a :class:`NavCommand`.
"""

from __future__ import annotations

from dataclasses import dataclass

from carbot.line_nav import LineNav, NavCommand, NavPolicy, NavState

#: Map-frame waypoints of the Task-1 route (metres, x east / y north).
DEPARTURE_ZONE = (0.590, 0.080)  # centre of the 150x150 mm departure box
T_JUNCTION = (0.590, 0.248)  # stem top / Phase 2 east crossing
PHASE2_TARGET_X = 0.660  # first waypoint east on Phase 2 (advisory)


@dataclass(frozen=True)
class TagNavPolicy:
    """Tunables for the tag-supervised layer.

    ``position_corridor_m`` is how far the tracked position may drift from
    the planned route centreline before the nav stops and reports (the
    car then needs a human to put it back). ``t_reached_y_m`` is the y at
    which the T is considered reached (the visual cross-bar triggers the
    spin; the tag position vetoes it until the car is genuinely there).
    """

    position_corridor_m: float = 0.12
    t_reached_y_m: float = 0.20
    #: Right-turn parameters for the anchored T junction turn: a fixed 90 deg
    #: in-place spin once the anchored position has reached the T. The
    #: 2026-08-17 runs instead let the vision steer smoothly right at the
    #: Phase 2 approach and the car looped past the junction, so the turn is
    #: a deliberate timed spin (spin rate anchored to the verified
    #: 53.5 deg/s at speed 200), not a steering correction.
    spin_right_deg: float = 90.0
    spin_deg_per_s_at_200: float = 53.5
    #: Seconds after a T turn during which no further turn is allowed. run-12
    #: (2026-08-17) turned at the T correctly, then the cross-bar evidence
    #: re-armed 0.6 s later and the car spun a second time; the bar after the
    #: turn is the same junction structure, not a new T.
    turn_cooldown_s: float = 6.0
    #: Departure-phase limits. The stem sits inside the camera blind zone at
    #: launch, so the car first aligns its heading to map-north (coarse, tag
    #: heading), then blind-creeps straight until the stem stroke appears in
    #: the BEV window. These bound that phase.
    heading_align_s: float = 3.0
    heading_align_tol_deg: float = 8.0
    depart_timeout_s: float = 5.0
    #: Vision right-turn evidence that may trigger the T spin: the cross-bar
    #: spin (LineNav "horizontal stroke") or a steer with the line far to the
    #: right (Phase 2 east entering the right side of view at the T).
    trigger_right_err_fraction: float = 0.12
    #: Wide windows for the initial departure confirmation — the raw tag
    #: position has a systematic offset (camera intrinsics scaling, verified
    #: 2026-08-17: x read 0.42-0.46 while the car sat on the stem at 0.59),
    #: so the *absolute* position cannot be trusted for a tight gate. The
    #: offset is anchored away after confirmation; the corridor then runs on
    #: the anchored (relative) position.
    depart_confirm_x_lo: float = 0.30
    depart_confirm_x_hi: float = 0.75
    depart_confirm_y_max: float = 0.40
    #: Seconds the last known position stays usable when a frame has no tag
    #: fix (tag detection flickers heavily near the departure zone — the
    #: forward-tilted camera sees only 1-2 tags there; verified 2026-08-17).
    #: During this window the black-line layer keeps driving (it is reliable
    #: on the stem); only a genuinely long tag outage holds the car.
    stale_position_s: float = 8.0

    def __post_init__(self) -> None:
        if self.position_corridor_m <= 0:
            raise ValueError("position_corridor_m must be positive")
        if not self.depart_confirm_x_lo < self.depart_confirm_x_hi:
            raise ValueError("departure x window must be ordered")
        if self.depart_confirm_y_max <= 0:
            raise ValueError("depart_confirm_y_max must be positive")
        if self.t_reached_y_m <= 0:
            raise ValueError("t_reached_y_m must be positive")


class TagSupervisedNav:
    """Wraps a :class:`LineNav` and supervises it with AprilTag position.

    States: ``STARTING`` (no position fix yet — hold), ``DEPART`` (confirmed
    in the departure zone — drive), ``FOLLOW`` (vision steers), ``TURN``
    (vision spin right, position-verified), ``OFF_TRACK`` (position drifted
    out of the corridor — hold with a report).
    """

    def __init__(
        self,
        nav: LineNav | None = None,
        policy: TagNavPolicy | None = None,
    ) -> None:
        self._nav = nav or LineNav(NavPolicy())
        self._policy = policy or TagNavPolicy()
        self.state = "STARTING"
        self._last_pos: tuple[float, float] | None = None
        self._last_pos_age = 0.0
        #: The raw (un-corrected) tag position at departure confirmation.
        #: The tag position has a systematic offset (camera intrinsics
        #: scaling), so after anchoring the supervised position is
        #: ``DEPARTURE_ZONE + (tracked - anchor)`` — relative motion, which
        #: the offset does not corrupt.
        self._anchor_tracked: tuple[float, float] | None = None
        self._turn_elapsed = 0.0
        self._spin_evidence_s = 0.0
        self._turn_cooldown = 0.0
        self._last_heading: float | None = None
        self._align_elapsed = 0.0
        self._depart_elapsed = 0.0
        self._stem_seen_s = 0.0

    @property
    def nav(self) -> LineNav:
        return self._nav

    def step(
        self,
        reading,
        dt: float,
        localization=None,
    ) -> NavCommand:
        """One frame. ``localization`` is a ``LandmarkLocalization`` or None.

        A missing fix is normal (tag detection flickers); the last known
        position stays authoritative for ``stale_position_s`` so the car does
        not crawl frame by frame. Only a genuinely stale fix holds the car —
        the point of this layer is that vision alone drifted off course on
        2026-08-17, so the car never drives without *some* recent position.
        """
        if localization is not None:
            self._last_pos = (localization.x_m, localization.y_m)
            self._last_heading = localization.heading_deg
            self._last_pos_age = 0.0
        else:
            self._last_pos_age += dt

        stale = self._last_pos is None or self._last_pos_age > self._policy.stale_position_s
        if stale:
            if self.state == "STARTING":
                return NavCommand(
                    "hold",
                    0,
                    0,
                    "no tag position fix; holding for departure confirmation",
                    NavState.SEARCH,
                )
            # Already departed: the black-line layer is reliable on the stem
            # (verified 2026-08-17 run-10: 5.5 s of steady stem tracking with
            # no tag fixes), so keep driving on vision until a fix returns.
            return self._nav.step(reading, dt)
        return self._supervised_step(reading, dt, self._last_pos, fresh=localization is not None)

    # ------------------------------------------------------------------
    def _supervised_step(
        self, reading, dt: float, pos: tuple[float, float], *, fresh: bool = True
    ) -> NavCommand:
        x, y = pos
        pol = self._policy

        if self.state == "STARTING":
            if (
                pol.depart_confirm_x_lo <= x <= pol.depart_confirm_x_hi
                and y <= pol.depart_confirm_y_max
            ):
                self._anchor_tracked = (x, y)
                self.state = "DEPART"
                self._depart_elapsed = 0.0
                self._stem_seen_s = 0.0
                return NavCommand(
                    "follow",
                    self._nav.policy.speed,
                    self._nav.policy.speed,
                    f"departure confirmed at raw ({x:.3f}, {y:.3f}); "
                    f"aligning to north",
                    NavState.FOLLOW,
                )
            return NavCommand(
                "hold",
                0,
                0,
                f"no departure fix yet (pos {x:.3f},{y:.3f}); waiting",
                NavState.SEARCH,
            )

        if self.state == "DEPART":
            # Departure phase: the stem is inside the camera blind zone
            # (~0.17 m — measured 2026-08-17), so the black-line detector
            # has nothing real to lock (it locked a departure-zone structure
            # at map x≈0.54 instead of the stem at 0.59, pulling the car
            # north-west every run). Instead: align the heading to north,
            # then drive straight until a *centred* narrow stroke (the stem)
            # appears in the BEV window.
            self._depart_elapsed += dt
            if self._depart_elapsed > pol.depart_timeout_s:
                self.state = "FAILED"
                return NavCommand(
                    "hold", 0, 0,
                    f"DEPART timeout {pol.depart_timeout_s:.0f}s: stem never "
                    "appeared; operator reset",
                    NavState.SEARCH,
                )
            # 1) Heading alignment (coarse): aim for map-north (90 deg).
            heading = getattr(self, "_last_heading", None)
            if heading is not None and self._align_elapsed < pol.heading_align_s:
                delta = (90.0 - heading + 180.0) % 360.0 - 180.0  # -180..180
                if abs(delta) > pol.heading_align_tol_deg:
                    # delta > 0 = heading below 90 (east-ish) -> turn left
                    # (heading increases counterclockwise: 0 east, 90 north).
                    left = delta > 0
                    return NavCommand(
                        "search",
                        (self._nav.policy.speed if left else -self._nav.policy.speed),
                        (-self._nav.policy.speed if left else self._nav.policy.speed),
                        f"depart align: heading {heading:.0f} -> 90, "
                        f"spin {'L' if left else 'R'} {abs(delta):.0f}deg",
                        NavState.SEARCH,
                    )
                self._align_elapsed = pol.heading_align_s  # aligned
            self._align_elapsed += dt
            # 2) Blind creep straight; switch to FOLLOW once a centred
            #    narrow stroke (the stem) is steadily visible.
            if self._stem_like(reading):
                self._stem_seen_s += dt
                if self._stem_seen_s >= 0.3:
                    self.state = "FOLLOW"
                    self._depart_elapsed = 0.0
                    return NavCommand(
                        "follow",
                        self._nav.policy.speed,
                        self._nav.policy.speed,
                        "stem acquired in BEV; line-follow",
                        NavState.FOLLOW,
                    )
            else:
                self._stem_seen_s = 0.0
            return NavCommand(
                "follow",
                self._nav.policy.speed,
                self._nav.policy.speed,
                f"depart: blind creep {self._depart_elapsed:.1f}s "
                f"(stem not in view yet)",
                NavState.FOLLOW,
            )

        # Anchor-correction: the tag position has a systematic offset, so
        # supervise on the *relative* motion from the anchored departure
        # point, not on the raw absolute position.
        ax, ay = self._anchor_tracked if self._anchor_tracked is not None else (x, y)
        sx = DEPARTURE_ZONE[0] + (x - ax)
        sy = DEPARTURE_ZONE[1] + (y - ay)

        # Off-track guard on the anchored position: the stem corridor is the
        # vertical line x = DEPARTURE_ZONE[0].
        if abs(sx - DEPARTURE_ZONE[0]) > pol.position_corridor_m:
            if self.state not in ("OFF_TRACK",):
                self.state = "OFF_TRACK"
                return NavCommand(
                    "off-track",
                    0,
                    0,
                    f"anchored x={sx:.3f} drifted >{pol.position_corridor_m:.2f} m "
                    f"from stem corridor; stop for operator",
                    NavState.FOLLOW,
                )
        elif self.state == "OFF_TRACK":
            self.state = "FOLLOW"  # position back in corridor; resume vision

        if self.state == "OFF_TRACK":
            return NavCommand("off-track", 0, 0, "held off-track; operator reset", NavState.SEARCH)

        if self.state == "TURN":
            # Fixed 90 deg in-place right spin, timed from the verified spin
            # rate. The vision reads during the spin are unreliable, so the
            # spin is open-loop (this is the one place timing is trusted).
            self._turn_elapsed += dt
            spin_s = self._spin_right_s()
            if self._turn_elapsed >= spin_s:
                self.state = "FOLLOW"
                self._turn_elapsed = 0.0
                self._turn_cooldown = pol.turn_cooldown_s
                return NavCommand(
                    "follow",
                    self._nav.policy.speed,
                    self._nav.policy.speed,
                    f"T right turn done ({spin_s:.1f}s); following Phase 2",
                    NavState.FOLLOW,
                )
            return NavCommand(
                "search",
                -self._nav.policy.speed,
                self._nav.policy.speed,
                f"T right turn: spinning {self._turn_elapsed:.1f}/{spin_s:.1f}s",
                NavState.SEARCH,
            )

        # Vision state machine drives; we only veto/allow the T turn. The
        # T right turn does not enter a separate nav state — LineNav stays
        # in FOLLOW and emits a spin command ("horizontal stroke: spin
        # right...") while `_horiz_spin_s` accumulates — so the veto is on
        # that command, not on a state transition.
        cmd = self._nav.step(reading, dt)
        if self._turn_cooldown > 0:
            self._turn_cooldown -= dt
        is_right_spin = cmd.action == "search" and cmd.left < 0 and cmd.right > 0
        right_steer = (
            reading.visible
            and reading.error_fraction is not None
            and reading.error_fraction > pol.trigger_right_err_fraction
        )
        if self._turn_cooldown > 0 and (is_right_spin or right_steer):
            return NavCommand(
                "follow",
                self._nav.policy.speed,
                self._nav.policy.speed,
                f"turn cooldown {self._turn_cooldown:.1f}s; driving straight",
                NavState.FOLLOW,
            )
        at_t = fresh and sy >= pol.t_reached_y_m
        if self.state in ("DEPART", "FOLLOW") and (is_right_spin or right_steer) and at_t:
            # A *fresh* anchored position confirms the T is reached and the
            # vision sees right-turn evidence — begin the fixed right spin.
            self.state = "TURN"
            self._turn_elapsed = 0.0
            self._spin_evidence_s = 0.0
            return NavCommand(
                "search",
                -self._nav.policy.speed,
                self._nav.policy.speed,
                f"T right turn start at anchored y={sy:.3f}",
                NavState.SEARCH,
            )
        if self.state in ("DEPART", "FOLLOW") and (is_right_spin or right_steer):
            # Not confirmed by a fresh position at the T. A *persistent*
            # cross-bar (stale position, but the bar has been in view for a
            # while) is a real T: the departure-zone structure that briefly
            # reads as a near bar (run-11) dies within a few frames, while
            # the actual T cross-bar stays in view as the car approaches.
            # Otherwise veto and keep driving straight.
            if fresh:
                self._spin_evidence_s = 0.0
            else:
                self._spin_evidence_s += dt
                if self._spin_evidence_s >= 0.5:
                    self.state = "TURN"
                    self._turn_elapsed = 0.0
                    self._spin_evidence_s = 0.0
                    return NavCommand(
                        "search",
                        -self._nav.policy.speed,
                        self._nav.policy.speed,
                        "T right turn (persistent bar, pos stale)",
                        NavState.SEARCH,
                    )
            return NavCommand(
                "follow",
                self._nav.policy.speed,
                self._nav.policy.speed,
                f"T-turn vetoed: y={sy:.3f} < {pol.t_reached_y_m:.2f} "
                f"({'stale' if not fresh else 'fresh'})",
                NavState.FOLLOW,
            )
        self._spin_evidence_s = 0.0
        return cmd

    def _spin_right_s(self) -> float:
        pol = self._policy
        rate = pol.spin_deg_per_s_at_200 * (self._nav.policy.speed / 200.0)
        return pol.spin_right_deg / rate if rate > 0 else 0.0

    def _stem_like(self, reading) -> bool:
        """True when the reading is a centred narrow vertical stroke — the
        stem, as opposed to the departure-zone structures (map x≈0.54) that
        the detector locked when nothing real was in view (2026-08-17)."""
        if not reading.visible:
            return False
        if reading.axis != "vertical":
            return False
        if reading.error_fraction is None or abs(reading.error_fraction) > 0.12:
            return False
        return 2 <= reading.line_width_px <= 14
