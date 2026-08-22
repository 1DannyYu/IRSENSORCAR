"""State machine that turns a line reading into wheel commands.

Bottom layer (`carbot.line_follow.detect_line`) reports *where the line is*;
this module decides *what to do about it*: follow the line proportionally,
search when it disappears, and treat a persistent fork as a roundabout with a
time-confirmed lap before the car picks the exit branch.

The whole module is pure and unit-testable — no camera, no car, no I2C. It
emits :class:`NavCommand` values that the caller applies via `carbot.Car`
(``car.drive(command.left, command.right)``). Timing enters through ``dt``
(frames per second), so the same logic drives tests at any simulated rate.

Roundabout exit uses the agreed double confirmation: a fork (``junction``) is
needed **and** the elapsed time inside the roundabout must reach
``roundabout_loop_min_s``, which is anchored to the verified spin rate
(53.5 deg/s at speed 200, `examples/23_cam_spin_rate_check.py`) — one full lap at
the calibrated speed takes ~6.7 s, so the default 6.5 s only counts a lap
that actually went around. Numbers stay tunable through :class:`NavPolicy`
and are expected to be adjusted against real runs.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
from enum import Enum

from carbot.line_follow import LineReading


class NavState(Enum):
    """Where the car is in the navigation plan."""

    FOLLOW = "follow"  # steering on the line
    SEARCH = "search"  # line lost; hold until a centred line returns
    RIGHT_TURN = "right_turn"  # first intersection: timed in-place right spin
    ROUNDABOUT = "roundabout"  # inside a fork; lap is time-confirmed before exit


@dataclass(frozen=True)
class NavPolicy:
    """Tunables for the state machine.

    ``speed`` is the base forward speed (Car range -1000..1000; 200 is the
    calibrated value for the spin rate used in roundabout timing).
    ``turn_gain`` maps |error_fraction| to the inside-wheel slowdown; the
    slower inside wheel is ``speed * ratio`` with ``ratio`` clamped between
    ``min_ratio`` and ``max_ratio``. ``junction_min_s`` is how long a fork must
    persist before it counts as a roundabout entry, and
    ``roundabout_loop_min_s`` is the minimum time inside the roundabout before
    an exit fork is believed — see the module docstring for its anchor.
    """

    speed: int = 200
    # Steering sensitivity. The first on-map run showed 0.45 was far too weak:
    # a 138 px offset produced only a 12-speed wheel difference (L200 R188),
    # the car drove almost straight and ran parallel to the line. 2.5 turns a
    # 0.14 error_fraction into a ~0.8 ratio (40-speed difference), with large
    # errors saturating at min_ratio.
    turn_gain: float = 2.5
    min_ratio: float = 0.15
    max_ratio: float = 1.0
    search_timeout_s: float = 4.0
    # A forward-tilted low camera has a blind cone right under/just ahead of
    # the wheels — the map is a loop with several places (a junction, a map
    # edge) where the 15 mm path is briefly outside that FOV even though the
    # chassis is still squarely on the route (verified 2026-08-16: a
    # confidently-centred stem run lost the line completely, with the
    # calibration target visibly closer/bigger between frames — the chassis
    # had moved, the line just was not in view). Sitting stopped can never
    # recover from that: a static camera never re-sees a line by waiting.
    # Creep straight, at the calibrated speed, for this many seconds before
    # falling back to the stopped wait/search below — long enough to clear a
    # typical blind cone, short enough that a genuine off-track loss is still
    # caught by the existing stop-and-search safety net afterwards. 0 (the
    # historical default) disables this and preserves the old stop-and-wait
    # behaviour.
    blind_creep_s: float = 0.0
    junction_min_s: float = 1.0
    roundabout_loop_min_s: float = 6.5
    # Roundabout entry is opt-in. The 2026-08-15 on-map run treated every
    # wide dark bar as a fork and left the line. Follow the line first.
    enable_roundabout: bool = False
    # A fork only counts as a roundabout entry when the main line also widens
    # beyond this multiple of its recent baseline width. The verified 2026-08-15
    # on-map run showed environment shadows keep the main-line width steady
    # (90-150 px) while a real crossing inflates it (500+ px), so the factor
    # separates the two without needing to know the map.
    junction_width_factor: float = 2.0
    # Minimum fraction of ROI height a branch must span to count as a real fork.
    # The default 0.05 (5%) is 88 rows on 1763-row ROI; scattered shadows die
    # in ~1% (17 rows), while a real crossing spans much longer. The verified
    # 2026-08-15 map shows branches at ~13% (230 rows). Increase this to reject
    # transient dark structure and only count persistent forks as junctions.
    junction_min_branch_rows_fraction: float = 0.10
    # Camera sits right of the axle and tilts forward, so a chassis-centred
    # 15 mm line is left of the image centre. 0.46 is ~4 % of width (~3–4 cm
    # at the look-ahead). Geometric 0.5 would steer the camera onto the line
    # and leave the wheels to the left of it.
    expected_center_fraction: float = 0.46
    # Ignore a one-frame flip larger than this (fraction of half-width).
    max_error_jump: float = 1.25
    # Holding the last steer after a jump drove off the map (L162 R200).
    # Stop immediately, then spin-search if the jump lasts this long.
    jump_search_s: float = 0.4
    # |error| threshold to exit search and enter follow mode (fraction of width).
    reacquire_error: float = 0.65
    search_give_up_s: float = 2.5
    search_sweep_deg: float = 20.0
    search_spin_speed_ratio: float = 0.75
    # The first intersection after the start box is a right turn onto the 15 mm line.
    prefer_right_branch: bool = True
    # How far right of frame centre a T-branch may sit (fraction of width).
    right_branch_max_offset: float = 0.42
    # Require this long of *vertical* follow before a near T may spin. The
    # forward camera sees the T as a far thin bar while the wheels are still
    # on the start-box stem (~10 cm); turning then is early. 1.0 s at speed 150 is
    # ~9 cm, about the stem. 0 lets tests spin on the first near bar.
    right_turn_after_s: float = 1.0
    # Far T (poem / crossing at look-ahead) was 18 px; the 15 mm stroke near
    # the bumper is ~86 px at 2028. Only a fat bar is "the car is at the T".
    # (52 = 86 * 0.61, the same fraction of the near width the old 2 cm-era
    # threshold 70 = 115 * 0.61 used.)
    t_bar_min_width_px: float = 52.0
    # Lower in the ROI is nearer the wheels (forward-tilted camera). A bar
    # in the top half is still ahead; keep driving straight to point 3.
    # On the Task-1 map the raw-y fraction maps non-linearly to distance:
    # frac 0.70 ≈ cross-bar still 40 cm away (the BEV window only spans
    # raw 600..1514 ≈ world 0.62..0.18 m), which triggered the spin ~17 cm
    # before the T and the car "turned early and left the line" (operator,
    # 2026-08-17). 0.85 ≈ raw y>1290 ≈ cross-bar within ~24 cm — still
    # visible (blind zone starts ~17 cm) but genuinely at the junction.
    t_min_roi_y_fraction: float = 0.85
    # Reject a lock thinner than this multiple of the recent line width
    # (the 20 s run followed a 65 px strip off the map after a 150 px path).
    min_width_ratio: float = 0.5
    # |error| below this is treated as on-line. The interior-white run held
    # err≈+0.06 for seconds (L150 R128) and drifted off the 15 mm stem.
    steer_deadband: float = 0.10
    # Timed first right: the 15 s deadband run drove straight off the top of
    # the paper because the T never switched the lock off the far stem.
    # 5 s of follow at speed 150 is ~0.44 m, past the start box and onto the first T.
    # 0 disables the timed spin. A clock-based 90° turn drove off the paper.
    first_right_s: float = 0.0
    first_right_deg: float = 90.0
    # Verified in-place yaw at speed 200 (examples/23_cam_spin_rate_check.py).
    spin_deg_per_s_at_200: float = 53.5

    def __post_init__(self) -> None:
        if not 0 <= self.speed <= 1000:
            raise ValueError("speed must be in [0, 1000]")
        if self.turn_gain <= 0:
            raise ValueError("turn_gain must be positive")
        if not 0.0 < self.min_ratio <= self.max_ratio <= 1.0:
            raise ValueError("min_ratio/max_ratio must satisfy 0 < min <= max <= 1")
        if self.search_timeout_s < 0:
            raise ValueError("search_timeout_s must be non-negative")
        if self.search_give_up_s < 0:
            raise ValueError("search_give_up_s must be non-negative")
        if self.search_sweep_deg < 0:
            raise ValueError("search_sweep_deg must be non-negative")
        if not 0.0 < self.search_spin_speed_ratio <= 1.0:
            raise ValueError("search_spin_speed_ratio must be in (0, 1]")
        if self.blind_creep_s < 0:
            raise ValueError("blind_creep_s must be non-negative")
        if self.junction_min_s < 0:
            raise ValueError("junction_min_s must be non-negative")
        if self.roundabout_loop_min_s <= 0:
            raise ValueError("roundabout_loop_min_s must be positive")
        if self.junction_width_factor <= 1.0:
            raise ValueError("junction_width_factor must be > 1")
        if not 0.0 <= self.junction_min_branch_rows_fraction <= 1.0:
            raise ValueError("junction_min_branch_rows_fraction must be in [0, 1]")
        if not 0.0 < self.expected_center_fraction < 1.0:
            raise ValueError("expected_center_fraction must be in (0, 1)")
        if not 0.0 < self.max_error_jump <= 2.0:
            raise ValueError("max_error_jump must be in (0, 2]")
        if self.jump_search_s < 0:
            raise ValueError("jump_search_s must be non-negative")
        if not 0.0 < self.reacquire_error <= 2.0:
            raise ValueError("reacquire_error must be in (0, 2]")
        if self.search_give_up_s < 0:
            raise ValueError("search_give_up_s must be non-negative")
        if not 0.0 < self.right_branch_max_offset <= 1.0:
            raise ValueError("right_branch_max_offset must be in (0, 1]")
        if self.right_turn_after_s < 0:
            raise ValueError("right_turn_after_s must be non-negative")
        if self.t_bar_min_width_px <= 0:
            raise ValueError("t_bar_min_width_px must be positive")
        if not 0.0 <= self.t_min_roi_y_fraction <= 1.0:
            raise ValueError("t_min_roi_y_fraction must be in [0, 1]")
        if not 0.0 < self.min_width_ratio <= 1.0:
            raise ValueError("min_width_ratio must be in (0, 1]")
        if not 0.0 <= self.steer_deadband < 1.0:
            raise ValueError("steer_deadband must be in [0, 1)")
        if self.first_right_s < 0:
            raise ValueError("first_right_s must be non-negative")
        if self.first_right_deg <= 0:
            raise ValueError("first_right_deg must be positive")
        if self.spin_deg_per_s_at_200 <= 0:
            raise ValueError("spin_deg_per_s_at_200 must be positive")


@dataclass(frozen=True)
class NavCommand:
    """One step of wheel speeds plus why, for the log."""

    action: str
    left: int
    right: int
    reason: str
    state: NavState


class LineNav:
    """Tracks state across frames and emits wheel commands.

    Call :meth:`step` once per camera frame with the frame-to-frame ``dt`` in
    seconds. The instance keeps the previous error for continuity, the time
    spent in each state, and the fork/roundabout bookkeeping. Stateless
    steering logic lives in the module-level :func:`steer_command`.
    """

    # Fraction of the frame width: a candidate line further than this from the
    # locked target counts as "the line left the view", releasing the lock.
    _LOCK_RELEASE_GAP = 0.15

    def __init__(self, policy: NavPolicy | None = None) -> None:
        self.policy = policy or NavPolicy()
        self.state = NavState.FOLLOW
        self._state_time = 0.0
        self._search_direction = "left"
        self._junction_elapsed = 0.0
        self._roundabout_elapsed = 0.0
        self._roundabout_pending = False
        self._prev_error_fraction: float | None = None
        self._jump_elapsed = 0.0
        self._follow_ok_s = 0.0
        self._first_right_done = False
        self._horiz_spin_s = 0.0
        self._t_turn_done = False
        self._blind_creep_elapsed = 0.0
        # Line continuity: the target line is the candidate closest to the last
        # frame's centroid, so the detector switching which dark structure it
        # counts as "main" does not yank the steering (the 2026-08-15 map run
        # jumped from err +0.91 to -0.34 in one frame and the car veered).
        self._last_centroid: float | None = None
        # Ground-view continuity: the BEV x of the last *accepted* reading
        # (never a jump-stop candidate — see `_follow_step`/`_search_step`),
        # fed back into `detect_line(..., prefer_u=...)` next frame so the
        # ground-view detector stays on the line the car was actually
        # driving on instead of re-deciding by BEV-centre proximity. None
        # outside ground-view mode (`reading.ground_u_px` is always None
        # there, so this simply never gets set).
        self.preferred_ground_u: float | None = None
        # Recent main-line widths while not in a fork; a junction only counts
        # as a roundabout entry when the line also widens past the baseline.
        self._baseline_widths: deque[float] = deque(maxlen=30)

    def step(self, reading: LineReading, dt: float) -> NavCommand:
        if dt < 0:
            raise ValueError("dt must be non-negative")
        self._state_time += dt

        if self.state is NavState.ROUNDABOUT:
            return self._roundabout_step(reading, dt)
        if self.state is NavState.SEARCH:
            return self._search_step(reading, dt)
        if self.state is NavState.RIGHT_TURN:
            return self._right_turn_step(reading, dt)
        return self._follow_step(reading, dt)

    def _locked(self, reading: LineReading) -> LineReading:
        """Look-ahead already picked the track path; do not retarget.

        Candidate locking was for the old "most persistent dark strip" detector,
        which flipped between chair legs. It now keeps steering at a stale
        left-edge lock (L30 R200) even when the path is centred.
        """
        return reading

    def _recenter(self, reading: LineReading) -> LineReading:
        """Steer against the calibrated camera offset, not the frame centre.

        ``expected_center_fraction`` corrects a raw-pixel quirk of the
        perspective detector (the camera sits right of the axle, so a
        centred line does not land at frame-centre in that image). A
        ground-view reading's `error_fraction` is already computed in BEV
        world-metres against the calibration target's own centreline — this
        raw-pixel correction does not apply there and previously corrupted
        an already-correct near-zero error into a large false one (verified
        2026-08-16: `error_fraction=-0.01` from the detector became `+0.27`
        after this step), veering the car right from frame one on every run
        instead of driving straight down the stem.
        """
        if not reading.visible or reading.centroid_x is None:
            return reading
        if reading.ground_u_px is not None:
            return reading
        width = reading.roi[3]
        error_px = reading.centroid_x - self.policy.expected_center_fraction * width
        return replace(
            reading,
            error_px=error_px,
            error_fraction=error_px / (width / 2),
        )

    # ------------------------------------------------------------- FOLLOW
    def _follow_step(self, reading: LineReading, dt: float) -> NavCommand:
        if not reading.visible:
            self._junction_elapsed = 0.0
            self._roundabout_pending = False
            self._blind_creep_elapsed += dt
            if self._blind_creep_elapsed <= self.policy.blind_creep_s:
                return self._drive(
                    "follow",
                    self.policy.speed,
                    self.policy.speed,
                    f"line lost: blind creep {self._blind_creep_elapsed:.1f}"
                    f"/{self.policy.blind_creep_s:.1f}s (camera FOV gap, not off-track)",
                )
            if self._state_time >= self.policy.search_timeout_s:
                self._enter(NavState.SEARCH)
                return self._search_step(reading, dt)
            return self._drive("follow", 0, 0, "line lost; waiting to search")

        self._blind_creep_elapsed = 0.0
        reading = self._locked(reading)
        reading = self._prefer_right(reading)
        reading = self._recenter(reading)

        # Finish a T-turn even if one frame looks like a vertical stem. The
        # 8 s junction run spun 0.3 s then froze on "too thin".
        spin_limit = self._right_spin_s()
        near_t = self._is_near_t(reading)
        finishing_t = 0.0 < self._horiz_spin_s < spin_limit
        if self._t_turn_done and reading.axis == "horizontal":
            error = reading.error_fraction or 0.0
            if abs(error) <= 0.20:
                return self._drive(
                    "follow",
                    self.policy.speed,
                    self.policy.speed,
                    "after T: drive the centred path",
                )
            # Not yet aligned with the next line: one short corrective nudge
            # (up to half the nominal spin), then visual search.
            self._horiz_spin_s += dt
            if self._horiz_spin_s >= 0.5 * spin_limit:
                self._horiz_spin_s = 0.0
                self._enter(NavState.SEARCH)
                return self._search_step(reading, dt)
            return self._drive(
                "search",
                -self.policy.speed,
                self.policy.speed,
                "after T: nudge right to align with the line",
            )
        if not self._t_turn_done and finishing_t:
            self._last_centroid = reading.centroid_x
            self.preferred_ground_u = reading.ground_u_px
            self._horiz_spin_s += dt
            aligned = (
                reading.axis == "vertical"
                and reading.error_fraction is not None
                and abs(reading.error_fraction) <= 0.12
                # A 15 mm stroke near the bumper reads ~86 px at 2028; a
                # centred vertical line at least this wide means the chassis
                # is aligned with the outer loop after the T spin.
                and reading.line_width_px >= 60
            )
            if aligned and self._horiz_spin_s >= 0.70 * spin_limit:
                self._t_turn_done = True
                self._horiz_spin_s = 0.0
            elif self._horiz_spin_s < spin_limit:
                return self._drive(
                    "search",
                    -self.policy.speed,
                    self.policy.speed,
                    "horizontal stroke: spin right to align with outer loop",
                )
            else:
                self._t_turn_done = True
                self._horiz_spin_s = 0.0
        elif not self._t_turn_done and near_t:
            self._last_centroid = reading.centroid_x
            self.preferred_ground_u = reading.ground_u_px
            if self._follow_ok_s < self.policy.right_turn_after_s:
                self._follow_ok_s += dt
                return self._drive(
                    "follow",
                    self.policy.speed,
                    self.policy.speed,
                    "stem: keep straight to the T",
                )
            if self._horiz_spin_s >= spin_limit:
                # Nominal 90-degree spin reached without visual alignment:
                # stop spinning; the post-turn handler will search.
                self._t_turn_done = True
                self._horiz_spin_s = 0.0
            else:
                self._horiz_spin_s += dt
                return self._drive(
                    "search",
                    -self.policy.speed,
                    self.policy.speed,
                    "horizontal stroke: spin right to align with outer loop",
                )
        elif reading.axis == "horizontal" and not self._t_turn_done:
            # Forward tilt: the T is already in view while wheels are on the
            # stem. A thin/high bar is look-ahead — drive straight to point 3.
            self._horiz_spin_s = 0.0
            self._follow_ok_s += dt
            if self._follow_ok_s >= self.policy.right_turn_after_s:
                self._last_centroid = reading.centroid_x
                self.preferred_ground_u = reading.ground_u_px
            return self._drive(
                "follow",
                self.policy.speed,
                self.policy.speed,
                "far crossing: keep straight to T",
            )
        elif not self._t_turn_done:
            self._horiz_spin_s = 0.0

        if self._too_thin(reading):
            self._junction_elapsed = 0.0
            self._roundabout_pending = False
            return self._drive("follow", 0, 0, "line too thin; hold")

        if self.policy.enable_roundabout and reading.junction and self._width_jumped(reading):
            self._junction_elapsed += dt
            if self._junction_elapsed >= self.policy.junction_min_s:
                # A persistent fork with a widened line is treated as a
                # roundabout entry. The car keeps following the main line
                # (continuous around the loop); only the exit decision changes.
                self._enter(NavState.ROUNDABOUT)
                self._roundabout_pending = True
                return self._roundabout_step(reading, dt)
        else:
            self._junction_elapsed = 0.0
            self._remember_baseline(reading)

        if (
            self._prev_error_fraction is not None
            and reading.error_fraction is not None
            and abs(reading.error_fraction - self._prev_error_fraction) > self.policy.max_error_jump
            and not self._right_turn_jump(reading)
        ):
            self._jump_elapsed += dt
            if self._jump_elapsed - dt > 0 and self._jump_elapsed >= self.policy.jump_search_s:
                self._enter(NavState.SEARCH)
                return self._search_step(reading, dt)
            return self._drive("follow", 0, 0, "jump: stop")

        self._jump_elapsed = 0.0
        self._follow_ok_s += dt
        self._last_centroid = reading.centroid_x
        self.preferred_ground_u = reading.ground_u_px
        if (
            self.policy.first_right_s > 0
            and not self._first_right_done
            and self._follow_ok_s >= self.policy.first_right_s
        ):
            self._enter(NavState.RIGHT_TURN)
            return self._right_turn_step(reading, 0.0)
        command = steer_command(reading, self.policy, self._prev_error_fraction)
        self._prev_error_fraction = reading.error_fraction
        return command

    def _remember_baseline(self, reading: LineReading) -> None:
        if reading.line_width_px <= 0 or reading.axis == "horizontal":
            return
        if self._baseline_widths:
            baseline = sorted(self._baseline_widths)[len(self._baseline_widths) // 2]
            if reading.line_width_px > self.policy.junction_width_factor * baseline:
                return
        self._baseline_widths.append(reading.line_width_px)

    def _width_jumped(self, reading: LineReading) -> bool:
        """True when the main line is markedly wider than its recent norm."""
        if not self._baseline_widths:
            return False
        baseline = sorted(self._baseline_widths)[len(self._baseline_widths) // 2]
        width_ok = reading.line_width_px > self.policy.junction_width_factor * baseline
        if not width_ok:
            return False
        # Row-count filtering of transient shadows happens in detect_line
        # (min_branch_rows_fraction). Here require a reported fork as well as
        # the width jump, so a merely fattened main line is not a junction.
        return reading.junction and len(reading.branch_centroids) >= 2

    def _right_spin_s(self) -> float:
        rate = self.policy.spin_deg_per_s_at_200 * (self.policy.speed / 200.0)
        return self.policy.first_right_deg / rate

    def _is_near_t(self, reading: LineReading) -> bool:
        """True when the chassis, not just the camera, has reached the T.

        The IMX500 sits forward and tilted down, so a 15 mm crossing is visible
        as a thin far bar long before the wheels arrive. Spin only when the
        bar is fat (near) and in the lower ROI (near the bumper).
        """
        if reading.axis != "horizontal" or reading.error_fraction is None:
            return False
        if abs(reading.error_fraction) > 0.15:
            return False
        if reading.line_width_px < self.policy.t_bar_min_width_px:
            return False
        y_top, y_bottom, _, _ = reading.roi
        if reading.centroid_y is not None and y_bottom > y_top:
            frac = (reading.centroid_y - y_top) / (y_bottom - y_top)
            if frac < self.policy.t_min_roi_y_fraction:
                return False
        return True

    def _right_turn_step(self, reading: LineReading, dt: float) -> NavCommand:
        if self._state_time >= self._right_spin_s():
            self._first_right_done = True
            self._enter(NavState.FOLLOW)
            return self._follow_step(reading, 0.0)
        return self._drive(
            "search",
            -self.policy.speed,
            self.policy.speed,
            f"first intersection: spin right {self._state_time:.1f}s",
        )

    # ------------------------------------------------------------ SEARCH
    def _search_step(self, reading: LineReading, dt: float) -> NavCommand:
        if reading.visible:
            reading = self._recenter(self._locked(reading))
            if reading.axis != "horizontal" and self._plausible_lock(reading):
                self._enter(NavState.FOLLOW)
                self._last_centroid = reading.centroid_x
                self.preferred_ground_u = reading.ground_u_px
                self._prev_error_fraction = reading.error_fraction
                self._jump_elapsed = 0.0
                return steer_command(reading, self.policy, self._prev_error_fraction)

        # Give up if maximum search time is exceeded
        if self.policy.search_give_up_s > 0 and self._state_time >= self.policy.search_give_up_s:
            return self._drive("search", 0, 0, f"search: give up after {self._state_time:.1f}s")

        # Step-by-step visual sweep search: oscillate left/right in small steps
        spin_speed = round(self.policy.speed * self.policy.search_spin_speed_ratio)
        spin_rate = self.policy.spin_deg_per_s_at_200 * (spin_speed / 200.0)

        step_time = self.policy.search_sweep_deg / spin_rate if spin_rate > 0 else 0.5
        cycle_time = 4 * step_time if step_time > 0 else 2.0

        if step_time <= 0 or spin_speed <= 0:
            return self._drive("search", 0, 0, "search: hold")

        t_mod = self._state_time % cycle_time
        if t_mod < step_time:
            # Step 1: Spin left
            left, right = -spin_speed, spin_speed
            dir_str = "left"
        elif t_mod < 3 * step_time:
            # Step 2: Spin right across center
            left, right = spin_speed, -spin_speed
            dir_str = "right"
        else:
            # Step 3: Spin left back to center
            left, right = -spin_speed, spin_speed
            dir_str = "left"

        return self._drive(
            "search",
            left,
            right,
            f"search: visual sweep {dir_str} ({self._state_time:.1f}/{self.policy.search_give_up_s:.1f}s)",
        )

    def _plausible_lock(self, reading: LineReading) -> bool:
        """Far-edge dark structure is not the track path in front of the wheels."""
        if reading.error_fraction is None or self._too_thin(reading):
            return False
        return abs(reading.error_fraction) <= self.policy.reacquire_error

    def _prefer_right(self, reading: LineReading) -> LineReading:
        """At a fork, lock the nearest branch to the right of the current line."""
        if (
            not self.policy.prefer_right_branch
            or not reading.junction
            or reading.centroid_x is None
            or not self._width_jumped(reading)
            or self._follow_ok_s < self.policy.right_turn_after_s
        ):
            return reading
        width = reading.roi[3]
        last = self._last_centroid if self._last_centroid is not None else reading.centroid_x
        ceiling = (
            self.policy.expected_center_fraction * width
            + self.policy.right_branch_max_offset * width
        )
        floor = last + 0.02 * width
        xs = [x for x in reading.branch_centroids if floor < x <= ceiling]
        if not xs:
            return reading
        chosen = min(xs)
        error_px = chosen - self.policy.expected_center_fraction * width
        return replace(
            reading,
            centroid_x=chosen,
            error_px=error_px,
            error_fraction=error_px / (width / 2),
        )

    def _right_turn_jump(self, reading: LineReading) -> bool:
        """A T-junction lock moving right is the planned first turn, not a glitch."""
        if (
            not reading.junction
            or reading.error_fraction is None
            or self._prev_error_fraction is None
        ):
            return False
        return reading.error_fraction > self._prev_error_fraction and reading.error_fraction <= 0.55

    def _too_thin(self, reading: LineReading) -> bool:
        if not reading.visible or not self._baseline_widths:
            return False
        if reading.axis == "horizontal":
            return False
        if (
            reading.error_fraction is not None
            and abs(reading.error_fraction) <= self.policy.steer_deadband
        ):
            return False
        baseline = sorted(self._baseline_widths)[len(self._baseline_widths) // 2]
        return reading.line_width_px < self.policy.min_width_ratio * baseline

    # -------------------------------------------------------- ROUNDABOUT
    def _roundabout_step(self, reading: LineReading, dt: float) -> NavCommand:
        if not reading.visible:
            self._junction_elapsed = 0.0
            self._enter(NavState.SEARCH)
            return self._search_step(reading, dt)

        reading = self._locked(reading)
        reading = self._recenter(reading)
        self._last_centroid = reading.centroid_x
        self.preferred_ground_u = reading.ground_u_px
        self._roundabout_elapsed += dt
        loop_done = self._roundabout_elapsed >= self.policy.roundabout_loop_min_s

        if reading.junction and loop_done:
            # Second fork after a full lap: this is the exit. Pick the branch
            # away from the line we came in on, i.e. the main centroid we are
            # still following is fine — exiting means keeping the line.
            reason = f"roundabout exit: lap {self._roundabout_elapsed:.1f}s >= min and fork seen"
            self._enter(NavState.FOLLOW)
            return steer_command(reading, self.policy, self._prev_error_fraction, reason)

        reason = (
            f"roundabout: lap {self._roundabout_elapsed:.1f}s / "
            f"{self.policy.roundabout_loop_min_s:.1f}s" + (" (fork)" if reading.junction else "")
        )
        return steer_command(reading, self.policy, self._prev_error_fraction, reason)

    # ------------------------------------------------------------- utils
    def _enter(self, state: NavState) -> None:
        self.state = state
        self._state_time = 0.0
        self._jump_elapsed = 0.0
        self._blind_creep_elapsed = 0.0

    def _drive(self, action: str, left: int, right: int, reason: str) -> NavCommand:
        return NavCommand(action=action, left=left, right=right, reason=reason, state=self.state)


def steer_command(
    reading: LineReading,
    policy: NavPolicy | None = None,
    _prev_error: float | None = None,
    reason: str | None = None,
) -> NavCommand:
    """Proportional steering on the line reading.

    Positive ``error_fraction`` (line right of centre) slows the right wheel;
    negative slows the left. Returns a ``follow`` command with the computed
    speeds. ``_prev_error`` is accepted for API symmetry with :class:`LineNav`
    but the first version is purely proportional — no integral or derivative
    term yet, so the argument is unused.
    """
    policy = policy or NavPolicy()
    err = reading.error_fraction
    if not reading.visible or err is None:
        return NavCommand("follow", 0, 0, reason or "no line", NavState.FOLLOW)

    if abs(err) <= policy.steer_deadband:
        return NavCommand(
            "follow",
            policy.speed,
            policy.speed,
            reason or f"follow: err={err:+.2f} deadband",
            NavState.FOLLOW,
        )

    raw_ratio = 1.0 - policy.turn_gain * abs(err)
    ratio = max(max(policy.min_ratio, 0.10), min(policy.max_ratio, raw_ratio))

    if err > 0:
        left, right = policy.speed, round(policy.speed * ratio)
    else:
        left, right = round(policy.speed * ratio), policy.speed

    return NavCommand(
        "follow",
        left,
        right,
        f"{reason or 'follow'}: err={err:+.2f} ratio={ratio:.2f}",
        NavState.FOLLOW,
    )
