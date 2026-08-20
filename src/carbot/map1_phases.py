"""Single source of truth for Map 1's ten physical route phases.

The route used to be represented only indirectly through junction names, arc heuristics, and
distance windows.  That made a log say which junction was pending without saying which physical
phase the car was actually attempting.  This module gives the route an explicit, testable phase
model shared by the hardware runners and the navigation code.

Distances remain open-loop estimates because the chassis has no wheel encoders.  They are useful
for bounded tests and coarse gating, not proof of physical position.  Callers must estimate travel
from the actual wheel command rather than adding the full-speed rate on every control cycle.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class Map1PhaseKind(Enum):
    """Physical shape/control mode of one route phase."""

    OPEN_LOOP = "open_loop"
    STRAIGHT = "straight"
    ARC = "arc"
    ROUNDABOUT = "roundabout"


@dataclass(frozen=True)
class Map1PhaseSpec:
    """Stable physical specification for one numbered Map 1 phase."""

    number: int
    name: str
    kind: Map1PhaseKind
    distance_cm: float
    entry_heading: str
    exit_heading: str
    instruction: str
    # -1 = left corrections only, +1 = right corrections only, 0 = unrestricted.
    # Used only by bounded phase tests whose curve direction is known from the map.
    steering_direction_limit: int = 0

    def __post_init__(self) -> None:
        if self.steering_direction_limit not in (-1, 0, 1):
            raise ValueError("steering_direction_limit must be -1, 0, or 1")


# A 2026-08-21 instrumented run proved P0000 did not stop the command. The apparent 7cm / 1.6s
# result was invalid because the underside caught on raised paper. After flattening the paper, an
# unobstructed 1.6s retest still fell short, so the operator requested a bounded 2.2s trial.
PHASE1_FORWARD_CM = 16.0
PHASE1_FORWARD_S = 2.2
PHASE1_FORWARD_PWM = 150
PHASE1_SPIN_PWM = 150
PHASE1_RIGHT_TURN_DEG = 90.0
PHASE1_RIGHT_TURN_COMPENSATION_DEG = 5.0
PHASE1_RIGHT_TURN_PULSE_DEG = PHASE1_RIGHT_TURN_DEG + PHASE1_RIGHT_TURN_COMPENSATION_DEG
# The 2.55s / 42deg/s model produced 85 degrees in the same physical run.  Keeping the measured
# 0.41s dead time gives an observed moving rate of about 39.7deg/s. A later test still turned short,
# so the 90-degree route target now receives a 5-degree pulse compensation (about 2.80s total).
PHASE1_SPIN_RATE_DEG_PER_S = 39.7
PHASE1_SPIN_DEAD_TIME_S = 0.41
PROVISIONAL_FORWARD_SPEED_CM_S = PHASE1_FORWARD_CM / PHASE1_FORWARD_S

# The 07:00 ARC 1 test used continuous left-curve feed-forward and stopped safely on the line, but
# x0.50 still ended at a northeast heading: about half of the required east-to-north turn. Credit a
# quarter of the straight-derived command distance so the bounded test runs roughly twice as long.
# The earlier x0.25 off-map trial predated continuous curve feed-forward and is not comparable.
ARC_TEST_DISTANCE_CREDIT_SCALE = 0.25

# A bounded ARC contains no junction, so sustained P1111 is carpet/off-paper rather than a
# legitimate crossbar. React before driving two full seconds deeper onto the carpet, and retain
# enough history to reverse past the edge. Reverse still ends early on a valid line reacquisition.
ARC_TEST_OFF_TRACK_DWELL_S = 0.3
ARC_TEST_REVERSE_REPLAY_WINDOW_S = 3.0

# 270 degrees of the measured 33.5cm inner black-line diameter.
ROUNDABOUT_270_ARC_CM = 3.141592653589793 * 33.5 * 270.0 / 360.0


MAP1_PHASES: tuple[Map1PhaseSpec, ...] = (
    Map1PhaseSpec(
        1,
        "Start stem",
        Map1PhaseKind.OPEN_LOOP,
        PHASE1_FORWARD_CM,
        "north",
        "east",
        "Ignore sensors, drive forward for the calibrated pulse, then spin right 90 degrees.",
    ),
    Map1PhaseSpec(
        2,
        "East straight",
        Map1PhaseKind.STRAIGHT,
        15.5,
        "east",
        "east",
        "Acquire the 2cm black line and follow it east.",
    ),
    Map1PhaseSpec(
        3,
        "ARC 1",
        Map1PhaseKind.ARC,
        12.0,
        "east",
        "north",
        "Follow ARC 1 continuously.",
        steering_direction_limit=-1,
    ),
    Map1PhaseSpec(
        4,
        "North straight",
        Map1PhaseKind.STRAIGHT,
        18.0,
        "north",
        "north",
        "Follow the 2cm black line north.",
    ),
    Map1PhaseSpec(
        5,
        "ARC 2",
        Map1PhaseKind.ARC,
        12.0,
        "north",
        "west",
        "Follow ARC 2 continuously.",
        steering_direction_limit=-1,
    ),
    Map1PhaseSpec(
        6,
        "West straight",
        Map1PhaseKind.STRAIGHT,
        47.0,
        "west",
        "west",
        "Follow the 2cm black line west.",
    ),
    Map1PhaseSpec(
        7,
        "ARC 3",
        Map1PhaseKind.ARC,
        12.0,
        "west",
        "southwest",
        "Follow ARC 3 continuously toward the roundabout.",
        steering_direction_limit=-1,
    ),
    Map1PhaseSpec(
        8,
        "Roundabout approach",
        Map1PhaseKind.STRAIGHT,
        7.5,
        "southwest",
        "southwest",
        "Follow the short approach that blends into the roundabout.",
    ),
    Map1PhaseSpec(
        9,
        "Roundabout 270 degrees",
        Map1PhaseKind.ROUNDABOUT,
        ROUNDABOUT_270_ARC_CM,
        "southwest",
        "west",
        "Follow the roundabout for 270 degrees and take the mapped exit.",
    ),
    Map1PhaseSpec(
        10,
        "Final west straight",
        Map1PhaseKind.STRAIGHT,
        21.5,
        "west",
        "west",
        "Follow west and stop at the final T junction.",
    ),
)


def map1_phase(number: int) -> Map1PhaseSpec:
    """Return one phase by its one-based number."""
    if not 1 <= number <= len(MAP1_PHASES):
        raise ValueError(f"phase must be in [1, {len(MAP1_PHASES)}]")
    return MAP1_PHASES[number - 1]


def phase_start_cm(number: int) -> float:
    """Planned cumulative distance before ``number`` begins."""
    map1_phase(number)  # validate before slicing
    return sum(phase.distance_cm for phase in MAP1_PHASES[: number - 1])


def estimate_forward_distance_cm(
    *,
    dt: float,
    left: int,
    right: int,
    reference_pwm: int,
    reference_speed_cm_s: float,
) -> float:
    """Estimate chassis-centre forward travel from the wheel command.

    Equal-and-opposite wheel commands are an in-place pivot and therefore add zero.  Reduced or
    asymmetric positive commands are scaled by their signed mean instead of receiving full-speed
    distance credit.  Reverse travel is deliberately not counted as planned forward progress.
    """
    if dt < 0:
        raise ValueError("dt must be non-negative")
    if reference_pwm <= 0:
        raise ValueError("reference_pwm must be positive")
    if reference_speed_cm_s <= 0:
        raise ValueError("reference_speed_cm_s must be positive")
    mean_pwm = (left + right) / 2.0
    if mean_pwm <= 0:
        return 0.0
    return dt * reference_speed_cm_s * mean_pwm / reference_pwm


@dataclass(frozen=True)
class PhaseTransition:
    """One estimated phase boundary crossed by :class:`Map1PhaseProgress`."""

    completed: Map1PhaseSpec
    current: Map1PhaseSpec | None


class Map1PhaseProgress:
    """Command-aware, open-loop progress tracker for bounded phase tests and logs."""

    def __init__(self, start_phase: int = 2) -> None:
        self._index = map1_phase(start_phase).number - 1
        self.phase_cm = 0.0
        self.total_cm = phase_start_cm(start_phase)

    @property
    def current(self) -> Map1PhaseSpec | None:
        if self._index >= len(MAP1_PHASES):
            return None
        return MAP1_PHASES[self._index]

    def advance_cm(self, distance_cm: float) -> tuple[PhaseTransition, ...]:
        """Advance by non-negative estimated distance and report every crossed boundary."""
        if distance_cm < 0:
            raise ValueError("distance_cm must be non-negative")
        transitions: list[PhaseTransition] = []
        remaining = distance_cm
        while remaining > 0 and self.current is not None:
            current = self.current
            to_boundary = max(0.0, current.distance_cm - self.phase_cm)
            travelled = min(remaining, to_boundary)
            self.phase_cm += travelled
            self.total_cm += travelled
            remaining -= travelled
            if self.phase_cm + 1e-9 < current.distance_cm:
                break
            self._index += 1
            self.phase_cm = 0.0
            transitions.append(PhaseTransition(current, self.current))
        return tuple(transitions)

    def observe_command(
        self,
        *,
        dt: float,
        left: int,
        right: int,
        reference_pwm: int,
        reference_speed_cm_s: float,
    ) -> tuple[PhaseTransition, ...]:
        """Estimate and apply progress from one issued wheel command."""
        distance = estimate_forward_distance_cm(
            dt=dt,
            left=left,
            right=right,
            reference_pwm=reference_pwm,
            reference_speed_cm_s=reference_speed_cm_s,
        )
        return self.advance_cm(distance)
