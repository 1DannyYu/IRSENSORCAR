"""Task-1 route plan for the IR-tracking-sensor system: two-lap phase list.

Car-level phase distances/angles, in the same ``RouteStep``/``RoutePlan`` shape as
``carbot.route_plan`` (the camera system's plan) but sourced only from the current
``docs/task1-single-source-of-truth.md`` (2026-08-19 revision, continuous-loop
correction). ``carbot.route_plan`` still encodes the retired single-lap "Phase 11
return stem" and must not be used as a reference for this module.

    Phase  1 stem 10.0 -> T right 90 -> [lap 1] Phase 2 east 16.0
        -> ARC1 left 90 -> Phase 4 north 19.2 -> ARC2 left 90
        -> Phase 6 west 58.5 -> ARC3 left 90 -> Phase 8 entry 7.5
        -> Phase 9 roundabout 3/4 left (84.8 cm, 270 deg)
        -> exit right 90 -> Phase 10 return 23.0
        -> CROSS the T junction straight through (no turn) -> [lap 2] Phase 2 east 16.0
        -> ... -> Phase 10 return 23.0 -> STOP at the T junction

There is no Phase 11: the car never returns to the start box. Phase 1 (the start-box
stem) runs once, before lap 1; every later lap starts straight at Phase 2.

This module is a planning/logging artifact, mirroring the role ``carbot.route_plan``
plays for the camera-based ``carbot.route_nav``. It does not drive the IR-sensor car:
that job belongs to ``carbot.ir_route`` (``task1_route_for_laps``) and
``carbot.ir_line_nav.IRLineNav``, whose junction-sequence + distance-gate design is
already verified for exactly this two-lap behaviour (see ``tests/test_ir_route.py``).
Keeping the actual navigation on one source avoids the 2026-08-19 failure class this
project already hit once: two structures describing the same lap that can drift apart.
"""

from __future__ import annotations

from dataclasses import dataclass


class StepKind:
    """Step kinds for this plan. ``CROSS`` and ``STOP`` mirror
    ``carbot.ir_route.JunctionAction`` (straight through a junction / halt the route)."""

    STRAIGHT = "straight"  # drive straight
    TURN_LEFT = "turn_left"  # spin left in place to a heading
    TURN_RIGHT = "turn_right"  # spin right in place to a heading
    ROUNDABOUT = "roundabout"  # left arc lap, finish on accumulated angle
    CROSS = "cross"  # keep going straight through a junction; no heading change
    STOP = "stop"  # the planned lap count ends here; halt


@dataclass(frozen=True)
class RouteStep:
    """One planned manoeuvre.

    ``distance_m`` is the distance to drive (STRAIGHT / ROUNDABOUT / CROSS) and
    ``angle_deg`` the heading change (TURN_* / ROUNDABOUT). ROUNDABOUT uses
    ``distance_m`` only for the log; completion is judged on ``angle_deg``.
    """

    kind: str
    distance_m: float = 0.0
    angle_deg: float = 0.0
    label: str = ""


@dataclass(frozen=True)
class RoutePlan:
    """An ordered list of steps plus the chassis pose at the start."""

    name: str
    steps: tuple[RouteStep, ...]

    def __len__(self) -> int:
        return len(self.steps)

    def __iter__(self):
        return iter(self.steps)


#: One lap's phases, Phase 2 through Phase 10, as (distance_m, label) pairs for
#: STRAIGHT/CROSS steps and the turn events between them. Source: SSOT section 3
#: (2026-08-19 revision), the 1000x700 px / 100x70 cm printed map — 10 px = 1 cm,
#: no reprint scaling.
_LAP_STRAIGHTS_M = {
    "Phase 2 east": 0.160,
    "Phase 4 north": 0.192,
    "Phase 6 west": 0.585,
    "Phase 8 entry": 0.075,
    "Phase 10 return": 0.230,
}
_ROUNDABOUT_M = 0.848
_ROUNDABOUT_DEG = 270.0


def _lap_steps(lap_no: int, *, closing: str) -> tuple[RouteStep, ...]:
    """Phase 2..10 for one lap. ``closing`` is the kind ("cross" or "stop") of the
    step that ends the lap at the T junction — the two ways a lap can finish."""
    suffix = f" (lap {lap_no})"
    return (
        RouteStep(StepKind.STRAIGHT, _LAP_STRAIGHTS_M["Phase 2 east"], 0.0, "Phase 2 east" + suffix),
        RouteStep(StepKind.TURN_LEFT, 0.0, 90.0, "ARC 1 SE corner" + suffix),
        RouteStep(StepKind.STRAIGHT, _LAP_STRAIGHTS_M["Phase 4 north"], 0.0, "Phase 4 north" + suffix),
        RouteStep(StepKind.TURN_LEFT, 0.0, 90.0, "ARC 2 NE corner" + suffix),
        RouteStep(StepKind.STRAIGHT, _LAP_STRAIGHTS_M["Phase 6 west"], 0.0, "Phase 6 west" + suffix),
        RouteStep(StepKind.TURN_LEFT, 0.0, 90.0, "ARC 3 NW corner" + suffix),
        RouteStep(StepKind.STRAIGHT, _LAP_STRAIGHTS_M["Phase 8 entry"], 0.0, "Phase 8 entry" + suffix),
        RouteStep(StepKind.ROUNDABOUT, _ROUNDABOUT_M, _ROUNDABOUT_DEG, "Phase 9 roundabout" + suffix),
        RouteStep(StepKind.TURN_RIGHT, 0.0, 90.0, "roundabout exit" + suffix),
        RouteStep(
            StepKind.STRAIGHT,
            _LAP_STRAIGHTS_M["Phase 10 return"],
            0.0,
            "Phase 10 return" + suffix,
        ),
        RouteStep(
            StepKind.STOP if closing == "stop" else StepKind.CROSS,
            0.0,
            0.0,
            "T junction: halt, task complete" if closing == "stop"
            else "T junction: cross straight through into next lap",
        ),
    )


def task1_route_for_laps(laps: int) -> RoutePlan:
    """The Task-1 lap driven ``laps`` times, halting at the T junction that closes
    the last lap. Phase 1 (the start-box stem) runs once, before lap 1; every lap
    after the first starts directly at Phase 2, reached by crossing the T junction
    straight through rather than turning into the stem.
    """
    if laps < 1:
        raise ValueError("laps must be at least 1")
    steps: list[RouteStep] = [
        RouteStep(StepKind.STRAIGHT, 0.10, 0.0, "Phase 1 stem"),
        RouteStep(StepKind.TURN_RIGHT, 0.0, 90.0, "T junction (start stem)"),
    ]
    for lap_no in range(1, laps + 1):
        closing = "stop" if lap_no == laps else "cross"
        steps.extend(_lap_steps(lap_no, closing=closing))
    return RoutePlan(name=f"task1-ir-{laps}lap", steps=tuple(steps))


def task1_route_two_laps() -> RoutePlan:
    """Convenience wrapper for the two-lap plan asked for on the track."""
    return task1_route_for_laps(2)


def total_distance_m(plan: RoutePlan) -> float:
    """Sum of the planned distances (straight + roundabout arcs)."""
    return sum(s.distance_m for s in plan.steps)
