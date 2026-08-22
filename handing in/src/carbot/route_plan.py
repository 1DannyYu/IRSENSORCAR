"""Task-1 route plan: the operator-approved 11-phase track as car-level steps.

The plan is expressed in car coordinates — straight distances to drive and
spin angles to turn — derived from the measured orthophoto geometry in
``docs/task1-single-source-of-truth.md`` (corrected v2, 2026-08-16):

    Phase  1 stem 10.0 cm -> T right 90 deg -> Phase 2 east 16.0 cm
        -> ARC1 left 90 -> Phase 4 north 19.2 cm -> ARC2 left 90
        -> Phase 6 west 58.5 cm -> ARC3 left 90 -> Phase 8 entry 7.5 cm
        -> Phase 9 roundabout 3/4 left (84.8 cm, ~270 deg)
        -> exit right 90 -> Phase 10 return 23.0 cm -> T right 90
        -> Phase 11 stem 10.0 cm

The roundabout step keeps both the arc distance and the accumulated spin
angle; the controller finishes the step on whichever target it trusts first
(angle is the robust one for a blind lap).
"""

from __future__ import annotations

from dataclasses import dataclass


class StepKind:
    """Step kinds understood by :class:`carbot.route_nav.RouteNav`."""

    STRAIGHT = "straight"  # drive straight, vision closes the loop
    TURN_LEFT = "turn_left"  # spin left in place to a heading
    TURN_RIGHT = "turn_right"  # spin right in place to a heading
    ROUNDABOUT = "roundabout"  # left arc lap, finish on accumulated angle


@dataclass(frozen=True)
class RouteStep:
    """One planned manoeuvre.

    ``distance_m`` is the distance to drive (STRAIGHT / ROUNDABOUT) and
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


#: The reprint map is 840x588 mm = the original 1000x700 mm Task-1 map scaled
#: by 0.84. Every straight/arc distance scales with the map; the 15 mm black
#: line and the 20 mm AprilTags keep their physical size (see
#: scripts/generate_task1_map.py). Turn angles are unaffected by the scale.
MAP_SCALE = 0.84


def task1_route() -> RoutePlan:
    """The full Task-1 lap: start zone -> outer loop -> roundabout -> return.

    Distances are the corrected-v2 SSOT values multiplied by ``MAP_SCALE``
    so the plan matches the reprint map's physical size.
    """
    s = MAP_SCALE
    return RoutePlan(
        name="task1",
        steps=(
            RouteStep(StepKind.STRAIGHT, round(0.10 * s, 4), 0.0, "Phase 1 stem"),
            RouteStep(StepKind.TURN_RIGHT, 0.0, 90.0, "T junction right"),
            RouteStep(StepKind.STRAIGHT, round(0.16 * s, 4), 0.0, "Phase 2 east"),
            RouteStep(StepKind.TURN_LEFT, 0.0, 90.0, "ARC 1 SE corner"),
            RouteStep(StepKind.STRAIGHT, round(0.19 * s, 4), 0.0, "Phase 4 north"),
            RouteStep(StepKind.TURN_LEFT, 0.0, 90.0, "ARC 2 NE corner"),
            RouteStep(StepKind.STRAIGHT, round(0.585 * s, 4), 0.0, "Phase 6 west"),
            RouteStep(StepKind.TURN_LEFT, 0.0, 90.0, "ARC 3 NW corner"),
            RouteStep(StepKind.STRAIGHT, round(0.075 * s, 4), 0.0, "Phase 8 entry"),
            RouteStep(StepKind.ROUNDABOUT, round(0.848 * s, 4), 270.0, "Phase 9 roundabout"),
            RouteStep(StepKind.TURN_RIGHT, 0.0, 90.0, "exit 3 right"),
            RouteStep(StepKind.STRAIGHT, round(0.23 * s, 4), 0.0, "Phase 10 return"),
            RouteStep(StepKind.TURN_RIGHT, 0.0, 90.0, "T junction right"),
            RouteStep(StepKind.STRAIGHT, round(0.10 * s, 4), 0.0, "Phase 11 stem back"),
        ),
    )


def total_distance_m(plan: RoutePlan) -> float:
    """Sum of the planned distances (straight + roundabout arcs)."""
    return sum(s.distance_m for s in plan.steps)
