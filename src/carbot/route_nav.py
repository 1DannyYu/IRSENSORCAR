"""Route-aware wrapper around the vision-driven ``carbot.line_nav.LineNav``.

The AI camera is the primary sensor: ``LineNav`` drives by what it sees
(follow the 2 cm line, spin at a detected horizontal crossing, lap the
roundabout on a persistent fork, search when the line drops). The planned
route (``carbot.route_plan``) is *advisory only* — it tells us which phase
the car is expected to be in so logs are readable and a blind-creep can be
steered in the planned direction. Wheel-speed timing is never used as the
authority: the chassis has no encoders and friction drifts, so the camera
decides.

``RouteTracker`` wraps a ``LineNav`` and keeps a phase index:

* the index advances when the nav performs a turn event (T right turn or
  roundabout exit), matching the route's turn steps;
* a straight step's expected remaining distance is estimated from time for
  logging only;
* when the nav is in search with the line lost past the blind-creep window,
  the tracker logs the planned next action so an operator can see whether
  the car is behaving as the plan expects.
"""

from __future__ import annotations

from dataclasses import dataclass

from carbot.line_nav import LineNav, NavState
from carbot.motion import DistanceIntegrator, MotionModel
from carbot.route_plan import RoutePlan, RouteStep, StepKind


@dataclass
class RouteStatus:
    """What the route tracker knows about the current phase."""

    step_index: int
    step_label: str
    step_kind: str
    expected_remaining_m: float
    nav_state: str
    message: str


class RouteTracker:
    """Tracks the planned phase around a vision-driven ``LineNav``."""

    def __init__(
        self,
        plan: RoutePlan,
        nav: LineNav,
        motion: MotionModel | None = None,
    ) -> None:
        self._plan = plan
        self._nav = nav
        self._motion = motion or MotionModel()
        self._integrator = DistanceIntegrator(self._motion)
        self._step_idx = 0
        self._step_elapsed_s = 0.0
        self._turns_seen = 0
        self._last_nav_state: NavState | None = None
        self._last_command = None

    # ------------------------------------------------------------- state
    @property
    def step_index(self) -> int:
        return self._step_idx

    @property
    def current_step(self) -> RouteStep | None:
        if self._step_idx >= len(self._plan):
            return None
        return self._plan.steps[self._step_idx]

    @property
    def nav(self) -> LineNav:
        return self._nav

    @property
    def last_command(self):
        return self._last_command

    def _advance_step(self, *, steps: int = 1) -> None:
        self._step_idx = min(self._step_idx + steps, len(self._plan))
        self._step_elapsed_s = 0.0

    def step(self, reading, dt: float) -> RouteStatus:
        """Feed one frame; returns the tracked phase status (does not drive)."""
        command = self._nav.step(reading, dt)
        self._last_command = command
        self._step_elapsed_s += dt

        state = self._nav.state
        if state is NavState.RIGHT_TURN and self._last_nav_state is not NavState.RIGHT_TURN:
            # Entering a T right turn: the straight phase before it and the
            # turn step are both consumed, landing on the next straight phase.
            self._advance_step(steps=2)
        if state is NavState.ROUNDABOUT and self._last_nav_state is not NavState.ROUNDABOUT:
            # Entering the roundabout: consume up to the roundabout step.
            while self.current_step is not None and self.current_step.kind != StepKind.ROUNDABOUT:
                self._advance_step()
        if state is NavState.FOLLOW and self._last_nav_state is NavState.ROUNDABOUT:
            # Exiting the roundabout: consume the roundabout step and the
            # exit turn, landing on the return straight phase.
            self._advance_step(steps=2)
        self._last_nav_state = state

        step = self.current_step
        if step is None:
            return RouteStatus(
                self._step_idx, "route complete", "", 0.0, state.value, command.reason
            )

        remaining = 0.0
        if step.kind == StepKind.STRAIGHT:
            travelled = self._integrator.distance_delta(self._step_elapsed_s)
            remaining = max(0.0, step.distance_m - travelled)
        elif step.kind == StepKind.ROUNDABOUT:
            remaining = max(
                0.0, step.distance_m - self._integrator.distance_delta(self._step_elapsed_s)
            )

        planned = {
            StepKind.STRAIGHT: f"straight {remaining:.2f} m left (plan)",
            StepKind.TURN_LEFT: "left turn next (plan)",
            StepKind.TURN_RIGHT: "right turn next (plan)",
            StepKind.ROUNDABOUT: f"roundabout {remaining:.2f} m left (plan)",
        }.get(step.kind, step.kind)

        return RouteStatus(
            self._step_idx,
            step.label,
            step.kind,
            remaining,
            state.value,
            f"{planned} | nav: {state.value}:{command.action} L{command.left} R{command.right} | {command.reason}",
        )
