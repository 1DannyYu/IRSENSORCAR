"""Time-based motion model for the chassis (no Hall encoders on this build).

``config.HAS_ENCODERS`` is False for this car: the two-wire DC motors have no
feedback, so dead reckoning falls back to a calibrated time model. The three
constants below were measured on the real chassis and live in
``docs/handoff-2026-08-16-line-follow.md``; the calibration example
``examples/30_cam_motion_calibrate.py`` can refine them for the current battery.

    forward @ 200  ~= 0.104 m/s (measured 2026-08-16)
    forward @ 150  ~= 0.088 m/s
    spin @ 200     ~= 53.5 deg/s
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class MotionModel:
    """Speed calibration constants for one base speed."""

    speed: int = 200  # motor speed the constants were measured at
    forward_mps: float = 0.104  # measured 2026-08-16: 10.4 cm in 1.0 s at speed 200
    spin_degps: float = 53.5  # in-place yaw rate at ``speed``

    def seconds_for_distance(self, distance_m: float) -> float:
        """Time to drive ``distance_m`` at the calibrated speed."""
        if self.forward_mps <= 0:
            raise ValueError("forward_mps must be positive")
        return distance_m / self.forward_mps

    def seconds_for_angle(self, angle_deg: float) -> float:
        """Time to spin ``angle_deg`` in place at the calibrated speed."""
        if self.spin_degps <= 0:
            raise ValueError("spin_degps must be positive")
        return abs(angle_deg) / self.spin_degps


class DistanceIntegrator:
    """Accumulates travelled distance from wall-clock time under a motion model.

    The caller feeds it ``dt`` every frame; it reports how much of the
    current step's distance budget is consumed. Turn steps accumulate angle
    instead. This keeps the controller free of direct ``time.time()`` calls
    so it stays unit-testable with fake clocks.
    """

    def __init__(self, model: MotionModel) -> None:
        self.model = model

    def distance_delta(self, dt: float) -> float:
        return dt * self.model.forward_mps

    def angle_delta(self, dt: float) -> float:
        return dt * self.model.spin_degps
