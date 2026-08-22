"""Pure command policy for the three operator-selected IR driving modes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from carbot.ir_geometry import IRState, Kind, resolve_blind, wheel_speeds

LEFT_CORRECTION_RATIO_SCALE = 0.60


class DriveMode(str, Enum):
    AUTO_TRACING = "auto-tracing"
    PHASE1_TO_PHASE2 = "phase1-to-phase2"
    CIRCLE = "circle"
    CHAINED = "chained"


@dataclass(frozen=True)
class ModeCommand:
    left: int
    right: int
    reason: str


def _forward(speed: int, reason: str) -> ModeCommand:
    return ModeCommand(speed, speed, reason)


def _left_from_state(state: IRState, speed: int, reason: str) -> ModeCommand:
    inner_ratio = max(0.0, min(1.0, state.inner_ratio * LEFT_CORRECTION_RATIO_SCALE))
    left, right = wheel_speeds(speed, -1, inner_ratio)
    return ModeCommand(left, right, reason)


def auto_tracing_command(
    state: IRState,
    *,
    speed: int,
    previous_command: ModeCommand | None = None,
    previous_localising: tuple[int, int, int, int] | None = None,
) -> ModeCommand:
    """Follow the 16-state table with a forward-or-left-only motion policy.

    Right-side drift readings are deliberately treated as forward commands. This policy never
    emits a right correction, right pivot, or reverse command.
    """
    if state.kind is Kind.DRIFT and state.direction < 0:
        return _left_from_state(state, speed, f"{state.label}; left correction")
    if state.kind is Kind.AMBIGUOUS:
        verdict, offset = resolve_blind(previous_localising)
        if verdict == "blind" and offset is not None and offset < 0:
            return _left_from_state(state, speed, "blind band after left drift; stronger left correction")
        if verdict == "hold" and previous_command is not None:
            return previous_command
        return _forward(speed, f"{state.label}; forward-only policy")
    if state.kind is Kind.NOISE:
        return _forward(speed, f"{state.label}; forward-only policy")
    if state.kind is Kind.JUNCTION:
        return _forward(speed, f"{state.label}; no right turns in auto-tracing")
    return _forward(speed, "centred or right drift; forward-only policy")


def circle_triggered(*, elapsed_s: float, bits: tuple[int, int, int, int], entered: bool) -> bool:
    """Return true once circle mode is active and P1110 is seen for the first time."""
    return elapsed_s >= 46.0 and not entered and bits == (1, 1, 1, 0)


def phase1_to_phase2_timing() -> tuple[float, float]:
    """Return calibrated seconds for 17 cm forward followed by a 90-degree right spin."""
    forward_s = 17.0 / 10.0
    turn_s = 0.41 + 90.0 / 39.7
    return forward_s, turn_s
