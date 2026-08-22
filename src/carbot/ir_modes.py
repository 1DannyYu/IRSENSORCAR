"""Pure command policy for the three operator-selected IR driving modes."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from carbot.ir_geometry import IRState, Kind, resolve_blind, wheel_speeds

LEFT_CORRECTION_RATIO_SCALE = 0.0
CIRCLE_MODE_START_S = 22.0
SEARCH_SWEEP_ANGLES_DEG = (5.0, 20.0, 45.0)
SEARCH_REPLAY_S = 2.0
SPIN_RATE_DEG_PER_S = 39.7
SPIN_DEAD_TIME_S = 0.41
ROUNDABOUT_ENTRY_TURN_DEG = 42.5
ROUNDABOUT_EXIT_SEQUENCE = (
    (0, 1, 1, 1),
    (0, 1, 0, 1),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
)


class DriveMode(str, Enum):
    AUTO_TRACING = "auto-tracing"
    PHASE1_TO_PHASE2 = "phase1-to-phase2"
    CIRCLE = "circle"
    CHAINED = "chained"


class CirclePhase(str, Enum):
    WAITING = "waiting-for-entry"
    INSIDE = "inside-roundabout"
    EXITED = "exited-roundabout"


@dataclass
class CircleModeState:
    phase: CirclePhase = CirclePhase.WAITING
    exit_sequence_index: int = 0

    def observe(self, *, elapsed_s: float, bits: tuple[int, int, int, int]) -> str | None:
        """Return ``enter`` at the timed boundary, or ``exit`` on its sequence."""
        if self.phase is CirclePhase.WAITING:
            if elapsed_s < CIRCLE_MODE_START_S:
                return None
            self.phase = CirclePhase.INSIDE
            self.exit_sequence_index = 0
            return "enter"

        if self.phase is CirclePhase.EXITED:
            return None

        expected = ROUNDABOUT_EXIT_SEQUENCE[self.exit_sequence_index]
        if bits == expected:
            self.exit_sequence_index += 1
            if self.exit_sequence_index == len(ROUNDABOUT_EXIT_SEQUENCE):
                self.phase = CirclePhase.EXITED
                return "exit"
            return None

        if bits == ROUNDABOUT_EXIT_SEQUENCE[0]:
            self.exit_sequence_index = 1
        elif bits != expected:
            self.exit_sequence_index = 0
        return None


@dataclass(frozen=True)
class ModeCommand:
    left: int
    right: int
    reason: str


def _forward(speed: int, reason: str) -> ModeCommand:
    return ModeCommand(speed, speed, reason)


def _left_from_state(state: IRState, speed: int, reason: str) -> ModeCommand:
    if state.inner_ratio < 0:
        return ModeCommand(-speed, speed, reason)
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
        if state.direction < 0:
            return _left_from_state(state, speed, f"{state.label}; stronger left curve correction")
        return _forward(speed, f"{state.label}; no right turns in auto-tracing")
    return _forward(speed, "centred or right drift; forward-only policy")


def enter_roundabout_command(state: IRState, *, speed: int) -> ModeCommand:
    """Use the entry table's right-turn action for every sensor state.

    Entry is deliberately biased toward the roundabout: even readings that
    normally produce a left correction are commanded as a right pivot.  The
    confirmed entry sequence still decides when the open-loop entry turn is
    completed.
    """
    return ModeCommand(speed, -speed, f"{state.label}; enter-roundabout right turn")


def phase1_to_phase2_timing() -> tuple[float, float]:
    """Return calibrated seconds for 17 cm forward followed by a 90-degree right spin."""
    forward_s = 22.0 / 10.0
    turn_s = 0.41 + 90.0 / 39.7
    return forward_s, turn_s


def roundabout_entry_turn_s() -> float:
    """Return the calibrated open-loop time for the documented 42.5-degree entry turn."""
    return 0.41 + ROUNDABOUT_ENTRY_TURN_DEG / 39.7


def search_sweep_turn_s(angle_deg: float) -> float:
    """Return the calibrated open-loop time for one search pivot."""
    if angle_deg <= 0:
        raise ValueError("search angle must be positive")
    return SPIN_DEAD_TIME_S + angle_deg / SPIN_RATE_DEG_PER_S


def line_search_required(
    state: IRState, previous_localising: tuple[int, int, int, int] | None
) -> bool:
    """Return true only for P0000 that resolves to genuine line loss."""
    if state.kind is not Kind.AMBIGUOUS:
        return False
    verdict, _ = resolve_blind(previous_localising)
    return verdict == "lost"
