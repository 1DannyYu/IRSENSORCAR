"""Pure command policy for the three operator-selected IR driving modes."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

from carbot.ir_geometry import IRState, Kind, resolve_blind, wheel_speeds

LEFT_CORRECTION_RATIO_SCALE = 0.0
CIRCLE_MODE_START_S = 25.6
SEARCH_SWEEP_ANGLES_DEG = (5.0, 20.0, 45.0)
SEARCH_REPLAY_S = 2.0
LINE_LOSS_CONFIRM_S = 1.0
ROUNDABOUT_P1001_HOLD_S = 0.1
ROUNDABOUT_P1001_FORWARD_CM = 5.0
ROUNDABOUT_P1001_TURN_DEG = 50.0
ROUNDABOUT_POST_EXIT_P0111_HOLD_S = 0.2
ROUNDABOUT_POST_EXIT_P0111_FORWARD_CM = 5.0
P0111_STOP_START_S = 45.0
ROUNDABOUT_ENTRY_PAIR_WINDOW_S = 1.0
SPIN_RATE_DEG_PER_S = 39.7
SPIN_DEAD_TIME_S = 0.41
ROUNDABOUT_ENTRY_TRIGGERS = {(1, 1, 1, 0), (1, 1, 1, 1)}
ROUNDABOUT_ENTRY_TURN_DEG = 42.5


class DriveMode(str, Enum):
    AUTO_TRACING_2_6 = "auto-tracing-2-6"
    AUTO_TRACING_AFTER_EXIT = "auto-tracing-after-exit"
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
    entry_window_started_s: float | None = None
    entry_window_bits: set[tuple[int, int, int, int]] = field(default_factory=set)

    def observe(self, *, elapsed_s: float, bits: tuple[int, int, int, int]) -> str | None:
        """Enter after 25.6s on one trigger or both triggers within one second."""
        if self.phase is CirclePhase.WAITING:
            if bits in ROUNDABOUT_ENTRY_TRIGGERS:
                if (
                    self.entry_window_started_s is None
                    or elapsed_s - self.entry_window_started_s > ROUNDABOUT_ENTRY_PAIR_WINDOW_S
                ):
                    self.entry_window_started_s = elapsed_s
                    self.entry_window_bits.clear()
                self.entry_window_bits.add(bits)
            elif (
                self.entry_window_started_s is not None
                and elapsed_s - self.entry_window_started_s > ROUNDABOUT_ENTRY_PAIR_WINDOW_S
            ):
                self.entry_window_started_s = None
                self.entry_window_bits.clear()

            pair_triggered = self.entry_window_bits == ROUNDABOUT_ENTRY_TRIGGERS
            timed_triggered = elapsed_s >= CIRCLE_MODE_START_S and bits in ROUNDABOUT_ENTRY_TRIGGERS
            if not pair_triggered and not timed_triggered:
                return None
            self.phase = CirclePhase.INSIDE
            return "enter"

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


def auto_tracing_original_command(
    state: IRState,
    *,
    speed: int,
    previous_command: ModeCommand | None = None,
    previous_localising: tuple[int, int, int, int] | None = None,
) -> ModeCommand:
    """Follow the original 16-state table, including right corrections."""
    if state.kind is Kind.DRIFT and state.direction != 0:
        left, right = wheel_speeds(speed, state.direction, state.inner_ratio)
        return ModeCommand(left, right, f"{state.label}; original-table correction")
    if state.kind is Kind.AMBIGUOUS:
        verdict, offset = resolve_blind(previous_localising)
        if verdict == "blind" and offset is not None:
            direction = 1 if offset > 0 else -1
            left, right = wheel_speeds(speed, direction, state.inner_ratio)
            return ModeCommand(left, right, "blind band; original-table correction")
        if verdict == "hold" and previous_command is not None:
            return previous_command
        return _forward(speed, f"{state.label}; original-table forward")
    if state.kind is Kind.JUNCTION and state.direction != 0:
        left, right = wheel_speeds(speed, state.direction, state.inner_ratio)
        return ModeCommand(left, right, f"{state.label}; original-table junction correction")
    if state.kind is Kind.NOISE and previous_command is not None:
        return previous_command
    return _forward(speed, "centred or unresolved; original-table forward")


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
    forward_s = 2.1
    turn_s = 0.41 + 90.0 / 39.7
    return forward_s, turn_s


def roundabout_entry_turn_s() -> float:
    """Return the calibrated open-loop time for the documented 42.5-degree entry turn."""
    return roundabout_turn_s(ROUNDABOUT_ENTRY_TURN_DEG)


def roundabout_turn_s(angle_deg: float) -> float:
    """Return calibrated open-loop time for a right spin at the configured speed."""
    if angle_deg <= 0:
        raise ValueError("turn angle must be positive")
    return SPIN_DEAD_TIME_S + angle_deg / SPIN_RATE_DEG_PER_S


def roundabout_p1001_action_timing() -> tuple[float, float]:
    """Return forward and right-turn times for the sustained-P1001 action."""
    forward_s = phase1_to_phase2_timing()[0] * ROUNDABOUT_P1001_FORWARD_CM / 17.0
    return forward_s, roundabout_turn_s(ROUNDABOUT_P1001_TURN_DEG)


def roundabout_post_exit_p0111_forward_s() -> float:
    """Return the calibrated time for the post-exit 5 cm forward move."""
    return phase1_to_phase2_timing()[0] * ROUNDABOUT_POST_EXIT_P0111_FORWARD_CM / 17.0


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
