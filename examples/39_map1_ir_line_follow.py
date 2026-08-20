#!/usr/bin/env python3
"""Map1 test: drive the circular track using only IR sensor (no camera).

Pure 4-channel IR sensor line following, plus a scripted junction turn.

**Motor-moving. Operator must stand beside the car able to cut main power instantly.**

Wiring and physical layout — see `carbot.ir_geometry`, and the mapping table in
docs/hardware/ir-tracing-sensor.md. Do NOT assume Out1..Out4 is left-to-right:

  position       P1      P2      P3      P4
  channel      Out2    Out1    Out3    Out4
  BCM GPIO       25      24      22      23     (measured 2026-08-19)
  offset      -3.2cm  -0.4cm  +0.4cm  +3.2cm

Readings are logged in **physical P1..P4 order**, not channel order, so the bit
string reads left-to-right as the bar is laid out.

Steering (see `carbot.ir_geometry.STATE_TABLE`, total over all 16 readings):
  - `0110` centred → straight. `0010`/`0100` → slight correction; these are the
    only warning before the blind band, and the window is just 0.8cm wide
  - `0000` is NOT automatically "line lost". The outer gap is 2.8cm and the line
    2.0cm, so there is a 0.8cm band where the car is on the line and sees
    nothing. The previous reading decides (`resolve_blind`): after `0010`/`0100`
    it is the blind band (steering continues at ±1.8cm offset); after `0110` it
    is paper undulation/sensor bounce (previous command held); after `0001`/`1000`
    the line really has left the bar (SEARCH starts). Sustained `0000`/`1111`
    (2.0s) triggers REVERSE replay recovery
  - Junctions are matched against ordered signal sequences (real-track tracing,
    2026-08-20 — see `carbot.ir_route.RouteJunction.approach`), not single
    sustained readings. Which junction it is and the turn/creep distances come
    from the route sequence, gated by distance since the previous one. Note:
    this script (`39_map1_ir_line_follow.py`) uses a hardcoded start (16cm forward
    + 90° spin) for the start stem T junction, bypassing sensor-driven detection
    for that initial junction
  - Non-contiguous readings (`0101`, `1001`, `1010`, `1011`, `1101`) cannot come
    from a single 2cm line on their own. Outside an approach sequence (where steps
    like `0101`/`1001` match expected steps) they never steer — the previous command
    is held and the frame is counted as noise
  - Junction turns are closed-loop on `0110` with safety guards (`spin_dead_time_s`
    dead time floor, `turn_confirm_s` sustained confirmation, and `turn_timeout_s`
    timeout ceiling) — see `IRLineNav._turn_step`

Usage (wheels lifted, operator ready):
    PYTHONPATH=src python3 examples/39_map1_ir_line_follow.py --duration 120

Usage (simulation, no motor):
    PYTHONPATH=src python3 examples/39_map1_ir_line_follow.py --dry-run --duration 30

Usage (Phase 1 only, sensor-blind motion followed by a stopped hold):
    PYTHONPATH=src python3 examples/39_map1_ir_line_follow.py \
        --phase1-only --duration 20 --heartbeat-s 2
"""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from datetime import datetime
from typing import Protocol

from carbot.map1_phases import (
    ARC_TEST_DISTANCE_CREDIT_SCALE,
    ARC_TEST_OFF_TRACK_DWELL_S,
    ARC_TEST_REVERSE_REPLAY_WINDOW_S,
    PHASE1_FORWARD_CM,
    PHASE1_FORWARD_PWM,
    PHASE1_FORWARD_S,
    PHASE1_RIGHT_TURN_COMPENSATION_DEG,
    PHASE1_RIGHT_TURN_DEG,
    PHASE1_RIGHT_TURN_PULSE_DEG,
    PHASE1_SPIN_DEAD_TIME_S,
    PHASE1_SPIN_RATE_DEG_PER_S,
    PROVISIONAL_FORWARD_SPEED_CM_S,
    Map1PhaseKind,
    Map1PhaseProgress,
    map1_phase,
)

# Start box -> stem -> T junction -> ARC 1 is hardcoded (forward + scripted spin) instead of
# sensor-driven, per the operator's real-track observation that sensor-based detection there
# was unreliable. The forward pulse uses the operator-requested PWM 150 / 2.2s trial and refreshes
# every 0.01s. The earlier 3.66s value was rejected after raised paper caught under the chassis;
# the unobstructed 1.6s baseline then proved short. Passive IR never influences motion, and the
# desired turn remains 90 degrees, with a 5-degree pulse compensation after the turn still fell
# short in a later physical test.
HARDCODE_START_FORWARD_CM = PHASE1_FORWARD_CM
HARDCODE_START_FORWARD_S = PHASE1_FORWARD_S
HARDCODE_START_FORWARD_PWM = PHASE1_FORWARD_PWM
HARDCODE_START_TURN_DEG = PHASE1_RIGHT_TURN_DEG
HARDCODE_START_TURN_COMPENSATION_DEG = PHASE1_RIGHT_TURN_COMPENSATION_DEG
HARDCODE_START_TURN_PULSE_DEG = PHASE1_RIGHT_TURN_PULSE_DEG
HARDCODE_START_PAUSE_S = 0.2


class Phase1Car(Protocol):
    """Small motor surface used by the sensor-blind Phase 1 manoeuvre."""

    def move_for(
        self,
        seconds: float,
        left: int,
        right: int,
        *,
        on_command: Callable[[int, float, int, int], None] | None = None,
    ) -> int: ...


def phase2_acquisition_command(
    reading,
    nav,
    dt: float,
    speed: int,
    last_direction: int,
) -> tuple[int, int, int, str]:
    """Choose a bounded, in-place command while an independent Phase 2 test finds the line.

    A visible offset is stronger evidence than a blind pendulum guess: pivot toward that side and
    keep the same direction through the adjacent 0000 blind/lost band. Once P0110 is centred, stop
    so the stability gate can confirm it before Phase 2 distance accounting begins. Other readings
    fall back to the normal navigation search state machine.
    """
    kind = reading.state.kind.value
    if kind == "on_line":
        return 0, 0, last_direction, "centred; holding stopped for confirmation"

    if kind == "drift":
        direction = reading.state.direction
        if direction:
            left, right = (speed, -speed) if direction > 0 else (-speed, speed)
            side = "right" if direction > 0 else "left"
            return left, right, direction, f"visible line {side}; pivoting {side} to centre"

    if kind == "ambiguous" and last_direction:
        left, right = (speed, -speed) if last_direction > 0 else (-speed, speed)
        side = "right" if last_direction > 0 else "left"
        return left, right, last_direction, f"P0000 after {side} drift; continuing {side} pivot"

    command = nav.step(reading, dt)
    return command.left, command.right, last_direction, command.reason


def phase3_lead_in_transition(
    stage: int, physical: tuple[int, int, int, int]
) -> tuple[int, bool, str]:
    """Recognise ARC 1 while approaching from an arbitrary point on Phase 2.

    On the real map, the left-curving line moves across the bar as
    ``P0100 -> P0000 -> P1000`` despite ordinary Phase 2 correction. Left-pair
    ``P1100/P1110`` shapes are direct curve evidence. Right-side patterns are excluded because
    the raised map paper produces false black on the car's right.
    """
    if stage not in (0, 1, 2):
        raise ValueError("phase3 lead-in stage must be 0, 1, or 2")

    slight_left = (0, 1, 0, 0)
    blind = (0, 0, 0, 0)
    far_left = (1, 0, 0, 0)
    strong_left_curve = {(1, 1, 0, 0), (1, 1, 1, 0)}

    if physical in strong_left_curve:
        bits = "".join(str(bit) for bit in physical)
        return 0, True, f"direct left-curve pattern P{bits}"

    if stage == 0:
        if physical == slight_left:
            return 1, False, "left inner sensor reached; waiting for blind band"
        return 0, False, "waiting for Phase 2-to-ARC 1 leftward sequence"

    if stage == 1:
        if physical == slight_left:
            return 1, False, "left inner sensor remains active"
        if physical == blind:
            return 2, False, "left inner sensor cleared into blind band"
        if physical == far_left:
            return 0, True, "left line reached outer sensor after inner sensor"
        return 0, False, "candidate reset before the line crossed left"

    if physical == blind:
        return 2, False, "blind band persists after left inner sensor"
    if physical == far_left:
        return 0, True, "confirmed P0100 -> P0000 -> P1000 ARC 1 entry"
    if physical == slight_left:
        return 1, False, "line returned to left inner sensor; retaining candidate"
    return 0, False, "candidate reset before the left outer sensor"


@dataclass
class Phase3CompletionGate:
    """Sensor-event completion gate for ARC 1 followed by a short Phase 4 proof."""

    exit_confirm_s: float = 0.8
    phase4_proof_s: float = 2.0
    mode: str = "arc"
    arc_turn_observed: bool = False
    centred_elapsed: float = 0.0
    phase4_valid_elapsed: float = 0.0

    def update(
        self,
        physical: tuple[int, int, int, int],
        kind: str,
        nav_state: str,
        dt: float,
    ) -> str | None:
        """Return ``phase4`` or ``complete`` when that transition is confirmed."""
        if dt < 0:
            raise ValueError("dt must be non-negative")

        left_curve_evidence = physical in {
            (0, 1, 0, 0),
            (1, 0, 0, 0),
            (1, 1, 0, 0),
            (1, 1, 1, 0),
        }
        localising_follow = kind in ("on_line", "drift") and nav_state == "follow"

        if self.mode == "arc":
            if left_curve_evidence:
                self.arc_turn_observed = True
            if (
                self.arc_turn_observed
                and physical == (0, 1, 1, 0)
                and localising_follow
            ):
                self.centred_elapsed += dt
            else:
                self.centred_elapsed = 0.0
            if self.centred_elapsed >= self.exit_confirm_s:
                self.mode = "phase4"
                self.centred_elapsed = 0.0
                return "phase4"
            return None

        if self.mode == "phase4":
            if localising_follow:
                self.phase4_valid_elapsed += dt
            else:
                self.phase4_valid_elapsed = 0.0
            if self.phase4_valid_elapsed >= self.phase4_proof_s:
                self.mode = "complete"
                return "complete"
            return None

        return None


def _phase1_log(message: str) -> None:
    """Timestamp Phase 1 separately so sensor logs cannot be mistaken for start control."""
    stamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    print(f"[{stamp}] [PHASE1 OPEN_LOOP] {message}", flush=True)


def run_hardcoded_phase1(
    car: Phase1Car | None,
    *,
    speed: int,
    forward_speed: int = HARDCODE_START_FORWARD_PWM,
    forward_s: float = HARDCODE_START_FORWARD_S,
    spin_rate_deg_per_s: float,
    spin_dead_time_s: float,
    pause_s: float = HARDCODE_START_PAUSE_S,
    sleep: Callable[[float], None] = time.sleep,
    log: Callable[[str], None] = _phase1_log,
    observe_bits: Callable[[], tuple[int, ...]] | None = None,
) -> tuple[float, float]:
    """Run the start stem without reading or reacting to any IR sensor signal.

    ``Car.move_for`` guarantees a best-effort stop after both the forward leg and the spin,
    including when the operator interrupts the sleep. Keeping all Phase 1 motion in this
    function also makes it impossible for a ``P0000`` reading from the later navigation loop
    to interrupt the 16 cm forward command or the stationary 90 degree right turn.
    """
    if not 1 <= forward_speed <= 1000:
        raise ValueError("forward_speed must be in [1, 1000]")
    if forward_s <= 0:
        raise ValueError("forward_s must be positive")
    if spin_rate_deg_per_s <= 0:
        raise ValueError("spin_rate_deg_per_s must be positive")
    if spin_dead_time_s < 0 or pause_s < 0:
        raise ValueError("spin_dead_time_s and pause_s must be non-negative")

    turn_s = spin_dead_time_s + HARDCODE_START_TURN_PULSE_DEG / spin_rate_deg_per_s
    log(
        f"SENSORS DISABLED; plan: forward {HARDCODE_START_FORWARD_CM:.1f}cm for "
        f"{forward_s:.2f}s at PWM {forward_speed} (command refresh 0.01s), then stationary "
        f"right turn target {HARDCODE_START_TURN_DEG:.0f}deg + "
        f"{HARDCODE_START_TURN_COMPENSATION_DEG:.0f}deg compensation "
        f"({HARDCODE_START_TURN_PULSE_DEG:.0f}deg pulse) for {turn_s:.2f}s at PWM {speed}"
    )

    if car is None:
        log("DRY RUN; no motor commands issued")
        return forward_s, turn_s

    def audit(label: str) -> Callable[[int, float, int, int], None]:
        def record(write: int, elapsed: float, left: int, right: int) -> None:
            if write != 1 and write % 10:
                return
            sensor = ""
            if observe_bits is not None:
                bits = "".join(str(bit) for bit in observe_bits())
                sensor = f"; passive IR=P{bits} (OBSERVE ONLY)"
            log(f"{label} COMMAND #{write:02d} t={elapsed:.3f}s L={left} R={right}{sensor}")

        return record

    log("FORWARD START; sensor input remains disabled")
    forward_writes = car.move_for(
        forward_s,
        forward_speed,
        forward_speed,
        on_command=audit("FORWARD"),
    )
    log(f"FORWARD COMPLETE; {forward_writes} non-zero writes; wheels stopped before the pivot")
    sleep(pause_s)

    log("RIGHT SPIN START; sensor input remains disabled")
    turn_writes = car.move_for(turn_s, speed, -speed, on_command=audit("RIGHT SPIN"))
    log(f"RIGHT SPIN COMPLETE; {turn_writes} non-zero writes; Phase 1 finished stopped")
    sleep(pause_s)
    return forward_s, turn_s


def main() -> int:
    parser = argparse.ArgumentParser(description="Map1 IR sensor line tracking (no camera)")
    parser.add_argument("--dry-run", action="store_true", help="detection only, no motor")
    parser.add_argument(
        "--hz",
        type=float,
        default=100.0,
        help="control loop rate (default 100). Free-running produced 6.2M frames and a "
        f"608MB log in 148s; at {PROVISIONAL_FORWARD_SPEED_CM_S:.1f}cm/s, 100Hz samples "
        "about 1mm of travel per cycle",
    )
    parser.add_argument(
        "--log-every",
        action="store_true",
        help="log every cycle instead of only on state change (very verbose)",
    )
    parser.add_argument(
        "--log-min-interval-s",
        type=float,
        default=0.0,
        help="with --log-every, skip cycles closer together than this to a previous log line "
        "(default 0 = every cycle); a state change is still logged immediately regardless",
    )
    parser.add_argument(
        "--heartbeat-s",
        type=float,
        default=2.0,
        help="print the current state at least this often even when unchanged",
    )
    parser.add_argument("--duration", type=float, default=120.0, help="run duration (seconds)")
    parser.add_argument(
        "--speed", type=int, default=150, help="base drive speed 0-1000 (default 150)"
    )
    parser.add_argument(
        "--phase1-forward-speed",
        type=int,
        default=HARDCODE_START_FORWARD_PWM,
        help=f"Phase 1 forward-only PWM (default {HARDCODE_START_FORWARD_PWM}); kept separate "
        "from --speed for diagnosis without changing the spin calibration",
    )
    parser.add_argument(
        "--phase1-forward-s",
        type=float,
        default=HARDCODE_START_FORWARD_S,
        help=f"Phase 1 forward pulse seconds (default {HARDCODE_START_FORWARD_S:.1f})",
    )
    parser.add_argument(
        "--turn-gain",
        type=float,
        default=2.0,
        help="steering sensitivity (default 2.0, higher = sharper turns)",
    )
    parser.add_argument(
        "--invert",
        default="0,1,2,3",
        help="IR channels to invert; default 0,1,2,3 (all) — verified 2026-08-18 "
        "after potentiometer retuning; re-check with examples/36 if pots are touched again",
    )
    parser.add_argument(
        "--turn-direction",
        choices=("right", "left"),
        default="right",
        help="fallback scripted turn direction before the first junction commits (every "
        "real junction sets its own direction from the route; Task-1 never reaches this)",
    )
    parser.add_argument(
        "--turn-timeout-scale",
        type=float,
        default=2.0,
        help="safety ceiling for a closed-loop junction turn (watches for 0110), as a "
        "multiple of the nominal timed duration for that junction's expected turn angle",
    )
    parser.add_argument(
        "--turn-confirm-s",
        type=float,
        default=0.08,
        help="how long TURN_COMPLETE_READING (0110) must be sustained before a closed-loop "
        "junction turn is considered done, not just seen for one frame",
    )
    parser.add_argument(
        "--approach-break-confirm-s",
        type=float,
        default=0.05,
        help="how long a genuine ON_LINE/DRIFT reading must be sustained mid-approach before "
        "it resets a junction's in-progress approach sequence, not just seen for one frame",
    )
    parser.add_argument(
        "--forward-speed-cm-per-s",
        type=float,
        default=PROVISIONAL_FORWARD_SPEED_CM_S,
        help="measured full-speed travel rate at --speed, used for command-aware distance "
        f"estimation and creep timing (provisional default {PROVISIONAL_FORWARD_SPEED_CM_S:.1f})",
    )
    parser.add_argument(
        "--start-on-loop",
        action="store_true",
        help="car starts on the east-west line facing east, not in the start box, so the "
        "one-time stem T junction is dropped and the first junction is the roundabout entry",
    )
    parser.add_argument(
        "--phase1-only",
        action="store_true",
        help="run only the sensor-blind 16cm forward + stationary 90deg right turn, then "
        "hold stopped until --duration expires; no IR sensor reading is taken",
    )
    parser.add_argument(
        "--start-acquire-timeout-s",
        "--start-ignore-s",
        dest="start_acquire_timeout_s",
        type=float,
        default=2.0,
        help="after the hardcoded start, remain stopped for this long while requiring a stable "
        "ON_LINE/DRIFT reading before Phase 2; the old --start-ignore-s name remains as a "
        "compatibility alias but no longer authorizes blind forward motion",
    )
    parser.add_argument(
        "--phase3-lead-in-timeout-s",
        type=float,
        default=5.0,
        help="for an independent Phase 3 test, follow east from any centred Phase 2 position "
        "for at most this long while detecting ARC 1 (default 5.0s)",
    )
    parser.add_argument(
        "--phase3-exit-confirm-s",
        type=float,
        default=0.8,
        help="stable centred P0110 required after observed ARC turning before switching to "
        "Phase 4 straight control (default 0.8s)",
    )
    parser.add_argument(
        "--phase4-proof-s",
        type=float,
        default=2.0,
        help="valid Phase 4 straight-line following required before the Phase 3 test stops "
        "successfully (default 2.0s)",
    )
    parser.add_argument(
        "--test-phase",
        type=int,
        choices=range(1, 11),
        help="independent bounded hardware test for one phase; Phase 1 uses the open-loop start, "
        "Phases 2-10 require manual placement at that phase's documented entry pose and disable "
        "all full-route junction actions",
    )
    parser.add_argument(
        "--stop-after-phase",
        type=int,
        choices=range(1, 11),
        help="stop when command-aware estimated progress completes this phase",
    )
    parser.add_argument(
        "--start-phase-transitions",
        type=int,
        default=0,
        help="seed the straight/arc phase tracker's transition count at startup, for testing "
        "a route segment in isolation (e.g. placing the car on Phase 6 with --start-on-loop "
        "and --start-phase-transitions 6 to satisfy the roundabout entry's own precondition "
        "without first driving Phase 2 through ARC 2) -- see RouteJunction.min_phase_transitions",
    )
    parser.add_argument(
        "--start-arc-cm",
        type=float,
        default=0.0,
        help="seed the phase tracker's accumulated arc distance at startup, alongside "
        "--start-phase-transitions -- see RouteJunction.min_arc_cm",
    )
    parser.add_argument(
        "--start-cm-since-previous",
        type=float,
        default=0.0,
        help="seed the distance-since-last-junction counter at startup, alongside "
        "--start-phase-transitions -- TASK1_CORNER_WINDOWS (carbot.ir_route) are keyed to "
        "this distance from the true roundabout entry, so testing a later segment (e.g. "
        "Phase 6, cumulative 57.5cm) without seeding it lets ARC 1/2/3's corner windows fire "
        "at the wrong physical spot",
    )
    parser.add_argument(
        "--laps",
        type=int,
        default=0,
        help="stop at the T junction closing lap N (0 = lap forever). The stop is an entry "
        "in the route sequence, so it only fires after every junction before it was reached",
    )
    parser.add_argument(
        "--search-sweep-deg",
        type=float,
        default=45.0,
        help="maximum in-place pendulum sweep angle (deg) during line-recovery search; the "
        "car pivots left then right on the spot (never creeps forward). Starts at "
        "--search-sweep-min-deg and steps up by --search-sweep-step-deg up to this ceiling",
    )
    parser.add_argument(
        "--search-sweep-min-deg",
        type=float,
        default=5.0,
        help="start angle of the in-place pendulum sweep (deg) during line-recovery search",
    )
    parser.add_argument(
        "--search-sweep-step-deg",
        type=float,
        default=5.0,
        help="how many degrees the pendulum sweep angle steps up each full pair (5 -> 10 -> "
        "15 ...), up to --search-sweep-deg",
    )
    parser.add_argument(
        "--search-creep-step-s",
        type=float,
        default=0.3,
        help="DEPRECATED/unused: the search is in-place only and never creeps forward",
    )
    parser.add_argument(
        "--search-creep-speed-ratio",
        type=float,
        default=0.5,
        help="DEPRECATED/unused: kept for CLI back-compat",
    )
    parser.add_argument(
        "--search-creep-steps",
        type=int,
        default=4,
        help="DEPRECATED/unused: kept for CLI back-compat",
    )
    parser.add_argument(
        "--search-give-up-s",
        type=float,
        default=30.0,
        help="stop the car after this many seconds of searching (0 = never)",
    )
    parser.add_argument(
        "--phase-transition-dwell-s",
        type=float,
        default=0.8,
        help="non-0110 reading must persist this long to count as a real arc-correction "
        "event (straight/arc phase tracker); shorter blips are noise and are ignored",
    )
    parser.add_argument(
        "--off-track-dwell-s",
        type=float,
        default=None,
        help="continuous 0000 (blank paper) or 1111 (off the paper onto carpet) for this "
        "long triggers reverse-replay recovery (default: 2.0s; bounded ARC test: 0.3s)",
    )
    parser.add_argument(
        "--reverse-replay-window-s",
        type=float,
        default=None,
        help="how much recent commanded wheel-speed history to replay backward, sign-flipped, "
        "during off-track recovery (default: 2.0s; bounded ARC test: 3.0s)",
    )
    args = parser.parse_args()

    if args.phase1_only and args.start_on_loop:
        parser.error("--phase1-only cannot be combined with --start-on-loop")
    if args.phase1_only and args.test_phase is not None:
        parser.error("--phase1-only cannot be combined with --test-phase")
    if args.test_phase is not None and args.laps:
        parser.error("--test-phase cannot be combined with --laps")
    if args.start_acquire_timeout_s < 0:
        parser.error("--start-acquire-timeout-s must be non-negative")
    if args.phase3_lead_in_timeout_s <= 0:
        parser.error("--phase3-lead-in-timeout-s must be positive")
    if args.phase3_exit_confirm_s <= 0:
        parser.error("--phase3-exit-confirm-s must be positive")
    if args.phase4_proof_s <= 0:
        parser.error("--phase4-proof-s must be positive")
    if not 1 <= args.phase1_forward_speed <= 1000:
        parser.error("--phase1-forward-speed must be in [1, 1000]")
    if args.phase1_forward_s <= 0:
        parser.error("--phase1-forward-s must be positive")
    if args.test_phase is not None:
        # Phase 3 has repeatedly stopped mid-ARC because this chassis has no wheel encoders and
        # its PWM/time distance estimate is not physical distance. Its independent test is bounded
        # only by --duration (and operator Ctrl+C), never by the provisional 12cm estimate.
        args.stop_after_phase = None if args.test_phase == 3 else args.test_phase
        args.start_on_loop = args.test_phase >= 2

    print("=" * 70)
    print("Map1 IR Sensor Line Tracking")
    print(f"Start time: {datetime.now().astimezone().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)
    print(f"Speed: {args.speed} | Turn gain: {args.turn_gain} | Duration: {args.duration}s")
    if args.test_phase is not None:
        phase = map1_phase(args.test_phase)
        print(
            f"Independent Phase {phase.number}: {phase.name} | {phase.distance_cm:.1f}cm | "
            f"entry heading {phase.entry_heading}"
        )
        print(f"Placement/instruction: {phase.instruction}")
        if phase.number == 3:
            print(
                "Phase 3 placement: any stable P0110 point on the Phase 2 eastbound straight, "
                "car heading east"
            )
            print(
                "Phase 3 completion: NO distance stop; switch to Phase 4 after sensor-confirmed "
                "ARC exit, then stop after the Phase 4 proof segment"
            )
        if phase.kind is Map1PhaseKind.ARC and phase.number != 3:
            print(
                f"ARC progress calibration: command distance credit "
                f"x{ARC_TEST_DISTANCE_CREDIT_SCALE:.2f} (observed curve-speed correction)"
            )
        if phase.steering_direction_limit:
            direction_name = "LEFT" if phase.steering_direction_limit < 0 else "RIGHT"
            print(
                f"Phase {phase.number} direction guard: {direction_name} ONLY; "
                "opposite-side IR patterns are treated as raised-paper noise"
            )
        if phase.kind is Map1PhaseKind.ARC:
            print(
                f"ARC off-track recovery: P1111/P0000 dwell "
                f"{ARC_TEST_OFF_TRACK_DWELL_S:.1f}s, reverse history "
                f"{ARC_TEST_REVERSE_REPLAY_WINDOW_S:.1f}s"
            )
    print()

    if not args.dry_run:
        answer = input("Operator beside car, track clear, power ready to cut? (yes/no) ").strip()
        if answer.lower() != "yes":
            print("Re-run when ready.")
            return 1

    from RPi import GPIO

    from carbot.ir_line_nav import (
        IRLineNav,
        IRNavCommand,
        IRNavPolicy,
        IRNavState,
        detect_ir_line,
    )
    from carbot.ir_tracing import IRTracingSensor

    # Setup IR sensor
    GPIO.setmode(GPIO.BCM)
    pins = (24, 25, 22, 23)
    for pin in pins:
        GPIO.setup(pin, GPIO.IN)

    invert_set = set()
    if args.invert.strip():
        invert_set = {int(x) for x in args.invert.split(",") if x.strip()}

    sensor = IRTracingSensor(pins, GPIO, invert=invert_set)

    from carbot.ir_route import TASK1_CORNER_WINDOWS, TASK1_LOOP_ONLY, task1_route_for_laps

    # Start~T junction -> ARC 1 is hardcoded below (forward + scripted turn) instead of
    # sensor-driven, so the route the nav tracks never includes the stem T junction -- it
    # is either handled by the hardcoded manoeuvre (--start-on-loop unset) or already done
    # by hand (--start-on-loop set). Either way TASK1_LOOP_ONLY (no prologue) is correct.
    if args.laps > 0:
        route = task1_route_for_laps(args.laps, start_on_loop=True)
    else:
        route = TASK1_LOOP_ONLY

    phase_test_spec = map1_phase(args.test_phase) if args.test_phase is not None else None
    bounded_arc_test = (
        phase_test_spec is not None and phase_test_spec.kind is Map1PhaseKind.ARC
    )
    off_track_dwell_s = args.off_track_dwell_s
    if off_track_dwell_s is None:
        off_track_dwell_s = ARC_TEST_OFF_TRACK_DWELL_S if bounded_arc_test else 2.0
    reverse_replay_window_s = args.reverse_replay_window_s
    if reverse_replay_window_s is None:
        reverse_replay_window_s = (
            ARC_TEST_REVERSE_REPLAY_WINDOW_S if bounded_arc_test else 2.0
        )
    # Do not silently reduce curve-test PWM. The 2026-08-21 Phase 3 logs proved that --speed 150
    # became PWM 90 (and as little as 12 on the inner wheel), leaving the motors buzzing without
    # moving the chassis. Steering ratios already slow the inside wheel; the outside/base wheel
    # must retain the operator-requested speed.
    nav_speed = args.speed

    nav_policy = IRNavPolicy(
        route=route,
        speed=nav_speed,
        turn_gain=args.turn_gain,
        turn_direction=1 if args.turn_direction == "right" else -1,
        turn_timeout_scale=args.turn_timeout_scale,
        turn_confirm_s=args.turn_confirm_s,
        approach_break_confirm_s=args.approach_break_confirm_s,
        forward_speed_cm_per_s=args.forward_speed_cm_per_s,
        forward_speed_reference_pwm=args.speed,
        search_sweep_deg=args.search_sweep_deg,
        search_sweep_min_deg=args.search_sweep_min_deg,
        search_sweep_step_deg=args.search_sweep_step_deg,
        search_creep_step_s=args.search_creep_step_s,
        search_creep_speed_ratio=args.search_creep_speed_ratio,
        search_creep_steps_per_cycle=args.search_creep_steps,
        search_give_up_s=args.search_give_up_s,
        phase_transition_dwell_s=args.phase_transition_dwell_s,
        off_track_dwell_s=off_track_dwell_s,
        reverse_replay_window_s=reverse_replay_window_s,
        corner_windows=() if phase_test_spec is not None else TASK1_CORNER_WINDOWS,
        junction_detection_enabled=phase_test_spec is None,
        curve_pattern_steering_enabled=phase_test_spec is not None
        and phase_test_spec.kind in (Map1PhaseKind.ARC, Map1PhaseKind.ROUNDABOUT),
        # ARC 1/2/3 are all mapped left curves. Hardware evidence first identified the
        # opposite-side readings as raised-paper false black on ARC 1; the phase model now
        # carries the same physical direction constraint for all three bounded ARC tests.
        steering_direction_limit=(
            phase_test_spec.steering_direction_limit if phase_test_spec is not None else 0
        ),
    )
    nav = IRLineNav(nav_policy)
    if args.start_phase_transitions or args.start_arc_cm:
        # Segment testing: the car is placed mid-route by hand, so the phase tracker has to
        # be told what a from-the-start run would have already accumulated by this point --
        # see RouteJunction.min_phase_transitions/.min_arc_cm (carbot.ir_route).
        nav._phase_transitions = args.start_phase_transitions
        nav._arc_cm = args.start_arc_cm
    if args.start_cm_since_previous:
        # TASK1_CORNER_WINDOWS are keyed to distance from the true roundabout entry -- without
        # this, they fire at whatever cm_since_previous happens to be counted up from 0 during
        # the segment, not the car's real physical position.
        nav.junctions.cm_since_previous = args.start_cm_since_previous

    from carbot import Car, NeZhaError

    car = None
    if not args.dry_run:
        try:
            car = Car()
        except NeZhaError as exc:
            print(f"Connection failed: {exc}")
            GPIO.cleanup()
            return 1

    # Phase 3's --duration is the safety ceiling for the whole independent run, including
    # centring and the Phase 2 lead-in. It must not restart when ARC 1 is detected.
    phase3_test_window_started = time.monotonic() if args.test_phase == 3 else None

    # Hardcoded start -> ARC 1: open-loop forward + scripted spin, no sensor. Skipped when
    # --start-on-loop -- the car is already placed past the T junction by hand.
    phase1_started = time.monotonic()
    if not args.start_on_loop:
        # Phase 1 has its own observed calibration. The 90-degree route target receives a
        # 5-degree open-loop pulse compensation, producing about 2.80s at 39.7deg/s plus the
        # measured 0.41s dead time. Route junction turns remain sensor-closed-loop.
        try:
            run_hardcoded_phase1(
                car,
                speed=args.speed,
                forward_speed=args.phase1_forward_speed,
                forward_s=args.phase1_forward_s,
                spin_rate_deg_per_s=PHASE1_SPIN_RATE_DEG_PER_S,
                spin_dead_time_s=PHASE1_SPIN_DEAD_TIME_S,
                observe_bits=sensor.read,
            )
        except KeyboardInterrupt:
            _phase1_log("INTERRUPTED BY OPERATOR; move_for issued a best-effort stop")
            if car:
                car.close()
            GPIO.cleanup()
            return 130
        except (NeZhaError, ValueError) as exc:
            _phase1_log(f"FAILED: {exc}")
            if car:
                car.close()
            GPIO.cleanup()
            return 1

    if args.phase1_only or args.test_phase == 1:
        # Keep the process alive for the requested evidence window, but keep all wheels at
        # zero. This is intentionally before every sensor read: Phase 1-only means no Pxxxx
        # line can appear in the log, which makes an old/incorrect Pi copy obvious.
        hold_until = phase1_started + max(0.0, args.duration)
        next_heartbeat = time.monotonic()
        try:
            while time.monotonic() < hold_until:
                now = time.monotonic()
                if now >= next_heartbeat:
                    remaining = max(0.0, hold_until - now)
                    _phase1_log(f"HOLD STOPPED; {remaining:.1f}s remaining")
                    next_heartbeat = now + max(0.1, args.heartbeat_s)
                time.sleep(min(0.05, max(0.0, hold_until - time.monotonic())))
        except KeyboardInterrupt:
            _phase1_log("STOPPED BY OPERATOR DURING HOLD")
        finally:
            if car:
                car.close()
            GPIO.cleanup()
        _phase1_log("PHASE1-ONLY TEST COMPLETE")
        return 0

    if args.stop_after_phase == 1:
        if car:
            car.close()
        GPIO.cleanup()
        _phase1_log("STOP-AFTER-PHASE 1 COMPLETE")
        return 0

    # Phase 3 is physically placed on Phase 2, so its lead-in uses ordinary unrestricted straight
    # following. The left-only ARC policy is kept aside until the measured curve entry appears.
    phase3_arc_policy = nav_policy if args.test_phase == 3 else None
    phase3_lead_in_policy = None
    if phase3_arc_policy is not None:
        phase3_lead_in_policy = replace(
            phase3_arc_policy,
            steering_direction_limit=0,
            curve_pattern_steering_enabled=False,
            off_track_dwell_s=2.0,
            reverse_replay_window_s=2.0,
        )
        nav = IRLineNav(phase3_lead_in_policy)

    # Independent Phase 2 and Phase 3's Phase-2 lead-in both require a stable P0110 before
    # moving east. Pivot toward a visible offset, then hold stopped for 0.10s confirmation.
    if args.test_phase in (2, 3) and args.start_acquire_timeout_s > 0 and car:
        acquisition_label = "Phase 3 lead-in" if args.test_phase == 3 else "Phase 2"
        print(
            f"{acquisition_label} acquisition: active in-place search; requiring stable "
            "P0110 for 0.10s "
            f"within {args.start_acquire_timeout_s:.1f}s",
            flush=True,
        )
        acquire_started = time.monotonic()
        acquire_last = acquire_started
        deadline = acquire_started + args.start_acquire_timeout_s
        stable_started: float | None = None
        acquired = False
        last_direction = 0
        last_key: tuple | None = None
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                dt = now - acquire_last
                acquire_last = now
                reading = detect_ir_line(sensor, speed=args.speed)
                left, right, last_direction, reason = phase2_acquisition_command(
                    reading, nav, dt, args.speed, last_direction
                )
                centred = reading.state.kind.value == "on_line"
                if centred:
                    stable_started = stable_started or now
                else:
                    stable_started = None
                car.drive(left, right)

                key = (reading.physical, left, right, reason)
                if key != last_key:
                    bits = "".join(str(bit) for bit in reading.physical)
                    print(
                        f"  acquire P{bits} {reading.state.kind.value} -> "
                        f"L{left} R{right}: {reason}",
                        flush=True,
                    )
                    last_key = key
                if stable_started is not None and now - stable_started >= 0.10:
                    acquired = True
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            car.stop(best_effort=True)
            car.close()
            GPIO.cleanup()
            print(f"{acquisition_label.upper()} ACQUISITION INTERRUPTED; wheels stopped", flush=True)
            return 130
        except NeZhaError as exc:
            car.stop(best_effort=True)
            car.close()
            GPIO.cleanup()
            print(f"{acquisition_label.upper()} ACQUISITION MOTOR ERROR: {exc}", flush=True)
            return 1

        if not car.stop(best_effort=True):
            print(f"*** {acquisition_label.upper()} STOP FAILED — CUT POWER NOW ***", flush=True)
            car.close()
            GPIO.cleanup()
            return 1
        if not acquired:
            print(
                f"{acquisition_label.upper()} ACQUISITION FAILED: P0110 was not stable "
                "before timeout; "
                "wheels stopped — reposition the car or inspect the sensor readings",
                flush=True,
            )
            car.close()
            GPIO.cleanup()
            return 2
        next_action = (
            "ARC-entry lead-in starts now"
            if args.test_phase == 3
            else "distance counter starts now"
        )
        print(
            f"{acquisition_label} line centred in "
            f"{time.monotonic() - acquire_started:.2f}s; {next_action}",
            flush=True,
        )
        # Acquisition commands must not leak into reverse history, junction distance, or state.
        nav = IRLineNav(phase3_lead_in_policy or nav_policy)

    # The operator cannot place the chassis precisely at the Phase 2/3 interface. Follow east
    # from any centred Phase 2 point, then switch controllers only after the real leftward sensor
    # sequence confirms ARC 1. The Phase 3 duration safety window already includes this lead-in.
    if args.test_phase == 3 and car:
        assert phase3_arc_policy is not None and phase3_lead_in_policy is not None
        print(
            "Phase 3 lead-in: following Phase 2 east; waiting for "
            "P0100 -> P0000 -> P1000 or a direct left-pair curve pattern",
            flush=True,
        )
        lead_started = time.monotonic()
        lead_last = lead_started
        lead_deadline = lead_started + args.phase3_lead_in_timeout_s
        lead_stage = 0
        lead_detected = False
        lead_last_key: tuple | None = None
        try:
            while time.monotonic() < lead_deadline:
                now = time.monotonic()
                dt = now - lead_last
                lead_last = now
                reading = detect_ir_line(sensor, speed=args.speed)
                command = nav.step(reading, dt)
                lead_stage, lead_detected, lead_reason = phase3_lead_in_transition(
                    lead_stage, reading.physical
                )
                car.drive(command.left, command.right)

                key = (reading.physical, command.left, command.right, lead_stage, lead_detected)
                if key != lead_last_key:
                    bits = "".join(str(bit) for bit in reading.physical)
                    print(
                        f"  lead-in P{bits} {reading.state.kind.value} -> "
                        f"L{command.left} R{command.right}: {lead_reason}",
                        flush=True,
                    )
                    lead_last_key = key
                if lead_detected:
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            car.stop(best_effort=True)
            car.close()
            GPIO.cleanup()
            print("PHASE 3 LEAD-IN INTERRUPTED; wheels stopped", flush=True)
            return 130
        except NeZhaError as exc:
            car.stop(best_effort=True)
            car.close()
            GPIO.cleanup()
            print(f"PHASE 3 LEAD-IN MOTOR ERROR: {exc}", flush=True)
            return 1

        if not car.stop(best_effort=True):
            print("*** PHASE 3 LEAD-IN STOP FAILED — CUT POWER NOW ***", flush=True)
            car.close()
            GPIO.cleanup()
            return 1
        if not lead_detected:
            print(
                "PHASE 3 LEAD-IN FAILED: ARC 1 was not confirmed before timeout; "
                "wheels stopped — left-only ARC control was not started",
                flush=True,
            )
            car.close()
            GPIO.cleanup()
            return 2
        print(
            f"Phase 3 ARC 1 detected after {time.monotonic() - lead_started:.2f}s; "
            "switching to LEFT ONLY, with distance stopping disabled",
            flush=True,
        )
        nav = IRLineNav(phase3_arc_policy)
    elif args.test_phase == 3:
        # A dry run has no live sensor/motor lead-in; simulate the actual ARC controller.
        assert phase3_arc_policy is not None
        nav = IRLineNav(phase3_arc_policy)

    # Phase 1 -> 2 handoff. The previous implementation drove blindly for as long as 3s and
    # stopped on *any* black channel; at the provisional 7.27cm/s calibration that could consume
    # almost all of Phase 2 before closed-loop control even started. Keep the wheels stopped and
    # require a stable, localising single-line reading instead. Failure is an operator placement
    # or calibration problem, not permission to drive farther without localisation.
    if not args.start_on_loop and args.start_acquire_timeout_s > 0 and car:
        car.stop()
        print(
            f"Phase 2 acquisition: wheels stopped; requiring stable ON_LINE/DRIFT for 0.10s "
            f"within {args.start_acquire_timeout_s:.1f}s",
            flush=True,
        )
        acquire_started = time.monotonic()
        deadline = acquire_started + args.start_acquire_timeout_s
        stable_started: float | None = None
        acquired = False
        last_bits: tuple[int, int, int, int] | None = None
        while time.monotonic() < deadline:
            reading = detect_ir_line(sensor, speed=args.speed)
            now = time.monotonic()
            if reading.physical != last_bits:
                bits = "".join(str(bit) for bit in reading.physical)
                print(f"  acquire P{bits} {reading.state.kind.value}", flush=True)
                last_bits = reading.physical
            localising = reading.state.kind.value in ("on_line", "drift")
            if localising:
                stable_started = stable_started or now
                if now - stable_started >= 0.10:
                    acquired = True
                    break
            else:
                stable_started = None
            time.sleep(0.02)
        if not acquired:
            print(
                "PHASE 2 ACQUISITION FAILED: no stable line while stopped; "
                "reposition the car or recalibrate the sensor before retrying",
                flush=True,
            )
            car.close()
            GPIO.cleanup()
            return 2
        print(
            f"Phase 2 line acquired in {time.monotonic() - acquire_started:.2f}s; "
            "closed-loop control enabled",
            flush=True,
        )

    progress_start_phase = args.test_phase if args.test_phase is not None else 2
    phase_progress = Map1PhaseProgress(start_phase=progress_start_phase)
    previous_phase_command = None

    loop_started = time.monotonic()
    start = phase3_test_window_started or loop_started
    last = loop_started
    frame_index = 0
    line_lost_count = 0
    line_found_count = 0
    search_entries = 0
    reverse_entries = 0
    last_state = None
    drive_error: NeZhaError | None = None

    period = 1.0 / args.hz if args.hz > 0 else 0.0
    last_logged: tuple | None = None
    last_log_time = 0.0
    logged_lines = 0
    phase3_completion = (
        Phase3CompletionGate(
            exit_confirm_s=args.phase3_exit_confirm_s,
            phase4_proof_s=args.phase4_proof_s,
        )
        if args.test_phase == 3
        else None
    )

    try:
        while True:
            now = time.monotonic()
            if period:
                sleep_for = period - (now - last)
                if sleep_for > 0:
                    time.sleep(sleep_for)
                    now = time.monotonic()
            dt = now - last
            last = now
            frame_index += 1
            elapsed = now - start

            if previous_phase_command is not None and args.test_phase != 3:
                progress_speed_cm_s = args.forward_speed_cm_per_s
                current_progress_phase = phase_progress.current
                if (
                    args.test_phase is not None
                    and current_progress_phase is not None
                    and current_progress_phase.kind is Map1PhaseKind.ARC
                ):
                    progress_speed_cm_s *= ARC_TEST_DISTANCE_CREDIT_SCALE
                transitions = phase_progress.observe_command(
                    dt=dt,
                    left=previous_phase_command.left,
                    right=previous_phase_command.right,
                    reference_pwm=args.speed,
                    reference_speed_cm_s=progress_speed_cm_s,
                )
                stop_for_phase = False
                for transition in transitions:
                    next_label = (
                        f"Phase {transition.current.number} {transition.current.name}"
                        if transition.current is not None
                        else "route-distance complete"
                    )
                    print(
                        f"\n[PHASE] Phase {transition.completed.number} "
                        f"{transition.completed.name} complete -> {next_label}",
                        flush=True,
                    )
                    if args.stop_after_phase == transition.completed.number:
                        stop_for_phase = True
                if stop_for_phase:
                    if car:
                        car.stop()
                    print(
                        f"\nSTOP-AFTER-PHASE {args.stop_after_phase}: wheels stopped at "
                        "the command-aware distance boundary",
                        flush=True,
                    )
                    break

            # Read IR sensor
            reading = detect_ir_line(sensor, speed=args.speed)
            command = nav.step(reading, dt)

            if phase3_completion is not None:
                phase3_event = phase3_completion.update(
                    reading.physical,
                    reading.state.kind.value,
                    command.state.value,
                    dt,
                )
                if phase3_event == "phase4":
                    assert phase3_lead_in_policy is not None
                    print(
                        "\n[PHASE] Phase 3 ARC 1 sensor exit confirmed -> "
                        "Phase 4 North straight",
                        flush=True,
                    )
                    nav = IRLineNav(phase3_lead_in_policy)
                    command = nav.step(reading, 0.0)
                elif phase3_event == "complete":
                    command = IRNavCommand(
                        0,
                        0,
                        "Phase 4 proof complete after sensor-confirmed ARC 1 exit",
                        IRNavState.STOPPED,
                    )

            # Track statistics
            if command.state is IRNavState.SEARCH and last_state is not IRNavState.SEARCH:
                search_entries += 1
            if command.state is IRNavState.REVERSE and last_state is not IRNavState.REVERSE:
                reverse_entries += 1
            last_state = command.state
            if reading.visible:
                line_found_count += 1
                line_lost_count = 0
            else:
                line_lost_count += 1

            # Drive. A bus error here must end the run through the normal path — letting it
            # escape skips the stop and leaves the wheels turning.
            if car:
                try:
                    car.drive(command.left, command.right)
                except NeZhaError as exc:
                    drive_error = exc
                    break
            previous_phase_command = command

            # Log — bits are physical P1..P4, left to right along the bar.
            # Only on change by default: a car tracking a straight line holds
            # one reading for thousands of cycles and printing each of them
            # buries the transitions that actually matter.
            key = (reading.physical, command.state, command.left, command.right)
            stale = (now - last_log_time) >= args.heartbeat_s
            log_every_due = args.log_every and (now - last_log_time) >= args.log_min_interval_s
            if log_every_due or key != last_logged or stale:
                ch_str = "".join(str(c) for c in reading.physical)
                status = "OK" if reading.visible else "LOST"
                where = f"{nav.junctions.pending.name[:14]:14s}"
                # pt/ac/cm: phase-tracker transition count, accumulated arc cm, and distance
                # since the last confirmed junction -- printed always (not just on request) so
                # a stuck run can be diagnosed against RouteJunction.min_phase_transitions/
                # .min_arc_cm/.min_cm_since_previous straight from the log, without guessing.
                current_phase = phase_progress.current
                if args.test_phase == 3:
                    assert phase3_completion is not None
                    if phase3_completion.mode == "arc":
                        phase_diag = "ph=3 pc=disabled"
                    else:
                        phase_diag = (
                            f"ph=4 proof={phase3_completion.phase4_valid_elapsed:.1f}/"
                            f"{phase3_completion.phase4_proof_s:.1f}s"
                        )
                else:
                    phase_diag = (
                        f"ph={current_phase.number} pc={phase_progress.phase_cm:.1f}cm"
                        if current_phase is not None
                        else "ph=done"
                    )
                diag = (
                    f"[{phase_diag} pt={nav._phase_transitions} ac={nav._arc_cm:.1f}cm "
                    f"cm={nav.junctions.cm_since_previous:.1f}cm]"
                )
                wall_stamp = datetime.now().astimezone().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                print(
                    f"[{wall_stamp}] [{elapsed:6.1f}s] #{frame_index:6d} "
                    f"{status:5s} P{ch_str} {reading.state.kind.value:9s} {where} -> "
                    f"{command.state.value:14s} L{command.left:4d} R{command.right:4d} "
                    f"{diag} | {command.reason}"
                )
                last_logged = key
                last_log_time = now
                logged_lines += 1

            if command.state is IRNavState.STOPPED:
                print(f"\nRoute complete: {command.reason}")
                break
            if command.state is IRNavState.FAILED:
                print(f"\nNAVIGATION SAFETY STOP: {command.reason}")
                break

            if args.duration and elapsed >= args.duration:
                print(f"\nDuration limit reached ({args.duration}s)")
                break

    except KeyboardInterrupt:
        print("\nStopped by operator")
    finally:
        if car:
            if not car.stop(best_effort=True):
                print("\n*** WHEELS MAY STILL BE TURNING — CUT POWER NOW ***")
            car.close()
        GPIO.cleanup()
        elapsed = time.monotonic() - start
        print()
        print("=" * 70)
        rate = frame_index / elapsed if elapsed else 0.0
        print(f"Test summary: {elapsed:.1f}s, {frame_index} cycles ({rate:.0f} Hz)")
        print(f"  Log lines written: {logged_lines}")
        print(f"  Line visible: {line_found_count} cycles")
        print(f"  Line lost: {line_lost_count} cycles")
        print(f"  Line-recovery searches: {search_entries}")
        print(f"  Off-track reverse-replays: {reverse_entries}")
        print(f"  Junctions taken: {nav.junctions_seen}  (last: {nav.last_junction or 'none'})")
        print(f"  Junctions rejected by the distance gate: {nav.junctions_rejected}")
        print(f"  Next junction expected: {nav.junctions.pending.name}")
        if args.test_phase == 3:
            assert phase3_completion is not None
            print(
                "  Phase 3 staged result: "
                f"mode={phase3_completion.mode}, distance stop disabled, "
                f"Phase 4 proof={phase3_completion.phase4_valid_elapsed:.1f}/"
                f"{phase3_completion.phase4_proof_s:.1f}s"
            )
        elif phase_progress.current is None:
            print("  Estimated route phase: distance plan complete")
        else:
            print(
                f"  Estimated route phase: {phase_progress.current.number} "
                f"({phase_progress.current.name}), {phase_progress.phase_cm:.1f}/"
                f"{phase_progress.current.distance_cm:.1f}cm"
            )
        if car:
            print(f"  I2C writes retried: {car.board.write_retries}")
        if drive_error:
            print(f"  Run ended early on a bus error: {drive_error}")
        noise_pct = 100 * nav.noise_frames / frame_index if frame_index else 0.0
        print(f"  Noise/hold frames: {nav.noise_frames} ({noise_pct:.1f}%)")
        if noise_pct > 5.0:
            print("    ^ over 5%: raise the sensor bar toward 2cm, or re-tune the pots.")
            print("      Non-contiguous readings cannot come from the line itself.")
        print("=" * 70)

    return 0


if __name__ == "__main__":
    sys.exit(main())
