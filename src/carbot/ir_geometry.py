"""Physical layout of the IR bar, and what each of the 16 readings means.

Everything the steering logic needs to know about the sensor's *physics* lives
here, separated from the navigation state machine so it can be unit-tested
without a robot.

Channel order
-------------
The driver reports channels in ``Out1..Out4`` order, which is **not** their
physical left-to-right order. Measured 2026-08-19 by sweeping a black card
across the bar and recording the order in which channels tripped — the leading
and trailing edges of the card gave the same order independently, and the
result agrees with the operator's separate observation that ``Out4`` is the
rightmost sensor:

    physical left -> right :  Out2   Out1   Out3   Out4
    position               :   P1     P2     P3     P4
    BCM GPIO               :   25     24     22     23
    Pi header pin          :   22     18     15     16

This is the exact mirror of the order previously recorded in
``ir_line_nav``, which had the bar as ``Out4 Out3 Out1 Out2``. With the old
order every steering correction was applied to the wrong side.

Geometry
--------
Spacings are operator-measured with a ruler: ``P1-P2 = 2.8 cm``,
``P2-P3 = 0.8 cm``, ``P3-P4 = 2.8 cm``, so the bar spans 6.4 cm. The route line
is 2.0 cm wide.

The consequence that drives the whole table: a 2 cm line **cannot cover two
sensors 2.8 cm apart**, so between P1 and P2 (and between P3 and P4) there is a
band where the line is under the bar but no channel sees it —

    blind band width = gap - line width = 2.8 - 2.0 = 0.8 cm, centred at +-1.8 cm

A car sitting squarely on the line reads ``0000`` inside that band. ``0000``
therefore does *not* mean "line lost", and the two cases are told apart by the
previous reading rather than by a timer: the line can only leave the bar past
an *outer* sensor, so ``0000`` after ``0010``/``0100`` is the blind band, while
``0000`` after ``0001``/``1000`` is a genuine loss.

Polarity
--------
Verified 2026-08-19: white paper reads GPIO HIGH with the board LED dark; a
black line, and an airborne sensor, both read LOW with the LED lit. So
``carbot.ir_tracing`` needs ``invert={0, 1, 2, 3}`` to produce 1 = black. An
airborne sensor reading the same as black is why undulating paper produces
false *black*, never false white.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

#: Indices into an ``Out1..Out4`` reading, ordered physically left to right.
#: ``Out2, Out1, Out3, Out4`` -> indices 1, 0, 2, 3.
PHYSICAL_ORDER: tuple[int, int, int, int] = (1, 0, 2, 3)

#: Lateral position of P1..P4 in cm; 0 is the bar centre, positive is right.
SENSOR_POSITIONS_CM: tuple[float, float, float, float] = (-3.2, -0.4, 0.4, 3.2)

#: Width of the printed route line.
LINE_WIDTH_CM = 2.0

#: Gap between an outer sensor and its neighbour, minus the line width: the
#: span of line-centre positions that no channel can see.
BLIND_BAND_CM = 2.8 - LINE_WIDTH_CM

#: Line-centre offset a blind-band reading corresponds to.
BLIND_OFFSET_CM = 1.8

#: Beyond this offset the line has left the bar entirely.
DETECTION_LIMIT_CM = 4.2


def to_physical(reading: tuple[int, ...]) -> tuple[int, int, int, int]:
    """Reorder an ``Out1..Out4`` reading into physical ``P1..P4`` order."""
    if len(reading) != 4:
        raise ValueError("expected a 4-channel reading")
    return tuple(reading[i] for i in PHYSICAL_ORDER)  # type: ignore[return-value]


class Kind(Enum):
    """What a reading means, which decides how the nav layer treats it."""

    ON_LINE = "on_line"  # centred, drive straight
    DRIFT = "drift"  # line offset by a known amount, steer proportionally
    AMBIGUOUS = "ambiguous"  # 0000 — blind band or lost, resolved by history
    JUNCTION = "junction"  # a second dark feature; sustained means a real junction
    NOISE = "noise"  # impossible from one 2 cm line, hold the previous command


@dataclass(frozen=True)
class IRState:
    """One of the 16 readings, with everything the steering layer needs."""

    bits: tuple[int, int, int, int]  # physical P1..P4, 1 = black
    kind: Kind
    #: Estimated line-centre offset in cm, positive = line is right of the bar
    #: centre (so the car has drifted left and must steer right). ``None`` when
    #: the reading does not localise the line.
    offset_cm: float | None
    #: Inner-wheel speed as a fraction of the base speed while correcting.
    inner_ratio: float
    label: str

    @property
    def direction(self) -> int:
        """+1 steer right, -1 steer left, 0 straight."""
        if self.offset_cm is None or self.offset_cm == 0.0:
            return 0
        return 1 if self.offset_cm > 0 else -1


def _s(bits, kind, offset, ratio, label):
    return IRState(bits, kind, offset, ratio, label)


#: The 16 readings, keyed by physical ``P1..P4`` bits. Offsets are the midpoint
#: of the line-centre window that produces each reading, derived from
#: :data:`SENSOR_POSITIONS_CM` and :data:`LINE_WIDTH_CM` rather than guessed.
STATE_TABLE: dict[tuple[int, int, int, int], IRState] = {
    # -- single 2 cm line, the only readings normal following can produce -----
    (0, 1, 1, 0): _s((0, 1, 1, 0), Kind.ON_LINE, 0.0, 1.00, "centred"),
    (0, 0, 1, 0): _s((0, 0, 1, 0), Kind.DRIFT, +1.0, 0.73, "drifting right, slight"),
    (0, 1, 0, 0): _s((0, 1, 0, 0), Kind.DRIFT, -1.0, 0.73, "drifting left, slight"),
    (0, 0, 0, 1): _s((0, 0, 0, 1), Kind.DRIFT, +3.2, 0.13, "far right, outer sensor only"),
    (1, 0, 0, 0): _s((1, 0, 0, 0), Kind.DRIFT, -3.2, 0.13, "far left, outer sensor only"),
    (0, 0, 0, 0): _s((0, 0, 0, 0), Kind.AMBIGUOUS, None, 0.40, "blind band or line lost"),
    # -- needs a second dark feature, or a badly skewed pass over a curve -----
    (1, 1, 1, 1): _s((1, 1, 1, 1), Kind.JUNCTION, 0.0, 1.00, "symmetric crossbar"),
    (0, 1, 1, 1): _s((0, 1, 1, 1), Kind.JUNCTION, +1.6, 0.40, "branch or curve on the right"),
    (1, 1, 1, 0): _s((1, 1, 1, 0), Kind.JUNCTION, -1.6, 0.40, "branch or curve on the left"),
    (0, 0, 1, 1): _s((0, 0, 1, 1), Kind.JUNCTION, +1.8, 0.40, "right pair, needs >2.8cm of black"),
    (1, 1, 0, 0): _s((1, 1, 0, 0), Kind.JUNCTION, -1.8, 0.40, "left pair, needs >2.8cm of black"),
    # -- non-contiguous black: impossible from one line, so never steer on it -
    (1, 0, 1, 0): _s((1, 0, 1, 0), Kind.NOISE, None, 1.00, "split dark regions"),
    (0, 1, 0, 1): _s((0, 1, 0, 1), Kind.NOISE, None, 1.00, "split dark regions"),
    (1, 0, 0, 1): _s((1, 0, 0, 1), Kind.NOISE, None, 1.00, "outer pair only"),
    (1, 0, 1, 1): _s((1, 0, 1, 1), Kind.NOISE, None, 1.00, "P2 dropped out"),
    (1, 1, 0, 1): _s((1, 1, 0, 1), Kind.NOISE, None, 1.00, "P3 dropped out"),
}

assert len(STATE_TABLE) == 16, "the table must be total over all 4-bit readings"

#: Readings that localise the line on the right / left, used to decide which
#: side a following ``0000`` fell off. Only the *outer* sensors mean the line
#: has actually left the bar.
BLIND_AFTER_RIGHT = {(0, 0, 1, 0)}
BLIND_AFTER_LEFT = {(0, 1, 0, 0)}
LOST_AFTER = {(0, 0, 0, 1), (1, 0, 0, 0)}


def classify(reading: tuple[int, ...], *, physical: bool = False) -> IRState:
    """Look up a reading. Pass ``physical=True`` if it is already P1..P4 order."""
    bits = tuple(reading) if physical else to_physical(reading)
    if len(bits) != 4 or any(b not in (0, 1) for b in bits):
        raise ValueError(f"not a 4-channel binary reading: {reading!r}")
    return STATE_TABLE[bits]  # type: ignore[index]


def resolve_blind(previous: tuple[int, int, int, int] | None) -> tuple[str, float | None]:
    """Decide what a ``0000`` means from the previous physical reading.

    Returns ``(verdict, offset_cm)`` where verdict is one of ``"blind"``,
    ``"lost"`` or ``"hold"``. The line can only leave the bar past an outer
    sensor, so the previous reading is a stronger signal than elapsed time.
    """
    if previous is None:
        return "lost", None
    if previous in BLIND_AFTER_RIGHT:
        return "blind", +BLIND_OFFSET_CM
    if previous in BLIND_AFTER_LEFT:
        return "blind", -BLIND_OFFSET_CM
    if previous in LOST_AFTER:
        return "lost", None
    # 0110 -> 0000 in one step is not reachable by drifting; the line cannot
    # jump the 0010/0100 windows. Treat it as paper undulation lifting every
    # channel out of range, which reads as black on this hardware.
    return "hold", None


def wheel_speeds(speed: int, direction: int, inner_ratio: float) -> tuple[int, int]:
    """Left/right wheel speeds for a correction of the given strength."""
    inner = round(speed * inner_ratio)
    if direction > 0:
        return speed, inner
    if direction < 0:
        return inner, speed
    return speed, speed
