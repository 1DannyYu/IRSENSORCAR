"""The Task-1 junction sequence: what to do at each junction, and when one counts.

Why the sequence decides, and not the reading
---------------------------------------------
The first attempt keyed the action off the reading, with one ``in_roundabout`` boolean to
separate the two junctions that share a signature. The 2026-08-19 track run showed all three
of its premises were false:

* The T junction reached from the start-box stem reads ``1111``, not ``0111`` — the crossbar
  runs both east and west, so it is symmetric. ``1111`` was assumed unique to the roundabout
  entry, so the very first junction of the run set the flag the wrong way.
* The roundabout entry that run read ``0111``, not ``1111`` — the car arrived skewed. The two
  signatures the design leaned on had swapped places.
* The roundabout produced four more sustained ``1111`` events. "Re-synchronise on every
  ``1111``" only works if ``1111`` is rare, and it is the most common junction reading there is.

Result: six junction events, six right turns, zero of the straight-through crossings the route
needs. The reading simply does not carry enough information — the same local pattern means
"turn" in one place and "straight on" in another.

What does carry it is *where the car is in the lap*, and the route is fixed and known. So the
reading's only job here is "a junction is under the bar"; the action comes from the sequence,
and a distance gate rejects re-reads of the junction just handled. The gaps between junctions
differ by more than 6x, which is what makes a coarse distance estimate enough to hold the
sequence together.

2026-08-20, second pass: still failed, two more premises down
--------------------------------------------------------------------
Running the two-lap plan end to end on real hardware did not complete. Two more assumptions
this design still leaned on turned out false:

* **The roundabout exit's dwell timer never completed.** The exit produced ``0111``, ``1001``,
  ``1111`` and ``1110`` across the approach, in no fixed order. ``0111``/``1111``/``1110`` are
  ``Kind.JUNCTION`` and fed the dwell counter; ``1001`` is ``Kind.NOISE`` (not a signature one
  2cm line can produce) and was outside it. Every time a ``1001`` frame landed between
  qualifying ones, the dwell counter reset to zero. The sustained bar was real; the counter
  just never survived long enough to see it.
* **Steering did not stop just because a reading was junction-shaped.** Before the dwell timer
  finished (and, separately, whenever the distance gate rejected an early reading), the nav
  layer was still steering proportionally on the confirming reading's offset. That offset is
  derived from where a single straight 2cm line sits under the bar; it does not describe a
  roundabout exit curve or a T-junction crossbar, so correcting on it pulled the car off its
  approach before the route-driven turn/cross ever got to run.

2026-08-20, third pass: the dwell-on-any-reading model was wrong at the root
------------------------------------------------------------------------------
Both fixes above patched the *dwell* model (confirm on one qualifying reading, sustained for a
time threshold) without questioning whether "one reading, sustained" is what these junctions
actually produce. Direct signal tracing on the real track (operator watching the printed P1..P4
log line by line) showed it is not: every one of these junctions produces an **ordered sequence
of distinct readings**, several of which are not junction-shaped at all under
``carbot.ir_geometry``'s single-line model:

* **Roundabout exit**: ``0111 -> 0101 -> 0100 -> 0110``. ``0101`` is ``Kind.NOISE`` ("no single
  line can produce this") and ``0100``/``0110`` are ordinary ``Kind.DRIFT``/``Kind.ON_LINE``
  readings — indistinguishable from normal FOLLOW in isolation. Only the *order* they appear in
  is the real signal; a single-reading dwell check can never see this, no matter how the confirm
  set is widened.
* **Roundabout entry**: ``1111`` (1.5-1.8cm) -> ``1001`` (0.2cm) -> ``0000``. The same
  ``1111``-then-clear-to-``0000`` shape as the T junctions below, but with a distinctive
  ``1001`` shoulder in between that the plain T junctions do not produce.
* **Start-stem T junction**: ``1111`` (2cm) -> ``0000``.
* **The lap-crossing T junction (Phase 10 approach)**: a *different* sequence from the
  start-stem T, despite being the same physical junction — ``0111`` (not ``1111``) sustained
  2cm, then ``0110``. The stem approaches the T head-on (symmetric crossbar, reads ``1111``);
  Phase 10 approaches at the return leg's angle, off-centre (asymmetric, reads ``0111``). This
  also means, per this same real-track data, the lap-crossing case needs **no turn at all** to
  confirm arrival — reaching ``0110`` after the 2cm hold *is* arrival, immediately followed by
  the route-driven CROSS/STOP action.

``SequenceStep``/the ``approach`` field below replace ``confirm_signatures`` and the old
sustained-dwell timer with a small ordered state machine per junction: each step names the
physical reading expected and the minimum distance it must persist before the sequence trusts
it and advances. Distance gating (``min_cm_since_previous``) is unchanged and still runs as an
outer sanity check once a sequence completes — sequence-matching says *this looks like the
junction shape*, the gate still says *and we're far enough from the last one for that to be
believable*.

The turn itself also changes for the three junctions that do turn (the start-stem T, the
roundabout entry, and the roundabout exit): the same real-track tracing showed the turn ending
in a distinctive re-centring sequence (``0001 -> 0011 -> 0000 -> 0010 -> 0110``), so
``IRLineNav`` now turns closed-loop — watching for ``0110`` — instead of a pure timed spin. See
the ``IRLineNav._turn_step`` docstring for why this does not reintroduce the "any channel
visible" false-early-exit bug the original timed design was built to avoid.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class JunctionAction(Enum):
    """What the car does at a junction the route has reached."""

    TURN_RIGHT = "turn_right"
    TURN_LEFT = "turn_left"
    CROSS = "cross"  # keep going straight; the extra black is off to one side
    STOP = "stop"  # the planned lap count ends here; halt


@dataclass(frozen=True)
class SequenceStep:
    """One step of an ordered approach/turn-completion sequence.

    ``bits`` is the physical P1..P4 reading this step expects; ``min_cm`` is how far that
    exact reading must persist (estimated travel) before the sequence trusts it and advances
    to the next step. Most steps here have ``min_cm=0`` — real-track tracing showed most of
    these transitions happen fast enough that requiring persistence would just make the
    sequence miss them; only the readings the operator specifically measured a hold distance
    for (the initial crossbar/roundabout signatures) carry one.
    """

    bits: tuple[int, int, int, int]
    min_cm: float = 0.0


#: Closed-loop turn-completion signal sequence.
#: Every turning junction (a/e/f below) ends the same way once the car has swept back onto
#: the new heading's line: 0001 -> 0011 -> 0000 -> 0010 -> 0110. Reaching the last step
#: (0110, ordinary centred FOLLOW) is what ends the turn -- see IRLineNav._turn_step. The
#: middle steps are not gated on individually; they are what real-track tracing showed
#: happens on the way there, kept here only as documentation of the observed sweep.
TURN_COMPLETE_READING: tuple[int, int, int, int] = (0, 1, 1, 0)


@dataclass(frozen=True)
class RouteJunction:
    name: str
    action: JunctionAction
    #: Distance the car must have covered since the previous junction before this one can be
    #: accepted. Set to roughly half the true spacing so a slow or wandering lap still clears
    #: it, while a second reading of the junction just handled does not. Runs as an outer
    #: sanity check once `approach` below completes -- unchanged by the 2026-08-20 sequence
    #: rewrite.
    min_cm_since_previous: float
    #: Ordered signal sequence this junction actually produces on approach, from real-track
    #: tracing (2026-08-20) -- see the module docstring. The car does not need to react
    #: differently to each step; IRLineNav just holds its last steady FOLLOW command while
    #: matching one so a still-only-partially-confirmed reading never feeds the generic
    #: offset-based correction (see IRLineNav._approach_step).
    approach: tuple[SequenceStep, ...]
    #: Blind straight distance after the approach sequence completes, before the turn starts.
    #: 0 for a junction with no turn (the lap-crossing T, CROSS/STOP).
    creep_cm: float = 0.0
    #: Expected turn magnitude in degrees -- used ONLY to bound the closed-loop turn's safety
    #: timeout (IRNavPolicy.turn_timeout_s). The turn itself ends on TURN_COMPLETE_READING,
    #: not this angle; 0 for CROSS/STOP junctions, which do not turn at all.
    turn_deg: float = 0.0
    #: Precondition on IRLineNav's straight/arc phase tracker (2026-08-20 planning doc,
    #: tasks/ir-sensor-tracking/phase-tracking-and-junction-detection-plan.md) -- this
    #: junction's `approach` sequence is not even attempted until the tracker's mode-flip
    #: count since the last accepted junction reaches this. 0 = no precondition (the distance
    #: gate alone decides). Only "roundabout entry" uses this today (Phase 6 + ARC 3 done).
    min_phase_transitions: int = 0
    #: Precondition on the phase tracker's accumulated arc distance (`_arc_cm`) since the last
    #: accepted junction -- for a junction reached only after one long continuous curve (the
    #: roundabout traversal), where mode-flip counting doesn't apply (it never flips back to
    #: straight until the exit). 0.0 = no precondition. Only "roundabout exit" uses this today
    #: (Phase 9's ~78.9cm traversal).
    min_arc_cm: float = 0.0
    #: 2026-08-20, twelfth pass, real-track: the operator confirmed Phase 8 barely exists as a
    #: straight -- ARC 3 blends almost directly into the roundabout's own curve, so the car
    #: physically never gets a chance to produce the `approach` sequence's symmetric 1111
    #: crossbar (a real run accumulated 287.5cm of continuous arc, ~3.6x the roundabout's own
    #: circumference, before ever seeing one). If > 0, IRLineNav._approach_step bypasses
    #: alternative evidence that opens `approach` bit-matching when the phase-transition count
    #: is unreliable. It never accepts a junction by itself: a real sensor approach sequence is
    #: still required before any turn. 0.0 = no alternative gate. Only "roundabout entry" uses
    #: this today.
    arc_trigger_cm: float = 0.0

    @property
    def turn_direction(self) -> int:
        """+1 right, -1 left, 0 straight — the convention `wheel_speeds` already uses."""
        if self.action is JunctionAction.TURN_RIGHT:
            return 1
        if self.action is JunctionAction.TURN_LEFT:
            return -1
        return 0


@dataclass(frozen=True)
class RoutePlan:
    """A one-time prologue, then a loop that repeats for as long as the car runs."""

    prologue: tuple[RouteJunction, ...]
    loop: tuple[RouteJunction, ...]

    def __post_init__(self) -> None:
        if not self.loop:
            raise ValueError("a route needs at least one junction in its loop")

    def at(self, index: int) -> RouteJunction:
        """The junction at `index`, counting the prologue first and then cycling the loop."""
        if index < 0:
            raise ValueError("junction index must be non-negative")
        if index < len(self.prologue):
            return self.prologue[index]
        return self.loop[(index - len(self.prologue)) % len(self.loop)]


# ---------------------------------------------------------------------------------------
# Task-1 junction map
#
# Where each named corner/junction of the lap lives in code, and how it's decided.
# All four physical junctions (with g/h sharing one location) use signal sequences measured
# on 2026-08-20; see the module docstring's "third pass" section.
#
#   a. Start-stem T junction, ~90 degree right turn
#      -> TASK1_ROUTE.prologue[0] below. Runs once, on lap 1 only.
#      approach: 1111(2cm) -> 0000. creep 8.5cm -> closed-loop right turn (~90°, watch 0110).
#      Turn itself: IRLineNav._commit_junction -> _creep_step -> _turn_step
#
#   b/c/d. ARC 1 SE / ARC 2 NE / ARC 3 NW corners
#      NOT a junction -- continuous printed curve, sensor tracks it the whole way.
#      -> TASK1_CORNER_WINDOWS below (temporary speed/gain boost, never a blind turn).
#
#   e. Phase 8 entry into the roundabout, right turn
#      -> TASK1_ROUTE.loop[0] below ("roundabout entry").
#      approach: 1111(1.65cm) -> 1001(0.2cm) -> 0000. creep 8.0cm -> closed-loop right turn
#      (~42.5°, watch 0110).
#
#   f. Roundabout exit into Phase 10, right turn
#      -> TASK1_ROUTE.loop[1] below ("roundabout exit").
#      approach: 0111 -> 0101 -> 0100 -> 0110 (car heading ~north here). creep 6.5cm ->
#      closed-loop right turn (~90°, watch 0110).
#
#   g. Lap 2+: Start~T junction, NO turn, straight through into Phase 2
#      -> TASK1_ROUTE.loop[2] below ("T junction"), JunctionAction.CROSS.
#      approach: 0111(2cm) -> 0110. No creep, no turn -- reaching 0110 IS arrival.
#
#   h. Final lap: car stops centred on the T junction, task complete
#      -> task1_route_for_laps() below synthesizes a JunctionAction.STOP entry from loop[2]
#      ("T junction" -> "final T junction"). Same approach sequence as g, since it's
#      physically the same junction, just the last time through.
# ---------------------------------------------------------------------------------------
TASK1_ROUTE = RoutePlan(
    # 2026-08-20 correction: this used to be 0.0 with the reasoning "no previous junction to
    # mistake it for" -- true, but that only rules out *re-reading* a junction, not a spurious
    # trigger before the car has gone anywhere (e.g. noise off the departure-box printing, or a
    # motor-start electrical transient). Photo evidence (2026-08-20) put the sensor roughly
    # 6cm from the T at rest -- half of that, ~3cm, follows the same "half the real distance"
    # rule used for every other gate below.
    # Corrected 2026-08-20 from 0.0. With no gate, noise could be accepted before the car
    # moved at all. The stationary sensor is about 6 cm from the T, so 3 cm follows the
    # same "half the physical distance" convention used by the other gates.
    prologue=(
        # (a) Start-stem T junction, ~90 degree right turn on lap 1 only.
        RouteJunction(
            "start stem T junction",
            JunctionAction.TURN_RIGHT,
            3.0,
            # 2026-08-20, sixth pass, real-track: the original two-step (1111, then 0000)
            # sequence let sensor flicker mid-crossbar reset the 1111 accumulation on every
            # stray 0000 blip -- exactly the readings the 2-second off-track dwell timer was
            # also counting, so the car oscillated forward/backward at the junction instead of
            # committing to the turn. Fixed at the mechanism level instead of dropping the
            # 0000 step: IRLineNav._approach_step now ignores the sensor reading entirely once
            # a step has genuinely started, until the last 0.1s of that step's window -- so a
            # stray blip mid-accumulation no longer resets anything.
            # 2026-08-20, eighth pass: back to two steps with that fix in place -- sustained
            # 1111 for 0.15-0.19s (1.5-1.9cm at 10cm/s, the operator's measured range), then a
            # confirmed 0000 for more than 0.05s (0.5cm), before committing to creep+turn.
            # Deliberately NOT the same 1111 value as the roundabout entry's 1.65cm below --
            # different approach geometry (head-on stem vs. the entry's own angle), the
            # operator measured them separately.
            approach=(
                SequenceStep((1, 1, 1, 1), min_cm=1.7),
                SequenceStep((0, 0, 0, 0), min_cm=0.5),
            ),
            creep_cm=8.5,
            turn_deg=90.0,
        ),
    ),
    loop=(
        # (e) Phase 8 entry -> roundabout, right.
        RouteJunction(
            "roundabout entry",
            JunctionAction.TURN_RIGHT,
            60.0,
            approach=(
                SequenceStep((1, 1, 1, 1), min_cm=1.65),
                SequenceStep((1, 0, 0, 1), min_cm=0.2),
                SequenceStep((0, 0, 0, 0)),
            ),
            creep_cm=8.0,
            turn_deg=42.5,
            # Phase 2 -> ARC1 -> Phase4 -> ARC2 -> Phase6 -> ARC3 -> Phase8 is 6 straight/arc
            # mode flips since the start-stem T (or the previous lap's T-junction cross) --
            # reaching flip 6 means Phase 6 and ARC 3 are both confirmed done. See the
            # 2026-08-20 planning doc.
            min_phase_transitions=6,
            # 2026-08-20, twelfth pass: Phase 8 does not exist as a real straight -- ARC 3
            # blends directly into the roundabout's own curve, so `approach`'s symmetric 1111
            # crossbar is never reliably produced (a real run rode 287.5cm of continuous arc,
            # ~3.6x the roundabout's own circumference, without ever seeing one). 20.0cm is
            # comfortably past any single arc's real length (~12cm each) but well short of a
            # full lap of the roundabout. This only opens the real approach matcher; it cannot
            # perform a turn without sensor evidence -- see RouteJunction.arc_trigger_cm and
            # IRLineNav._approach_step. First estimate, re-tune from real track logs.
            arc_trigger_cm=20.0,
        ),
        # (f) Roundabout exit -> Phase 10, right.
        RouteJunction(
            "roundabout exit",
            JunctionAction.TURN_RIGHT,
            40.0,
            approach=(
                SequenceStep((0, 1, 1, 1)),
                SequenceStep((0, 1, 0, 1)),
                SequenceStep((0, 1, 0, 0)),
                SequenceStep((0, 1, 1, 0)),
            ),
            creep_cm=6.5,
            turn_deg=90.0,
            # Phase 9 (the roundabout traversal, ~84.8cm) is one continuous curve -- it never
            # flips back to straight mode until the exit, so mode-flip counting doesn't apply
            # here the way it does for "roundabout entry" above. 68.0cm (~80% of 84.8cm)
            # confirms most of the traversal is done without demanding an exact match -- see
            # the "probabilistic, not exact" governing principle in the planning doc.
            min_arc_cm=68.0,
        ),
        # (g on lap 2+, h on the final lap) T junction: cross straight through, or --
        # on the last lap -- stop here. Which one applies is decided by
        # task1_route_for_laps() below, not by this entry itself. No turn either way, so
        # no creep_cm/turn_deg -- reaching the approach sequence's last step IS arrival.
        # (g/h) Cross the T, or stop there on the final lap. task1_route_for_laps()
        # decides which action applies. No turn is needed; completing the approach is arrival.
        RouteJunction(
            "T junction",
            JunctionAction.CROSS,
            10.0,
            approach=(SequenceStep((0, 1, 1, 1), min_cm=2.0), SequenceStep((0, 1, 1, 0))),
        ),
    ),
)


@dataclass(frozen=True)
class CornerWindow:
    """A stretch of continuous curve too tight for the steady-state follow gains.

    This is NOT a scripted/blind turn -- the sensor keeps reading the printed line the whole
    way through. It only asks IRLineNav to drive slower and correct harder while the estimated
    position falls inside the window, so the reactive proportional steering (carbot.ir_geometry)
    can actually keep the bar on a curve tighter than its steady-state gains were tuned for.

    ``start_cm``/``end_cm`` are cm since the *pending* junction's own last accept (i.e.
    ``JunctionSequencer.cm_since_previous``), not since some junction named here -- the window
    is keyed to ``while_pending`` (which junction is currently expected next) because the
    physical stretch a corner sits on is walked once per lap while the junction the car most
    recently left differs by lap (the start-stem T on lap 1's prologue, the crossed T junction
    every lap after), whereas the pending one (roundabout entry) does not.
    """

    name: str
    while_pending: str
    start_cm: float
    end_cm: float
    speed_scale: float = 1.0
    inner_ratio_scale: float = 1.0


#: 2026-08-20: continuous proportional line-following was not tight enough to track these three
#: corners -- ARC 1's radius (~2.3cm, back-computed from its old, wrong 3.6cm arc-length
#: estimate over a 90deg heading change) is smaller than the car's own footprint, so at full
#: speed and steady-state gains the car ran wide off the curve and off the map before the
#: correction caught up. These windows slow down and sharpen the correction for that stretch
#: without ever stopping line tracking or scripting a blind turn -- unlike a T junction, a
#: single continuous curve gives the sensor everything it needs; it just needs a tighter
#: response to keep up.
#:
#: Distances (cm since the pending "roundabout entry" junction's last accept) are the operator's
#: 2026-08-20 direct re-measurement (superseding the original docs/task1-single-source-of-truth.md
#: section 3 figures, which were wrong for both the straights and the arcs): Phase 2 = 15.5,
#: ARC 1/2/3 all ~12.0 (same shape, only position differs -- not the old 3.6/14.2/7.2), Phase 4
#: = 18.0, Phase 6 = 47.0, Phase 8 = 7.5. Cumulative: Phase2 0-15.5, ARC1 15.5-27.5, Phase4
#: 27.5-45.5, ARC2 45.5-57.5, Phase6 57.5-104.5, ARC3 104.5-116.5, Phase8 116.5-124.0. Margins
#: below widen for later corners because the 10cm/s distance estimate drifts further from the
#: real position the longer the car has driven since the last confirmed junction. Margins and
#: scale factors are a first estimate, not a measured constant -- re-tune from real track logs.
#: See tasks/ir-sensor-tracking/phase-tracking-and-junction-detection-plan.md.
TASK1_CORNER_WINDOWS: tuple[CornerWindow, ...] = (
    CornerWindow("ARC 1 SE corner", "roundabout entry", 13.0, 30.0, 0.6, 0.5),  # (b)
    CornerWindow("ARC 2 NE corner", "roundabout entry", 42.0, 61.0, 0.6, 0.5),  # (c)
    CornerWindow("ARC 3 NW corner", "roundabout entry", 101.0, 120.0, 0.6, 0.5),  # (d)
    # 2026-08-20, eleventh pass, real-track: Phase 9 (the roundabout's own 270 degree
    # traversal) had no corner window at all, so the car corrected at the same strength as a
    # straight and stayed pinned on the widest DRIFT reading (1000/0001, offset +-3.2cm) the
    # whole way around -- never recentring to 0110, which made the roundabout exit's own 0111
    # harder to pick out of the noise. Arc length from the operator's measured inner black-line
    # diameter: circumference = pi * 33.5cm = 105.24cm, 270/360 of that = 78.93cm. Margins
    # trimmed off both ends: ~5cm at the start (creep/turn settling) and ~4cm at the end
    # (leave the exit approach's own crossbar detection unaffected by the sharper turn). Same
    # scale factors as ARC 1-3, first estimate -- re-tune from real track logs.
    CornerWindow("roundabout traversal", "roundabout exit", 5.0, 75.0, 0.6, 0.5),  # (f/g)
)


#: The same lap without the one-time stem out of the start box, for starting the car already
#: on the east-west line facing east. The prologue T is the junction that contaminated the
#: 2026-08-19 run's sequence, so dropping it isolates the loop logic from that interaction.
TASK1_LOOP_ONLY = RoutePlan(prologue=(), loop=TASK1_ROUTE.loop)


def task1_route_for_laps(laps: int, *, start_on_loop: bool = False) -> RoutePlan:
    """Task 1 driven `laps` times, halting at the T junction that closes the last lap.

    The stop is a specific entry in the sequence, not a tally compared against a target,
    for the same reason the actions are: a counter that slips once stays wrong. Every
    junction before the last is spelled out in the prologue, so reaching the stop means the
    car has actually been through all of them. `RoutePlan.at` cycles the one-entry loop, so
    once the route is complete it stays complete.
    """
    if laps < 1:
        raise ValueError("laps must be at least 1")
    base = TASK1_LOOP_ONLY if start_on_loop else TASK1_ROUTE
    # (h) Final lap: the T junction that would normally CROSS (g) becomes a STOP instead --
    # same physical junction, same approach sequence, just the last time through.
    final_t = RouteJunction(
        "final T junction",
        JunctionAction.STOP,
        base.loop[-1].min_cm_since_previous,
        approach=base.loop[-1].approach,
    )
    return RoutePlan(
        prologue=base.prologue + base.loop * (laps - 1) + base.loop[:-1],
        loop=(final_t,),
    )


class JunctionSequencer:
    """Tracks which junction is next and how far the car has come since the last one."""

    def __init__(self, plan: RoutePlan | None = None) -> None:
        self.plan = plan or TASK1_ROUTE
        self.index = 0
        self.cm_since_previous = 0.0

    @property
    def pending(self) -> RouteJunction:
        """The junction the route says comes next."""
        return self.plan.at(self.index)

    def travel(self, cm: float) -> None:
        """Add forward distance. Pure rotation must not be counted."""
        if cm < 0:
            raise ValueError("distance travelled must be non-negative")
        self.cm_since_previous += cm

    def shortfall_cm(self) -> float:
        """How much further the car must travel before the pending junction is believable."""
        return max(0.0, self.pending.min_cm_since_previous - self.cm_since_previous)

    def accept(self) -> RouteJunction:
        """Consume the pending junction and start measuring toward the next one."""
        junction = self.pending
        self.index += 1
        self.cm_since_previous = 0.0
        return junction
