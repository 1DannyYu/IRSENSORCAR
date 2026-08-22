#!/usr/bin/env python3
"""Measure the IR bar's physical layout by sweeping a black strip across it.

No motors. Reads only — safe to run over SSH with nobody beside the robot.

Nothing in this repository records which header pin (``Out1``..``Out4``) sits at
which physical sensor position (``P1``..``P4``), and the sensor spacing has been
re-adjusted by hand. This script recovers both from the sensors themselves: the
operator slides a black strip of *known width* across the bar, and the strip is
used as the ruler, so no external measurement is needed.

    duration a channel reads black  =  strip_width / sweep_speed

Averaging that over all four channels gives the sweep speed, and every other
distance follows from a time difference multiplied by it:

    spacing(A, B)   = speed x (midpoint_B - midpoint_A)
    blind band      = speed x (rise_B - fall_A)      # nothing sees the strip

The blind band is the interval where the strip has left one sensor but not yet
reached the next. It is the reason a car can sit squarely on the line and read
0000, so measuring it directly matters more than measuring the raw spacing.

Polarity (verified 2026-08-19 on this build, by continuous trace plus the
operator's simultaneous LED observation): white paper reads **HIGH** with the
board LED dark; a black strip, and an airborne sensor, both read **LOW** with
the LED lit. Readings are inverted here so 1 = black throughout, matching
``carbot.ir_tracing`` with ``invert={0,1,2,3}``.

The run refuses to start until every channel has read white continuously for
``--baseline-s``. A first attempt was wasted because the car was still parked on
the track line, so two channels read black for 118 of 150 seconds and every
derived number was meaningless. The gate makes that failure impossible rather
than merely documented.

Procedure:
  1. Put the car on plain white paper — all four board LEDs must be dark.
  2. Start this script; it waits for a clean baseline before recording.
  3. Slide the strip steadily from well outside one end of the bar to well
     outside the other. Keep it flat and keep the speed even.
  4. Pause ~1s, sweep back the other way. Repeat 2-3 times.

Usage:
    python3 42_ir_geometry_sweep.py --seconds 60 --strip-cm 2.0
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from itertools import pairwise

PINS = (24, 25, 22, 23)  # Out1, Out2, Out3, Out4 -> BCM GPIO
NAMES = ("Out1", "Out2", "Out3", "Out4")


def _reader(pins, invert):
    from RPi import GPIO

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in pins:
        GPIO.setup(p, GPIO.IN)

    def read():
        raw = tuple(GPIO.input(p) for p in pins)
        return tuple(1 - v if i in invert else v for i, v in enumerate(raw))

    return read, GPIO


def wait_for_baseline(read, hold_s, timeout_s):
    """Block until every channel reads white (0) continuously for `hold_s`."""
    deadline = time.monotonic() + timeout_s
    clean_since = None
    while time.monotonic() < deadline:
        if any(read()):
            clean_since = None
        elif clean_since is None:
            clean_since = time.monotonic()
        elif time.monotonic() - clean_since >= hold_s:
            return True
        time.sleep(0.01)
    return False


def record(read, seconds, hz):
    period = 1.0 / hz
    trace = []
    t0 = time.monotonic()
    nxt = t0
    while True:
        t = time.monotonic()
        if t - t0 >= seconds:
            break
        if t >= nxt:
            trace.append((t - t0, read()))
            nxt += period
    return trace


def debounce(trace, n_channels, hold):
    """Collapse per-channel chatter: a level must repeat `hold` times to count."""
    out = []
    state = list(trace[0][1])
    run = [0] * n_channels
    for t, bits in trace:
        for i in range(n_channels):
            if bits[i] == state[i]:
                run[i] = 0
            else:
                run[i] += 1
                if run[i] >= hold:
                    state[i] = bits[i]
                    run[i] = 0
        out.append((t, tuple(state)))
    return out


def intervals(trace, index):
    """Time spans where channel `index` reads black (normalised 1)."""
    spans, start = [], None
    for t, bits in trace:
        if bits[index] and start is None:
            start = t
        elif not bits[index] and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, trace[-1][0]))
    return spans


def split_passes(all_spans, gap_s):
    """Group spans from every channel into sweep passes separated by quiet gaps."""
    flat = sorted((a, b, i) for i, spans in enumerate(all_spans) for a, b in spans)
    if not flat:
        return []
    passes, cur, last_end = [], [flat[0]], flat[0][1]
    for span in flat[1:]:
        if span[0] - last_end > gap_s:
            passes.append(cur)
            cur = []
        cur.append(span)
        last_end = max(last_end, span[1])
    passes.append(cur)
    return passes


def analyse_pass(spans, strip_cm):
    """One sweep -> channel order, speed, spacings, blind bands."""
    best = {}
    for a, b, i in spans:  # keep the longest span per channel in this pass
        if i not in best or (b - a) > (best[i][1] - best[i][0]):
            best[i] = (a, b)
    if len(best) < 2:
        return None

    ordered = sorted(best.items(), key=lambda kv: (kv[1][0] + kv[1][1]) / 2)
    durations = [b - a for _, (a, b) in ordered]
    speed = strip_cm / statistics.fmean(durations)  # cm/s

    gaps = [
        (nxt_start - prev_end) * speed  # cm; negative = the two spans overlapped
        for (_, (_, prev_end)), (_, (nxt_start, _)) in pairwise(ordered)
    ]
    mids = [(a + b) / 2 for _, (a, b) in ordered]
    spacings = [(m2 - m1) * speed for m1, m2 in pairwise(mids)]

    return {
        "order": [i for i, _ in ordered],
        "durations": durations,
        "speed": speed,
        "spacings": spacings,
        "gaps": gaps,
        "t0": ordered[0][1][0],
        "spread": max(durations) / min(durations),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description="Measure IR bar geometry with a strip sweep")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--strip-cm", type=float, default=2.0, help="width of the black strip")
    ap.add_argument("--hz", type=float, default=2000.0, help="sampling rate cap")
    ap.add_argument("--hold", type=int, default=8, help="samples a level must repeat to count")
    ap.add_argument("--pass-gap-s", type=float, default=0.6)
    ap.add_argument("--max-span-s", type=float, default=3.0, help="longer spans are not sweeps")
    ap.add_argument("--baseline-s", type=float, default=2.0, help="all-white hold before recording")
    ap.add_argument("--baseline-timeout-s", type=float, default=120.0)
    ap.add_argument("--invert", default="0,1,2,3")
    args = ap.parse_args()

    invert = {int(x) for x in args.invert.split(",") if x.strip() != ""}
    read, gpio = _reader(PINS, invert)
    print(f"invert={sorted(invert)}   1 = black after inversion")

    print(f"waiting for all-white baseline ({args.baseline_s:.0f}s clean)...", flush=True)
    if not wait_for_baseline(read, args.baseline_s, args.baseline_timeout_s):
        now = read()
        black = [NAMES[i] for i, v in enumerate(now) if v]
        gpio.cleanup()
        print(f"BASELINE NEVER CLEAN — still reading black on: {', '.join(black) or 'none'}")
        print("Put the car on plain white paper (all four board LEDs dark) and re-run.")
        return 1

    print(f"baseline clean — recording {args.seconds:.0f}s, sweep now", flush=True)
    trace = record(read, args.seconds, args.hz)
    gpio.cleanup()
    print(f"samples={len(trace)}  rate={len(trace) / trace[-1][0]:.0f} Hz")

    raw_changes = sum(1 for a, b in pairwise(trace) if a[1] != b[1])
    clean = debounce(trace, len(PINS), args.hold)
    clean_changes = sum(1 for a, b in pairwise(clean) if a[1] != b[1])
    print(f"pattern changes: raw={raw_changes}  after debounce={clean_changes}")

    print("\n=== every black interval (after debounce) ===")
    all_spans = []
    for i in range(len(PINS)):
        spans = intervals(clean, i)
        kept = [(a, b) for a, b in spans if (b - a) <= args.max_span_s]
        dropped = len(spans) - len(kept)
        note = "" if not dropped else f", {dropped} over {args.max_span_s}s dropped"
        print(f"  {NAMES[i]} (GPIO{PINS[i]:2d}): {len(kept)} span(s){note}")
        detail = "  ".join(f"{a:.2f}-{b:.2f}s({b - a:.3f})" for a, b in kept[:12])
        if detail:
            print(f"      {detail}")
        all_spans.append(kept)

    passes = split_passes(all_spans, args.pass_gap_s)
    print(f"\ndetected {len(passes)} sweep pass(es)\n")

    results = []
    for n, spans in enumerate(passes, 1):
        r = analyse_pass(spans, args.strip_cm)
        if r is None:
            print(f"pass {n}: fewer than 2 channels triggered — ignored")
            continue
        results.append(r)
        flag = "  <-- uneven speed, treat with care" if r["spread"] > 2.5 else ""
        print(f"pass {n} @ t={r['t0']:.1f}s   speed={r['speed']:.1f} cm/s{flag}")
        print(f"  order: {' -> '.join(NAMES[i] for i in r['order'])}")
        print(
            "  black durations (s): "
            + ", ".join(f"{NAMES[i]}={d:.3f}" for i, d in zip(r["order"], r["durations"]))
        )
        print(
            "  centre spacing (cm): "
            + ", ".join(
                f"{NAMES[a]}-{NAMES[b]}={s:.2f}"
                for a, b, s in zip(r["order"], r["order"][1:], r["spacings"])
            )
        )
        print(
            "  BLIND band  (cm): "
            + ", ".join(
                f"{NAMES[a]}|{NAMES[b]}={g:+.2f}"
                for a, b, g in zip(r["order"], r["order"][1:], r["gaps"])
            )
        )
        print()

    full = [r for r in results if len(r["order"]) == len(PINS)]
    if not full:
        print("no pass triggered all four channels — sweep further past both ends")
        return 1

    print("=== aggregate (only passes that hit all four channels) ===")
    fwd = full[0]["order"]
    rev = list(reversed(fwd))
    same = [r for r in full if r["order"] == fwd]
    opp = [r for r in full if r["order"] == rev]
    odd = len(full) - len(same) - len(opp)
    print(f"passes in first order: {len(same)}   reversed: {len(opp)}   inconsistent: {odd}")
    print(f"\nphysical order along the bar: {' -> '.join(NAMES[i] for i in fwd)}")
    print("  (tell me which way you swept first and 'left' is pinned)")

    for label, group in (("first-direction", same), ("reverse-direction", opp)):
        if not group:
            continue
        order = group[0]["order"]
        print(f"\n{label} mean over {len(group)} pass(es):")
        for k, (a, b) in enumerate(pairwise(order)):
            sp = statistics.fmean(g["spacings"][k] for g in group)
            bl = statistics.fmean(g["gaps"][k] for g in group)
            print(f"  {NAMES[a]}-{NAMES[b]}: spacing {sp:.2f} cm   blind band {bl:+.2f} cm")
        print(f"  sweep speed {statistics.fmean(g['speed'] for g in group):.1f} cm/s")

    return 0


if __name__ == "__main__":
    sys.exit(main())
