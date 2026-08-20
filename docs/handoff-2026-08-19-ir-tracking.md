# Handoff — IR Line Tracking, Task-1 Continuous Loop

Date: 2026-08-19

## Read First

1. [docs/progress/2026-08-19-ir-channel-order-and-state-table.md](progress/2026-08-19-ir-channel-order-and-state-table.md)
   — what changed today and why, including the measurements that proved it
2. [docs/hardware/ir-tracing-sensor.md](hardware/ir-tracing-sensor.md)
   — wiring, the position ↔ channel ↔ GPIO table, polarity, blind band
3. [tasks/ir-sensor-tracking/design.md](../tasks/ir-sensor-tracking/design.md)
   — the 16-reading table and the junction sequencing
4. [tasks/ir-sensor-tracking/run-book.md](../tasks/ir-sensor-tracking/run-book.md)
   — the operating procedure this handoff resumes
5. [CLAUDE.md](../CLAUDE.md) hard rule 3 — motor-moving programs need an
   operator beside the robot who can cut power instantly

## Headline

The committed IR channel order was **mirror-reversed**, so every steering
correction was applied to the wrong side. It is fixed and unit-tested, but
**nothing has been driven under the corrected model.** The whole point of the
next session is to find out whether the car now steers toward the line.

## State

### Git

| Where | State |
|---|---|
| `origin/main` | everything below is pushed: the channel-order fix and 16-state table (`cd716a3`), the route spec losing Phase 11 (`d11972e`), and the loop rate limit |
| Local working tree | clean |
| Pi `~/Car-and-Robotic-Arm` | `4ca4eae` — **far behind**, and dirty: `examples/08`, `examples/14`, `examples/20`, `src/carbot/sonar.py`, `src/carbot/vision.py` all modified locally by someone else |

### How the new code got onto the Pi

The Pi's repo was **not** touched. The corrected code was rsync'd to a scratch
copy instead:

```bash
ssh carpi 'rm -rf /tmp/carbot-test && mkdir -p /tmp/carbot-test'
rsync -a src/ carpi:/tmp/carbot-test/src/
rsync -a examples/39_map1_ir_line_follow.py examples/36_ir_tracing_check.py \
         examples/42_ir_geometry_sweep.py carpi:/tmp/carbot-test/
```

Everything then runs as `cd /tmp/carbot-test && PYTHONPATH=src python3 39_...`.

> **`/tmp` on this Pi is a 2 GB tmpfs and is wiped on reboot** — which happened
> once today at 11:33 and silently deleted a script mid-session, wasting a test
> run. Re-run the rsync above at the start of every session, and verify with
> `ls /tmp/carbot-test/src/carbot/ir_geometry.py` before trusting any result.
> If `ir_geometry.py` is missing, the run is using the old mirrored order.

Resolving the Pi's own repo (stash or commit those five files, then pull) is
the cleaner long-term fix but was deliberately left alone — they are someone
else's uncommitted changes.

## What Is Verified

| Item | Evidence |
|---|---|
| Polarity `invert={0,1,2,3}` | Continuous trace: white paper → raw `1111`, LED dark; black and airborne → LOW, LED lit |
| Physical order `Out2 Out1 Out3 Out4` | Card sweep; leading and trailing edges agreed independently; operator confirmed `Out4` is rightmost |
| Position ↔ GPIO table | `P1`=`Out2`=GPIO25, `P2`=`Out1`=GPIO24, `P3`=`Out3`=GPIO22, `P4`=`Out4`=GPIO23 |
| 16-state table | 42 unit tests; a property test asserts the noise class is exactly the non-contiguous readings |
| Car centred in the start box reads `P0110` | 300 samples, 100%, classified `on_line`, zero transitions |
| Full suite | 445 pass, ruff clean |

## What Is NOT Verified

- **Steering direction on a moving car.** This is the whole reason the fix
  exists and it has never been exercised.
- **Junction sequencing.** The `in_roundabout` boolean has never seen a real
  `1111`.
- **Wheel speeds 110 / 60 / 20 (inner).** Proposed from the geometry, never
  tuned.
- **Spacing `2.8 / 0.8 / 2.8 cm`.** Operator ruler measurement. The sweep could
  not confirm it — the card used was wider than the 6.4 cm bar so there was no
  known scale, and the hand sweep averaged 0.56 cm/s with visibly uneven speed.

## Fixed: The Free-Running Control Loop

A 148-second dry run produced **6.2 million frames** (~42 kHz) and a **608 MB
log** — minutes from filling the 2 GB tmpfs, and unreadable as evidence. The
logic was unaffected (`junction_min_s` still accumulated correctly over ~6300
frames) but no run could be used to judge anything.

Fixed in `examples/39_map1_ir_line_follow.py`:

- `--hz` (default **100**) rate-limits the loop. Not arbitrary: at 10 cm/s the
  0.8 cm blind band takes 0.08 s to cross, so 100 Hz still samples it 8 times.
- Logging is **on state change** plus a 2 s heartbeat. A car tracking a straight
  line holds one reading for thousands of cycles, and printing each of them
  buries the transitions that matter. `--log-every` restores per-cycle output.
- The closing summary now prints the achieved rate and the line count, so a
  regression is visible immediately.

Measured over the same 20 s window:

| | Before | After |
|---|---|---|
| Loop rate | ~42,000 Hz | **99 Hz** |
| Cycles | ~840,000 | **1,989** |
| Log size | ~82 MB | **1.5 KB** |
| Log lines | one per cycle | **10** |

That run also confirmed the static noise floor is clean: 1,989 cycles parked on
the line in the start box, **0 noise frames, 0 lost frames**.

The earlier dry run produced no route data because the car was never pushed —
6.8 million frames of `P0110 on_line`. Not a fault, just an aborted attempt.

## Acceptance Gates

Run in order. Do not skip a gate.

### Gate 1 — sensor sanity (no motors, SSH safe) ✅ already passed

Car on the line in the start box:

```bash
cd /tmp/carbot-test && PYTHONPATH=src python3 36_ir_tracing_check.py \
    --pins 24,25,22,23 --invert 0,1,2,3 --count 30 --interval 0.3
```

Pass: white → `0 0 0 0`; on the line → middle two = 1; airborne → `1 1 1 1`.

### Gate 2 — dry run, car pushed by hand (no motors)

With the car pushed by hand around one full lap:

```bash
cd /tmp/carbot-test && PYTHONPATH=src python3 39_map1_ir_line_follow.py \
    --dry-run --duration 180 --invert 0,1,2,3
```

The log is now transitions-only, so a clean lap should be a few dozen lines —
if it is thousands, the rate limit is not in effect and the deployment is stale.

Pass conditions, all four required:

1. **Steering direction correct** — push the car so it drifts *right* of the
   line; the log must show `P0010` / `P0001` and the command must slow the
   **right** wheel (`R` < `L`). If it slows the left wheel, the order is still
   wrong and everything downstream is suspect. **This is the single most
   important check in this handoff.**
2. `P0110` dominates while tracking the line.
3. Exactly **three** junction events per lap: `P1111` (roundabout entry),
   `P0111` (roundabout exit, `RND` column set), `P0111` (T junction, `RND`
   clear, crossed straight).
4. **Noise/hold frames < 5%** in the closing summary. The script prints a
   warning above that threshold.

### Gate 3 — motor check (wheels LIFTED, operator beside the car)

🔴 Not to be started remotely. Lift all four wheels or secure the chassis.

```bash
cd ~/Car-and-Robotic-Arm && PYTHONPATH=src python3 examples/37_map1_motor_test.py
```

### Gate 4 — live lap (operator beside the car, hand on the power cut)

🔴 Only after gates 1–3 pass.

```bash
cd /tmp/carbot-test && PYTHONPATH=src python3 39_map1_ir_line_follow.py \
    --duration 120 --invert 0,1,2,3
```

Start the car on the line in the start box facing north. After the first
`1111` it laps continuously and **never returns to the start box**.

## Recommended Before Gate 4

**Raise the sensor bar to ~2 cm.** It currently sits at ~1 cm, the near edge of
the 1–3 cm detection range, so undulating paper pushes channels out of range in
*both* directions — and an out-of-range sensor reads as **black**, not white.
This is the cheapest available improvement and it attacks the dominant noise
source directly. Cheaper than any amount of parameter tuning.

## Risks

| Risk | Why it matters | Mitigation |
|---|---|---|
| `/tmp/carbot-test` wiped by a reboot | Silently reverts to the old mirrored order — a run would look plausible and be wrong | `ls /tmp/carbot-test/src/carbot/ir_geometry.py` before every session |
| Steering still mirrored | Everything downstream is invalid | Gate 2 check 1 is designed to catch exactly this |
| Stale deployment reverts the rate limit | Log balloons again, disk fills mid-run | Summary prints the achieved Hz — anything above ~200 means the old file is running |
| Spacing figures wrong | Blind band offsets (±1.8 cm) and the correction ladder both derive from them | Re-sweep with a strip *narrower* than 6.4 cm at a steady speed |
| Loose VCC/GND | Looks exactly like a dead sensor | Four channels misbehaving together is power/wiring; a real fault sticks on one channel |

## Next Steps, In Order

1. Re-rsync to `/tmp/carbot-test`, verify `ir_geometry.py` is present.
2. Run Gate 2 with the car pushed by hand. **Check the steering direction
   first** — if it is mirrored, stop and re-examine `PHYSICAL_ORDER` before
   anything else.
3. Raise the sensor bar to ~2 cm, re-run Gate 2, compare the noise percentage.
4. Gates 3 and 4 with the operator present.
5. Add oversampling + per-channel debounce + a stuck-channel check to
   `carbot.ir_tracing` — a 90-second static trace recorded 22 sub-0.25 s
   flickers reaching the wheels unfiltered.
