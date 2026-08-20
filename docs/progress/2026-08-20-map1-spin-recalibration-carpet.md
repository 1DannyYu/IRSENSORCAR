# Map1 Spin Recalibration on Carpet — Work Log (2026-08-20)

Continues
[`2026-08-18-map1-turn-and-travel-calibration.md`](2026-08-18-map1-turn-and-travel-calibration.md).
That session's spin numbers (40.5 deg/s, 0.2s dead time) stopped matching
observed behaviour on a same-day two-lap IR-nav track run: the first T-junction
turn undershot to roughly 45 degrees instead of 90. This session re-ran
[`examples/41_motor_spin_angle_sweep.py`](../../examples/41_motor_spin_angle_sweep.py)
(speed 150, direction right, durations `2,4,6,8,10`) to get a current number,
on the Map1 printed track paper, **with carpet underneath** — the surface
this build has been running on today, not necessarily the same surface as the
2026-08-18 measurement.

All runs were done with an operator standing beside the car, able to cut main
power instantly.

## 1. First sweep — discarded: not a true pivot

| Duration (s) | Observed angle (deg) | Deg/s |
|---|---|---|
| 2 | 45 | 22.5 |
| 4 | 95 | 23.8 |
| 6 | 155 | 25.8 |
| 8 | 220 | 27.5 |
| 10 | 400 | 40.0 |

The deg/s column does not hold together (22.5 up to 40.0, not a consistent
rate), and a linear `angle = rate * (duration - dead_time)` fit over the whole
set has large residuals. Partway through, the operator noticed the car was
not turning in place: during at least one reading the chassis visibly
translated roughly **15cm toward the 2:30 clock direction (north-east)**
while it was supposed to be a zero-radius in-place pivot. `car.move_for`
commands the two sides at equal and opposite magnitude
(`left=150, right=-150`), which is symmetric in software — a real drift means
something asymmetric in the hardware, not the command. The most likely cause:
today's earlier off-map excursions during IR-nav track runs and the abrupt
power cuts that followed may have knocked a wheel or axle slightly out of
alignment.

**This sweep's numbers were not used.** The operator checked the wheels and
axles by hand before continuing.

## 2. Second sweep — clean, all 5 readings confirmed a true pivot

Same script, same settings, after the wheel check. The operator confirmed by
eye for every single reading that the car rotated in place with no visible
chassis translation before recording the angle.

| Duration (s) | Observed angle (deg) | Deg/s |
|---|---|---|
| 2 | 70 | 35.0 |
| 4 | 150 | 37.5 |
| 6 | 225 | 37.5 |
| 8 | 330 | 41.2 |
| 10 | 400 | 40.0 |

Linear least-squares fit of `angle = rate * (duration - dead_time)`:

- **rate ≈ 42.0 deg/s**
- **dead_time ≈ 0.41 s**

Residuals against this fit are all under 11 degrees (2s: 3°, 4s: 1°, 6s: 10°,
8s: 11°, 10s: 3°) — a much tighter fit than the first sweep, consistent with
all 5 points sharing the same physical behaviour.

## 3. Comparison to 2026-08-18

| Constant | 2026-08-18 | 2026-08-20 |
|---|---|---|
| `spin_rate_deg_per_s` | 40.5 | 42.0 |
| `spin_dead_time_s` | 0.2 | 0.41 |

The rate is close to the old value; the dead time roughly doubled. Most
likely explanation is the surface underneath the paper (carpet here,
unconfirmed for the 08-18 session) or residual mechanical change from
whatever knocked the wheel alignment off before the wheel check in §1 — not
treated as a fixed property of the car. Re-run this sweep (and re-check the
wheels first if a "pivot" starts drifting again) before trusting these
numbers on a different surface or after any collision/abrupt stop.

## 4. Applied

Updated in [`src/carbot/ir_line_nav.py`](../../src/carbot/ir_line_nav.py)
(`IRNavPolicy.spin_rate_deg_per_s`, `.spin_dead_time_s`) and
[`tasks/ir-sensor-tracking/design.md`](../../tasks/ir-sensor-tracking/design.md).
