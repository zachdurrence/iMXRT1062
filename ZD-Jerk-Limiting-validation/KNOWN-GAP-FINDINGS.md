# Closing the jerk gap — findings

The gap: peak jerk goes as `a_eff²/Δv`, and the planner sizes `a_eff` from
`programmed_rate`. A block whose real speed change is smaller overshoots `$jerk`
by `programmed_rate/Δv`.

## 1. Curve reshaping cannot fix it

The identity `T = 2S/(v0+v1)` does not depend on the curve being a quintic. It
holds for **any** basis that is antisymmetric about τ=0.5, since that forces
`∫P = 1/2`. So the quintic can be swapped for any symmetric S-curve without
touching a single distance budget — a free knob.

Blending the cubic smoothstep with the quintic smootherstep gives a
one-parameter family with, exactly, `A(β) = 1.5 + 0.375β`:

| β | peak accel factor | peak jerk factor |
|---|---|---|
| 0.00 (cubic) | 1.5000 | 6.0000 |
| 0.333 | 1.6249 | **4.6476** ← min jerk |
| 1.00 (quintic, shipped) | 1.8750 | 5.7735 |

Best jerk reduction available from reshaping: **1.24×**, or **1.65×** after
re-derating `a_eff` for the higher accel factor. Against a 7.8–16× gap, that is
not a fix. *(`shape_family.py`)*

## 2. The naive fix has the physics backwards

The obvious move — slow short moves down — makes it worse. For a ramp at fixed
acceleration, `s = (v1²−v0²)/2a`, so `T = 2s/(v0+v1) = Δv/a` and

```
peak jerk = J_FACTOR · Δv/T² = J_FACTOR · a² / Δv
```

Jerk **falls** as Δv grows. Lowering peak speed shortens the ramp and raises
jerk. The lever is **acceleration**, and it enters squared. Inverting gives
`a ≤ sqrt(J·Δv/J_FACTOR)` — the planner's existing second bound, evaluated with
the block's *actual* Δv instead of `programmed_rate`. The stepper knows the
actual Δv at profile-setup time; the planner cannot, because entry and exit
speeds are only fixed later by the lookahead passes.

Bisecting for the largest jerk-feasible acceleration reproduces the closed form
`(J·√L/J_FACTOR)^(2/3)` already written into the `KNOWN GAP` comment in
`planner.c`, to **0.0000 relative deviation**. Two independent derivations,
same answer.

## 3. Applying it in the planner is as bad as expected

Variant B — a jerk-aware entry-speed bound in the reverse pass — throttles every
short block whether or not it ramps:

| scenario | feed rate vs shipped |
|---|---|
| arc: 200 × 0.1 mm chords | **0.34×** |
| arc: 400 × 0.05 mm chords | **0.27×** |
| contour: 100 × 0.5 mm | 0.56× |

Confirmed. It stays out of the planner.

## 4. Applying it per-block in the stepper leaves arcs alone

This was the whole objection, and it does not apply when the clamp uses each
block's real geometry:

| scenario | blocks clamped | feed vs shipped |
|---|---|---|
| arc: 200 × 0.1 mm chords | **0 / 200** | 1.00× |
| arc: 400 × 0.05 mm chords | **0 / 400** | 1.00× |
| contour: 100 × 0.5 mm | 2 / 100 | 1.00× |

Arc chords cruise; Δv per block is ~0; no ramp, nothing to clamp.

## 5. But compliance is not free on the moves it does affect

Isolated moves, F3000, A=900000, $jerk=3e8:

| move length | jerk as shipped | after clamp | feed rate |
|---|---|---|---|
| 0.05 mm | 21.2× | 1.00× | **36%** |
| 0.19 mm | 10.8× | 1.00× | 45% |
| 1.0 mm | 4.7× | 1.00× | 60% |
| 2.9 mm | 2.8× | 1.00× | 71% |
| 10 mm | 1.5× | 1.00× | 87% |
| 20 mm | 1.05× | 1.00× | 98% |
| >25 mm | 1.00× | 1.00× | 100% |

**Above ~25 mm the shipped code is already compliant.** Below that, the price
rises as the move shortens. That is not an implementation inefficiency — it is
what the jerk limit costs.

## 6. For chained short blocks the metric is quantization, not ramp math

Arc chains report ~180× `$jerk` in *every* variant, including ones that clamp.
That number is DT_SEGMENT quantization: a 0.1 mm chord at F2900 with a
6.8 mm/min speed change has T = 0.6 ms, shorter than one 2.5 ms segment tick.
The generator emits one velocity step. Peak accel stays at 0.82×A and the step
is ~65 mm/min — small. No ramp-level change touches this; the only lever is
`ACCELERATION_TICKS_PER_SECOND`, already at 400.

**Don't chase the arc number.** It is an artefact of applying a continuous-curve
metric below the sampling rate.

## Recommendation

Implement §4 as an opt-in build flag, default off, gated to ramps spanning at
least ~4 segments so quantization noise cannot trigger it. Then measure on the
machine — the 36%-of-feed figure for sub-0.1 mm moves is the kind of thing that
either doesn't matter at all or is unacceptable, depending on the toolpath, and
that is not decidable from simulation.

## Files

`shape_family.py` (§1), `planner_sim.py` (§2–4, §6), `gap_sweep.py` +
`jerk_gap_sweep.png` (§5).

Two modelling errors were found and corrected along the way; both are documented
in the `planner_sim.py` header, since each produced plausible-looking but wrong
numbers before being caught.

---

# Addendum — recomputed against the actual machine settings

Earlier sections used placeholder limits. Real values, with the unit conversions
confirmed from `settings.c` (`$120` is mm/s² stored ×60², `$800` is mm/s³ stored
×60³):

| setting | value | internal |
|---|---|---|
| `$120`/`$121` X/Y accel | 800 mm/s² | 2.88e6 mm/min² |
| `$122` Z accel | 100 mm/s² | 3.6e5 mm/min² |
| `$800`–`$802` jerk | 3000 mm/s³ | 6.48e8 mm/min³ |
| `$110`–`$112` max rate | 20000 mm/min | — |

## `$800` is the binding limit on X/Y at every commandable feed

`a_eff = min(A·8/15, sqrt(J·v/5.7735))`. The accel term is 1,536,000; the jerk
term only reaches that at **F21,000**, above the `$110` max rate of 20,000. So
the jerk bound binds everywhere and **`$120=800` is never actually reached**:

| feed | a_eff | % of `$120` |
|---|---|---|
| F1000 | 93 mm/s² | 12% |
| F3000 | 161 mm/s² | 20% |
| F6000 | 228 mm/s² | 28% |
| F20000 | 416 mm/s² | 52% |

If cornering feels sluggish after the switch, `$800` is the knob, not `$120`.

## The quintic costs a uniform 0.83× versus the old code

Both old and new are jerk-limited at these settings, and the ratio is constant:

```
old (trapezoidal double-jerk ramp) : a_eff = 0.5000·sqrt(J·v)   → factor 4
new (quintic)                      : a_eff = 0.4162·sqrt(J·v)   → factor 10/√3 = 5.7735
ratio = sqrt(4 / 5.7735) = 0.8325
```

That is the price of pinning jerk to zero at both ramp ends instead of letting
it step. Recover it exactly by raising `$800` by 5.7735/4 = 1.443×:
**3000 → 4330 mm/s³**.

## Rebuilt risk table — X/Y at F3000

| move | jerk over | ramp | excitation | if compliant |
|---|---|---|---|---|
| 0.05 mm | 17.6× | 18 ms | **57 Hz** | 13.5 Hz |
| 0.1 mm | 12.5× | 25 ms | **40 Hz** | 11.4 Hz |
| 0.3 mm | 7.2× | 43 ms | 23 Hz | 8.6 Hz |
| 1 mm | 3.9× | 79 ms | 12.7 Hz | 6.4 Hz |
| 3 mm | 2.3× | 136 ms | 7.3 Hz | 4.9 Hz |
| 10 mm | 1.2× | 249 ms | 4.0 Hz | 3.6 Hz |
| ≥30 mm | 1.0× | 310 ms | 3.2 Hz | — |

## Rapids are the more realistic exposure

At F20000 the ramps are longer, so short rapids sit further up the curve:

| rapid | jerk over | ramp | excitation |
|---|---|---|---|
| 1 mm | 16.3× | 49 ms | 20 Hz |
| 10 mm | 5.2× | 155 ms | 6.5 Hz |
| 100 mm | 1.6× | 490 ms | 2.0 Hz |
| ≥500 mm | 1.0× | 801 ms | 1.2 Hz |

Short XY rapids — hop moves, drilling patterns, repositioning — are far more
common than isolated 0.05 mm cutting moves, so this is where the gap is most
likely to actually be met in practice.

## Z is clean

`$122=100` is low enough that Z is accel-limited above ~F328, and its jerk never
approaches `$802`: 0.75× at worst across plunge and rapid cases, usually far
below. No action needed on Z.

---

# Implementation — `JERK_LIMIT_SHORT_MOVES`

Opt-in build flag, **default off**. One file (`stepper.c`), no planner change.
Enable with `-D JERK_LIMIT_SHORT_MOVES=1`.

## How it works

At velocity-profile setup the stepper knows the block's real entry, peak and
exit speeds, so it can evaluate each ramp's actual jerk and solve

```
a <= sqrt(J * dv / JERK_QUINTIC_FACTOR)
```

for the largest compliant acceleration. On a triangle the peak speed depends on
the acceleration being solved for, so the profile is recomputed as a fixed point
(`JERK_LIMIT_PASSES`, default 5).

Three guards keep it from firing where it would only cost feed rate:

- **`JERK_LIMIT_MIN_TICKS` (default 4)** — a ramp shorter than 4 `DT_SEGMENT`
  ticks is delivered as one or two velocity steps. Its apparent jerk is
  quantization, not curve shape, and no acceleration change can affect it.
- **Shape check before commit** — lowering acceleration moves the accel/decel
  intersection. If the candidate would flip the profile to acceleration-only or
  deceleration-only, it is rejected and the current profile is left untouched;
  those shapes' speeds are a contract with the neighbouring blocks.
- **Feed holds and override reductions excluded** — a feed hold keeps the
  planner's acceleration so stopping distance is not extended.

## Measured, at the machine's actual settings

`$120`=800 mm/s², `$800`=3000 mm/s³, verbatim `stepper.c` in the C harness:

| case | jerk off | jerk on | peak speed | time | verdict |
|---|---|---|---|---|---|
| XY rapid F20000, 1 mm hop | 16.34× | **1.00×** | 1224 → 483 | 1.85× | clamped |
| XY F3000, corner to F2000 | 3.00× | **1.00×** | 3000 → 3000 | **1.03×** | clamped, nearly free |
| XY F3000, 3 mm triangle | 2.27× | **1.00×** | 1319 → 1004 | 1.25× | clamped |
| XY rapid F20000, 100 mm | 1.63× | **1.00×** | 12240 → 10394 | 1.16× | clamped |
| XY F3000, 100 mm | 1.00× | 1.00× | 3000 | 1.00× | untouched |
| Z plunge F500, 10 mm | 1.00× | 1.00× | 500 | 1.00× | untouched |
| arc chord 0.1 mm cruise | — | — | 2910 | 1.00× | **untouched** |
| arc chord 0.05 mm cruise | — | — | 2905 | 1.00× | **untouched** |
| contour 0.5 mm cruise | — | — | 2557 | 1.00× | **untouched** |
| pure cruise 50 mm | — | — | 3000 | 1.00× | untouched |

Every violating case lands on 1.00×. Chained short blocks are untouched — they
cruise, so dv per block is ~0 and there is no ramp to clamp. That was the whole
objection to doing this in the planner, and it does not apply here.

Note the corner case: 3.00× → 1.00× at **1.03× the time and no loss of peak
speed**. That is the trapezoid regime, where lowering acceleration converts
cruise distance into ramp distance. Corners are the common case and they are
nearly free; only triangles and short rapids pay real time.

## Verified

- **Flag off is a bit-exact no-op** — harness output with
  `JERK_LIMIT_SHORT_MOVES=0` is identical to the branch as pushed.
- Both flag states compile clean at `-O2 -Wall`.
- The clamp's numerical answer matches the closed form `(J·√L/JF)^(2/3)`.

## Still not verified

Not compiled for the Teensy target, and not run on hardware. Untested paths are
unchanged from the base branch: feed hold interrupting a ramp, override changes
mid-block, parking, tool-change probing.
