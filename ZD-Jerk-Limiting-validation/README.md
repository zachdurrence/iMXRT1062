# ZD-Jerk-Limiting — quintic Bézier ramp validation bundle

Replaces grblHAL's per-segment quantized jerk integration in `stepper.c` with a
closed-form quintic ("smootherstep") ramp, and replaces the trapezoidal-jerk
effective-acceleration derivation in `planner.c` with the quintic's own
accel/jerk ceilings.

## Files

| file | what it is |
|---|---|
| `quintic_derivation.py` | sympy derivation of every constant used. Exact rationals/radicals, no fitted numbers. |
| `harness2.c` | A/B test harness. Compiles code **extracted verbatim** from both versions of `stepper.c` and runs it in real C float arithmetic. |
| `harness_*.inc` | the new `jerk_ramp_advance()`, profile setup, and segment loop, cut from the modified `stepper.c`. |
| `orig_*.inc` | the same two sections cut from the original `stepper.c`. |
| `results_quantized_original.txt` / `results_quintic_new.txt` | harness output for 12 profile cases. |
| `plot_ab.py`, `quintic_ab_validation.png` | velocity / segment-accel / step-rate-impulse comparison. |
| `stepper_quintic.diff`, `planner_quintic.diff` | unified diffs against the ZD branch as checked out. |

## Reproducing

```
python3 quintic_derivation.py
gcc -O2 -DARM_NEW=1 -o harness_new harness2.c -lm && ./harness_new
gcc -O2 -DARM_NEW=0 -o harness_old harness2.c -lm && ./harness_old
python3 plot_ab.py
```

## The math

Quintic Bézier, control points `P0=P1=P2=v0`, `P3=P4=P5=v1` — acceleration and
jerk both pinned to zero at each end of the ramp. That collapses exactly to
smootherstep:

```
v(τ) = v0 + Δv·P(τ)          P(τ) = 6τ⁵ − 15τ⁴ + 10τ³
s(τ) = S·Snorm(τ)            Snorm(τ) = (2·v0·τ + Δv·τ⁴(2τ² − 6τ + 5)) / (v0 + v1)
```

`∫₀¹ P dτ = 1/2` exactly, so the ramp's **mean velocity is exactly (v0+v1)/2** —
the same as a straight-line ramp. Therefore:

```
T = 2S / (v0 + v1)
```

which is the identical expression grblHAL's linear path already uses at ramp
junctions. This is the whole reason the substitution is safe: the planner's
distance budgets (`accelerate_until`, `decelerate_after`, `mm_complete`) need no
geometric correction. Only the derivatives change:

```
peak accel = 15/8       · Δv/T   = 1.875   · a_eff      (P′ = 30τ²(1−τ)², max at τ = 0.5)
peak jerk  = 10/√3      · Δv/T²  = 5.77350 · a_eff²/Δv  (P″ max at τ = (3∓√3)/6)
```

Inverting both against the axis limits gives the planner's `a_eff`:

```
a_eff = min( A·8/15 ,  sqrt(J·v_prog / 5.7735) )
```

The first term binds at high feed rate, the second at low feed rate. For
reference, the trapezoidal-jerk formula this replaces derated by a factor
running from 2.0 (pure double-jerk ramp) to 1.0 (long constant-accel plateau);
the quintic's 1.875 sits inside that range, so effective acceleration on an
already-tuned machine changes only modestly.

## What the A/B shows

Max step-rate impulse per segment on uniform 2.5 ms segments — i.e. the velocity
discontinuity the drives are actually commanded to absorb:

| case | quantized (current) | quintic (new) |
|---|---|---|
| full trapezoid, F3000, 100 mm | 76.6 mm/min | **30.8** |
| triangle, 3 mm, F3000 | **910.6 mm/min** | **30.8** |
| decel to F800 (a planned corner) | **336.9 mm/min** | **30.8** |
| decel to zero | 37.5 mm/min | 30.8 |
| rapid F10000, 200 mm | 37.5 mm/min | 37.5 |
| low jerk setting (1e7) | 200.2 mm/min | **5.6** |
| jog (linear path, jerk off) | 37.5 mm/min | 37.5 — **unchanged, as it must be** |

The quintic's impulse is `1.875·a_eff·DT_SEGMENT` — constant by construction,
independent of profile shape. The quantized version's varies by 30× depending on
the shape, which is the finding below.

### The real defect in the old implementation

The quantized ramp integrates jerk forward in *time* without reference to the
planner's *distance* budget, so the two disagree, and the disagreement is
absorbed as a commanded velocity step at the ramp junction:

- **Triangle, 3 mm**: the ramp reaches `accelerate_until` at ~860 mm/min while
  the planner expected 1192, and `prep.current_speed` is snapped to
  `maximum_speed` — a 330 mm/min instantaneous jump mid-move, then a second
  ~800 mm/min drop at the end of the decel.
- **Decel to F800**: arrives at the block end at ~1100 mm/min and is snapped
  down to the 800 mm/min exit speed.

Both are visible in `quintic_ab_validation.png` (blue traces, columns 2 and 3).
Over a 2.5 ms segment a 910 mm/min step is a commanded acceleration of
21.9 × 10⁶ mm/min² — 24× the axis limit. Those are exactly the conditions
(short moves, planned corners) where corner overshoot and servo ringing show up.

With the quintic, `T = 2S/(v0+v1)` forces the ramp and the budget to agree, so
the target speed and the distance boundary are reached at the same instant and
both are snapped to exact values with nothing to absorb.

### Terminal-segment stability

The `last_time_var` workaround is gone, cause removed rather than symptom
patched. Segment time at ramp end is now `(1 − τ)·T`, drawn from the ramp's own
duration fixed at ramp start from the cruise speed. It never involves
`2·d/(v0 + v1)` with both speeds approaching zero.

Distance is evaluated **absolutely** (`mm_remaining = ramp_end + S·(1 − Snorm)`)
rather than by accumulating per-segment increments, so rounding cannot walk the
endpoint. Measured terminal position error across all 12 cases: ≤ 1.3e−9 mm.

## Known gap

The jerk ceiling assumes the ramp spans `Δv ≈ programmed_rate`. Peak jerk goes
as `a_eff²/Δv`, so a block whose actual speed change is smaller overshoots the
jerk setting by `programmed_rate/Δv` — measured at 7.8× for a 3 mm triangle at
F3000 and 16× for a 0.05 mm move. The old implementation had the same exposure
and was worse in absolute terms, so this is not a regression, but it is not
closed either. The exact fix, and why it is **not** enabled by default (it would
collapse arc feed rates), is documented at the `KNOWN GAP` comment in
`planner.c`.

## Not touched

Per-axis step distribution comes from `st_prep_block->steps.value[idx]` and
`step_event_count`, fixed once per block at load time and consumed by the
Bresenham DDA in the ISR. Multi-axis lockstep is independent of the velocity
profile. The linear (non-jerk) path used for jog, probe and spindle-sync moves
is byte-identical — confirmed by case 12 in the A/B.
