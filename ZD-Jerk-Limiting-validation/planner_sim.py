"""
Multi-block planner simulator (v3) -- corrected model.

TWO MODELLING ERRORS FOUND AND FIXED ALONG THE WAY, both worth recording:

(1) v1 reported 1e25 jerk on arc chains. Not real. Peak jerk goes as dv/T^2, so a
    ramp with tiny dv over tiny T reports a huge number while delivering almost
    nothing. A 0.1 mm chord cruising at F2900 with a 6.8 mm/min speed change has
    T = 0.6 ms -- shorter than one DT_SEGMENT tick (2.5 ms). The segment
    generator emits ONE velocity step and moves on. Continuous-curve jerk is only
    physical for ramps spanning several segments; below that the quantity that
    matters is the step-rate impulse. Fixed by saturating T at DT_SEGMENT.

(2) v2's "clamp maximum_speed" fix had the sign of the physics backwards. For a
    ramp at fixed acceleration, s = (v1^2-v0^2)/2a and T = 2s/(v0+v1) = dv/a, so

        peak jerk = J_FACTOR * dv / T^2 = J_FACTOR * a^2 / dv

    Jerk FALLS as dv grows. Lowering peak speed shortens the ramp and makes jerk
    WORSE. The lever is acceleration, not speed -- and it enters squared.

So the correct per-block fix is to REDUCE the block's effective acceleration,
stretching its ramps over more of the available distance. Inverting the relation
above gives a <= sqrt(J*dv/J_FACTOR): the planner's existing second bound, but
evaluated with the block's ACTUAL dv instead of programmed_rate. The stepper
knows the actual dv at profile-setup time; the planner cannot, because entry and
exit speeds are only fixed later by the lookahead passes.

Two regimes, with very different costs:
  * trapezoid block (a cruise segment exists): lowering a converts cruise
    distance into ramp distance. Peak speed is untouched; only a little time is
    lost. Nearly free.
  * triangle block (no cruise): ramps already span the block, so lowering a also
    lowers peak speed. Real time cost. This is where the 7.8x/16x cases live.

Variants:
  base : shipped ZD-Jerk-Limiting behaviour
  A    : reduce per-block acceleration, but never past the point where the cruise
         segment is exhausted -- peak speed and profile type preserved
  A+   : as A, but also allowed to reduce peak speed on triangle blocks
  B    : jerk-aware entry-speed bound in the planner lookahead (for comparison)
"""
import numpy as np
from dataclasses import dataclass

SQRT3 = np.sqrt(3.0)
A_FACTOR = 15.0 / 8.0
J_FACTOR = 10.0 / SQRT3

ACCELERATION_TICKS_PER_SECOND = 400
DT_SEGMENT = 1.0 / (60.0 * ACCELERATION_TICKS_PER_SECOND)


def a_eff_quintic(A, J, prog_rate):
    return min(A * 8.0 / 15.0, np.sqrt(J * prog_rate / J_FACTOR))


RESOLVED_TICKS = 4      # a ramp shorter than this is a step, not a curve


def ramp_metrics(v0, v1, S):
    """(T, peak accel, commanded jerk). T saturates at DT_SEGMENT: a ramp
    shorter than one segment is delivered as a single velocity step, so its
    commanded jerk cannot exceed dv/DT_SEGMENT^2."""
    if S <= 0.0 or (v0 + v1) <= 0.0:
        return 0.0, 0.0, 0.0
    T = 2.0 * S / (v0 + v1)
    dv = abs(v1 - v0)
    Te = max(T, DT_SEGMENT)
    return T, A_FACTOR * dv / Te, J_FACTOR * dv / (Te * Te)


def max_entry_jerk_limited(v_exit, L, J):
    K = 4.0 * J * L * L / J_FACTOR
    lo, hi = v_exit, v_exit + K ** (1.0 / 3.0) + 1.0
    while (hi - v_exit) * (hi + v_exit) ** 2 < K and hi < 1e9:
        hi *= 2.0
    for _ in range(90):
        mid = 0.5 * (lo + hi)
        if (mid - v_exit) * (mid + v_exit) ** 2 <= K:
            lo = mid
        else:
            hi = mid
    return lo


@dataclass
class Block:
    millimeters: float
    programmed_rate: float
    max_junction_speed_sqr: float
    A: float
    J: float
    acceleration: float = 0.0
    a_profile: float = 0.0          # acceleration actually used for the profile
    max_entry_speed_sqr: float = 0.0
    entry_speed_sqr: float = 0.0
    maximum_speed: float = 0.0
    exit_speed: float = 0.0
    accelerate_until: float = 0.0
    decelerate_after: float = 0.0


def build_chain(lengths, feed, A, J, junction_speed):
    blocks = []
    for L in lengths:
        b = Block(L, feed, junction_speed ** 2, A, J)
        b.acceleration = a_eff_quintic(A, J, feed)
        b.max_entry_speed_sqr = min(b.max_junction_speed_sqr, feed ** 2)
        blocks.append(b)
    blocks[0].max_entry_speed_sqr = 0.0
    return blocks


def recalculate(blocks, variant):
    n = len(blocks)
    last = blocks[-1]
    cap = 2.0 * last.acceleration * last.millimeters
    if variant == "B":
        cap = min(cap, max_entry_jerk_limited(0.0, last.millimeters, last.J) ** 2)
    last.entry_speed_sqr = min(last.max_entry_speed_sqr, cap)
    for i in range(n - 2, -1, -1):
        cur, nxt = blocks[i], blocks[i + 1]
        if cur.entry_speed_sqr != cur.max_entry_speed_sqr:
            e = nxt.entry_speed_sqr + 2.0 * cur.acceleration * cur.millimeters
            if variant == "B":
                e = min(e, max_entry_jerk_limited(np.sqrt(nxt.entry_speed_sqr),
                                                  cur.millimeters, cur.J) ** 2)
            cur.entry_speed_sqr = min(e, cur.max_entry_speed_sqr)
    for i in range(n - 1):
        cur, nxt = blocks[i], blocks[i + 1]
        if cur.entry_speed_sqr < nxt.entry_speed_sqr:
            e = cur.entry_speed_sqr + 2.0 * cur.acceleration * cur.millimeters
            if e < nxt.entry_speed_sqr:
                nxt.entry_speed_sqr = e


def profile_for(b, a, exit_speed):
    """st_prep_buffer velocity profile setup with a given acceleration."""
    inv_2a = 0.5 / a
    entry_sqr, exit_sqr = b.entry_speed_sqr, exit_speed ** 2
    nominal_sqr = b.programmed_rate ** 2
    accelerate_until, decelerate_after = b.millimeters, 0.0
    intersect = 0.5 * (b.millimeters + inv_2a * (entry_sqr - exit_sqr))
    if 0.0 < intersect < b.millimeters:
        decelerate_after = inv_2a * (nominal_sqr - exit_sqr)
        if decelerate_after < intersect:
            maximum_speed = b.programmed_rate
            accelerate_until -= inv_2a * (nominal_sqr - entry_sqr)
            triangle = False
        else:
            accelerate_until = decelerate_after = intersect
            maximum_speed = np.sqrt(2.0 * a * intersect + exit_sqr)
            triangle = True
    elif intersect <= 0.0:
        maximum_speed = np.sqrt(entry_sqr)
        accelerate_until = decelerate_after = b.millimeters
        triangle = False
    else:
        accelerate_until, maximum_speed, triangle = 0.0, exit_speed, False
    return maximum_speed, accelerate_until, decelerate_after, triangle


def block_worst_jerk(b, a, exit_speed, resolved_only=False):
    """Worst commanded jerk over the block's ramps.

    resolved_only: ignore ramps shorter than RESOLVED_TICKS segments. Those are
    delivered as one or two velocity steps, so their apparent "jerk" is pure
    DT_SEGMENT quantization -- no ramp reshaping can change it, and clamping the
    block's acceleration on account of it spends feed rate for nothing. Used
    when DECIDING whether to clamp; the reported metric stays ungated."""
    v_max, acc_until, dec_after, tri = profile_for(b, a, exit_speed)
    v_entry = np.sqrt(b.entry_speed_sqr)
    worst = 0.0
    for (v0, v1, S) in ((v_entry, v_max, b.millimeters - acc_until),
                        (v_max, exit_speed, dec_after)):
        T, _, j = ramp_metrics(v0, v1, S)
        if T <= 0.0:
            continue
        if resolved_only and T < RESOLVED_TICKS * DT_SEGMENT:
            continue
        worst = max(worst, j)
    return worst, tri


def choose_acceleration(b, exit_speed, allow_triangle):
    """
    Walk the block's acceleration down until its ramps satisfy J.
    Floor: the acceleration below which the profile turns triangular (peak speed
    would drop). With allow_triangle, go below that floor too.
    """
    a_hi = b.acceleration
    j, tri = block_worst_jerk(b, a_hi, exit_speed, resolved_only=True)
    if j <= b.J:
        return a_hi
    # hard floor -- still has to get from entry to exit within the block
    a_min = abs(exit_speed ** 2 - b.entry_speed_sqr) / (2.0 * b.millimeters)
    a_min = max(a_min, 1e-6)
    if not allow_triangle and tri:
        return a_hi                     # already triangular: no free move
    # largest acceleration that is still jerk-feasible (bisection; feasibility is
    # monotone in a because jerk ~ a^2/dv and lowering a only lengthens ramps)
    lo, hi, best = a_min, a_hi, a_min
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        jm, trim = block_worst_jerk(b, mid, exit_speed, resolved_only=True)
        if jm <= b.J and (allow_triangle or not trim):
            best, lo = mid, mid
        else:
            hi = mid
    return best


def run(lengths, feed, A, J, junction_speed, variant):
    blocks = build_chain(lengths, feed, A, J, junction_speed)
    recalculate(blocks, "B" if variant == "B" else "base")

    worst_j = worst_a = worst_step = 0.0
    total_time = 0.0
    n_clamped = 0

    for i, b in enumerate(blocks):
        exit_speed = np.sqrt(blocks[i + 1].entry_speed_sqr) if i + 1 < len(blocks) else 0.0
        a = b.acceleration
        if variant in ("A", "A+"):
            a = choose_acceleration(b, exit_speed, allow_triangle=(variant == "A+"))
            if a < b.acceleration * 0.999:
                n_clamped += 1
        b.a_profile = a
        v_max, acc_until, dec_after, _ = profile_for(b, a, exit_speed)
        b.maximum_speed, b.accelerate_until, b.decelerate_after = v_max, acc_until, dec_after
        b.exit_speed = exit_speed

        v_entry = np.sqrt(b.entry_speed_sqr)
        s_cru = max(0.0, acc_until - dec_after)
        for (v0, v1, S) in ((v_entry, v_max, b.millimeters - acc_until),
                            (v_max, exit_speed, dec_after)):
            T, aa, jj = ramp_metrics(v0, v1, S)
            if T <= 0.0:
                continue
            total_time += T
            worst_j = max(worst_j, jj)
            worst_a = max(worst_a, aa)
            worst_step = max(worst_step, aa * DT_SEGMENT)
        if s_cru > 0 and v_max > 0:
            total_time += s_cru / v_max

    return dict(worst_jerk=worst_j, worst_accel=worst_a, worst_step=worst_step,
                clamped=n_clamped,
                avg_feed=sum(lengths) / total_time if total_time > 0 else 0.0)


A_MAX, J_MAX, FEED = 900000.0, 3.0e8, 3000.0

SCENARIOS = [
    ("isolated 0.05 mm move",           [0.05],                 0.0),
    ("isolated 3 mm move",              [3.0],                  0.0),
    ("isolated 20 mm move",             [20.0],                 0.0),
    ("isolated 100 mm move",            [100.0],                0.0),
    ("arc: 200 x 0.1 mm chords",        [0.1] * 200,         2900.0),
    ("arc: 400 x 0.05 mm chords",       [0.05] * 400,        2900.0),
    ("contour: 100 x 0.5 mm",           [0.5] * 100,         2500.0),
    ("mixed: 20mm then 50 x 0.2mm",     [20.0] + [0.2] * 50, 2000.0),
    ("zigzag: 60 x 1 mm, hard corners", [1.0] * 60,           400.0),
]

print(f"A_max={A_MAX:.0f}  J_max={J_MAX:.3g}  F{FEED:.0f}  DT_SEGMENT={DT_SEGMENT*60000:.2f} ms\n")
hdr = (f"{'scenario':<34}{'var':<6}{'peak jerk':>11}{'xJ':>7}{'peak a':>9}{'xA':>6}"
       f"{'blocks clamped':>16}{'avg feed':>10}{'vs base':>9}")
print(hdr); print("-" * len(hdr))

for name, lengths, junc in SCENARIOS:
    base = None
    for variant in ("base", "A", "A+", "B"):
        r = run(lengths, FEED, A_MAX, J_MAX, junc, variant)
        if variant == "base":
            base = r
        ratio = r["avg_feed"] / base["avg_feed"] if base["avg_feed"] > 0 else float("nan")
        print(f"{name if variant=='base' else '':<34}{variant:<6}"
              f"{r['worst_jerk']:11.3g}{r['worst_jerk']/J_MAX:7.2f}"
              f"{r['worst_accel']:9.0f}{r['worst_accel']/A_MAX:6.2f}"
              f"{r['clamped']:>10d}/{len(lengths):<5d}"
              f"{r['avg_feed']:10.1f}{ratio:8.2f}x")
    print()
