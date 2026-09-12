import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

CASES = [
    ("full_trapezoid_F3000_100mm", "Full trapezoid, F3000, 100 mm"),
    ("triangle_3mm_F3000",         "Triangle, 3 mm move, F3000"),
    ("decel_to_F800",              "Decel to F800 (a planned corner)"),
    ("low_jerk_setting",           "Low jerk setting ($jerk = 1e7)"),
]

def load(arm, key):
    d = np.genfromtxt(f"traces/trace_{arm}_{key}_csv.csv", delimiter=",", names=True)
    t = d["t_min"] * 60.0            # seconds
    v = d["v"]
    dt = d["dt_min"]
    a = np.zeros_like(v)
    a[1:] = np.where(dt[1:] > 0, (v[1:] - v[:-1]) / np.maximum(dt[1:], 1e-12), 0.0)
    dv = np.zeros_like(v)
    dv[1:] = v[1:] - v[:-1]
    return t, v, a, dv, dt

fig, axs = plt.subplots(3, len(CASES), figsize=(4.6 * len(CASES), 10.5))
OLD, NEW = "#2a78d6", "#eb6834"

for c, (key, title) in enumerate(CASES):
    to, vo, ao, dvo, dto = load("old", key)
    tn, vn, an, dvn, dtn = load("new", key)

    ax = axs[0][c]
    ax.step(to, vo, where="post", color=OLD, lw=1.2, label="quantized (current)")
    ax.step(tn, vn, where="post", color=NEW, lw=1.2, label="quintic (new)")
    ax.set_title(title, fontsize=10)
    if c == 0:
        ax.set_ylabel("commanded speed\n(mm/min)")
        ax.legend(fontsize=8)

    ax = axs[1][c]
    ax.step(to, ao, where="post", color=OLD, lw=1.0)
    ax.step(tn, an, where="post", color=NEW, lw=1.0)
    ax.axhline(900000, color="k", ls="--", lw=0.8)
    ax.axhline(-900000, color="k", ls="--", lw=0.8)
    ax.set_yscale("symlog", linthresh=1e5)
    if c == 0:
        ax.set_ylabel("segment accel (mm/min$^2$)\ndashed = $A_{max}$, symlog")

    ax = axs[2][c]
    ax.step(to, np.abs(dvo), where="post", color=OLD, lw=1.0)
    ax.step(tn, np.abs(dvn), where="post", color=NEW, lw=1.0)
    ax.set_yscale("log")
    ax.set_xlabel("time (s)")
    if c == 0:
        ax.set_ylabel("|step-rate impulse| per\nsegment (mm/min), log")

    print(f"{title:38s} old max|dv| {np.abs(dvo).max():9.2f}   new max|dv| {np.abs(dvn).max():8.2f}"
          f"   old max|a| {np.abs(ao).max():11.0f}  new max|a| {np.abs(an).max():9.0f}")

fig.suptitle("grblHAL jerk-limited ramp: quantized-jerk integration vs closed-form quintic Bezier\n"
             "(code extracted verbatim from stepper.c, run in float, ACCELERATION_TICKS_PER_SECOND=400)",
             fontsize=11)
plt.tight_layout(rect=[0, 0, 1, 0.955])
plt.savefig("quintic_ab_validation.png", dpi=140)
print("\nsaved quintic_ab_validation.png")
