"""
Where does the jerk gap actually bite, and what does closing it cost?

Sweeps isolated move length (the case where the gap is physically real -- ramps
span many segments) and plots the jerk overshoot against the feed-rate price of
the per-block acceleration clamp.
"""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from planner_sim import (run, A_FACTOR, J_FACTOR, a_eff_quintic, DT_SEGMENT)

A_MAX, J_MAX, FEED = 900000.0, 3.0e8, 3000.0
lengths = np.logspace(np.log10(0.02), np.log10(200.0), 90)

base_j, fix_j, feed_ratio, a_used = [], [], [], []
for L in lengths:
    b = run([L], FEED, A_MAX, J_MAX, 0.0, "base")
    f = run([L], FEED, A_MAX, J_MAX, 0.0, "A+")
    base_j.append(b["worst_jerk"] / J_MAX)
    fix_j.append(f["worst_jerk"] / J_MAX)
    feed_ratio.append(f["avg_feed"] / b["avg_feed"])
    a_used.append(f["worst_accel"] / A_FACTOR)

base_j, fix_j, feed_ratio = map(np.array, (base_j, fix_j, feed_ratio))
a_nom = a_eff_quintic(A_MAX, J_MAX, FEED)

# closed-form triangle bound, for comparison: a = (J*sqrt(L)/J_FACTOR)^(2/3)
a_closed = np.minimum(a_nom, (J_MAX * np.sqrt(lengths) / J_FACTOR) ** (2.0 / 3.0))

fig, axs = plt.subplots(1, 3, figsize=(15, 4.4))
OLD, NEW, REF = "#2a78d6", "#eb6834", "#666666"

ax = axs[0]
ax.loglog(lengths, base_j, color=OLD, lw=1.8, label="as shipped")
ax.loglog(lengths, fix_j, color=NEW, lw=1.8, label="with per-block clamp")
ax.axhline(1.0, color="k", ls="--", lw=0.9)
ax.set_xlabel("isolated move length (mm)")
ax.set_ylabel("peak jerk  /  \\$jerk setting")
ax.set_title("Jerk overshoot")
ax.legend(fontsize=9)
ax.grid(alpha=0.25, which="both")

ax = axs[1]
ax.semilogx(lengths, feed_ratio * 100.0, color=NEW, lw=1.8)
ax.axhline(100.0, color="k", ls="--", lw=0.9)
ax.set_xlabel("isolated move length (mm)")
ax.set_ylabel("average feed rate, % of as-shipped")
ax.set_title("Price of compliance")
ax.set_ylim(0, 110)
ax.grid(alpha=0.25, which="both")

ax = axs[2]
ax.loglog(lengths, a_used, color=NEW, lw=1.8, label="clamp (numerical)")
ax.loglog(lengths, a_closed, color=REF, lw=1.2, ls="--",
          label=r"closed form $(J\sqrt{L}/J_{factor})^{2/3}$")
ax.axhline(a_nom, color="k", ls=":", lw=0.9, label="planner $a_{eff}$")
ax.set_xlabel("isolated move length (mm)")
ax.set_ylabel("effective acceleration (mm/min$^2$)")
ax.set_title("Acceleration actually used")
ax.legend(fontsize=8)
ax.grid(alpha=0.25, which="both")

fig.suptitle("Closing the jerk gap: only isolated short moves are affected, and compliance is not free\n"
             f"A_max={A_MAX:.0f} mm/min², \\$jerk={J_MAX:.2g} mm/min³, F{FEED:.0f}", fontsize=11)
plt.tight_layout(rect=[0, 0, 1, 0.88])
plt.savefig("jerk_gap_sweep.png", dpi=140)

cross = lengths[np.argmax(base_j <= 1.0)] if np.any(base_j <= 1.0) else float("nan")
print(f"compliant without any fix above ~{cross:.1f} mm")
for L in (0.05, 0.2, 1.0, 3.0, 10.0, 20.0, 50.0):
    i = int(np.argmin(np.abs(lengths - L)))
    print(f"  L={lengths[i]:7.2f} mm   base {base_j[i]:7.2f}xJ -> fixed {fix_j[i]:5.2f}xJ   "
          f"feed {feed_ratio[i]*100:5.1f}% of as-shipped")
print("\nnumerical clamp vs closed form: max relative deviation "
      f"{np.max(np.abs(np.array(a_used) - a_closed) / a_closed):.4f}")
print("saved jerk_gap_sweep.png")
