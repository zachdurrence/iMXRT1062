"""
Can curve-shape tuning close the jerk gap?

The distance identity T = 2S/(v0+v1) does NOT depend on the curve being a
quintic. It holds for ANY basis P with P(0)=0, P(1)=1 that is antisymmetric
about tau=0.5, i.e. P(1-tau) = 1 - P(tau) -- because that forces
integral(P) over [0,1] = 1/2 exactly, hence mean velocity = (v0+v1)/2.

So the quintic can be swapped for any symmetric S-curve without touching a
single distance budget. That gives a free knob. The question is whether the
knob has enough range to matter.

Family tested: blend of the cubic smoothstep and the quintic smootherstep,
    P_b(tau) = (1-b)*(3t^2 - 2t^3) + b*(6t^5 - 15t^4 + 10t^3)
both symmetric, so the blend is symmetric for any b.
"""
import numpy as np
import sympy as sp

t, b = sp.symbols('tau beta', nonnegative=True)

smoothstep   = 3*t**2 - 2*t**3            # cubic:   a=0 at ends, jerk STEPS at ends
smootherstep = 6*t**5 - 15*t**4 + 10*t**3 # quintic: a=0 AND j=0 at ends (current)

P = sp.expand((1 - b)*smoothstep + b*smootherstep)

# symmetry check -> guarantees integral = 1/2 -> guarantees T = 2S/(v0+v1)
assert sp.simplify(P.subs(t, 1 - t) - (1 - P)) == 0
assert sp.simplify(sp.integrate(P, (t, 0, 1)) - sp.Rational(1, 2)) == 0
print("family is symmetric for all beta -> integral = 1/2 -> T = 2S/(v0+v1) preserved\n")

dP  = sp.diff(P, t)
ddP = sp.diff(P, t, 2)

print("peak accel factor A(beta) = max P'  (both components peak at tau=0.5):")
A_beta = sp.simplify(dP.subs(t, sp.Rational(1, 2)))
print("   A(beta) =", sp.expand(A_beta), "   ->  1.5 at beta=0,  1.875 at beta=1\n")

f_dP  = sp.lambdify((t, b), dP, 'numpy')
f_ddP = sp.lambdify((t, b), ddP, 'numpy')

taus = np.linspace(0, 1, 200001)
betas = np.linspace(0, 1, 1001)

rows = []
for bb in betas:
    a_pk = np.max(f_dP(taus, bb))
    j_pk = np.max(np.abs(f_ddP(taus, bb)))
    rows.append((bb, a_pk, j_pk))
rows = np.array(rows)

print(f"{'beta':>6} {'peak accel factor':>18} {'peak jerk factor':>17}")
for bb in [0.0, 0.2, 0.4, 0.5, 0.6, 0.8, 1.0]:
    i = int(round(bb * (len(betas) - 1)))
    print(f"{rows[i,0]:6.2f} {rows[i,1]:18.4f} {rows[i,2]:17.4f}")

i_min = int(np.argmin(rows[:, 2]))
print(f"\nMINIMUM peak jerk factor over the family: {rows[i_min,2]:.4f} at beta={rows[i_min,0]:.3f}"
      f"  (peak accel factor there = {rows[i_min,1]:.4f})")
print(f"Current quintic (beta=1):                 {rows[-1,2]:.4f}          "
      f"  (peak accel factor       = {rows[-1,1]:.4f})")
print(f"\nBest achievable jerk reduction by reshaping alone: "
      f"{rows[-1,2] / rows[i_min,2]:.3f}x")
print(f"Cost: peak accel factor rises {rows[i_min,1] / rows[-1,1]:.3f}x "
      f"(i.e. a_eff must drop by that factor to stay inside A_max)")
print(f"Net jerk benefit after re-derating a_eff for the higher accel factor, "
      f"in the accel-limited regime (J ~ a_eff^2): "
      f"{(rows[-1,2] / rows[i_min,2]) * (rows[-1,1] / rows[i_min,1])**2:.3f}x")

print("\n" + "=" * 72)
print("MEASURED OVERSHOOT THAT NEEDS CLOSING (from the A/B harness):")
print("=" * 72)
print("   3 mm triangle at F3000 :  7.8x over the jerk setting")
print("   0.05 mm move           : 16.4x over the jerk setting")
print("\nVERDICT: reshaping buys at most the factor printed above. It cannot")
print("close a 7.8x gap, let alone 16x. The gap is not a curve-shape problem.")
