"""
Symbolic derivation of the constants used by the quintic Bezier jerk-limited
ramp in stepper.c / planner.c.

Everything below is exact rational/radical arithmetic via sympy -- these are
not fitted or empirical constants.
"""
import sympy as sp

tau, T, v0, v1, S = sp.symbols('tau T v0 v1 S', positive=True)
dv = v1 - v0

# Quintic Bezier with control points P0=P1=P2=v0, P3=P4=P5=v1.
# Acceleration AND jerk are pinned to zero at both ramp endpoints, which
# collapses the general quintic to the "smootherstep" basis polynomial.
B = sp.Rational(0)
P = [v0, v0, v0, v1, v1, v1]
for i, Pi in enumerate(P):
    B += sp.binomial(5, i) * (1 - tau)**(5 - i) * tau**i * Pi
B = sp.expand(sp.simplify(B))

print("v(tau) =", sp.factor(sp.simplify(B - v0)) , "+ v0")
Pbasis = sp.simplify((B - v0) / dv)
print("basis P(tau) =", sp.expand(Pbasis))
assert sp.simplify(sp.expand(Pbasis) - (6*tau**5 - 15*tau**4 + 10*tau**3)) == 0
print("  -> confirmed identical to smootherstep 6t^5 - 15t^4 + 10t^3")

# --- distance integral -----------------------------------------------------
Q = sp.integrate(Pbasis, (tau, 0, tau))
print("\nQ(tau) = integral of P =", sp.expand(Q))
print("Q(1)   =", sp.simplify(Q.subs(tau, 1)), " (so mean velocity = (v0+v1)/2 exactly)")
assert sp.simplify(Q.subs(tau, 1) - sp.Rational(1, 2)) == 0

# s(tau) = T * (v0*tau + dv*Q(tau)); s(1) = S  =>  T = 2S/(v0+v1)
s_of_tau = T * (v0*tau + dv*Q)
T_solved = sp.solve(sp.Eq(s_of_tau.subs(tau, 1), S), T)[0]
print("\nT from distance budget S:", sp.simplify(T_solved))
assert sp.simplify(T_solved - 2*S/(v0 + v1)) == 0
print("  -> T = 2S/(v0+v1): IDENTICAL to the trapezoidal junction formula.")
print("     A quintic ramp and a LINEAR ramp over the same duration cover the")
print("     same distance, so the planner's existing distance budgets need no")
print("     geometric correction -- only its acceleration scaling changes.")

# normalized distance fraction, used directly in stepper.c
s_norm = sp.simplify(s_of_tau.subs(T, T_solved) / S)
print("\ns_norm(tau) = s/S =", sp.simplify(sp.expand(s_norm)))

# --- peak acceleration -----------------------------------------------------
dP = sp.diff(Pbasis, tau)
crit = sp.solve(sp.diff(dP, tau), tau)
peak_a_factor = sp.Max(*[dP.subs(tau, c) for c in crit if c.is_real and 0 <= c <= 1])
peak_a_factor = sp.nsimplify(sp.simplify(peak_a_factor))
print("\nP'(tau) =", sp.expand(dP), " -> factored:", sp.factor(dP))
print("peak at tau =", [c for c in crit if c.is_real and 0 < c < 1])
print("PEAK ACCEL FACTOR = max P' =", peak_a_factor, "=", float(peak_a_factor))
assert sp.simplify(peak_a_factor - sp.Rational(15, 8)) == 0

# --- peak jerk -------------------------------------------------------------
ddP = sp.diff(Pbasis, tau, 2)
crit2 = sp.solve(sp.diff(ddP, tau), tau)
vals = [sp.simplify(ddP.subs(tau, c)) for c in crit2 if c.is_real]
peak_j_factor = sp.simplify(sp.Max(*[sp.Abs(v) for v in vals]))
print("\nP''(tau) =", sp.expand(ddP), " -> factored:", sp.factor(ddP))
print("extrema at tau =", [sp.nsimplify(c) for c in crit2 if c.is_real])
print("PEAK JERK FACTOR = max|P''| =", sp.radsimp(peak_j_factor),
      "=", float(peak_j_factor))
assert sp.simplify(peak_j_factor - 10/sp.sqrt(3)) == 0

# --- resulting planner limits ---------------------------------------------
print("\n" + "=" * 70)
print("CONSEQUENCES FOR planner.c block->acceleration (a_eff = dv/T):")
print("=" * 70)
A, J, dvs, aeff = sp.symbols('A J dv a_eff', positive=True)
print("  peak accel = 15/8 * a_eff              <= A  =>  a_eff <= A/1.875")
a_from_A = A / sp.Rational(15, 8)
print("    a_eff <=", a_from_A, "=", sp.nsimplify(a_from_A))
# peak jerk = (10/sqrt3)*dv/T^2, T = dv/a_eff  =>  = (10/sqrt3)*a_eff^2/dv
j_expr = (10/sp.sqrt(3)) * aeff**2 / dvs
a_from_J = sp.solve(sp.Eq(j_expr, J), aeff)[0]
print("  peak jerk  = (10/sqrt3) * a_eff^2 / dv <= J  =>  a_eff <=", sp.simplify(a_from_J))
print("    i.e. a_eff <= sqrt(J*dv/5.7735) = sqrt(J*dv*sqrt(3)/10)")
print("\n  a_eff = min(A*8/15, sqrt(J*dv*sqrt(3)/10))")

print("\n" + "=" * 70)
print("COMPARISON WITH grblHAL's EXISTING trapezoidal-jerk a_eff (planner.c 577-585)")
print("=" * 70)
# existing: high-speed regime a_eff -> A ; low-speed regime a_eff = 0.5*sqrt(J*v)
print("  existing high-speed limit : a_eff -> A          (derate factor -> 1.000)")
print("  existing low-speed  limit : a_eff = 0.5*sqrt(Jv) (derate factor  = 2.000)")
print("  quintic accel limit       : a_eff = 0.5333*A     (derate factor  = 1.875)")
print("  quintic jerk  limit       : a_eff = 0.4162*sqrt(Jv)")
print("\n  The quintic's constant 1.875 sits INSIDE the existing 1.0-2.0 range,")
print("  so a machine already tuned for grblHAL jerk mode will not see a large")
print("  change in effective acceleration -- it loses some accel at high speed")
print("  and gains a little at low speed.")
