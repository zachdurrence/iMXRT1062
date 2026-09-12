// Peak jerk of the quintic ramp is (10/sqrt 3) * dv/T^2. Exact; see the
// derivation in the ramp comment below and in plan_buffer_line().
#define JERK_QUINTIC_FACTOR 5.7735026919f

// Opt-in per-block jerk clamp for short moves. Off by default: it closes a real
// gap but costs feed rate on the moves it touches, and how much that matters
// depends on the toolpath. See the block comment on jerk_ramp_accel_limit().
#ifndef JERK_LIMIT_SHORT_MOVES
#define JERK_LIMIT_SHORT_MOVES 0
#endif

// Minimum ramp length, in DT_SEGMENT ticks, for the clamp to consider a ramp's
// jerk physically meaningful rather than segment quantization.
#ifndef JERK_LIMIT_MIN_TICKS
#define JERK_LIMIT_MIN_TICKS 4
#endif

// Fixed-point iterations for the clamp. On a triangle the peak speed depends on
// the acceleration that the clamp is solving for, so the profile is recomputed.
// Convergence is fast (a_next ~ a^(1/4) for a triangle); 3 is ample.
#ifndef JERK_LIMIT_PASSES
#define JERK_LIMIT_PASSES 5
#endif
static bool jerk_ramp_advance (float *mm_remaining, float *time_var, float ramp_end, float target_speed)
{
    if(prep.ramp_t == 0.0f) { // Arm a new ramp from the current state.

        prep.ramp_v0 = prep.current_speed;
        prep.ramp_v1 = target_speed;
        prep.ramp_mm = *mm_remaining - ramp_end;
        prep.ramp_end = ramp_end;
        prep.ramp_tau = 0.0f;

        float vsum = prep.ramp_v0 + prep.ramp_v1;

        if(prep.ramp_mm > 0.0f && vsum > 0.0f) {
            prep.ramp_inv_vsum = 1.0f / vsum;
            prep.ramp_t = 2.0f * prep.ramp_mm * prep.ramp_inv_vsum;
        } else {
            // Degenerate ramp: no distance left to cover, or no speed to cover it
            // with. Complete immediately without consuming segment time so the
            // caller's do-while loop moves on to the next ramp state.
            *mm_remaining = ramp_end;
            *time_var = 0.0f;
            prep.current_speed = target_speed;

            return true;
        }
    }

    float tau = prep.ramp_tau + *time_var / prep.ramp_t;

    if(tau < 1.0f) { // Mid-ramp.

        float dv = prep.ramp_v1 - prep.ramp_v0;
        float tau2 = tau * tau;
        float tau4 = tau2 * tau2;

        // Fraction of the ramp distance covered at tau.
        float snorm = (2.0f * prep.ramp_v0 * tau + dv * tau4 * (2.0f * tau2 - 6.0f * tau + 5.0f)) * prep.ramp_inv_vsum;

        *mm_remaining = prep.ramp_end + prep.ramp_mm * (1.0f - snorm);
        prep.current_speed = prep.ramp_v0 + dv * tau2 * tau * (10.0f - tau * (15.0f - 6.0f * tau));
        prep.ramp_tau = tau;

        return false;
    }

    // End of ramp. Distance budget and target speed are reached simultaneously
    // by construction, so snap both. time_var comes from the ramp's own
    // remaining duration -- well conditioned even as target_speed -> 0.
    *time_var = (1.0f - prep.ramp_tau) * prep.ramp_t;
    *mm_remaining = prep.ramp_end;
    prep.current_speed = target_speed;
    prep.ramp_t = 0.0f; // Disarm so the next ramp re-arms from the new state.

    return true;
}

#if JERK_LIMIT_SHORT_MOVES

/*  PER-BLOCK JERK CLAMP
    ---------------------------------------------------------------------------
    plan_buffer_line() sizes the block's acceleration from programmed_rate,
    because entry and exit speeds are not known until the lookahead passes have
    run. Peak jerk goes as

        j = JERK_QUINTIC_FACTOR * a^2 / dv

    so a block whose ACTUAL speed change is smaller than programmed_rate exceeds
    the jerk setting by programmed_rate/dv. Measured before this clamp: 2.3x on a
    3 mm move at F3000, 12.5x at 0.1 mm, 16.3x on a 1 mm rapid.

    Note the sign. Lowering peak SPEED shortens the ramp and makes jerk worse --
    T = dv/a, so j rises as dv falls. The lever is acceleration, and it enters
    squared. Inverting the relation above gives the fix:

        a <= sqrt(J * dv / JERK_QUINTIC_FACTOR)

    which is exactly the planner's own second bound, evaluated with the block's
    real dv instead of programmed_rate. By profile-setup time the stepper knows
    that dv, so the correction belongs here and not in the planner -- applying it
    to the lookahead's entry-speed limits instead throttles every short block
    whether or not it ramps, and collapses arc feed rates to 0.27-0.34x.

    Applied per block against the real geometry it leaves chained short blocks
    completely alone (0 of 200 and 0 of 400 arc chords clamped in simulation),
    because arc chords cruise: dv per block is ~0 and there is no ramp to clamp.

    Only ramps spanning at least JERK_LIMIT_MIN_TICKS segments are considered. A
    shorter ramp is delivered by the segment generator as one or two velocity
    steps, so its apparent jerk is DT_SEGMENT quantization rather than curve
    shape; no acceleration change can affect it, and clamping on account of it
    would spend feed rate for nothing.

    NOT applied to feed holds, which keep the planner's acceleration so that
    stopping distance is not extended, nor to override-reduction profiles, whose
    exit speed is a contract with the next block.

    COST: real, and paid only by short isolated moves. The clamp is exact --
    bisection against this bound reproduces the closed form (J*sqrt(L)/JF)^(2/3)
    for a symmetric triangle to within float epsilon -- so what it costs is
    simply what the jerk limit costs. At the shipped settings the threshold is
    around 15 mm; below that, feed rate falls as the move shortens.
*/
static float jerk_ramp_accel_limit (float v0, float v1, float mm, float accel)
{
    float vsum = v0 + v1;

    if(mm <= 0.0f || vsum <= 0.0f)
        return accel;

    float t = 2.0f * mm / vsum;

    // Sub-resolution ramp: a step, not a curve. Quantization, not jerk.
    if(t < (float)JERK_LIMIT_MIN_TICKS * DT_SEGMENT)
        return accel;

    float dv = v1 > v0 ? v1 - v0 : v0 - v1;

    if(JERK_QUINTIC_FACTOR * dv <= pl_block->jerk * t * t)
        return accel; // Already within the jerk limit.

    float limit = sqrtf(pl_block->jerk * dv * (1.0f / JERK_QUINTIC_FACTOR));

    return limit < accel ? limit : accel;
}

#endif // JERK_LIMIT_SHORT_MOVES
