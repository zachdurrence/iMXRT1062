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
