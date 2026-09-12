        float dt_max = DT_SEGMENT; // Maximum segment time
        float dt = 0.0f; // Initialize segment time
        float time_var = dt_max; // Time worker variable
        float mm_var; // mm - Distance worker variable
        float speed_var; // Speed worker variable
        float mm_remaining = pl_block->millimeters; // New segment distance from end of block.
        float minimum_mm = mm_remaining - prep.req_mm_increment; // Guarantee at least one step.

        if (minimum_mm < 0.0f)
            minimum_mm = 0.0f;

        do {

            switch (prep.ramp_type) {

                case Ramp_DecelOverride:
                    speed_var = pl_block->acceleration * time_var;
                    if ((prep.current_speed - prep.maximum_speed) <= speed_var) {
                        // Cruise or cruise-deceleration types only for deceleration override.
                        mm_remaining = prep.accelerate_until;
                        time_var = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type = Ramp_Cruise;
                        prep.current_speed = prep.maximum_speed;
                    } else {// Mid-deceleration override ramp.
                        mm_remaining -= time_var * (prep.current_speed - 0.5f * speed_var);
                        prep.current_speed -= speed_var;
                    }
#if ENABLE_JERK_ACCELERATION
                    prep.last_accel = 0.0f;  // reset acceleration variable to 0 for next accel ramp
#endif
                    break;

                case Ramp_Accel:
                    // NOTE: Acceleration ramp only computes during first do-while loop.
#if ENABLE_JERK_ACCELERATION
                    if(prep.jerk) {
                        float accel_var = pl_block->jerk * time_var; // Delta acceleration
                        float time_to_jerk = prep.last_accel / pl_block->jerk;
                        //Distance to 0 acceleration at speed (mm == V(0)*T + 1/2 A0*T^2 + 1/6 J*T^3)
                        float jerk_rampdown = time_to_jerk * (prep.current_speed +
                                                              (0.5f * prep.last_accel * time_to_jerk) +
                                                                pl_block->jerk * time_to_jerk * time_to_jerk * (1.0f / 6.0f));
#if 0
                        jlog.data[jlog.idx].s0 = mm_remaining - prep.accelerate_until;
                        jlog.data[jlog.idx].v0 = prep.current_speed;
                        jlog.data[jlog.idx].a0 = prep.last_accel;
                        jlog.data[jlog.idx].da = jlog.idx == 0 ? prep.last_accel : (prep.last_accel - jlog.data[jlog.idx-1].a0);
                        jlog.data[jlog.idx].s = jerk_rampdown;
                        jlog.data[jlog.idx].t = time_to_jerk;
#endif
                        //Distance to 0 acceleration at speed (mm == V(0)*T + 1/2 A0*T^2 + 1/6 J*T^3)
                        if((mm_remaining - prep.accelerate_until) > jerk_rampdown) {
                            //+1.0f to avoid divide by 0 speed, minor effect on jerk ramp - ??
                            // Check if we are on ramp up or ramp down. Ramp down if distance to end of acceleration is less than distance needed to reach 0 acceleration.
                            // Then limit acceleration change by jerk up to max acceleration and update for next segment.
                            // Minimum acceleration jerk per time_var to ensure acceleration completes. Acceleration change at end of ramp is in acceptable jerk range.
                            prep.last_accel = min(prep.last_accel + accel_var, pl_block->max_acceleration);
                        } else {
#if 0
                            if(jlog.rd == 0) jlog.rd = jlog.idx;
                            jlog.data[jlog.idx].ramp_down = true;
#endif
                            prep.last_accel = max(prep.last_accel - accel_var, accel_var);
                        }
                        speed_var = prep.last_accel * time_var;
                    } else
#endif // ENABLE_JERK_ACCELERATION
                    speed_var = pl_block->acceleration * time_var;

                    mm_remaining -= time_var * (prep.current_speed + 0.5f * speed_var);
                    if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
                        // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
                        mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
                        time_var = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type = mm_remaining == prep.decelerate_after ? Ramp_Decel : Ramp_Cruise;
                        prep.current_speed = prep.maximum_speed;
#if ENABLE_JERK_ACCELERATION
                        if(prep.jerk) {
                            prep.last_accel = 0.0f;  // reset acceleration variable to 0 for next accel ramp
#if 0
                            if(prep.ramp_type == Ramp_Decel) jlog.d = jlog.idx;
#endif
                        }
#endif // ENABLE_JERK_ACCELERATION
                    } else // Acceleration only.
                        prep.current_speed += speed_var;
#if 0
                    jlog.data[jlog.idx].accel = prep.ramp_type == Ramp_Accel;
                    jlog.data[jlog.idx].v0 = prep.current_speed;
                    jlog.data[jlog.idx].time_var = time_var;
                    jlog.data[jlog.idx].mm_remaining = mm_remaining;
#endif
                    break;

                case Ramp_Cruise:
                    // NOTE: mm_var used to retain the last mm_remaining for incomplete segment time_var calculations.
                    // NOTE: If maximum_speed*time_var value is too low, round-off can cause mm_var to not change. To
                    //   prevent this, simply enforce a minimum speed threshold in the planner.
                    mm_var = mm_remaining - prep.maximum_speed * time_var;
                    if (mm_var < prep.decelerate_after) { // End of cruise.
                        // Cruise-deceleration junction or end of block.
                        time_var = (mm_remaining - prep.decelerate_after) / prep.maximum_speed;
                        mm_remaining = prep.decelerate_after; // NOTE: 0.0 at EOB
                        prep.ramp_type = Ramp_Decel;
#if 0
                        if(jlog.d == 0) jlog.d = jlog.idx;
#endif
                    } else // Cruising only.
                        mm_remaining = mm_var;
                    break;

                default: // case Ramp_Decel:
                    // NOTE: mm_var used as a misc worker variable to prevent errors when near zero speed.
#if ENABLE_JERK_ACCELERATION
                    if(prep.jerk) {
                        float accel_var = pl_block->jerk * time_var; // Delta acceleration
                        float time_to_jerk = prep.last_accel == 0.0f ? accel_var : (prep.last_accel / pl_block->jerk);
                        float jerk_rampdown = prep.exit_speed +
                                               time_to_jerk * (prep.last_accel -
                                                               (0.5f * pl_block->jerk * time_to_jerk)); // Speedpoint to start ramping down deceleration. (V = a * t - 1/2 j * t^2)
#if 0
                        jlog.data[jlog.idx].s0 = prep.decelerate_after - mm_remaining;
                        jlog.data[jlog.idx].v0 = prep.current_speed;
                        jlog.data[jlog.idx].a0 = prep.last_accel;
                        jlog.data[jlog.idx].da = jlog.idx == 0 ? prep.last_accel : (prep.last_accel - jlog.data[jlog.idx-1].a0);
                        jlog.data[jlog.idx].s = jerk_rampdown;
                        jlog.data[jlog.idx].t = time_to_jerk;
#endif
                        if(prep.current_speed > jerk_rampdown) {
                            // Check if we are on ramp up or ramp down. Ramp down if speed is less than speed needed for reaching 0 acceleration.
                            // Then limit acceleration change by jerk up to max acceleration and update for next segment.
                            // Minimum acceleration of jerk per time_var to ensure deceleration completes. Acceleration change at end of ramp is in acceptable jerk range.
                            prep.last_accel = min(prep.last_accel + accel_var, pl_block->max_acceleration);
                        } else {
#if 0
                            if(jlog.ru == 0) jlog.ru = jlog.idx;
                            jlog.data[jlog.idx].ramp_down = true;
#endif
                            prep.last_accel = max(prep.last_accel - accel_var, accel_var);
                        }
                        speed_var = prep.last_accel * time_var; // Used as delta speed (mm/min)
                    } else
#endif //ENABLE_JERK_ACCELERATION
                    speed_var = pl_block->acceleration * time_var; // Used as delta speed (mm/min)

                    if (prep.current_speed > speed_var) { // Check if at or below zero speed.
                        // Compute distance from end of segment to end of block.
                        mm_var = mm_remaining - time_var * (prep.current_speed - 0.5f * speed_var); // (mm)
                        if (mm_var > prep.mm_complete) {
                            mm_remaining = mm_var;
                                prep.current_speed -= speed_var;
#if ENABLE_JERK_ACCELERATION
                            prep.last_time_var = time_var; // Store for final segment continuity.
                        #endif
                            break;// Segment complete. Exit switch-case statement. Continue do-while loop.
                        }
                    }
                    // Otherwise, at end of block or end of forced-deceleration.
// Otherwise, at end of block or end of forced-deceleration.
#if ENABLE_JERK_ACCELERATION
    if(prep.jerk) {
        // The trapezoidal formula (2*d / (v0+v1)) is numerically unstable under jerk-limited
        // motion when exit_speed approaches zero — denominator collapses and time_var blows up
        // by orders of magnitude, producing an erroneously long final segment.
        //
        // The S-curve is fully executed by this point. Use the duration of the last normal
        // segment to maintain step rate continuity into the final segment — no rate increase,
        // no snap, no drag. Position and exit_speed are snapped to correct values unconditionally
        // after this regardless of time_var.
        time_var = prep.last_time_var;
        prep.last_accel = 0.0f; // Reset for next block's acceleration ramp.
    } else
#endif
    time_var = 2.0f * (mm_remaining - prep.mm_complete) / (prep.current_speed + prep.exit_speed);
                

                    mm_remaining = prep.mm_complete;
                    prep.current_speed = prep.exit_speed;
            }

#if 0
            jlog.data[jlog.idx].v = prep.current_speed;
            jlog.data[jlog.idx].accel = prep.ramp_type == Ramp_Accel;
            jlog.data[jlog.idx].time_var = time_var;
            jlog.data[jlog.idx].mm_remaining = mm_remaining;
#endif

            dt += time_var; // Add computed ramp time to total segment time.

            if (dt < dt_max)
                time_var = dt_max - dt;// **Incomplete** At ramp junction.
            else {
                if (mm_remaining > minimum_mm) { // Check for very slow segments with zero steps.
                    // Increase segment time to ensure at least one step in segment. Override and loop
                    // through distance calculations until minimum_mm or mm_complete.
                    dt_max += DT_SEGMENT;
                    time_var = dt_max - dt;
                } else
                    break; // **Complete** Exit loop. Segment execution time maxed.
            }

        } while (mm_remaining > prep.mm_complete); // **Complete** Exit loop. Profile complete.
