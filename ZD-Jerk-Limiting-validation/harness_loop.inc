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
#if ENABLE_JERK_ACCELERATION
                    // Structurally identical to Ramp_Accel: ramp current_speed to
                    // maximum_speed, ending at accelerate_until. Only the sign of dv
                    // differs, and the quintic is symmetric in that.
                    if(prep.jerk) {
                        if(jerk_ramp_advance(&mm_remaining, &time_var, prep.accelerate_until, prep.maximum_speed))
                            prep.ramp_type = mm_remaining == prep.decelerate_after ? Ramp_Decel : Ramp_Cruise;
                        break;
                    }
#endif
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
                    break;

                case Ramp_Accel:
                    // NOTE: Acceleration ramp only computes during first do-while loop.
#if ENABLE_JERK_ACCELERATION
                    if(prep.jerk) {
#if 0
                        jlog.data[jlog.idx].s0 = prep.ramp_mm - (mm_remaining - prep.accelerate_until);
                        jlog.data[jlog.idx].v0 = prep.current_speed;
                        // Instantaneous accel: dv/T * P'(tau), P' = 30*tau^2*(1-tau)^2.
                        float jlog_tt = prep.ramp_tau * (1.0f - prep.ramp_tau);
                        jlog.data[jlog.idx].a0 = prep.ramp_t == 0.0f ? 0.0f
                                               : 30.0f * (prep.ramp_v1 - prep.ramp_v0) / prep.ramp_t
                                                 * jlog_tt * jlog_tt;
                        jlog.data[jlog.idx].da = prep.ramp_tau;
                        jlog.data[jlog.idx].s = prep.ramp_mm;
                        jlog.data[jlog.idx].t = prep.ramp_t;
#endif
                        if(jerk_ramp_advance(&mm_remaining, &time_var, prep.accelerate_until, prep.maximum_speed)) {
                            prep.ramp_type = mm_remaining == prep.decelerate_after ? Ramp_Decel : Ramp_Cruise;
#if 0
                            if(prep.ramp_type == Ramp_Decel) jlog.d = jlog.idx;
#endif
                        }
                        break;
                    }
#endif // ENABLE_JERK_ACCELERATION
                    speed_var = pl_block->acceleration * time_var;

                    mm_remaining -= time_var * (prep.current_speed + 0.5f * speed_var);
                    if (mm_remaining < prep.accelerate_until) { // End of acceleration ramp.
                        // Acceleration-cruise, acceleration-deceleration ramp junction, or end of block.
                        mm_remaining = prep.accelerate_until; // NOTE: 0.0 at EOB
                        time_var = 2.0f * (pl_block->millimeters - mm_remaining) / (prep.current_speed + prep.maximum_speed);
                        prep.ramp_type = mm_remaining == prep.decelerate_after ? Ramp_Decel : Ramp_Cruise;
                        prep.current_speed = prep.maximum_speed;
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
#if 0
                        jlog.data[jlog.idx].s0 = prep.ramp_mm - (mm_remaining - prep.mm_complete);
                        jlog.data[jlog.idx].v0 = prep.current_speed;
                        float jlog_tt = prep.ramp_tau * (1.0f - prep.ramp_tau);
                        jlog.data[jlog.idx].a0 = prep.ramp_t == 0.0f ? 0.0f
                                               : 30.0f * (prep.ramp_v1 - prep.ramp_v0) / prep.ramp_t
                                                 * jlog_tt * jlog_tt;
                        jlog.data[jlog.idx].da = prep.ramp_tau;
                        jlog.data[jlog.idx].s = prep.ramp_mm;
                        jlog.data[jlog.idx].t = prep.ramp_t;
#endif
                        // Ramp end is the end of the velocity profile (0.0 at end of block,
                        // or the feed hold stop point mid-block). The do-while condition
                        // terminates the segment loop once mm_complete is reached.
                        jerk_ramp_advance(&mm_remaining, &time_var, prep.mm_complete, prep.exit_speed);
                        break;
                    }
#endif //ENABLE_JERK_ACCELERATION
                    speed_var = pl_block->acceleration * time_var; // Used as delta speed (mm/min)

                    if (prep.current_speed > speed_var) { // Check if at or below zero speed.
                        // Compute distance from end of segment to end of block.
                        mm_var = mm_remaining - time_var * (prep.current_speed - 0.5f * speed_var); // (mm)
                        if (mm_var > prep.mm_complete) {
                            mm_remaining = mm_var;
                            prep.current_speed -= speed_var;
                            break;// Segment complete. Exit switch-case statement. Continue do-while loop.
                        }
                    }
                    // Otherwise, at end of block or end of forced-deceleration.
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
