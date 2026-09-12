            prep.mm_complete = 0.0f; // Default velocity profile complete at 0.0mm from end of block.
            // Acceleration used to lay out THIS block's profile. Normally the
            // planner's value; the optional per-block jerk clamp may reduce it.
            float block_accel = pl_block->acceleration;
            float inv_2_accel = 0.5f / block_accel;

#if ENABLE_JERK_ACCELERATION
            // Disarm any in-flight quintic ramp. This block runs both on a new
            // planner block and on a mid-block velocity profile recalculation
            // (feed/rapid override change, feed hold, replan), all of which move
            // the ramp's endpoints out from under it. The ramp is re-armed on the
            // next segment from the *current* speed and the *new* distance budget,
            // so the restart is velocity continuous; acceleration restarts from
            // zero, which is slightly slower but never exceeds a limit.
            prep.ramp_t = 0.0f;
#endif

            if (sys.step_control.execute_hold) { // [Forced Deceleration to Zero Velocity]
                // Compute velocity profile parameters for a feed hold in-progress. This profile overrides
                // the planner block profile, enforcing a deceleration to zero speed.
                prep.ramp_type = Ramp_Decel;
                // Compute decelerate distance relative to end of block.
                float decel_dist = pl_block->millimeters - inv_2_accel * pl_block->entry_speed_sqr;
                if(decel_dist < -0.0001f) {
                    // Deceleration through entire planner block. End of feed hold is not in this block.
                    prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2.0f * pl_block->acceleration * pl_block->millimeters);
                } else {
                    prep.mm_complete = decel_dist < 0.0001f ? 0.0f : decel_dist; // End of feed hold.
                    prep.exit_speed = 0.0f;
                }
            } else { // [Normal Operation]

                float exit_speed_sqr;
                if (sys.step_control.execute_sys_motion)
                    prep.exit_speed = exit_speed_sqr = 0.0f; // Enforce stop at end of system motion.
                else {
                    exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
                    prep.exit_speed = sqrtf(exit_speed_sqr);
                }

                float nominal_speed = plan_compute_profile_nominal_speed(pl_block);
                float nominal_speed_sqr = nominal_speed * nominal_speed;

                prep.target_feed = nominal_speed;

#if ENABLE_JERK_ACCELERATION && JERK_LIMIT_SHORT_MOVES
                uint_fast8_t jerk_pass = prep.jerk ? JERK_LIMIT_PASSES : 0;
                bool clampable;
              do {
                clampable = false;
#endif
                // Compute or recompute velocity profile parameters of the prepped planner block.
                prep.ramp_type = Ramp_Accel; // Initialize as acceleration ramp.
                prep.accelerate_until = pl_block->millimeters;

                float intersect_distance = 0.5f * (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));

                if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.

                    prep.accelerate_until = pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);

                    if (prep.accelerate_until <= 0.0f) { // Deceleration-only.
                        prep.ramp_type = Ramp_Decel;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;

                        // Compute override block exit speed since it doesn't match the planner exit speed.
                        prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2.0f * block_accel * pl_block->millimeters);
                        prep.recalculate.decel_override = On; // Flag to load next block as deceleration override.

                        // TODO: Determine correct handling of parameters in deceleration-only.
                        // Can be tricky since entry speed will be current speed, as in feed holds.
                        // Also, look into near-zero speed handling issues with this.

                    } else {
                        // Decelerate to cruise or cruise-decelerate types. Guaranteed to intersect updated plan.
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr); // Should always be >= 0.0 due to planner reinit.
                        prep.maximum_speed = nominal_speed;
                        prep.ramp_type = Ramp_DecelOverride;
                    }
                } else if (intersect_distance > 0.0f) {
                    if (intersect_distance < pl_block->millimeters) { // Either trapezoid or triangle types
                        // NOTE: For acceleration-cruise and cruise-only types, following calculation will be 0.0.
                        prep.decelerate_after = inv_2_accel * (nominal_speed_sqr - exit_speed_sqr);
                        if (prep.decelerate_after < intersect_distance) { // Trapezoid type
                            prep.maximum_speed = nominal_speed;
                            if (pl_block->entry_speed_sqr == nominal_speed_sqr) {
                                // Cruise-deceleration or cruise-only type.
                                prep.ramp_type = Ramp_Cruise;
                            } else {
                                // Full-trapezoid or acceleration-cruise types
                                prep.accelerate_until -= inv_2_accel * (nominal_speed_sqr - pl_block->entry_speed_sqr);
                            }
                        } else { // Triangle type
                            prep.accelerate_until = prep.decelerate_after = intersect_distance;
                            prep.maximum_speed = sqrtf(2.0f * block_accel * intersect_distance + exit_speed_sqr);
                        }
#if ENABLE_JERK_ACCELERATION && JERK_LIMIT_SHORT_MOVES
                        // Trapezoid and triangle are the shapes the clamp may
                        // reduce. Override reductions and deceleration-only
                        // profiles are left alone -- their exit speed is a
                        // contract with the next block.
                        clampable = true;
#endif
                    } else { // Deceleration-only type
                        prep.ramp_type = Ramp_Decel;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;
                    }
                } else { // Acceleration-only type
                    prep.accelerate_until = 0.0f;
                    // prep.decelerate_after = 0.0f;
                    prep.maximum_speed = prep.exit_speed;
                }
#if ENABLE_JERK_ACCELERATION && JERK_LIMIT_SHORT_MOVES
                if(jerk_pass == 0 || !clampable)
                    break;

                // Constrain the acceleration so both of this block's ramps stay
                // inside pl_block->jerk, using their real geometry.
                float limited = jerk_ramp_accel_limit(sqrtf(pl_block->entry_speed_sqr), prep.maximum_speed,
                                                       pl_block->millimeters - prep.accelerate_until, block_accel);
                limited = jerk_ramp_accel_limit(prep.maximum_speed, prep.exit_speed,
                                                 prep.decelerate_after, limited);

                if(limited >= block_accel)
                    break; // Already compliant.

                // Lowering the acceleration moves the accel/decel intersection.
                // Far enough and the profile would flip to acceleration-only or
                // deceleration-only, neither of which this clamp is allowed to
                // produce (their speeds are a contract with the neighbouring
                // blocks). Verify the shape survives BEFORE committing, so a
                // rejected candidate leaves the current profile untouched.
                float inv_2_limited = 0.5f / limited;
                float intersect_limited = 0.5f * (pl_block->millimeters +
                                                   inv_2_limited * (pl_block->entry_speed_sqr - exit_speed_sqr));

                if(intersect_limited <= 0.0f || intersect_limited >= pl_block->millimeters)
                    break;

                block_accel = limited;
                inv_2_accel = inv_2_limited;

              } while(jerk_pass--);
#endif
            }

