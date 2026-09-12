            prep.mm_complete = 0.0f; // Default velocity profile complete at 0.0mm from end of block.
            float inv_2_accel = 0.5f / pl_block->acceleration;

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
                // Compute or recompute velocity profile parameters of the prepped planner block.
                prep.ramp_type = Ramp_Accel; // Initialize as acceleration ramp.
                prep.accelerate_until = pl_block->millimeters;

                float exit_speed_sqr;
                if (sys.step_control.execute_sys_motion)
                    prep.exit_speed = exit_speed_sqr = 0.0f; // Enforce stop at end of system motion.
                else {
                    exit_speed_sqr = plan_get_exec_block_exit_speed_sqr();
                    prep.exit_speed = sqrtf(exit_speed_sqr);
                }

                float nominal_speed = plan_compute_profile_nominal_speed(pl_block);
                float nominal_speed_sqr = nominal_speed * nominal_speed;
                float intersect_distance = 0.5f * (pl_block->millimeters + inv_2_accel * (pl_block->entry_speed_sqr - exit_speed_sqr));

                prep.target_feed = nominal_speed;

                if (pl_block->entry_speed_sqr > nominal_speed_sqr) { // Only occurs during override reductions.

                    prep.accelerate_until = pl_block->millimeters - inv_2_accel * (pl_block->entry_speed_sqr - nominal_speed_sqr);

                    if (prep.accelerate_until <= 0.0f) { // Deceleration-only.
                        prep.ramp_type = Ramp_Decel;
                        // prep.decelerate_after = pl_block->millimeters;
                        // prep.maximum_speed = prep.current_speed;

                        // Compute override block exit speed since it doesn't match the planner exit speed.
                        prep.exit_speed = sqrtf(pl_block->entry_speed_sqr - 2.0f * pl_block->acceleration * pl_block->millimeters);
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
                            prep.maximum_speed = sqrtf(2.0f * pl_block->acceleration * intersect_distance + exit_speed_sqr);
                        }
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
            }

