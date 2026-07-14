package com.ruckig;

/**
 * Port of {@code TargetCalculator} from {@code calculator_target.hpp} (upstream v0.17.3).
 * Dynamic DoFs only; Community (no cloud waypoints).
 */
public final class TargetCalculator {

    private static final double EPS = Utils.DBL_EPSILON;
    private static final boolean RETURN_ERROR_AT_MAXIMAL_DURATION = true;

    public final int degrees_of_freedom;

    private final double[] new_phase_control;
    private final double[] pd;
    private final double[] possible_t_syncs;
    private final int[] idx;

    private final Block[] blocks;
    private final double[] inp_min_velocity;
    private final double[] inp_min_acceleration;
    private final ControlInterface[] inp_per_dof_control_interface;
    private final Synchronization[] inp_per_dof_synchronization;

    private final double[] brakeState = new double[3];

    public TargetCalculator(int dofs) {
        this.degrees_of_freedom = dofs;
        blocks = new Block[dofs];
        for (int i = 0; i < dofs; ++i) {
            blocks[i] = new Block();
        }
        inp_min_velocity = new double[dofs];
        inp_min_acceleration = new double[dofs];
        inp_per_dof_control_interface = new ControlInterface[dofs];
        inp_per_dof_synchronization = new Synchronization[dofs];
        new_phase_control = new double[dofs];
        pd = new double[dofs];
        possible_t_syncs = new double[3 * dofs + 1];
        idx = new int[3 * dofs + 1];
    }

    private boolean is_input_collinear(InputParameter inp, Profile.Direction limiting_direction, int limiting_dof) {
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            pd[dof] = inp.target_position[dof] - inp.current_position[dof];
        }

        double[] scale_vector = null;
        int scale_dof = -1;
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            if (inp_per_dof_synchronization[dof] != Synchronization.Phase) {
                continue;
            }

            if (inp_per_dof_control_interface[dof] == ControlInterface.Position && Math.abs(pd[dof]) > EPS) {
                scale_vector = pd;
                scale_dof = dof;
                break;

            } else if (Math.abs(inp.current_velocity[dof]) > EPS) {
                scale_vector = inp.current_velocity;
                scale_dof = dof;
                break;

            } else if (Math.abs(inp.current_acceleration[dof]) > EPS) {
                scale_vector = inp.current_acceleration;
                scale_dof = dof;
                break;

            } else if (Math.abs(inp.target_velocity[dof]) > EPS) {
                scale_vector = inp.target_velocity;
                scale_dof = dof;
                break;

            } else if (Math.abs(inp.target_acceleration[dof]) > EPS) {
                scale_vector = inp.target_acceleration;
                scale_dof = dof;
                break;
            }
        }

        if (scale_dof < 0) {
            return false;
        }

        final double scale = scale_vector[scale_dof];
        final double pd_scale = pd[scale_dof] / scale;
        final double v0_scale = inp.current_velocity[scale_dof] / scale;
        final double vf_scale = inp.target_velocity[scale_dof] / scale;
        final double a0_scale = inp.current_acceleration[scale_dof] / scale;
        final double af_scale = inp.target_acceleration[scale_dof] / scale;

        final double scale_limiting = scale_vector[limiting_dof];
        double control_limiting = (limiting_direction == Profile.Direction.UP) ? inp.max_jerk[limiting_dof] : -inp.max_jerk[limiting_dof];
        if (Double.isInfinite(inp.max_jerk[limiting_dof])) {
            control_limiting = (limiting_direction == Profile.Direction.UP) ? inp.max_acceleration[limiting_dof] : inp_min_acceleration[limiting_dof];
        }

        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            if (inp_per_dof_synchronization[dof] != Synchronization.Phase) {
                continue;
            }

            final double current_scale = scale_vector[dof];
            if (
                (inp_per_dof_control_interface[dof] == ControlInterface.Position && Math.abs(pd[dof] - pd_scale * current_scale) > EPS)
                || Math.abs(inp.current_velocity[dof] - v0_scale * current_scale) > EPS
                || Math.abs(inp.current_acceleration[dof] - a0_scale * current_scale) > EPS
                || Math.abs(inp.target_velocity[dof] - vf_scale * current_scale) > EPS
                || Math.abs(inp.target_acceleration[dof] - af_scale * current_scale) > EPS
            ) {
                return false;
            }

            new_phase_control[dof] = control_limiting * current_scale / scale_limiting;
        }

        return true;
    }

    /**
     * @param t_min_present whether minimum_duration is set
     * @param t_min value if present
     * @param t_sync_out length-1 for result duration
     * @param limiting_dof_out length-1; -1 means nullopt
     * @param profiles traj.profiles[0]
     */
    private boolean synchronize(boolean t_min_present, double t_min, double[] t_sync_out, int[] limiting_dof_out, Profile[] profiles, boolean discrete_duration, double delta_time) {
        boolean any_interval = false;
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            if (inp_per_dof_synchronization[dof] == Synchronization.None) {
                possible_t_syncs[dof] = 0.0;
                possible_t_syncs[degrees_of_freedom + dof] = Double.POSITIVE_INFINITY;
                possible_t_syncs[2 * degrees_of_freedom + dof] = Double.POSITIVE_INFINITY;
                continue;
            }

            possible_t_syncs[dof] = blocks[dof].t_min;
            possible_t_syncs[degrees_of_freedom + dof] = blocks[dof].has_a ? blocks[dof].a.right : Double.POSITIVE_INFINITY;
            possible_t_syncs[2 * degrees_of_freedom + dof] = blocks[dof].has_b ? blocks[dof].b.right : Double.POSITIVE_INFINITY;
            any_interval |= blocks[dof].has_a || blocks[dof].has_b;
        }
        possible_t_syncs[3 * degrees_of_freedom] = t_min_present ? t_min : Double.POSITIVE_INFINITY;
        any_interval |= t_min_present;

        if (discrete_duration) {
            for (int i = 0; i < possible_t_syncs.length; ++i) {
                double possible_t_sync = possible_t_syncs[i];
                if (Double.isInfinite(possible_t_sync)) {
                    continue;
                }

                final double remainder = possible_t_sync % delta_time; // in [0, delta_time) for positive
                // Java % can be negative; for positive durations OK
                if (remainder > EPS) {
                    possible_t_syncs[i] = possible_t_sync + delta_time - remainder;
                }
            }
        }

        final int idx_end = any_interval ? (3 * degrees_of_freedom + 1) : degrees_of_freedom;
        for (int i = 0; i < idx_end; ++i) {
            idx[i] = i;
        }
        // Sort idx[0..idx_end) by possible_t_syncs
        for (int i = 1; i < idx_end; ++i) {
            int key = idx[i];
            int j = i - 1;
            while (j >= 0 && possible_t_syncs[idx[j]] > possible_t_syncs[key]) {
                idx[j + 1] = idx[j];
                --j;
            }
            idx[j + 1] = key;
        }

        // Start at last tmin (or worse)
        for (int ii = degrees_of_freedom - 1; ii < idx_end; ++ii) {
            final int i = idx[ii];
            final double possible_t_sync = possible_t_syncs[i];
            boolean is_blocked = false;
            for (int dof = 0; dof < degrees_of_freedom; ++dof) {
                if (inp_per_dof_synchronization[dof] == Synchronization.None) {
                    continue;
                }
                if (blocks[dof].is_blocked(possible_t_sync)) {
                    is_blocked = true;
                    break;
                }
            }
            final double t_min_val = t_min_present ? t_min : 0.0;
            if (is_blocked || possible_t_sync < t_min_val || Double.isInfinite(possible_t_sync)) {
                continue;
            }

            t_sync_out[0] = possible_t_sync;
            if (i == 3 * degrees_of_freedom) {
                limiting_dof_out[0] = -1;
                return true;
            }

            final int quot = i / degrees_of_freedom;
            final int rem = i % degrees_of_freedom;
            limiting_dof_out[0] = rem;
            switch (quot) {
                case 0:
                    profiles[rem].copyFrom(blocks[rem].p_min);
                    break;
                case 1:
                    profiles[rem].copyFrom(blocks[rem].a.profile);
                    break;
                case 2:
                    profiles[rem].copyFrom(blocks[rem].b.profile);
                    break;
            }
            return true;
        }

        return false;
    }

    public int calculate(InputParameter inp, Trajectory traj, double delta_time, boolean[] was_interrupted_out) {
        was_interrupted_out[0] = false;

        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            Profile p = traj.profiles[0][dof];

            inp_min_velocity[dof] = inp.min_velocity != null ? inp.min_velocity[dof] : -inp.max_velocity[dof];
            inp_min_acceleration[dof] = inp.min_acceleration != null ? inp.min_acceleration[dof] : -inp.max_acceleration[dof];
            inp_per_dof_control_interface[dof] = inp.per_dof_control_interface != null ? inp.per_dof_control_interface[dof] : inp.control_interface;
            inp_per_dof_synchronization[dof] = inp.per_dof_synchronization != null ? inp.per_dof_synchronization[dof] : inp.synchronization;

            if (!inp.enabled[dof]) {
                p.p[7] = inp.current_position[dof];
                p.v[7] = inp.current_velocity[dof];
                p.a[7] = inp.current_acceleration[dof];
                p.t_sum[6] = 0.0;
                blocks[dof].t_min = 0.0;
                blocks[dof].has_a = false;
                blocks[dof].has_b = false;
                continue;
            }

            // Calculate brake
            switch (inp_per_dof_control_interface[dof]) {
                case Position: {
                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                        p.brake.get_position_brake_trajectory(inp.current_velocity[dof], inp.current_acceleration[dof], inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                    } else if (!Double.isInfinite(inp.max_acceleration[dof])) {
                        p.brake.get_second_order_position_brake_trajectory(inp.current_velocity[dof], inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                    }
                    p.set_boundary(inp.current_position[dof], inp.current_velocity[dof], inp.current_acceleration[dof], inp.target_position[dof], inp.target_velocity[dof], inp.target_acceleration[dof]);
                } break;
                case Velocity: {
                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                        p.brake.get_velocity_brake_trajectory(inp.current_acceleration[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                    } else {
                        p.brake.get_second_order_velocity_brake_trajectory();
                    }
                    p.set_boundary_for_velocity(inp.current_position[dof], inp.current_velocity[dof], inp.current_acceleration[dof], inp.target_velocity[dof], inp.target_acceleration[dof]);
                } break;
            }

            // Finalize pre-trajectories
            if (!Double.isInfinite(inp.max_jerk[dof])) {
                brakeState[0] = p.p[0];
                brakeState[1] = p.v[0];
                brakeState[2] = p.a[0];
                p.brake.finalize(brakeState);
                p.p[0] = brakeState[0];
                p.v[0] = brakeState[1];
                p.a[0] = brakeState[2];
            } else if (!Double.isInfinite(inp.max_acceleration[dof])) {
                brakeState[0] = p.p[0];
                brakeState[1] = p.v[0];
                brakeState[2] = p.a[0];
                p.brake.finalize_second_order(brakeState);
                p.p[0] = brakeState[0];
                p.v[0] = brakeState[1];
                p.a[0] = brakeState[2];
            }

            boolean found_profile = false;
            switch (inp_per_dof_control_interface[dof]) {
                case Position: {
                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                        PositionThirdOrderStep1 step1 = new PositionThirdOrderStep1(p.p[0], p.v[0], p.a[0], p.pf, p.vf, p.af, inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                        found_profile = step1.get_profile(p, blocks[dof]);
                    } else if (!Double.isInfinite(inp.max_acceleration[dof])) {
                        PositionSecondOrderStep1 step1 = new PositionSecondOrderStep1(p.p[0], p.v[0], p.pf, p.vf, inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                        found_profile = step1.get_profile(p, blocks[dof]);
                    } else {
                        PositionFirstOrderStep1 step1 = new PositionFirstOrderStep1(p.p[0], p.pf, inp.max_velocity[dof], inp_min_velocity[dof]);
                        found_profile = step1.get_profile(p, blocks[dof]);
                    }
                } break;
                case Velocity: {
                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                        VelocityThirdOrderStep1 step1 = new VelocityThirdOrderStep1(p.v[0], p.a[0], p.vf, p.af, inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                        found_profile = step1.get_profile(p, blocks[dof]);
                    } else {
                        VelocitySecondOrderStep1 step1 = new VelocitySecondOrderStep1(p.v[0], p.vf, inp.max_acceleration[dof], inp_min_acceleration[dof]);
                        found_profile = step1.get_profile(p, blocks[dof]);
                    }
                } break;
            }

            if (!found_profile) {
                final boolean has_zero_limits = (inp.max_acceleration[dof] == 0.0 || inp_min_acceleration[dof] == 0.0 || inp.max_jerk[dof] == 0.0);
                if (has_zero_limits) {
                    return Result.ErrorZeroLimits;
                } else {
                    return Result.ErrorExecutionTimeCalculation;
                }
            }

            traj.independent_min_durations[dof] = blocks[dof].t_min;
        }

        final boolean discrete_duration = (inp.duration_discretization == DurationDiscretization.Discrete);
        if (degrees_of_freedom == 1 && !inp.has_minimum_duration && !discrete_duration) {
            traj.duration = blocks[0].t_min;
            traj.profiles[0][0].copyFrom(blocks[0].p_min);
            traj.cumulative_times[0] = traj.duration;
            return Result.Working;
        }

        int[] limiting_dof = new int[] { -1 };
        double[] t_sync = new double[1];
        final boolean found_synchronization = synchronize(inp.has_minimum_duration, inp.minimum_duration, t_sync, limiting_dof, traj.profiles[0], discrete_duration, delta_time);
        if (!found_synchronization) {
            boolean has_zero_limits = false;
            for (int dof = 0; dof < degrees_of_freedom; ++dof) {
                if (inp.max_acceleration[dof] == 0.0 || inp_min_acceleration[dof] == 0.0 || inp.max_jerk[dof] == 0.0) {
                    has_zero_limits = true;
                    break;
                }
            }

            if (has_zero_limits) {
                return Result.ErrorZeroLimits;
            } else {
                return Result.ErrorSynchronizationCalculation;
            }
        }
        traj.duration = t_sync[0];

        // None Synchronization
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            if (inp.enabled[dof] && inp_per_dof_synchronization[dof] == Synchronization.None) {
                traj.profiles[0][dof].copyFrom(blocks[dof].p_min);
                if (blocks[dof].t_min > traj.duration) {
                    traj.duration = blocks[dof].t_min;
                    limiting_dof[0] = dof;
                }
            }
        }
        traj.cumulative_times[0] = traj.duration;

        if (RETURN_ERROR_AT_MAXIMAL_DURATION) {
            if (traj.duration > 7.6e3) {
                return Result.ErrorTrajectoryDuration;
            }
        }

        if (traj.duration == 0.0) {
            for (int dof = 0; dof < degrees_of_freedom; ++dof) {
                traj.profiles[0][dof].copyFrom(blocks[dof].p_min);
            }
            return Result.Working;
        }

        boolean all_none = true;
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            if (inp_per_dof_synchronization[dof] != Synchronization.None) {
                all_none = false;
                break;
            }
        }
        if (!discrete_duration && all_none) {
            return Result.Working;
        }

        // Phase Synchronization
        boolean any_phase = false;
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            if (inp_per_dof_synchronization[dof] == Synchronization.Phase) {
                any_phase = true;
                break;
            }
        }
        if (limiting_dof[0] >= 0 && any_phase) {
            final Profile p_limiting = traj.profiles[0][limiting_dof[0]];
            if (is_input_collinear(inp, p_limiting.direction, limiting_dof[0])) {
                boolean found_time_synchronization = true;
                for (int dof = 0; dof < degrees_of_freedom; ++dof) {
                    if (!inp.enabled[dof] || dof == limiting_dof[0] || inp_per_dof_synchronization[dof] != Synchronization.Phase) {
                        continue;
                    }

                    Profile p = traj.profiles[0][dof];
                    final double t_profile = traj.duration - p.brake.duration - p.accel.duration;

                    System.arraycopy(p_limiting.t, 0, p.t, 0, 7);
                    p.control_signs = p_limiting.control_signs;

                    switch (inp_per_dof_control_interface[dof]) {
                        case Position: {
                            switch (p.control_signs) {
                                case UDDU: {
                                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                                        found_time_synchronization &= p.check_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                                    } else if (!Double.isInfinite(inp.max_acceleration[dof])) {
                                        found_time_synchronization &= p.check_for_second_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], -new_phase_control[dof], inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                                    } else {
                                        found_time_synchronization &= p.check_for_first_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_velocity[dof], inp_min_velocity[dof]);
                                    }
                                } break;
                                case UDUD: {
                                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                                        found_time_synchronization &= p.check_with_timing(Profile.ControlSigns.UDUD, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                                    } else {
                                        found_time_synchronization &= p.check_for_second_order_with_timing(Profile.ControlSigns.UDUD, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], -new_phase_control[dof], inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                                    }
                                } break;
                            }
                        } break;
                        case Velocity: {
                            switch (p.control_signs) {
                                case UDDU: {
                                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                                        found_time_synchronization &= p.check_for_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                                    } else {
                                        found_time_synchronization &= p.check_for_second_order_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                                    }
                                } break;
                                case UDUD: {
                                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                                        found_time_synchronization &= p.check_for_velocity_with_timing(Profile.ControlSigns.UDUD, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                                    } else {
                                        found_time_synchronization &= p.check_for_second_order_velocity_with_timing(Profile.ControlSigns.UDUD, Profile.ReachedLimits.NONE, t_profile, new_phase_control[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                                    }
                                } break;
                            }
                        } break;
                    }

                    p.limits = p_limiting.limits;
                }

                boolean only_phase_or_none = true;
                for (int dof = 0; dof < degrees_of_freedom; ++dof) {
                    Synchronization s = inp_per_dof_synchronization[dof];
                    if (s != Synchronization.Phase && s != Synchronization.None) {
                        only_phase_or_none = false;
                        break;
                    }
                }
                if (found_time_synchronization && only_phase_or_none) {
                    return Result.Working;
                }
            }
        }

        // Time Synchronization
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            final boolean skip_synchronization = (dof == limiting_dof[0] || inp_per_dof_synchronization[dof] == Synchronization.None) && !discrete_duration;
            if (!inp.enabled[dof] || skip_synchronization) {
                continue;
            }

            Profile p = traj.profiles[0][dof];
            final double t_profile = traj.duration - p.brake.duration - p.accel.duration;

            if (inp_per_dof_synchronization[dof] == Synchronization.TimeIfNecessary && Math.abs(inp.target_velocity[dof]) < EPS && Math.abs(inp.target_acceleration[dof]) < EPS) {
                p.copyFrom(blocks[dof].p_min);
                continue;
            }

            if (Math.abs(t_profile - blocks[dof].t_min) < 2 * EPS) {
                p.copyFrom(blocks[dof].p_min);
                continue;
            } else if (blocks[dof].has_a && Math.abs(t_profile - blocks[dof].a.right) < 2 * EPS) {
                p.copyFrom(blocks[dof].a.profile);
                continue;
            } else if (blocks[dof].has_b && Math.abs(t_profile - blocks[dof].b.right) < 2 * EPS) {
                p.copyFrom(blocks[dof].b.profile);
                continue;
            }

            boolean found_time_synchronization = false;
            switch (inp_per_dof_control_interface[dof]) {
                case Position: {
                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                        PositionThirdOrderStep2 step2 = new PositionThirdOrderStep2(t_profile, p.p[0], p.v[0], p.a[0], p.pf, p.vf, p.af, inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                        found_time_synchronization = step2.get_profile(p);
                    } else if (!Double.isInfinite(inp.max_acceleration[dof])) {
                        PositionSecondOrderStep2 step2 = new PositionSecondOrderStep2(t_profile, p.p[0], p.v[0], p.pf, p.vf, inp.max_velocity[dof], inp_min_velocity[dof], inp.max_acceleration[dof], inp_min_acceleration[dof]);
                        found_time_synchronization = step2.get_profile(p);
                    } else {
                        PositionFirstOrderStep2 step2 = new PositionFirstOrderStep2(t_profile, p.p[0], p.pf, inp.max_velocity[dof], inp_min_velocity[dof]);
                        found_time_synchronization = step2.get_profile(p);
                    }
                } break;
                case Velocity: {
                    if (!Double.isInfinite(inp.max_jerk[dof])) {
                        VelocityThirdOrderStep2 step2 = new VelocityThirdOrderStep2(t_profile, p.v[0], p.a[0], p.vf, p.af, inp.max_acceleration[dof], inp_min_acceleration[dof], inp.max_jerk[dof]);
                        found_time_synchronization = step2.get_profile(p);
                    } else {
                        VelocitySecondOrderStep2 step2 = new VelocitySecondOrderStep2(t_profile, p.v[0], p.vf, inp.max_acceleration[dof], inp_min_acceleration[dof]);
                        found_time_synchronization = step2.get_profile(p);
                    }
                } break;
            }
            if (!found_time_synchronization) {
                return Result.ErrorSynchronizationCalculation;
            }
        }

        return Result.Working;
    }
}
