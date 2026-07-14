package com.ruckig;

/**
 * Port of {@code InputParameter} from {@code input_parameter.hpp} (upstream v0.17.3).
 * Dynamic DoFs only ({@code double[]} sized at construction).
 */
public final class InputParameter {

    public final int degrees_of_freedom;

    public ControlInterface control_interface = ControlInterface.Position;
    public Synchronization synchronization = Synchronization.Time;
    public DurationDiscretization duration_discretization = DurationDiscretization.Continuous;

    public final double[] current_position;
    public final double[] current_velocity;
    public final double[] current_acceleration;

    public final double[] target_position;
    public final double[] target_velocity;
    public final double[] target_acceleration;

    public final double[] max_velocity;
    public final double[] max_acceleration;
    public final double[] max_jerk;

    /** Optional min velocity; null means use -max_velocity. */
    public double[] min_velocity;
    /** Optional min acceleration; null means use -max_acceleration. */
    public double[] min_acceleration;

    public final boolean[] enabled;

    /** Optional per-DoF control interface; null means use global. */
    public ControlInterface[] per_dof_control_interface;
    /** Optional per-DoF synchronization; null means use global. */
    public Synchronization[] per_dof_synchronization;

    /** Optional minimum trajectory duration; NaN means unset. */
    public double minimum_duration = Double.NaN;
    public boolean has_minimum_duration = false;

    public InputParameter(int dofs) {
        this.degrees_of_freedom = dofs;
        current_position = new double[dofs];
        current_velocity = new double[dofs];
        current_acceleration = new double[dofs];
        target_position = new double[dofs];
        target_velocity = new double[dofs];
        target_acceleration = new double[dofs];
        max_velocity = new double[dofs];
        max_acceleration = new double[dofs];
        max_jerk = new double[dofs];
        enabled = new boolean[dofs];
        initialize();
    }

    private void initialize() {
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            current_velocity[dof] = 0.0;
            current_acceleration[dof] = 0.0;
            target_velocity[dof] = 0.0;
            target_acceleration[dof] = 0.0;
            max_acceleration[dof] = Double.POSITIVE_INFINITY;
            max_jerk[dof] = Double.POSITIVE_INFINITY;
            enabled[dof] = true;
        }
    }

    private static double v_at_a_zero(double v0, double a0, double j) {
        return v0 + (a0 * a0) / (2 * j);
    }

    public void setMinimumDuration(double duration) {
        minimum_duration = duration;
        has_minimum_duration = true;
    }

    public void clearMinimumDuration() {
        has_minimum_duration = false;
        minimum_duration = Double.NaN;
    }

    /**
     * Validate the input for trajectory calculation.
     * @param throwValidationError if true, throw {@link RuckigError} on failure
     */
    public boolean validate(boolean throwValidationError, boolean check_current_state_within_limits, boolean check_target_state_within_limits) {
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            final double jMax = max_jerk[dof];
            if (Double.isNaN(jMax) || jMax < 0.0) {
                if (throwValidationError) {
                    throw new RuckigError("maximum jerk limit " + jMax + " of DoF " + dof + " should be larger than or equal to zero.");
                }
                return false;
            }

            final double aMax = max_acceleration[dof];
            if (Double.isNaN(aMax) || aMax < 0.0) {
                if (throwValidationError) {
                    throw new RuckigError("maximum acceleration limit " + aMax + " of DoF " + dof + " should be larger than or equal to zero.");
                }
                return false;
            }

            final double aMin = min_acceleration != null ? min_acceleration[dof] : -max_acceleration[dof];
            if (Double.isNaN(aMin) || aMin > 0.0) {
                if (throwValidationError) {
                    throw new RuckigError("minimum acceleration limit " + aMin + " of DoF " + dof + " should be smaller than or equal to zero.");
                }
                return false;
            }

            final double a0 = current_acceleration[dof];
            if (Double.isNaN(a0)) {
                if (throwValidationError) {
                    throw new RuckigError("current acceleration " + a0 + " of DoF " + dof + " should be a valid number.");
                }
                return false;
            }
            final double af = target_acceleration[dof];
            if (Double.isNaN(af)) {
                if (throwValidationError) {
                    throw new RuckigError("target acceleration " + af + " of DoF " + dof + " should be a valid number.");
                }
                return false;
            }

            if (check_current_state_within_limits) {
                if (a0 > aMax) {
                    if (throwValidationError) {
                        throw new RuckigError("current acceleration " + a0 + " of DoF " + dof + " exceeds its maximum acceleration limit " + aMax + ".");
                    }
                    return false;
                }
                if (a0 < aMin) {
                    if (throwValidationError) {
                        throw new RuckigError("current acceleration " + a0 + " of DoF " + dof + " undercuts its minimum acceleration limit " + aMin + ".");
                    }
                    return false;
                }
            }
            if (check_target_state_within_limits) {
                if (af > aMax) {
                    if (throwValidationError) {
                        throw new RuckigError("target acceleration " + af + " of DoF " + dof + " exceeds its maximum acceleration limit " + aMax + ".");
                    }
                    return false;
                }
                if (af < aMin) {
                    if (throwValidationError) {
                        throw new RuckigError("target acceleration " + af + " of DoF " + dof + " undercuts its minimum acceleration limit " + aMin + ".");
                    }
                    return false;
                }
            }

            final double v0 = current_velocity[dof];
            if (Double.isNaN(v0)) {
                if (throwValidationError) {
                    throw new RuckigError("current velocity " + v0 + " of DoF " + dof + " should be a valid number.");
                }
                return false;
            }
            final double vf = target_velocity[dof];
            if (Double.isNaN(vf)) {
                if (throwValidationError) {
                    throw new RuckigError("target velocity " + vf + " of DoF " + dof + " should be a valid number.");
                }
                return false;
            }

            ControlInterface control_interface_ = per_dof_control_interface != null ? per_dof_control_interface[dof] : control_interface;
            if (control_interface_ == ControlInterface.Position) {
                final double p0 = current_position[dof];
                if (Double.isNaN(p0)) {
                    if (throwValidationError) {
                        throw new RuckigError("current position " + p0 + " of DoF " + dof + " should be a valid number.");
                    }
                    return false;
                }
                final double pf = target_position[dof];
                if (Double.isNaN(pf)) {
                    if (throwValidationError) {
                        throw new RuckigError("target position " + pf + " of DoF " + dof + " should be a valid number.");
                    }
                    return false;
                }

                final double vMax = max_velocity[dof];
                if (Double.isNaN(vMax) || vMax < 0.0) {
                    if (throwValidationError) {
                        throw new RuckigError("maximum velocity limit " + vMax + " of DoF " + dof + " should be larger than or equal to zero.");
                    }
                    return false;
                }

                final double vMin = min_velocity != null ? min_velocity[dof] : -max_velocity[dof];
                if (Double.isNaN(vMin) || vMin > 0.0) {
                    if (throwValidationError) {
                        throw new RuckigError("minimum velocity limit " + vMin + " of DoF " + dof + " should be smaller than or equal to zero.");
                    }
                    return false;
                }

                if (check_current_state_within_limits) {
                    if (v0 > vMax) {
                        if (throwValidationError) {
                            throw new RuckigError("current velocity " + v0 + " of DoF " + dof + " exceeds its maximum velocity limit " + vMax + ".");
                        }
                        return false;
                    }
                    if (v0 < vMin) {
                        if (throwValidationError) {
                            throw new RuckigError("current velocity " + v0 + " of DoF " + dof + " undercuts its minimum velocity limit " + vMin + ".");
                        }
                        return false;
                    }
                }
                if (check_target_state_within_limits) {
                    if (vf > vMax) {
                        if (throwValidationError) {
                            throw new RuckigError("target velocity " + vf + " of DoF " + dof + " exceeds its maximum velocity limit " + vMax + ".");
                        }
                        return false;
                    }
                    if (vf < vMin) {
                        if (throwValidationError) {
                            throw new RuckigError("target velocity " + vf + " of DoF " + dof + " undercuts its minimum velocity limit " + vMin + ".");
                        }
                        return false;
                    }
                }

                if (check_current_state_within_limits) {
                    if (a0 > 0 && jMax > 0 && v_at_a_zero(v0, a0, jMax) > vMax) {
                        if (throwValidationError) {
                            throw new RuckigError("DoF " + dof + " will inevitably reach a velocity " + v_at_a_zero(v0, a0, jMax) + " from the current kinematic state that will exceed its maximum velocity limit " + vMax + ".");
                        }
                        return false;
                    }
                    if (a0 < 0 && jMax > 0 && v_at_a_zero(v0, a0, -jMax) < vMin) {
                        if (throwValidationError) {
                            throw new RuckigError("DoF " + dof + " will inevitably reach a velocity " + v_at_a_zero(v0, a0, -jMax) + " from the current kinematic state that will undercut its minimum velocity limit " + vMin + ".");
                        }
                        return false;
                    }
                }
                if (check_target_state_within_limits) {
                    if (af < 0 && jMax > 0 && v_at_a_zero(vf, af, jMax) > vMax) {
                        if (throwValidationError) {
                            throw new RuckigError("DoF " + dof + " will inevitably have reached a velocity " + v_at_a_zero(vf, af, jMax) + " from the target kinematic state that will exceed its maximum velocity limit " + vMax + ".");
                        }
                        return false;
                    }
                    if (af > 0 && jMax > 0 && v_at_a_zero(vf, af, -jMax) < vMin) {
                        if (throwValidationError) {
                            throw new RuckigError("DoF " + dof + " will inevitably have reached a velocity " + v_at_a_zero(vf, af, -jMax) + " from the target kinematic state that will undercut its minimum velocity limit " + vMin + ".");
                        }
                        return false;
                    }
                }
            }
        }

        return true;
    }

    public boolean validate() {
        return validate(true, false, true);
    }

    /** Deep-compare fields relevant to recalculation (operator!= inverted). */
    public boolean differsFrom(InputParameter rhs) {
        if (degrees_of_freedom != rhs.degrees_of_freedom) {
            return true;
        }
        if (control_interface != rhs.control_interface
                || synchronization != rhs.synchronization
                || duration_discretization != rhs.duration_discretization) {
            return true;
        }
        if (has_minimum_duration != rhs.has_minimum_duration) {
            return true;
        }
        if (has_minimum_duration && minimum_duration != rhs.minimum_duration) {
            return true;
        }
        if (!Utils.arraysEqual(current_position, rhs.current_position)
                || !Utils.arraysEqual(current_velocity, rhs.current_velocity)
                || !Utils.arraysEqual(current_acceleration, rhs.current_acceleration)
                || !Utils.arraysEqual(target_position, rhs.target_position)
                || !Utils.arraysEqual(target_velocity, rhs.target_velocity)
                || !Utils.arraysEqual(target_acceleration, rhs.target_acceleration)
                || !Utils.arraysEqual(max_velocity, rhs.max_velocity)
                || !Utils.arraysEqual(max_acceleration, rhs.max_acceleration)
                || !Utils.arraysEqual(max_jerk, rhs.max_jerk)
                || !Utils.boolArraysEqual(enabled, rhs.enabled)) {
            return true;
        }
        // Optional arrays
        if ((min_velocity == null) != (rhs.min_velocity == null)) {
            return true;
        }
        if (min_velocity != null && !Utils.arraysEqual(min_velocity, rhs.min_velocity)) {
            return true;
        }
        if ((min_acceleration == null) != (rhs.min_acceleration == null)) {
            return true;
        }
        if (min_acceleration != null && !Utils.arraysEqual(min_acceleration, rhs.min_acceleration)) {
            return true;
        }
        return false;
    }

    /** Copy all fields from other into this (for current_input stash). */
    public void copyFrom(InputParameter o) {
        control_interface = o.control_interface;
        synchronization = o.synchronization;
        duration_discretization = o.duration_discretization;
        System.arraycopy(o.current_position, 0, current_position, 0, degrees_of_freedom);
        System.arraycopy(o.current_velocity, 0, current_velocity, 0, degrees_of_freedom);
        System.arraycopy(o.current_acceleration, 0, current_acceleration, 0, degrees_of_freedom);
        System.arraycopy(o.target_position, 0, target_position, 0, degrees_of_freedom);
        System.arraycopy(o.target_velocity, 0, target_velocity, 0, degrees_of_freedom);
        System.arraycopy(o.target_acceleration, 0, target_acceleration, 0, degrees_of_freedom);
        System.arraycopy(o.max_velocity, 0, max_velocity, 0, degrees_of_freedom);
        System.arraycopy(o.max_acceleration, 0, max_acceleration, 0, degrees_of_freedom);
        System.arraycopy(o.max_jerk, 0, max_jerk, 0, degrees_of_freedom);
        System.arraycopy(o.enabled, 0, enabled, 0, degrees_of_freedom);
        if (o.min_velocity != null) {
            if (min_velocity == null) {
                min_velocity = new double[degrees_of_freedom];
            }
            System.arraycopy(o.min_velocity, 0, min_velocity, 0, degrees_of_freedom);
        } else {
            min_velocity = null;
        }
        if (o.min_acceleration != null) {
            if (min_acceleration == null) {
                min_acceleration = new double[degrees_of_freedom];
            }
            System.arraycopy(o.min_acceleration, 0, min_acceleration, 0, degrees_of_freedom);
        } else {
            min_acceleration = null;
        }
        has_minimum_duration = o.has_minimum_duration;
        minimum_duration = o.minimum_duration;
        if (o.per_dof_control_interface != null) {
            if (per_dof_control_interface == null) {
                per_dof_control_interface = new ControlInterface[degrees_of_freedom];
            }
            System.arraycopy(o.per_dof_control_interface, 0, per_dof_control_interface, 0, degrees_of_freedom);
        } else {
            per_dof_control_interface = null;
        }
        if (o.per_dof_synchronization != null) {
            if (per_dof_synchronization == null) {
                per_dof_synchronization = new Synchronization[degrees_of_freedom];
            }
            System.arraycopy(o.per_dof_synchronization, 0, per_dof_synchronization, 0, degrees_of_freedom);
        } else {
            per_dof_synchronization = null;
        }
    }

    @Override
    public String toString() {
        StringBuilder ss = new StringBuilder("\n");
        if (control_interface == ControlInterface.Velocity) {
            ss.append("inp.control_interface = ControlInterface.Velocity\n");
        }
        if (synchronization == Synchronization.Phase) {
            ss.append("inp.synchronization = Synchronization.Phase\n");
        } else if (synchronization == Synchronization.None) {
            ss.append("inp.synchronization = Synchronization.No\n");
        }
        if (duration_discretization == DurationDiscretization.Discrete) {
            ss.append("inp.duration_discretization = DurationDiscretization.Discrete\n");
        }
        ss.append("inp.current_position = [").append(Utils.join(current_position, true)).append("]\n");
        ss.append("inp.current_velocity = [").append(Utils.join(current_velocity, true)).append("]\n");
        ss.append("inp.current_acceleration = [").append(Utils.join(current_acceleration, true)).append("]\n");
        ss.append("inp.target_position = [").append(Utils.join(target_position, true)).append("]\n");
        ss.append("inp.target_velocity = [").append(Utils.join(target_velocity, true)).append("]\n");
        ss.append("inp.target_acceleration = [").append(Utils.join(target_acceleration, true)).append("]\n");
        ss.append("inp.max_velocity = [").append(Utils.join(max_velocity, true)).append("]\n");
        ss.append("inp.max_acceleration = [").append(Utils.join(max_acceleration, true)).append("]\n");
        ss.append("inp.max_jerk = [").append(Utils.join(max_jerk, true)).append("]\n");
        if (min_velocity != null) {
            ss.append("inp.min_velocity = [").append(Utils.join(min_velocity, true)).append("]\n");
        }
        if (min_acceleration != null) {
            ss.append("inp.min_acceleration = [").append(Utils.join(min_acceleration, true)).append("]\n");
        }
        if (has_minimum_duration) {
            ss.append("inp.minimum_duration = ").append(minimum_duration).append("\n");
        }
        return ss.toString();
    }
}
