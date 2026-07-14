package com.ruckig;

/**
 * Port of {@code Trajectory} from {@code trajectory.hpp} (upstream v0.17.3).
 * Community version: single section (no intermediate waypoints).
 * Dynamic DoFs only.
 */
public final class Trajectory {

    public final int degrees_of_freedom;

    /** profiles[0][dof] — single section for Community. */
    public final Profile[][] profiles;

    public double duration;

    /** cumulative_times[0] = duration for Community. */
    public final double[] cumulative_times;

    public final double[] independent_min_durations;
    public final Bound[] position_extrema;

    private final double[] integrateScratch = new double[3];

    public Trajectory(int dofs) {
        this.degrees_of_freedom = dofs;
        profiles = new Profile[1][dofs];
        for (int i = 0; i < dofs; ++i) {
            profiles[0][i] = new Profile();
        }
        cumulative_times = new double[1];
        independent_min_durations = new double[dofs];
        position_extrema = new Bound[dofs];
        for (int i = 0; i < dofs; ++i) {
            position_extrema[i] = new Bound();
        }
        duration = 0.0;
    }

    /**
     * Get kinematic state at a given time.
     * Arrays must have length == degrees_of_freedom.
     */
    public void at_time(double time, double[] new_position, double[] new_velocity, double[] new_acceleration) {
        if (degrees_of_freedom != new_position.length
                || degrees_of_freedom != new_velocity.length
                || degrees_of_freedom != new_acceleration.length) {
            throw new RuckigError("mismatch in degrees of freedom (vector size).");
        }
        int[] section = new int[1];
        state_to_integrate_from(time, section, new_position, new_velocity, new_acceleration, null);
    }

    public void at_time(double time, double[] new_position, double[] new_velocity, double[] new_acceleration, double[] new_jerk, int[] new_section) {
        if (degrees_of_freedom != new_position.length
                || degrees_of_freedom != new_velocity.length
                || degrees_of_freedom != new_acceleration.length
                || degrees_of_freedom != new_jerk.length) {
            throw new RuckigError("mismatch in degrees of freedom (vector size).");
        }
        state_to_integrate_from(time, new_section, new_position, new_velocity, new_acceleration, new_jerk);
    }

    /** Single-DoF convenience. */
    public void at_time(double time, double[] pvaOut) {
        double[] p = new double[1];
        double[] v = new double[1];
        double[] a = new double[1];
        at_time(time, p, v, a);
        pvaOut[0] = p[0];
        pvaOut[1] = v[0];
        pvaOut[2] = a[0];
    }

    private void state_to_integrate_from(double time, int[] new_section, double[] new_position, double[] new_velocity, double[] new_acceleration, double[] new_jerk) {
        if (time >= duration) {
            // Keep constant acceleration
            new_section[0] = profiles.length;
            Profile[] profiles_dof = profiles[profiles.length - 1];
            for (int dof = 0; dof < degrees_of_freedom; ++dof) {
                final double t_pre = (profiles.length > 1) ? cumulative_times[cumulative_times.length - 2] : profiles_dof[dof].brake.duration;
                final double t_diff = time - (t_pre + profiles_dof[dof].t_sum[6]);
                Utils.integrate(t_diff, profiles_dof[dof].p[7], profiles_dof[dof].v[7], profiles_dof[dof].a[7], 0.0, integrateScratch);
                new_position[dof] = integrateScratch[0];
                new_velocity[dof] = integrateScratch[1];
                new_acceleration[dof] = integrateScratch[2];
                if (new_jerk != null) {
                    new_jerk[dof] = 0.0;
                }
            }
            return;
        }

        // upper_bound on cumulative_times
        int section = 0;
        while (section < cumulative_times.length && cumulative_times[section] <= time) {
            section++;
        }
        // upper_bound returns first element > time; for time < cumulative_times[0], section=0
        // std::upper_bound: first element greater than time
        section = 0;
        for (int i = 0; i < cumulative_times.length; ++i) {
            if (cumulative_times[i] > time) {
                section = i;
                break;
            }
            section = i + 1;
        }
        new_section[0] = section;

        double t_diff = time;
        if (section > 0) {
            t_diff -= cumulative_times[section - 1];
        }

        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            final Profile p = profiles[section][dof];
            double t_diff_dof = t_diff;

            // Brake pre-trajectory
            if (section == 0 && p.brake.duration > 0) {
                if (t_diff_dof < p.brake.duration) {
                    final int index = (t_diff_dof < p.brake.t[0]) ? 0 : 1;
                    if (index > 0) {
                        t_diff_dof -= p.brake.t[index - 1];
                    }

                    Utils.integrate(t_diff_dof, p.brake.p[index], p.brake.v[index], p.brake.a[index], p.brake.j[index], integrateScratch);
                    new_position[dof] = integrateScratch[0];
                    new_velocity[dof] = integrateScratch[1];
                    new_acceleration[dof] = integrateScratch[2];
                    if (new_jerk != null) {
                        new_jerk[dof] = p.brake.j[index];
                    }
                    continue;
                } else {
                    t_diff_dof -= p.brake.duration;
                }
            }

            // Non-time synchronization
            if (t_diff_dof >= p.t_sum[6]) {
                Utils.integrate(t_diff_dof - p.t_sum[6], p.p[7], p.v[7], p.a[7], 0.0, integrateScratch);
                new_position[dof] = integrateScratch[0];
                new_velocity[dof] = integrateScratch[1];
                new_acceleration[dof] = integrateScratch[2];
                if (new_jerk != null) {
                    new_jerk[dof] = 0.0;
                }
                continue;
            }

            // upper_bound on p.t_sum
            int index_dof = 0;
            for (int i = 0; i < 7; ++i) {
                if (p.t_sum[i] > t_diff_dof) {
                    index_dof = i;
                    break;
                }
                index_dof = i + 1;
            }

            if (index_dof > 0) {
                t_diff_dof -= p.t_sum[index_dof - 1];
            }

            Utils.integrate(t_diff_dof, p.p[index_dof], p.v[index_dof], p.a[index_dof], p.j[index_dof], integrateScratch);
            new_position[dof] = integrateScratch[0];
            new_velocity[dof] = integrateScratch[1];
            new_acceleration[dof] = integrateScratch[2];
            if (new_jerk != null) {
                new_jerk[dof] = p.j[index_dof];
            }
        }
    }

    public double get_duration() {
        return duration;
    }

    public double[] get_independent_min_durations() {
        return independent_min_durations;
    }

    public Bound[] get_position_extrema() {
        for (int dof = 0; dof < degrees_of_freedom; ++dof) {
            Bound e = profiles[0][dof].get_position_extrema();
            position_extrema[dof].copyFrom(e);
        }
        return position_extrema;
    }
}
