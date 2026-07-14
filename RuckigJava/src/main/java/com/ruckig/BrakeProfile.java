package com.ruckig;

/**
 * Port of {@code BrakeProfile} from {@code brake.hpp} / {@code brake.cpp}
 * (upstream v0.17.3).
 */
public final class BrakeProfile {

    /** Upstream: {@code static constexpr double eps {2.2e-14};} */
    private static final double EPS = 2.2e-14;

    /** Overall duration. */
    public double duration;

    /** Profile information for a two-step profile. */
    public final double[] t = new double[2];
    public final double[] j = new double[2];
    public final double[] a = new double[2];
    public final double[] v = new double[2];
    public final double[] p = new double[2];

    private final double[] integrateScratch = new double[3];

    public BrakeProfile() {
        duration = 0.0;
    }

    public void copyFrom(BrakeProfile o) {
        duration = o.duration;
        System.arraycopy(o.t, 0, t, 0, 2);
        System.arraycopy(o.j, 0, j, 0, 2);
        System.arraycopy(o.a, 0, a, 0, 2);
        System.arraycopy(o.v, 0, v, 0, 2);
        System.arraycopy(o.p, 0, p, 0, 2);
    }

    private static double v_at_t(double v0, double a0, double j, double t) {
        return v0 + t * (a0 + j * t / 2);
    }

    private static double v_at_a_zero(double v0, double a0, double j) {
        return v0 + (a0 * a0) / (2 * j);
    }

    private void acceleration_brake(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax) {
        j[0] = -jMax;

        final double t_to_a_max = (a0 - aMax) / jMax;
        final double t_to_a_zero = a0 / jMax;

        final double v_at_a_max = v_at_t(v0, a0, -jMax, t_to_a_max);
        final double v_at_a_zero = v_at_t(v0, a0, -jMax, t_to_a_zero);

        if ((v_at_a_zero > vMax && jMax > 0) || (v_at_a_zero < vMax && jMax < 0)) {
            velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

        } else if ((v_at_a_max < vMin && jMax > 0) || (v_at_a_max > vMin && jMax < 0)) {
            final double t_to_v_min = -(v_at_a_max - vMin) / aMax;
            final double t_to_v_max = -aMax / (2 * jMax) - (v_at_a_max - vMax) / aMax;

            t[0] = t_to_a_max + EPS;
            t[1] = Utils.cppMax(Utils.cppMin(t_to_v_min, t_to_v_max - EPS), 0.0);

        } else {
            t[0] = t_to_a_max + EPS;
        }
    }

    private void velocity_brake(double v0, double a0, double vMax, double vMin, double aMaxIgnored, double aMin, double jMax) {
        j[0] = -jMax;
        final double t_to_a_min = (a0 - aMin) / jMax;
        final double t_to_v_max = a0 / jMax + Math.sqrt(a0 * a0 + 2 * jMax * (v0 - vMax)) / Math.abs(jMax);
        final double t_to_v_min = a0 / jMax + Math.sqrt(a0 * a0 / 2 + jMax * (v0 - vMin)) / Math.abs(jMax);
        final double t_min_to_v_max = Utils.cppMin(t_to_v_max, t_to_v_min);

        if (t_to_a_min < t_min_to_v_max) {
            final double v_at_a_min = v_at_t(v0, a0, -jMax, t_to_a_min);
            final double t_to_v_max_with_constant = -(v_at_a_min - vMax) / aMin;
            final double t_to_v_min_with_constant = aMin / (2 * jMax) - (v_at_a_min - vMin) / aMin;

            t[0] = Utils.cppMax(t_to_a_min - EPS, 0.0);
            t[1] = Utils.cppMax(Utils.cppMin(t_to_v_max_with_constant, t_to_v_min_with_constant), 0.0);

        } else {
            t[0] = Utils.cppMax(t_min_to_v_max - EPS, 0.0);
        }
    }

    /** Calculate brake trajectory for third-order position interface. */
    public void get_position_brake_trajectory(double v0, double a0, double vMax, double vMin, double aMax, double aMin, double jMax) {
        t[0] = 0.0;
        t[1] = 0.0;
        j[0] = 0.0;
        j[1] = 0.0;

        if (jMax == 0.0 || aMax == 0.0 || aMin == 0.0) {
            return; // Ignore braking for zero-limits
        }

        if (a0 > aMax) {
            acceleration_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

        } else if (a0 < aMin) {
            acceleration_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);

        } else if ((v0 > vMax && v_at_a_zero(v0, a0, -jMax) > vMin) || (a0 > 0 && v_at_a_zero(v0, a0, jMax) > vMax)) {
            velocity_brake(v0, a0, vMax, vMin, aMax, aMin, jMax);

        } else if ((v0 < vMin && v_at_a_zero(v0, a0, jMax) < vMax) || (a0 < 0 && v_at_a_zero(v0, a0, -jMax) < vMin)) {
            velocity_brake(v0, a0, vMin, vMax, aMin, aMax, -jMax);
        }
    }

    /** Calculate brake trajectory for second-order position interface. */
    public void get_second_order_position_brake_trajectory(double v0, double vMax, double vMin, double aMax, double aMin) {
        t[0] = 0.0;
        t[1] = 0.0;
        j[0] = 0.0;
        j[1] = 0.0;
        a[0] = 0.0;
        a[1] = 0.0;

        if (aMax == 0.0 || aMin == 0.0) {
            return; // Ignore braking for zero-limits
        }

        if (v0 > vMax) {
            a[0] = aMin;
            t[0] = (vMax - v0) / aMin + EPS;

        } else if (v0 < vMin) {
            a[0] = aMax;
            t[0] = (vMin - v0) / aMax + EPS;
        }
    }

    /** Calculate brake trajectory for third-order velocity interface. */
    public void get_velocity_brake_trajectory(double a0, double aMax, double aMin, double jMax) {
        t[0] = 0.0;
        t[1] = 0.0;
        j[0] = 0.0;
        j[1] = 0.0;

        if (jMax == 0.0) {
            return; // Ignore braking for zero-limits
        }

        if (a0 > aMax) {
            j[0] = -jMax;
            t[0] = (a0 - aMax) / jMax + EPS;

        } else if (a0 < aMin) {
            j[0] = jMax;
            t[0] = -(a0 - aMin) / jMax + EPS;
        }
    }

    /** Calculate brake trajectory for second-order velocity interface. */
    public void get_second_order_velocity_brake_trajectory() {
        t[0] = 0.0;
        t[1] = 0.0;
        j[0] = 0.0;
        j[1] = 0.0;
    }

    /**
     * Finalize third-order braking by integrating along kinematic state.
     * Updates ps, vs, as in place via the length-3 state array [p, v, a].
     */
    public void finalize(double[] state) {
        double ps = state[0];
        double vs = state[1];
        double as = state[2];

        if (t[0] <= 0.0 && t[1] <= 0.0) {
            duration = 0.0;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        a[0] = as;
        Utils.integrate(t[0], ps, vs, as, j[0], integrateScratch);
        ps = integrateScratch[0];
        vs = integrateScratch[1];
        as = integrateScratch[2];

        if (t[1] > 0.0) {
            duration += t[1];
            p[1] = ps;
            v[1] = vs;
            a[1] = as;
            Utils.integrate(t[1], ps, vs, as, j[1], integrateScratch);
            ps = integrateScratch[0];
            vs = integrateScratch[1];
            as = integrateScratch[2];
        }

        state[0] = ps;
        state[1] = vs;
        state[2] = as;
    }

    /** Finalize second-order braking. state = [p, v, a]. */
    public void finalize_second_order(double[] state) {
        double ps = state[0];
        double vs = state[1];
        double as = state[2];

        if (t[0] <= 0.0) {
            duration = 0.0;
            return;
        }

        duration = t[0];
        p[0] = ps;
        v[0] = vs;
        Utils.integrate(t[0], ps, vs, a[0], 0.0, integrateScratch);
        ps = integrateScratch[0];
        vs = integrateScratch[1];
        as = integrateScratch[2];

        state[0] = ps;
        state[1] = vs;
        state[2] = as;
    }
}
