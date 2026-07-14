package com.ruckig;

/**
 * Port of {@code Profile} from {@code profile.hpp} (upstream v0.17.3).
 * Template check&lt;ControlSigns, ReachedLimits&gt; becomes enum parameters.
 */
public final class Profile {

    // --- Epsilon constants (verbatim from upstream; names for audit) ---
    /** Upstream: {@code v_eps = 1e-12} */
    public static final double V_EPS = 1e-12;
    /** Upstream: {@code a_eps = 1e-12} */
    public static final double A_EPS = 1e-12;
    /** Upstream: {@code j_eps = 1e-12} */
    public static final double J_EPS = 1e-12;
    /** Upstream: {@code p_precision = 1e-8} */
    public static final double P_PRECISION = 1e-8;
    /** Upstream: {@code v_precision = 1e-8} */
    public static final double V_PRECISION = 1e-8;
    /** Upstream: {@code a_precision = 1e-10} */
    public static final double A_PRECISION = 1e-10;
    /** Upstream: {@code t_precision = 1e-12} */
    public static final double T_PRECISION = 1e-12;
    /** Upstream: {@code t_max = 1e12} */
    public static final double T_MAX = 1e12;

    public enum ReachedLimits {
        ACC0_ACC1_VEL, VEL, ACC0, ACC1, ACC0_ACC1, ACC0_VEL, ACC1_VEL, NONE
    }

    public enum Direction {
        UP, DOWN
    }

    public enum ControlSigns {
        UDDU, UDUD
    }

    public final double[] t = new double[7];
    public final double[] t_sum = new double[7];
    public final double[] j = new double[7];
    public final double[] a = new double[8];
    public final double[] v = new double[8];
    public final double[] p = new double[8];

    public final BrakeProfile brake = new BrakeProfile();
    public final BrakeProfile accel = new BrakeProfile();

    public double pf, vf, af;

    public ReachedLimits limits = ReachedLimits.NONE;
    public Direction direction = Direction.UP;
    public ControlSigns control_signs = ControlSigns.UDDU;

    // Scratch for extrema / root solve in get_first_state_at_position
    private final Roots.PositiveDoubleSet cubicRoots = new Roots.PositiveDoubleSet(3);
    private final double[] stateScratch = new double[3];

    public Profile() {}

    public void copyFrom(Profile o) {
        System.arraycopy(o.t, 0, t, 0, 7);
        System.arraycopy(o.t_sum, 0, t_sum, 0, 7);
        System.arraycopy(o.j, 0, j, 0, 7);
        System.arraycopy(o.a, 0, a, 0, 8);
        System.arraycopy(o.v, 0, v, 0, 8);
        System.arraycopy(o.p, 0, p, 0, 8);
        brake.copyFrom(o.brake);
        accel.copyFrom(o.accel);
        pf = o.pf;
        vf = o.vf;
        af = o.af;
        limits = o.limits;
        direction = o.direction;
        control_signs = o.control_signs;
    }

    // For third-order velocity interface
    public boolean check_for_velocity(ControlSigns control_signs, ReachedLimits limits, double jf, double aMax, double aMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (int i = 0; i < 6; ++i) {
            if (t[i + 1] < 0) {
                return false;
            }

            t_sum[i + 1] = t_sum[i] + t[i + 1];
        }

        if (limits == ReachedLimits.ACC0) {
            if (t[1] < Utils.DBL_EPSILON) {
                return false;
            }
        }

        if (t_sum[6] > T_MAX) {
            return false;
        }

        if (control_signs == ControlSigns.UDDU) {
            j[0] = (t[0] > 0 ? jf : 0);
            j[1] = 0;
            j[2] = (t[2] > 0 ? -jf : 0);
            j[3] = 0;
            j[4] = (t[4] > 0 ? -jf : 0);
            j[5] = 0;
            j[6] = (t[6] > 0 ? jf : 0);
        } else {
            j[0] = (t[0] > 0 ? jf : 0);
            j[1] = 0;
            j[2] = (t[2] > 0 ? -jf : 0);
            j[3] = 0;
            j[4] = (t[4] > 0 ? jf : 0);
            j[5] = 0;
            j[6] = (t[6] > 0 ? -jf : 0);
        }

        for (int i = 0; i < 7; ++i) {
            a[i + 1] = a[i] + t[i] * j[i];
            v[i + 1] = v[i] + t[i] * (a[i] + t[i] * j[i] / 2);
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * (a[i] / 2 + t[i] * j[i] / 6));
        }

        this.control_signs = control_signs;
        this.limits = limits;

        direction = (aMax > 0) ? Direction.UP : Direction.DOWN;
        final double aUppLim = (direction == Direction.UP ? aMax : aMin) + A_EPS;
        final double aLowLim = (direction == Direction.UP ? aMin : aMax) - A_EPS;

        return Math.abs(v[7] - vf) < V_PRECISION && Math.abs(a[7] - af) < A_PRECISION
            && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
            && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim;
    }

    public boolean check_for_velocity_with_timing(ControlSigns control_signs, ReachedLimits limits, double tfIgnored, double jf, double aMax, double aMin) {
        return check_for_velocity(control_signs, limits, jf, aMax, aMin);
    }

    public boolean check_for_velocity_with_timing(ControlSigns control_signs, ReachedLimits limits, double tf, double jf, double aMax, double aMin, double jMax) {
        return (Math.abs(jf) < Math.abs(jMax) + J_EPS) && check_for_velocity_with_timing(control_signs, limits, tf, jf, aMax, aMin);
    }

    public void set_boundary_for_velocity(double p0_new, double v0_new, double a0_new, double vf_new, double af_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        af = af_new;
        vf = vf_new;
    }

    // For second-order velocity interface
    public boolean check_for_second_order_velocity(ControlSigns control_signs, ReachedLimits limits, double aUp) {
        // ReachedLimits::ACC0
        if (t[1] < 0.0) {
            return false;
        }

        t_sum[0] = 0;
        t_sum[1] = t[1];
        t_sum[2] = t[1];
        t_sum[3] = t[1];
        t_sum[4] = t[1];
        t_sum[5] = t[1];
        t_sum[6] = t[1];
        if (t_sum[6] > T_MAX) {
            return false;
        }

        for (int i = 0; i < 7; ++i) {
            j[i] = 0;
        }
        a[0] = 0;
        a[1] = (t[1] > 0) ? aUp : 0;
        a[2] = 0;
        a[3] = 0;
        a[4] = 0;
        a[5] = 0;
        a[6] = 0;
        a[7] = af;
        for (int i = 0; i < 7; ++i) {
            v[i + 1] = v[i] + t[i] * a[i];
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this.control_signs = control_signs;
        this.limits = limits;

        direction = (aUp > 0) ? Direction.UP : Direction.DOWN;

        return Math.abs(v[7] - vf) < V_PRECISION;
    }

    public boolean check_for_second_order_velocity_with_timing(ControlSigns control_signs, ReachedLimits limits, double tfIgnored, double aUp) {
        return check_for_second_order_velocity(control_signs, limits, aUp);
    }

    public boolean check_for_second_order_velocity_with_timing(ControlSigns control_signs, ReachedLimits limits, double tf, double aUp, double aMax, double aMin) {
        return (aMin - A_EPS < aUp) && (aUp < aMax + A_EPS) && check_for_second_order_velocity_with_timing(control_signs, limits, tf, aUp);
    }

    // For third-order position interface
    public boolean check(ControlSigns control_signs, ReachedLimits limits, double jf, double vMax, double vMin, double aMax, double aMin) {
        return check(control_signs, limits, false, jf, vMax, vMin, aMax, aMin);
    }

    public boolean check(ControlSigns control_signs, ReachedLimits limits, boolean set_limits, double jf, double vMax, double vMin, double aMax, double aMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (int i = 0; i < 6; ++i) {
            if (t[i + 1] < 0) {
                return false;
            }

            t_sum[i + 1] = t_sum[i] + t[i + 1];
        }

        if (limits == ReachedLimits.ACC0_ACC1_VEL || limits == ReachedLimits.ACC0_VEL || limits == ReachedLimits.ACC1_VEL || limits == ReachedLimits.VEL) {
            if (t[3] < Utils.DBL_EPSILON) {
                return false;
            }
        }

        if (limits == ReachedLimits.ACC0 || limits == ReachedLimits.ACC0_ACC1) {
            if (t[1] < Utils.DBL_EPSILON) {
                return false;
            }
        }

        if (limits == ReachedLimits.ACC1 || limits == ReachedLimits.ACC0_ACC1) {
            if (t[5] < Utils.DBL_EPSILON) {
                return false;
            }
        }

        if (t_sum[6] > T_MAX) {
            return false;
        }

        if (control_signs == ControlSigns.UDDU) {
            j[0] = (t[0] > 0 ? jf : 0);
            j[1] = 0;
            j[2] = (t[2] > 0 ? -jf : 0);
            j[3] = 0;
            j[4] = (t[4] > 0 ? -jf : 0);
            j[5] = 0;
            j[6] = (t[6] > 0 ? jf : 0);
        } else {
            j[0] = (t[0] > 0 ? jf : 0);
            j[1] = 0;
            j[2] = (t[2] > 0 ? -jf : 0);
            j[3] = 0;
            j[4] = (t[4] > 0 ? jf : 0);
            j[5] = 0;
            j[6] = (t[6] > 0 ? -jf : 0);
        }

        direction = (vMax > 0) ? Direction.UP : Direction.DOWN;
        final double vUppLim = (direction == Direction.UP ? vMax : vMin) + V_EPS;
        final double vLowLim = (direction == Direction.UP ? vMin : vMax) - V_EPS;

        for (int i = 0; i < 7; ++i) {
            a[i + 1] = a[i] + t[i] * j[i];
            v[i + 1] = v[i] + t[i] * (a[i] + t[i] * j[i] / 2);
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * (a[i] / 2 + t[i] * j[i] / 6));

            if (limits == ReachedLimits.ACC0_ACC1_VEL || limits == ReachedLimits.ACC0_ACC1 || limits == ReachedLimits.ACC0_VEL || limits == ReachedLimits.ACC1_VEL || limits == ReachedLimits.VEL) {
                if (i == 2) {
                    a[3] = 0.0;
                }
            }

            if (set_limits) {
                if (limits == ReachedLimits.ACC1) {
                    if (i == 2) {
                        a[3] = aMin;
                    }
                }

                if (limits == ReachedLimits.ACC0_ACC1) {
                    if (i == 0) {
                        a[1] = aMax;
                    }

                    if (i == 4) {
                        a[5] = aMin;
                    }
                }
            }

            if (i > 1 && a[i + 1] * a[i] < -Utils.DBL_EPSILON) {
                final double v_a_zero = v[i] - (a[i] * a[i]) / (2 * j[i]);
                if (v_a_zero > vUppLim || v_a_zero < vLowLim) {
                    return false;
                }
            }
        }

        this.control_signs = control_signs;
        this.limits = limits;

        final double aUppLim = (direction == Direction.UP ? aMax : aMin) + A_EPS;
        final double aLowLim = (direction == Direction.UP ? aMin : aMax) - A_EPS;

        return Math.abs(p[7] - pf) < P_PRECISION && Math.abs(v[7] - vf) < V_PRECISION && Math.abs(a[7] - af) < A_PRECISION
            && a[1] >= aLowLim && a[3] >= aLowLim && a[5] >= aLowLim
            && a[1] <= aUppLim && a[3] <= aUppLim && a[5] <= aUppLim
            && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
            && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    public boolean check_with_timing(ControlSigns control_signs, ReachedLimits limits, double tfIgnored, double jf, double vMax, double vMin, double aMax, double aMin) {
        return check(control_signs, limits, jf, vMax, vMin, aMax, aMin);
    }

    public boolean check_with_timing(ControlSigns control_signs, ReachedLimits limits, double tf, double jf, double vMax, double vMin, double aMax, double aMin, double jMax) {
        return (Math.abs(jf) < Math.abs(jMax) + J_EPS) && check_with_timing(control_signs, limits, tf, jf, vMax, vMin, aMax, aMin);
    }

    public void set_boundary(Profile profile) {
        a[0] = profile.a[0];
        v[0] = profile.v[0];
        p[0] = profile.p[0];
        af = profile.af;
        vf = profile.vf;
        pf = profile.pf;
        brake.copyFrom(profile.brake);
        accel.copyFrom(profile.accel);
    }

    public void set_boundary(double p0_new, double v0_new, double a0_new, double pf_new, double vf_new, double af_new) {
        a[0] = a0_new;
        v[0] = v0_new;
        p[0] = p0_new;
        af = af_new;
        vf = vf_new;
        pf = pf_new;
    }

    // For second-order position interface
    public boolean check_for_second_order(ControlSigns control_signs, ReachedLimits limits, double aUp, double aDown, double vMax, double vMin) {
        if (t[0] < 0) {
            return false;
        }

        t_sum[0] = t[0];
        for (int i = 0; i < 6; ++i) {
            if (t[i + 1] < 0) {
                return false;
            }

            t_sum[i + 1] = t_sum[i] + t[i + 1];
        }

        if (t_sum[6] > T_MAX) {
            return false;
        }

        for (int i = 0; i < 7; ++i) {
            j[i] = 0;
        }
        if (control_signs == ControlSigns.UDDU) {
            a[0] = (t[0] > 0 ? aUp : 0);
            a[1] = 0;
            a[2] = (t[2] > 0 ? aDown : 0);
            a[3] = 0;
            a[4] = (t[4] > 0 ? aDown : 0);
            a[5] = 0;
            a[6] = (t[6] > 0 ? aUp : 0);
            a[7] = af;
        } else {
            a[0] = (t[0] > 0 ? aUp : 0);
            a[1] = 0;
            a[2] = (t[2] > 0 ? aDown : 0);
            a[3] = 0;
            a[4] = (t[4] > 0 ? aUp : 0);
            a[5] = 0;
            a[6] = (t[6] > 0 ? aDown : 0);
            a[7] = af;
        }

        direction = (vMax > 0) ? Direction.UP : Direction.DOWN;
        final double vUppLim = (direction == Direction.UP ? vMax : vMin) + V_EPS;
        final double vLowLim = (direction == Direction.UP ? vMin : vMax) - V_EPS;

        for (int i = 0; i < 7; ++i) {
            v[i + 1] = v[i] + t[i] * a[i];
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this.control_signs = control_signs;
        this.limits = limits;

        return Math.abs(p[7] - pf) < P_PRECISION && Math.abs(v[7] - vf) < V_PRECISION
            && v[2] <= vUppLim && v[3] <= vUppLim && v[4] <= vUppLim && v[5] <= vUppLim && v[6] <= vUppLim
            && v[2] >= vLowLim && v[3] >= vLowLim && v[4] >= vLowLim && v[5] >= vLowLim && v[6] >= vLowLim;
    }

    public boolean check_for_second_order_with_timing(ControlSigns control_signs, ReachedLimits limits, double tfIgnored, double aUp, double aDown, double vMax, double vMin) {
        return check_for_second_order(control_signs, limits, aUp, aDown, vMax, vMin);
    }

    public boolean check_for_second_order_with_timing(ControlSigns control_signs, ReachedLimits limits, double tf, double aUp, double aDown, double vMax, double vMin, double aMax, double aMin) {
        return (aMin - A_EPS < aUp) && (aUp < aMax + A_EPS) && (aMin - A_EPS < aDown) && (aDown < aMax + A_EPS)
            && check_for_second_order_with_timing(control_signs, limits, tf, aUp, aDown, vMax, vMin);
    }

    // For first-order position interface
    public boolean check_for_first_order(ControlSigns control_signs, ReachedLimits limits, double vUp) {
        // ReachedLimits::VEL
        if (t[3] < 0.0) {
            return false;
        }

        t_sum[0] = 0;
        t_sum[1] = 0;
        t_sum[2] = 0;
        t_sum[3] = t[3];
        t_sum[4] = t[3];
        t_sum[5] = t[3];
        t_sum[6] = t[3];
        if (t_sum[6] > T_MAX) {
            return false;
        }

        for (int i = 0; i < 7; ++i) {
            j[i] = 0;
        }
        for (int i = 0; i < 7; ++i) {
            a[i] = 0;
        }
        a[7] = af;
        v[0] = 0;
        v[1] = 0;
        v[2] = 0;
        v[3] = t[3] > 0 ? vUp : 0;
        v[4] = 0;
        v[5] = 0;
        v[6] = 0;
        v[7] = vf;
        for (int i = 0; i < 7; ++i) {
            p[i + 1] = p[i] + t[i] * (v[i] + t[i] * a[i] / 2);
        }

        this.control_signs = control_signs;
        this.limits = limits;

        direction = (vUp > 0) ? Direction.UP : Direction.DOWN;

        return Math.abs(p[7] - pf) < P_PRECISION;
    }

    public boolean check_for_first_order_with_timing(ControlSigns control_signs, ReachedLimits limits, double tfIgnored, double vUp) {
        return check_for_first_order(control_signs, limits, vUp);
    }

    public boolean check_for_first_order_with_timing(ControlSigns control_signs, ReachedLimits limits, double tf, double vUp, double vMax, double vMin) {
        return (vMin - V_EPS < vUp) && (vUp < vMax + V_EPS) && check_for_first_order_with_timing(control_signs, limits, tf, vUp);
    }

    // Secondary features
    private static void check_position_extremum(double t_ext, double t_sum, double t, double p, double v, double a, double j, Bound ext) {
        if (0 < t_ext && t_ext < t) {
            double p_ext = Utils.integrateP(t_ext, p, v, a, j);
            double a_ext = Utils.integrateA(t_ext, a, j);
            if (a_ext > 0 && p_ext < ext.min) {
                ext.min = p_ext;
                ext.t_min = t_sum + t_ext;
            } else if (a_ext < 0 && p_ext > ext.max) {
                ext.max = p_ext;
                ext.t_max = t_sum + t_ext;
            }
        }
    }

    private static void check_step_for_position_extremum(double t_sum, double t, double p, double v, double a, double j, Bound ext) {
        if (p < ext.min) {
            ext.min = p;
            ext.t_min = t_sum;
        }
        if (p > ext.max) {
            ext.max = p;
            ext.t_max = t_sum;
        }

        if (j != 0) {
            final double D = a * a - 2 * j * v;
            if (Math.abs(D) < Utils.DBL_EPSILON) {
                check_position_extremum(-a / j, t_sum, t, p, v, a, j, ext);

            } else if (D > 0.0) {
                final double D_sqrt = Math.sqrt(D);
                check_position_extremum((-a - D_sqrt) / j, t_sum, t, p, v, a, j, ext);
                check_position_extremum((-a + D_sqrt) / j, t_sum, t, p, v, a, j, ext);
            }
        }
    }

    public Bound get_position_extrema() {
        Bound extrema = new Bound();
        extrema.min = Double.POSITIVE_INFINITY;
        extrema.max = Double.NEGATIVE_INFINITY;

        if (brake.duration > 0.0) {
            if (brake.t[0] > 0.0) {
                check_step_for_position_extremum(0.0, brake.t[0], brake.p[0], brake.v[0], brake.a[0], brake.j[0], extrema);

                if (brake.t[1] > 0.0) {
                    check_step_for_position_extremum(brake.t[0], brake.t[1], brake.p[1], brake.v[1], brake.a[1], brake.j[1], extrema);
                }
            }
        }

        double t_current_sum = 0.0;
        for (int i = 0; i < 7; ++i) {
            if (i > 0) {
                t_current_sum = t_sum[i - 1];
            }
            check_step_for_position_extremum(t_current_sum + brake.duration, t[i], p[i], v[i], a[i], j[i], extrema);
        }

        if (pf < extrema.min) {
            extrema.min = pf;
            extrema.t_min = t_sum[6] + brake.duration;
        }
        if (pf > extrema.max) {
            extrema.max = pf;
            extrema.t_max = t_sum[6] + brake.duration;
        }

        return extrema;
    }

    /**
     * @param timeOut length-1 array to receive the time if found
     * @return true if a crossing was found
     */
    public boolean get_first_state_at_position(double pt, double[] timeOut, double time_after) {
        double t_cum = 0.0;

        for (int i = 0; i < 7; ++i) {
            if (t[i] == 0.0) {
                continue;
            }

            if (Math.abs(p[i] - pt) < Utils.DBL_EPSILON && t_cum >= time_after) {
                timeOut[0] = t_cum;
                return true;
            }

            Roots.solveCubic(j[i] / 6, a[i] / 2, v[i], p[i] - pt, cubicRoots);
            int n = cubicRoots.sortedSize();
            for (int k = 0; k < n; ++k) {
                double _t = cubicRoots.get(k);
                if (0 < _t && time_after - t_cum <= _t && _t <= t[i]) {
                    timeOut[0] = _t + t_cum;
                    return true;
                }
            }

            t_cum += t[i];
        }

        if ((t[6] > 0.0 || t_sum[6] == 0.0) && Math.abs(pf - pt) < 1e-9 && t_sum[6] >= time_after) {
            timeOut[0] = t_sum[6];
            return true;
        }

        return false;
    }

    public boolean get_first_state_at_position(double pt, double[] timeOut) {
        return get_first_state_at_position(pt, timeOut, 0.0);
    }

    public String toString() {
        String result = "";
        switch (direction) {
            case UP:
                result += "UP_";
                break;
            case DOWN:
                result += "DOWN_";
                break;
        }
        switch (limits) {
            case ACC0_ACC1_VEL:
                result += "ACC0_ACC1_VEL";
                break;
            case VEL:
                result += "VEL";
                break;
            case ACC0:
                result += "ACC0";
                break;
            case ACC1:
                result += "ACC1";
                break;
            case ACC0_ACC1:
                result += "ACC0_ACC1";
                break;
            case ACC0_VEL:
                result += "ACC0_VEL";
                break;
            case ACC1_VEL:
                result += "ACC1_VEL";
                break;
            case NONE:
                result += "NONE";
                break;
        }
        switch (control_signs) {
            case UDDU:
                result += "_UDDU";
                break;
            case UDUD:
                result += "_UDUD";
                break;
        }
        return result;
    }
}
