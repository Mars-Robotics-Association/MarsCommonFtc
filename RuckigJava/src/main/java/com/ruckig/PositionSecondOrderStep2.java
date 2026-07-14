package com.ruckig;

/**
 * Port of {@code PositionSecondOrderStep2} from {@code position.hpp} /
 * {@code position_second_step2.cpp} (upstream v0.17.3).
 */
public final class PositionSecondOrderStep2 {

    private final double v0, tf, vf;
    private final double _vMax, _vMin, _aMax, _aMin;
    private final double pd, vd;

    public PositionSecondOrderStep2(double tf, double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin) {
        this.v0 = v0;
        this.tf = tf;
        this.vf = vf;
        this._vMax = vMax;
        this._vMin = vMin;
        this._aMax = aMax;
        this._aMin = aMin;
        this.pd = pf - p0;
        this.vd = vf - v0;
    }

    private boolean time_acc0(Profile profile, double vMax, double vMin, double aMax, double aMin) {
        // UD Solution 1/2
        {
            final double h1 = Math.sqrt((2 * aMax * (pd - tf * vf) - 2 * aMin * (pd - tf * v0) + vd * vd) / (aMax * aMin) + tf * tf);

            profile.t[0] = (aMax * vd - aMax * aMin * (tf - h1)) / (aMax * (aMax - aMin));
            profile.t[1] = h1;
            profile.t[2] = tf - (profile.t[0] + h1);
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check_for_second_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, tf, aMax, aMin, vMax, vMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        // UU Solution
        {
            final double h1 = (-vd + aMax * tf);

            profile.t[0] = -vd * vd / (2 * aMax * h1) + (pd - v0 * tf) / h1;
            profile.t[1] = -vd / aMax + tf;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = tf - (profile.t[0] + profile.t[1]);

            if (profile.check_for_second_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, tf, aMax, aMin, vMax, vMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        // UU Solution - 2 step
        {
            profile.t[0] = 0;
            profile.t[1] = -vd / aMax + tf;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = vd / aMax;

            if (profile.check_for_second_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, tf, aMax, aMin, vMax, vMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        return false;
    }

    private boolean time_none(Profile profile, double vMax, double vMin, double aMax, double aMin) {
        if (Math.abs(v0) < Utils.DBL_EPSILON && Math.abs(vf) < Utils.DBL_EPSILON && Math.abs(pd) < Utils.DBL_EPSILON) {
            profile.t[0] = 0;
            profile.t[1] = tf;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check_for_second_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, tf, aMax, aMin, vMax, vMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        // UD Solution 1/2
        {
            final double h1 = 2 * (vf * tf - pd);

            profile.t[0] = h1 / vd;
            profile.t[1] = tf - profile.t[0];
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            final double af = vd * vd / h1;

            if ((aMin - 1e-12 < af) && (af < aMax + 1e-12)
                    && profile.check_for_second_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, tf, af, -af, vMax, vMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        return false;
    }

    private boolean check_all(Profile profile, double vMax, double vMin, double aMax, double aMin) {
        return time_acc0(profile, vMax, vMin, aMax, aMin) || time_none(profile, vMax, vMin, aMax, aMin);
    }

    public boolean get_profile(Profile profile) {
        // Test all cases to get ones that match
        // However we should guess which one is correct and try them first...
        if (pd > 0) {
            return check_all(profile, _vMax, _vMin, _aMax, _aMin) || check_all(profile, _vMin, _vMax, _aMin, _aMax);
        }

        return check_all(profile, _vMin, _vMax, _aMin, _aMax) || check_all(profile, _vMax, _vMin, _aMax, _aMin);
    }
}
