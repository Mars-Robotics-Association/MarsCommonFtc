package com.ruckig;

/**
 * Port of {@code VelocityThirdOrderStep2} from {@code velocity.hpp} /
 * {@code velocity_third_step2.cpp} (upstream v0.17.3).
 */
public final class VelocityThirdOrderStep2 {

    private final double a0, tf, af;
    private final double _aMax, _aMin, _jMax;
    private final double vd, ad;

    public VelocityThirdOrderStep2(double tf, double v0, double a0, double vf, double af, double aMax, double aMin, double jMax) {
        this.a0 = a0;
        this.tf = tf;
        this.af = af;
        this._aMax = aMax;
        this._aMin = aMin;
        this._jMax = jMax;
        this.vd = vf - v0;
        this.ad = af - a0;
    }

    private boolean time_acc0(Profile profile, double aMax, double aMin, double jMax) {
        // UD Solution 1/2
        {
            final double h1 = Math.sqrt((-ad * ad + 2 * jMax * ((a0 + af) * tf - 2 * vd)) / (jMax * jMax) + tf * tf);

            profile.t[0] = ad / (2 * jMax) + (tf - h1) / 2;
            profile.t[1] = h1;
            profile.t[2] = tf - (profile.t[0] + h1);
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check_for_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, tf, jMax, aMax, aMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        // UU Solution
        {
            final double h1 = (-ad + jMax * tf);

            profile.t[0] = -ad * ad / (2 * jMax * h1) + (vd - a0 * tf) / h1;
            profile.t[1] = -ad / jMax + tf;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = tf - (profile.t[0] + profile.t[1]);

            if (profile.check_for_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, tf, jMax, aMax, aMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        // UU Solution - 2 step
        {
            profile.t[0] = 0;
            profile.t[1] = -ad / jMax + tf;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = ad / jMax;

            if (profile.check_for_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, tf, jMax, aMax, aMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        return false;
    }

    private boolean time_none(Profile profile, double aMax, double aMin, double jMax) {
        if (Math.abs(a0) < Utils.DBL_EPSILON && Math.abs(af) < Utils.DBL_EPSILON && Math.abs(vd) < Utils.DBL_EPSILON) {
            profile.t[0] = 0;
            profile.t[1] = tf;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check_for_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, tf, jMax, aMax, aMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        // UD Solution 1/2
        {
            final double h1 = 2 * (af * tf - vd);

            profile.t[0] = h1 / ad;
            profile.t[1] = tf - profile.t[0];
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            final double jf = ad * ad / h1;

            if (Math.abs(jf) < Math.abs(jMax) + 1e-12
                    && profile.check_for_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, tf, jf, aMax, aMin)) {
                profile.pf = profile.p[7];
                return true;
            }
        }

        return false;
    }

    private boolean check_all(Profile profile, double aMax, double aMin, double jMax) {
        return time_acc0(profile, aMax, aMin, jMax) || time_none(profile, aMax, aMin, jMax);
    }

    public boolean get_profile(Profile profile) {
        if (vd > 0) {
            return check_all(profile, _aMax, _aMin, _jMax) || check_all(profile, _aMin, _aMax, -_jMax);
        }

        return check_all(profile, _aMin, _aMax, -_jMax) || check_all(profile, _aMax, _aMin, _jMax);
    }
}
