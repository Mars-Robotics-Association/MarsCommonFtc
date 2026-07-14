package com.ruckig;

/**
 * Port of {@code VelocitySecondOrderStep2} from {@code velocity.hpp} /
 * {@code velocity_second_step2.cpp} (upstream v0.17.3).
 */
public final class VelocitySecondOrderStep2 {

    private final double tf, _aMax, _aMin;
    private final double vd;

    public VelocitySecondOrderStep2(double tf, double v0, double vf, double aMax, double aMin) {
        this.tf = tf;
        this._aMax = aMax;
        this._aMin = aMin;
        this.vd = vf - v0;
    }

    public boolean get_profile(Profile profile) {
        final double af = vd / tf;

        profile.t[0] = 0;
        profile.t[1] = tf;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_second_order_velocity_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, tf, af, _aMax, _aMin)) {
            profile.pf = profile.p[7];
            return true;
        }

        return false;
    }
}
