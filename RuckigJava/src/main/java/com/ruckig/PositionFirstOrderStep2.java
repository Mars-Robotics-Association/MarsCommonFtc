package com.ruckig;

/**
 * Port of {@code PositionFirstOrderStep2} from {@code position.hpp} /
 * {@code position_first_step2.cpp} (upstream v0.17.3).
 */
public final class PositionFirstOrderStep2 {

    private final double tf, _vMax, _vMin;
    private final double pd;

    public PositionFirstOrderStep2(double tf, double p0, double pf, double vMax, double vMin) {
        this.tf = tf;
        this._vMax = vMax;
        this._vMin = vMin;
        this.pd = pf - p0;
    }

    public boolean get_profile(Profile profile) {
        final double vf = pd / tf;

        profile.t[0] = 0;
        profile.t[1] = 0;
        profile.t[2] = 0;
        profile.t[3] = tf;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        return profile.check_for_first_order_with_timing(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, tf, vf, _vMax, _vMin);
    }
}
