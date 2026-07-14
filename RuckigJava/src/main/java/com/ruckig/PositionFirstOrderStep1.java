package com.ruckig;

/**
 * Port of {@code PositionFirstOrderStep1} from {@code position.hpp} /
 * {@code position_first_step1.cpp} (upstream v0.17.3).
 */
public final class PositionFirstOrderStep1 {

    private final double _vMax, _vMin;
    private final double pd;

    public PositionFirstOrderStep1(double p0, double pf, double vMax, double vMin) {
        this._vMax = vMax;
        this._vMin = vMin;
        this.pd = pf - p0;
    }

    public boolean get_profile(Profile input, Block block) {
        Profile p = block.p_min;
        p.set_boundary(input);

        final double vf = (pd > 0) ? _vMax : _vMin;
        p.t[0] = 0;
        p.t[1] = 0;
        p.t[2] = 0;
        p.t[3] = pd / vf;
        p.t[4] = 0;
        p.t[5] = 0;
        p.t[6] = 0;

        if (p.check_for_first_order(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, vf)) {
            block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
            return true;
        }
        return false;
    }
}
