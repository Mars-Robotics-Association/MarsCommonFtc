package com.ruckig;

/**
 * Port of {@code VelocitySecondOrderStep1} from {@code velocity.hpp} /
 * {@code velocity_second_step1.cpp} (upstream v0.17.3).
 */
public final class VelocitySecondOrderStep1 {

    private final double _aMax, _aMin;
    private final double vd;

    public VelocitySecondOrderStep1(double v0, double vf, double aMax, double aMin) {
        this._aMax = aMax;
        this._aMin = aMin;
        this.vd = vf - v0;
    }

    public boolean get_profile(Profile input, Block block) {
        Profile p = block.p_min;
        p.set_boundary(input);

        final double af = (vd > 0) ? _aMax : _aMin;
        p.t[0] = 0;
        p.t[1] = vd / af;
        p.t[2] = 0;
        p.t[3] = 0;
        p.t[4] = 0;
        p.t[5] = 0;
        p.t[6] = 0;

        if (p.check_for_second_order_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, af)) {
            block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
            return true;
        }
        return false;
    }
}
