package com.ruckig;

/**
 * Port of {@code PositionSecondOrderStep1} from {@code position.hpp} /
 * {@code position_second_step1.cpp} (upstream v0.17.3).
 */
public final class PositionSecondOrderStep1 {

    private final double v0, vf;
    private final double _vMax, _vMin, _aMax, _aMin;
    private final double pd;

    // Max 3 valid profiles
    private final Profile[] valid_profiles = new Profile[] {
        new Profile(), new Profile(), new Profile()
    };

    public PositionSecondOrderStep1(double p0, double v0, double pf, double vf, double vMax, double vMin, double aMax, double aMin) {
        this.v0 = v0;
        this.vf = vf;
        this._vMax = vMax;
        this._vMin = vMin;
        this._aMax = aMax;
        this._aMin = aMin;
        this.pd = pf - p0;
    }

    private void add_profile(int[] profileIdx) {
        int prev = profileIdx[0];
        profileIdx[0] = prev + 1;
        valid_profiles[profileIdx[0]].set_boundary(valid_profiles[prev]);
    }

    private void time_acc0(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, boolean return_after_found) {
        Profile profile = valid_profiles[profileIdx[0]];
        profile.t[0] = (-v0 + vMax) / aMax;
        profile.t[1] = (aMin * v0 * v0 - aMax * vf * vf) / (2 * aMax * aMin * vMax) + vMax * (aMax - aMin) / (2 * aMax * aMin) + pd / vMax;
        profile.t[2] = (vf - vMax) / aMin;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_second_order(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, aMax, aMin, vMax, vMin)) {
            add_profile(profileIdx);
        }
    }

    private void time_none(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, boolean return_after_found) {
        double h1 = (aMax * vf * vf - aMin * v0 * v0 - 2 * aMax * aMin * pd) / (aMax - aMin);
        if (h1 >= 0.0) {
            h1 = Math.sqrt(h1);

            // Solution 1
            {
                Profile profile = valid_profiles[profileIdx[0]];
                profile.t[0] = -(v0 + h1) / aMax;
                profile.t[1] = 0;
                profile.t[2] = (vf + h1) / aMin;
                profile.t[3] = 0;
                profile.t[4] = 0;
                profile.t[5] = 0;
                profile.t[6] = 0;

                if (profile.check_for_second_order(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, aMax, aMin, vMax, vMin)) {
                    add_profile(profileIdx);
                    if (return_after_found) {
                        return;
                    }
                }
            }

            // Solution 2
            {
                Profile profile = valid_profiles[profileIdx[0]];
                profile.t[0] = (-v0 + h1) / aMax;
                profile.t[1] = 0;
                profile.t[2] = (vf - h1) / aMin;
                profile.t[3] = 0;
                profile.t[4] = 0;
                profile.t[5] = 0;
                profile.t[6] = 0;

                if (profile.check_for_second_order(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, aMax, aMin, vMax, vMin)) {
                    add_profile(profileIdx);
                }
            }
        }
    }

    private boolean time_all_single_step(Profile profile, double vMax, double vMin, double aMaxIgnored, double aMinIgnored) {
        if (Math.abs(vf - v0) > Utils.DBL_EPSILON) {
            return false;
        }

        profile.t[0] = 0;
        profile.t[1] = 0;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (Math.abs(v0) > Utils.DBL_EPSILON) {
            profile.t[3] = pd / v0;
            if (profile.check_for_second_order(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, 0.0, vMax, vMin)) {
                return true;
            }

        } else if (Math.abs(pd) < Utils.DBL_EPSILON) {
            if (profile.check_for_second_order(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, 0.0, vMax, vMin)) {
                return true;
            }
        }

        return false;
    }

    public boolean get_profile(Profile input, Block block) {
        // Zero-limits special case
        if (_vMax == 0.0 && _vMin == 0.0) {
            Profile p = block.p_min;
            p.set_boundary(input);

            if (time_all_single_step(p, _vMax, _vMin, _aMax, _aMin)) {
                block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
                if (Math.abs(v0) > Utils.DBL_EPSILON) {
                    block.a.set(block.t_min, Double.POSITIVE_INFINITY);
                    block.has_a = true;
                } else {
                    block.has_a = false;
                }
                block.has_b = false;
                return true;
            }
            return false;
        }

        final int start = 0;
        int[] profileIdx = new int[] { start };
        valid_profiles[profileIdx[0]].set_boundary(input);

        if (Math.abs(vf) < Utils.DBL_EPSILON) {
            // There is no blocked interval when vf==0, so return after first found profile
            final double vMax = (pd >= 0) ? _vMax : _vMin;
            final double vMin = (pd >= 0) ? _vMin : _vMax;
            final double aMax = (pd >= 0) ? _aMax : _aMin;
            final double aMin = (pd >= 0) ? _aMin : _aMax;

            time_none(profileIdx, vMax, vMin, aMax, aMin, true);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc0(profileIdx, vMax, vMin, aMax, aMin, true);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }

            time_none(profileIdx, vMin, vMax, aMin, aMax, true);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc0(profileIdx, vMin, vMax, aMin, aMax, true);

        } else {
            time_none(profileIdx, _vMax, _vMin, _aMax, _aMin, false);
            time_none(profileIdx, _vMin, _vMax, _aMin, _aMax, false);
            time_acc0(profileIdx, _vMax, _vMin, _aMax, _aMin, false);
            time_acc0(profileIdx, _vMin, _vMax, _aMin, _aMax, false);
        }

        return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
    }
}
