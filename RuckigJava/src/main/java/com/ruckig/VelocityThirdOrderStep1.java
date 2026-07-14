package com.ruckig;

/**
 * Port of {@code VelocityThirdOrderStep1} from {@code velocity.hpp} /
 * {@code velocity_third_step1.cpp} (upstream v0.17.3).
 */
public final class VelocityThirdOrderStep1 {

    private final double a0, af;
    private final double _aMax, _aMin, _jMax;
    private final double vd;

    // Max 3 valid profiles
    private final Profile[] valid_profiles = new Profile[] {
        new Profile(), new Profile(), new Profile()
    };

    public VelocityThirdOrderStep1(double v0, double a0, double vf, double af, double aMax, double aMin, double jMax) {
        this.a0 = a0;
        this.af = af;
        this._aMax = aMax;
        this._aMin = aMin;
        this._jMax = jMax;
        this.vd = vf - v0;
    }

    private void add_profile(int[] profileIdx) {
        int prev = profileIdx[0];
        profileIdx[0] = prev + 1;
        valid_profiles[profileIdx[0]].set_boundary(valid_profiles[prev]);
    }

    private void time_acc0(int[] profileIdx, double aMax, double aMin, double jMax, boolean return_after_found) {
        Profile profile = valid_profiles[profileIdx[0]];
        profile.t[0] = (-a0 + aMax) / jMax;
        profile.t[1] = (a0 * a0 + af * af) / (2 * aMax * jMax) - aMax / jMax + vd / aMax;
        profile.t[2] = (-af + aMax) / jMax;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (profile.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, aMax, aMin)) {
            add_profile(profileIdx);
        }
    }

    private void time_none(int[] profileIdx, double aMax, double aMin, double jMax, boolean return_after_found) {
        double h1 = (a0 * a0 + af * af) / 2 + jMax * vd;
        if (h1 >= 0.0) {
            h1 = Math.sqrt(h1);

            // Solution 1
            {
                Profile profile = valid_profiles[profileIdx[0]];
                profile.t[0] = -(a0 + h1) / jMax;
                profile.t[1] = 0;
                profile.t[2] = -(af + h1) / jMax;
                profile.t[3] = 0;
                profile.t[4] = 0;
                profile.t[5] = 0;
                profile.t[6] = 0;

                if (profile.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, aMax, aMin)) {
                    add_profile(profileIdx);
                    if (return_after_found) {
                        return;
                    }
                }
            }

            // Solution 2
            {
                Profile profile = valid_profiles[profileIdx[0]];
                profile.t[0] = (-a0 + h1) / jMax;
                profile.t[1] = 0;
                profile.t[2] = (-af + h1) / jMax;
                profile.t[3] = 0;
                profile.t[4] = 0;
                profile.t[5] = 0;
                profile.t[6] = 0;

                if (profile.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, aMax, aMin)) {
                    add_profile(profileIdx);
                }
            }
        }
    }

    private boolean time_all_single_step(Profile profile, double aMax, double aMin, double jMaxIgnored) {
        if (Math.abs(af - a0) > Utils.DBL_EPSILON) {
            return false;
        }

        profile.t[0] = 0;
        profile.t[1] = 0;
        profile.t[2] = 0;
        profile.t[3] = 0;
        profile.t[4] = 0;
        profile.t[5] = 0;
        profile.t[6] = 0;

        if (Math.abs(a0) > Utils.DBL_EPSILON) {
            profile.t[3] = vd / a0;
            if (profile.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, aMax, aMin)) {
                return true;
            }

        } else if (Math.abs(vd) < Utils.DBL_EPSILON) {
            if (profile.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, aMax, aMin)) {
                return true;
            }
        }

        return false;
    }

    public boolean get_profile(Profile input, Block block) {
        // Zero-limits special case
        if (_jMax == 0.0) {
            Profile p = block.p_min;
            p.set_boundary(input);

            if (time_all_single_step(p, _aMax, _aMin, _jMax)) {
                block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
                if (Math.abs(a0) > Utils.DBL_EPSILON) {
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

        if (Math.abs(af) < Utils.DBL_EPSILON) {
            // There is no blocked interval when af==0, so return after first found profile
            final double aMax = (vd >= 0) ? _aMax : _aMin;
            final double aMin = (vd >= 0) ? _aMin : _aMax;
            final double jMax = (vd >= 0) ? _jMax : -_jMax;

            time_none(profileIdx, aMax, aMin, jMax, true);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc0(profileIdx, aMax, aMin, jMax, true);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }

            time_none(profileIdx, aMin, aMax, -jMax, true);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc0(profileIdx, aMin, aMax, -jMax, true);

        } else {
            time_none(profileIdx, _aMax, _aMin, _jMax, false);
            time_none(profileIdx, _aMin, _aMax, -_jMax, false);
            time_acc0(profileIdx, _aMax, _aMin, _jMax, false);
            time_acc0(profileIdx, _aMin, _aMax, -_jMax, false);
        }

        return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
    }
}
