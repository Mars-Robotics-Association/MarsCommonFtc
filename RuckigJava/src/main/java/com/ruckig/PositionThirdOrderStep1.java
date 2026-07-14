package com.ruckig;

/**
 * Port of {@code PositionThirdOrderStep1} from {@code position.hpp} /
 * {@code position_third_step1.cpp} (upstream v0.17.3).
 * Faithful transliteration — same variables, expression order, and control flow.
 */
public final class PositionThirdOrderStep1 {

    private final double v0, a0;
    private final double vf, af;
    private final double _vMax, _vMin, _aMax, _aMin, _jMax;

    // Pre-calculated expressions
    private final double pd;
    private final double v0_v0, vf_vf;
    private final double a0_a0, a0_p3, a0_p4;
    private final double af_af, af_p3, af_p4;
    private final double jMax_jMax;

    // Max 5 valid profiles + 1 spare for numerical issues
    private final Profile[] valid_profiles = new Profile[] {
        new Profile(), new Profile(), new Profile(),
        new Profile(), new Profile(), new Profile()
    };

    public PositionThirdOrderStep1(double p0, double v0, double a0, double pf, double vf, double af,
            double vMax, double vMin, double aMax, double aMin, double jMax) {
        this.v0 = v0;
        this.a0 = a0;
        this.vf = vf;
        this.af = af;
        this._vMax = vMax;
        this._vMin = vMin;
        this._aMax = aMax;
        this._aMin = aMin;
        this._jMax = jMax;

        pd = pf - p0;

        v0_v0 = v0 * v0;
        vf_vf = vf * vf;

        a0_a0 = a0 * a0;
        af_af = af * af;

        a0_p3 = a0 * a0_a0;
        a0_p4 = a0_a0 * a0_a0;
        af_p3 = af * af_af;
        af_p4 = af_af * af_af;

        // max values needs to be invariant to plus minus sign change
        jMax_jMax = jMax * jMax;
    }

    private void add_profile(int[] profileIdx) {
        int prev = profileIdx[0];
        profileIdx[0] = prev + 1;
        valid_profiles[profileIdx[0]].set_boundary(valid_profiles[prev]);
    }

    private void time_all_vel(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax, boolean return_after_found_unused) {
        Profile profile = valid_profiles[profileIdx[0]];

        // ACC0_ACC1_VEL
        profile.t[0] = (-a0 + aMax) / jMax;
        profile.t[1] = (a0_a0 / 2 - aMax * aMax - jMax * (v0 - vMax)) / (aMax * jMax);
        profile.t[2] = aMax / jMax;
        profile.t[3] = (3 * (a0_p4 * aMin - af_p4 * aMax) + 8 * aMax * aMin * (af_p3 - a0_p3 + 3 * jMax * (a0 * v0 - af * vf)) + 6 * a0_a0 * aMin * (aMax * aMax - 2 * jMax * v0) - 6 * af_af * aMax * (aMin * aMin - 2 * jMax * vf) - 12 * jMax * (aMax * aMin * (aMax * (v0 + vMax) - aMin * (vf + vMax) - 2 * jMax * pd) + (aMin - aMax) * jMax * vMax * vMax + jMax * (aMax * vf_vf - aMin * v0_v0))) / (24 * aMax * aMin * jMax_jMax * vMax);
        profile.t[4] = -aMin / jMax;
        profile.t[5] = -(af_af / 2 - aMin * aMin - jMax * (vf - vMax)) / (aMin * jMax);
        profile.t[6] = profile.t[4] + af / jMax;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_ACC1_VEL, jMax, vMax, vMin, aMax, aMin)) {
            add_profile(profileIdx);
            return;
        }

        // ACC1_VEL
        final double t_acc0 = Math.sqrt(a0_a0 / (2 * jMax_jMax) + (vMax - v0) / jMax);

        profile = valid_profiles[profileIdx[0]];
        profile.t[0] = t_acc0 - a0 / jMax;
        profile.t[1] = 0;
        profile.t[2] = t_acc0;
        profile.t[3] = -(3 * af_p4 - 8 * aMin * (af_p3 - a0_p3) - 24 * aMin * jMax * (a0 * v0 - af * vf) + 6 * af_af * (aMin * aMin - 2 * jMax * vf) - 12 * jMax * (2 * aMin * jMax * pd + aMin * aMin * (vf + vMax) + jMax * (vMax * vMax - vf_vf) + aMin * t_acc0 * (a0_a0 - 2 * jMax * (v0 + vMax)))) / (24 * aMin * jMax_jMax * vMax);
        // profile.t[4] = -aMin/jMax;
        // profile.t[5] = -(af_af/2 - aMin*aMin + jMax*(vMax - vf))/(aMin*jMax);
        // profile.t[6] = profile.t[4] + af/jMax;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC1_VEL, jMax, vMax, vMin, aMax, aMin)) {
            add_profile(profileIdx);
            return;
        }

        // ACC0_VEL
        final double t_acc1 = Math.sqrt(af_af / (2 * jMax_jMax) + (vMax - vf) / jMax);

        profile = valid_profiles[profileIdx[0]];
        profile.t[0] = (-a0 + aMax) / jMax;
        profile.t[1] = (a0_a0 / 2 - aMax * aMax - jMax * (v0 - vMax)) / (aMax * jMax);
        profile.t[2] = aMax / jMax;
        profile.t[3] = (3 * a0_p4 + 8 * aMax * (af_p3 - a0_p3) + 24 * aMax * jMax * (a0 * v0 - af * vf) + 6 * a0_a0 * (aMax * aMax - 2 * jMax * v0) - 12 * jMax * (-2 * aMax * jMax * pd + aMax * aMax * (v0 + vMax) + jMax * (vMax * vMax - v0_v0) + aMax * t_acc1 * (-af_af + 2 * (vf + vMax) * jMax))) / (24 * aMax * jMax_jMax * vMax);
        profile.t[4] = t_acc1;
        profile.t[5] = 0;
        profile.t[6] = t_acc1 + af / jMax;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_VEL, jMax, vMax, vMin, aMax, aMin)) {
            add_profile(profileIdx);
            return;
        }

        // VEL
        // Solution 3/4
        profile = valid_profiles[profileIdx[0]];
        profile.t[0] = t_acc0 - a0 / jMax;
        profile.t[1] = 0;
        profile.t[2] = t_acc0;
        profile.t[3] = (af_p3 - a0_p3) / (3 * jMax_jMax * vMax) + (a0 * v0 - af * vf + (af_af * t_acc1 + a0_a0 * t_acc0) / 2) / (jMax * vMax) - (v0 / vMax + 1.0) * t_acc0 - (vf / vMax + 1.0) * t_acc1 + pd / vMax;
        // profile.t[4] = t_acc1;
        // profile.t[5] = 0;
        // profile.t[6] = t_acc1 + af/jMax;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, jMax, vMax, vMin, aMax, aMin)) {
            add_profile(profileIdx);
        }
    }

    private void time_acc0_acc1(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax, boolean return_after_found) {
        double h1 = (3 * (af_p4 * aMax - a0_p4 * aMin) + aMax * aMin * (8 * (a0_p3 - af_p3) + 3 * aMax * aMin * (aMax - aMin) + 6 * aMin * af_af - 6 * aMax * a0_a0) + 12 * jMax * (aMax * aMin * ((aMax - 2 * a0) * v0 - (aMin - 2 * af) * vf) + aMin * a0_a0 * v0 - aMax * af_af * vf)) / (3 * (aMax - aMin) * jMax_jMax) + 4 * (aMax * vf_vf - aMin * v0_v0 - 2 * aMin * aMax * pd) / (aMax - aMin);

        if (h1 >= 0) {
            h1 = Math.sqrt(h1) / 2;
            final double h2 = a0_a0 / (2 * aMax * jMax) + (aMin - 2 * aMax) / (2 * jMax) - v0 / aMax;
            final double h3 = -af_af / (2 * aMin * jMax) - (aMax - 2 * aMin) / (2 * jMax) + vf / aMin;

            // UDDU: Solution 2
            if (h2 > h1 / aMax && h3 > -h1 / aMin) {
                Profile profile = valid_profiles[profileIdx[0]];
                profile.t[0] = (-a0 + aMax) / jMax;
                profile.t[1] = h2 - h1 / aMax;
                profile.t[2] = aMax / jMax;
                profile.t[3] = 0;
                profile.t[4] = -aMin / jMax;
                profile.t[5] = h3 + h1 / aMin;
                profile.t[6] = profile.t[4] + af / jMax;

                if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_ACC1, true, jMax, vMax, vMin, aMax, aMin)) {
                    add_profile(profileIdx);
                    if (return_after_found) {
                        return;
                    }
                }
            }

            // UDDU: Solution 1
            if (h2 > -h1 / aMax && h3 > h1 / aMin) {
                Profile profile = valid_profiles[profileIdx[0]];
                profile.t[0] = (-a0 + aMax) / jMax;
                profile.t[1] = h2 + h1 / aMax;
                profile.t[2] = aMax / jMax;
                profile.t[3] = 0;
                profile.t[4] = -aMin / jMax;
                profile.t[5] = h3 - h1 / aMin;
                profile.t[6] = profile.t[4] + af / jMax;

                if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0_ACC1, true, jMax, vMax, vMin, aMax, aMin)) {
                    add_profile(profileIdx);
                }
            }
        }
    }

    private void time_all_none_acc0_acc1(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax, boolean return_after_found) {
        // NONE UDDU / UDUD Strategy: t7 == 0 (equals UDDU), this one is in particular prone to numerical issues
        final double h2_none = (a0_a0 - af_af) / (2 * jMax) + (vf - v0);
        final double h2_h2 = h2_none * h2_none;
        final double t_min_none = (a0 - af) / jMax;
        final double t_max_none = (aMax - aMin) / jMax;

        final double[] polynom_none = new double[4];
        polynom_none[0] = 0;
        polynom_none[1] = -2 * (a0_a0 + af_af - 2 * jMax * (v0 + vf)) / jMax_jMax;
        polynom_none[2] = 4 * (a0_p3 - af_p3 + 3 * jMax * (af * vf - a0 * v0)) / (3 * jMax * jMax_jMax) - 4 * pd / jMax;
        polynom_none[3] = -h2_h2 / jMax_jMax;


        // ACC0
        final double h3_acc0 = (a0_a0 - af_af) / (2 * aMax * jMax) + (vf - v0) / aMax;
        final double t_min_acc0 = (aMax - af) / jMax;
        final double t_max_acc0 = (aMax - aMin) / jMax;

        final double h0_acc0 = 3 * (af_p4 - a0_p4) + 8 * (a0_p3 - af_p3) * aMax + 24 * aMax * jMax * (af * vf - a0 * v0) - 6 * a0_a0 * (aMax * aMax - 2 * jMax * v0) + 6 * af_af * (aMax * aMax - 2 * jMax * vf) + 12 * jMax * (jMax * (vf_vf - v0_v0 - 2 * aMax * pd) - aMax * aMax * (vf - v0));
        final double h2_acc0 = -af_af + aMax * aMax + 2 * jMax * vf;

        final double[] polynom_acc0 = new double[4];
        polynom_acc0[0] = -2 * aMax / jMax;
        polynom_acc0[1] = h2_acc0 / jMax_jMax;
        polynom_acc0[2] = 0;
        polynom_acc0[3] = h0_acc0 / (12 * jMax_jMax * jMax_jMax);


        // ACC1
        final double h3_acc1 = -(a0_a0 + af_af) / (2 * jMax * aMin) + aMin / jMax + (vf - v0) / aMin;
        final double t_min_acc1 = (aMin - a0) / jMax;
        final double t_max_acc1 = (aMax - a0) / jMax;

        final double h0_acc1 = (a0_p4 - af_p4) / 4 + 2 * (af_p3 - a0_p3) * aMin / 3 + (a0_a0 - af_af) * aMin * aMin / 2 + jMax * (af_af * vf + a0_a0 * v0 + 2 * aMin * (jMax * pd - a0 * v0 - af * vf) + aMin * aMin * (v0 + vf) + jMax * (v0_v0 - vf_vf));
        final double h2_acc1 = a0_a0 - a0 * aMin + 2 * jMax * v0;

        final double[] polynom_acc1 = new double[4];
        polynom_acc1[0] = 2 * (2 * a0 - aMin) / jMax;
        polynom_acc1[1] = (5 * a0_a0 + aMin * (aMin - 6 * a0) + 2 * jMax * v0) / jMax_jMax;
        polynom_acc1[2] = 2 * (a0 - aMin) * h2_acc1 / (jMax_jMax * jMax);
        polynom_acc1[3] = h0_acc1 / (jMax_jMax * jMax_jMax);


        final Roots.PositiveDoubleSet roots_none = new Roots.PositiveDoubleSet(4);
        final Roots.PositiveDoubleSet roots_acc0 = new Roots.PositiveDoubleSet(4);
        final Roots.PositiveDoubleSet roots_acc1 = new Roots.PositiveDoubleSet(4);
        Roots.solveQuartMonic(polynom_none, roots_none);
        Roots.solveQuartMonic(polynom_acc0, roots_acc0);
        Roots.solveQuartMonic(polynom_acc1, roots_acc1);

        final int nNone = roots_none.sortedSize();
        for (int ri = 0; ri < nNone; ++ri) {
            double t = roots_none.get(ri);
            if (t < t_min_none || t > t_max_none) {
                continue;
            }

            // Single Newton-step (regarding pd)
            if (t > Utils.DBL_EPSILON) {
                final double h1 = jMax * t * t;
                final double orig = -h2_h2 / (4 * jMax * t) + h2_none * (af / jMax + t) + (4 * a0_p3 + 2 * af_p3 - 6 * a0_a0 * (af + 2 * jMax * t) + 12 * (af - a0) * jMax * v0 + 3 * jMax_jMax * (-4 * pd + (h1 + 8 * v0) * t)) / (12 * jMax_jMax);
                final double deriv = h2_none + 2 * v0 - a0_a0 / jMax + h2_h2 / (4 * h1) + (3 * h1) / 4;

                t -= orig / deriv;
            }

            final double h0 = h2_none / (2 * jMax * t);
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = h0 + t / 2 - a0 / jMax;
            profile.t[1] = 0;
            profile.t[2] = t;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = -h0 + t / 2 + af / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                if (return_after_found) {
                    return;
                }
            }
        }

        final int nAcc0 = roots_acc0.sortedSize();
        for (int ri = 0; ri < nAcc0; ++ri) {
            double t = roots_acc0.get(ri);
            if (t < t_min_acc0 || t > t_max_acc0) {
                continue;
            }

            // Single Newton step (regarding pd)
            if (t > Utils.DBL_EPSILON) {
                final double h1 = jMax * t;
                final double orig = h0_acc0 / (12 * jMax_jMax * t) + t * (h2_acc0 + h1 * (h1 - 2 * aMax));
                final double deriv = 2 * (h2_acc0 + h1 * (2 * h1 - 3 * aMax));

                t -= orig / deriv;
            }

            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = (-a0 + aMax) / jMax;
            profile.t[1] = h3_acc0 - 2 * t + jMax / aMax * t * t;
            profile.t[2] = t;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = (af - aMax) / jMax + t;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                if (return_after_found) {
                    return;
                }
            }
        }

        final int nAcc1 = roots_acc1.sortedSize();
        for (int ri = 0; ri < nAcc1; ++ri) {
            double t = roots_acc1.get(ri);
            if (t < t_min_acc1 || t > t_max_acc1) {
                continue;
            }

            // Double Newton step (regarding pd)
            if (t > Utils.DBL_EPSILON) {
                final double h5 = a0_p3 + 2 * jMax * a0 * v0;
                double h1 = jMax * t;
                double orig = -(h0_acc1 / 2 + h1 * (h5 + a0 * (aMin - 2 * h1) * (aMin - h1) + a0_a0 * (5 * h1 / 2 - 2 * aMin) + aMin * aMin * h1 / 2 + jMax * (h1 / 2 - aMin) * (h1 * t + 2 * v0))) / jMax;
                double deriv = (aMin - a0 - h1) * (h2_acc1 + h1 * (4 * a0 - aMin + 2 * h1));
                t -= Utils.cppMin(orig / deriv, t);

                h1 = jMax * t;
                orig = -(h0_acc1 / 2 + h1 * (h5 + a0 * (aMin - 2 * h1) * (aMin - h1) + a0_a0 * (5 * h1 / 2 - 2 * aMin) + aMin * aMin * h1 / 2 + jMax * (h1 / 2 - aMin) * (h1 * t + 2 * v0))) / jMax;

                if (Math.abs(orig) > 1e-9) {
                    deriv = (aMin - a0 - h1) * (h2_acc1 + h1 * (4 * a0 - aMin + 2 * h1));
                    t -= orig / deriv;

                    h1 = jMax * t;
                    orig = -(h0_acc1 / 2 + h1 * (h5 + a0 * (aMin - 2 * h1) * (aMin - h1) + a0_a0 * (5 * h1 / 2 - 2 * aMin) + aMin * aMin * h1 / 2 + jMax * (h1 / 2 - aMin) * (h1 * t + 2 * v0))) / jMax;

                    if (Math.abs(orig) > 1e-9) {
                        deriv = (aMin - a0 - h1) * (h2_acc1 + h1 * (4 * a0 - aMin + 2 * h1));
                        t -= orig / deriv;
                    }
                }
            }

            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = t;
            profile.t[1] = 0;
            profile.t[2] = (a0 - aMin) / jMax + t;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = h3_acc1 - (2 * a0 + jMax * t) * t / aMin;
            profile.t[6] = (af - aMin) / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC1, true, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                if (return_after_found) {
                    return;
                }
            }
        }
    }

    private void time_acc1_vel_two_step(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax) {
        Profile profile = valid_profiles[profileIdx[0]];
        profile.t[0] = 0;
        profile.t[1] = 0;
        profile.t[2] = a0 / jMax;
        profile.t[3] = -(3 * af_p4 - 8 * aMin * (af_p3 - a0_p3) - 24 * aMin * jMax * (a0 * v0 - af * vf) + 6 * af_af * (aMin * aMin - 2 * jMax * vf) - 12 * jMax * (2 * aMin * jMax * pd + aMin * aMin * (vf + vMax) + jMax * (vMax * vMax - vf_vf) + aMin * a0 * (a0_a0 - 2 * jMax * (v0 + vMax)) / jMax)) / (24 * aMin * jMax_jMax * vMax);
        profile.t[4] = -aMin / jMax;
        profile.t[5] = -(af_af / 2 - aMin * aMin + jMax * (vMax - vf)) / (aMin * jMax);
        profile.t[6] = profile.t[4] + af / jMax;

        if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC1_VEL, jMax, vMax, vMin, aMax, aMin)) {
            add_profile(profileIdx);
        }
    }

    private void time_acc0_two_step(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax) {
        // Two step
        {
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = 0;
            profile.t[1] = (af_af - a0_a0 + 2 * jMax * (vf - v0)) / (2 * a0 * jMax);
            profile.t[2] = (a0 - af) / jMax;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }

        // Three step - Removed pf
        {
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = (-a0 + aMax) / jMax;
            profile.t[1] = (a0_a0 + af_af - 2 * aMax * aMax + 2 * jMax * (vf - v0)) / (2 * aMax * jMax);
            profile.t[2] = (-af + aMax) / jMax;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }

        // Three step - Removed aMax
        {
            final double h0 = 3 * (af_af - a0_a0 + 2 * jMax * (v0 + vf));
            final double h2 = a0_p3 + 2 * af_p3 + 6 * jMax_jMax * pd + 6 * (af - a0) * jMax * vf - 3 * a0 * af_af;
            final double h1 = Math.sqrt(2 * (2 * h2 * h2 + h0 * (a0_p4 - 6 * a0_a0 * (af_af + 2 * jMax * vf) + 8 * a0 * (af_p3 + 3 * jMax_jMax * pd + 3 * af * jMax * vf) - 3 * (af_p4 + 4 * af_af * jMax * vf + 4 * jMax_jMax * (vf_vf - v0_v0))))) * Math.abs(jMax) / jMax;
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = (4 * af_p3 + 2 * a0_p3 - 6 * a0 * af_af + 12 * jMax_jMax * pd + 12 * (af - a0) * jMax * vf + h1) / (2 * jMax * h0);
            profile.t[1] = -h1 / (jMax * h0);
            profile.t[2] = (-4 * a0_p3 - 2 * af_p3 + 6 * a0_a0 * af + 12 * jMax_jMax * pd - 12 * (af - a0) * jMax * v0 + h1) / (2 * jMax * h0);
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }

        // Three step - t=(aMax - aMin)/jMax
        {
            final double t = (aMax - aMin) / jMax;

            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = (-a0 + aMax) / jMax;
            profile.t[1] = (a0_a0 - af_af) / (2 * aMax * jMax) + (vf - v0 + jMax * t * t) / aMax - 2 * t;
            profile.t[2] = t;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = (af - aMin) / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }
    }

    private void time_vel_two_step(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax) {
        final double h1 = Math.sqrt(af_af / (2 * jMax_jMax) + (vMax - vf) / jMax);

        // Four step
        {
            // Solution 3/4
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = -a0 / jMax;
            profile.t[1] = 0;
            profile.t[2] = 0;
            profile.t[3] = (af_p3 - a0_p3) / (3 * jMax_jMax * vMax) + (a0 * v0 - af * vf + (af_af * h1) / 2) / (jMax * vMax) - (vf / vMax + 1.0) * h1 + pd / vMax;
            profile.t[4] = h1;
            profile.t[5] = 0;
            profile.t[6] = h1 + af / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }


        // Four step
        {
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = 0;
            profile.t[1] = 0;
            profile.t[2] = a0 / jMax;
            profile.t[3] = (af_p3 - a0_p3) / (3 * jMax_jMax * vMax) + (a0 * v0 - af * vf + (af_af * h1 + a0_p3 / jMax) / 2) / (jMax * vMax) - (v0 / vMax + 1.0) * a0 / jMax - (vf / vMax + 1.0) * h1 + pd / vMax;
            profile.t[4] = h1;
            profile.t[5] = 0;
            profile.t[6] = h1 + af / jMax;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.VEL, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }
    }

    private void time_none_two_step(int[] profileIdx, double vMax, double vMin, double aMax, double aMin, double jMax) {
        // Two step
        {
            final double h0 = Math.sqrt((a0_a0 + af_af) / 2 + jMax * (vf - v0)) * Math.abs(jMax) / jMax;
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = (h0 - a0) / jMax;
            profile.t[1] = 0;
            profile.t[2] = (h0 - af) / jMax;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }

        // Single step
        {
            Profile profile = valid_profiles[profileIdx[0]];
            profile.t[0] = (af - a0) / jMax;
            profile.t[1] = 0;
            profile.t[2] = 0;
            profile.t[3] = 0;
            profile.t[4] = 0;
            profile.t[5] = 0;
            profile.t[6] = 0;

            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, jMax, vMax, vMin, aMax, aMin)) {
                add_profile(profileIdx);
                return;
            }
        }
    }

    private boolean time_all_single_step(Profile profile, double vMax, double vMin, double aMax, double aMin, double jMaxIgnored) {
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
            final double q = Math.sqrt(2 * a0 * pd + v0_v0);

            // Solution 1
            profile.t[3] = (-v0 + q) / a0;
            if (profile.t[3] >= 0.0 && profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, vMax, vMin, aMax, aMin)) {
                return true;
            }

            // Solution 2
            profile.t[3] = -(v0 + q) / a0;
            if (profile.t[3] >= 0.0 && profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, vMax, vMin, aMax, aMin)) {
                return true;
            }

        } else if (Math.abs(v0) > Utils.DBL_EPSILON) {
            profile.t[3] = pd / v0;
            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, vMax, vMin, aMax, aMin)) {
                return true;
            }

        } else if (Math.abs(pd) < Utils.DBL_EPSILON) {
            if (profile.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 0.0, vMax, vMin, aMax, aMin)) {
                return true;
            }
        }

        return false;
    }

    public boolean get_profile(Profile input, Block block) {
        // Zero-limits special case
        if (_jMax == 0.0 || _aMax == 0.0 || _aMin == 0.0) {
            Profile p = block.p_min;
            p.set_boundary(input);

            if (time_all_single_step(p, _vMax, _vMin, _aMax, _aMin, _jMax)) {
                block.t_min = p.t_sum[6] + p.brake.duration + p.accel.duration;
                if (Math.abs(v0) > Utils.DBL_EPSILON || Math.abs(a0) > Utils.DBL_EPSILON) {
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

        if (Math.abs(vf) < Utils.DBL_EPSILON && Math.abs(af) < Utils.DBL_EPSILON) {
            final double vMax = (pd >= 0) ? _vMax : _vMin;
            final double vMin = (pd >= 0) ? _vMin : _vMax;
            final double aMax = (pd >= 0) ? _aMax : _aMin;
            final double aMin = (pd >= 0) ? _aMin : _aMax;
            final double jMax = (pd >= 0) ? _jMax : -_jMax;

            if (Math.abs(v0) < Utils.DBL_EPSILON && Math.abs(a0) < Utils.DBL_EPSILON && Math.abs(pd) < Utils.DBL_EPSILON) {
                time_all_none_acc0_acc1(profileIdx, vMax, vMin, aMax, aMin, jMax, true);

            } else {
                // There is no blocked interval when vf==0 && af==0, so return after first found profile
                time_all_vel(profileIdx, vMax, vMin, aMax, aMin, jMax, true);
                if (profileIdx[0] > start) {
                    return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
                }
                time_all_none_acc0_acc1(profileIdx, vMax, vMin, aMax, aMin, jMax, true);
                if (profileIdx[0] > start) {
                    return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
                }
                time_acc0_acc1(profileIdx, vMax, vMin, aMax, aMin, jMax, true);
                if (profileIdx[0] > start) {
                    return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
                }

                time_all_vel(profileIdx, vMin, vMax, aMin, aMax, -jMax, true);
                if (profileIdx[0] > start) {
                    return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
                }
                time_all_none_acc0_acc1(profileIdx, vMin, vMax, aMin, aMax, -jMax, true);
                if (profileIdx[0] > start) {
                    return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
                }
                time_acc0_acc1(profileIdx, vMin, vMax, aMin, aMax, -jMax, true);
            }

        } else {
            time_all_none_acc0_acc1(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax, false);
            time_all_none_acc0_acc1(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax, false);
            time_acc0_acc1(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax, false);
            time_acc0_acc1(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax, false);
            time_all_vel(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax, false);
            time_all_vel(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax, false);
        }

        if (profileIdx[0] == start) {
            time_none_two_step(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_none_two_step(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc0_two_step(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc0_two_step(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_vel_two_step(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_vel_two_step(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc1_vel_two_step(profileIdx, _vMax, _vMin, _aMax, _aMin, _jMax);
            if (profileIdx[0] > start) {
                return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
            }
            time_acc1_vel_two_step(profileIdx, _vMin, _vMax, _aMin, _aMax, -_jMax);
        }

        return Block.calculate_block(block, valid_profiles, profileIdx[0] - start);
    }
}
