package com.ruckig;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * M2 gate: integrate known 7-phase profiles; check accepts good / rejects bad.
 */
class ProfileTest {

    @Test
    void velocityUdduRestToRestAccepted() {
        // Symmetric rest-to-rest velocity change with bang-coast-bang accel
        Profile p = new Profile();
        p.set_boundary_for_velocity(0, 0, 0, 1.0, 0);
        double aMax = 1.0;
        double aMin = -1.0;
        double jMax = 2.0;
        // j to aMax, coast, j to 0 at vf
        // t0 = aMax/jMax = 0.5; t2 = 0.5; t1 for vd: vd = aMax*(t0+t1+t2)/2 * 2 ... use step1 formulas
        p.t[0] = (-0 + aMax) / jMax;
        p.t[1] = (0 + 0) / (2 * aMax * jMax) - aMax / jMax + 1.0 / aMax;
        p.t[2] = (-0 + aMax) / jMax;
        p.t[3] = 0;
        p.t[4] = 0;
        p.t[5] = 0;
        p.t[6] = 0;
        assertTrue(p.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, jMax, aMax, aMin));
        assertTrue(p.t_sum[6] > 0);
        assertTrue(Math.abs(p.v[7] - 1.0) < Profile.V_PRECISION);
    }

    @Test
    void negativePhaseTimeRejected() {
        Profile p = new Profile();
        p.set_boundary_for_velocity(0, 0, 0, 1.0, 0);
        p.t[0] = -1.0;
        p.t[1] = 1.0;
        p.t[2] = 1.0;
        p.t[3] = 0;
        p.t[4] = 0;
        p.t[5] = 0;
        p.t[6] = 0;
        assertFalse(p.check_for_velocity(Profile.ControlSigns.UDDU, Profile.ReachedLimits.ACC0, 2.0, 1.0, -1.0));
    }

    @Test
    void positionCheckSimpleTrapezoid() {
        // Hand-built short move that should fail if target wrong
        Profile p = new Profile();
        p.set_boundary(0, 0, 0, 1.0, 0, 0);
        // Zero-time profile at same state is invalid for distance 1
        for (int i = 0; i < 7; ++i) {
            p.t[i] = 0;
        }
        assertFalse(p.check(Profile.ControlSigns.UDDU, Profile.ReachedLimits.NONE, 1.0, 2.0, -2.0, 2.0, -2.0));
    }

    @Test
    void copyFromDeep() {
        Profile a = new Profile();
        a.t[0] = 1.5;
        a.pf = 3.0;
        a.brake.duration = 0.2;
        a.brake.t[0] = 0.1;
        Profile b = new Profile();
        b.copyFrom(a);
        a.t[0] = 0;
        a.brake.t[0] = 0;
        assertTrue(b.t[0] == 1.5);
        assertTrue(b.pf == 3.0);
        assertTrue(b.brake.duration == 0.2);
        assertTrue(b.brake.t[0] == 0.1);
    }
}
