package org.marsroboticsassociation.controllib.motion;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

import static org.junit.jupiter.api.Assertions.*;

class SCurvePositionTest {

    // ---------------------------------------------------------------
    // Test configurations for parametric tests
    // ---------------------------------------------------------------

    record Config(String label, double p0, double pTarget, double v0, double a0,
                  double vMax, double aMaxAccel, double aMaxDecel, double jMax) {}

    static Stream<Config> allConfigs() {
        return Stream.of(
            new Config("sym_short",       0, 1,   0, 0, 5,   3,   3,   10),
            new Config("sym_long",        0, 100, 0, 0, 5,   3,   3,   10),
            new Config("asym_accel",      0, 50,  0, 0, 8,   5,   2,   12),
            new Config("asym_decel",      0, 50,  0, 0, 8,   2,   5,   12),
            new Config("negative_dir",    10, -40, 0, 0, 6,   3,   4,   8),
            new Config("nonzero_v0",      0, 100, 2, 0, 8,   4,   4,   10),
            new Config("nonzero_a0_pos",  0, 80,  0, 2, 8,   4,   4,   10),
            new Config("direction_reversal", 0, 50, -3, 0, 8, 2, 5, 12)
        );
    }

    // ---------------------------------------------------------------
    // Individual structural tests
    // ---------------------------------------------------------------

    @Test
    void restToRest_shortDistance_bothTriangular() {
        // With jMax=100, vMax=10, aMax=1: vAccelMin = aMax^2/jMax = 0.01
        // D_triangular(vAccelMin) = 2*(0.01)^1.5/sqrt(100) = 0.0002
        // Use pTarget=1e-4 < 0.0002 to guarantee vPeak < vAccelMin => triangular, T2=T4=T6=0
        SCurvePosition s = new SCurvePosition(0, 1e-4, 0, 0, 10, 1, 1, 100);
        assertEquals(0, s.T2, 1e-9, "T2 should be 0 (triangular accel)");
        assertEquals(0, s.T4, 1e-9, "T4 should be 0 (no cruise)");
        assertEquals(0, s.T6, 1e-9, "T6 should be 0 (triangular decel)");
        assertTrue(s.vPeak < s.aMaxAccel * s.aMaxAccel / s.jMax, "vPeak below vAccelMin");
    }

    @Test
    void restToRest_reachesVMax_withT4() {
        SCurvePosition s = new SCurvePosition(0, 200, 0, 0, 5, 3, 3, 10);
        assertEquals(5.0, s.vPeak, 1e-6, "should reach vMax");
        assertTrue(s.T4 > 0, "cruise phase should exist");
        assertEquals(200.0, s.getPosition(s.getTotalTime()), 1e-4, "end position");
    }

    @Test
    void asymmetricAccel_T1_neq_T5() {
        // Different aMaxAccel and aMaxDecel => different ramp times
        SCurvePosition s = new SCurvePosition(0, 100, 0, 0, 10, 6, 3, 12);
        assertNotEquals(s.T1, s.T5, 1e-6, "T1 (accel ramp) should differ from T5 (decel ramp)");
    }

    @Test
    void negative_direction_mirrored() {
        SCurvePosition s = new SCurvePosition(50, -50, 0, 0, 8, 4, 4, 10);
        double tf = s.getTotalTime();
        assertTrue(tf > 0);
        assertEquals(-50.0, s.getPosition(tf), 1e-4, "should arrive at pTarget");
        assertEquals(0.0, s.getVelocity(tf), 1e-4, "should come to rest");
    }

    @Test
    void trivial_zeroDistance() {
        SCurvePosition s = new SCurvePosition(5, 5, 2, 1, 8, 4, 4, 10);
        assertEquals(0, s.getTotalTime(), "trivial: zero total time");
        assertEquals(5.0, s.getPosition(0));
        assertEquals(5.0, s.getPosition(1));
        assertDoesNotThrow(() -> s.getVelocity(0));
    }

    @Test
    void nonZeroV0_forward_endConditionsMet() {
        // Start already moving toward target
        SCurvePosition s = new SCurvePosition(0, 100, 3, 0, 8, 4, 4, 10);
        double tf = s.getTotalTime();
        assertEquals(100.0, s.getPosition(tf), 1e-3);
        assertEquals(0.0,   s.getVelocity(tf),  1e-3);
        assertEquals(0.0,   s.getAcceleration(tf), 1e-3);
    }

    @Test
    void nonZeroA0_positive_endConditionsMet() {
        // Start with non-zero positive acceleration
        SCurvePosition s = new SCurvePosition(0, 80, 0, 2, 8, 4, 4, 10);
        double tf = s.getTotalTime();
        assertEquals(80.0, s.getPosition(tf), 1e-3);
        assertEquals(0.0,  s.getVelocity(tf),  1e-3);
        assertEquals(0.0,  s.getAcceleration(tf), 1e-3);
    }

    // ---------------------------------------------------------------
    // Parametric: initial conditions
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void initialConditions(Config c) {
        SCurvePosition s = make(c);
        assertEquals(c.p0(), s.getPosition(0),     1e-9, "p(0) == p0");
        assertEquals(c.v0(), s.getVelocity(0),     1e-9, "v(0) == v0");
        assertEquals(c.a0(), s.getAcceleration(0), 1e-9, "a(0) == a0");
    }

    // ---------------------------------------------------------------
    // Parametric: end conditions
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void endConditions(Config c) {
        SCurvePosition s = make(c);
        double tf = s.getTotalTime();
        assertEquals(c.pTarget(), s.getPosition(tf),     1e-3, "p(tf) == pTarget");
        assertEquals(0.0,         s.getVelocity(tf),     1e-3, "v(tf) == 0");
        assertEquals(0.0,         s.getAcceleration(tf), 1e-3, "a(tf) == 0");
    }

    // ---------------------------------------------------------------
    // Parametric: kinematic continuity (v ≈ dp/dt, a ≈ dv/dt)
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void kinematicContinuity_velocity(Config c) {
        SCurvePosition s = make(c);
        double tf = s.getTotalTime();
        if (tf < 1e-9) return; // trivial, skip
        double h = 1e-5;
        int samples = 100;
        // Loop over interior samples only (i=1..99) so t±h stays safely inside (0, tf)
        for (int i = 1; i < samples; i++) {
            double t = tf * i / samples;
            double dpdt = (s.getPosition(t + h) - s.getPosition(t - h)) / (2 * h);
            double v    = s.getVelocity(t);
            assertEquals(dpdt, v, 1e-4,
                    c.label() + " velocity mismatch at t=" + t);
        }
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void kinematicContinuity_acceleration(Config c) {
        SCurvePosition s = make(c);
        double tf = s.getTotalTime();
        if (tf < 1e-9) return;
        double h = 1e-5;
        int samples = 100;
        // Loop over interior samples only (i=1..99) so t±h stays safely inside (0, tf)
        for (int i = 1; i < samples; i++) {
            double t = tf * i / samples;
            double dvdt = (s.getVelocity(t + h) - s.getVelocity(t - h)) / (2 * h);
            double a    = s.getAcceleration(t);
            assertEquals(dvdt, a, 1e-3,
                    c.label() + " acceleration mismatch at t=" + t);
        }
    }

    @Test
    void brakingPrefix_respectsJerkLimit() {
        // v0=-3 means wrong-way velocity; braking prefix must use jerk-limited decel
        SCurvePosition s = new SCurvePosition(0, 50, -3, 0, 8, 2, 5, 12);
        double jMax = s.jMax;
        double tBrakeEnd = s.tPrefix + s.tBrake;
        assertTrue(tBrakeEnd > 0, "expected non-trivial brake prefix");

        double h = 1e-6;
        int samples = 1000;
        for (int i = 1; i < samples; i++) {
            double t = tBrakeEnd * i / samples;
            double dadt = (s.getAcceleration(t + h) - s.getAcceleration(t - h)) / (2 * h);
            assertTrue(Math.abs(dadt) <= jMax + 1.0,
                    "jerk |da/dt|=" + dadt + " exceeds jMax=" + jMax + " at t=" + t);
        }
    }

    @Test
    void reversalAcceleration_noDipToZero() {
        SCurvePosition s = new SCurvePosition(0, 50, -3, 0, 8, 2, 5, 12);
        double tBrakeEnd = s.tPrefix + s.tBrake;
        assertTrue(tBrakeEnd > 0, "expected braking prefix");
        // Acceleration at the brake→main handoff must be positive (no zero dip)
        double aAtHandoff = s.getAcceleration(tBrakeEnd);
        assertTrue(aAtHandoff > 1e-6,
            "expected positive acceleration at handoff, got " + aAtHandoff);
        // Acceleration must stay non-negative through the entire T1 ramp
        double tPhase1End = tBrakeEnd + s.T1;
        for (int i = 0; i <= 200; i++) {
            double t = tBrakeEnd + (tPhase1End - tBrakeEnd) * i / 200.0;
            assertTrue(s.getAcceleration(t) > -1e-6,
                "acceleration dip at t=" + t);
        }
    }

    @Test
    void reversalWithHelpfulBrakingAcceleration_keepsBrakingAtLimit() {
        SCurvePosition s = new SCurvePosition(-95, 100, -5, 5, 10, 5, 5, 50);

        for (double t : new double[] {0.1, 0.3, 0.5, 0.7, 0.9}) {
            assertEquals(
                    5.0,
                    s.getAcceleration(t),
                    1e-6,
                    "should keep max helpful braking acceleration at t=" + t);
        }

        assertEquals(-97.5, s.getPosition(1.0), 1e-3, "should stop with minimum overshoot");
        assertEquals(0.0, s.getVelocity(1.0), 1e-3, "should be stopped after 1 second");
    }

    // ---------------------------------------------------------------
    // Helper
    // ---------------------------------------------------------------

    private static SCurvePosition make(Config c) {
        return new SCurvePosition(c.p0(), c.pTarget(), c.v0(), c.a0(),
                                  c.vMax(), c.aMaxAccel(), c.aMaxDecel(), c.jMax());
    }
}
