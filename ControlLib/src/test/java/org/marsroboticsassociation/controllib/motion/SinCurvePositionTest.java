package org.marsroboticsassociation.controllib.motion;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;

import java.util.stream.Stream;

class SinCurvePositionTest {

    // ---------------------------------------------------------------
    // Test configurations (mirrors SCurvePositionTest)
    // ---------------------------------------------------------------

    record Config(
            String label,
            double p0,
            double pTarget,
            double v0,
            double a0,
            double vMax,
            double aMaxAccel,
            double aMaxDecel,
            double jMax) {}

    static Stream<Config> allConfigs() {
        return Stream.of(
                new Config("sym_short", 0, 1, 0, 0, 5, 3, 3, 10),
                new Config("no_cruise_sym", 0, 0.2, 0, 0, 5, 3, 3, 10),
                new Config("no_cruise_asym", 0, 0.2, 0, 0, 5, 4, 2, 10),
                new Config("reversal_no_cruise", 0, 0.75, -1.5, 0, 5, 2, 4, 10),
                new Config("sym_long", 0, 100, 0, 0, 5, 3, 3, 10),
                new Config("asym_accel", 0, 50, 0, 0, 8, 5, 2, 12),
                new Config("asym_decel", 0, 50, 0, 0, 8, 2, 5, 12),
                new Config("negative_dir", 10, -40, 0, 0, 6, 3, 4, 8),
                new Config("nonzero_v0", 0, 100, 2, 0, 8, 4, 4, 10),
                new Config("nonzero_a0_pos", 0, 80, 0, 2, 8, 4, 4, 10),
                new Config("direction_reversal", 0, 50, -3, 0, 8, 2, 5, 12));
    }

    // ---------------------------------------------------------------
    // Individual structural tests
    // ---------------------------------------------------------------

    @Test
    void restToRest_shortDistance_triangularProfile() {
        // Very small distance: vPeak < aMax^2/jMax => triangular, T2=T4=T6=0
        SinCurvePosition s = new SinCurvePosition(0, 1e-4, 0, 0, 10, 1, 1, 100);
        assertEquals(0, s.T2, 1e-9, "T2 should be 0 (triangular accel)");
        assertEquals(0, s.T4, 1e-9, "T4 should be 0 (no cruise)");
        assertEquals(0, s.T6, 1e-9, "T6 should be 0 (triangular decel)");
        assertTrue(s.vPeak < s.aMaxAccel * s.aMaxAccel / s.jMax, "vPeak below vAccelMin");
    }

    @Test
    void restToRest_reachesVMax_withT4() {
        SinCurvePosition s = new SinCurvePosition(0, 200, 0, 0, 5, 3, 3, 10);
        assertEquals(5.0, s.vPeak, 1e-6, "should reach vMax");
        assertTrue(s.T4 > 0, "cruise phase should exist");
        assertEquals(200.0, s.getPosition(s.getTotalTime()), 1e-4, "end position");
    }

    @Test
    void asymmetricAccel_T1_neq_T5() {
        SinCurvePosition s = new SinCurvePosition(0, 100, 0, 0, 10, 6, 3, 12);
        assertNotEquals(s.T1, s.T5, 1e-6, "T1 (accel ramp) should differ from T5 (decel ramp)");
    }

    @Test
    void negative_direction_mirrored() {
        SinCurvePosition s = new SinCurvePosition(50, -50, 0, 0, 8, 4, 4, 10);
        double tf = s.getTotalTime();
        assertTrue(tf > 0);
        assertEquals(-50.0, s.getPosition(tf), 1e-4, "should arrive at pTarget");
        assertEquals(0.0, s.getVelocity(tf), 1e-4, "should come to rest");
    }

    @Test
    void trivial_zeroDistance() {
        SinCurvePosition s = new SinCurvePosition(5, 5, 2, 1, 8, 4, 4, 10);
        assertEquals(0, s.getTotalTime(), "trivial: zero total time");
        assertEquals(5.0, s.getPosition(0));
        assertEquals(5.0, s.getPosition(1));
        assertDoesNotThrow(() -> s.getVelocity(0));
    }

    @Test
    void nonZeroV0_forward_endConditionsMet() {
        SinCurvePosition s = new SinCurvePosition(0, 100, 3, 0, 8, 4, 4, 10);
        double tf = s.getTotalTime();
        assertEquals(100.0, s.getPosition(tf), 1e-3);
        assertEquals(0.0, s.getVelocity(tf), 1e-3);
        assertEquals(0.0, s.getAcceleration(tf), 1e-3);
    }

    @Test
    void nonZeroA0_positive_endConditionsMet() {
        SinCurvePosition s = new SinCurvePosition(0, 80, 0, 2, 8, 4, 4, 10);
        double tf = s.getTotalTime();
        assertEquals(80.0, s.getPosition(tf), 1e-3);
        assertEquals(0.0, s.getVelocity(tf), 1e-3);
        assertEquals(0.0, s.getAcceleration(tf), 1e-3);
    }

    // ---------------------------------------------------------------
    // Parametric: initial conditions
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void initialConditions(Config c) {
        SinCurvePosition s = make(c);
        assertEquals(c.p0(), s.getPosition(0), 1e-9, "p(0) == p0");
        assertEquals(c.v0(), s.getVelocity(0), 1e-9, "v(0) == v0");
        assertEquals(c.a0(), s.getAcceleration(0), 1e-9, "a(0) == a0");
    }

    // ---------------------------------------------------------------
    // Parametric: end conditions
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void endConditions(Config c) {
        SinCurvePosition s = make(c);
        double tf = s.getTotalTime();
        assertEquals(c.pTarget(), s.getPosition(tf), 1e-3, "p(tf) == pTarget");
        assertEquals(0.0, s.getVelocity(tf), 1e-3, "v(tf) == 0");
        assertEquals(0.0, s.getAcceleration(tf), 1e-3, "a(tf) == 0");
    }

    // ---------------------------------------------------------------
    // Parametric: kinematic continuity (v ≈ dp/dt, a ≈ dv/dt)
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void kinematicContinuity_velocity(Config c) {
        SinCurvePosition s = make(c);
        double tf = s.getTotalTime();
        if (tf < 1e-9) return;
        double h = 1e-5;
        int samples = 100;
        for (int i = 1; i < samples; i++) {
            double t = tf * i / samples;
            double dpdt = (s.getPosition(t + h) - s.getPosition(t - h)) / (2 * h);
            assertEquals(dpdt, s.getVelocity(t), 1e-4, c.label() + " velocity mismatch at t=" + t);
        }
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void kinematicContinuity_acceleration(Config c) {
        SinCurvePosition s = make(c);
        double tf = s.getTotalTime();
        if (tf < 1e-9) return;
        double h = 1e-5;
        int samples = 100;
        for (int i = 1; i < samples; i++) {
            double t = tf * i / samples;
            double dvdt = (s.getVelocity(t + h) - s.getVelocity(t - h)) / (2 * h);
            assertEquals(
                    dvdt,
                    s.getAcceleration(t),
                    1e-3,
                    c.label() + " acceleration mismatch at t=" + t);
        }
    }

    // ---------------------------------------------------------------
    // Sinusoidal-specific: smooth acceleration across phase boundaries
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void smoothAcceleration_noPhaseBoundaryJump(Config c) {
        SinCurvePosition s = make(c);
        double tf = s.getTotalTime();
        if (tf < 1e-9) return;
        // Sample at high density and verify acceleration never jumps discontinuously.
        // Since acceleration is continuous for sinusoidal profiles, |a(t+h) - a(t-h)| ~ h*jerk.
        // With h=1e-4, a continuous function changes by at most ~peak_jerk * 2e-4 ≈ 2 (generous).
        double h = 1e-4;
        int samples = 500;
        for (int i = 1; i < samples; i++) {
            double t = tf * i / samples;
            double aBefore = s.getAcceleration(t - h);
            double aAfter = s.getAcceleration(t + h);
            assertEquals(aBefore, aAfter, 2.0, c.label() + " acceleration jump at t=" + t);
        }
    }

    // ---------------------------------------------------------------
    // Sinusoidal-specific: acceleration bounded by aMaxAccel / aMaxDecel
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    void accelBounded_neverExceedsAMax(Config c) {
        SinCurvePosition s = make(c);
        double tf = s.getTotalTime();
        if (tf < 1e-9) return;
        int samples = 1000;
        double bound = Math.max(c.aMaxAccel(), c.aMaxDecel()) + 1e-6;
        for (int i = 0; i <= samples; i++) {
            double t = tf * i / samples;
            assertTrue(
                    Math.abs(s.getAcceleration(t)) <= bound,
                    c.label() + " |a| exceeds max at t=" + t + ": " + s.getAcceleration(t));
        }
    }

    // ---------------------------------------------------------------
    // Braking prefix tests (mirrors SCurvePositionTest)
    // ---------------------------------------------------------------

    @Test
    void caseB_handoff_endConditionsContinuous() {
        // direction_reversal (v0=-3) triggers Case B: braking and main accel are in the same
        // direction, so the handoff arc replaces braking-offset + T1-onset.
        SinCurvePosition s = new SinCurvePosition(0, 50, -3, 0, 8, 2, 5, 12);
        assertTrue(s.handoffCombined, "expected Case B handoff for direction_reversal");
        double tf = s.getTotalTime();
        // Verify velocity and position reach their endpoints CONTINUOUSLY (not via endpoint clamp).
        // With the T2 bug, velocity near the end was ~0.7 instead of 0.
        assertEquals(0.0, s.getVelocity(tf - 1e-6), 1e-3, "v should continuously reach 0");
        assertEquals(50.0, s.getPosition(tf - 1e-6), 1e-3, "p should continuously reach pTarget");
    }

    @Test
    void brakingPrefix_smoothDeceleration() {
        // v0=-3: wrong-way velocity, braking prefix fires
        SinCurvePosition s = new SinCurvePosition(0, 50, -3, 0, 8, 2, 5, 12);
        double tBrakeEnd = s.tPrefix + s.tBrake;
        assertTrue(tBrakeEnd > 0, "expected non-trivial brake prefix");

        // For a sinusoidal profile, acceleration should be continuous throughout braking.
        double h = 1e-6;
        double maxJump = 1.0; // generous: |a(t+h) - a(t-h)| < 1 for any smooth profile
        for (int i = 1; i < 1000; i++) {
            double t = tBrakeEnd * i / 1000;
            double jump = Math.abs(s.getAcceleration(t + h) - s.getAcceleration(t - h));
            assertTrue(jump < maxJump, "acceleration jump " + jump + " at t=" + t);
        }
    }

    @Test
    void brakingPrefix_velocityReachesZero() {
        SinCurvePosition s = new SinCurvePosition(0, 50, -3, 0, 8, 2, 5, 12);
        double tBrakeEnd = s.tPrefix + s.tBrake;
        assertTrue(tBrakeEnd > 0, "expected braking prefix");
        // At end of braking, velocity should always be ~0
        assertEquals(0.0, s.getVelocity(tBrakeEnd), 1e-6, "velocity at brake end should be 0");
        // When Case B (handoff) is active, braking ends at a = brkAmpl (the handoff start value),
        // not 0. Only assert a=0 when no handoff is present.
        if (!s.handoffCombined) {
            assertEquals(
                    0.0,
                    s.getAcceleration(tBrakeEnd),
                    1e-6,
                    "acceleration at brake end should be 0");
        }
    }

    @Test
    void a0prefix_smoothTransition() {
        // a0=2: prefix fires to bring acceleration from 2 to 0
        SinCurvePosition s = new SinCurvePosition(0, 80, 0, 2, 8, 4, 4, 10);
        assertTrue(s.tPrefix > 0, "expected a0 prefix");
        // Acceleration at end of prefix should be ~0
        assertEquals(0.0, s.getAcceleration(s.tPrefix), 1e-9, "a at end of prefix should be 0");
        // Acceleration at start should match a0
        assertEquals(2.0, s.getAcceleration(0), 1e-9, "a(0) should match a0");
    }

    @Test
    void noCruiseSymmetric_usesCombinedMidpointArc() {
        SinCurvePosition s = new SinCurvePosition(0, 0.2, 0, 0, 5, 3, 3, 10);
        assertEquals(0.0, s.T4, 1e-9, "expected no cruise");
        assertTrue(s.midpointCombined, "expected midpoint arc to be combined");
        assertEquals(0.0, s.T5, 1e-9, "T5 should be absorbed into the midpoint arc");
        assertTrue(s.T3 > 0, "combined midpoint arc should occupy T3");
    }

    @Test
    void noCruiseReversal_combinesHandoffAndMidpoint() {
        SinCurvePosition s = new SinCurvePosition(0, 0.75, -1.5, 0, 5, 2, 4, 10);
        assertEquals(0.0, s.T4, 1e-9, "expected no cruise");
        assertTrue(s.handoffCombined, "expected Case B handoff");
        assertTrue(s.midpointCombined, "expected midpoint arc to be combined");
        assertEquals(0.0, s.T5, 1e-9, "T5 should be absorbed into the midpoint arc");
        assertEquals(0.0, s.getVelocity(s.getTotalTime() - 1e-6), 1e-3, "velocity should end continuously");
        assertEquals(0.75, s.getPosition(s.getTotalTime() - 1e-6), 1e-3, "position should end continuously");
    }

    @Test
    void reversalWithHelpfulBrakingAcceleration_doesNotAddExtraOvershoot() {
        SinCurvePosition s = new SinCurvePosition(-95, 100, -5, 5, 10, 5, 5, 50);

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

    @Test
    void smallWrongWaySpeed_withHelpfulAcceleration_keepsBrakingInsteadOfBleedingToZero() {
        SinCurvePosition s = new SinCurvePosition(0, 50, -0.6, 1.0, 8, 4, 4, 10);

        assertTrue(s.tBrake > 0, "expected braking prefix");
        assertTrue(
                s.getAcceleration(0.02) > 1.0,
                "helpful braking acceleration should keep building early in the replan");
        assertTrue(
                s.getAcceleration(s.tPrefix + 1e-6) > 1.0,
                "merged braking onset should not dip back to zero before stopping");
        assertEquals(
                0.0,
                s.getVelocity(s.tPrefix + s.tBrake),
                1e-6,
                "wrong-way velocity should still be cancelled exactly");
    }

    // ---------------------------------------------------------------
    // sinHalfDist helper tests
    // ---------------------------------------------------------------

    @Test
    void sinHalfDist_zeroOrNegativeDv_returnsZero() {
        assertEquals(0, SinCurvePosition.sinHalfDist(5, 5, 3, 10), 1e-12);
        assertEquals(0, SinCurvePosition.sinHalfDist(5, 3, 3, 10), 1e-12);
    }

    @Test
    void sinHalfDist_triangular_velocityGainCorrect() {
        // Very small dv: triangular. Use aMax=1, jMax=100 => vMin = 0.01.
        // dv = 0.001 < vMin: triangular.
        double v0 = 0, vPeak = 0.001, aMax = 1, jMax = 100;
        double d = SinCurvePosition.sinHalfDist(v0, vPeak, aMax, jMax);
        assertTrue(d > 0);
        // Rough check: distance ≈ average_velocity * total_time, avg_v ≈ vPeak/2,
        // total_time = 2 * sqrt(vPeak/jMax)
        double T = Math.sqrt(vPeak / jMax);
        double expectedApprox = (vPeak / 2.0) * 2 * T;
        assertEquals(expectedApprox, d, expectedApprox * 0.5); // within 50%, just a sanity check
    }

    @Test
    void sinHalfDist_trapezoidal_largerThanTriangular() {
        // Same params, larger dv should give longer distance
        double d_tri = SinCurvePosition.sinHalfDist(0, 0.1, 1, 10); // triangular (dv<vMin=0.1)
        double d_trap = SinCurvePosition.sinHalfDist(0, 0.5, 1, 10); // trapezoidal (dv>vMin=0.1)
        assertTrue(d_trap > d_tri);
    }

    // ---------------------------------------------------------------
    // Manager integration: SinCurvePosition as factory
    // ---------------------------------------------------------------

    @Test
    void managerWithSinCurveFactory_reachesTarget() {
        long[] clock = {0};
        org.marsroboticsassociation.controllib.util.TelemetryAddData noop = (key, fmt, args) -> {};
        PositionTrajectoryManager mgr =
                new PositionTrajectoryManager(
                        5, 3, 3, 10, 0.01, noop, () -> clock[0], SinCurvePosition::new);

        mgr.setTarget(100);
        // Advance to well past total time (estimate ~30 s is more than enough)
        clock[0] = (long) (30e9);
        mgr.update();
        assertEquals(100.0, mgr.getPosition(), 1e-3);
        assertEquals(0.0, mgr.getVelocity(), 1e-3);
    }

    // ---------------------------------------------------------------
    // Helper
    // ---------------------------------------------------------------

    private static SinCurvePosition make(Config c) {
        return new SinCurvePosition(
                c.p0(),
                c.pTarget(),
                c.v0(),
                c.a0(),
                c.vMax(),
                c.aMaxAccel(),
                c.aMaxDecel(),
                c.jMax());
    }
}
