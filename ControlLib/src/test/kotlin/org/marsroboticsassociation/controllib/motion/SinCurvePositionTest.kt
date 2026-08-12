package org.marsroboticsassociation.controllib.motion

import java.util.stream.Stream
import kotlin.math.abs
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertDoesNotThrow
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.MethodSource
import org.marsroboticsassociation.controllib.util.TelemetryAddData

class SinCurvePositionTest {

    // ---------------------------------------------------------------
    // Test configurations (mirrors SCurvePositionTest)
    // ---------------------------------------------------------------

    data class Config(
        val label: String,
        val p0: Double,
        val pTarget: Double,
        val v0: Double,
        val a0: Double,
        val vMax: Double,
        val aMaxAccel: Double,
        val aMaxDecel: Double,
        val jMax: Double,
    ) {
        override fun toString(): String = label
    }

    companion object {
        @JvmStatic
        fun allConfigs(): Stream<Config> =
            Stream.of(
                Config("sym_short", 0.0, 1.0, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0),
                Config("no_cruise_sym", 0.0, 0.2, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0),
                Config("no_cruise_asym", 0.0, 0.2, 0.0, 0.0, 5.0, 4.0, 2.0, 10.0),
                Config("reversal_no_cruise", 0.0, 0.75, -1.5, 0.0, 5.0, 2.0, 4.0, 10.0),
                Config("sym_long", 0.0, 100.0, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0),
                Config("asym_accel", 0.0, 50.0, 0.0, 0.0, 8.0, 5.0, 2.0, 12.0),
                Config("asym_decel", 0.0, 50.0, 0.0, 0.0, 8.0, 2.0, 5.0, 12.0),
                Config("negative_dir", 10.0, -40.0, 0.0, 0.0, 6.0, 3.0, 4.0, 8.0),
                Config("nonzero_v0", 0.0, 100.0, 2.0, 0.0, 8.0, 4.0, 4.0, 10.0),
                Config("nonzero_a0_pos", 0.0, 80.0, 0.0, 2.0, 8.0, 4.0, 4.0, 10.0),
                Config("direction_reversal", 0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0),
            )

        private fun make(c: Config): SinCurvePosition =
            SinCurvePosition(
                c.p0,
                c.pTarget,
                c.v0,
                c.a0,
                c.vMax,
                c.aMaxAccel,
                c.aMaxDecel,
                c.jMax,
            )
    }

    // ---------------------------------------------------------------
    // Individual structural tests
    // ---------------------------------------------------------------

    @Test
    fun restToRest_shortDistance_triangularProfile() {
        // Very small distance: vPeak < aMax^2/jMax => triangular, T2=T4=T6=0
        val s = SinCurvePosition(0.0, 1e-4, 0.0, 0.0, 10.0, 1.0, 1.0, 100.0)
        assertEquals(0.0, s.T2, 1e-9, "T2 should be 0 (triangular accel)")
        assertEquals(0.0, s.T4, 1e-9, "T4 should be 0 (no cruise)")
        assertEquals(0.0, s.T6, 1e-9, "T6 should be 0 (triangular decel)")
        assertTrue(s.vPeak < s.aMaxAccel * s.aMaxAccel / s.jMax, "vPeak below vAccelMin")
    }

    @Test
    fun restToRest_reachesVMax_withT4() {
        val s = SinCurvePosition(0.0, 200.0, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0)
        assertEquals(5.0, s.vPeak, 1e-6, "should reach vMax")
        assertTrue(s.T4 > 0, "cruise phase should exist")
        assertEquals(200.0, s.getPosition(s.getTotalTime()), 1e-4, "end position")
    }

    @Test
    fun asymmetricAccel_T1_neq_T5() {
        val s = SinCurvePosition(0.0, 100.0, 0.0, 0.0, 10.0, 6.0, 3.0, 12.0)
        assertNotEquals(s.T1, s.T5, 1e-6, "T1 (accel ramp) should differ from T5 (decel ramp)")
    }

    @Test
    fun negative_direction_mirrored() {
        val s = SinCurvePosition(50.0, -50.0, 0.0, 0.0, 8.0, 4.0, 4.0, 10.0)
        val tf = s.getTotalTime()
        assertTrue(tf > 0)
        assertEquals(-50.0, s.getPosition(tf), 1e-4, "should arrive at pTarget")
        assertEquals(0.0, s.getVelocity(tf), 1e-4, "should come to rest")
    }

    @Test
    fun trivial_zeroDistance() {
        val s = SinCurvePosition(5.0, 5.0, 2.0, 1.0, 8.0, 4.0, 4.0, 10.0)
        assertEquals(0.0, s.getTotalTime(), "trivial: zero total time")
        assertEquals(5.0, s.getPosition(0.0))
        assertEquals(5.0, s.getPosition(1.0))
        assertDoesNotThrow { s.getVelocity(0.0) }
    }

    @Test
    fun nonZeroV0_forward_endConditionsMet() {
        val s = SinCurvePosition(0.0, 100.0, 3.0, 0.0, 8.0, 4.0, 4.0, 10.0)
        val tf = s.getTotalTime()
        assertEquals(100.0, s.getPosition(tf), 1e-3)
        assertEquals(0.0, s.getVelocity(tf), 1e-3)
        assertEquals(0.0, s.getAcceleration(tf), 1e-3)
    }

    @Test
    fun nonZeroA0_positive_endConditionsMet() {
        val s = SinCurvePosition(0.0, 80.0, 0.0, 2.0, 8.0, 4.0, 4.0, 10.0)
        val tf = s.getTotalTime()
        assertEquals(80.0, s.getPosition(tf), 1e-3)
        assertEquals(0.0, s.getVelocity(tf), 1e-3)
        assertEquals(0.0, s.getAcceleration(tf), 1e-3)
    }

    // ---------------------------------------------------------------
    // Parametric: initial conditions
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    fun initialConditions(c: Config) {
        val s = make(c)
        assertEquals(c.p0, s.getPosition(0.0), 1e-9, "p(0) == p0")
        assertEquals(c.v0, s.getVelocity(0.0), 1e-9, "v(0) == v0")
        assertEquals(c.a0, s.getAcceleration(0.0), 1e-9, "a(0) == a0")
    }

    // ---------------------------------------------------------------
    // Parametric: end conditions
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    fun endConditions(c: Config) {
        val s = make(c)
        val tf = s.getTotalTime()
        assertEquals(c.pTarget, s.getPosition(tf), 1e-3, "p(tf) == pTarget")
        assertEquals(0.0, s.getVelocity(tf), 1e-3, "v(tf) == 0")
        assertEquals(0.0, s.getAcceleration(tf), 1e-3, "a(tf) == 0")
    }

    // ---------------------------------------------------------------
    // Parametric: kinematic continuity (v ≈ dp/dt, a ≈ dv/dt)
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    fun kinematicContinuity_velocity(c: Config) {
        val s = make(c)
        val tf = s.getTotalTime()
        if (tf < 1e-9) return
        val h = 1e-5
        val samples = 100
        for (i in 1 until samples) {
            val t = tf * i / samples
            val dpdt = (s.getPosition(t + h) - s.getPosition(t - h)) / (2 * h)
            assertEquals(dpdt, s.getVelocity(t), 1e-4, "${c.label} velocity mismatch at t=$t")
        }
    }

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    fun kinematicContinuity_acceleration(c: Config) {
        val s = make(c)
        val tf = s.getTotalTime()
        if (tf < 1e-9) return
        val h = 1e-5
        val samples = 100
        for (i in 1 until samples) {
            val t = tf * i / samples
            val dvdt = (s.getVelocity(t + h) - s.getVelocity(t - h)) / (2 * h)
            assertEquals(
                dvdt,
                s.getAcceleration(t),
                1e-3,
                "${c.label} acceleration mismatch at t=$t",
            )
        }
    }

    // ---------------------------------------------------------------
    // Sinusoidal-specific: smooth acceleration across phase boundaries
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    fun smoothAcceleration_noPhaseBoundaryJump(c: Config) {
        val s = make(c)
        val tf = s.getTotalTime()
        if (tf < 1e-9) return
        // Sample at high density and verify acceleration never jumps discontinuously.
        // Since acceleration is continuous for sinusoidal profiles, |a(t+h) - a(t-h)| ~ h*jerk.
        // With h=1e-4, a continuous function changes by at most ~peak_jerk * 2e-4 ≈ 2 (generous).
        val h = 1e-4
        val samples = 500
        for (i in 1 until samples) {
            val t = tf * i / samples
            val aBefore = s.getAcceleration(t - h)
            val aAfter = s.getAcceleration(t + h)
            assertEquals(aBefore, aAfter, 2.0, "${c.label} acceleration jump at t=$t")
        }
    }

    // ---------------------------------------------------------------
    // Sinusoidal-specific: acceleration bounded by aMaxAccel / aMaxDecel
    // ---------------------------------------------------------------

    @ParameterizedTest(name = "{0}")
    @MethodSource("allConfigs")
    fun accelBounded_neverExceedsAMax(c: Config) {
        val s = make(c)
        val tf = s.getTotalTime()
        if (tf < 1e-9) return
        val samples = 1000
        val bound = maxOf(c.aMaxAccel, c.aMaxDecel) + 1e-6
        for (i in 0..samples) {
            val t = tf * i / samples
            assertTrue(
                abs(s.getAcceleration(t)) <= bound,
                "${c.label} |a| exceeds max at t=$t: ${s.getAcceleration(t)}",
            )
        }
    }

    // ---------------------------------------------------------------
    // Braking prefix tests (mirrors SCurvePositionTest)
    // ---------------------------------------------------------------

    @Test
    fun caseB_handoff_endConditionsContinuous() {
        // direction_reversal (v0=-3) triggers Case B: braking and main accel are in the same
        // direction, so the handoff arc replaces braking-offset + T1-onset.
        val s = SinCurvePosition(0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0)
        assertTrue(s.handoffCombined, "expected Case B handoff for direction_reversal")
        val tf = s.getTotalTime()
        // Verify velocity and position reach their endpoints CONTINUOUSLY (not via endpoint clamp).
        // With the T2 bug, velocity near the end was ~0.7 instead of 0.
        assertEquals(0.0, s.getVelocity(tf - 1e-6), 1e-3, "v should continuously reach 0")
        assertEquals(50.0, s.getPosition(tf - 1e-6), 1e-3, "p should continuously reach pTarget")
    }

    @Test
    fun brakingPrefix_smoothDeceleration() {
        // v0=-3: wrong-way velocity, braking prefix fires
        val s = SinCurvePosition(0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0)
        val tBrakeEnd = s.tPrefix + s.tBrake
        assertTrue(tBrakeEnd > 0, "expected non-trivial brake prefix")

        // For a sinusoidal profile, acceleration should be continuous throughout braking.
        val h = 1e-6
        val maxJump = 1.0 // generous: |a(t+h) - a(t-h)| < 1 for any smooth profile
        for (i in 1 until 1000) {
            val t = tBrakeEnd * i / 1000
            val jump = abs(s.getAcceleration(t + h) - s.getAcceleration(t - h))
            assertTrue(jump < maxJump, "acceleration jump $jump at t=$t")
        }
    }

    @Test
    fun brakingPrefix_velocityReachesZero() {
        val s = SinCurvePosition(0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0)
        val tBrakeEnd = s.tPrefix + s.tBrake
        assertTrue(tBrakeEnd > 0, "expected braking prefix")
        // At end of braking, velocity should always be ~0
        assertEquals(0.0, s.getVelocity(tBrakeEnd), 1e-6, "velocity at brake end should be 0")
        // When Case B (handoff) is active, braking ends at a = brkAmpl (the handoff start value),
        // not 0. Only assert a=0 when no handoff is present.
        if (!s.handoffCombined) {
            assertEquals(
                0.0,
                s.getAcceleration(tBrakeEnd),
                1e-6,
                "acceleration at brake end should be 0",
            )
        }
    }

    @Test
    fun a0prefix_smoothTransition() {
        // a0=2: prefix fires to bring acceleration from 2 to 0
        val s = SinCurvePosition(0.0, 80.0, 0.0, 2.0, 8.0, 4.0, 4.0, 10.0)
        assertTrue(s.tPrefix > 0, "expected a0 prefix")
        // Acceleration at end of prefix should be ~0
        assertEquals(0.0, s.getAcceleration(s.tPrefix), 1e-9, "a at end of prefix should be 0")
        // Acceleration at start should match a0
        assertEquals(2.0, s.getAcceleration(0.0), 1e-9, "a(0) should match a0")
    }

    @Test
    fun noCruiseSymmetric_usesCombinedMidpointArc() {
        val s = SinCurvePosition(0.0, 0.2, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0)
        assertEquals(0.0, s.T4, 1e-9, "expected no cruise")
        assertTrue(s.midpointCombined, "expected midpoint arc to be combined")
        assertEquals(0.0, s.T5, 1e-9, "T5 should be absorbed into the midpoint arc")
        assertTrue(s.T3 > 0, "combined midpoint arc should occupy T3")
    }

    @Test
    fun noCruiseReversal_combinesHandoffAndMidpoint() {
        val s = SinCurvePosition(0.0, 0.75, -1.5, 0.0, 5.0, 2.0, 4.0, 10.0)
        assertEquals(0.0, s.T4, 1e-9, "expected no cruise")
        assertTrue(s.handoffCombined, "expected Case B handoff")
        assertTrue(s.midpointCombined, "expected midpoint arc to be combined")
        assertEquals(0.0, s.T5, 1e-9, "T5 should be absorbed into the midpoint arc")
        assertEquals(
            0.0,
            s.getVelocity(s.getTotalTime() - 1e-6),
            1e-3,
            "velocity should end continuously",
        )
        assertEquals(
            0.75,
            s.getPosition(s.getTotalTime() - 1e-6),
            1e-3,
            "position should end continuously",
        )
    }

    @Test
    fun reversalWithHelpfulBrakingAcceleration_doesNotAddExtraOvershoot() {
        val s = SinCurvePosition(-95.0, 100.0, -5.0, 5.0, 10.0, 5.0, 5.0, 50.0)

        for (t in doubleArrayOf(0.1, 0.3, 0.5, 0.7, 0.9)) {
            assertEquals(
                5.0,
                s.getAcceleration(t),
                1e-6,
                "should keep max helpful braking acceleration at t=$t",
            )
        }

        assertEquals(-97.5, s.getPosition(1.0), 1e-3, "should stop with minimum overshoot")
        assertEquals(0.0, s.getVelocity(1.0), 1e-3, "should be stopped after 1 second")
    }

    @Test
    fun smallWrongWaySpeed_withHelpfulAcceleration_keepsBrakingInsteadOfBleedingToZero() {
        val s = SinCurvePosition(0.0, 50.0, -0.6, 1.0, 8.0, 4.0, 4.0, 10.0)

        assertTrue(s.tBrake > 0, "expected braking prefix")
        assertTrue(
            s.getAcceleration(0.02) > 1.0,
            "helpful braking acceleration should keep building early in the replan",
        )
        assertTrue(
            s.getAcceleration(s.tPrefix + 1e-6) > 1.0,
            "merged braking onset should not dip back to zero before stopping",
        )
        assertEquals(
            0.0,
            s.getVelocity(s.tPrefix + s.tBrake),
            1e-6,
            "wrong-way velocity should still be cancelled exactly",
        )
    }

    // ---------------------------------------------------------------
    // sinHalfDist helper tests
    // ---------------------------------------------------------------

    @Test
    fun sinHalfDist_zeroOrNegativeDv_returnsZero() {
        assertEquals(0.0, SinCurvePosition.sinHalfDist(5.0, 5.0, 3.0, 10.0), 1e-12)
        assertEquals(0.0, SinCurvePosition.sinHalfDist(5.0, 3.0, 3.0, 10.0), 1e-12)
    }

    @Test
    fun sinHalfDist_triangular_velocityGainCorrect() {
        // Very small dv: triangular. Use aMax=1, jMax=100 => vMin = 0.01.
        // dv = 0.001 < vMin: triangular.
        val v0 = 0.0
        val vPeak = 0.001
        val aMax = 1.0
        val jMax = 100.0
        val d = SinCurvePosition.sinHalfDist(v0, vPeak, aMax, jMax)
        assertTrue(d > 0)
        // Rough check: distance ≈ average_velocity * total_time, avg_v ≈ vPeak/2,
        // total_time = 2 * sqrt(vPeak/jMax)
        val T = sqrt(vPeak / jMax)
        val expectedApprox = (vPeak / 2.0) * 2 * T
        assertEquals(expectedApprox, d, expectedApprox * 0.5) // within 50%, just a sanity check
    }

    @Test
    fun sinHalfDist_trapezoidal_largerThanTriangular() {
        // Same params, larger dv should give longer distance
        val dTri = SinCurvePosition.sinHalfDist(0.0, 0.1, 1.0, 10.0) // triangular (dv<vMin=0.1)
        val dTrap = SinCurvePosition.sinHalfDist(0.0, 0.5, 1.0, 10.0) // trapezoidal (dv>vMin=0.1)
        assertTrue(dTrap > dTri)
    }

    // ---------------------------------------------------------------
    // Manager integration: SinCurvePosition as factory
    // ---------------------------------------------------------------

    @Test
    fun managerWithSinCurveFactory_reachesTarget() {
        val clock = longArrayOf(0)
        val noop = TelemetryAddData { _, _, _ -> }
        val mgr =
            PositionTrajectoryManager(
                5.0,
                3.0,
                3.0,
                10.0,
                0.01,
                noop,
                { clock[0] },
                ::SinCurvePosition,
            )

        mgr.setTarget(100.0)
        // Advance to well past total time (estimate ~30 s is more than enough)
        clock[0] = 30e9.toLong()
        mgr.update()
        assertEquals(100.0, mgr.getPosition(), 1e-3)
        assertEquals(0.0, mgr.getVelocity(), 1e-3)
    }
}
