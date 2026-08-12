package org.marsroboticsassociation.controllib.motion

import java.util.stream.Stream
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertDoesNotThrow
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.params.ParameterizedTest
import org.junit.jupiter.params.provider.MethodSource

class SCurvePositionTest {

    // ---------------------------------------------------------------
    // Test configurations for parametric tests
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
                Config("sym_long", 0.0, 100.0, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0),
                Config("asym_accel", 0.0, 50.0, 0.0, 0.0, 8.0, 5.0, 2.0, 12.0),
                Config("asym_decel", 0.0, 50.0, 0.0, 0.0, 8.0, 2.0, 5.0, 12.0),
                Config("negative_dir", 10.0, -40.0, 0.0, 0.0, 6.0, 3.0, 4.0, 8.0),
                Config("nonzero_v0", 0.0, 100.0, 2.0, 0.0, 8.0, 4.0, 4.0, 10.0),
                Config("nonzero_a0_pos", 0.0, 80.0, 0.0, 2.0, 8.0, 4.0, 4.0, 10.0),
                Config("direction_reversal", 0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0),
            )

        private fun make(c: Config): SCurvePosition =
            SCurvePosition(
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
    fun restToRest_shortDistance_bothTriangular() {
        // With jMax=100, vMax=10, aMax=1: vAccelMin = aMax^2/jMax = 0.01
        // D_triangular(vAccelMin) = 2*(0.01)^1.5/sqrt(100) = 0.0002
        // Use pTarget=1e-4 < 0.0002 to guarantee vPeak < vAccelMin => triangular, T2=T4=T6=0
        val s = SCurvePosition(0.0, 1e-4, 0.0, 0.0, 10.0, 1.0, 1.0, 100.0)
        assertEquals(0.0, s.T2, 1e-9, "T2 should be 0 (triangular accel)")
        assertEquals(0.0, s.T4, 1e-9, "T4 should be 0 (no cruise)")
        assertEquals(0.0, s.T6, 1e-9, "T6 should be 0 (triangular decel)")
        assertTrue(s.vPeak < s.aMaxAccel * s.aMaxAccel / s.jMax, "vPeak below vAccelMin")
    }

    @Test
    fun restToRest_reachesVMax_withT4() {
        val s = SCurvePosition(0.0, 200.0, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0)
        assertEquals(5.0, s.vPeak, 1e-6, "should reach vMax")
        assertTrue(s.T4 > 0, "cruise phase should exist")
        assertEquals(200.0, s.getPosition(s.getTotalTime()), 1e-4, "end position")
    }

    @Test
    fun asymmetricAccel_T1_neq_T5() {
        // Different aMaxAccel and aMaxDecel => different ramp times
        val s = SCurvePosition(0.0, 100.0, 0.0, 0.0, 10.0, 6.0, 3.0, 12.0)
        assertNotEquals(s.T1, s.T5, 1e-6, "T1 (accel ramp) should differ from T5 (decel ramp)")
    }

    @Test
    fun negative_direction_mirrored() {
        val s = SCurvePosition(50.0, -50.0, 0.0, 0.0, 8.0, 4.0, 4.0, 10.0)
        val tf = s.getTotalTime()
        assertTrue(tf > 0)
        assertEquals(-50.0, s.getPosition(tf), 1e-4, "should arrive at pTarget")
        assertEquals(0.0, s.getVelocity(tf), 1e-4, "should come to rest")
    }

    @Test
    fun trivial_zeroDistance() {
        val s = SCurvePosition(5.0, 5.0, 2.0, 1.0, 8.0, 4.0, 4.0, 10.0)
        assertEquals(0.0, s.getTotalTime(), "trivial: zero total time")
        assertEquals(5.0, s.getPosition(0.0))
        assertEquals(5.0, s.getPosition(1.0))
        assertDoesNotThrow { s.getVelocity(0.0) }
    }

    @Test
    fun nonZeroV0_forward_endConditionsMet() {
        // Start already moving toward target
        val s = SCurvePosition(0.0, 100.0, 3.0, 0.0, 8.0, 4.0, 4.0, 10.0)
        val tf = s.getTotalTime()
        assertEquals(100.0, s.getPosition(tf), 1e-3)
        assertEquals(0.0, s.getVelocity(tf), 1e-3)
        assertEquals(0.0, s.getAcceleration(tf), 1e-3)
    }

    @Test
    fun nonZeroA0_positive_endConditionsMet() {
        // Start with non-zero positive acceleration
        val s = SCurvePosition(0.0, 80.0, 0.0, 2.0, 8.0, 4.0, 4.0, 10.0)
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
        if (tf < 1e-9) return // trivial, skip
        val h = 1e-5
        val samples = 100
        // Loop over interior samples only (i=1..99) so t±h stays safely inside (0, tf)
        for (i in 1 until samples) {
            val t = tf * i / samples
            val dpdt = (s.getPosition(t + h) - s.getPosition(t - h)) / (2 * h)
            val v = s.getVelocity(t)
            assertEquals(dpdt, v, 1e-4, "${c.label} velocity mismatch at t=$t")
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
        // Loop over interior samples only (i=1..99) so t±h stays safely inside (0, tf)
        for (i in 1 until samples) {
            val t = tf * i / samples
            val dvdt = (s.getVelocity(t + h) - s.getVelocity(t - h)) / (2 * h)
            val a = s.getAcceleration(t)
            assertEquals(dvdt, a, 1e-3, "${c.label} acceleration mismatch at t=$t")
        }
    }

    @Test
    fun brakingPrefix_respectsJerkLimit() {
        // v0=-3 means wrong-way velocity; braking prefix must use jerk-limited decel
        val s = SCurvePosition(0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0)
        val jMax = s.jMax
        val tBrakeEnd = s.tPrefix + s.tBrake
        assertTrue(tBrakeEnd > 0, "expected non-trivial brake prefix")

        val h = 1e-6
        val samples = 1000
        for (i in 1 until samples) {
            val t = tBrakeEnd * i / samples
            val dadt = (s.getAcceleration(t + h) - s.getAcceleration(t - h)) / (2 * h)
            assertTrue(
                abs(dadt) <= jMax + 1.0,
                "jerk |da/dt|=$dadt exceeds jMax=$jMax at t=$t",
            )
        }
    }

    @Test
    fun reversalAcceleration_noDipToZero() {
        val s = SCurvePosition(0.0, 50.0, -3.0, 0.0, 8.0, 2.0, 5.0, 12.0)
        val tBrakeEnd = s.tPrefix + s.tBrake
        assertTrue(tBrakeEnd > 0, "expected braking prefix")
        // Acceleration at the brake→main handoff must be positive (no zero dip)
        val aAtHandoff = s.getAcceleration(tBrakeEnd)
        assertTrue(aAtHandoff > 1e-6, "expected positive acceleration at handoff, got $aAtHandoff")
        // Acceleration must stay non-negative through the entire T1 ramp
        val tPhase1End = tBrakeEnd + s.T1
        for (i in 0..200) {
            val t = tBrakeEnd + (tPhase1End - tBrakeEnd) * i / 200.0
            assertTrue(s.getAcceleration(t) > -1e-6, "acceleration dip at t=$t")
        }
    }

    @Test
    fun reversalWithHelpfulBrakingAcceleration_keepsBrakingAtLimit() {
        val s = SCurvePosition(-95.0, 100.0, -5.0, 5.0, 10.0, 5.0, 5.0, 50.0)

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
}
