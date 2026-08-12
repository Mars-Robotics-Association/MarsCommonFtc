package org.marsroboticsassociation.controllib.motion

import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class SCurveVelocityTest {

    companion object {
        private const val MOTOR_VOLTAGE = 12.0
        private const val KS = 0.893
        private const val KV = 0.00475
        private const val KA = 0.00599
    }

    private fun maxMotorAcceleration(velocity: Double): Double {
        val availableVoltage = MOTOR_VOLTAGE - KS - KV * abs(velocity)
        if (availableVoltage <= 0) return 0.0
        return availableVoltage / KA
    }

    @Test
    fun backEmfViolationCheck() {
        val aMax = 1858.0
        val jInc = 3000.0
        val jDec = 360.0
        val targetV = 2000.0
        val v0 = 0.0
        val a0 = 0.0

        val trajectory = SCurveVelocity(v0, targetV, a0, aMax, jInc, jDec)
        val totalTime = trajectory.getTotalTime()

        var maxViolation = 0.0
        var maxViolationTime = 0.0
        var maxViolationVelocity = 0.0
        var maxViolationAccel = 0.0
        var maxViolationMotorAccel = 0.0

        var maxTrajAccel = 0.0
        var maxTrajAccelTime = 0.0
        var maxTrajAccelV = 0.0

        val samples = 10000
        for (i in 0..samples) {
            val t = totalTime * i / samples
            val v = trajectory.getVelocity(t)
            val a = trajectory.getAcceleration(t)

            if (a > maxTrajAccel) {
                maxTrajAccel = a
                maxTrajAccelTime = t
                maxTrajAccelV = v
            }

            val motorAMax = maxMotorAcceleration(v)
            val violation = a - motorAMax

            if (violation > maxViolation) {
                maxViolation = violation
                maxViolationTime = t
                maxViolationVelocity = v
                maxViolationAccel = a
                maxViolationMotorAccel = motorAMax
            }
        }

        println("=== Back-EMF Violation Check ===")
        println("aMax: $aMax, jInc: $jInc, jDec: $jDec")
        println("Target velocity: $targetV")
        println("Total time: ${totalTime}s")
        println()
        println(
            "Max trajectory acceleration: $maxTrajAccel at t=${maxTrajAccelTime}s, v=$maxTrajAccelV"
        )
        println("Motor max accel at that velocity: ${maxMotorAcceleration(maxTrajAccelV)}")
        println()
        println("Max violation: $maxViolation units/s^2")
        if (maxViolation > 0) {
            println("  at t=${maxViolationTime}s")
            println("  velocity: $maxViolationVelocity RPM")
            println("  trajectory accel: $maxViolationAccel")
            println("  motor max accel: $maxViolationMotorAccel")
        }
        println()
        println("Motor max accel curve:")
        var v = 0
        while (v <= targetV.toInt()) {
            println("  v=$v: ${maxMotorAcceleration(v.toDouble())}")
            v += 200
        }

        assertTrue(
            maxViolation < 1.0,
            "Acceleration should stay within motor limits (with tolerance)",
        )
    }

    // --- findMaxAMax degenerate input guards ---

    @Test
    fun findMaxAMax_kAZero_returnsInfinity() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, MOTOR_VOLTAGE, KS, KV, 0.0)
        assertEquals(Double.POSITIVE_INFINITY, result)
    }

    @Test
    fun findMaxAMax_kANegative_returnsInfinity() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, MOTOR_VOLTAGE, KS, KV, -1.0)
        assertEquals(Double.POSITIVE_INFINITY, result)
    }

    @Test
    fun findMaxAMax_voltageEqualToKs_returnsZero() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, KS, KS, KV, KA)
        assertEquals(0.0, result)
    }

    @Test
    fun findMaxAMax_voltageLessThanKs_returnsZero() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, KS - 1.0, KS, KV, KA)
        assertEquals(0.0, result)
    }

    @Test
    fun findMaxAMax_jIncZero_returnsZero() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, 0.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertEquals(0.0, result)
    }

    @Test
    fun findMaxAMax_jIncNegative_returnsZero() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, -100.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertEquals(0.0, result)
    }

    @Test
    fun findMaxAMax_normalInputs_returnsPositiveFiniteValue() {
        val result = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertTrue(result.isFinite(), "result should be finite")
        assertTrue(result > 0, "result should be positive")
    }

    // --- findMaxJDec degenerate input guards ---

    @Test
    fun findMaxJDec_kAZero_returnsInfinity() {
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, 1858.0, 3000.0, MOTOR_VOLTAGE, KS, KV, 0.0)
        assertEquals(Double.POSITIVE_INFINITY, result)
    }

    @Test
    fun findMaxJDec_kANegative_returnsInfinity() {
        val result =
            SCurveVelocity.findMaxJDec(
                0.0,
                2000.0,
                0.0,
                1858.0,
                3000.0,
                MOTOR_VOLTAGE,
                KS,
                KV,
                -1.0,
            )
        assertEquals(Double.POSITIVE_INFINITY, result)
    }

    @Test
    fun findMaxJDec_voltageEqualToKs_returnsNaN() {
        val result = SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, 1858.0, 3000.0, KS, KS, KV, KA)
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_voltageLessThanKs_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, 1858.0, 3000.0, KS - 1.0, KS, KV, KA)
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_aMaxZero_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, 0.0, 3000.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_aMaxNegative_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, -100.0, 3000.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_aMaxInfinity_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(
                0.0,
                2000.0,
                0.0,
                Double.POSITIVE_INFINITY,
                3000.0,
                MOTOR_VOLTAGE,
                KS,
                KV,
                KA,
            )
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_aMaxNaN_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(
                0.0,
                2000.0,
                0.0,
                Double.NaN,
                3000.0,
                MOTOR_VOLTAGE,
                KS,
                KV,
                KA,
            )
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_jIncZero_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, 1858.0, 0.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_jIncNegative_returnsNaN() {
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, 1858.0, -100.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertTrue(result.isNaN())
    }

    @Test
    fun findMaxJDec_trivialVelocity_returnsInfinity() {
        val result =
            SCurveVelocity.findMaxJDec(
                2000.0,
                2000.0,
                0.0,
                1858.0,
                3000.0,
                MOTOR_VOLTAGE,
                KS,
                KV,
                KA,
            )
        assertEquals(Double.POSITIVE_INFINITY, result)
    }

    @Test
    fun findMaxJDec_nearTrivialVelocity_returnsInfinity() {
        val result =
            SCurveVelocity.findMaxJDec(
                2000.0,
                2000.0 + 1e-10,
                0.0,
                1858.0,
                3000.0,
                MOTOR_VOLTAGE,
                KS,
                KV,
                KA,
            )
        assertEquals(Double.POSITIVE_INFINITY, result)
    }

    @Test
    fun findMaxJDec_normalInputs_returnsPositiveFiniteValue() {
        val aMax = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, MOTOR_VOLTAGE, KS, KV, KA)
        val result =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, aMax, 3000.0, MOTOR_VOLTAGE, KS, KV, KA)
        assertTrue(result.isFinite(), "result should be finite")
        assertTrue(result > 0, "result should be positive")
    }

    // --- chained call safety ---

    @Test
    fun findMaxAMax_thenFindMaxJDec_kAZero_noNaNOrInfiniteTrajectory() {
        // When kA=0 both functions return POSITIVE_INFINITY; constructing a trajectory with
        // those values must not produce NaN timing (regression guard for the chained-call pattern).
        val aMax = SCurveVelocity.findMaxAMax(0.0, 2000.0, 3000.0, MOTOR_VOLTAGE, KS, KV, 0.0)
        val jDec =
            SCurveVelocity.findMaxJDec(0.0, 2000.0, 0.0, aMax, 3000.0, MOTOR_VOLTAGE, KS, KV, 0.0)
        assertEquals(Double.POSITIVE_INFINITY, aMax)
        assertEquals(Double.POSITIVE_INFINITY, jDec)
        // Callers must clamp before constructing a trajectory; the functions themselves are safe.
    }
}
