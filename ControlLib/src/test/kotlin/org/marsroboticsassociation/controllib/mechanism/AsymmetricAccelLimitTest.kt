package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Asymmetric configured accel/decel caps: a mechanism may be allowed to launch harder than it stops
 * (maxAcceleration > maxDeceleration), e.g. a flexible arm whose arrival swing constrains braking
 * but not starting. Verifies the configured deceleration cap — not the acceleration cap — bounds
 * the braking phase, both through the convenience constructor (which builds the model-aware Ruckig
 * profiler itself) and with a caller-supplied profiler.
 *
 * <p>The model is deliberately weak (tiny kV/kA, no gravity) so its back-EMF ceilings sit far above
 * the configured caps and the configured caps are what bind.
 */
class AsymmetricAccelLimitTest {

    @Test
    fun convenienceConstructorBrakesAtDecelCapAndLaunchesAboveIt() {
        val model = ArmModel(0.05, 0.1, 0.1, /* kCos= */ 0.0, /* kSin= */ 0.0)
        val controller =
            MotorMechanismController(
                model,
                40.0,
                8.0,
                1.5,
                MAX_VEL,
                MAX_ACCEL,
                MAX_DECEL,
                MAX_JERK,
                /* feedbackVoltageMargin= */ 1.5,
                /* initialPosition= */ 0.0,
            )
        assertAsymmetricShaping(controller)
    }

    @Test
    fun suppliedProfilerBrakesAtDecelCapAndLaunchesAboveIt() {
        val model = ArmModel(0.05, 0.1, 0.1, /* kCos= */ 0.0, /* kSin= */ 0.0)
        val controller =
            MotorMechanismController(
                model,
                40.0,
                8.0,
                1.5,
                /* feedbackVoltageMargin= */ 1.5,
                ModelAwareRuckigProfiler(model, MAX_VEL, MAX_ACCEL, MAX_DECEL, MAX_JERK, 0.0),
            )
        assertAsymmetricShaping(controller)
    }

    /**
     * Drive the controller with a perfect measurement (the profile's own state) and check the
     * setpoint trajectory: speeding up may exceed the decel cap (proving the launch is not pinned
     * to it), braking never does.
     */
    private fun assertAsymmetricShaping(controller: MotorMechanismController) {
        var peakSpeedUpAccel = 0.0
        var peakBrakingAccel = 0.0
        for (i in 0 until 4000) {
            controller.calculate(
                TARGET,
                controller.setpointPosition,
                controller.setpointVelocity,
                VOLTAGE,
                DT,
            )
            val v = controller.setpointVelocity
            val a = controller.setpointAcceleration
            if (v * a > 0) {
                peakSpeedUpAccel = maxOf(peakSpeedUpAccel, abs(a))
            } else if (v * a < 0) {
                peakBrakingAccel = maxOf(peakBrakingAccel, abs(a))
            }
        }
        assertEquals(TARGET, controller.setpointPosition, 1e-6, "did not land on the target")
        assertTrue(
            peakSpeedUpAccel > MAX_DECEL * 1.5,
            "launch never used the headroom above the decel cap: $peakSpeedUpAccel",
        )
        assertTrue(
            peakSpeedUpAccel <= MAX_ACCEL + 1e-6,
            "launch exceeded the accel cap: $peakSpeedUpAccel",
        )
        assertTrue(
            peakBrakingAccel > MAX_DECEL * 0.9,
            "braking never approached its cap: $peakBrakingAccel",
        )
        assertTrue(
            peakBrakingAccel <= MAX_DECEL + 1e-6,
            "braking exceeded the decel cap: $peakBrakingAccel",
        )
    }

    companion object {
        private const val MAX_VEL = 2.0
        private const val MAX_ACCEL = 12.0
        private const val MAX_DECEL = 4.0
        private const val MAX_JERK = 1000.0
        private const val VOLTAGE = 12.0
        private const val TARGET = 2.0
        private const val DT = 0.005
    }
}
