package org.marsroboticsassociation.controllib.mechanism

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Focused tests for [MotorMechanismEkf] behaviors not covered by the closed-loop sweeps, notably
 * saturation awareness in the predict step. Uses a [LiftModel] (constant gravity, so the EKF is
 * linear and the arithmetic is easy to reason about).
 */
class MotorMechanismEkfTest {

    @Test
    fun overRangePowerClampsToRail() {
        val dt = 0.02
        val busVoltage = 12.0

        // An over-command (power 100) cannot outrun full power (1.0): both apply 12 V, so the
        // filter
        // predicts identical motion. Saturation awareness means an over-command can never inflate
        // the
        // estimate the way trusting the raw request would.
        val over = newEkf()
        over.predict(dt, 100.0, busVoltage)

        val full = newEkf()
        full.predict(dt, 1.0, busVoltage)

        assertEquals(full.position, over.position, 1e-9)
        assertEquals(full.velocity, over.velocity, 1e-9)
        // Guard against a vacuous pass: full power should actually move the mechanism.
        assertTrue(
            full.velocity > 0.0,
            "full power should produce motion, got ${full.velocity}",
        )

        // The clamp is symmetric: an over-command the other way matches full reverse power.
        val overReverse = newEkf()
        overReverse.predict(dt, -100.0, busVoltage)
        val fullReverse = newEkf()
        fullReverse.predict(dt, -1.0, busVoltage)
        assertEquals(fullReverse.velocity, overReverse.velocity, 1e-9)
    }

    @Test
    fun appliedVoltageIsPowerTimesBusVoltage() {
        val dt = 0.02

        // Half power on a 12 V bus is the same 6 V the motor would see at full power on a 6 V bus,
        // so
        // the prediction must match — confirming the power-times-bus scaling.
        val halfOnFull = newEkf()
        halfOnFull.predict(dt, 0.5, 12.0)

        val fullOnHalf = newEkf()
        fullOnHalf.predict(dt, 1.0, 6.0)

        assertEquals(fullOnHalf.position, halfOnFull.position, 1e-12)
        assertEquals(fullOnHalf.velocity, halfOnFull.velocity, 1e-12)
    }

    companion object {
        private const val K_S = 0.1
        private const val K_G = 2.0
        private const val K_V = 0.006
        private const val K_A = 0.0016

        private fun newEkf(): MotorMechanismEkf =
            MotorMechanismEkf(
                LiftModel(K_S, K_V, K_A, K_G),
                /* velocityLagSec= */ 0.025,
                /* modelAccelStdDev= */ 1500.0,
                /* positionStdDev= */ 1.0,
                /* velocityStdDev= */ 30.0,
                /* positionTimingJitterStdDev= */ 0.0,
                /* initialPosition= */ 0.0,
            )
    }
}
