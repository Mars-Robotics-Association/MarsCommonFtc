package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.abs
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Closed-loop test of a [MotorMechanismController] (built on a [LiftModel]) driving [LiftPlantSim],
 * with a [MotorMechanismEkf] on the same model estimating the state. Besides reaching and holding
 * the target, this exercises the back-EMF limits: the configured velocity limit (2500) is set
 * higher than the motor can sustain, so the controller throttles the cruise speed down toward the
 * voltage ceiling `(12 - margin - kS - kG)/kV ~= 1400` instead. The velocity-dependent acceleration
 * ceiling does most of that throttling on the way up.
 */
class LiftControllerTest {

    @Test
    fun reachesTargetAndObeysBackEmfVelocityCeiling() {
        val start = 0.0
        val target = 1500.0

        val plant = LiftPlantSim(K_S, K_G, K_V, K_A, -100.0, 3000.0, start)
        val model = LiftModel(K_S, K_V, K_A, K_G)
        val ekf = MotorMechanismEkf(model, 0.025, 1500.0, 1.0, 30.0, 0.0, start)
        val controller =
            MotorMechanismController(
                model,
                /* kP= */ 0.10,
                /* kI= */ 0.15,
                /* kD= */ 0.02,
                CONFIGURED_MAX_VELOCITY,
                /* maxAcceleration= */ 6000.0,
                /* maxJerk= */ 40000.0,
                FEEDBACK_MARGIN,
                start,
            )

        val rng = Random(3L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var peakVelocity = 0.0
        var maxVoltage = 0.0
        var settleMs = -1.0

        for (ms in 0..3000) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(plant.getEncoderPosition().toDouble(), plant.getEncoderVelocityTps())

                val voltage = controller.calculate(target, ekf.position, ekf.velocity, VOLTAGE, dt)
                maxVoltage = max(maxVoltage, abs(voltage))
                power = voltage / VOLTAGE

                val truePos = plant.getTruePositionTicks()
                peakVelocity = max(peakVelocity, abs(plant.getTrueVelocityTps()))
                if (settleMs < 0 && abs(truePos - target) < 10.0) {
                    settleMs = ms.toDouble()
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val finalPos = plant.getTruePositionTicks()

        assertTrue(abs(finalPos - target) < 25.0, "did not reach target, final $finalPos")
        assertTrue(settleMs >= 0 && settleMs < 2000.0, "settled too slowly: $settleMs ms")
        assertTrue(maxVoltage <= VOLTAGE + 1e-6, "voltage exceeded the clamp: $maxVoltage")
        // The back-EMF ceiling, not the configured limit, governed the cruise speed.
        assertTrue(
            peakVelocity < BACK_EMF_CEILING * 1.05,
            "exceeded the back-EMF ceiling: $peakVelocity",
        )
        assertTrue(
            peakVelocity < 0.8 * CONFIGURED_MAX_VELOCITY && peakVelocity > 0.85 * BACK_EMF_CEILING,
            "back-EMF ceiling did not actually bind: $peakVelocity",
        )
    }

    @Test
    fun descendsFasterThanItCouldClimb() {
        // Same lift, but now dropping from the top to near the bottom. Gravity aids the descent, so
        // the direction-aware ceiling lets the setpoint run past the climb ceiling it could never
        // exceed going up.
        val start = 2000.0
        val target = 200.0

        val plant = LiftPlantSim(K_S, K_G, K_V, K_A, -100.0, 3000.0, start)
        val model = LiftModel(K_S, K_V, K_A, K_G)
        val ekf = MotorMechanismEkf(model, 0.025, 1500.0, 1.0, 30.0, 0.0, start)
        val controller =
            MotorMechanismController(
                model,
                /* kP= */ 0.10,
                /* kI= */ 0.15,
                /* kD= */ 0.02,
                CONFIGURED_MAX_VELOCITY,
                /* maxAcceleration= */ 6000.0,
                /* maxJerk= */ 40000.0,
                FEEDBACK_MARGIN,
                start,
            )

        val rng = Random(5L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var peakSpeed = 0.0
        var maxVoltage = 0.0

        for (ms in 0..4000) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(plant.getEncoderPosition().toDouble(), plant.getEncoderVelocityTps())

                val voltage = controller.calculate(target, ekf.position, ekf.velocity, VOLTAGE, dt)
                maxVoltage = max(maxVoltage, abs(voltage))
                power = voltage / VOLTAGE

                peakSpeed = max(peakSpeed, abs(plant.getTrueVelocityTps()))

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val finalPos = plant.getTruePositionTicks()

        assertTrue(abs(finalPos - target) < 25.0, "did not reach target, final $finalPos")
        assertTrue(maxVoltage <= VOLTAGE + 1e-6, "voltage exceeded the clamp: $maxVoltage")
        // The whole point of charging gravity in the direction of travel: the descent runs past
        // the climb ceiling, where worst-case gravity charging would clamp it.
        assertTrue(
            peakSpeed > 1.05 * BACK_EMF_CEILING,
            "descent did not exploit gravity, peak $peakSpeed",
        )
        // But still within the descent ceiling, where gravity assist runs out.
        assertTrue(
            peakSpeed < DESCENT_CEILING * 1.05,
            "exceeded the descent ceiling: $peakSpeed",
        )
    }

    companion object {
        private const val K_S = 0.1
        private const val K_G = 2.0
        private const val K_V = 0.006
        private const val K_A = 0.0016
        private const val VOLTAGE = 12.0
        private const val FEEDBACK_MARGIN = 1.5
        private const val CONFIGURED_MAX_VELOCITY = 2500.0
        // The cruise ceiling climbing, once the feedback margin is held back from the feedforward.
        // Gravity opposes the motion, so it is subtracted.
        private val BACK_EMF_CEILING = (VOLTAGE - FEEDBACK_MARGIN - K_S - K_G) / K_V // ~1400 TPS
        // The cruise ceiling descending: gravity aids the motion, so it is added instead.
        private val DESCENT_CEILING = (VOLTAGE - FEEDBACK_MARGIN - K_S + K_G) / K_V // ~2067 TPS
    }
}
