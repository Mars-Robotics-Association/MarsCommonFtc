package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Closed-loop test of a [MotorMechanismController] (built on an [ArmModel]) driving [ArmPlantSim],
 * with a [MotorMechanismEkf] on the same model estimating the state. The controller sees only the
 * filter's estimates and the bus voltage; it profiles the move, feeds forward from the model,
 * corrects with PID, and clamps the output. The move sweeps -45 deg through horizontal to +45 deg.
 */
class ArmControllerTest {

    @Test
    fun reachesAndHoldsTargetWithinAllLimits() {
        val start = -PI / 4
        val target = PI / 4

        val plant = ArmPlantSim(K_S, K_G, K_V, K_A, TICKS_PER_RAD, -PI * 0.9, PI * 0.9, start)
        // One model, shared by the filter and the controller.
        val model = ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(
                model,
                /* kP= */ 40.0,
                /* kI= */ 8.0,
                /* kD= */ 1.5,
                /* maxVelocity= */ 8.0,
                /* maxAcceleration= */ 12.0,
                /* maxJerk= */ 480.0,
                /* feedbackVoltageMargin= */ 1.5,
                start,
            )

        val rng = Random(1L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var maxOvershootDeg = 0.0
        var settleMs = -1.0
        var peakVelocity = 0.0
        var maxVoltage = 0.0
        var sumSteadyAbsErrDeg = 0.0
        var steadySamples = 0

        for (ms in 0..3000) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(
                    plant.getEncoderPosition() / TICKS_PER_RAD,
                    plant.getEncoderVelocityTps() / TICKS_PER_RAD,
                )

                val voltage = controller.calculate(target, ekf.position, ekf.velocity, VOLTAGE, dt)
                maxVoltage = max(maxVoltage, abs(voltage))
                power = voltage / VOLTAGE // voltage compensation; clamped inside the controller
                // already

                val trueDeg = Math.toDegrees(plant.getTrueAngleRad())
                peakVelocity = max(peakVelocity, abs(plant.getTrueAngularVelocityRadPerSec()))
                maxOvershootDeg = max(maxOvershootDeg, trueDeg - 45.0)
                if (settleMs < 0 && abs(trueDeg - 45.0) < 1.0) {
                    settleMs = ms.toDouble()
                }
                if (ms / 1000.0 > 2.0) {
                    sumSteadyAbsErrDeg += abs(trueDeg - 45.0)
                    steadySamples++
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val finalDeg = Math.toDegrees(plant.getTrueAngleRad())
        val steadyAbsErrDeg = sumSteadyAbsErrDeg / steadySamples

        assertTrue(
            abs(finalDeg - 45.0) < 2.0,
            "did not reach target, final $finalDeg deg",
        )
        assertTrue(
            steadyAbsErrDeg < 1.5,
            "steady-state error too high: $steadyAbsErrDeg deg",
        )
        assertTrue(maxOvershootDeg < 8.0, "overshoot too high: $maxOvershootDeg deg")
        assertTrue(settleMs >= 0 && settleMs < 1500.0, "settled too slowly: $settleMs ms")
        // Back-EMF-aware velocity ceiling: ~ (12 - kS - gravity)/kV ~= 5.4 rad/s here.
        assertTrue(peakVelocity < 5.6, "exceeded the back-EMF velocity ceiling: $peakVelocity")
        // Voltage clamp respected.
        assertTrue(maxVoltage <= VOLTAGE + 1e-6, "voltage exceeded the clamp: $maxVoltage")
    }

    companion object {
        private val TICKS_PER_RAD = 28 * 100 / (2 * PI)
        private const val K_S = 0.1
        private const val K_G = 1.5
        private const val K_V = 2.0
        private const val K_A = 0.2
        private const val VOLTAGE = 12.0
    }
}
