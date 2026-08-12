package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Closed-loop test: a [MotorMechanismEkf] on an [ArmModel] driving a real second-order arm plant
 * ([ArmPlantSim]) through a PIDF controller. The controller never sees the true state; it uses only
 * the filter's estimated angle (P and gravity feedforward) and estimated velocity (D), and feeds
 * the filter the voltage it commanded. This exercises the EKF the way it would actually be used on
 * a robot.
 *
 * <p>The move sweeps the arm from -45 deg, down through horizontal where gravity torque peaks, up
 * to +45 deg, then holds.
 */
class ArmPlantControlTest {

    @Test
    fun armReachesAndHoldsTargetUsingOnlyFilterEstimates() {
        val plant =
            ArmPlantSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_RAD,
                -PI * 0.9,
                PI * 0.9,
                START_RAD,
            )
        val filter =
            MotorMechanismEkf(
                ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0),
                /* velocityLagSec= */ 0.025,
                /* modelAccelStdDev= */ 5.0,
                /* positionStdDev= */ 0.003,
                /* velocityStdDev= */ 0.1,
                /* positionTimingJitterStdDev= */ 0.0,
                START_RAD,
            )

        val targetTicks = TARGET_RAD * TICKS_PER_RAD
        val rng = Random(2024L)

        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var maxOvershoot = 0.0
        var settleMs = -1.0
        var sumAngleErrSq = 0.0
        var angleSamples = 0
        var sumSteadyAbsErr = 0.0
        var steadySamples = 0

        for (ms in 0..2500) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                val position = plant.getEncoderPosition()
                val velocityTps = plant.getEncoderVelocityTps()

                // The voltage held over the last interval was the previously commanded power.
                filter.predict(dt, power, VOLTAGE)
                filter.correct(position / TICKS_PER_RAD, velocityTps / TICKS_PER_RAD)

                val estAngleRad = filter.getPosition()
                val estAngleTicks = estAngleRad * TICKS_PER_RAD
                val estVelocityTps = filter.getVelocity() * TICKS_PER_RAD

                power =
                    K_P * (targetTicks - estAngleTicks) - K_D * estVelocityTps +
                        K_F * cos(estAngleRad)
                power = max(-1.0, min(1.0, power))

                val trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD
                val t = ms / 1000.0
                val overshoot = trueTicks - targetTicks
                if (overshoot > maxOvershoot) {
                    maxOvershoot = overshoot
                }
                if (settleMs < 0 && abs(trueTicks - targetTicks) < 10.0) {
                    settleMs = ms.toDouble()
                }
                if (t > 0.3) {
                    sumAngleErrSq += sq(estAngleTicks - trueTicks)
                    angleSamples++
                }
                if (t > 1.5) {
                    sumSteadyAbsErr += abs(trueTicks - targetTicks)
                    steadySamples++
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val finalErrTicks = plant.getTrueAngleRad() * TICKS_PER_RAD - targetTicks
        val steadyAbsErr = sumSteadyAbsErr / steadySamples
        val filterAngleRms = sqrt(sumAngleErrSq / angleSamples)

        assertTrue(
            abs(finalErrTicks) < 10.0,
            "did not reach target, final error $finalErrTicks ticks",
        )
        assertTrue(steadyAbsErr < 8.0, "steady-state error too high: $steadyAbsErr ticks")
        assertTrue(maxOvershoot < 30.0, "overshoot too high: $maxOvershoot ticks")
        assertTrue(settleMs >= 0 && settleMs < 2000.0, "settled too slowly: $settleMs ms")
        assertTrue(
            filterAngleRms < 4.0,
            "filter angle estimate drifted from truth: $filterAngleRms ticks RMS",
        )
    }

    companion object {
        private val TICKS_PER_RAD = 28 * 100 / (2 * PI)
        private const val K_S = 0.1 // static friction, volts
        private const val K_G = 1.5 // gravity, volts at horizontal
        private const val K_V = 2.0 // volts per rad/s (back-EMF)
        private const val K_A = 0.2 // volts per rad/s^2
        private const val VOLTAGE = 12.0

        // PIDF gains (normalized power, tick units) and filter tuning, from calibration.
        private const val K_P = 0.004
        private const val K_D = 0.0008
        private val K_F = K_G / VOLTAGE

        private val START_RAD = -PI / 4
        private val TARGET_RAD = PI / 4

        private fun sq(v: Double): Double = v * v
    }
}
