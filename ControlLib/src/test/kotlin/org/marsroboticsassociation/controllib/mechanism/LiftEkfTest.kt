package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Measures how well a [MotorMechanismEkf] on a [LiftModel] tracks over a lift move, driven through
 * the realistic [EncoderSim] and an unsynced, jittery control loop. As with the arm test, the
 * controller runs on the plant's true state so the motion is fixed and the filter rides along as a
 * passive observer. The lift's gravity is constant, so its EKF is really a linear Kalman filter
 * with a control input, but it still benefits from back-EMF-aware prediction and velocity-lag
 * compensation.
 */
class LiftEkfTest {

    @Test
    fun tracksPositionAndVelocityThroughMove() {
        val start = 0.0
        val targetTicks = 1500.0

        val plant = LiftPlantSim(K_S, K_G, K_V, K_A, -100.0, 3000.0, start)

        val ekf =
            MotorMechanismEkf(
                LiftModel(K_S, K_V, K_A, K_G),
                /* velocityLagSec= */ 0.025,
                /* modelAccelStdDev= */ 1500.0,
                /* positionStdDev= */ 1.0,
                /* velocityStdDev= */ 30.0,
                /* positionTimingJitterStdDev= */ 0.0,
                start,
            )

        val rng = Random(5L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var ekfVelSq = 0.0
        var ekfPosSq = 0.0
        var samples = 0

        for (ms in 0..2500) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                val pos = plant.getEncoderPosition()
                val velTps = plant.getEncoderVelocityTps()

                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(pos.toDouble(), velTps)

                val truePos = plant.getTruePositionTicks()
                val trueVel = plant.getTrueVelocityTps()
                power = 0.002 * (targetTicks - truePos) - 0.0006 * trueVel + (K_G / VOLTAGE)
                power = max(-1.0, min(1.0, power))

                val t = ms / 1000.0
                if (t > 0.2) {
                    ekfVelSq += sq(ekf.getVelocity() - trueVel)
                    ekfPosSq += sq(ekf.getPosition() - truePos)
                    samples++
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val velRms = sqrt(ekfVelSq / samples)
        val posRms = sqrt(ekfPosSq / samples)

        assertTrue(velRms < 25.0, "velocity RMS too high: $velRms TPS")
        assertTrue(posRms < 6.0, "position RMS too high: $posRms ticks")
    }

    companion object {
        private const val K_S = 0.1
        private const val K_G = 2.0
        private const val K_V = 0.006
        private const val K_A = 0.0016
        private const val VOLTAGE = 12.0

        private fun sq(v: Double): Double = v * v
    }
}
