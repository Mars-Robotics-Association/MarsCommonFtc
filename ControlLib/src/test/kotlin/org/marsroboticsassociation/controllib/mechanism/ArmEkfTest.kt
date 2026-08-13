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
 * Measures how well a [MotorMechanismEkf] on an [ArmModel] tracks as the arm sweeps from -45 deg
 * through horizontal to +45 deg, driven through the realistic [EncoderSim] and an unsynced, jittery
 * control loop.
 *
 * <p>To isolate estimator quality from the controller, the controller runs on the plant's true
 * state, so the motion is fixed; the filter rides along as a passive observer on the same encoder
 * stream and the same commanded voltage. The model and the velocity-lag compensation should keep
 * velocity error low even while the arm is accelerating, where the 50 ms velocity reading lags
 * hardest.
 */
class ArmEkfTest {

    @Test
    fun tracksAngleAndVelocityThroughGravitySweep() {
        val startRad = -PI / 4
        val targetRad = PI / 4

        val plant = ArmPlantSim(K_S, K_G, K_V, K_A, TICKS_PER_RAD, -PI * 0.9, PI * 0.9, startRad)

        val ekf =
            MotorMechanismEkf(
                ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0),
                /* velocityLagSec= */ 0.025,
                /* modelAccelStdDev= */ 5.0,
                /* positionStdDev= */ 0.003,
                /* velocityStdDev= */ 0.1,
                /* positionTimingJitterStdDev= */ 0.0,
                startRad,
            )

        val rng = Random(99L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var ekfVelSqAll = 0.0
        var ekfVelSqMoving = 0.0
        var ekfAngleSq = 0.0
        var allSamples = 0
        var movingSamples = 0

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
                ekf.correct(pos / TICKS_PER_RAD, velTps / TICKS_PER_RAD)

                // Controller acts on TRUE state so the trajectory is fixed for the measurement.
                val trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD
                val trueVelTps = plant.getTrueAngularVelocityRadPerSec() * TICKS_PER_RAD
                power =
                    0.004 * (targetRad * TICKS_PER_RAD - trueTicks) - 0.0008 * trueVelTps +
                        (K_G / VOLTAGE) * cos(plant.getTrueAngleRad())
                power = max(-1.0, min(1.0, power))

                val trueVelRad = plant.getTrueAngularVelocityRadPerSec()
                val velErr = ekf.velocity - trueVelRad

                val t = ms / 1000.0
                if (t > 0.2) {
                    ekfVelSqAll += velErr * velErr
                    ekfAngleSq += sq(ekf.position - plant.getTrueAngleRad())
                    allSamples++
                    if (abs(trueVelRad) > 0.5) {
                        ekfVelSqMoving += velErr * velErr
                        movingSamples++
                    }
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val velRmsAll = sqrt(ekfVelSqAll / allSamples)
        val velRmsMoving = sqrt(ekfVelSqMoving / movingSamples)
        val angleRms = sqrt(ekfAngleSq / allSamples)

        assertTrue(velRmsAll < 0.04, "velocity RMS too high: $velRmsAll rad/s")
        assertTrue(
            velRmsMoving < 0.07,
            "velocity RMS while moving too high: $velRmsMoving rad/s",
        )
        assertTrue(angleRms < 0.02, "angle RMS too high: $angleRms rad")
    }

    companion object {
        private val TICKS_PER_RAD = 28 * 100 / (2 * PI)
        private const val K_S = 0.1
        private const val K_G = 1.5
        private const val K_V = 2.0
        private const val K_A = 0.2
        private const val VOLTAGE = 12.0

        private fun sq(v: Double): Double = v * v
    }
}
