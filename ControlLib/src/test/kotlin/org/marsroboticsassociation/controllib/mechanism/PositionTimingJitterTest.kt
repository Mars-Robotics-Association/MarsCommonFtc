package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Exercises the live-position read-timing jitter model: the dominant real noise once position is a
 * live counter instead of a 10 ms latch. Two things should hold — the read error grows with speed
 * (the `v * delta` signature) and with the hub (Expansion's RS485 hop is noisier than a Control
 * Hub) — and the EKF's repurposed timing-jitter noise term should keep the estimate stable under
 * it. A fixed seed makes every assertion reproducible.
 */
class PositionTimingJitterTest {

    @Test
    fun readNoiseScalesWithVelocityAndHub() {
        // Same seed, same hub: a faster sweep reads noisier (error is velocity * delta).
        val slow = rawReadNoiseRms(ReadTimingJitter.controlHub(1L), 1.0)
        val fast = rawReadNoiseRms(ReadTimingJitter.controlHub(1L), 6.0)
        assertTrue(
            fast > slow,
            "faster motion should read noisier: $slow vs $fast ticks RMS",
        )

        // Same speed, noisier hub: the Expansion-Hub hop beats the Control Hub.
        val fastControl = rawReadNoiseRms(ReadTimingJitter.controlHub(1L), 6.0)
        val fastExpansion = rawReadNoiseRms(ReadTimingJitter.expansionHub(1L), 6.0)
        assertTrue(
            fastExpansion > fastControl,
            "expansion hub should be noisier than control: $fastControl vs $fastExpansion ticks RMS",
        )
    }

    @Test
    fun stationaryEncoderHasNoReadError() {
        // delta only bites through velocity, so a stationary mechanism reads exactly.
        val still = rawReadNoiseRms(ReadTimingJitter.expansionHub(3L), 0.0)
        assertTrue(still == 0.0, "a stationary encoder should have no read error, got $still")
    }

    @Test
    fun ekfStaysStableUnderControlHubJitter() {
        // The same gravity sweep as ArmEkfTest, but with Control-Hub jitter on the position reads
        // and a matching timing-jitter noise term in the EKF. The estimate should stay tight.
        val angleRms = sweepAngleRms(ReadTimingJitter.CONTROL_HUB_STD_SEC, 1234L)
        assertTrue(angleRms < 0.03, "EKF angle RMS too high under jitter: $angleRms rad")
    }

    companion object {
        private val TICKS_PER_RAD = 28 * 100 / (2 * PI)
        private const val K_S = 0.1
        private const val K_G = 1.5
        private const val K_V = 2.0
        private const val K_A = 0.2
        private const val VOLTAGE = 12.0

        /** RMS of the jittered position read against the exact moving count, in ticks. */
        private fun rawReadNoiseRms(jitter: ReadTimingJitter, velRadPerSec: Double): Double {
            val velTps = velRadPerSec * TICKS_PER_RAD
            var sumSq = 0.0
            var n = 0
            for (i in 1..500) {
                val t = i * 0.005
                val trueTicks = velTps * t
                val err = jitter.read(trueTicks, velTps) - trueTicks
                sumSq += err * err
                n++
            }
            return sqrt(sumSq / n)
        }

        /**
         * Runs the -45..+45 deg sweep with jittered reads and returns the EKF angle RMS in radians.
         */
        private fun sweepAngleRms(jitterStdSec: Double, seed: Long): Double {
            val startRad = -PI / 4
            val targetRad = PI / 4

            val plant =
                ArmPlantSim(
                    K_S,
                    K_G,
                    K_V,
                    K_A,
                    TICKS_PER_RAD,
                    -PI * 0.9,
                    PI * 0.9,
                    startRad,
                    ReadTimingJitter(jitterStdSec, seed),
                )
            val ekf =
                MotorMechanismEkf(
                    ArmModel(K_S, K_V, K_A, K_G, 0.0),
                    /* velocityLagSec= */ 0.025,
                    /* modelAccelStdDev= */ 5.0,
                    /* positionStdDev= */ 0.003,
                    /* velocityStdDev= */ 0.1,
                    /* positionTimingJitterStdDev= */ jitterStdSec,
                    startRad,
                )

            val rng = Random(seed)
            var power = 0.0
            var lastLoopMs = 0
            var nextLoopMs = 20
            var sumSq = 0.0
            var n = 0

            for (ms in 0..2500) {
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

                    val trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD
                    val trueVelTps = plant.getTrueAngularVelocityRadPerSec() * TICKS_PER_RAD
                    power =
                        0.004 * (targetRad * TICKS_PER_RAD - trueTicks) - 0.0008 * trueVelTps +
                            (K_G / VOLTAGE) * cos(plant.getTrueAngleRad())
                    power = max(-1.0, min(1.0, power))

                    if (ms / 1000.0 > 0.2) {
                        val err = ekf.position - plant.getTrueAngleRad()
                        sumSq += err * err
                        n++
                    }

                    lastLoopMs = ms
                    nextLoopMs = ms + 20 + rng.nextInt(10)
                }
            }
            return sqrt(sumSq / n)
        }
    }
}
