package org.marsroboticsassociation.controllib.sim

import kotlin.math.PI
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Tests for [ArmMotorSim.setEncoder], the parity with [BacklashArmMotorSim.setEncoder] that lets
 * the rigid plant honor a read-timing-jitter encoder model too.
 */
class ArmMotorSimTest {

    companion object {
        const val KS = 0.3
        const val KG = 1.5
        const val KV = 1.2
        const val KA = 0.15
        const val TICKS_PER_REV = 28
        const val GEAR_RATIO = 100.0
        val TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * PI)
        val ENCODER_ZERO_OFFSET_RAD = -PI / 4
        val MIN_ANGLE_RAD = -PI * 5 / 4
        val MAX_ANGLE_RAD = -PI / 4
        const val HUB_VOLTAGE = 12.0
        const val DT = 0.016
    }

    private fun makeSim(initialAngleRad: Double): ArmMotorSim =
        ArmMotorSim(
            KS,
            KG,
            KV,
            KA,
            TICKS_PER_REV,
            GEAR_RATIO,
            ENCODER_ZERO_OFFSET_RAD,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            initialAngleRad,
        )

    @Test
    fun setEncoder_reseedsToCurrentPosition() {
        val start = Math.toRadians(-90.0)
        val sim = makeSim(start)
        sim.setEncoder(EncoderSim()) // zero-jitter model

        val expectedTicks = Math.round((start - ENCODER_ZERO_OFFSET_RAD) * TICKS_PER_RAD).toInt()
        assertTrue(
            abs(expectedTicks - sim.getPositionTicks()) <= 1,
            "swapped encoder should read the current arm position immediately",
        )
        assertEquals(
            0.0,
            sim.getVelocityTps(),
            1e-9,
            "freshly seeded encoder should report zero velocity",
        )
    }

    @Test
    fun setEncoder_rejectsNull() {
        val sim = makeSim(Math.toRadians(-90.0))
        assertThrows(IllegalArgumentException::class.java) { sim.setEncoder(null) }
    }

    @Test
    fun setEncoder_windowedVelocityStillWorksUnderMotion() {
        // Start deep in range (back stop is -225 deg) so the arm keeps moving through the flight.
        val sim = makeSim(Math.toRadians(-200.0))
        sim.setEncoder(EncoderSim.expansionHub(7L))

        var peakWindowed = 0.0
        var peakTrue = 0.0
        for (i in 0 until 40) {
            sim.step(DT, 0.5, HUB_VOLTAGE) // drive up, away from the back stop
            peakWindowed = maxOf(peakWindowed, sim.getVelocityTps())
            peakTrue = maxOf(peakTrue, sim.getTrueVelocityRadPerSec() * TICKS_PER_RAD)
        }
        assertTrue(peakWindowed.isFinite())
        assertTrue(peakWindowed > 0, "windowed velocity should track the upward drive")
        // The 50 ms window / 20-TPS quantization keeps the peak near the true peak, not exact.
        assertEquals(
            peakTrue,
            peakWindowed,
            maxOf(60.0, peakTrue * 0.25),
            "peak windowed velocity should be close to the true peak rate",
        )
    }

    @Test
    fun expansionHubJitter_stalesPositionAtSpeed() {
        // At speed, the Expansion-Hub read-timing delay stales the live position by velocity*delta,
        // so a jittered encoder generally reads a different integer tick than a zero-jitter one for
        // the same motion. Run both from the same state and confirm they can differ.
        val clean = makeSim(Math.toRadians(-90.0))
        clean.setEncoder(EncoderSim())
        val jittery = makeSim(Math.toRadians(-90.0))
        jittery.setEncoder(EncoderSim.expansionHub(1L))

        var maxDiff = 0
        for (i in 0 until 80) {
            clean.step(DT, 1.0, HUB_VOLTAGE)
            jittery.step(DT, 1.0, HUB_VOLTAGE)
            maxDiff =
                maxOf(
                    maxDiff,
                    abs(clean.getPositionTicks() - jittery.getPositionTicks()),
                )
        }
        assertTrue(maxDiff > 0, "Expansion-Hub jitter should stale the position read at speed")
    }
}
