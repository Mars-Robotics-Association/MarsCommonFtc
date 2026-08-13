package org.marsroboticsassociation.controllib.sim

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Behavioral tests for [BacklashArmMotorSim]. These assert the qualitative consequences of gearbox
 * backlash — lost motion on reversal, gravity free-fall across the gap, and the encoder being blind
 * to the load — rather than exact numbers, since the contact parameters are defaults pending sysid.
 */
class BacklashArmMotorSimTest {

    companion object {
        // Feedforward gains (output-shaft rad units, volts) — same family as ArmControllerTest.
        const val KS = 0.3
        const val KG = 1.5
        const val KV = 1.2
        const val KA = 0.15

        const val TICKS_PER_REV = 28
        const val GEAR_RATIO = 100.0
        val TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * PI)

        // Front hard stop at -45 deg, back hard stop at -225 deg; encoder reads 0 at the front
        // stop.
        val ENCODER_ZERO_OFFSET_RAD = -PI / 4
        val MIN_ANGLE_RAD = -PI * 5 / 4
        val MAX_ANGLE_RAD = -PI / 4

        val BACKLASH_RAD = Math.toRadians(5.0)
        const val HUB_VOLTAGE = 12.0
        const val DT = 0.016

        // A mid-range angle with strong gravity (cos = -0.707). The arm's range is [-225 deg, -45
        // deg],
        // so horizontal (0 deg) is out of range — gravity tests must use an in-range angle.
        val MID_ANGLE_RAD = Math.toRadians(-135.0)
    }

    private fun makeSim(initialAngleRad: Double): BacklashArmMotorSim =
        BacklashArmMotorSim(
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
            BACKLASH_RAD,
        )

    /** Encoder ticks -> motor-side angle in output radians. */
    private fun encoderAngleRad(sim: BacklashArmMotorSim): Double =
        sim.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD

    @Test
    fun holdsStaticallyUnderGravityFeedforward() {
        // Apply the gravity-canceling holding voltage; the arm should barely move.
        val start = MID_ANGLE_RAD
        val sim = makeSim(start)
        val holdPower = (KG * cos(start)) / HUB_VOLTAGE
        for (i in 0 until 200) {
            sim.step(DT, holdPower, HUB_VOLTAGE)
        }
        assertEquals(
            start,
            sim.getTruePositionRad(),
            Math.toRadians(2.0),
            "arm should hold under gravity feedforward",
        )
    }

    @Test
    fun lostMotionOnReversal() {
        // Drive up firmly to seat the forward tooth, then command downward. The motor (encoder)
        // must cross the backlash before the load reacts: while crossing, the load moves far less
        // than the motor.
        val sim = makeSim(MID_ANGLE_RAD)
        for (i in 0 until 40) sim.step(DT, 1.0, HUB_VOLTAGE)

        val loadBefore = sim.getTruePositionRad()
        val encBefore = encoderAngleRad(sim)

        // Reverse just long enough to traverse roughly the backlash gap, not far beyond it.
        var encDelta = 0.0
        var loadDelta = 0.0
        for (i in 0 until 6) {
            sim.step(DT, -1.0, HUB_VOLTAGE)
            encDelta = abs(encoderAngleRad(sim) - encBefore)
            loadDelta = abs(sim.getTruePositionRad() - loadBefore)
            if (encDelta >= BACKLASH_RAD) break
        }

        assertTrue(
            encDelta > BACKLASH_RAD * 0.5,
            "motor/encoder should move appreciably during the reversal",
        )
        assertTrue(
            loadDelta < encDelta * 0.5,
            "load should lag the motor while the lash is being crossed (lost motion): " +
                "loadDelta=$loadDelta encDelta=$encDelta",
        )
    }

    @Test
    fun freeFallAndBlindEncoderAcrossTheGap() {
        val start = MID_ANGLE_RAD
        // Direction gravity accelerates the load: dω_L ∝ −cos(θ).
        val gravityDir = -sign(cos(start))

        // Settle on the gravity-loaded contact face (the seed already rests there).
        val sim = makeSim(start)
        val holdPower = (KG * cos(start)) / HUB_VOLTAGE
        for (i in 0 until 20) sim.step(DT, holdPower, HUB_VOLTAGE)

        val initialLoadVel = sim.getTrueVelocityRadPerSec()

        // Drive the motor across the gap in the gravity direction. The teeth separate and the load,
        // now unsupported, free-falls under gravity until the far face catches it.
        var sawSeparation = false
        var maxDisagreement = 0.0
        for (i in 0 until 12) {
            sim.step(DT, gravityDir, HUB_VOLTAGE)
            if (!sim.isEngaged) sawSeparation = true
            val disagreement = abs(encoderAngleRad(sim) - sim.getTruePositionRad())
            maxDisagreement = maxOf(maxDisagreement, disagreement)
        }
        val finalLoadVel = sim.getTrueVelocityRadPerSec()

        assertTrue(sawSeparation, "the gear teeth should separate (a real dead-band gap opens)")
        assertTrue(
            gravityDir * finalLoadVel > gravityDir * initialLoadVel + 0.2,
            "the unsupported load should accelerate under gravity (free-fall across the gap)",
        )
        assertTrue(
            maxDisagreement > Math.toRadians(0.5),
            "encoder (motor side) should diverge from the true arm angle across the gap: " +
                "${Math.toDegrees(maxDisagreement)} deg",
        )
        assertTrue(
            maxDisagreement <= BACKLASH_RAD + Math.toRadians(1.0),
            "divergence should not exceed the backlash by much: " +
                "${Math.toDegrees(maxDisagreement)} deg",
        )
    }

    @Test
    fun hardStopsClampTheLoad() {
        // Drive hard toward the front stop; the load must not pass it.
        val sim = makeSim(MAX_ANGLE_RAD + Math.toRadians(10.0))
        for (i in 0 until 200) sim.step(DT, 1.0, HUB_VOLTAGE)
        assertTrue(
            sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6,
            "load should not exceed the front hard stop",
        )

        // And toward the back stop.
        val sim2 = makeSim(MIN_ANGLE_RAD - Math.toRadians(10.0) + Math.toRadians(20.0))
        for (i in 0 until 400) sim2.step(DT, -1.0, HUB_VOLTAGE)
        assertTrue(
            sim2.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6,
            "load should not exceed the back hard stop",
        )
    }

    @Test
    fun zeroBacklashStaysWellBehaved() {
        // With no backlash the contact is always engaged; the plant should still be stable and
        // hold roughly still under a gravity-canceling command.
        val start = MID_ANGLE_RAD
        val sim =
            BacklashArmMotorSim(
                KS,
                KG,
                KV,
                KA,
                TICKS_PER_REV,
                GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                start,
                0.0,
            )
        val holdPower = (KG * cos(start)) / HUB_VOLTAGE
        for (i in 0 until 200) sim.step(DT, holdPower, HUB_VOLTAGE)
        assertEquals(start, sim.getTruePositionRad(), Math.toRadians(2.0))
        assertTrue(sim.getMotorVelocityRadPerSec().isFinite())
    }

    @Test
    fun statesStayBoundedUnderAggressiveInput() {
        // Slam the input back and forth to excite the contact; nothing should diverge.
        val sim = makeSim(-PI / 2)
        for (i in 0 until 1000) {
            val power = if (i % 10 < 5) 1.0 else -1.0
            sim.step(DT, power, HUB_VOLTAGE)
            assertTrue(sim.getMotorPositionRad().isFinite(), "motor pos finite")
            assertTrue(sim.getTrueVelocityRadPerSec().isFinite(), "load vel finite")
            assertTrue(abs(sim.getMotorVelocityRadPerSec()) < 1e4, "motor vel bounded")
        }
        assertTrue(
            sim.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6 &&
                sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6
        )
    }
}
