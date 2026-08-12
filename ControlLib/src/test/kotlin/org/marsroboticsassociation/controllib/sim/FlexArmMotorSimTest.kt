package org.marsroboticsassociation.controllib.sim

import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Behavioral tests for [FlexArmMotorSim]. These assert the qualitative consequences of arm
 * structural flex on top of gearbox backlash — above all the long-heavy-arm signature the rigid
 * plants cannot reproduce: the arm is bouncy on the way down but not on the way up — rather than
 * exact numbers, since the flex and contact parameters are defaults pending sysid.
 */
class FlexArmMotorSimTest {

    companion object {
        // Heavy long-arm plant (matches the ControlLab arm defaults) with an over-the-top
        // workspace.
        const val KS = 0.3
        const val KG = 3.5
        const val KV = 1.2
        const val KA = 0.35

        const val TICKS_PER_REV = 28
        const val GEAR_RATIO = 100.0
        val TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI)

        const val ENCODER_ZERO_OFFSET_RAD = 0.0
        val MIN_ANGLE_RAD = Math.toRadians(-45.0)
        val MAX_ANGLE_RAD = Math.toRadians(225.0)

        val BACKLASH_RAD = Math.toRadians(5.0)
        const val FLEX_HZ = 3.0
        const val FLEX_ZETA = 0.03
        const val HUB_VOLTAGE = 12.0
        const val DT = 0.016
    }

    private fun makeSim(initialAngleRad: Double): FlexArmMotorSim =
        FlexArmMotorSim(
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
            FLEX_HZ,
            FLEX_ZETA,
        )

    /** Encoder ticks -> motor-side angle in output radians. */
    private fun encoderAngleRad(sim: FlexArmMotorSim): Double =
        sim.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD

    @Test
    fun seedsInStaticEquilibriumAndHoldsUnderGravityFeedforward() {
        val start = Math.toRadians(30.0) // strong gravity (cos = 0.87)
        val sim = makeSim(start)
        val holdPower = (KG * cos(start)) / HUB_VOLTAGE
        for (i in 0 until 200) {
            sim.step(DT, holdPower, HUB_VOLTAGE)
        }
        assertEquals(
            start,
            sim.getTruePositionRad(),
            Math.toRadians(2.0),
            "tip should hold under gravity feedforward",
        )
        // The flex spring must carry the tip's gravity: hub sits above the tip by the static
        // deflection, on the same side gravity pulls the tip down from.
        assertTrue(
            sim.getHubPositionRad() > sim.getTruePositionRad(),
            "flex spring should deflect under the tip's gravity",
        )
    }

    @Test
    fun lostMotionOnReversal() {
        // Drive up firmly to seat the forward tooth, then reverse: the motor crosses the lash
        // while the tip barely reacts.
        val sim = makeSim(Math.toRadians(90.0))
        for (i in 0 until 40) sim.step(DT, 1.0, HUB_VOLTAGE)

        val tipBefore = sim.getTruePositionRad()
        val encBefore = encoderAngleRad(sim)

        var encDelta = 0.0
        var tipDelta = 0.0
        for (i in 0 until 6) {
            sim.step(DT, -1.0, HUB_VOLTAGE)
            encDelta = abs(encoderAngleRad(sim) - encBefore)
            tipDelta = abs(sim.getTruePositionRad() - tipBefore)
            if (encDelta >= BACKLASH_RAD) break
        }

        assertTrue(
            encDelta > BACKLASH_RAD * 0.5,
            "motor/encoder should move appreciably during the reversal",
        )
        assertTrue(
            tipDelta < encDelta * 0.5,
            "tip should lag the motor while the lash is being crossed (lost motion): " +
                "tipDelta=$tipDelta encDelta=$encDelta",
        )
    }

    /**
     * The reason this sim exists: under the same closed-loop ramp, a long heavy flexible arm is
     * bouncy on the way <b>down</b> (the tip oscillation repeatedly unloads the gear mesh and each
     * re-impact pumps gravity energy back into the flex mode) and comparatively smooth on the way
     * <b>up</b> (drive torque and gravity load the same tooth face, so the mesh never unloads). The
     * two-inertia rigid-load plant shows neither.
     */
    @Test
    fun descentIsBouncyButAscentIsNot() {
        val downOsc = trackedMoveOscillation(Math.toRadians(90.0), Math.toRadians(-40.0))
        val upOsc = trackedMoveOscillation(Math.toRadians(-40.0), Math.toRadians(90.0))

        assertTrue(
            downOsc > 0.15,
            "descent should visibly bounce (detrended tip-velocity std, rad/s): $downOsc",
        )
        assertTrue(
            downOsc > 1.8 * upOsc,
            "descent should be clearly bouncier than ascent: down=$downOsc up=$upOsc",
        )
    }

    /**
     * Runs a PD + gravity + velocity feedforward controller (on the motor side, like a real
     * motor-encoder controller) along a constant-velocity ramp from [from] to [to], and returns the
     * standard deviation of the tip velocity about the commanded ramp rate over the middle of the
     * move — the "bounciness" of the arm itself.
     */
    private fun trackedMoveOscillation(from: Double, to: Double): Double {
        val sim = makeSim(from)
        val kP = 12.0
        val kD = 0.8
        val spVel = sign(to - from) * 1.2
        val moveEnd = (to - from) / spVel

        val mid = ArrayList<Double>()
        var t = 0.0
        while (t < moveEnd) {
            val sp = from + spVel * t
            val measPos = sim.getMotorPositionRad()
            val measVel = sim.getMotorVelocityRadPerSec()
            val u = kP * (sp - measPos) + kD * (0 - measVel) + KG * cos(measPos) + KV * spVel
            val power = maxOf(-1.0, minOf(1.0, u / HUB_VOLTAGE))
            sim.step(DT, power, HUB_VOLTAGE)
            if (t > 0.1 * moveEnd && t < 0.9 * moveEnd) {
                mid.add(sim.getTrueVelocityRadPerSec() - spVel)
            }
            t += DT
        }
        var mean = 0.0
        for (v in mid) mean += v
        mean /= mid.size
        var variance = 0.0
        for (v in mid) variance += (v - mean) * (v - mean)
        return sqrt(variance / mid.size)
    }

    @Test
    fun encoderIsBlindToTheTipAcrossLashAndFlex() {
        // During a hard descent the encoder (motor side) diverges from the tip by more than the
        // lash alone — the flex deflection adds to the lost motion a motor encoder cannot see.
        val sim = makeSim(Math.toRadians(90.0))
        var maxDisagreement = 0.0
        for (i in 0 until 60) {
            sim.step(DT, -0.6, HUB_VOLTAGE)
            val disagreement = abs(encoderAngleRad(sim) - sim.getTruePositionRad())
            maxDisagreement = maxOf(maxDisagreement, disagreement)
        }
        assertTrue(
            maxDisagreement > Math.toRadians(1.0),
            "encoder should diverge from the true tip angle: " +
                "${Math.toDegrees(maxDisagreement)} deg",
        )
    }

    @Test
    fun hardStopsClampTheArm() {
        val sim = makeSim(MAX_ANGLE_RAD - Math.toRadians(10.0))
        for (i in 0 until 300) sim.step(DT, 1.0, HUB_VOLTAGE)
        assertTrue(
            sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6,
            "tip should not exceed the back hard stop",
        )

        val sim2 = makeSim(MIN_ANGLE_RAD + Math.toRadians(20.0))
        for (i in 0 until 400) sim2.step(DT, -1.0, HUB_VOLTAGE)
        assertTrue(
            sim2.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6,
            "tip should not exceed the front hard stop",
        )
    }

    @Test
    fun statesStayBoundedUnderAggressiveInput() {
        val sim = makeSim(Math.toRadians(90.0))
        for (i in 0 until 1000) {
            val power = if (i % 10 < 5) 1.0 else -1.0
            sim.step(DT, power, HUB_VOLTAGE)
            assertTrue(sim.getMotorPositionRad().isFinite(), "motor pos finite")
            assertTrue(sim.getTrueVelocityRadPerSec().isFinite(), "tip vel finite")
            assertTrue(abs(sim.getMotorVelocityRadPerSec()) < 1e4, "motor vel bounded")
        }
        assertTrue(
            sim.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6 &&
                sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6
        )
    }

    @Test
    fun rejectsNonPositiveFlexParams() {
        assertThrows(IllegalArgumentException::class.java) { makeSim(0.0).setFlex(0.0, 0.03) }
        assertThrows(IllegalArgumentException::class.java) { makeSim(0.0).setFlex(3.0, 0.0) }
    }
}
