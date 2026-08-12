package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.abs
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class ModelAwareRuckigProfilerTest {

    /** High-kV arm: the sustainable-velocity ceiling sits far below the mechanical cap. */
    private fun highKvArm(): ArmModel = ArmModel(0.3, 4.0, 0.35, 3.5, 0.0)

    private fun profiler(model: ArmModel, p0: Double): ModelAwareRuckigProfiler {
        val p = ModelAwareRuckigProfiler(model, 8.0, 12.0, 480.0, p0)
        p.setAvailableVoltage(AVAILABLE_VOLTS)
        return p
    }

    @Test
    fun velocityNeverExceedsTheLocalModelCeiling() {
        // A falling-ceiling stretch: a
        // descent from the back stop (225°) toward 90°, where the sustainable-velocity ceiling
        // drops as the arm approaches straight-down-over-the-top (180°). The profiler's own
        // one-step lookahead must keep the cruise inside the ceiling at every sample.
        val model = highKvArm()
        val target = Math.toRadians(90.0)
        val p = profiler(model, Math.toRadians(225.0))

        var steps = 0
        while (steps < 3000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS)
            p.update(target, DT)
            steps++
            val ceiling =
                model.maxSustainableVelocity(
                    AVAILABLE_VOLTS,
                    p.getPosition(),
                    target - p.getPosition(),
                )
            assertTrue(
                abs(p.getVelocity()) <= ceiling + 1e-3,
                "cruise inside the local ceiling at step $steps" +
                    ": |v|=${abs(p.getVelocity())} ceiling=$ceiling",
            )
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break
            }
        }
        assertEquals(target, p.getPosition(), 0.0, "arrives exactly")
        assertTrue(steps < 3000, "settled")
    }

    @Test
    fun stopStaysJerkBoundedUnderModelCeilings() {
        val model = highKvArm()
        val target = 0.0
        val p = profiler(model, Math.toRadians(90.0))

        var prevA = p.getAcceleration()
        var steps = 0
        while (steps < 3000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS)
            p.update(target, DT)
            steps++
            assertTrue(
                abs(p.getAcceleration() - prevA) <= 480.0 * DT * 1.05 + 1e-9,
                "jerk bounded under model ceilings at step $steps",
            )
            prevA = p.getAcceleration()
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break
            }
        }
        assertEquals(target, p.getPosition(), 0.0)
        assertTrue(steps < 3000, "settled")
    }

    @Test
    fun heavyGravityDescentRecoversFromOvershootAndClimbsBack() {
        // With heavy gravity (kCos=8 against 10.5 V), the
        // braking ceiling collapses toward a horizontal target, so a descent can carry wrong-way
        // speed past the target. Pushing back toward the target is physically braking, so it must
        // get the braking authority; a limit mapping keyed to the travel direction hands it the
        // acceleration ceiling instead — zero at speed against heavy gravity — and the profile
        // freezes just past the target forever (no plan, and the clamped state needs the same
        // missing authority).
        val model = ArmModel(0.3, 1.2, 0.35, 8.0, 0.0)
        val target = 0.0
        val p = ModelAwareRuckigProfiler(model, 8.0, 12.0, 480.0, Math.toRadians(225.0))

        var maxOvershootDeg = 0.0
        var steps = 0
        while (steps < 4000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS)
            p.update(target, DT)
            steps++
            maxOvershootDeg = max(maxOvershootDeg, Math.toDegrees(target - p.getPosition()))
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break
            }
        }
        assertEquals(target, p.getPosition(), 0.0, "descent lands at the target")
        assertTrue(steps < 4000, "descent settled")
        // The target-position braking ceiling keeps the planned stop honest: overshoot past the
        // horizontal target stays negligible.
        assertTrue(maxOvershootDeg < 0.5, "overshoot bounded: $maxOvershootDeg°")

        // From the landed state, climbing back against the gravity must also work.
        val up = Math.toRadians(90.0)
        steps = 0
        while (steps < 4000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS)
            p.update(up, DT)
            steps++
            if (p.getPosition() == up && p.getVelocity() == 0.0) {
                break
            }
        }
        assertEquals(up, p.getPosition(), 0.0, "ascent arrives against heavy gravity")
        assertTrue(steps < 4000, "ascent settled")
    }

    @Test
    fun noVoltageMeansNoAuthority() {
        // setAvailableVoltage never called after construction default of zero: ceilings are all
        // zero, the setpoint must hold rather than move or teleport.
        val model = highKvArm()
        val p = ModelAwareRuckigProfiler(model, 8.0, 12.0, 480.0, Math.toRadians(45.0))
        for (i in 0 until 100) {
            p.update(Math.toRadians(90.0), DT)
        }
        assertEquals(Math.toRadians(45.0), p.getPosition(), 1e-6, "no volts, no motion")
    }

    @Test
    fun mechanicalCapsStillBindWhenTheModelAllowsMore() {
        // Weak gravity, low kV: the model ceilings sit far above the configured caps, so the
        // configured caps must be the binding limits.
        val model = ArmModel(0.1, 0.5, 0.2, 0.5, 0.0)
        val maxVel = 2.0
        val p = ModelAwareRuckigProfiler(model, maxVel, 6.0, 100.0, 0.0)
        val target = 10.0
        var peakVel = 0.0
        var steps = 0
        while (steps < 3000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS)
            p.update(target, DT)
            steps++
            peakVel = max(peakVel, abs(p.getVelocity()))
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break
            }
        }
        assertEquals(target, p.getPosition(), 0.0)
        assertTrue(peakVel <= maxVel + 1e-9, "configured cap binds: peak=$peakVel")
        assertTrue(peakVel > maxVel * 0.95, "cruises near the configured cap: peak=$peakVel")
    }

    companion object {
        private const val DT = 0.016
        private const val AVAILABLE_VOLTS = 10.5 // 12 V bus minus 1.5 V feedback margin
    }
}
