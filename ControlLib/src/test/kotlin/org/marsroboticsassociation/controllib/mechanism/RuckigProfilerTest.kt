package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class RuckigProfilerTest {

    /** Step until the profile settles exactly on the target, with a wall-clock budget. */
    private fun runToTarget(
        p: RuckigProfiler,
        target: Double,
        dt: Double,
        maxSimTime: Double,
    ): Int {
        var steps = 0
        val budget = ceil(maxSimTime / dt).toInt()
        while (steps < budget) {
            p.update(target, dt)
            steps++
            if (p.position == target && p.velocity == 0.0 && p.acceleration == 0.0) {
                return steps
            }
        }
        throw AssertionError(
            "did not settle on $target within $maxSimTime" +
                " s: p=${p.position} v=${p.velocity} a=${p.acceleration}"
        )
    }

    // ---------------------------------------------------------------
    // The headline: stops are planned, not clamped
    // ---------------------------------------------------------------

    @Test
    fun stopIsJerkBoundedAllTheWayIntoTheTarget() {
        val maxV = 2.0
        val maxA = 4.0
        val maxD = 4.0
        val maxJ = 20.0
        val p = RuckigProfiler(maxV, maxA, maxD, maxJ, 0.0)

        var prevA = p.acceleration
        var prevV = p.velocity
        var maxJerkObserved = 0.0
        var steps = 0
        while (steps < 2000) {
            p.update(1.0, DT)
            steps++
            maxJerkObserved = max(maxJerkObserved, abs(p.acceleration - prevA) / DT)
            assertTrue(abs(p.velocity) <= maxV + 1e-9, "velocity limit")
            assertTrue(
                abs(p.velocity - prevV) <= max(maxA, maxD) * DT + 1e-9,
                "accel limit step $steps",
            )
            prevA = p.acceleration
            prevV = p.velocity
            if (p.position == 1.0 && p.velocity == 0.0 && p.acceleration == 0.0) {
                break
            }
        }
        // Lands exactly (the final step snaps when the remaining plan fits inside dt)
        assertEquals(1.0, p.position, 0.0, "exact landing")
        assertEquals(0.0, p.velocity, 0.0)
        // Jerk stays bounded through the entire stop, including the very last steps into the
        // target: the deceleration is planned, not clamped.
        assertTrue(
            maxJerkObserved <= maxJ * 1.05 + 1e-9,
            "jerk bounded through the stop; observed $maxJerkObserved",
        )
        assertTrue(steps < 2000, "settled")
    }

    @Test
    fun asymmetricDecelInNegativeDirectionStaysInTravelFrameBounds() {
        val aAccel = 1.0
        val aDecel = 5.0
        val p = RuckigProfiler(1.0, aAccel, aDecel, 10.0, 0.0)
        val steps = runToTarget(p, -1.0, DT, 10.0)
        assertTrue(steps > 10)

        // Re-run sampling accelerations: moving in −, speeding up is a >= -aAccel,
        // braking is a <= +aDecel.
        val q = RuckigProfiler(1.0, aAccel, aDecel, 10.0, 0.0)
        for (i in 0 until steps) {
            q.update(-1.0, DT)
            val a = q.acceleration
            assertTrue(a >= -aAccel - 1e-9, "accel bound (speeding up in −): $a")
            assertTrue(a <= aDecel + 1e-9, "decel bound (braking in −): $a")
        }
    }

    // ---------------------------------------------------------------
    // Back-EMF-style per-loop limit rewrites
    // ---------------------------------------------------------------

    @Test
    fun survivesBackEmfStyleAccelDecay() {
        // Accel authority collapses with speed (back-EMF), braking held at a conservative
        // constant — the §7 usage pattern. The profile must stay continuous and still land.
        val maxV = 2.0
        val maxJ = 20.0
        val p = RuckigProfiler(maxV, 4.0, 2.0, maxJ, 0.0)

        var prevP = p.position
        var prevA = p.acceleration
        var settled = false
        for (i in 0 until 1500) {
            val speed = abs(p.velocity)
            p.setMaxAcceleration(max(0.0, 4.0 - 1.8 * speed))
            p.setMaxDeceleration(2.0) // conservative braking ceiling
            p.update(3.0, DT)
            assertTrue(
                abs(p.position - prevP) <= maxV * DT + 1e-9,
                "position continuous at step $i",
            )
            assertTrue(
                abs(p.acceleration - prevA) <= maxJ * DT * 1.05 + 1e-9,
                "jerk bounded under decaying limits at step $i",
            )
            prevP = p.position
            prevA = p.acceleration
            if (p.position == 3.0 && p.velocity == 0.0) {
                settled = true
                break
            }
        }
        assertTrue(settled, "lands despite per-loop limit rewrites")
    }

    @Test
    fun velocityCapDroppedBelowCurrentSpeedBrakesViaPreTrajectory() {
        val p = RuckigProfiler(2.0, 4.0, 4.0, 20.0, 0.0)
        // Get up to speed
        while (p.velocity < 1.5) {
            p.update(10.0, DT)
        }
        // Drop the cap below current speed: Ruckig absorbs this with a brake pre-trajectory.
        p.setMaxVelocity(0.75)
        var prevV = p.velocity
        var belowCap = false
        for (i in 0 until 1500) {
            p.update(10.0, DT)
            assertTrue(
                abs(p.velocity - prevV) <= 4.0 * DT + 1e-9,
                "velocity continuous while braking to the new cap",
            )
            prevV = p.velocity
            if (belowCap) {
                assertTrue(p.velocity <= 0.75 + 1e-9, "stays under the new cap")
            } else if (p.velocity <= 0.75) {
                belowCap = true
            }
            if (p.position == 10.0 && p.velocity == 0.0) {
                break
            }
        }
        assertTrue(belowCap, "came down to the new velocity cap")
        assertEquals(10.0, p.position, 0.0, "still lands on the target")
    }

    @Test
    fun velocityCeilingChatterDoesNotFreezeTheProfile() {
        // Regression: MotorMechanismController evaluates the back-EMF velocity ceiling at the
        // pre-update profile state, so a profile cruising at the ceiling sits a hair ABOVE each
        // freshly lowered ceiling — while the accel ceiling is ~0 by construction at sustainable
        // velocity. From that exact state Ruckig has no feasible plan (result -110), and a
        // hold-on-error fallback freezes the profile forever. The profiler must clamp into the
        // band and keep making progress instead.
        val p = RuckigProfiler(2.0, 4.0, 4.0, 480.0, 0.0)
        while (p.velocity < 1.99) {
            p.update(50.0, DT)
        }
        val posBefore = p.position
        val velBefore = p.velocity
        for (i in 0 until 60) {
            // Each loop the ceiling lands just below the current speed and the speed-up
            // authority is revoked — the exact controller-induced chatter regime.
            p.setMaxVelocity(p.velocity * 0.997)
            p.setMaxAcceleration(0.0)
            p.update(50.0, DT)
        }
        val advanced = p.position - posBefore
        assertTrue(
            advanced > velBefore * DT * 60 * 0.5,
            "profile keeps moving through ceiling chatter; advanced=$advanced",
        )
        assertTrue(p.velocity <= velBefore + 1e-9, "no speed gained without authority")
        assertTrue(p.velocity > 0.5 * velBefore, "no spurious hard braking either")
    }

    @Test
    fun zeroVelocityAuthorityHoldsPosition() {
        val p = RuckigProfiler(2.0, 4.0, 4.0, 20.0, 0.0)
        p.setMaxVelocity(0.0)
        for (i in 0 until 200) {
            p.update(1.0, DT)
        }
        assertTrue(abs(p.position) < 1e-6, "no authority: setpoint holds")
    }

    // ---------------------------------------------------------------
    // Robustness
    // ---------------------------------------------------------------

    @Test
    fun unlimitedJerkRunsSecondOrderProfiles() {
        val maxA = 3.0
        val p = RuckigProfiler(1.5, maxA, maxA, RuckigProfiler.UNLIMITED_JERK, 0.0)
        var prevV = p.velocity
        var steps = 0
        while (steps < 1000) {
            p.update(2.0, DT)
            steps++
            assertTrue(
                abs(p.velocity - prevV) <= maxA * DT * 1.05 + 1e-9,
                "second-order accel bound",
            )
            prevV = p.velocity
            if (p.position == 2.0 && p.velocity == 0.0) {
                break
            }
        }
        assertEquals(2.0, p.position, 0.0)
    }

    @Test
    fun jitteryDtStillLandsWithBoundedJerk() {
        val maxJ = 20.0
        val p = RuckigProfiler(2.0, 4.0, 4.0, maxJ, 0.0)
        val rng = Random(42)
        var prevA = p.acceleration
        var steps = 0
        while (steps < 2000) {
            val dt = 0.005 + rng.nextDouble() * 0.020 // 5–25 ms
            p.update(-2.0, dt)
            steps++
            assertTrue(
                abs(p.acceleration - prevA) <= maxJ * dt * 1.05 + 1e-9,
                "jerk bounded under dt jitter at step $steps",
            )
            prevA = p.acceleration
            if (p.position == -2.0 && p.velocity == 0.0) {
                break
            }
        }
        assertEquals(-2.0, p.position, 0.0)
        assertTrue(steps < 2000, "settled")
    }

    @Test
    fun invalidDtAndTargetAreNoOps() {
        val p = RuckigProfiler(2.0, 4.0, 4.0, 20.0, 5.0)
        p.update(1.0, DT) // move a little
        val pos = p.position
        val vel = p.velocity
        p.update(1.0, 0.0)
        p.update(1.0, -0.1)
        p.update(1.0, Double.NaN)
        p.update(Double.NaN, DT)
        p.update(Double.POSITIVE_INFINITY, DT)
        assertEquals(pos, p.position, 0.0, "no-op inputs leave state untouched")
        assertEquals(vel, p.velocity, 0.0)
    }

    @Test
    fun resetSnapsToRest() {
        val p = RuckigProfiler(2.0, 4.0, 4.0, 20.0, 0.0)
        for (i in 0 until 20) {
            p.update(5.0, DT)
        }
        p.reset(1.25)
        assertEquals(1.25, p.position, 0.0)
        assertEquals(0.0, p.velocity, 0.0)
        assertEquals(0.0, p.acceleration, 0.0)
    }

    @Test
    fun validatesLimitsAtConstructionAndRewrite() {
        assertThrows(IllegalArgumentException::class.java) {
            RuckigProfiler(-1.0, 1.0, 1.0, 1.0, 0.0)
        }
        assertThrows(
            IllegalArgumentException::class.java,
            {
                RuckigProfiler(1.0, 1.0, 1.0, 0.0, 0.0)
            },
            "zero jerk rejected",
        )
        assertThrows(IllegalArgumentException::class.java) {
            RuckigProfiler(1.0, 1.0, 1.0, Double.NaN, 0.0)
        }
        val p = RuckigProfiler(1.0, 1.0, 1.0, 1.0, 0.0)
        assertThrows(IllegalArgumentException::class.java) { p.setMaxDeceleration(-2.0) }
        p.setMaxVelocity(0.0) // zero is a legitimate ceiling ("no authority")
    }

    companion object {
        private const val DT = 0.016 // typical FTC loop
    }
}
