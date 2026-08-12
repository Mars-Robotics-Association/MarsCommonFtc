package org.marsroboticsassociation.controllib.motion

import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.util.TelemetryAddData

class RuckigPositionTrajectoryTest {

    companion object {
        private val NO_OP_TELEMETRY = TelemetryAddData { _, _, _ -> }
    }

    // ---------------------------------------------------------------
    // Parity with SCurvePosition where both are time-optimal
    // ---------------------------------------------------------------

    @Test
    fun restToRestDurationMatchesSCurve() {
        // Rest-to-rest with a cruise phase: both planners should find the optimal 7-phase profile.
        val sc = SCurvePosition(0.0, 2.0, 0.0, 0.0, 1.5, 2.0, 3.0, 8.0)
        val rk = RuckigPositionTrajectory(0.0, 2.0, 0.0, 0.0, 1.5, 2.0, 3.0, 8.0)

        assertEquals(sc.getTotalTime(), rk.getTotalTime(), 1e-6, "optimal duration")
        assertEquals(2.0, rk.getPosition(rk.getTotalTime()), 1e-8)
        assertEquals(0.0, rk.getVelocity(rk.getTotalTime()), 1e-8)
        assertEquals(0.0, rk.getAcceleration(rk.getTotalTime()), 1e-8)
    }

    @Test
    fun negativeDirectionMapsAsymmetricLimitsToTravelFrame() {
        // Moving in the − direction with aAccel != aDecel: a signed-frame mapping bug would show
        // up as a different duration than SCurvePosition's travel-frame plan.
        val aAccel = 1.0
        val aDecel = 4.0
        val sc = SCurvePosition(2.0, 0.0, 0.0, 0.0, 1.5, aAccel, aDecel, 8.0)
        val rk = RuckigPositionTrajectory(2.0, 0.0, 0.0, 0.0, 1.5, aAccel, aDecel, 8.0)

        assertEquals(
            sc.getTotalTime(),
            rk.getTotalTime(),
            1e-6,
            "asymmetric negative-move duration",
        )

        // Signed acceleration must stay within the mapped bounds: speeding up in − is bounded by
        // aAccel (a >= -1), braking in − is bounded by aDecel (a <= +4).
        for (i in 0..200) {
            val t = rk.getTotalTime() * i / 200.0
            val a = rk.getAcceleration(t)
            assertTrue(a >= -aAccel - 1e-9, "accel bound at t=$t: $a")
            assertTrue(a <= aDecel + 1e-9, "decel bound at t=$t: $a")
        }
    }

    // ---------------------------------------------------------------
    // Initial and target states beyond SCurvePosition's reach
    // ---------------------------------------------------------------

    @Test
    fun wrongWayInitialStateEndsAtRestOnTarget() {
        val rk = RuckigPositionTrajectory(0.0, 1.0, -0.8, 0.5, 1.0, 1.0, 1.0, 4.0)
        val tf = rk.getTotalTime()
        assertTrue(tf > 0)
        assertEquals(1.0, rk.getPosition(tf), 1e-8)
        assertEquals(0.0, rk.getVelocity(tf), 1e-8)
        assertEquals(0.0, rk.getAcceleration(tf), 1e-8)
        // Initial state is reproduced exactly at t=0
        assertEquals(0.0, rk.getPosition(0.0), 1e-12)
        assertEquals(-0.8, rk.getVelocity(0.0), 1e-12)
        assertEquals(0.5, rk.getAcceleration(0.0), 1e-12)
    }

    @Test
    fun initialVelocityOverLimitIsBrakedNotClamped() {
        // v0 far above vMax: Ruckig plans a brake pre-trajectory. Velocity must come down
        // continuously (bounded by decel+jerk), never jump.
        val rk = RuckigPositionTrajectory(0.0, 2.0, 3.0, 0.0, 1.0, 1.0, 2.0, 8.0)
        val tf = rk.getTotalTime()
        var prevV = rk.getVelocity(0.0)
        assertEquals(3.0, prevV, 1e-12)
        val n = 400
        for (i in 1..n) {
            val t = tf * i / n
            val vNow = rk.getVelocity(t)
            assertTrue(
                abs(vNow - prevV) < 2.0 * (tf / n) + 1e-6,
                "velocity continuous at t=$t",
            )
            prevV = vNow
        }
        assertEquals(2.0, rk.getPosition(tf), 1e-8)
        assertEquals(0.0, rk.getVelocity(tf), 1e-8)
    }

    @Test
    fun nonzeroTargetVelocityPassThrough() {
        val vf = 0.5
        val rk = RuckigPositionTrajectory(0.0, 2.0, 0.0, 0.0, vf, 0.0, 1.0, 1.0, 1.0, 4.0)
        val tf = rk.getTotalTime()
        assertEquals(2.0, rk.getPosition(tf), 1e-8)
        assertEquals(vf, rk.getVelocity(tf), 1e-8)
    }

    // ---------------------------------------------------------------
    // Interface conformance
    // ---------------------------------------------------------------

    @Test
    fun beforeStartAndAfterEndHoldBoundaryStates() {
        val rk = RuckigPositionTrajectory(1.0, 3.0, 0.2, 0.0, 1.0, 1.0, 1.0, 4.0)
        assertEquals(1.0, rk.getPosition(-0.5), 1e-12, "t<0 returns initial state")
        assertEquals(0.2, rk.getVelocity(-0.5), 1e-12)
        val after = rk.getTotalTime() + 5.0
        assertEquals(3.0, rk.getPosition(after), 1e-8, "t>tf holds target")
        assertEquals(0.0, rk.getVelocity(after), 1e-8)
        assertTrue(rk.isZeroJerk(after))
    }

    @Test
    fun isZeroJerkTracksProfilePhases() {
        // A long move with a cruise: jerk is nonzero in the accel ramps, zero mid-cruise.
        val rk = RuckigPositionTrajectory(0.0, 10.0, 0.0, 0.0, 1.0, 1.0, 1.0, 2.0)
        assertTrue(!rk.isZeroJerk(1e-3), "jerk active at start of accel ramp")
        assertTrue(rk.isZeroJerk(rk.getTotalTime() / 2), "cruise is zero-jerk")
    }

    @Test
    fun invalidPlanThrows() {
        // Target velocity above the velocity limit is upstream-invalid input.
        assertThrows(IllegalArgumentException::class.java) {
            RuckigPositionTrajectory(0.0, 1.0, 0.0, 0.0, 2.0, 0.0, 1.0, 1.0, 1.0, 4.0)
        }
    }

    // ---------------------------------------------------------------
    // Factory drop-in under PositionTrajectoryManager
    // ---------------------------------------------------------------

    @Test
    fun managerReplanPreservesStateContinuity() {
        val clock = AtomicLong(0)
        val m =
            PositionTrajectoryManager(
                5.0,
                3.0,
                3.0,
                10.0,
                0.01,
                NO_OP_TELEMETRY,
                clock::get,
                ::RuckigPositionTrajectory,
            )

        m.setTarget(50.0)
        clock.set(2e9.toLong()) // mid-flight, at cruise
        m.update()
        val pBefore = m.getPosition()
        val vBefore = m.getVelocity()
        val aBefore = m.getAcceleration()
        assertTrue(vBefore > 0.1, "should be moving mid-flight")

        // Retarget: replan must start from the sampled p/v/a state.
        m.setTarget(-20.0)
        val dtNs = 1e6 // 1 ms later
        clock.set((2e9 + dtNs).toLong())
        m.update()
        assertEquals(
            pBefore + vBefore * 1e-3,
            m.getPosition(),
            5 * 3 * 1e-6 + 1e-6,
            "position continuous across replan",
        )
        assertEquals(vBefore, m.getVelocity(), 3 * 1e-3 + 1e-6, "velocity continuous across replan")
        assertEquals(
            aBefore,
            m.getAcceleration(),
            10 * 1e-3 + 1e-6,
            "accel continuous across replan",
        )

        clock.set(60e9.toLong())
        m.update()
        assertEquals(-20.0, m.getPosition(), 1e-2, "reaches the new target")
        assertEquals(0.0, m.getVelocity(), 0.1, "at rest on the new target")
    }
}
