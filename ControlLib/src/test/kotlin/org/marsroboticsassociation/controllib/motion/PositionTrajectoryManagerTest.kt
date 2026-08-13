package org.marsroboticsassociation.controllib.motion

import java.util.concurrent.atomic.AtomicLong
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.util.TelemetryAddData

class PositionTrajectoryManagerTest {

    companion object {
        // Silent telemetry stub
        private val NO_OP_TELEMETRY = TelemetryAddData { _, _, _ -> }

        /** Build a manager with a controllable clock (nanoseconds). */
        private fun makeManager(
            clockNs: AtomicLong,
            vMax: Double,
            aMaxAccel: Double,
            aMaxDecel: Double,
            jMax: Double,
            pTol: Double,
        ): PositionTrajectoryManager =
            PositionTrajectoryManager(
                vMax,
                aMaxAccel,
                aMaxDecel,
                jMax,
                pTol,
                NO_OP_TELEMETRY,
                clockNs::get,
            )

        private fun makeManager(
            clockNs: AtomicLong,
            vMax: Double,
            aMaxAccel: Double,
            aMaxDecel: Double,
            jMax: Double,
            pTol: Double,
            factory: PositionTrajectoryManager.TrajectoryFactory,
        ): PositionTrajectoryManager =
            PositionTrajectoryManager(
                vMax,
                aMaxAccel,
                aMaxDecel,
                jMax,
                pTol,
                NO_OP_TELEMETRY,
                clockNs::get,
                factory,
            )
    }

    // ---------------------------------------------------------------

    @Test
    fun setTarget_plansTrajectory_positionMovesTowardTarget() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 5.0, 3.0, 3.0, 10.0, 0.01)

        m.setTarget(50.0)

        // Estimate an upper bound on travel time: 50 / 5 = 10 s, add 4 s margin
        val tfNs = 14e9.toLong()
        clock.set(tfNs)
        m.update()

        assertEquals(50.0, m.position, 1e-2, "should reach target after tf")
        assertEquals(0.0, m.velocity, 0.1, "should be at rest")
    }

    @Test
    fun resetFromMeasurement_rebuildsFromInjectedState() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 5.0, 3.0, 3.0, 10.0, 0.01)

        m.setTarget(100.0)

        // Advance partway, then inject a measurement
        clock.set(2e9.toLong())
        m.update()
        m.resetFromMeasurement(20.0, 0.0, 0.0)

        // Verify getters reflect injected state immediately
        assertEquals(20.0, m.position, 1e-9)
        assertEquals(0.0, m.velocity, 1e-9)

        // Advance well past end of new trajectory
        clock.set(30e9.toLong())
        m.update()
        assertEquals(100.0, m.position, 1e-2, "should still reach original target")
    }

    @Test
    fun targetChange_withinTolerance_doesNotReplan() {
        val clock = AtomicLong(0)
        // Tolerance = 1.0 unit
        val m = makeManager(clock, 5.0, 3.0, 3.0, 10.0, 1.0)

        m.setTarget(50.0)

        // Advance a bit so the trajectory starts
        clock.set(0.5e9.toLong())
        m.update()
        val posAfterFirst = m.position

        // Command a target within tolerance — should NOT replan
        m.setTarget(50.4)

        clock.set(1e9.toLong())
        m.update()

        // If replanning had occurred, the position trajectory would restart from 0
        // and the value would likely be < posAfterFirst. Without replan, it continues.
        assertTrue(
            m.position >= posAfterFirst,
            "position should continue increasing (no replan within tolerance)",
        )
    }

    @Test
    fun updateConfig_affectsNextTrajectory() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 5.0, 3.0, 3.0, 10.0, 0.01)

        m.setTarget(100.0)
        clock.set(1e9.toLong())
        m.update()
        val tfFast = SCurvePosition(0.0, 100.0, 0.0, 0.0, 5.0, 3.0, 3.0, 10.0).totalTime

        // Slow it down
        m.updateConfig(2.0, 1.0, 1.0, 5.0)
        m.resetFromMeasurement(0.0, 0.0, 0.0)

        val tfSlow = SCurvePosition(0.0, 100.0, 0.0, 0.0, 2.0, 1.0, 1.0, 5.0).totalTime

        assertTrue(tfSlow > tfFast, "slower config should take longer")
        clock.set((tfSlow * 1.05e9).toLong())
        m.update()
        assertEquals(100.0, m.position, 0.5, "should reach target with slow config")
    }

    /**
     * Sanity check: changing target in the same direction while moving should produce a continuous
     * velocity at the next 20 ms tick. This test is expected to PASS today.
     */
    @Test
    fun midMove_sameDirection_velocityIsContinuous() {
        val clock = AtomicLong(0)
        // vMax=10, aMax=50, jMax=500 → reaches cruise (a=0, v=vMax) in ~0.3 s;
        // at t=1 s we are in steady cruise with a0≈0 so tPrefix=0 on replan.
        val m = makeManager(clock, 10.0, 50.0, 50.0, 500.0, 0.01)

        m.setTarget(100.0)
        clock.set(1e9.toLong())
        m.update()
        val vBefore = m.velocity

        // Change target to +50 (same direction, still forward)
        m.setTarget(50.0)
        clock.set(1e9.toLong() + 20_000_000L) // +20 ms
        m.update()
        val vAfter = m.velocity

        assertEquals(
            vBefore,
            vAfter,
            2.0,
            String.format(
                "same-direction replan should be continuous: vBefore=%.3f vAfter=%.3f diff=%.3f",
                vBefore,
                vAfter,
                abs(vAfter - vBefore),
            ),
        )
    }

    /**
     * Regression test for mid-move target reversal velocity discontinuity.
     *
     * <p>When the target reverses mid-move, the braking prefix in `SCurvePosition` ensures velocity
     * continuity by decelerating to 0 before starting the 7-phase section toward the new target.
     *
     * <p>Parameters are chosen so the trajectory is in steady cruise (a≈0) at t=1s, ensuring
     * tPrefix=0 on replan and the continuity is immediately verifiable.
     */
    @Test
    fun midMove_targetReversal_velocityIsContinuous() {
        val clock = AtomicLong(0)
        // Same fast-accel params: at t=1s we are cruising at v≈10, a≈0
        val m = makeManager(clock, 10.0, 50.0, 50.0, 500.0, 0.01)

        m.setTarget(100.0)
        clock.set(1e9.toLong())
        m.update()
        val vBefore = m.velocity

        // Reverse target mid-move (vBefore ≈ +10, new direction is negative)
        m.setTarget(-100.0)
        clock.set(1e9.toLong() + 20_000_000L) // +20 ms
        m.update()
        val vAfter = m.velocity

        // BUG: vAfter ≈ 0 instead of continuing from vBefore ≈ +10
        assertEquals(
            vBefore,
            vAfter,
            2.0,
            String.format(
                "reversal replan should be continuous: vBefore=%.3f vAfter=%.3f diff=%.3f",
                vBefore,
                vAfter,
                abs(vAfter - vBefore),
            ),
        )
    }

    @Test
    fun sinCurve_midMove_sameDirection_velocityIsContinuous() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 10.0, 50.0, 50.0, 500.0, 0.01, ::SinCurvePosition)

        m.setTarget(100.0)
        clock.set(1e9.toLong())
        m.update()
        val vBefore = m.velocity

        m.setTarget(50.0)
        clock.set(1e9.toLong() + 20_000_000L)
        m.update()
        val vAfter = m.velocity

        assertEquals(
            vBefore,
            vAfter,
            2.0,
            String.format(
                "SinCurve same-direction replan should be continuous: vBefore=%.3f vAfter=%.3f diff=%.3f",
                vBefore,
                vAfter,
                abs(vAfter - vBefore),
            ),
        )
    }

    @Test
    fun sinCurve_midMove_targetReversal_velocityIsContinuous() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 10.0, 50.0, 50.0, 500.0, 0.01, ::SinCurvePosition)

        m.setTarget(100.0)
        clock.set(1e9.toLong())
        m.update()
        val vBefore = m.velocity

        m.setTarget(-100.0)
        clock.set(1e9.toLong() + 20_000_000L)
        m.update()
        val vAfter = m.velocity

        assertEquals(
            vBefore,
            vAfter,
            2.0,
            String.format(
                "SinCurve reversal replan should be continuous: vBefore=%.3f vAfter=%.3f diff=%.3f",
                vBefore,
                vAfter,
                abs(vAfter - vBefore),
            ),
        )
    }

    @Test
    fun sinCurve_lateRetargetWhileAlreadyBraking_doesNotAddOvershoot() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 10.0, 5.0, 5.0, 50.0, 0.01, ::SinCurvePosition)

        m.resetFromMeasurement(-95.0, -5.0, 5.0)
        m.setTarget(100.0)

        for (i in 1..50) {
            clock.set(i * 20_000_000L)
            m.update()
        }

        assertEquals(-97.5, m.position, 1e-2, "should stop with minimum overshoot")
        assertEquals(0.0, m.velocity, 1e-2, "should be stopped after 1 second")
        assertEquals(5.0, m.acceleration, 1e-2, "should keep helpful braking acceleration")
    }

    @Test
    fun getters_returnCachedValues_fromLastUpdate() {
        val clock = AtomicLong(0)
        val m = makeManager(clock, 5.0, 3.0, 3.0, 10.0, 0.01)

        m.setTarget(50.0)
        clock.set(1e9.toLong())
        m.update()

        // Advance clock further WITHOUT calling update — getters should return stale values
        clock.set(5e9.toLong())

        val p = m.position
        val v = m.velocity
        val a = m.acceleration

        // Call update now
        m.update()

        // After update the position should have advanced (trajectory is in progress)
        assertTrue(
            m.position > p,
            "position should increase after update at later time; cached=$p" + " new=${m.position}",
        )
        assertTrue(m.velocity.isFinite())
        assertTrue(m.acceleration.isFinite())
    }
}
