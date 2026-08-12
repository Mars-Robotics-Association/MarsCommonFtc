package org.marsroboticsassociation.controllib.control

import java.util.Random
import kotlin.math.abs
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim

class FlywheelStateSpaceTest {

    // ── setup ─────────────────────────────────────────────────────────────────

    companion object {
        /**
         * 10 TPS Gaussian noise approximates encoder jitter; 20 TPS quantization matches the real
         * hub's tick-count-per-50ms velocity measurement. Together these mean the Kalman filter and
         * LQR are exercised against imperfect measurements rather than a noiseless oracle.
         */
        private fun makeSim(): FlywheelMotorSim = FlywheelTestFixture.makeSim()

        private fun makeSystem(adapter: FlywheelTestFixture.SimMotorAdapter): FlywheelStateSpace =
            FlywheelStateSpace(adapter) { _, _, _ -> }

        /**
         * Advance by a normally-distributed dt (mean 20 ms, σ 4 ms), step controller and plant.
         * Returns the actual dt in seconds.
         */
        private fun step(
            flywheel: FlywheelStateSpace,
            adapter: FlywheelTestFixture.SimMotorAdapter,
            sim: FlywheelMotorSim,
            rng: Random,
        ): Double {
            val dt = maxOf(0.001, 0.020 + rng.nextGaussian() * 0.004)
            flywheel.update(dt, FlywheelTestFixture.HUB_VOLTAGE)
            sim.step(dt, adapter.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
            return dt
        }
    }

    // ── tests ─────────────────────────────────────────────────────────────────

    @Test
    fun testSpinUpConverges() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val flywheel = makeSystem(adapter)

        flywheel.setTps(2000.0)

        val rng = FlywheelTestFixture.makeRng()
        var elapsedSeconds = 0.0
        var firstConvergedSeconds = -1.0
        for (i in 0 until 800) {
            elapsedSeconds += step(flywheel, adapter, sim, rng)
            if (firstConvergedSeconds < 0 && abs(sim.getTrueVelocityTps() - 2000) < 30) {
                firstConvergedSeconds = elapsedSeconds
            }
        }
        System.out.printf("testSpinUpConverges: within 30 TPS at %.2f s%n", firstConvergedSeconds)

        assertTrue(flywheel.isReady(), "isReady() should be true after spin-up converges")
        assertEquals(
            2000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "true velocity should be within 30 TPS of setpoint after 16 s of simulated spin-up",
        )
    }

    @Test
    fun testCoastsWhenSetpointZero() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val flywheel = makeSystem(adapter)

        flywheel.setTps(2000.0)
        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 10) step(flywheel, adapter, sim, rng)

        flywheel.setTps(0.0)
        step(flywheel, adapter, sim, rng)

        assertEquals(0.0, adapter.lastPower, "motor power should be zero when target TPS is 0")
        assertFalse(flywheel.isReady(), "isReady() must be false when setpoint is zero")
    }

    @Test
    fun testDisturbancePulseAndRecovery() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val flywheel = makeSystem(adapter)

        flywheel.setTps(2000.0)
        val rng = FlywheelTestFixture.makeRng()

        for (i in 0 until 800) step(flywheel, adapter, sim, rng)
        assertEquals(
            2000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "should be near setpoint before disturbance",
        )

        sim.setDisturbanceVoltage(-0.5)
        for (i in 0 until 50) step(flywheel, adapter, sim, rng)

        sim.setDisturbanceVoltage(0.0)
        for (i in 0 until 300) step(flywheel, adapter, sim, rng)

        assertTrue(flywheel.isReady(), "isReady() should be true after disturbance recovery")
        assertEquals(
            2000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "true velocity should recover within 30 TPS of setpoint after disturbance pulse",
        )
    }

    @Test
    fun testSetpointStepDown() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val flywheel = makeSystem(adapter)

        flywheel.setTps(2000.0)
        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 800) step(flywheel, adapter, sim, rng)
        assertEquals(
            2000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "should be near 2000 TPS before step",
        )

        flywheel.setTps(1000.0)
        for (i in 0 until 300) step(flywheel, adapter, sim, rng)

        System.out.printf(
            "testSetpointStepDown: true vel=%.1f TPS, estimated=%.1f TPS%n",
            sim.getTrueVelocityTps(),
            flywheel.getEstimatedTps(),
        )

        assertTrue(flywheel.isReady(), "isReady() should be true after settling at new setpoint")
        assertEquals(
            1000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "true velocity should be within 30 TPS of new setpoint after step",
        )
    }

    @Test
    fun testKalmanEstimateTracksTrue() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val flywheel = makeSystem(adapter)

        flywheel.setTps(2000.0)
        val rng = FlywheelTestFixture.makeRng()

        for (i in 0 until 800) step(flywheel, adapter, sim, rng)

        val estimatedTps = flywheel.getEstimatedTps()
        val trueTps = sim.getTrueVelocityTps()
        System.out.printf(
            "testKalmanEstimateTracksTrue: estimated=%.1f TPS, true=%.1f TPS%n",
            estimatedTps,
            trueTps,
        )

        assertEquals(
            trueTps,
            estimatedTps,
            25.0,
            "Kalman estimate should track true velocity within 25 TPS at steady state (20 TPS quantization floor)",
        )
    }
}
