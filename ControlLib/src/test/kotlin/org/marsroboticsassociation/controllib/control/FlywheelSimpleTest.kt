package org.marsroboticsassociation.controllib.control

import java.util.Random
import java.util.function.LongSupplier
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim

class FlywheelSimpleTest {

    // ── setup ─────────────────────────────────────────────────────────────────

    companion object {
        private fun makeSim(): FlywheelMotorSim {
            val kS = FlywheelSimple.PARAMS.kS
            val sim = FlywheelTestFixture.makeSim()
            sim.setDisturbanceVoltage(-kS) // cancel kS feedforward → zero steady-state error
            return sim
        }

        private fun makeSystem(
            adapter: FlywheelTestFixture.SimMotorAdapter,
            timeSecs: DoubleArray,
        ): FlywheelSimple {
            val clock = LongSupplier { (timeSecs[0] * 1e9).toLong() }
            return FlywheelSimple({ _, _, _ -> }, clock, adapter)
        }

        /**
         * Advance clock by a normally-distributed dt (mean 20 ms, σ 4 ms), step controller and
         * plant. Returns the actual dt in seconds so callers can accumulate real elapsed time.
         */
        private fun step(
            flywheel: FlywheelSimple,
            adapter: FlywheelTestFixture.SimMotorAdapter,
            sim: FlywheelMotorSim,
            timeSecs: DoubleArray,
            rng: Random,
        ): Double {
            val dt = maxOf(0.001, 0.020 + rng.nextGaussian() * 0.004)
            timeSecs[0] += dt
            flywheel.update()
            sim.step(dt, adapter.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
            return dt
        }
    }

    // ── tests ─────────────────────────────────────────────────────────────────

    @Test
    fun testSpinUpConverges() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val flywheel = makeSystem(adapter, timeSecs)

        flywheel.setTps(2000.0)
        flywheel.update() // seed lastTimeNanos; returns early

        val rng = FlywheelTestFixture.makeRng()
        var firstReadySeconds = -1.0
        var elapsedSeconds = 0.0
        var slewTime = 0.0
        var approachTime = 0.0
        for (i in 0 until 800) {
            val pv = flywheel.getProfiledVelocity()
            val dt = step(flywheel, adapter, sim, timeSecs, rng)
            elapsedSeconds += dt
            if (pv < 2000.0) {
                val accelLimit =
                    (FlywheelTestFixture.HUB_VOLTAGE -
                        FlywheelSimple.PARAMS.kS -
                        FlywheelSimple.PARAMS.kV * pv) / FlywheelSimple.PARAMS.kA
                if (FlywheelSimple.PARAMS.maxAccel < accelLimit) slewTime += dt
                else approachTime += dt
            }
            if (firstReadySeconds < 0 && flywheel.isReady()) firstReadySeconds = elapsedSeconds
        }
        System.out.printf(
            "testSpinUpConverges: isReady() at %.2f s  (slew: %.2f s, approach: %.2f s)%n",
            firstReadySeconds,
            slewTime,
            approachTime,
        )

        assertTrue(flywheel.isReady(), "flywheel should be ready after 16 s of simulated spin-up")
        assertEquals(
            2000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "true velocity should be within 30 TPS of setpoint",
        )
    }

    @Test
    fun testCoastsWhenSetpointZero() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val flywheel = makeSystem(adapter, timeSecs)

        flywheel.setTps(2000.0)
        flywheel.update() // seed

        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 10) step(flywheel, adapter, sim, timeSecs, rng)

        flywheel.setTps(0.0)
        step(flywheel, adapter, sim, timeSecs, rng)

        assertEquals(0.0, adapter.lastPower, "motor power should be zero when coasting")
        assertFalse(flywheel.isReady(), "isReady() must be false when setpoint is zero")
    }

    @Test
    fun testSetpointStepDown() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val flywheel = makeSystem(adapter, timeSecs)

        flywheel.setTps(2000.0)
        flywheel.update() // seed

        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 800) step(flywheel, adapter, sim, timeSecs, rng)
        assertTrue(flywheel.isReady(), "should be ready at 2000 TPS before step")

        flywheel.setTps(1000.0)
        for (i in 0 until 300) step(flywheel, adapter, sim, timeSecs, rng)

        System.out.printf(
            "testSetpointStepDown: true vel=%.1f TPS, isReady=%b%n",
            sim.getTrueVelocityTps(),
            flywheel.isReady(),
        )

        assertTrue(flywheel.isReady(), "should be ready at 1000 TPS after step")
        assertEquals(
            1000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "true velocity should be within 30 TPS of new setpoint",
        )
    }

    @Test
    fun testDisturbancePulseAndRecovery() {
        val kS = FlywheelSimple.PARAMS.kS
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val flywheel = makeSystem(adapter, timeSecs)

        flywheel.setTps(2000.0)
        flywheel.update() // seed

        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 800) step(flywheel, adapter, sim, timeSecs, rng)
        assertTrue(flywheel.isReady(), "should be ready at steady state before disturbance")

        sim.setDisturbanceVoltage(-kS - 0.1)
        for (i in 0 until 50) step(flywheel, adapter, sim, timeSecs, rng)

        sim.setDisturbanceVoltage(-kS)
        for (i in 0 until 300) step(flywheel, adapter, sim, timeSecs, rng)

        assertTrue(flywheel.isReady(), "should recover and be ready after disturbance pulse")
    }
}
