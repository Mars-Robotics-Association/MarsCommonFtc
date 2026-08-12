package org.marsroboticsassociation.controllib.control

import java.util.Random
import java.util.function.LongSupplier
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim

class VelocityMotorPFTest {

    // ── setup ─────────────────────────────────────────────────────────────────

    companion object {
        /** kS from the default config — must match to cancel the feedforward in the sim. */
        private val kS = VelocityMotorPF.VelocityMotorPFConfig().kS

        /**
         * 10 TPS Gaussian noise + 20 TPS quantization (matches FlywheelStateSpaceTest). kS
         * disturbance cancelled so the feedforward yields zero steady-state error.
         */
        private fun makeSim(): FlywheelMotorSim {
            val sim = FlywheelTestFixture.makeSim()
            sim.setDisturbanceVoltage(-kS)
            return sim
        }

        /**
         * Construct a VelocityMotorPF wired to [adapter] with a simulated clock. Uses gearRatio=1
         * and motorPPR=1 so TPS is the native unit throughout.
         */
        private fun makeSystem(
            adapter: FlywheelTestFixture.SimMotorAdapter,
            timeSecs: DoubleArray,
        ): VelocityMotorPF =
            makeSystemWithConfig(adapter, timeSecs, VelocityMotorPF.VelocityMotorPFConfig())

        /** Like [makeSystem] but accepts a custom config. */
        private fun makeSystemWithConfig(
            adapter: FlywheelTestFixture.SimMotorAdapter,
            timeSecs: DoubleArray,
            config: VelocityMotorPF.VelocityMotorPFConfig,
        ): VelocityMotorPF {
            val clock = LongSupplier { (timeSecs[0] * 1e9).toLong() }
            return VelocityMotorPF(
                { _, _, _ -> },
                /*gearRatio=*/ 1.0,
                /*motorPPR=*/ 1.0,
                /*motorPowerChangeTolerance=*/ 0.005,
                config,
                adapter,
                clock,
            )
        }

        /**
         * Advance the simulated clock by a normally-distributed dt (mean 20 ms, σ 4 ms), then step
         * the controller and plant. Returns the actual dt in seconds.
         */
        private fun step(
            controller: VelocityMotorPF,
            adapter: FlywheelTestFixture.SimMotorAdapter,
            sim: FlywheelMotorSim,
            timeSecs: DoubleArray,
            rng: Random,
        ): Double {
            val dt = maxOf(0.001, 0.020 + rng.nextGaussian() * 0.004)
            timeSecs[0] += dt
            controller.update(dt)
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
        val controller = makeSystem(adapter, timeSecs)

        controller.setTPS(2000.0)

        val rng = FlywheelTestFixture.makeRng()
        var elapsedSeconds = 0.0
        var firstAtSpeedSeconds = -1.0
        for (i in 0 until 800) {
            elapsedSeconds += step(controller, adapter, sim, timeSecs, rng)
            if (firstAtSpeedSeconds < 0 && controller.isAtTargetSpeed()) {
                firstAtSpeedSeconds = elapsedSeconds
            }
        }
        System.out.printf("testSpinUpConverges: isAtTargetSpeed() at %.2f s%n", firstAtSpeedSeconds)

        assertTrue(
            controller.isAtTargetSpeed(),
            "controller should be at target speed after ~16 s of simulated spin-up",
        )
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
        val controller = makeSystem(adapter, timeSecs)

        controller.setTPS(2000.0)
        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 10) step(controller, adapter, sim, timeSecs, rng)

        controller.setTPS(0.0)
        step(controller, adapter, sim, timeSecs, rng)

        assertEquals(0.0, adapter.lastPower, "motor power should be zero when setpoint is zero")
        assertFalse(
            controller.isAtTargetSpeed(),
            "isAtTargetSpeed() must be false when setpoint is zero",
        )
    }

    @Test
    fun testSetpointStepDown() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val controller = makeSystem(adapter, timeSecs)

        controller.setTPS(2000.0)
        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 800) step(controller, adapter, sim, timeSecs, rng)
        assertTrue(controller.isAtTargetSpeed(), "should be at speed at 2000 TPS before step")

        controller.setTPS(1000.0)
        for (i in 0 until 300) step(controller, adapter, sim, timeSecs, rng)

        System.out.printf(
            "testSetpointStepDown: true vel=%.1f TPS, isAtTargetSpeed=%b%n",
            sim.getTrueVelocityTps(),
            controller.isAtTargetSpeed(),
        )

        assertTrue(controller.isAtTargetSpeed(), "should be at speed at 1000 TPS after step")
        assertEquals(
            1000.0,
            sim.getTrueVelocityTps(),
            30.0,
            "true velocity should be within 30 TPS of new setpoint",
        )
    }

    @Test
    fun testDisturbancePulseAndRecovery() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val controller = makeSystem(adapter, timeSecs)

        controller.setTPS(2000.0)
        val rng = FlywheelTestFixture.makeRng()

        for (i in 0 until 800) step(controller, adapter, sim, timeSecs, rng)
        assertTrue(controller.isAtTargetSpeed(), "should be at speed before disturbance")

        sim.setDisturbanceVoltage(-kS - 0.1)
        for (i in 0 until 50) step(controller, adapter, sim, timeSecs, rng)

        sim.setDisturbanceVoltage(-kS)
        for (i in 0 until 300) step(controller, adapter, sim, timeSecs, rng)

        assertTrue(
            controller.isAtTargetSpeed(),
            "should recover and be at speed after disturbance pulse",
        )
    }

    @Test
    fun testIsAtTargetSpeedFalseWhileRamping() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val controller = makeSystem(adapter, timeSecs)

        controller.setTPS(2000.0)
        val rng = FlywheelTestFixture.makeRng()
        step(controller, adapter, sim, timeSecs, rng)
        step(controller, adapter, sim, timeSecs, rng)

        assertFalse(
            controller.isAtTargetSpeed(),
            "isAtTargetSpeed() should be false while the jerk-limited trajectory is still ramping",
        )
    }

    @Test
    fun testKpSuppressedDuringAcceleration() {
        val zeroKpConfig = VelocityMotorPF.VelocityMotorPFConfig()
        zeroKpConfig.kP = 0.0
        zeroKpConfig.accelMax = 500.0
        val defaultKpConfig = VelocityMotorPF.VelocityMotorPFConfig()
        defaultKpConfig.accelMax = 500.0

        val simFF = makeSim()
        val simPF = makeSim()
        val adapterFF = FlywheelTestFixture.SimMotorAdapter(simFF)
        val adapterPF = FlywheelTestFixture.SimMotorAdapter(simPF)
        val timeFF = doubleArrayOf(1.0)
        val timePF = doubleArrayOf(1.0)

        val controllerFF = makeSystemWithConfig(adapterFF, timeFF, zeroKpConfig)
        val controllerPF = makeSystemWithConfig(adapterPF, timePF, defaultKpConfig)

        controllerFF.setTPS(2000.0)
        controllerPF.setTPS(2000.0)

        val dt = 0.020

        for (i in 0 until 15) {
            timeFF[0] += dt
            timePF[0] += dt
            controllerFF.update(dt)
            controllerPF.update(dt)
            simFF.step(dt, adapterFF.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
            simPF.step(dt, adapterPF.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
        }

        assertEquals(
            adapterFF.lastPower,
            adapterPF.lastPower,
            1e-9,
            "During acceleration phase kP is fully suppressed, so both controllers must output identical FF power",
        )

        for (i in 0 until 800) {
            timeFF[0] += dt
            timePF[0] += dt
            controllerFF.update(dt)
            controllerPF.update(dt)
            simFF.step(dt, adapterFF.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
            simPF.step(dt, adapterPF.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
        }

        simFF.setDisturbanceVoltage(-kS - 2.0)
        simPF.setDisturbanceVoltage(-kS - 2.0)

        for (i in 0 until 50) {
            timeFF[0] += dt
            timePF[0] += dt
            controllerFF.update(dt)
            controllerPF.update(dt)
            simFF.step(dt, adapterFF.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
            simPF.step(dt, adapterPF.lastPower, FlywheelTestFixture.HUB_VOLTAGE)
        }

        assertTrue(
            adapterPF.lastPower > adapterFF.lastPower,
            "At steady state with disturbance, nonzero kP should produce more corrective power than kP=0",
        )
    }

    @Test
    fun testStopCutsPower() {
        val sim = makeSim()
        val adapter = FlywheelTestFixture.SimMotorAdapter(sim)
        val timeSecs = doubleArrayOf(1.0)
        val controller = makeSystem(adapter, timeSecs)

        controller.setTPS(2000.0)
        val rng = FlywheelTestFixture.makeRng()
        for (i in 0 until 10) step(controller, adapter, sim, timeSecs, rng)

        controller.stop()

        assertEquals(0.0, adapter.lastPower, "stop() should set motor power to zero")
        assertFalse(controller.isAtTargetSpeed(), "isAtTargetSpeed() should be false after stop()")
    }
}
