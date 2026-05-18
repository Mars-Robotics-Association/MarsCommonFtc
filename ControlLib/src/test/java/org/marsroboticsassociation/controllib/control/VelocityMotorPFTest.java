package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;

import java.util.Random;
import java.util.function.LongSupplier;

import static org.junit.jupiter.api.Assertions.*;

class VelocityMotorPFTest {

    // ── setup ─────────────────────────────────────────────────────────────────

    /** kS from the default config — must match to cancel the feedforward in the sim. */
    private static final double kS = new VelocityMotorPF.VelocityMotorPFConfig().kS;

    /**
     * 10 TPS Gaussian noise + 20 TPS quantization (matches FlywheelStateSpaceTest).
     * kS disturbance cancelled so the feedforward yields zero steady-state error.
     */
    private static FlywheelMotorSim makeSim() {
        FlywheelMotorSim sim = FlywheelTestFixture.makeSim();
        sim.setDisturbanceVoltage(-kS);
        return sim;
    }

    /**
     * Construct a VelocityMotorPF wired to {@code adapter} with a simulated clock.
     * Uses gearRatio=1 and motorPPR=1 so TPS is the native unit throughout.
     */
    private static VelocityMotorPF makeSystem(FlywheelTestFixture.SimMotorAdapter adapter,
                                              double[] timeSecs) {
        return makeSystemWithConfig(adapter, timeSecs, new VelocityMotorPF.VelocityMotorPFConfig());
    }

    /** Like {@link #makeSystem} but accepts a custom config. */
    private static VelocityMotorPF makeSystemWithConfig(FlywheelTestFixture.SimMotorAdapter adapter,
                                                        double[] timeSecs,
                                                        VelocityMotorPF.VelocityMotorPFConfig config) {
        LongSupplier clock = () -> (long) (timeSecs[0] * 1e9);
        return new VelocityMotorPF(
                (caption, format, value) -> {},
                /*gearRatio=*/1.0,
                /*motorPPR=*/1.0,
                /*motorPowerChangeTolerance=*/0.005,
                config,
                adapter,
                clock);
    }

    /**
     * Advance the simulated clock by a normally-distributed dt (mean 20 ms, σ 4 ms),
     * then step the controller and plant. Returns the actual dt in seconds.
     */
    private static double step(VelocityMotorPF controller,
                                FlywheelTestFixture.SimMotorAdapter adapter,
                                FlywheelMotorSim sim, double[] timeSecs, Random rng) {
        double dt = Math.max(0.001, 0.020 + rng.nextGaussian() * 0.004);
        timeSecs[0] += dt;
        controller.update(dt);
        sim.step(dt, adapter.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
        return dt;
    }

    // ── tests ─────────────────────────────────────────────────────────────────

    @Test
    void testSpinUpConverges() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = { 1.0 };
        VelocityMotorPF controller = makeSystem(adapter, timeSecs);

        controller.setTPS(2000);

        Random rng = FlywheelTestFixture.makeRng();
        double elapsedSeconds = 0;
        double firstAtSpeedSeconds = -1;
        for (int i = 0; i < 800; i++) {
            elapsedSeconds += step(controller, adapter, sim, timeSecs, rng);
            if (firstAtSpeedSeconds < 0 && controller.isAtTargetSpeed()) {
                firstAtSpeedSeconds = elapsedSeconds;
            }
        }
        System.out.printf("testSpinUpConverges: isAtTargetSpeed() at %.2f s%n", firstAtSpeedSeconds);

        assertTrue(controller.isAtTargetSpeed(),
                "controller should be at target speed after ~16 s of simulated spin-up");
        assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should be within 30 TPS of setpoint");
    }

    @Test
    void testCoastsWhenSetpointZero() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = { 1.0 };
        VelocityMotorPF controller = makeSystem(adapter, timeSecs);

        controller.setTPS(2000);
        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 10; i++) step(controller, adapter, sim, timeSecs, rng);

        controller.setTPS(0);
        step(controller, adapter, sim, timeSecs, rng);

        assertEquals(0.0, adapter.lastPower, "motor power should be zero when setpoint is zero");
        assertFalse(controller.isAtTargetSpeed(),
                "isAtTargetSpeed() must be false when setpoint is zero");
    }

    @Test
    void testSetpointStepDown() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = { 1.0 };
        VelocityMotorPF controller = makeSystem(adapter, timeSecs);

        controller.setTPS(2000);
        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 800; i++) step(controller, adapter, sim, timeSecs, rng);
        assertTrue(controller.isAtTargetSpeed(), "should be at speed at 2000 TPS before step");

        controller.setTPS(1000);
        for (int i = 0; i < 300; i++) step(controller, adapter, sim, timeSecs, rng);

        System.out.printf("testSetpointStepDown: true vel=%.1f TPS, isAtTargetSpeed=%b%n",
                sim.getTrueVelocityTps(), controller.isAtTargetSpeed());

        assertTrue(controller.isAtTargetSpeed(), "should be at speed at 1000 TPS after step");
        assertEquals(1000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should be within 30 TPS of new setpoint");
    }

    @Test
    void testDisturbancePulseAndRecovery() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = { 1.0 };
        VelocityMotorPF controller = makeSystem(adapter, timeSecs);

        controller.setTPS(2000);
        Random rng = FlywheelTestFixture.makeRng();

        for (int i = 0; i < 800; i++) step(controller, adapter, sim, timeSecs, rng);
        assertTrue(controller.isAtTargetSpeed(), "should be at speed before disturbance");

        sim.setDisturbanceVoltage(-kS - 0.1);
        for (int i = 0; i < 50; i++) step(controller, adapter, sim, timeSecs, rng);

        sim.setDisturbanceVoltage(-kS);
        for (int i = 0; i < 300; i++) step(controller, adapter, sim, timeSecs, rng);

        assertTrue(controller.isAtTargetSpeed(),
                "should recover and be at speed after disturbance pulse");
    }

    @Test
    void testIsAtTargetSpeedFalseWhileRamping() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = { 1.0 };
        VelocityMotorPF controller = makeSystem(adapter, timeSecs);

        controller.setTPS(2000);
        Random rng = FlywheelTestFixture.makeRng();
        step(controller, adapter, sim, timeSecs, rng);
        step(controller, adapter, sim, timeSecs, rng);

        assertFalse(controller.isAtTargetSpeed(),
                "isAtTargetSpeed() should be false while the jerk-limited trajectory is still ramping");
    }

    @Test
    void testKpSuppressedDuringAcceleration() {
        VelocityMotorPF.VelocityMotorPFConfig zeroKpConfig = new VelocityMotorPF.VelocityMotorPFConfig();
        zeroKpConfig.kP = 0.0;
        zeroKpConfig.accelMax = 500;
        VelocityMotorPF.VelocityMotorPFConfig defaultKpConfig = new VelocityMotorPF.VelocityMotorPFConfig();
        defaultKpConfig.accelMax = 500;

        FlywheelMotorSim simFF = makeSim();
        FlywheelMotorSim simPF = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapterFF = new FlywheelTestFixture.SimMotorAdapter(simFF);
        FlywheelTestFixture.SimMotorAdapter adapterPF = new FlywheelTestFixture.SimMotorAdapter(simPF);
        double[] timeFF = { 1.0 };
        double[] timePF = { 1.0 };

        VelocityMotorPF controllerFF = makeSystemWithConfig(adapterFF, timeFF, zeroKpConfig);
        VelocityMotorPF controllerPF = makeSystemWithConfig(adapterPF, timePF, defaultKpConfig);

        controllerFF.setTPS(2000);
        controllerPF.setTPS(2000);

        double dt = 0.020;

        for (int i = 0; i < 15; i++) {
            timeFF[0] += dt; timePF[0] += dt;
            controllerFF.update(dt); controllerPF.update(dt);
            simFF.step(dt, adapterFF.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
            simPF.step(dt, adapterPF.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
        }

        assertEquals(adapterFF.lastPower, adapterPF.lastPower, 1e-9,
                "During acceleration phase kP is fully suppressed, so both controllers must output identical FF power");

        for (int i = 0; i < 800; i++) {
            timeFF[0] += dt; timePF[0] += dt;
            controllerFF.update(dt); controllerPF.update(dt);
            simFF.step(dt, adapterFF.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
            simPF.step(dt, adapterPF.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
        }

        simFF.setDisturbanceVoltage(-kS - 2.0);
        simPF.setDisturbanceVoltage(-kS - 2.0);

        for (int i = 0; i < 50; i++) {
            timeFF[0] += dt; timePF[0] += dt;
            controllerFF.update(dt); controllerPF.update(dt);
            simFF.step(dt, adapterFF.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
            simPF.step(dt, adapterPF.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
        }

        assertTrue(adapterPF.lastPower > adapterFF.lastPower,
                "At steady state with disturbance, nonzero kP should produce more corrective power than kP=0");
    }

    @Test
    void testStopCutsPower() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = { 1.0 };
        VelocityMotorPF controller = makeSystem(adapter, timeSecs);

        controller.setTPS(2000);
        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 10; i++) step(controller, adapter, sim, timeSecs, rng);

        controller.stop();

        assertEquals(0.0, adapter.lastPower, "stop() should set motor power to zero");
        assertFalse(controller.isAtTargetSpeed(),
                "isAtTargetSpeed() should be false after stop()");
    }
}
