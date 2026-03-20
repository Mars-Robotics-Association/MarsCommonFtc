package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class FlywheelStateSpaceTest {

    // ── setup ─────────────────────────────────────────────────────────────────

    /**
     * 10 TPS Gaussian noise approximates encoder jitter; 20 TPS quantization matches
     * the real hub's tick-count-per-50ms velocity measurement.  Together these mean
     * the Kalman filter and LQR are exercised against imperfect measurements rather
     * than a noiseless oracle.
     */
    private static FlywheelMotorSim makeSim() {
        return FlywheelTestFixture.makeSim();
    }

    private static FlywheelStateSpace makeSystem(FlywheelTestFixture.SimMotorAdapter adapter) {
        return new FlywheelStateSpace(
                adapter,
                (caption, format, value) -> {});
    }

    /** Advance by a normally-distributed dt (mean 20 ms, σ 4 ms), step controller and plant.
     *  Returns the actual dt in seconds. */
    private static double step(FlywheelStateSpace flywheel, FlywheelTestFixture.SimMotorAdapter adapter,
                                FlywheelMotorSim sim, Random rng) {
        double dt = Math.max(0.001, 0.020 + rng.nextGaussian() * 0.004);
        flywheel.update(dt, FlywheelTestFixture.HUB_VOLTAGE);
        sim.step(dt, adapter.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
        return dt;
    }

    // ── tests ─────────────────────────────────────────────────────────────────

    @Test
    void testSpinUpConverges() {
        FlywheelMotorSim                   sim      = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter  = new FlywheelTestFixture.SimMotorAdapter(sim);
        FlywheelStateSpace                 flywheel = makeSystem(adapter);

        flywheel.setTps(2000);

        Random rng = FlywheelTestFixture.makeRng();
        double elapsedSeconds = 0;
        double firstConvergedSeconds = -1;
        for (int i = 0; i < 800; i++) {
            elapsedSeconds += step(flywheel, adapter, sim, rng);
            if (firstConvergedSeconds < 0 && Math.abs(sim.getTrueVelocityTps() - 2000) < 30) {
                firstConvergedSeconds = elapsedSeconds;
            }
        }
        System.out.printf("testSpinUpConverges: within 30 TPS at %.2f s%n", firstConvergedSeconds);

        assertTrue(flywheel.isReady(), "isReady() should be true after spin-up converges");
        assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should be within 30 TPS of setpoint after 16 s of simulated spin-up");
    }

    @Test
    void testCoastsWhenSetpointZero() {
        FlywheelMotorSim                   sim      = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter  = new FlywheelTestFixture.SimMotorAdapter(sim);
        FlywheelStateSpace                 flywheel = makeSystem(adapter);

        flywheel.setTps(2000);
        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 10; i++) step(flywheel, adapter, sim, rng);

        flywheel.setTps(0);
        step(flywheel, adapter, sim, rng);

        assertEquals(0.0, adapter.lastPower, "motor power should be zero when target TPS is 0");
        assertFalse(flywheel.isReady(), "isReady() must be false when setpoint is zero");
    }

    @Test
    void testDisturbancePulseAndRecovery() {
        FlywheelMotorSim                   sim      = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter  = new FlywheelTestFixture.SimMotorAdapter(sim);
        FlywheelStateSpace                 flywheel = makeSystem(adapter);

        flywheel.setTps(2000);
        Random rng = FlywheelTestFixture.makeRng();

        for (int i = 0; i < 800; i++) step(flywheel, adapter, sim, rng);
        assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0,
                "should be near setpoint before disturbance");

        sim.setDisturbanceVoltage(-0.5);
        for (int i = 0; i < 50; i++) step(flywheel, adapter, sim, rng);

        sim.setDisturbanceVoltage(0.0);
        for (int i = 0; i < 300; i++) step(flywheel, adapter, sim, rng);

        assertTrue(flywheel.isReady(), "isReady() should be true after disturbance recovery");
        assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should recover within 30 TPS of setpoint after disturbance pulse");
    }

    @Test
    void testSetpointStepDown() {
        FlywheelMotorSim                   sim      = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter  = new FlywheelTestFixture.SimMotorAdapter(sim);
        FlywheelStateSpace                 flywheel = makeSystem(adapter);

        flywheel.setTps(2000);
        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 800; i++) step(flywheel, adapter, sim, rng);
        assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0,
                "should be near 2000 TPS before step");

        flywheel.setTps(1000);
        for (int i = 0; i < 300; i++) step(flywheel, adapter, sim, rng);

        System.out.printf("testSetpointStepDown: true vel=%.1f TPS, estimated=%.1f TPS%n",
                sim.getTrueVelocityTps(), flywheel.getEstimatedTps());

        assertTrue(flywheel.isReady(), "isReady() should be true after settling at new setpoint");
        assertEquals(1000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should be within 30 TPS of new setpoint after step");
    }

    @Test
    void testKalmanEstimateTracksTrue() {
        FlywheelMotorSim                   sim      = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter  = new FlywheelTestFixture.SimMotorAdapter(sim);
        FlywheelStateSpace                 flywheel = makeSystem(adapter);

        flywheel.setTps(2000);
        Random rng = FlywheelTestFixture.makeRng();

        for (int i = 0; i < 800; i++) step(flywheel, adapter, sim, rng);

        double estimatedTps = flywheel.getEstimatedTps();
        double trueTps = sim.getTrueVelocityTps();
        System.out.printf("testKalmanEstimateTracksTrue: estimated=%.1f TPS, true=%.1f TPS%n",
                estimatedTps, trueTps);

        assertEquals(trueTps, estimatedTps, 25.0,
                "Kalman estimate should track true velocity within 25 TPS at steady state (20 TPS quantization floor)");
    }
}
