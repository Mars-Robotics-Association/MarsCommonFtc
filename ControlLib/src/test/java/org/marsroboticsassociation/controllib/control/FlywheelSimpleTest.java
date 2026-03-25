package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;

import java.util.Random;
import java.util.function.LongSupplier;

import static org.junit.jupiter.api.Assertions.*;

class FlywheelSimpleTest {

    // ── setup ─────────────────────────────────────────────────────────────────

    private static FlywheelMotorSim makeSim() {
        double kS = FlywheelSimple.PARAMS.kS;
        FlywheelMotorSim sim = FlywheelTestFixture.makeSim();
        sim.setDisturbanceVoltage(-kS);   // cancel kS feedforward → zero steady-state error
        return sim;
    }

    private static FlywheelSimple makeSystem(FlywheelTestFixture.SimMotorAdapter adapter,
                                             double[] timeSecs) {
        LongSupplier clock = () -> (long) (timeSecs[0] * 1e9);
        return new FlywheelSimple(
                (caption, format, value) -> {},
                clock,
                adapter);
    }

    /** Advance clock by a normally-distributed dt (mean 20 ms, σ 4 ms), step controller and plant.
     *  Returns the actual dt in seconds so callers can accumulate real elapsed time. */
    private static double step(FlywheelSimple flywheel, FlywheelTestFixture.SimMotorAdapter adapter,
                               FlywheelMotorSim sim, double[] timeSecs, Random rng) {
        double dt = Math.max(0.001, 0.020 + rng.nextGaussian() * 0.004);
        timeSecs[0] += dt;
        flywheel.update();
        sim.step(dt, adapter.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
        return dt;
    }

    // ── tests ─────────────────────────────────────────────────────────────────

    @Test
    void testSpinUpConverges() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = {1.0};
        FlywheelSimple flywheel = makeSystem(adapter, timeSecs);

        flywheel.setTps(2000);
        flywheel.update();   // seed lastTimeNanos; returns early

        Random rng = FlywheelTestFixture.makeRng();
        double firstReadySeconds = -1, elapsedSeconds = 0;
        double slewTime = 0, approachTime = 0;
        for (int i = 0; i < 800; i++) {
            double pv = flywheel.getProfiledVelocity();
            double dt = step(flywheel, adapter, sim, timeSecs, rng);
            elapsedSeconds += dt;
            if (pv < 2000.0) {
                double accelLimit = (FlywheelTestFixture.HUB_VOLTAGE - FlywheelSimple.PARAMS.kS
                        - FlywheelSimple.PARAMS.kV * pv) / FlywheelSimple.PARAMS.kA;
                if (FlywheelSimple.PARAMS.maxAccel < accelLimit) slewTime += dt;
                else approachTime += dt;
            }
            if (firstReadySeconds < 0 && flywheel.isReady()) firstReadySeconds = elapsedSeconds;
        }
        System.out.printf("testSpinUpConverges: isReady() at %.2f s  (slew: %.2f s, approach: %.2f s)%n",
                firstReadySeconds, slewTime, approachTime);

        assertTrue(flywheel.isReady(), "flywheel should be ready after 16 s of simulated spin-up");
        assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should be within 30 TPS of setpoint");
    }

    @Test
    void testCoastsWhenSetpointZero() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = {1.0};
        FlywheelSimple flywheel = makeSystem(adapter, timeSecs);

        flywheel.setTps(2000);
        flywheel.update();   // seed

        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 10; i++) step(flywheel, adapter, sim, timeSecs, rng);

        flywheel.setTps(0);
        step(flywheel, adapter, sim, timeSecs, rng);

        assertEquals(0.0, adapter.lastPower, "motor power should be zero when coasting");
        assertFalse(flywheel.isReady(), "isReady() must be false when setpoint is zero");
    }

    @Test
    void testSetpointStepDown() {
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = {1.0};
        FlywheelSimple flywheel = makeSystem(adapter, timeSecs);

        flywheel.setTps(2000);
        flywheel.update();   // seed

        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 800; i++) step(flywheel, adapter, sim, timeSecs, rng);
        assertTrue(flywheel.isReady(), "should be ready at 2000 TPS before step");

        flywheel.setTps(1000);
        for (int i = 0; i < 300; i++) step(flywheel, adapter, sim, timeSecs, rng);

        System.out.printf("testSetpointStepDown: true vel=%.1f TPS, isReady=%b%n",
                sim.getTrueVelocityTps(), flywheel.isReady());

        assertTrue(flywheel.isReady(), "should be ready at 1000 TPS after step");
        assertEquals(1000.0, sim.getTrueVelocityTps(), 30.0,
                "true velocity should be within 30 TPS of new setpoint");
    }

    @Test
    void testDisturbancePulseAndRecovery() {
        double kS = FlywheelSimple.PARAMS.kS;
        FlywheelMotorSim sim = makeSim();
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = {1.0};
        FlywheelSimple flywheel = makeSystem(adapter, timeSecs);

        flywheel.setTps(2000);
        flywheel.update();   // seed

        Random rng = FlywheelTestFixture.makeRng();
        for (int i = 0; i < 800; i++) step(flywheel, adapter, sim, timeSecs, rng);
        assertTrue(flywheel.isReady(), "should be ready at steady state before disturbance");

        sim.setDisturbanceVoltage(-kS - 0.1);
        for (int i = 0; i < 50; i++) step(flywheel, adapter, sim, timeSecs, rng);

        sim.setDisturbanceVoltage(-kS);
        for (int i = 0; i < 300; i++) step(flywheel, adapter, sim, timeSecs, rng);

        assertTrue(flywheel.isReady(), "should recover and be ready after disturbance pulse");
    }
}
