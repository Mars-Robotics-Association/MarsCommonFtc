package org.marsroboticsassociation.controllib.control;

import org.marsroboticsassociation.controllib.control.FlywheelSimple;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;

import java.util.Random;

/**
 * Shared simulation infrastructure for flywheel controller unit tests.
 */
class FlywheelTestFixture {

    /** Nominal hub voltage used across all flywheel sim tests. */
    static final double HUB_VOLTAGE = 13.75;

    /** RNG seed shared by all flywheel tests for reproducible noise sequences. */
    static final long SEED = 42L;

    /** Create a new seeded RNG for test-local use. */
    static Random makeRng() { return new Random(SEED); }

    /**
     * IMotor stub that bridges {@link FlywheelMotorSim} to the controller under test.
     */
    static class SimMotorAdapter implements IMotor {
        private final FlywheelMotorSim sim;
        double lastPower = 0.0;

        SimMotorAdapter(FlywheelMotorSim sim) {
            this.sim = sim;
        }

        @Override public String getName()              { return "sim"; }
        @Override public double getVelocity()          { return sim.getVelocityTps(); }
        @Override public void   setPower(double power) { lastPower = power; }
        @Override public double getHubVoltage()        { return HUB_VOLTAGE; }
        @Override public void   setVelocity(double tps) {}
        @Override public void   setVelocityPIDFCoefficients(double p, double i, double d, double f) {}
    }

    /**
     * Create a flywheel plant simulation with the standard characterization constants.
     */
    static FlywheelMotorSim makeSim() {
        return new FlywheelMotorSim(
                FlywheelSimple.PARAMS.kV,
                FlywheelSimple.PARAMS.kA);
    }
}
