package org.marsroboticsassociation.controllib.mechanism;

/**
 * Physics model of a linear lift. Gravity pulls down with the same force at every height, so the
 * hold voltage is a single constant {@code kG} and its derivative with respect to position is zero.
 * State is in the lift's length units (or encoder ticks), voltages in volts.
 */
public class LiftModel extends MechanismModel {

    private final double kG;

    /**
     * @param kS static friction (volts)
     * @param kV back-EMF / viscous term, volts per (unit/sec)
     * @param kA volts per (unit/sec^2)
     * @param kG constant gravity term, volts
     */
    public LiftModel(double kS, double kV, double kA, double kG) {
        super(kS, kV, kA);
        this.kG = kG;
    }

    @Override
    public double gravityVoltage(double position) {
        return kG;
    }

    @Override
    public double gravityVoltageDerivative(double position) {
        return 0.0;
    }
}
