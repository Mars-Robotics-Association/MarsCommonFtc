package org.marsroboticsassociation.controllib.mechanism;

/**
 * Physics model of a rotating arm. Gravity's hold voltage varies with angle as {@code
 * kCos*cos(theta) + kSin*sin(theta)}; the {@code kSin} term covers an arm whose center of mass is
 * offset from the encoder's zero angle (pass 0 if gravity is purely a cosine). State is in radians,
 * voltages in volts.
 */
public class ArmModel extends MechanismModel {

    private final double kCos;
    private final double kSin;

    /**
     * @param kS static friction (volts)
     * @param kV back-EMF / viscous term, volts per rad/s
     * @param kA volts per rad/s^2
     * @param kCos gravity term, volts at horizontal
     * @param kSin gravity term for a center-of-mass angular offset, volts (0 if none)
     */
    public ArmModel(double kS, double kV, double kA, double kCos, double kSin) {
        super(kS, kV, kA);
        this.kCos = kCos;
        this.kSin = kSin;
    }

    @Override
    public double gravityVoltage(double theta) {
        return kCos * Math.cos(theta) + kSin * Math.sin(theta);
    }

    @Override
    public double gravityVoltageDerivative(double theta) {
        return -kCos * Math.sin(theta) + kSin * Math.cos(theta);
    }
}
