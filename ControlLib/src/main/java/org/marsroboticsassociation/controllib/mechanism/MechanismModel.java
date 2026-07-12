package org.marsroboticsassociation.controllib.mechanism;

/**
 * The physics model of a DC-motor driven mechanism, shared by the estimator ({@link
 * MotorMechanismEkf}) and the controller ({@link MotorMechanismController}). Building one model and
 * handing it to both guarantees they agree on the constants.
 *
 * <p>The feedforward relationship between motion and voltage is:
 *
 * <pre>
 *   voltage = kS*sign(velocity) + kV*velocity + kA*acceleration + gravityVoltage(position)
 * </pre>
 *
 * Read forward it predicts acceleration from an applied voltage (what the filter's predict step
 * needs); read backward it predicts the voltage a desired motion needs (the F in the controller's
 * PIDF). The {@code kV*velocity} term is the motor's back-EMF: the faster it spins, the more
 * applied voltage is spent balancing the voltage the motor itself generates.
 *
 * <p>The only thing a concrete model supplies is gravity: a constant for a lift, an angle-dependent
 * term for an arm. See {@link ArmModel} and {@link LiftModel}.
 *
 * <p>Units are the mechanism's own (radians and volts for an arm, length units and volts for a
 * lift); stay consistent.
 */
public abstract class MechanismModel {

    private final double kS;
    private final double kV; // back-EMF / viscous term
    private final double kA;

    protected MechanismModel(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    /** Voltage needed to hold against gravity at this position. */
    public abstract double gravityVoltage(double position);

    /** Derivative of {@link #gravityVoltage} with respect to position. Zero for a lift. */
    public abstract double gravityVoltageDerivative(double position);

    /** Forward dynamics: the acceleration produced by an applied voltage. */
    public double acceleration(double position, double velocity, double voltage) {
        return (voltage - kS * Math.signum(velocity) - kV * velocity - gravityVoltage(position))
                / kA;
    }

    /** Inverse dynamics: the voltage a desired motion needs (the feedforward term). */
    public double feedforwardVoltage(double position, double velocity, double acceleration) {
        return kS * Math.signum(velocity)
                + kV * velocity
                + kA * acceleration
                + gravityVoltage(position);
    }

    /** Partial derivative of acceleration with respect to position, for the EKF Jacobian. */
    public double accelerationSlopeByPosition(double position) {
        return -gravityVoltageDerivative(position) / kA;
    }

    /** Partial derivative of acceleration with respect to velocity (the back-EMF term). */
    public double accelerationSlopeByVelocity() {
        return -kV / kA;
    }

    /**
     * The steady velocity the motor can hold while travelling in {@code travelDirection} at this
     * position, given the available voltage: {@code (availableVoltage - kS - gravityAlongTravel) /
     * kV}, where {@code gravityAlongTravel} is the gravity voltage signed by the direction of
     * travel. Going faster would need more voltage than exists, because back-EMF grows with speed.
     * Used as a velocity ceiling.
     *
     * <p>Gravity is charged in the direction of travel rather than at worst case: climbing, gravity
     * opposes the motion and lowers the ceiling; descending, it aids the motion and raises it, so a
     * mechanism is allowed to run down faster than it can drive up. {@code travelDirection} is any
     * value whose sign is the direction of intended motion (e.g. target minus current position).
     */
    public double maxSustainableVelocity(
            double availableVoltage, double position, double travelDirection) {
        double gravityAlongTravel = Math.signum(travelDirection) * gravityVoltage(position);
        return (availableVoltage - kS - gravityAlongTravel) / kV;
    }

    /**
     * The acceleration the motor can still deliver at this position and velocity given the
     * available voltage: {@code (availableVoltage - kS - kV*|velocity| - |gravity|) / kA}.
     *
     * <p>This is the more immediate face of the back-EMF limit. The {@code kV*|velocity|} term
     * means the headroom for acceleration shrinks the instant the mechanism moves, not just at top
     * speed; it reaches zero exactly at {@link #maxSustainableVelocity}. Friction is taken at worst
     * case (opposing the motion), but gravity is charged in the direction of travel: descending, it
     * aids the acceleration and opens up headroom the worst case would have hidden. {@code
     * travelDirection} is any value whose sign is the direction of intended motion.
     */
    public double maxSustainableAcceleration(
            double availableVoltage, double position, double velocity, double travelDirection) {
        double gravityAlongTravel = Math.signum(travelDirection) * gravityVoltage(position);
        return (availableVoltage - kS - kV * Math.abs(velocity) - gravityAlongTravel) / kA;
    }

    /**
     * The deceleration the motor can deliver while braking at this position and velocity: {@code
     * (availableVoltage + kV*|velocity| - kS - |gravity|) / kA}.
     *
     * <p>The key difference from {@link #maxSustainableAcceleration} is the sign of the back-EMF
     * term: when braking, the voltage the spinning motor generates <em>aids</em> the brake, so the
     * available deceleration <em>grows</em> with speed instead of shrinking.
     *
     * <p>Unlike the acceleration and velocity ceilings, gravity is held at worst case (opposing the
     * brake) here on purpose, so this ceiling takes no direction. The braking limit is what stops
     * the setpoint at the target, so an optimistic value overshoots; and worst case is also the
     * true value for the case that matters most, braking a gravity-driven descent.
     */
    public double maxSustainableDeceleration(
            double availableVoltage, double position, double velocity) {
        return (availableVoltage
                        + kV * Math.abs(velocity)
                        - kS
                        - Math.abs(gravityVoltage(position)))
                / kA;
    }
}
