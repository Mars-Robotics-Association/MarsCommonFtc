package org.marsroboticsassociation.controllib.mechanism;

/**
 * PIDF position controller for a DC-motor driven mechanism, the companion to {@link
 * MotorMechanismEkf}: the filter estimates where the mechanism is, this drives it where you want it
 * to go. Both take the same {@link MechanismModel}.
 *
 * <p>Each loop it does four things:
 *
 * <ol>
 *   <li><b>Profiles the target</b> through a {@link CascadedRateLimiter}, so the setpoint it chases
 *       has bounded velocity, acceleration, and jerk.
 *   <li><b>Feeds forward</b> the voltage the model says the motion needs (the F in PIDF), including
 *       the back-EMF term.
 *   <li><b>Corrects with PID</b> on the gap between the profile and the filter's estimate.
 *   <li><b>Clamps</b> the result to the bus voltage.
 * </ol>
 *
 * <p>The measured <b>bus voltage</b> is passed in every loop, not fixed at construction, because an
 * FTC battery is only nominally 12 V: a fresh pack is a bit over 14 V and it sags under load and
 * drains over a match. Every limit keyed off voltage (the clamp and the back-EMF ceilings) tracks
 * that live value. The output is a voltage; convert it to a motor power with the same reading
 * ({@code power = volts / busVolts}) so sag does not change the behavior, and feed that applied
 * voltage back into the filter's predict step.
 *
 * <p><b>Back-EMF ceilings.</b> Each loop the profile's acceleration limit is capped to {@link
 * MechanismModel#maxSustainableAcceleration} (which shrinks with speed) and its deceleration limit
 * to {@link MechanismModel#maxSustainableDeceleration} (which grows with speed, since back-EMF aids
 * braking). The velocity limit is capped to {@link MechanismModel#maxSustainableVelocity} as a
 * backstop. The acceleration and velocity ceilings charge gravity in the direction of travel, so a
 * mechanism may run downhill faster than it drives uphill; the braking ceiling keeps gravity at
 * worst case so it never plans to stop more abruptly than gravity will allow.
 */
public class MotorMechanismController {

    private final MechanismModel model;

    // PID feedback gains, in volts per unit of error.
    private final double kP;
    private final double kI;
    private final double kD;

    private final double feedbackVoltageMargin;
    private final double configuredMaxVelocity;
    private final double configuredMaxAcceleration;

    private final CascadedRateLimiter profile;
    private double integral = 0.0;
    private double lastVoltage = 0.0;

    /**
     * @param model the shared mechanism model (arm or lift)
     * @param kP proportional gain, volts per unit of error
     * @param kI integral gain, volts per (unit-second)
     * @param kD derivative gain, volts per (unit/sec) of velocity error
     * @param maxVelocity velocity limit
     * @param maxAcceleration acceleration limit (the mechanical cap for both speeding up and
     *     braking; back-EMF throttles acceleration below it at speed)
     * @param maxJerk jerk limit
     * @param feedbackVoltageMargin volts held back from the profile's feedforward so the PID has
     *     room to correct (e.g. 1.5); the bus voltage comes in per loop
     * @param initialPosition the mechanism's position right now
     */
    public MotorMechanismController(
            MechanismModel model,
            double kP,
            double kI,
            double kD,
            double maxVelocity,
            double maxAcceleration,
            double maxJerk,
            double feedbackVoltageMargin,
            double initialPosition) {
        this.model = model;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.feedbackVoltageMargin = feedbackVoltageMargin;
        this.configuredMaxVelocity = maxVelocity;
        this.configuredMaxAcceleration = maxAcceleration;
        this.profile =
                new CascadedRateLimiter(
                        maxVelocity, maxAcceleration, maxAcceleration, maxJerk, initialPosition);
    }

    /** Restart the profile from a known position at rest and clear the integrator. */
    public void reset(double position) {
        profile.reset(position);
        integral = 0.0;
        lastVoltage = 0.0;
    }

    /**
     * Compute the commanded motor voltage for this loop.
     *
     * @param targetPosition where you want the mechanism to end up
     * @param measuredPosition the filter's position estimate
     * @param measuredVelocity the filter's velocity estimate
     * @param busVoltage the measured battery/bus voltage this loop
     * @param dt seconds since the previous call
     * @return commanded voltage, clamped to +/- busVoltage
     */
    public double calculate(
            double targetPosition,
            double measuredPosition,
            double measuredVelocity,
            double busVoltage,
            double dt) {
        // Voltage available to the feedforward, leaving a margin for the PID to correct with.
        double availableVoltage = Math.max(0.0, busVoltage - feedbackVoltageMargin);
        // The ceilings below are evaluated at the profile's current state, before update() steps
        // it forward, and on purpose. The acceleration ceiling depends on velocity, but the
        // post-update velocity is itself the result of applying that ceiling, so evaluating after
        // update() would be an algebraic loop. Using the profile's own state (not the noisy
        // estimate) also keeps the setpoint trajectory self-consistent and noise-independent. The
        // one-step lag this leaves is bounded and absorbed by the voltage margin and the clamp.
        double profilePosition = profile.getPosition();
        double profileVelocity = profile.getVelocity();

        // The direction the setpoint is travelling. The acceleration and velocity ceilings charge
        // gravity along it, so descending (where gravity aids the motion) gets more headroom than
        // the worst case would allow.
        double travelDirection = targetPosition - profilePosition;

        // Back-EMF ceilings: acceleration headroom shrinks with speed (back-EMF opposes it),
        // braking headroom grows with speed (back-EMF aids it). The velocity ceiling, where
        // acceleration headroom reaches zero, is kept as a backstop.
        double backEmfAccelLimit =
                model.maxSustainableAcceleration(
                        availableVoltage, profilePosition, profileVelocity, travelDirection);
        profile.setMaxAcceleration(
                Math.min(configuredMaxAcceleration, Math.max(0, backEmfAccelLimit)));

        double backEmfDecelLimit =
                model.maxSustainableDeceleration(
                        availableVoltage, profilePosition, profileVelocity);
        profile.setMaxDeceleration(
                Math.min(configuredMaxAcceleration, Math.max(0, backEmfDecelLimit)));

        double backEmfVelocityLimit =
                model.maxSustainableVelocity(availableVoltage, profilePosition, travelDirection);
        profile.setMaxVelocity(Math.min(configuredMaxVelocity, Math.max(0, backEmfVelocityLimit)));

        profile.update(targetPosition, dt);
        double setpointPosition = profile.getPosition();
        double setpointVelocity = profile.getVelocity();
        double setpointAcceleration = profile.getAcceleration();

        // F: feedforward from the model (back-EMF aware).
        double feedforward =
                model.feedforwardVoltage(setpointPosition, setpointVelocity, setpointAcceleration);

        // PID feedback on the profile-versus-estimate gap.
        double positionError = setpointPosition - measuredPosition;
        double velocityError = setpointVelocity - measuredVelocity;
        double tentativeIntegral = integral + positionError * dt;
        double feedback = kP * positionError + kI * tentativeIntegral + kD * velocityError;

        double voltage = feedforward + feedback;
        double clamped = clamp(voltage, -busVoltage, busVoltage);
        // Anti-windup: only keep the new integral if the command was not saturated.
        if (clamped == voltage) {
            integral = tentativeIntegral;
        }

        lastVoltage = clamped;
        return clamped;
    }

    /** The voltage returned by the most recent {@link #calculate}. */
    public double getLastVoltage() {
        return lastVoltage;
    }

    public double getSetpointPosition() {
        return profile.getPosition();
    }

    public double getSetpointVelocity() {
        return profile.getVelocity();
    }

    public double getSetpointAcceleration() {
        return profile.getAcceleration();
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }
}
