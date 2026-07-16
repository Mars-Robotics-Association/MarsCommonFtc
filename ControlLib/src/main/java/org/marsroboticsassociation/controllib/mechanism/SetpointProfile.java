package org.marsroboticsassociation.controllib.mechanism;

/**
 * A per-loop setpoint profiler: turns a raw target position into a smooth setpoint whose
 * velocity, acceleration, and jerk are bounded, under limits that may be rewritten every loop
 * (e.g. from back-EMF headroom).
 *
 * <p>The concrete implementation is {@link RuckigProfiler} — online trajectory generation
 * (replans a time-optimal, jerk-limited profile from the setpoint's own state every loop), so the
 * stop is a planned jerk ramp-out that lands exactly at rest on the target. {@link
 * MotorMechanismController} drives the {@link ModelAwareSetpointProfile} extension of this
 * surface.
 *
 * <p>Contract notes:
 *
 * <ul>
 *   <li>{@code update} advances the setpoint by wall-clock {@code dt}; non-positive or NaN
 *       {@code dt} is a no-op.
 *   <li>Velocity/acceleration/deceleration limits accept zero, meaning "no authority right now";
 *       the setpoint should hold rather than teleport.
 *   <li>Acceleration and deceleration are expressed in the travel frame: acceleration limits
 *       speeding up, deceleration limits braking, regardless of the direction of motion.
 * </ul>
 */
public interface SetpointProfile {

    /** Snap the profile to a position and stop, e.g. when (re)starting a move from rest. */
    void reset(double position);

    /** Cap the velocity limit. Used to lower it dynamically, such as for the back-EMF ceiling. */
    void setMaxVelocity(double maxVelocity);

    /** The limit on acceleration that increases speed. */
    void setMaxAcceleration(double maxAcceleration);

    /** The limit on acceleration that decreases speed (braking). */
    void setMaxDeceleration(double maxDeceleration);

    void setMaxJerk(double maxJerk);

    /**
     * Advance the setpoint one step toward the target, respecting the rate limits.
     *
     * @param targetPosition where the setpoint should end up
     * @param dt wall-clock seconds since the previous call
     */
    void update(double targetPosition, double dt);

    double getPosition();

    double getVelocity();

    double getAcceleration();
}
