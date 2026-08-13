package org.marsroboticsassociation.controllib.mechanism

/**
 * A per-loop setpoint profiler: turns a raw target position into a smooth setpoint whose velocity,
 * acceleration, and jerk are bounded, under limits that may be rewritten every loop (e.g. from
 * back-EMF headroom).
 *
 * <p>The concrete implementation is [RuckigProfiler] — online trajectory generation (replans a
 * time-optimal, jerk-limited profile from the setpoint's own state every loop), so the stop is a
 * planned jerk ramp-out that lands exactly at rest on the target. [MotorMechanismController] drives
 * the [ModelAwareSetpointProfile] extension of this surface.
 *
 * <p>Contract notes:
 * <ul>
 * <li>`update` advances the setpoint by wall-clock `dt`; non-positive or NaN `dt` is a no-op.
 * <li>Velocity/acceleration/deceleration limits accept zero, meaning "no authority right now"; the
 *   setpoint should hold rather than teleport.
 * <li>Acceleration and deceleration are expressed in the travel frame: acceleration limits speeding
 *   up, deceleration limits braking, regardless of the direction of motion.
 * </ul>
 */
interface SetpointProfile {

    /** Snap the profile to a position and stop, e.g. when (re)starting a move from rest. */
    fun reset(position: Double)

    /** Cap the velocity limit. Used to lower it dynamically, such as for the back-EMF ceiling. */
    fun setMaxVelocity(maxVelocity: Double)

    /** The limit on acceleration that increases speed. */
    fun setMaxAcceleration(maxAcceleration: Double)

    /** The limit on acceleration that decreases speed (braking). */
    fun setMaxDeceleration(maxDeceleration: Double)

    fun setMaxJerk(maxJerk: Double)

    /**
     * Advance the setpoint one step toward the target, respecting the rate limits.
     *
     * @param targetPosition where the setpoint should end up
     * @param dt wall-clock seconds since the previous call
     */
    fun update(targetPosition: Double, dt: Double)

    val position: Double

    val velocity: Double

    val acceleration: Double
}
