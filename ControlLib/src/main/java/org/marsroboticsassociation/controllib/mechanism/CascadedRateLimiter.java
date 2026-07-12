package org.marsroboticsassociation.controllib.mechanism;

/**
 * Cascaded numeric slew-rate limiter. It turns a raw target position into a smooth setpoint whose
 * velocity, acceleration, and jerk are all bounded, so a controller never asks the mechanism to do
 * something the hardware cannot.
 *
 * <p>Three nested stages, computed top to bottom each loop:
 *
 * <ol>
 *   <li>the velocity needed to reach the target this step, clamped to {@code maxVelocity};
 *   <li>the acceleration needed to reach that velocity, clamped to the acceleration limit;
 *   <li>the jerk needed to reach that acceleration, clamped to {@code maxJerk}.
 * </ol>
 *
 * The clamped jerk is then integrated back up into acceleration, then velocity, then position.
 * <b>Acceleration and jerk are bounded exactly</b> by this construction.
 *
 * <p><b>Separate acceleration and deceleration limits.</b> Speeding up and slowing down are clamped
 * independently: the limit used depends on whether the requested acceleration increases or
 * decreases the current speed. This matters for a motor, whose back-EMF fights acceleration but
 * aids braking, so it can usually decelerate harder than it can accelerate.
 *
 * <p>The velocity command is deceleration-aware: it is capped to the fastest speed from which the
 * setpoint can still brake to a stop at the target under the <em>deceleration</em> limit and the
 * jerk limit. That is what keeps the setpoint from running past the target.
 *
 * <p>This is still a setpoint <em>filter</em>, not a full time-optimal planner, so velocity may
 * transiently exceed its cap by about {@code maxAcceleration^2 / (2*maxJerk)} on the ramp up. Keep
 * the jerk limit comfortably larger than the acceleration limit and it stays small; in closed loop
 * the controller's feedback absorbs it. Near the target the velocity command is additionally capped
 * to {@code error/h} (the speed that lands exactly on the target this step, with {@code h} a
 * jitter-hardened timestep — see below), so the setpoint settles to rest instead of limit-cycling
 * around the target.
 *
 * <p><b>dt jitter hardening.</b> Real control loops are not exactly periodic (FTC is typically ~16
 * ms ± a few ms). Raw {@code 1/dt} in the landing and discrete-derivative stages turns a short
 * sample into a large command that the next longer sample then integrates too far, so velocity
 * rings near the target. This class:
 *
 * <ul>
 *   <li>floors the timestep used for inverse-dt command math at {@link #MIN_COMMAND_DT};
 *   <li>substeps wall-clock intervals longer than {@link #MAX_INTEGRATION_DT};
 *   <li>snaps to rest inside a small settle band once error, velocity, and acceleration are all
 *       negligible relative to the configured limits.
 * </ul>
 */
public class CascadedRateLimiter {

    /**
     * Pass as {@code maxJerk} to disable the jerk limit: acceleration may then step straight to its
     * limit in a single call (bang-bang acceleration). The stopping law degrades to the plain
     * {@code sqrt(2*maxDeceleration*distance)} form. This is the only non-finite {@code maxJerk}
     * the limiter accepts; any other non-positive value (including {@code NaN}) is rejected.
     */
    public static final double UNLIMITED_JERK = Double.POSITIVE_INFINITY;

    /**
     * Floor on the timestep used for inverse-dt command math (landing velocity, accel, jerk). Real
     * loops run near 10–20 ms; a shorter sample inflates {@code error/dt} and {@code (Δ)/dt} into
     * spikes that ring when the next sample is longer. Flooring prevents that without changing
     * behavior at normal loop rates.
     */
    static final double MIN_COMMAND_DT = 0.005; // 5 ms

    /**
     * Ceiling on a single integration substep. A late loop is broken into several steps so the
     * profile does not leap under a command shaped for a shorter prior interval.
     */
    static final double MAX_INTEGRATION_DT = 0.025; // 25 ms

    /** Settle velocity as a fraction of {@code maxVelocity} (and a tiny floor when the cap is 0). */
    private static final double SETTLE_VEL_FRAC = 1e-3;

    /** Settle acceleration as a fraction of the larger of the accel/decel caps. */
    private static final double SETTLE_ACCEL_FRAC = 1e-3;

    private double position;
    private double velocity;
    private double acceleration;

    private double maxVelocity;
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxJerk;

    public CascadedRateLimiter(
            double maxVelocity,
            double maxAcceleration,
            double maxDeceleration,
            double maxJerk,
            double initialPosition) {
        this.maxVelocity = requireNonNegative(maxVelocity, "maxVelocity");
        this.maxAcceleration = requireNonNegative(maxAcceleration, "maxAcceleration");
        this.maxDeceleration = requireNonNegative(maxDeceleration, "maxDeceleration");
        this.maxJerk = requirePositiveJerk(maxJerk);
        this.position = initialPosition;
    }

    /**
     * Validate a jerk limit. It must be strictly positive; {@link #UNLIMITED_JERK} is the
     * sanctioned way to ask for no limit. Rejecting zero, negatives, and {@code NaN} keeps the
     * integrator from silently freezing (a zero jerk limit clamps every acceleration change to
     * nothing) or poisoning the profile with {@code NaN}. The {@code !(x > 0)} form rejects {@code
     * NaN} as well.
     */
    private static double requirePositiveJerk(double maxJerk) {
        if (!(maxJerk > 0)) {
            throw new IllegalArgumentException(
                    "maxJerk must be positive (use CascadedRateLimiter.UNLIMITED_JERK for no jerk"
                            + " limit); got "
                            + maxJerk);
        }
        return maxJerk;
    }

    /**
     * Validate a velocity, acceleration, or deceleration limit. Zero is allowed: it is a legitimate
     * ceiling (the controller lowers these to zero when the back-EMF headroom runs out, meaning "no
     * authority right now"). Negatives and {@code NaN} are rejected rather than silently clamped to
     * zero, so a bad limit surfaces at its source instead of quietly freezing the profile. The
     * {@code !(x >= 0)} form rejects {@code NaN} as well.
     */
    private static double requireNonNegative(double value, String name) {
        if (!(value >= 0)) {
            throw new IllegalArgumentException(name + " must be non-negative; got " + value);
        }
        return value;
    }

    /** Snap the profile to a position and stop, e.g. when (re)starting a move from rest. */
    public void reset(double position) {
        this.position = position;
        this.velocity = 0.0;
        this.acceleration = 0.0;
    }

    /** Cap the velocity limit. Used to lower it dynamically, such as for the back-EMF ceiling. */
    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = requireNonNegative(maxVelocity, "maxVelocity");
    }

    /** The limit on acceleration that increases speed. */
    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = requireNonNegative(maxAcceleration, "maxAcceleration");
    }

    /** The limit on acceleration that decreases speed (braking). */
    public void setMaxDeceleration(double maxDeceleration) {
        this.maxDeceleration = requireNonNegative(maxDeceleration, "maxDeceleration");
    }

    public void setMaxJerk(double maxJerk) {
        this.maxJerk = requirePositiveJerk(maxJerk);
    }

    /**
     * Advance the setpoint one step toward the target, respecting the rate limits.
     *
     * @param targetPosition where the setpoint should end up
     * @param dt wall-clock seconds since the previous call (may be jittery; hardened internally)
     */
    public void update(double targetPosition, double dt) {
        // Reject non-positive and NaN. The !(dt > 0) form catches NaN as well.
        if (!(dt > 0)) {
            return;
        }
        // Substep long wall-clock gaps so a late loop cannot integrate one large step under a
        // command that assumed a shorter interval.
        double remaining = dt;
        while (remaining > 0.0) {
            double step = Math.min(remaining, MAX_INTEGRATION_DT);
            if (updateStep(targetPosition, step)) {
                return; // settled on the target
            }
            remaining -= step;
        }
    }

    /**
     * One integration substep. Returns {@code true} if the profile has snapped to rest on the
     * target (caller may stop substepping).
     */
    private boolean updateStep(double targetPosition, double dt) {
        if (trySettle(targetPosition)) {
            return true;
        }

        double error = targetPosition - position;
        double direction = Math.signum(error);
        // The stopping law assumes braking begins instantly, but if the setpoint is still
        // accelerating toward the target the acceleration must first swing back to zero at the
        // jerk limit, and the distance covered during that swing is not available for braking.
        // Charge it against the error, or braking starts one accel-reversal too late and the
        // setpoint sails past the target.
        double towardVelocity = velocity * direction;
        double towardAcceleration = acceleration * direction;
        double swingTime = Math.max(0, towardAcceleration) / maxJerk;
        double swingDistance =
                Math.max(0, towardVelocity + 0.5 * towardAcceleration * swingTime) * swingTime;
        double brakingDistance = Math.max(0, Math.abs(error) - swingDistance);
        double stoppingVelocity = jerkLimitedStoppingVelocity(brakingDistance);

        // Floor the inverse-dt timestep used for landing and discrete derivatives. A short sample
        // would otherwise inflate error/dt and (Δ)/dt into a spike that rings when the next sample
        // is longer. Integration still uses the real substep `dt` so wall-clock progress is honest.
        double h = Math.max(dt, MIN_COMMAND_DT);
        // The stopping-velocity law has unbounded slope at zero distance (its cube-root branch),
        // so on its own it always commands enough speed to overshoot a nearly-reached target
        // within one step, and the setpoint limit-cycles instead of settling. Landing exactly on
        // the target this step needs only error/h; capping to it makes the command vanish with
        // the error, so the profile actually comes to rest.
        double landingVelocity = Math.abs(error) / h;
        double velocityCommand =
                direction * Math.min(maxVelocity, Math.min(stoppingVelocity, landingVelocity));

        // Clamp the acceleration command directionally: the acceleration limit applies to
        // speeding up, the deceleration limit to slowing down. Which bound is which depends
        // on the sign of the current velocity.
        double lowerBound;
        double upperBound;
        if (velocity > 0) {
            upperBound = maxAcceleration; // faster in +
            lowerBound = -maxDeceleration; // braking
        } else if (velocity < 0) {
            upperBound = maxDeceleration; // braking
            lowerBound = -maxAcceleration; // faster in -
        } else {
            upperBound = maxAcceleration; // from rest, either direction is speeding up
            lowerBound = -maxAcceleration;
        }
        double accelCommand = clamp((velocityCommand - velocity) / h, lowerBound, upperBound);
        double jerkCommand = clamp((accelCommand - acceleration) / h, -maxJerk, maxJerk);

        acceleration += jerkCommand * dt;
        velocity += acceleration * dt;
        position += velocity * dt;

        // Catch residual chatter after the step so the next loop does not re-excite feedforward.
        return trySettle(targetPosition);
    }

    /**
     * If the setpoint is already effectively at the target with negligible rates, snap exactly to
     * rest. Thresholds scale with the configured limits so the same code works for radians or
     * meters without unit-specific constants. Does not teleport when the velocity ceiling is zero
     * and there is still meaningful error (no authority to close the gap).
     */
    private boolean trySettle(double targetPosition) {
        double error = targetPosition - position;
        double velScale = Math.max(maxVelocity, 1e-6);
        double accelScale = Math.max(Math.max(maxAcceleration, maxDeceleration), 1e-6);
        double settleVel = SETTLE_VEL_FRAC * velScale;
        double settleAccel = SETTLE_ACCEL_FRAC * accelScale;
        double settlePos = Math.max(1e-12, settleVel * MIN_COMMAND_DT);
        if (Math.abs(error) <= settlePos
                && Math.abs(velocity) <= settleVel
                && Math.abs(acceleration) <= settleAccel) {
            position = targetPosition;
            velocity = 0.0;
            acceleration = 0.0;
            return true;
        }
        return false;
    }

    /**
     * The highest speed from which the setpoint can still brake to a stop within {@code distance},
     * under the deceleration and jerk limits. Below the critical distance the braking never reaches
     * full deceleration (a triangular pulse); above it there is a constant-deceleration phase.
     */
    private double jerkLimitedStoppingVelocity(double distance) {
        // Already at (or past) the target: no speed is allowed. Guarding this also keeps the
        // cube-root branch below from evaluating 0*0*UNLIMITED_JERK, which is NaN.
        if (distance <= 0) {
            return 0.0;
        }
        // No braking authority (the controller can drive the deceleration ceiling to zero): the
        // only speed from which the setpoint can "stop" within any distance is zero. Return it
        // rather than dividing by zero into NaN.
        if (maxDeceleration <= 0) {
            return 0.0;
        }
        // With maxJerk == UNLIMITED_JERK the critical distance is 0 (so every real distance takes
        // the
        // constant-deceleration branch) and that branch reduces to
        // sqrt(2*maxDeceleration*distance),
        // the jerk-unlimited stopping law.
        double criticalDistance =
                (maxDeceleration * maxDeceleration * maxDeceleration) / (maxJerk * maxJerk);
        if (distance <= criticalDistance) {
            return Math.cbrt(distance * distance * maxJerk);
        }
        double a2 = 1.0 / (2.0 * maxDeceleration);
        double b = maxDeceleration / (2.0 * maxJerk);
        return (-b + Math.sqrt(b * b + 4.0 * a2 * distance)) / (2.0 * a2);
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }
}
