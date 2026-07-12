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
 * to {@code error/dt} (the speed that lands exactly on the target this step), so the setpoint
 * settles to rest instead of limit-cycling around the target.
 */
public class CascadedRateLimiter {

    /**
     * Pass as {@code maxJerk} to disable the jerk limit: acceleration may then step straight to its
     * limit in a single call (bang-bang acceleration). The stopping law degrades to the plain
     * {@code sqrt(2*maxDeceleration*distance)} form. This is the only non-finite {@code maxJerk}
     * the limiter accepts; any other non-positive value (including {@code NaN}) is rejected.
     */
    public static final double UNLIMITED_JERK = Double.POSITIVE_INFINITY;

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

    /** Advance the setpoint one step toward the target, respecting the rate limits. */
    public void update(double targetPosition, double dt) {
        if (dt <= 0) {
            return;
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
        // The stopping-velocity law has unbounded slope at zero distance (its cube-root branch),
        // so on its own it always commands enough speed to overshoot a nearly-reached target
        // within one step, and the setpoint limit-cycles instead of settling. Landing exactly on
        // the target this step needs only error/dt; capping to it makes the command vanish with
        // the error, so the profile actually comes to rest.
        double landingVelocity = Math.abs(error) / dt;
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
        double accelCommand = clamp((velocityCommand - velocity) / dt, lowerBound, upperBound);
        double jerkCommand = clamp((accelCommand - acceleration) / dt, -maxJerk, maxJerk);

        acceleration += jerkCommand * dt;
        velocity += acceleration * dt;
        position += velocity * dt;
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
