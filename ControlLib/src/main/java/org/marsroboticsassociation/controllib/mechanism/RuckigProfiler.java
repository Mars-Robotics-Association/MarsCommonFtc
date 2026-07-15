package org.marsroboticsassociation.controllib.mechanism;

import com.ruckig.InputParameter;
import com.ruckig.Ruckig;
import com.ruckig.Trajectory;

/**
 * Online-trajectory-generation setpoint profiler backed by the Ruckig port ({@code com.ruckig}).
 * Drop-in alternative to {@link CascadedRateLimiter} behind {@link SetpointProfile}.
 *
 * <p>Every {@link #update} replans a time-optimal, jerk-limited trajectory from the setpoint's own
 * {@code (position, velocity, acceleration)} state to the target at rest, under the current
 * limits, then samples it {@code dt} ahead. This is Ruckig's intended usage pattern, and it is
 * what the greedy cascade cannot do: the deceleration phase is <em>planned</em> to land at
 * {@code (target, 0, 0)} exactly, so a move finishes with a jerk ramp-out instead of the
 * late-braking clamps of a one-step filter. Limits rewritten between calls (back-EMF ceilings)
 * are simply picked up by the next replan; states outside the new limits (velocity or residual
 * acceleration over a freshly lowered cap) are absorbed by Ruckig's brake pre-trajectory rather
 * than clamped.
 *
 * <p><b>Limit-frame mapping.</b> This class takes travel-frame limits like the cascade
 * (acceleration = speeding up, deceleration = braking); Ruckig's are signed. Each replan maps by
 * the direction of travel: moving in +, {@code max_acceleration = maxAcceleration} and
 * {@code min_acceleration = -maxDeceleration}; flipped for a − move. A single plan containing a
 * direction reversal (brake, then proceed the other way) gets the accel/decel roles swapped on
 * one segment; per-loop replanning refreshes the mapping, so the executed slice of each plan is
 * correct. This is the deliberate trade documented in the port plan (§7).
 *
 * <p><b>Back-EMF braking caveat.</b> A plan assumes its limits hold for the whole remaining move,
 * but {@link MechanismModel#maxSustainableDeceleration} shrinks as speed drops. Feeding this
 * profiler the instantaneous braking ceiling is therefore optimistic during a stop. Callers that
 * rewrite limits from the model each loop should pass a braking ceiling evaluated at low speed
 * (see {@link MotorMechanismController}'s conservative-braking option), which is a lower bound on
 * the authority available anywhere in the remaining brake.
 *
 * <p><b>Zero-authority limits.</b> Zero limits mean "no authority right now" (the controller
 * lowers them to zero when the back-EMF headroom runs out). Ruckig cannot plan with a zero limit,
 * so limits are floored at a tiny positive value: the resulting plan brakes normally and then
 * crawls at negligible speed — effectively a hold, matching the cascade's behavior. If no valid
 * plan exists from the exact current state (per-loop ceiling rewrites can strand it just outside
 * the fresh bounds with no matching authority), the state is clamped into the band and replanned
 * — the cascade's coping strategy — and only if that also fails does the setpoint hold for the
 * step ({@link #getLastResult} exposes the Ruckig result code for debugging).
 *
 * <p>Steady-state {@code update} does not allocate. Not thread-safe.
 */
public class RuckigProfiler implements SetpointProfile {

    /** Pass as {@code maxJerk} to disable the jerk limit (second-order, bang-bang acceleration). */
    public static final double UNLIMITED_JERK = Double.POSITIVE_INFINITY;

    /**
     * Floor applied to zero limits so a plan always exists. Small enough that "crawling" at this
     * speed is indistinguishable from holding in any practical unit system.
     */
    static final double MIN_LIMIT = 1e-9;

    private final Ruckig otg = new Ruckig(1);
    private final InputParameter input = new InputParameter(1);
    private final Trajectory trajectory = new Trajectory(1);

    // Preallocated at_time outputs (the 3-array overload allocates internally; this one doesn't).
    private final double[] pOut = new double[1];
    private final double[] vOut = new double[1];
    private final double[] aOut = new double[1];
    private final double[] jOut = new double[1];
    private final int[] sectionOut = new int[1];

    private double position;
    private double velocity;
    private double acceleration;

    private double maxVelocity;
    private double maxAcceleration;
    private double maxDeceleration;
    private double maxJerk;

    private int lastResult = com.ruckig.Result.Finished;

    public RuckigProfiler(
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
        // The min_acceleration array is set per replan; allocate it once here.
        input.min_acceleration = new double[1];
        input.target_velocity[0] = 0.0;
        input.target_acceleration[0] = 0.0;
    }

    @Override
    public void reset(double position) {
        this.position = position;
        this.velocity = 0.0;
        this.acceleration = 0.0;
    }

    @Override
    public void setMaxVelocity(double maxVelocity) {
        this.maxVelocity = requireNonNegative(maxVelocity, "maxVelocity");
    }

    @Override
    public void setMaxAcceleration(double maxAcceleration) {
        this.maxAcceleration = requireNonNegative(maxAcceleration, "maxAcceleration");
    }

    @Override
    public void setMaxDeceleration(double maxDeceleration) {
        this.maxDeceleration = requireNonNegative(maxDeceleration, "maxDeceleration");
    }

    @Override
    public void setMaxJerk(double maxJerk) {
        this.maxJerk = requirePositiveJerk(maxJerk);
    }

    @Override
    public void update(double targetPosition, double dt) {
        // Reject non-positive and NaN dt, and non-finite targets. The !(x > 0) form catches NaN.
        if (!(dt > 0) || !isFinite(targetPosition)) {
            return;
        }
        if (position == targetPosition && velocity == 0.0 && acceleration == 0.0) {
            return; // settled; nothing to plan
        }

        // Travel-frame -> signed-frame limit mapping by direction of travel. At zero error the
        // move is pure braking of residual motion; take the direction that motion points.
        double travel = targetPosition - position;
        boolean movingPositive = travel > 0 || (travel == 0 && velocity >= 0);
        double accelCap = Math.max(movingPositive ? maxAcceleration : maxDeceleration, MIN_LIMIT);
        double decelCap = Math.max(movingPositive ? maxDeceleration : maxAcceleration, MIN_LIMIT);

        input.current_position[0] = position;
        input.current_velocity[0] = velocity;
        input.current_acceleration[0] = acceleration;
        input.target_position[0] = targetPosition;
        input.max_velocity[0] = Math.max(maxVelocity, MIN_LIMIT);
        input.max_acceleration[0] = accelCap;
        input.min_acceleration[0] = -decelCap;
        input.max_jerk[0] = maxJerk; // UNLIMITED_JERK (infinity) selects second-order profiles

        lastResult = otg.calculate(input, trajectory);
        if (lastResult < 0) {
            // Per-loop limit rewrites can strand the state just outside the fresh bounds while
            // the matching authority is collapsed — e.g. cruising at the back-EMF velocity
            // ceiling as it inches down each loop with the accel ceiling at zero. From that
            // exact state no jerk-limited plan exists (Ruckig needs a brake pre-trajectory it
            // has no authority for), and holding would freeze the profile forever, because the
            // held state reproduces the same inputs next loop. Cope the way the cascade does:
            // clamp the state into the band and plan from there. The velocity nick is bounded
            // by how far the ceiling moved in one loop, so it stays negligible.
            input.current_velocity[0] =
                    clamp(velocity, -input.max_velocity[0], input.max_velocity[0]);
            input.current_acceleration[0] = clamp(acceleration, -decelCap, accelCap);
            lastResult = otg.calculate(input, trajectory);
            if (lastResult < 0) {
                return; // still infeasible: hold this step
            }
        }

        if (dt >= trajectory.get_duration()) {
            // The whole remaining move fits in this step: land exactly, like the cascade does.
            position = targetPosition;
            velocity = 0.0;
            acceleration = 0.0;
            return;
        }
        trajectory.at_time(dt, pOut, vOut, aOut, jOut, sectionOut);
        position = pOut[0];
        velocity = vOut[0];
        acceleration = aOut[0];
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public double getAcceleration() {
        return acceleration;
    }

    /** Ruckig {@link com.ruckig.Result} code of the most recent replan, for telemetry/debugging. */
    public int getLastResult() {
        return lastResult;
    }

    private static boolean isFinite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    /** Same validation contract as {@link CascadedRateLimiter}: zero allowed, negatives/NaN not. */
    private static double requireNonNegative(double value, String name) {
        if (!(value >= 0)) {
            throw new IllegalArgumentException(name + " must be non-negative; got " + value);
        }
        return value;
    }

    /** Same validation contract as {@link CascadedRateLimiter#UNLIMITED_JERK}'s. */
    private static double requirePositiveJerk(double maxJerk) {
        if (!(maxJerk > 0)) {
            throw new IllegalArgumentException(
                    "maxJerk must be positive (use RuckigProfiler.UNLIMITED_JERK for no jerk"
                            + " limit); got "
                            + maxJerk);
        }
        return maxJerk;
    }
}
