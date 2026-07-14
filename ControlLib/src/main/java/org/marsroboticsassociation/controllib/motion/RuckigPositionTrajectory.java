package org.marsroboticsassociation.controllib.motion;

import com.ruckig.InputParameter;
import com.ruckig.Ruckig;
import com.ruckig.Trajectory;

/**
 * {@link PositionTrajectory} backed by the Ruckig port ({@code com.ruckig}): the time-optimal,
 * jerk-limited profile from {@code (p0, v0, a0)} to the target state, planned once at
 * construction. Drop-in comparable with {@link SCurvePosition} — the primary constructor matches
 * {@link PositionTrajectoryManager.TrajectoryFactory}, so swapping planners is
 * {@code new PositionTrajectoryManager(..., RuckigPositionTrajectory::new)}.
 *
 * <p>Beyond {@link SCurvePosition}, this planner supports a nonzero target velocity and
 * acceleration (pass-through moves) via the extended constructor, and handles initial states
 * outside the limits (velocity or acceleration over their caps) with a proper brake
 * pre-trajectory instead of a special-cased prefix.
 *
 * <p><b>Limit-frame mapping.</b> {@code aMaxAccel}/{@code aMaxDecel} are travel-frame (speeding
 * up vs braking), like {@link SCurvePosition}; Ruckig's limits are signed. They are mapped once
 * at construction by the direction of {@code pTarget - p0} — the same convention
 * {@link SCurvePosition} uses for its {@code dir} frame.
 *
 * <p>After {@link #getTotalTime()}, samples hold the target state extrapolated at constant
 * acceleration: exactly {@code (pTarget, 0, 0)} for the rest-target constructor. Like
 * {@link SCurvePosition} under the manager, replans preserve p/v/a continuity but not jerk.
 *
 * <p>Getter calls cache the most recent sample time, so the manager's
 * position-then-velocity-then-acceleration pattern costs one trajectory evaluation per loop.
 * Not thread-safe.
 */
public class RuckigPositionTrajectory implements PositionTrajectory {

    private final Trajectory trajectory = new Trajectory(1);
    private final double totalTime;

    // Sample cache: one at_time evaluation serves the p/v/a getter triple.
    private final double[] p = new double[1];
    private final double[] v = new double[1];
    private final double[] a = new double[1];
    private final double[] j = new double[1];
    private final int[] section = new int[1];
    private double cachedT = Double.NaN;

    /**
     * Plan to the target at rest. Signature matches
     * {@link PositionTrajectoryManager.TrajectoryFactory}.
     */
    public RuckigPositionTrajectory(double p0, double pTarget, double v0, double a0,
                                    double vMax, double aMaxAccel, double aMaxDecel, double jMax) {
        this(p0, pTarget, v0, a0, 0.0, 0.0, vMax, aMaxAccel, aMaxDecel, jMax);
    }

    /** Plan to a target moving at {@code vf} with acceleration {@code af} (pass-through move). */
    public RuckigPositionTrajectory(double p0, double pTarget, double v0, double a0,
                                    double vf, double af,
                                    double vMax, double aMaxAccel, double aMaxDecel, double jMax) {
        // Same direction convention as SCurvePosition: ties (pTarget == p0) plan in +.
        boolean movingPositive = pTarget >= p0;

        InputParameter input = new InputParameter(1);
        input.current_position[0] = p0;
        input.current_velocity[0] = v0;
        input.current_acceleration[0] = a0;
        input.target_position[0] = pTarget;
        input.target_velocity[0] = vf;
        input.target_acceleration[0] = af;
        input.max_velocity[0] = Math.abs(vMax);
        input.max_acceleration[0] = Math.abs(movingPositive ? aMaxAccel : aMaxDecel);
        input.min_acceleration = new double[] {-Math.abs(movingPositive ? aMaxDecel : aMaxAccel)};
        input.max_jerk[0] = Math.abs(jMax);

        int result = new Ruckig(1).calculate(input, trajectory);
        if (result < 0) {
            throw new IllegalArgumentException(
                    "Ruckig could not plan (result " + result + "): p0=" + p0 + " pTarget=" + pTarget
                            + " v0=" + v0 + " a0=" + a0 + " vf=" + vf + " af=" + af
                            + " vMax=" + vMax + " aMaxAccel=" + aMaxAccel
                            + " aMaxDecel=" + aMaxDecel + " jMax=" + jMax);
        }
        this.totalTime = trajectory.get_duration();
    }

    private void sample(double t) {
        double clamped = Math.max(0.0, t);
        if (clamped == cachedT) {
            return;
        }
        trajectory.at_time(clamped, p, v, a, j, section);
        cachedT = clamped;
    }

    @Override
    public double getPosition(double t) {
        sample(t);
        return p[0];
    }

    @Override
    public double getVelocity(double t) {
        sample(t);
        return v[0];
    }

    @Override
    public double getAcceleration(double t) {
        sample(t);
        return a[0];
    }

    @Override
    public double getTotalTime() {
        return totalTime;
    }

    @Override
    public boolean isZeroJerk(double t) {
        sample(t);
        return j[0] == 0.0;
    }
}
