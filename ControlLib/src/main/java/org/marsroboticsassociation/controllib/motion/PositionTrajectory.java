package org.marsroboticsassociation.controllib.motion;

/**
 * Time-parameterized position profile.
 *
 * <p>Implementations are expected to provide meaningful position, velocity, and acceleration for
 * any {@code t} in {@code [0, getTotalTime()]}. When a {@link PositionTrajectoryManager} replans
 * mid-motion, it seeds the next trajectory from the sampled {@code p/v/a} state only. That means
 * manager-level replans are designed to preserve position, velocity, and acceleration continuity,
 * but they do not in general preserve jerk continuity across the handoff.
 */
public interface PositionTrajectory {
    double getPosition(double t);
    double getVelocity(double t);
    double getAcceleration(double t);
    double getTotalTime();
    boolean isZeroJerk(double t);
}
