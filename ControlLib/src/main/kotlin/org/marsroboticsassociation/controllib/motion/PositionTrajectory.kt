package org.marsroboticsassociation.controllib.motion

/**
 * Time-parameterized position profile.
 *
 * <p>Implementations are expected to provide meaningful position, velocity, and acceleration for
 * any `t` in `[0, getTotalTime()]`. When a [PositionTrajectoryManager] replans mid-motion, it seeds
 * the next trajectory from the sampled `p/v/a` state only. That means manager-level replans are
 * designed to preserve position, velocity, and acceleration continuity, but they do not in general
 * preserve jerk continuity across the handoff.
 */
interface PositionTrajectory {
    fun getPosition(t: Double): Double

    fun getVelocity(t: Double): Double

    fun getAcceleration(t: Double): Double

    fun getTotalTime(): Double

    fun isZeroJerk(t: Double): Boolean
}
