package org.marsroboticsassociation.controllib.projectile

import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan

/**
 * Pure-math projectile motion utilities for a fixed-angle launcher. All methods are stateless —
 * caller supplies every needed value.
 */
object ProjectileMotion {

    /** Gravitational acceleration in inches per second squared. */
    const val g: Double = 386.09

    /**
     * Calculates the required initial launch speed (in/s) for a projectile to hit a target at the
     * given horizontal distance and height difference.
     *
     * @param distance Horizontal distance to the target (inches).
     * @param launchAngleRad Launch angle above horizontal (radians).
     * @param heightDiff Vertical distance from launcher to target (inches, positive = target above
     *   launcher).
     * @return Required launch speed in inches per second.
     */
    fun getLaunchSpeed(distance: Double, launchAngleRad: Double, heightDiff: Double): Double {
        val cosTheta = cos(launchAngleRad)
        return sqrt(
            (g * distance * distance) /
                (2 * cosTheta * cosTheta * (distance * tan(launchAngleRad) - heightDiff))
        )
    }

    /**
     * Calculates the maximum height above the floor that the projectile reaches.
     *
     * @param launchSpeed Initial launch speed in inches per second.
     * @param launchAngleRad Launch angle above horizontal (radians).
     * @param launcherHeight Height of the launcher exit point above the floor (inches).
     * @return Maximum height above the floor in inches.
     */
    fun getMaxTrajectoryHeight(
        launchSpeed: Double,
        launchAngleRad: Double,
        launcherHeight: Double,
    ): Double {
        val vVertical = launchSpeed * sin(launchAngleRad)
        val maxHeightAboveLauncher = (vVertical * vVertical) / (2 * g)
        return launcherHeight + maxHeightAboveLauncher
    }

    /**
     * Computes the effective launch-speed-to-RPM factor for a given distance, applying a linear
     * correction centered around a reference distance.
     *
     * @param distance Distance to the goal (inches).
     * @param referenceDistance Distance at which [baseFactor] applies exactly.
     * @param baseFactor The base speed factor (launch speed in/s → flywheel RPM).
     * @param distanceCorrection Correction per inch of distance from reference.
     * @return Effective speed factor.
     */
    fun getEffectiveSpeedFactor(
        distance: Double,
        referenceDistance: Double,
        baseFactor: Double,
        distanceCorrection: Double,
    ): Double {
        val correction = distanceCorrection * (distance - referenceDistance)
        return baseFactor * (1 + correction)
    }

    /**
     * Computes the motion-compensated launch speed, accounting for robot velocity relative to the
     * goal. Flight time is set by the (unchanged) vertical kinematics, so the required ball
     * horizontal velocity in robot frame is (d/T − scale·vParallel, −scale·vPerp); launch speed is
     * its magnitude divided by cos(angle).
     *
     * @param distanceToGoal Horizontal distance to the goal (inches).
     * @param vParallel Robot velocity component toward the goal (in/s).
     * @param vPerp Robot velocity component perpendicular to goal direction (in/s).
     * @param launchAngleRad Launch angle above horizontal (radians).
     * @param heightDiff Height difference from launcher to target (inches).
     * @param speedCompensationScale Scale factor for velocity compensation (0–1).
     * @return Compensated launch speed (in/s), clamped to >= 0.
     */
    fun compensatedLaunchSpeed(
        distanceToGoal: Double,
        vParallel: Double,
        vPerp: Double,
        launchAngleRad: Double,
        heightDiff: Double,
        speedCompensationScale: Double,
    ): Double {
        val baseSpeed = getLaunchSpeed(distanceToGoal, launchAngleRad, heightDiff)
        val cosAngle = cos(launchAngleRad)
        // Parallel ball velocity needed in robot frame. If the robot approaches so fast that this
        // goes negative, the ball would have to travel backward toward the goal, which is
        // impossible; clamp it to zero so the compensated speed floors out rather than growing
        // again.
        val vHx = max(0.0, baseSpeed * cosAngle - speedCompensationScale * vParallel)
        val vHy = speedCompensationScale * vPerp
        return hypot(vHx, vHy) / cosAngle
    }
}
