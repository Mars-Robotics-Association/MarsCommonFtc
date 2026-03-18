package org.marsroboticsassociation.controllib.projectile;

/**
 * Pure-math projectile motion utilities for a fixed-angle launcher.
 * All methods are stateless — caller supplies every needed value.
 */
public final class ProjectileMotion {

    /** Gravitational acceleration in inches per second squared. */
    public static final double g = 386.09;

    private ProjectileMotion() {}

    /**
     * Calculates the required initial launch speed (in/s) for a projectile
     * to hit a target at the given horizontal distance and height difference.
     *
     * @param distance       Horizontal distance to the target (inches).
     * @param launchAngleRad Launch angle above horizontal (radians).
     * @param heightDiff     Vertical distance from launcher to target (inches, positive = target above launcher).
     * @return Required launch speed in inches per second.
     */
    public static double getLaunchSpeed(double distance, double launchAngleRad, double heightDiff) {
        double cosTheta = Math.cos(launchAngleRad);
        return Math.sqrt(
                (g * distance * distance) /
                        (2 * cosTheta * cosTheta * (distance * Math.tan(launchAngleRad) - heightDiff))
        );
    }

    /**
     * Calculates the maximum height above the floor that the projectile reaches.
     *
     * @param launchSpeed    Initial launch speed in inches per second.
     * @param launchAngleRad Launch angle above horizontal (radians).
     * @param launcherHeight Height of the launcher exit point above the floor (inches).
     * @return Maximum height above the floor in inches.
     */
    public static double getMaxTrajectoryHeight(double launchSpeed, double launchAngleRad, double launcherHeight) {
        double vVertical = launchSpeed * Math.sin(launchAngleRad);
        double maxHeightAboveLauncher = (vVertical * vVertical) / (2 * g);
        return launcherHeight + maxHeightAboveLauncher;
    }

    /**
     * Computes the effective launch-speed-to-RPM factor for a given distance,
     * applying a linear correction centered around a reference distance.
     *
     * @param distance           Distance to the goal (inches).
     * @param referenceDistance   Distance at which {@code baseFactor} applies exactly.
     * @param baseFactor         The base speed factor (launch speed in/s → flywheel RPM).
     * @param distanceCorrection Correction per inch of distance from reference.
     * @return Effective speed factor.
     */
    public static double getEffectiveSpeedFactor(double distance, double referenceDistance,
                                                 double baseFactor, double distanceCorrection) {
        double correction = distanceCorrection * (distance - referenceDistance);
        return baseFactor * (1 + correction);
    }

    /**
     * Computes the motion-compensated launch speed, accounting for robot velocity
     * along the goal direction.
     *
     * @param distanceToGoal         Horizontal distance to the goal (inches).
     * @param vParallel              Robot velocity component toward the goal (in/s).
     * @param launchAngleRad         Launch angle above horizontal (radians).
     * @param heightDiff             Height difference from launcher to target (inches).
     * @param speedCompensationScale Scale factor for velocity compensation (0–1).
     * @return Compensated launch speed (in/s), clamped to >= 0.
     */
    public static double compensatedLaunchSpeed(double distanceToGoal, double vParallel,
                                                double launchAngleRad, double heightDiff,
                                                double speedCompensationScale) {
        double baseSpeed = getLaunchSpeed(distanceToGoal, launchAngleRad, heightDiff);
        double cosAngle = Math.cos(launchAngleRad);
        return Math.max(0, baseSpeed - speedCompensationScale * vParallel / cosAngle);
    }
}
