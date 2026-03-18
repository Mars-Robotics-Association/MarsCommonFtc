package org.marsroboticsassociation.controllib.projectile;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for ProjectileMotion's pure-math static methods.
 */
class ProjectileMotionTest {

    // Default launcher config values (mirrored from Launcher.LauncherConfig defaults)
    private static final double LAUNCH_ANGLE_DEG = 67.0;
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double GOAL_HEIGHT = 38.75;
    private static final double LAUNCHER_HEIGHT = 15.0;
    private static final double HEIGHT_DIFF = GOAL_HEIGHT - LAUNCHER_HEIGHT;
    private static final double LAUNCH_SPEED_FACTOR = 11.53;
    private static final double REFERENCE_DISTANCE = 60.0;
    private static final double DISTANCE_CORRECTION = 0.0005;
    private static final double SPEED_COMPENSATION_SCALE = 0.7;

    // ── getLaunchSpeed ────────────────────────────────────────────────────────

    @Test
    void testLaunchSpeedPositive() {
        assertTrue(ProjectileMotion.getLaunchSpeed(60.0, LAUNCH_ANGLE_RAD, HEIGHT_DIFF) > 0,
                "launch speed must be positive at a reachable distance");
    }

    @Test
    void testLaunchSpeedIncreasesWithDistance() {
        double speed60 = ProjectileMotion.getLaunchSpeed(60.0, LAUNCH_ANGLE_RAD, HEIGHT_DIFF);
        double speed120 = ProjectileMotion.getLaunchSpeed(120.0, LAUNCH_ANGLE_RAD, HEIGHT_DIFF);
        assertTrue(speed120 > speed60, "farther shots require higher launch speed");
    }

    @Test
    void testLaunchSpeedMatchesFormula() {
        double d = 60.0;
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double expected = Math.sqrt(
                (ProjectileMotion.g * d * d) /
                        (2 * cosTheta * cosTheta * (d * Math.tan(LAUNCH_ANGLE_RAD) - HEIGHT_DIFF))
        );
        assertEquals(expected, ProjectileMotion.getLaunchSpeed(d, LAUNCH_ANGLE_RAD, HEIGHT_DIFF), 1e-9);
    }

    // ── getMaxTrajectoryHeight ────────────────────────────────────────────────

    @Test
    void testMaxTrajectoryHeightAtKnownSpeed() {
        double speed = 200.0;
        double vVert = speed * Math.sin(LAUNCH_ANGLE_RAD);
        double expected = LAUNCHER_HEIGHT + (vVert * vVert) / (2 * ProjectileMotion.g);
        assertEquals(expected,
                ProjectileMotion.getMaxTrajectoryHeight(speed, LAUNCH_ANGLE_RAD, LAUNCHER_HEIGHT), 1e-6);
    }

    @Test
    void testMaxTrajectoryHeightAtZeroSpeedEqualsLauncherHeight() {
        assertEquals(LAUNCHER_HEIGHT,
                ProjectileMotion.getMaxTrajectoryHeight(0.0, LAUNCH_ANGLE_RAD, LAUNCHER_HEIGHT), 1e-9,
                "zero launch speed → peak equals launcher height");
    }

    // ── getEffectiveSpeedFactor ───────────────────────────────────────────────

    @Test
    void testEffectiveSpeedFactorAtReferenceDistance() {
        assertEquals(LAUNCH_SPEED_FACTOR,
                ProjectileMotion.getEffectiveSpeedFactor(REFERENCE_DISTANCE,
                        REFERENCE_DISTANCE, LAUNCH_SPEED_FACTOR, DISTANCE_CORRECTION), 1e-9,
                "at reference distance the correction term is zero");
    }

    @Test
    void testEffectiveSpeedFactorScalesLinearly() {
        double delta = 100.0;
        double d = REFERENCE_DISTANCE + delta;
        double expected = LAUNCH_SPEED_FACTOR * (1 + DISTANCE_CORRECTION * delta);
        assertEquals(expected,
                ProjectileMotion.getEffectiveSpeedFactor(d,
                        REFERENCE_DISTANCE, LAUNCH_SPEED_FACTOR, DISTANCE_CORRECTION), 1e-9);
    }

    // ── compensatedLaunchSpeed ────────────────────────────────────────────────

    @Test
    void testCompensatedSpeedStationaryEqualsBase() {
        double d = 60.0;
        double base = ProjectileMotion.getLaunchSpeed(d, LAUNCH_ANGLE_RAD, HEIGHT_DIFF);
        assertEquals(base,
                ProjectileMotion.compensatedLaunchSpeed(d, 0.0,
                        LAUNCH_ANGLE_RAD, HEIGHT_DIFF, SPEED_COMPENSATION_SCALE), 1e-9,
                "no robot motion → compensated speed equals base speed");
    }

    @Test
    void testCompensatedSpeedReducedWhenApproaching() {
        double d = 60.0;
        double base = ProjectileMotion.getLaunchSpeed(d, LAUNCH_ANGLE_RAD, HEIGHT_DIFF);
        assertTrue(ProjectileMotion.compensatedLaunchSpeed(d, 50.0,
                        LAUNCH_ANGLE_RAD, HEIGHT_DIFF, SPEED_COMPENSATION_SCALE) < base,
                "approaching the goal → lower launch speed needed");
    }

    @Test
    void testCompensatedSpeedClampedAtZero() {
        assertEquals(0.0, ProjectileMotion.compensatedLaunchSpeed(60.0, 100_000.0,
                        LAUNCH_ANGLE_RAD, HEIGHT_DIFF, SPEED_COMPENSATION_SCALE),
                "compensated speed is clamped at zero");
    }
}
