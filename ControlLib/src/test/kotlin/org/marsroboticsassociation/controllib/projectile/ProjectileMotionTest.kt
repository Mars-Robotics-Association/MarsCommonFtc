package org.marsroboticsassociation.controllib.projectile

import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.tan
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/** Unit tests for ProjectileMotion's pure-math methods. */
class ProjectileMotionTest {

    // Default launcher config values (mirrored from Launcher.LauncherConfig defaults)
    companion object {
        private const val LAUNCH_ANGLE_DEG = 67.0
        private val LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG)
        private const val GOAL_HEIGHT = 38.75
        private const val LAUNCHER_HEIGHT = 15.0
        private const val HEIGHT_DIFF = GOAL_HEIGHT - LAUNCHER_HEIGHT
        private const val LAUNCH_SPEED_FACTOR = 11.53
        private const val REFERENCE_DISTANCE = 60.0
        private const val DISTANCE_CORRECTION = 0.0005
        private const val SPEED_COMPENSATION_SCALE = 0.7
    }

    @Test
    fun testLaunchSpeedPositive() {
        assertTrue(
            ProjectileMotion.getLaunchSpeed(60.0, LAUNCH_ANGLE_RAD, HEIGHT_DIFF) > 0,
            "launch speed must be positive at a reachable distance",
        )
    }

    @Test
    fun testLaunchSpeedIncreasesWithDistance() {
        val speed60 = ProjectileMotion.getLaunchSpeed(60.0, LAUNCH_ANGLE_RAD, HEIGHT_DIFF)
        val speed120 = ProjectileMotion.getLaunchSpeed(120.0, LAUNCH_ANGLE_RAD, HEIGHT_DIFF)
        assertTrue(speed120 > speed60, "farther shots require higher launch speed")
    }

    @Test
    fun testLaunchSpeedMatchesFormula() {
        val d = 60.0
        val cosTheta = cos(LAUNCH_ANGLE_RAD)
        val expected =
            sqrt(
                (ProjectileMotion.g * d * d) /
                    (2 * cosTheta * cosTheta * (d * tan(LAUNCH_ANGLE_RAD) - HEIGHT_DIFF))
            )
        assertEquals(
            expected,
            ProjectileMotion.getLaunchSpeed(d, LAUNCH_ANGLE_RAD, HEIGHT_DIFF),
            1e-9,
        )
    }

    @Test
    fun testMaxTrajectoryHeightAtKnownSpeed() {
        val speed = 200.0
        val vVert = speed * sin(LAUNCH_ANGLE_RAD)
        val expected = LAUNCHER_HEIGHT + (vVert * vVert) / (2 * ProjectileMotion.g)
        assertEquals(
            expected,
            ProjectileMotion.getMaxTrajectoryHeight(speed, LAUNCH_ANGLE_RAD, LAUNCHER_HEIGHT),
            1e-6,
        )
    }

    @Test
    fun testMaxTrajectoryHeightAtZeroSpeedEqualsLauncherHeight() {
        assertEquals(
            LAUNCHER_HEIGHT,
            ProjectileMotion.getMaxTrajectoryHeight(0.0, LAUNCH_ANGLE_RAD, LAUNCHER_HEIGHT),
            1e-9,
            "zero launch speed → peak equals launcher height",
        )
    }

    @Test
    fun testEffectiveSpeedFactorAtReferenceDistance() {
        assertEquals(
            LAUNCH_SPEED_FACTOR,
            ProjectileMotion.getEffectiveSpeedFactor(
                REFERENCE_DISTANCE,
                REFERENCE_DISTANCE,
                LAUNCH_SPEED_FACTOR,
                DISTANCE_CORRECTION,
            ),
            1e-9,
            "at reference distance the correction term is zero",
        )
    }

    @Test
    fun testEffectiveSpeedFactorScalesLinearly() {
        val delta = 100.0
        val d = REFERENCE_DISTANCE + delta
        val expected = LAUNCH_SPEED_FACTOR * (1 + DISTANCE_CORRECTION * delta)
        assertEquals(
            expected,
            ProjectileMotion.getEffectiveSpeedFactor(
                d,
                REFERENCE_DISTANCE,
                LAUNCH_SPEED_FACTOR,
                DISTANCE_CORRECTION,
            ),
            1e-9,
        )
    }

    @Test
    fun testCompensatedSpeedStationaryEqualsBase() {
        val d = 60.0
        val base = ProjectileMotion.getLaunchSpeed(d, LAUNCH_ANGLE_RAD, HEIGHT_DIFF)
        assertEquals(
            base,
            ProjectileMotion.compensatedLaunchSpeed(
                d,
                0.0,
                0.0,
                LAUNCH_ANGLE_RAD,
                HEIGHT_DIFF,
                SPEED_COMPENSATION_SCALE,
            ),
            1e-9,
            "no robot motion → compensated speed equals base speed",
        )
    }

    @Test
    fun testCompensatedSpeedReducedWhenApproaching() {
        val d = 60.0
        val base = ProjectileMotion.getLaunchSpeed(d, LAUNCH_ANGLE_RAD, HEIGHT_DIFF)
        assertTrue(
            ProjectileMotion.compensatedLaunchSpeed(
                d,
                50.0,
                0.0,
                LAUNCH_ANGLE_RAD,
                HEIGHT_DIFF,
                SPEED_COMPENSATION_SCALE,
            ) < base,
            "approaching the goal → lower launch speed needed",
        )
    }

    @Test
    fun testCompensatedSpeedClampedAtZero() {
        assertEquals(
            0.0,
            ProjectileMotion.compensatedLaunchSpeed(
                60.0,
                100_000.0,
                0.0,
                LAUNCH_ANGLE_RAD,
                HEIGHT_DIFF,
                SPEED_COMPENSATION_SCALE,
            ),
            "compensated speed is clamped at zero",
        )
    }
}
