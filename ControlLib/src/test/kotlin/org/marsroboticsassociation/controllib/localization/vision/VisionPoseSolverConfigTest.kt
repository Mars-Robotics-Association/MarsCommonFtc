package org.marsroboticsassociation.controllib.localization.vision

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Pure (OpenCV-free) checks that [VisionPoseSolverConfig] composes a Limelight `rs*` extrinsic and
 * freezes a caller-supplied tag table — guarding against an index typo in the 4×4 extraction or a
 * transposed axis. Does not assert the extrinsic *rotation* convention. Fixture numbers below are
 * one robot/field pair (Curiosity DECODE); real robots/seasons supply their own from TeamCode.
 */
class VisionPoseSolverConfigTest {

    companion object {
        private const val TOL = 1e-9

        // Example fixture only — not a library default. Matches curiosity_Limelight (5).vpr /
        // hand-eye.
        private const val FIXTURE_FORWARD_M = 0.0816
        private const val FIXTURE_SIDE_M = -0.0400
        private const val FIXTURE_UP_M = 0.3835
        private const val FIXTURE_ROLL_DEG = 180.0
        private const val FIXTURE_PITCH_DEG = 0.23
        private const val FIXTURE_YAW_DEG = -0.78

        // ftc2025DECODE.fmap goal tags (fixture for table-parse tests).
        private val TAG20_FIELD_FROM_TAG =
            doubleArrayOf(
                0.5877852522924731,
                -0.8090169943749473,
                0.0,
                -1.4827,
                0.8090169943749473,
                0.5877852522924731,
                0.0,
                -1.4133,
                0.0,
                0.0,
                1.0,
                0.7493,
                0.0,
                0.0,
                0.0,
                1.0,
            )
        private val TAG24_FIELD_FROM_TAG =
            doubleArrayOf(
                0.5877852522924731,
                0.8090169943749473,
                0.0,
                -1.4827,
                -0.8090169943749473,
                0.5877852522924731,
                0.0,
                1.4133,
                0.0,
                0.0,
                1.0,
                0.7493,
                0.0,
                0.0,
                0.0,
                1.0,
            )

        private fun fixtureExtrinsic(): Transform3D {
            return VisionPoseSolverConfig.robotFromCameraFromLimelightRs(
                FIXTURE_FORWARD_M,
                FIXTURE_SIDE_M,
                FIXTURE_UP_M,
                FIXTURE_ROLL_DEG,
                FIXTURE_PITCH_DEG,
                FIXTURE_YAW_DEG,
            )
        }

        private fun fixtureTagMap(): Map<Int, Transform3D> {
            val m = HashMap<Int, Transform3D>()
            m[20] = VisionPoseSolverConfig.fromRowMajor4x4(TAG20_FIELD_FROM_TAG)
            m[24] = VisionPoseSolverConfig.fromRowMajor4x4(TAG24_FIELD_FROM_TAG)
            return m.toMap()
        }

        private fun fixtureConfig(): VisionPoseSolverConfig {
            return VisionPoseSolverConfig(fixtureExtrinsic(), fixtureTagMap())
        }
    }

    @Test
    fun tagTableHoldsBothDecodeGoalTags() {
        val cfg = fixtureConfig()
        assertTrue(cfg.tagFieldPoses().containsKey(20))
        assertTrue(cfg.tagFieldPoses().containsKey(24))
        assertEquals(2, cfg.tagFieldPoses().size)
    }

    @Test
    fun tag20PositionAndYawMatchFmap() {
        val t = fixtureConfig().tagFieldPoses()[20]!!
        assertEquals(-1.4827, t.x(), TOL)
        assertEquals(-1.4133, t.y(), TOL)
        assertEquals(0.7493, t.z(), TOL) // tag height, metres
        assertEquals(Math.toRadians(54.0), t.yaw(), 1e-6)
    }

    @Test
    fun tag24MirrorsTag20InYAndYaw() {
        val t = fixtureConfig().tagFieldPoses()[24]!!
        assertEquals(-1.4827, t.x(), TOL)
        assertEquals(1.4133, t.y(), TOL)
        assertEquals(Math.toRadians(-54.0), t.yaw(), 1e-6)
    }

    @Test
    fun limelightRsTranslationMatchesInput() {
        // The translation is an exact fact (unlike the rotation hypothesis): forward/side/up
        // metres.
        val e = fixtureExtrinsic()
        assertEquals(FIXTURE_FORWARD_M, e.x(), TOL)
        assertEquals(FIXTURE_SIDE_M, e.y(), TOL)
        assertEquals(FIXTURE_UP_M, e.z(), TOL)
    }

    @Test
    fun fromRowMajor4x4ExtractsTranslationAndRotationBlock() {
        val t = VisionPoseSolverConfig.fromRowMajor4x4(TAG20_FIELD_FROM_TAG)
        assertEquals(-1.4827, t.x(), TOL)
        assertEquals(-1.4133, t.y(), TOL)
        assertEquals(0.7493, t.z(), TOL)
        assertEquals(0.5877852522924731, t.r[0][0], TOL)
        assertEquals(-0.8090169943749473, t.r[0][1], TOL)
    }

    @Test
    fun opticalToRobotBaseIsAProperRotation() {
        // det = +1 and orthonormal: a real rotation, not a reflection (a reflection would silently
        // mirror every recovered pose).
        val b = VisionPoseSolverConfig.OPTICAL_TO_ROBOT_BASE
        val r = b.r
        val det =
            r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
        assertEquals(1.0, det, TOL)
    }
}
