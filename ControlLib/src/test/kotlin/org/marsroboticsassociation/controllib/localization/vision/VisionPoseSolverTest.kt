package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * OpenCV-free unit tests for the "DIY MegaTag2" transform chain and heading disambiguation. The
 * real `solvePnPGeneric` candidate recovery is OpenCV/Android-only and validated on-robot; this
 * pins the pure geometry around it.
 *
 * <p>Strategy: a **forward model** synthesizes the exact camera-relative tag pose a perfect camera
 * would see from a known robot pose (`cameraFromTag = (fieldFromRobot · robotFromCamera)^-1 ·
 * fieldFromTag`), then we assert [VisionPoseSolver.solve] inverts it back to the robot pose.
 * Because the chain only round-trips if [Transform3D.multiply]/[Transform3D.inverse] are both
 * correct, this exercises the whole math — and a separate hand-computed case guards against a
 * forward/inverse bug that could cancel.
 */
class VisionPoseSolverTest {

    companion object {
        private const val TOL = 1e-9
        private const val TAG = 7

        /** √½ — the exact w and z of a unit quaternion for a 90° rotation about +z. */
        private val ROOT_HALF = sqrt(0.5)

        /** The camera-relative tag pose a perfect camera at [fieldFromRobot] would observe. */
        private fun forwardCameraFromTag(
            fieldFromRobot: Transform3D,
            robotFromCamera: Transform3D,
            fieldFromTag: Transform3D,
        ): Transform3D {
            val fieldFromCamera = fieldFromRobot.multiply(robotFromCamera)
            return fieldFromCamera.inverse().multiply(fieldFromTag)
        }

        private fun tagTable(fieldFromTag: Transform3D): Map<Int, Transform3D> {
            val m = HashMap<Int, Transform3D>()
            m[TAG] = fieldFromTag
            return m
        }

        private fun assertPose(x: Double, y: Double, headingRad: Double, actual: Pose2d) {
            assertEquals(x, actual.x, 1e-7)
            assertEquals(y, actual.y, 1e-7)
            assertEquals(headingRad, actual.rotation.radians, 1e-7)
        }
    }

    // --- Transform3D primitives (the chain rests on these) -----------------------------------

    @Test
    fun inverseRoundTripsToIdentity() {
        val t = Transform3D.fromTranslationYPR(12.0, -5.0, 3.0, 0.7, -0.2, 0.4)
        val shouldBeI = t.multiply(t.inverse())
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                assertEquals(if (i == j) 1.0 else 0.0, shouldBeI.r[i][j], TOL)
            }
            assertEquals(0.0, shouldBeI.t[i], TOL)
        }
    }

    @Test
    fun yawComesFromYprYaw() {
        assertEquals(0.9, Transform3D.fromTranslationYPR(0.0, 0.0, 0.0, 0.9, 0.3, -0.1).yaw(), TOL)
    }

    @Test
    fun rodriguesZeroVectorIsIdentityRotationWithTranslation() {
        val t =
            Transform3D.fromRodrigues(doubleArrayOf(0.0, 0.0, 0.0), doubleArrayOf(1.0, 2.0, 3.0))
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                assertEquals(if (i == j) 1.0 else 0.0, t.r[i][j], TOL)
            }
        }
        assertEquals(1.0, t.x(), TOL)
        assertEquals(2.0, t.y(), TOL)
        assertEquals(3.0, t.z(), TOL)
    }

    @Test
    fun rodriguesAboutZMatchesYaw() {
        // A Rodrigues vector along +z of magnitude θ is a yaw of θ.
        val yaw = 0.42
        val t =
            Transform3D.fromRodrigues(doubleArrayOf(0.0, 0.0, yaw), doubleArrayOf(0.0, 0.0, 0.0))
        assertEquals(yaw, t.yaw(), TOL)
        assertEquals(kotlin.math.cos(yaw), t.r[0][0], TOL)
        assertEquals(-kotlin.math.sin(yaw), t.r[0][1], TOL)
    }

    @Test
    fun rodriguesProducesAProperRotation() {
        // Arbitrary axis/angle → orthonormal, det +1 (a rotation, not a reflection/scale).
        val t =
            Transform3D.fromRodrigues(doubleArrayOf(0.3, -0.7, 1.1), doubleArrayOf(0.0, 0.0, 0.0))
        val shouldBeI = t.multiply(t.inverse())
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                assertEquals(if (i == j) 1.0 else 0.0, shouldBeI.r[i][j], 1e-12)
            }
        }
        val r = t.r
        val det =
            r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
                r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
                r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0])
        assertEquals(1.0, det, 1e-12)
    }

    @Test
    fun yawOnlyQuaternionMatchesYprYaw() {
        val yaw = 1.1
        val q =
            Transform3D.fromTranslationQuaternion(
                0.0,
                0.0,
                0.0,
                kotlin.math.cos(yaw / 2),
                0.0,
                0.0,
                kotlin.math.sin(yaw / 2),
            )
        assertEquals(yaw, q.yaw(), TOL)
    }

    // --- Hand-computed sanity case ------------------------------------------------------------

    @Test
    fun recoversOriginWithIdentityExtrinsicAndTagAhead() {
        // Camera == robot center (identity extrinsic), robot at the field origin facing +x, so the
        // camera frame coincides with the field frame. A tag 60 in ahead, axis-aligned, therefore
        // sits at camera coords (60, 0, 0) with identity rotation.
        val solver =
            VisionPoseSolver(
                Transform3D.identity(),
                tagTable(Transform3D.fromTranslationYPR(60.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            )
        val cameraFromTag = Transform3D.fromTranslationYPR(60.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        val r = solver.solve(TAG, arrayOf(cameraFromTag), 0.0)!!

        assertPose(0.0, 0.0, 0.0, r.fieldPose)
        assertEquals(0.0, r.yawResidualRad, TOL)
        assertEquals(0, r.chosenIndex)
    }

    // --- Forward-model round trips (arbitrary poses) ------------------------------------------

    @Test
    fun recoversArbitraryPoseThroughNonTrivialExtrinsicAndTag() {
        val fieldFromRobot = Transform3D.fromTranslationYPR(84.0, 36.0, 0.0, 1.2, 0.0, 0.0)
        // A camera mounted forward-left, raised, yawed 15 deg and pitched up 10 deg on the robot.
        val robotFromCamera =
            Transform3D.fromTranslationYPR(
                7.0,
                3.0,
                9.0,
                Math.toRadians(15.0),
                Math.toRadians(-10.0),
                0.0,
            )
        val fieldFromTag =
            Transform3D.fromTranslationQuaternion(120.0, 50.0, 20.0, ROOT_HALF, 0.0, 0.0, ROOT_HALF)

        val solver = VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag))
        val trueCam = forwardCameraFromTag(fieldFromRobot, robotFromCamera, fieldFromTag)

        val r = solver.solve(TAG, arrayOf(trueCam), fieldFromRobot.yaw())!!

        assertPose(fieldFromRobot.x(), fieldFromRobot.y(), fieldFromRobot.yaw(), r.fieldPose)
        assertEquals(0.0, r.yawResidualRad, 1e-7)
    }

    @Test
    fun picksHeadingConsistentCandidateOverFlip() {
        val robotFromCamera = Transform3D.fromTranslationYPR(6.0, 0.0, 8.0, 0.0, 0.0, 0.0)
        val fieldFromTag =
            Transform3D.fromTranslationQuaternion(120.0, 50.0, 20.0, ROOT_HALF, 0.0, 0.0, ROOT_HALF)
        val solver = VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag))

        // The true pose (heading ~0.30 rad) and a decoy "flip" pose well away in heading (~2.7
        // rad).
        val trueRobot = Transform3D.fromTranslationYPR(70.0, 40.0, 0.0, 0.30, 0.0, 0.0)
        val decoyRobot = Transform3D.fromTranslationYPR(95.0, 15.0, 0.0, 2.70, 0.0, 0.0)
        val trueCam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag)
        val decoyCam = forwardCameraFromTag(decoyRobot, robotFromCamera, fieldFromTag)

        // Decoy first, so a correct pick can't be an artifact of preferring index 0.
        val r = solver.solve(TAG, arrayOf(decoyCam, trueCam), trueRobot.yaw())!!

        assertEquals(1, r.chosenIndex)
        assertPose(trueRobot.x(), trueRobot.y(), trueRobot.yaw(), r.fieldPose)
        assertTrue(r.yawResidualRad < 1e-6, "residual should be tiny for the matching candidate")
    }

    @Test
    fun substitutesFedHeadingRatherThanSolvedYaw() {
        // Build the candidate from a robot at heading exactly 0, but feed a heading 2 deg off. The
        // result must carry the *fed* heading (the "zero the rotational DOF" step), and the
        // residual
        // must report the 2 deg gap against the solve's own yaw.
        val robotFromCamera = Transform3D.fromTranslationYPR(6.0, 0.0, 8.0, 0.0, 0.0, 0.0)
        val fieldFromTag =
            Transform3D.fromTranslationQuaternion(120.0, 50.0, 20.0, ROOT_HALF, 0.0, 0.0, ROOT_HALF)
        val solver = VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag))

        val trueRobot = Transform3D.fromTranslationYPR(70.0, 40.0, 0.0, 0.0, 0.0, 0.0)
        val cam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag)
        val fedHeading = Math.toRadians(2.0)

        val r = solver.solve(TAG, arrayOf(cam), fedHeading)!!

        assertEquals(fedHeading, r.fieldPose.rotation.radians, TOL)
        assertEquals(0.0, r.solvedYawRad, 1e-7)
        assertEquals(Math.toRadians(2.0), r.yawResidualRad, 1e-7)
    }

    // --- Mirror branch (full 6-DOF, heading-free disambiguation) ------------------------------

    @Test
    fun solveBranchesReturnsBothFull6DofPoses() {
        val robotFromCamera = Transform3D.fromTranslationYPR(6.0, 0.0, 8.0, 0.0, 0.0, 0.0)
        val fieldFromTag =
            Transform3D.fromTranslationQuaternion(120.0, 50.0, 20.0, ROOT_HALF, 0.0, 0.0, ROOT_HALF)
        val solver = VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag))

        // True branch: a level robot on the floor. Mirror: floating and tilted (a rotational flip).
        val trueRobot = Transform3D.fromTranslationYPR(70.0, 40.0, 0.0, 0.30, 0.0, 0.0)
        val mirrorRobot = Transform3D.fromTranslationYPR(95.0, 15.0, 9.0, 2.70, 0.5, 0.0)
        val trueCam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag)
        val mirrorCam = forwardCameraFromTag(mirrorRobot, robotFromCamera, fieldFromTag)

        val branches = solver.solveBranches(TAG, arrayOf(trueCam, mirrorCam))!!

        assertEquals(2, branches.size)
        // Branch 0 reconstructs the level/on-floor robot to full 6-DOF.
        assertEquals(0, branches[0].index)
        assertEquals(70.0, branches[0].fieldFromRobot.x(), 1e-7)
        assertEquals(40.0, branches[0].fieldFromRobot.y(), 1e-7)
        assertEquals(0.30, branches[0].yawRad, 1e-7)
        assertEquals(0.0, branches[0].zOffset, 1e-7)
        assertEquals(0.0, branches[0].tiltRad, 1e-7)
        // Branch 1 reconstructs the floating/tilted mirror — admissibility diagnostics flag it.
        assertEquals(1, branches[1].index)
        assertEquals(9.0, branches[1].zOffset, 1e-7)
        assertEquals(0.5, branches[1].tiltRad, 1e-7)
    }

    @Test
    fun admissibleBranchPicksTheLevelOneWithoutAHeading() {
        val robotFromCamera = Transform3D.fromTranslationYPR(6.0, 0.0, 8.0, 0.0, 0.0, 0.0)
        val fieldFromTag =
            Transform3D.fromTranslationQuaternion(120.0, 50.0, 20.0, ROOT_HALF, 0.0, 0.0, ROOT_HALF)
        val solver = VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag))

        val trueRobot = Transform3D.fromTranslationYPR(70.0, 40.0, 0.0, 0.30, 0.0, 0.0)
        val tiltedMirror = Transform3D.fromTranslationYPR(95.0, 15.0, 9.0, 2.70, 0.5, 0.0)
        val trueCam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag)
        val mirrorCam = forwardCameraFromTag(tiltedMirror, robotFromCamera, fieldFromTag)

        // Mirror first, so the pick can't be an artifact of preferring index 0 — and no heading
        // fed.
        val picked = solver.admissibleBranch(TAG, arrayOf(mirrorCam, trueCam))!!

        assertEquals(1, picked.index)
        assertEquals(70.0, picked.fieldFromRobot.x(), 1e-7)
        assertEquals(40.0, picked.fieldFromRobot.y(), 1e-7)
        assertEquals(0.0, picked.tiltRad, 1e-7)
    }

    @Test
    fun solveBranchesGuardsUnknownTagAndEmpty() {
        val solver = VisionPoseSolver(Transform3D.identity(), tagTable(Transform3D.identity()))
        assertNull(solver.solveBranches(999, arrayOf(Transform3D.identity())))
        assertNull(solver.solveBranches(TAG, arrayOf()))
        assertNull(solver.solveBranches(TAG, null))
        assertNull(solver.admissibleBranch(TAG, arrayOf(null)))
    }

    // --- Guards -------------------------------------------------------------------------------

    @Test
    fun returnsNullForUnknownTag() {
        val solver = VisionPoseSolver(Transform3D.identity(), tagTable(Transform3D.identity()))
        assertNull(solver.solve(999, arrayOf(Transform3D.identity()), 0.0))
    }

    @Test
    fun returnsNullForNoCandidates() {
        val solver = VisionPoseSolver(Transform3D.identity(), tagTable(Transform3D.identity()))
        assertNull(solver.solve(TAG, arrayOf(), 0.0))
        assertNull(solver.solve(TAG, null, 0.0))
    }
}
