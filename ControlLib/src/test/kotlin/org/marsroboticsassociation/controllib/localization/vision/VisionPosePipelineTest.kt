package org.marsroboticsassociation.controllib.localization.vision

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeAll
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.localization.vision.replay.OpenCvPnpSolver
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat

/**
 * End-to-end DIY-MegaTag2 pipeline test through the *real* OpenCV solver, run on the desktop JVM
 * against `org.openpnp:opencv` (see ControlLib/build.gradle). Where [VisionPoseSolverTest] feeds
 * the solver synthetic `cameraFromTag` candidates, this drives the whole chain:
 * ```
 * known world → project tag corners → TagAmbiguitySolver.solve() (real solvePnPGeneric)
 *             → Transform3D.fromRodrigues → VisionPoseSolver.solve() → recovered robot pose
 * ```
 *
 * so it exercises the actual PnP, the actual corner ordering, and [Transform3D.fromRodrigues]
 * against OpenCV's real `rvec` output. (Clean synthetic corners only: this proves the code/math
 * path, not robustness to the noisy detections that cause real on-robot flips.)
 */
class VisionPosePipelineTest {

    companion object {
        private const val FX = 800.0
        private const val FY = 800.0
        private const val CX = 640.0
        private const val CY = 480.0
        private val CAM_MAT = doubleArrayOf(FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0)
        private val NO_DIST = DoubleArray(0)
        private const val TAG = 0.1651 // 165.1 mm — the DECODE field tag
        private const val TAG_ID = 11

        @JvmStatic
        @BeforeAll
        fun loadNative() {
            nu.pattern.OpenCV.loadLocally()
        }

        /** PnpSolutions → 1–2 `cameraFromTag` candidates. */
        private fun candidatesFrom(s: TagAmbiguitySolver.PnpSolutions): Array<Transform3D> {
            val best = Transform3D.fromRodrigues(s.rvecBest, s.tvecBest)
            if (s.rvecAlt == null) {
                return arrayOf(best)
            }
            return arrayOf(best, Transform3D.fromRodrigues(s.rvecAlt, s.tvecAlt!!))
        }

        /**
         * Projects the tag's four corners (squareObjectPoints order) through a clean pinhole model.
         */
        private fun project(cameraFromTag: Transform3D): List<List<Double>> {
            val h = TAG / 2.0
            val obj =
                arrayOf(
                    doubleArrayOf(-h, h, 0.0),
                    doubleArrayOf(h, h, 0.0),
                    doubleArrayOf(h, -h, 0.0),
                    doubleArrayOf(-h, -h, 0.0),
                ) // TL, TR, BR, BL
            val out = ArrayList<List<Double>>()
            for (o in obj) {
                val cam = matVec(cameraFromTag.r, o)
                cam[0] += cameraFromTag.t[0]
                cam[1] += cameraFromTag.t[1]
                cam[2] += cameraFromTag.t[2]
                out.add(listOf(FX * cam[0] / cam[2] + CX, FY * cam[1] / cam[2] + CY))
            }
            return out
        }

        private fun rotY(theta: Double): Array<DoubleArray> {
            val c = kotlin.math.cos(theta)
            val s = kotlin.math.sin(theta)
            return arrayOf(
                doubleArrayOf(c, 0.0, s),
                doubleArrayOf(0.0, 1.0, 0.0),
                doubleArrayOf(-s, 0.0, c),
            )
        }

        private fun matVec(m: Array<DoubleArray>, v: DoubleArray): DoubleArray {
            return doubleArrayOf(
                m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
                m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
                m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2],
            )
        }
    }

    /**
     * Full round trip: a self-consistent synthetic world (metres) with the tag visibly in front of
     * the camera and well-conditioned (25° oblique). We derive `fieldFromTag` from the world,
     * project the tag's corners, run the real solve, and assert the pipeline recovers the known
     * robot pose. With clean corners the recovery is essentially exact.
     */
    @Test
    fun recoversKnownRobotPoseThroughRealSolve() {
        // Pose the tag in the camera optical frame: 1.5 m ahead, slightly off-centre, 25° about y.
        val cameraFromTag = Transform3D(rotY(Math.toRadians(25.0)), doubleArrayOf(0.1, -0.05, 1.5))
        // Extrinsic: optical→robot axis swap (camera looks along robot +x), 0.1 m fwd, 0.2 m up.
        val robotFromCamera =
            Transform3D(
                arrayOf(
                    doubleArrayOf(0.0, 0.0, 1.0),
                    doubleArrayOf(-1.0, 0.0, 0.0),
                    doubleArrayOf(0.0, -1.0, 0.0),
                ),
                doubleArrayOf(0.1, 0.0, 0.2),
            )
        // Known robot pose to recover: field (2.0, 1.0) m, heading 0.3 rad, level.
        val fieldFromRobot = Transform3D.fromTranslationYPR(2.0, 1.0, 0.0, 0.3, 0.0, 0.0)
        // Derive the tag's field pose from the chain so the world is self-consistent.
        val fieldFromTag = fieldFromRobot.multiply(robotFromCamera).multiply(cameraFromTag)

        val table = HashMap<Int, Transform3D>()
        table[TAG_ID] = fieldFromTag
        val solver = VisionPoseSolver(robotFromCamera, table)

        val est = TagAmbiguitySolver(OpenCvPnpSolver(), CAM_MAT, NO_DIST, TAG)
        val s = est.solve(project(cameraFromTag))
        assertNotNull(s, "real solve returned null")

        val candidates = candidatesFrom(s!!)
        val r = solver.solve(TAG_ID, candidates, fieldFromRobot.yaw())
        assertNotNull(r)

        // Position recovered from the chosen candidate; heading is the fed value (exact).
        assertEquals(2.0, r!!.fieldPose.x, 2e-3)
        assertEquals(1.0, r.fieldPose.y, 2e-3)
        assertEquals(0.3, r.fieldPose.rotation.radians, 1e-12)
        // The chosen candidate's own solved yaw agrees with truth → the right candidate won and the
        // corner order / fromRodrigues / chain are all consistent end to end.
        assertTrue(
            r.yawResidualRad < 0.02,
            "solved yaw should agree with truth: residual ${r.yawResidualRad}",
        )
    }

    /**
     * Pins the pure [Transform3D.fromRodrigues] to OpenCV's `Calib3d.Rodrigues` across several
     * rotation vectors — what lets `fromRodrigues` stay in the OpenCV-free tier without risking
     * divergence from the real thing.
     */
    @Test
    fun fromRodriguesMatchesOpenCvRodrigues() {
        val rvecs =
            arrayOf(
                doubleArrayOf(0.0, 0.0, 0.42),
                doubleArrayOf(0.3, -0.7, 1.1),
                doubleArrayOf(1.5, 0.0, 0.0),
                doubleArrayOf(-0.2, 0.9, -0.4),
                doubleArrayOf(0.0, 0.0, 0.0),
            )
        for (rvec in rvecs) {
            val rv = Mat(3, 1, CvType.CV_64F)
            rv.put(0, 0, *rvec)
            val rMat = Mat()
            try {
                Calib3d.Rodrigues(rv, rMat)
                val t = Transform3D.fromRodrigues(rvec, doubleArrayOf(0.0, 0.0, 0.0))
                for (i in 0 until 3) {
                    for (j in 0 until 3) {
                        assertEquals(
                            rMat.get(i, j)[0],
                            t.r[i][j],
                            1e-12,
                            "rvec=${rvec.contentToString()} [$i][$j]",
                        )
                    }
                }
            } finally {
                rv.release()
                rMat.release()
            }
        }
    }
}
