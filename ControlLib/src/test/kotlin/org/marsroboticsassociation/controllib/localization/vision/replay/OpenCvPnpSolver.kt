package org.marsroboticsassociation.controllib.localization.vision.replay

import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver
import org.opencv.calib3d.Calib3d
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfDouble
import org.opencv.core.MatOfPoint2f
import org.opencv.core.MatOfPoint3f
import org.opencv.core.Point
import org.opencv.core.Point3

/**
 * Desktop OpenCV implementation of [PlanarPnpSolver] for the unit tests — the same
 * `solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE, ...)` marshalling the robot's `OpenCvPlanarPnpSolver`
 * does, but against the desktop natives (org.openpnp:opencv) so replay can re-solve the PnP
 * off-robot. Load the natives once with `nu.pattern.OpenCV.loadLocally()` (e.g. in a JUnit
 * `@BeforeAll`) before constructing this.
 */
class OpenCvPnpSolver : PlanarPnpSolver {

    override fun solveIppeSquare(
        objectPoints: DoubleArray,
        imagePoints: DoubleArray,
        cameraMatrix: DoubleArray,
        distCoeffs: DoubleArray?,
    ): List<PlanarPnpSolver.PnpSolution> {
        val camMat = Mat(3, 3, CvType.CV_64F)
        camMat.put(0, 0, *cameraMatrix)
        val dist =
            if (distCoeffs != null && distCoeffs.isNotEmpty()) {
                MatOfDouble(*distCoeffs)
            } else {
                MatOfDouble(0.0, 0.0, 0.0, 0.0, 0.0)
            }

        val op =
            Array(4) { i ->
                Point3(objectPoints[3 * i], objectPoints[3 * i + 1], objectPoints[3 * i + 2])
            }
        val objPts = MatOfPoint3f(*op)

        val ip =
            Array(4) { i ->
                Point(imagePoints[2 * i], imagePoints[2 * i + 1])
            }
        val imgPts = MatOfPoint2f(*ip)

        val rvecs = ArrayList<Mat>()
        val tvecs = ArrayList<Mat>()
        val reproj = Mat()
        val rvec = Mat()
        val tvec = Mat()
        val out = ArrayList<PlanarPnpSolver.PnpSolution>()
        try {
            val n =
                Calib3d.solvePnPGeneric(
                    objPts,
                    imgPts,
                    camMat,
                    dist,
                    rvecs,
                    tvecs,
                    false,
                    Calib3d.SOLVEPNP_IPPE_SQUARE,
                    rvec,
                    tvec,
                    reproj,
                )
            if (n < 1) {
                return out
            }
            val total = reproj.total().toInt()
            for (i in rvecs.indices) {
                var err = Double.NaN
                if (i < total) {
                    val v = reproj.get(i, 0)
                    err = if (v != null && v.isNotEmpty()) v[0] else Double.NaN
                }
                out.add(PlanarPnpSolver.PnpSolution(vec3(rvecs[i]), vec3(tvecs[i]), err))
            }
            return out
        } finally {
            camMat.release()
            dist.release()
            objPts.release()
            imgPts.release()
            reproj.release()
            rvec.release()
            tvec.release()
            for (m in rvecs) m.release()
            for (m in tvecs) m.release()
        }
    }

    companion object {
        private fun vec3(m: Mat): DoubleArray {
            val o = DoubleArray(3)
            m.get(0, 0, o)
            return o
        }
    }
}
