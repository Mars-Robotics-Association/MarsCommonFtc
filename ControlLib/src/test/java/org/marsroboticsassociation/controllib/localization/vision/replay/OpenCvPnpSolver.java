package org.marsroboticsassociation.controllib.localization.vision.replay;

import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;

import java.util.ArrayList;
import java.util.List;

/**
 * Desktop OpenCV implementation of {@link PlanarPnpSolver} for the unit tests — the same {@code
 * solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE, ...)} marshalling the robot's {@code
 * OpenCvPlanarPnpSolver} does, but against the desktop natives (org.openpnp:opencv) so replay can
 * re-solve the PnP off-robot. Load the natives once with {@code nu.pattern.OpenCV.loadLocally()}
 * (e.g. in a JUnit {@code @BeforeAll}) before constructing this.
 */
public final class OpenCvPnpSolver implements PlanarPnpSolver {

    @Override
    public List<PnpSolution> solveIppeSquare(
            double[] objectPoints, double[] imagePoints, double[] cameraMatrix, double[] distCoeffs) {
        Mat camMat = new Mat(3, 3, CvType.CV_64F);
        camMat.put(0, 0, cameraMatrix);
        MatOfDouble dist =
                (distCoeffs != null && distCoeffs.length > 0)
                        ? new MatOfDouble(distCoeffs)
                        : new MatOfDouble(0, 0, 0, 0, 0);

        Point3[] op = new Point3[4];
        for (int i = 0; i < 4; i++) {
            op[i] = new Point3(objectPoints[3 * i], objectPoints[3 * i + 1], objectPoints[3 * i + 2]);
        }
        MatOfPoint3f objPts = new MatOfPoint3f(op);

        Point[] ip = new Point[4];
        for (int i = 0; i < 4; i++) {
            ip[i] = new Point(imagePoints[2 * i], imagePoints[2 * i + 1]);
        }
        MatOfPoint2f imgPts = new MatOfPoint2f(ip);

        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        Mat reproj = new Mat();
        Mat rvec = new Mat();
        Mat tvec = new Mat();
        List<PnpSolution> out = new ArrayList<>();
        try {
            int n =
                    Calib3d.solvePnPGeneric(
                            objPts, imgPts, camMat, dist, rvecs, tvecs, false,
                            Calib3d.SOLVEPNP_IPPE_SQUARE, rvec, tvec, reproj);
            if (n < 1) {
                return out;
            }
            int total = (int) reproj.total();
            for (int i = 0; i < rvecs.size(); i++) {
                double err = Double.NaN;
                if (i < total) {
                    double[] v = reproj.get(i, 0);
                    err = (v != null && v.length > 0) ? v[0] : Double.NaN;
                }
                out.add(new PnpSolution(vec3(rvecs.get(i)), vec3(tvecs.get(i)), err));
            }
            return out;
        } finally {
            camMat.release();
            dist.release();
            objPts.release();
            imgPts.release();
            reproj.release();
            rvec.release();
            tvec.release();
            for (Mat m : rvecs) m.release();
            for (Mat m : tvecs) m.release();
        }
    }

    private static double[] vec3(Mat m) {
        double[] o = new double[3];
        m.get(0, 0, o);
        return o;
    }
}
