package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.localization.vision.replay.OpenCvPnpSolver;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * End-to-end DIY-MegaTag2 pipeline test through the <i>real</i> OpenCV solver, run on the desktop
 * JVM against {@code org.openpnp:opencv} (see ControlLib/build.gradle). Where {@link
 * VisionPoseSolverTest} feeds the solver synthetic {@code cameraFromTag} candidates, this drives the
 * whole chain:
 *
 * <pre>{@code
 * known world → project tag corners → TagAmbiguitySolver.solve() (real solvePnPGeneric)
 *             → Transform3D.fromRodrigues → VisionPoseSolver.solve() → recovered robot pose
 * }</pre>
 *
 * so it exercises the actual PnP, the actual corner ordering, and {@link Transform3D#fromRodrigues}
 * against OpenCV's real {@code rvec} output. (Clean synthetic corners only: this proves the
 * code/math path, not robustness to the noisy detections that cause real on-robot flips.)
 */
class VisionPosePipelineTest {

    private static final double FX = 800, FY = 800, CX = 640, CY = 480;
    private static final double[] CAM_MAT = {FX, 0, CX, 0, FY, CY, 0, 0, 1};
    private static final double[] NO_DIST = new double[0];
    private static final double TAG = 0.1651; // 165.1 mm — the DECODE field tag
    private static final int TAG_ID = 11;

    @BeforeAll
    static void loadNative() {
        nu.pattern.OpenCV.loadLocally();
    }

    /**
     * Full round trip: a self-consistent synthetic world (metres) with the tag visibly in front of
     * the camera and well-conditioned (25° oblique). We derive {@code fieldFromTag} from the world,
     * project the tag's corners, run the real solve, and assert the pipeline recovers the known
     * robot pose. With clean corners the recovery is essentially exact.
     */
    @Test
    void recoversKnownRobotPoseThroughRealSolve() {
        // Pose the tag in the camera optical frame: 1.5 m ahead, slightly off-centre, 25° about y.
        Transform3D cameraFromTag =
                new Transform3D(rotY(Math.toRadians(25)), new double[] {0.1, -0.05, 1.5});
        // Extrinsic: optical→robot axis swap (camera looks along robot +x), 0.1 m fwd, 0.2 m up.
        Transform3D robotFromCamera =
                new Transform3D(
                        new double[][] {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}},
                        new double[] {0.1, 0.0, 0.2});
        // Known robot pose to recover: field (2.0, 1.0) m, heading 0.3 rad, level.
        Transform3D fieldFromRobot = Transform3D.fromTranslationYPR(2.0, 1.0, 0, 0.3, 0, 0);
        // Derive the tag's field pose from the chain so the world is self-consistent.
        Transform3D fieldFromTag = fieldFromRobot.multiply(robotFromCamera).multiply(cameraFromTag);

        Map<Integer, Transform3D> table = new HashMap<>();
        table.put(TAG_ID, fieldFromTag);
        VisionPoseSolver solver = new VisionPoseSolver(robotFromCamera, table);

        TagAmbiguitySolver est = new TagAmbiguitySolver(new OpenCvPnpSolver(), CAM_MAT, NO_DIST, TAG);
        TagAmbiguitySolver.PnpSolutions s = est.solve(project(cameraFromTag));
        assertNotNull(s, "real solve returned null");

        Transform3D[] candidates = candidatesFrom(s);
        VisionPoseSolver.Result r = solver.solve(TAG_ID, candidates, fieldFromRobot.yaw());
        assertNotNull(r);

        // Position recovered from the chosen candidate; heading is the fed value (exact).
        assertEquals(2.0, r.fieldPose.getX(), 2e-3);
        assertEquals(1.0, r.fieldPose.getY(), 2e-3);
        assertEquals(0.3, r.fieldPose.getRotation().getRadians(), 1e-12);
        // The chosen candidate's own solved yaw agrees with truth → the right candidate won and the
        // corner order / fromRodrigues / chain are all consistent end to end.
        assertTrue(
                r.yawResidualRad < 0.02,
                "solved yaw should agree with truth: residual " + r.yawResidualRad);
    }

    /**
     * Pins the pure {@link Transform3D#fromRodrigues} to OpenCV's {@code Calib3d.Rodrigues} across
     * several rotation vectors — what lets {@code fromRodrigues} stay in the OpenCV-free tier without
     * risking divergence from the real thing.
     */
    @Test
    void fromRodriguesMatchesOpenCvRodrigues() {
        double[][] rvecs = {
            {0, 0, 0.42}, {0.3, -0.7, 1.1}, {1.5, 0, 0}, {-0.2, 0.9, -0.4}, {0, 0, 0}
        };
        for (double[] rvec : rvecs) {
            Mat rv = new Mat(3, 1, CvType.CV_64F);
            rv.put(0, 0, rvec);
            Mat rMat = new Mat();
            try {
                Calib3d.Rodrigues(rv, rMat);
                Transform3D t = Transform3D.fromRodrigues(rvec, new double[] {0, 0, 0});
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        assertEquals(
                                rMat.get(i, j)[0],
                                t.r[i][j],
                                1e-12,
                                "rvec=" + Arrays.toString(rvec) + " [" + i + "][" + j + "]");
                    }
                }
            } finally {
                rv.release();
                rMat.release();
            }
        }
    }

    // --- helpers ------------------------------------------------------------------------------

    /** PnpSolutions → 1–2 {@code cameraFromTag} candidates. */
    private static Transform3D[] candidatesFrom(TagAmbiguitySolver.PnpSolutions s) {
        Transform3D best = Transform3D.fromRodrigues(s.rvecBest, s.tvecBest);
        if (s.rvecAlt == null) {
            return new Transform3D[] {best};
        }
        return new Transform3D[] {best, Transform3D.fromRodrigues(s.rvecAlt, s.tvecAlt)};
    }

    /** Projects the tag's four corners (squareObjectPoints order) through a clean pinhole model. */
    private static List<List<Double>> project(Transform3D cameraFromTag) {
        double h = TAG / 2.0;
        double[][] obj = {{-h, h, 0}, {h, h, 0}, {h, -h, 0}, {-h, -h, 0}}; // TL, TR, BR, BL
        List<List<Double>> out = new ArrayList<>();
        for (double[] o : obj) {
            double[] cam = matVec(cameraFromTag.r, o);
            cam[0] += cameraFromTag.t[0];
            cam[1] += cameraFromTag.t[1];
            cam[2] += cameraFromTag.t[2];
            out.add(Arrays.asList(FX * cam[0] / cam[2] + CX, FY * cam[1] / cam[2] + CY));
        }
        return out;
    }

    private static double[][] rotY(double theta) {
        double c = Math.cos(theta), s = Math.sin(theta);
        return new double[][] {{c, 0, s}, {0, 1, 0}, {-s, 0, c}};
    }

    private static double[] matVec(double[][] m, double[] v) {
        return new double[] {
            m[0][0] * v[0] + m[0][1] * v[1] + m[0][2] * v[2],
            m[1][0] * v[0] + m[1][1] * v[1] + m[1][2] * v[2],
            m[2][0] * v[0] + m[2][1] * v[1] + m[2][2] * v[2]
        };
    }
}
