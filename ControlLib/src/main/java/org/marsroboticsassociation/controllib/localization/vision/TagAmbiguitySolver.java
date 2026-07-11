package org.marsroboticsassociation.controllib.localization.vision;

import java.util.List;

/**
 * Computes the PnP pose-ambiguity of a single AprilTag from its corner pixels, recovering the
 * metric Limelight calculates internally but only exposes over NetworkTables (unreachable on FTC).
 * We re-solve the planar pose with the injected {@link PlanarPnpSolver} (OpenCV's {@code
 * SOLVEPNP_IPPE_SQUARE} on-robot), which returns the <i>two</i> solutions of the orientation
 * ambiguity along with their reprojection errors, and report {@code best / secondBest} (see {@link
 * TagAmbiguityMath#ambiguityRatio}).
 *
 * <p>This class is pure: it prepares the object/image point arrays, delegates the single native call
 * to {@link PlanarPnpSolver}, and interprets the result. That keeps the whole ambiguity + pose
 * pipeline desktop-testable — only the {@link PlanarPnpSolver} implementation needs OpenCV.
 *
 * <h3>Requirements</h3>
 *
 * <ul>
 *   <li><b>Corner output:</b> the Limelight pipeline must be emitting corner points ("send/output
 *       corners"), or the corner list is empty and {@link #ambiguity} returns null.
 *   <li><b>Intrinsics:</b> the camera matrix + distortion, taken straight from the Limelight's own
 *       calibration — the real per-device values at the active resolution, so no manual calibration
 *       entry is needed.
 * </ul>
 *
 * <p><b>Corner order doesn't matter.</b> A square tag's 4-fold symmetry makes the ambiguity ratio
 * invariant to the order Limelight reports its corners in, so the corners are fed straight through.
 * (This holds only because we consume the ratio, not the pose; the recovered <i>pose</i> would depend
 * on order.)
 */
public class TagAmbiguitySolver {

    private final PlanarPnpSolver pnp;
    private final double[] objectPoints; // flattened 4x3 tag corners in tag frame
    private final double[] cameraMatrix; // row-major 3x3
    private final double[] distCoeffs;

    /**
     * @param pnp the planar PnP seam (OpenCV-backed on-robot)
     * @param camMatVector the Limelight's row-major 3x3 intrinsics ({@code [fx,0,cx,0,fy,cy,0,0,1]});
     *     must be usable per {@link TagAmbiguityMath#isUsableCameraMatrix}
     * @param distortion the Limelight's distortion coefficients; null/empty is treated as zero
     *     distortion
     * @param tagSizeMeters AprilTag black-square side length
     */
    public TagAmbiguitySolver(
            PlanarPnpSolver pnp, double[] camMatVector, double[] distortion, double tagSizeMeters) {
        this.pnp = pnp;
        this.cameraMatrix = camMatVector.clone();
        this.distCoeffs =
                (distortion != null && distortion.length > 0)
                        ? distortion.clone()
                        : new double[] {0, 0, 0, 0, 0};

        double[][] o = TagAmbiguityMath.squareObjectPoints(tagSizeMeters);
        objectPoints = new double[12];
        for (int i = 0; i < 4; i++) {
            objectPoints[3 * i] = o[i][0];
            objectPoints[3 * i + 1] = o[i][1];
            objectPoints[3 * i + 2] = o[i][2];
        }
    }

    /**
     * Both IPPE solutions for one tag — the {@code ratio} (see {@link #ambiguity}) plus the two
     * tag-in-camera poses ordered best-first by reprojection error. {@code rvecAlt}/{@code tvecAlt}
     * are null when only one solution exists. Each {@code rvec}/{@code tvec} is a length-3 Rodrigues
     * / translation vector, ready for {@link Transform3D#fromRodrigues}.
     */
    public static final class PnpSolutions {
        public final double ratio;

        /**
         * Reprojection error (pixels) of the best solution — how well the detection fits a rigid
         * square tag, a prior-free detection-quality signal (rises with blur / occlusion / far /
         * oblique). The ambiguity {@link #ratio} is err_best/err_alt; this is the absolute err_best.
         */
        public final double reprojErrBest;

        public final double[] rvecBest;
        public final double[] tvecBest;
        public final double[] rvecAlt; // null if a single solution
        public final double[] tvecAlt;

        PnpSolutions(
                double ratio,
                double reprojErrBest,
                double[] rvecBest,
                double[] tvecBest,
                double[] rvecAlt,
                double[] tvecAlt) {
            this.ratio = ratio;
            this.reprojErrBest = reprojErrBest;
            this.rvecBest = rvecBest;
            this.tvecBest = tvecBest;
            this.rvecAlt = rvecAlt;
            this.tvecAlt = tvecAlt;
        }
    }

    /**
     * Ambiguity in [0,1] for one tag's corners (null if corners are absent/malformed or the solve
     * fails). Near 0 = one pose clearly wins (trust it); near 1 = the flip is indistinguishable
     * (reject). Delegates to {@link #solve}.
     */
    public Double ambiguity(List<List<Double>> corners) {
        PnpSolutions s = solve(corners);
        return s == null ? null : s.ratio;
    }

    /**
     * Solve a tag's planar pose, returning both IPPE solutions (null if corners are
     * absent/malformed or the solve fails).
     */
    public PnpSolutions solve(List<List<Double>> corners) {
        double[] img = TagAmbiguityMath.flattenCorners(corners);
        if (img == null) {
            return null;
        }

        List<PlanarPnpSolver.PnpSolution> sols =
                pnp.solveIppeSquare(objectPoints, img, cameraMatrix, distCoeffs);
        if (sols == null || sols.isEmpty()) {
            return null;
        }

        int total = sols.size();
        double[] errs = new double[total];
        for (int i = 0; i < total; i++) {
            errs[i] = sols.get(i).reprojErr;
        }
        // Index of the lowest-reproj (best) solution and the next-lowest (the flip).
        int bi = 0;
        for (int i = 1; i < total; i++) {
            if (errs[i] < errs[bi]) {
                bi = i;
            }
        }
        int ai = -1;
        for (int i = 0; i < total; i++) {
            if (i != bi && (ai < 0 || errs[i] < errs[ai])) {
                ai = i;
            }
        }
        return new PnpSolutions(
                TagAmbiguityMath.ambiguityRatio(errs),
                errs[bi], // absolute reprojection error of the best solution (px)
                sols.get(bi).rvec,
                sols.get(bi).tvec,
                ai >= 0 ? sols.get(ai).rvec : null,
                ai >= 0 ? sols.get(ai).tvec : null);
    }
}
