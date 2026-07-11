package org.marsroboticsassociation.controllib.localization.vision;

import java.util.List;

/**
 * Pure (OpenCV-free) helpers for the AprilTag pose-ambiguity estimate. Kept separate from {@link
 * TagAmbiguitySolver}'s injected {@link PlanarPnpSolver} so this geometry/ratio math is unit-testable
 * on the desktop JVM — the OpenCV native library that {@code solvePnPGeneric} needs only loads on the
 * robot (Android), so anything that touches it can't run under a plain JVM unit test. Everything here
 * is plain arithmetic and is covered by {@code TagAmbiguityMathTest}.
 *
 * <p>Background: a single planar AprilTag has two pose solutions that reproject almost equally well
 * (the orientation "flip"). PhotonVision quantifies this as {@code ambiguity = bestReprojErr /
 * secondBestReprojErr} in [0,1]; near 0 means one solution is far better (trustworthy), near 1
 * means the two are indistinguishable (flip-prone, reject). Limelight computes this internally but
 * only exposes it over NetworkTables — unreachable on FTC — so we recover it by re-solving the PnP
 * ourselves from the tag's corner pixels.
 */
public final class TagAmbiguityMath {

    private TagAmbiguityMath() {}

    /**
     * The four corners of a tag of side {@code sizeM} (meters) in the tag's own frame, in the order
     * OpenCV's {@code SOLVEPNP_IPPE_SQUARE} requires: top-left, top-right, bottom-right,
     * bottom-left, with +x right, +y up, z=0 (tag lying in its own xy-plane). The image points
     * handed to the solver must correspond to these in the same order — see {@link
     * #flattenCorners}.
     */
    static double[][] squareObjectPoints(double sizeM) {
        double h = sizeM / 2.0;
        return new double[][] {
            {-h, h, 0.0}, // top-left
            {h, h, 0.0}, // top-right
            {h, -h, 0.0}, // bottom-right
            {-h, -h, 0.0} // bottom-left
        };
    }

    /**
     * Validate Limelight's corner list (must be exactly four {@code [x,y]} finite pixel pairs) and
     * flatten it to {@code [x0,y0,x1,y1,x2,y2,x3,y3]} in the given order. Returns null if the
     * corners are missing or malformed (e.g. the pipeline isn't emitting corners), which the caller
     * treats as "can't score this frame" rather than an error.
     *
     * <p>Note we deliberately do <i>not</i> reorder the corners to match {@code
     * SOLVEPNP_IPPE_SQUARE}'s expected order: a square tag's 4-fold symmetry makes the ambiguity
     * ratio invariant to corner ordering (any rotation is a valid relabeling; a winding flip merely
     * swaps the two solutions), so since we only consume the ratio, Limelight's order doesn't
     * matter. This is proven in {@code
     * TagAmbiguityMathTest.ambiguityIsInvariantToCornerOrdering}.
     */
    static double[] flattenCorners(List<List<Double>> corners) {
        if (corners == null || corners.size() != 4) {
            return null;
        }
        double[] out = new double[8];
        for (int i = 0; i < 4; i++) {
            List<Double> c = corners.get(i);
            if (c == null || c.size() < 2) {
                return null;
            }
            double x = c.get(0);
            double y = c.get(1);
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                return null;
            }
            out[2 * i] = x;
            out[2 * i + 1] = y;
        }
        return out;
    }

    /**
     * Whether a Limelight {@code camMatVector} (the row-major 3x3 intrinsics, {@code [fx,0,cx,
     * 0,fy,cy,0,0,1]}) is usable: at least nine elements, all finite, and positive focal lengths.
     * Guards against an invalid/empty calibration before we hand it to the PnP solve.
     */
    public static boolean isUsableCameraMatrix(double[] camMatVector) {
        if (camMatVector == null || camMatVector.length < 9) {
            return false;
        }
        for (double v : camMatVector) {
            if (!Double.isFinite(v)) {
                return false;
            }
        }
        return camMatVector[0] > 0 && camMatVector[4] > 0; // fx, fy
    }

    /**
     * PhotonVision-style ambiguity from a set of per-solution reprojection errors: {@code best /
     * secondBest}, clamped to [0,1]. Fewer than two finite solutions → 0 (a lone solution is
     * unambiguous); a second error of ~0 → 1 (two equally perfect fits, maximally ambiguous).
     */
    static double ambiguityRatio(double[] reprojErrors) {
        double best = Double.POSITIVE_INFINITY;
        double second = Double.POSITIVE_INFINITY;
        int count = 0;
        for (double e : reprojErrors) {
            if (!Double.isFinite(e) || e < 0) {
                continue;
            }
            count++;
            if (e < best) {
                second = best;
                best = e;
            } else if (e < second) {
                second = e;
            }
        }
        if (count < 2) {
            return 0.0;
        }
        if (second <= 0) {
            return 1.0;
        }
        double ratio = best / second;
        return ratio > 1.0 ? 1.0 : ratio;
    }
}
