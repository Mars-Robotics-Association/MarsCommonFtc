package org.marsroboticsassociation.controllib.localization.vision;

import java.util.List;

/**
 * The single OpenCV-shaped operation the vision pipeline needs, expressed as a pure Java seam so the
 * rest of the localizer stays OpenCV-free and desktop-testable.
 *
 * <p>A planar AprilTag has two poses that reproject almost equally well (the orientation "flip").
 * Recovering both — and their reprojection errors — is exactly what OpenCV's {@code
 * solvePnPGeneric(..., SOLVEPNP_IPPE_SQUARE, ...)} does, and it is the <i>only</i> place the
 * localizer genuinely needs OpenCV. This interface isolates that one call: {@link TagAmbiguitySolver}
 * (pure) prepares the point arrays and interprets the results, while an implementation supplies the
 * actual solve.
 *
 * <p>On-robot, the quickstart provides an OpenCV-backed implementation. {@code SOLVEPNP_IPPE_SQUARE}
 * (the planar-square two-solution solver) is not trivially reimplementable in pure Java, so a real
 * implementation still needs OpenCV (or an equivalent IPPE routine) — but the library only ever sees
 * this interface, and unit tests can supply a canned or analytic stand-in.
 */
public interface PlanarPnpSolver {

    /**
     * Solve the planar (IPPE-square) PnP for one tag, returning every solution the solver produced
     * (typically the two flip candidates) with each one's reprojection error. Order is not
     * significant — the caller selects best/second by {@link PnpSolution#reprojErr}.
     *
     * @param objectPoints the four tag corners in the tag frame, flattened row-major as {@code
     *     [x0,y0,z0, x1,y1,z1, x2,y2,z2, x3,y3,z3]} (length 12), in the order {@link
     *     TagAmbiguityMath#squareObjectPoints} produces
     * @param imagePoints the four corresponding corner pixels, flattened as {@code
     *     [x0,y0, x1,y1, x2,y2, x3,y3]} (length 8), same corner order as {@code objectPoints}
     * @param cameraMatrix row-major 3x3 intrinsics {@code [fx,0,cx, 0,fy,cy, 0,0,1]} (length 9)
     * @param distCoeffs distortion coefficients (OpenCV order {@code [k1,k2,p1,p2,k3,...]}); null or
     *     empty is treated as zero distortion
     * @return the solutions (never null; empty when the solve fails)
     */
    List<PnpSolution> solveIppeSquare(
            double[] objectPoints, double[] imagePoints, double[] cameraMatrix, double[] distCoeffs);

    /**
     * One PnP solution: a length-3 Rodrigues rotation vector and length-3 translation (metres, as the
     * object points are in metres) giving the tag-in-camera pose, plus that solution's reprojection
     * error in pixels. Feed {@code rvec}/{@code tvec} to {@link Transform3D#fromRodrigues} to build a
     * camera-relative candidate for {@link VisionPoseSolver}.
     */
    final class PnpSolution {
        public final double[] rvec;
        public final double[] tvec;
        public final double reprojErr;

        public PnpSolution(double[] rvec, double[] tvec, double reprojErr) {
            this.rvec = rvec;
            this.tvec = tvec;
            this.reprojErr = reprojErr;
        }
    }
}
