package org.marsroboticsassociation.controllib.localization.vision

/**
 * Computes the PnP pose-ambiguity of a single AprilTag from its corner pixels, recovering the
 * metric Limelight calculates internally but only exposes over NetworkTables (unreachable on FTC).
 * We re-solve the planar pose with the injected [PlanarPnpSolver] (OpenCV's `SOLVEPNP_IPPE_SQUARE`
 * on-robot), which returns the *two* solutions of the orientation ambiguity along with their
 * reprojection errors, and report `best / secondBest` (see [TagAmbiguityMath.ambiguityRatio]).
 *
 * <p>This class is pure: it prepares the object/image point arrays, delegates the single native
 * call to [PlanarPnpSolver], and interprets the result. That keeps the whole ambiguity + pose
 * pipeline desktop-testable — only the [PlanarPnpSolver] implementation needs OpenCV.
 *
 * <h3>Requirements</h3>
 * <ul>
 * <li><b>Corner output:</b> the Limelight pipeline must be emitting corner points ("send/output
 *   corners"), or the corner list is empty and [ambiguity] returns null.
 * <li><b>Intrinsics:</b> the camera matrix + distortion, taken straight from the Limelight's own
 *   calibration — the real per-device values at the active resolution, so no manual calibration
 *   entry is needed.
 * </ul>
 *
 * <p><b>Corner order doesn't matter.</b> A square tag's 4-fold symmetry makes the ambiguity ratio
 * invariant to the order Limelight reports its corners in, so the corners are fed straight through.
 * (This holds only because we consume the ratio, not the pose; the recovered *pose* would depend on
 * order.)
 */
class TagAmbiguitySolver(
    pnp: PlanarPnpSolver,
    camMatVector: DoubleArray,
    distortion: DoubleArray?,
    tagSizeMeters: Double,
) {
    private val pnp: PlanarPnpSolver = pnp
    private val objectPoints: DoubleArray // flattened 4x3 tag corners in tag frame
    private val cameraMatrix: DoubleArray // row-major 3x3
    private val distCoeffs: DoubleArray

    init {
        this.cameraMatrix = camMatVector.clone()
        this.distCoeffs =
            if (distortion != null && distortion.isNotEmpty()) {
                distortion.clone()
            } else {
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0)
            }

        val o = TagAmbiguityMath.squareObjectPoints(tagSizeMeters)
        objectPoints = DoubleArray(12)
        for (i in 0 until 4) {
            objectPoints[3 * i] = o[i][0]
            objectPoints[3 * i + 1] = o[i][1]
            objectPoints[3 * i + 2] = o[i][2]
        }
    }

    /**
     * Both IPPE solutions for one tag — the `ratio` (see [ambiguity]) plus the two tag-in-camera
     * poses ordered best-first by reprojection error. `rvecAlt`/`tvecAlt` are null when only one
     * solution exists. Each `rvec`/`tvec` is a length-3 Rodrigues / translation vector, ready for
     * [Transform3D.fromRodrigues].
     */
    class PnpSolutions
    internal constructor(
        @JvmField val ratio: Double,
        /**
         * Reprojection error (pixels) of the best solution — how well the detection fits a rigid
         * square tag, a prior-free detection-quality signal (rises with blur / occlusion / far /
         * oblique). The ambiguity [ratio] is err_best/err_alt; this is the absolute err_best.
         */
        @JvmField val reprojErrBest: Double,
        @JvmField val rvecBest: DoubleArray,
        @JvmField val tvecBest: DoubleArray,
        @JvmField val rvecAlt: DoubleArray?, // null if a single solution
        @JvmField val tvecAlt: DoubleArray?,
    )

    /**
     * Ambiguity in [0,1] for one tag's corners (null if corners are absent/malformed or the solve
     * fails). Near 0 = one pose clearly wins (trust it); near 1 = the flip is indistinguishable
     * (reject). Delegates to [solve].
     */
    fun ambiguity(corners: List<List<Double>>?): Double? {
        val s = solve(corners)
        return s?.ratio
    }

    /**
     * Solve a tag's planar pose, returning both IPPE solutions (null if corners are
     * absent/malformed or the solve fails).
     */
    fun solve(corners: List<List<Double>>?): PnpSolutions? {
        val img = TagAmbiguityMath.flattenCorners(corners) ?: return null

        val sols = pnp.solveIppeSquare(objectPoints, img, cameraMatrix, distCoeffs)
        if (sols.isEmpty()) {
            return null
        }

        val total = sols.size
        val errs = DoubleArray(total)
        for (i in 0 until total) {
            errs[i] = sols[i].reprojErr
        }
        // Index of the lowest-reproj (best) solution and the next-lowest (the flip).
        var bi = 0
        for (i in 1 until total) {
            if (errs[i] < errs[bi]) {
                bi = i
            }
        }
        var ai = -1
        for (i in 0 until total) {
            if (i != bi && (ai < 0 || errs[i] < errs[ai])) {
                ai = i
            }
        }
        return PnpSolutions(
            TagAmbiguityMath.ambiguityRatio(errs),
            errs[bi], // absolute reprojection error of the best solution (px)
            sols[bi].rvec,
            sols[bi].tvec,
            if (ai >= 0) sols[ai].rvec else null,
            if (ai >= 0) sols[ai].tvec else null,
        )
    }
}
