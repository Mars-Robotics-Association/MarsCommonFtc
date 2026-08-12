package org.marsroboticsassociation.controllib.localization.vision

/**
 * Pure (OpenCV-free) helpers for the AprilTag pose-ambiguity estimate. Kept separate from
 * [TagAmbiguitySolver]'s injected [PlanarPnpSolver] so this geometry/ratio math is unit-testable on
 * the desktop JVM — the OpenCV native library that `solvePnPGeneric` needs only loads on the robot
 * (Android), so anything that touches it can't run under a plain JVM unit test. Everything here is
 * plain arithmetic and is covered by `TagAmbiguityMathTest`.
 *
 * <p>Background: a single planar AprilTag has two pose solutions that reproject almost equally well
 * (the orientation "flip"). PhotonVision quantifies this as `ambiguity = bestReprojErr /
 * secondBestReprojErr` in [0,1]; near 0 means one solution is far better (trustworthy), near 1
 * means the two are indistinguishable (flip-prone, reject). Limelight computes this internally but
 * only exposes it over NetworkTables — unreachable on FTC — so we recover it by re-solving the PnP
 * ourselves from the tag's corner pixels.
 */
object TagAmbiguityMath {

    /**
     * The four corners of a tag of side `sizeM` (meters) in the tag's own frame, in the order
     * OpenCV's `SOLVEPNP_IPPE_SQUARE` requires: top-left, top-right, bottom-right, bottom-left,
     * with +x right, +y up, z=0 (tag lying in its own xy-plane). The image points handed to the
     * solver must correspond to these in the same order — see [flattenCorners].
     */
    @JvmStatic
    fun squareObjectPoints(sizeM: Double): Array<DoubleArray> {
        val h = sizeM / 2.0
        return arrayOf(
            doubleArrayOf(-h, h, 0.0), // top-left
            doubleArrayOf(h, h, 0.0), // top-right
            doubleArrayOf(h, -h, 0.0), // bottom-right
            doubleArrayOf(-h, -h, 0.0), // bottom-left
        )
    }

    /**
     * Validate Limelight's corner list (must be exactly four `[x,y]` finite pixel pairs) and
     * flatten it to `[x0,y0,x1,y1,x2,y2,x3,y3]` in the given order. Returns null if the corners are
     * missing or malformed (e.g. the pipeline isn't emitting corners), which the caller treats as
     * "can't score this frame" rather than an error.
     *
     * <p>Note we deliberately do *not* reorder the corners to match `SOLVEPNP_IPPE_SQUARE`'s
     * expected order: a square tag's 4-fold symmetry makes the ambiguity ratio invariant to corner
     * ordering (any rotation is a valid relabeling; a winding flip merely swaps the two solutions),
     * so since we only consume the ratio, Limelight's order doesn't matter. This is proven in
     * `TagAmbiguityMathTest.ambiguityIsInvariantToCornerOrdering`.
     */
    @JvmStatic
    fun flattenCorners(corners: List<List<Double>>?): DoubleArray? {
        if (corners == null || corners.size != 4) {
            return null
        }
        val out = DoubleArray(8)
        for (i in 0 until 4) {
            val c = corners[i]
            if (c == null || c.size < 2) {
                return null
            }
            val x = c[0]
            val y = c[1]
            if (!x.isFinite() || !y.isFinite()) {
                return null
            }
            out[2 * i] = x
            out[2 * i + 1] = y
        }
        return out
    }

    /**
     * Whether a Limelight `camMatVector` (the row-major 3x3 intrinsics, `[fx,0,cx, 0,fy,cy,0,0,1]`)
     * is usable: at least nine elements, all finite, and positive focal lengths. Guards against an
     * invalid/empty calibration before we hand it to the PnP solve.
     */
    @JvmStatic
    fun isUsableCameraMatrix(camMatVector: DoubleArray?): Boolean {
        if (camMatVector == null || camMatVector.size < 9) {
            return false
        }
        for (v in camMatVector) {
            if (!v.isFinite()) {
                return false
            }
        }
        return camMatVector[0] > 0 && camMatVector[4] > 0 // fx, fy
    }

    /**
     * PhotonVision-style ambiguity from a set of per-solution reprojection errors: `best /
     * secondBest`, clamped to [0,1]. Fewer than two finite solutions → 0 (a lone solution is
     * unambiguous); a second error of ~0 → 1 (two equally perfect fits, maximally ambiguous).
     */
    @JvmStatic
    fun ambiguityRatio(reprojErrors: DoubleArray): Double {
        var best = Double.POSITIVE_INFINITY
        var second = Double.POSITIVE_INFINITY
        var count = 0
        for (e in reprojErrors) {
            if (!e.isFinite() || e < 0) {
                continue
            }
            count++
            if (e < best) {
                second = best
                best = e
            } else if (e < second) {
                second = e
            }
        }
        if (count < 2) {
            return 0.0
        }
        if (second <= 0) {
            return 1.0
        }
        val ratio = best / second
        return if (ratio > 1.0) 1.0 else ratio
    }
}
