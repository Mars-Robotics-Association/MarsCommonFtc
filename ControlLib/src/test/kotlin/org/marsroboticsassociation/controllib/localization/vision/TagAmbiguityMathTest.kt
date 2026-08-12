package org.marsroboticsassociation.controllib.localization.vision

import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Unit tests for the OpenCV-free ambiguity math. The actual `solvePnPGeneric` call (the injected
 * [PlanarPnpSolver]) needs the OpenCV native lib (Android-only) and is validated on-robot; this
 * pins everything around it: object-point geometry, corner validation/reordering, and the
 * reprojection-error ratio.
 */
class TagAmbiguityMathTest {

    companion object {
        private const val TOL = 1e-9

        private fun corners(vararg xy: Double): List<List<Double>> {
            val out = ArrayList<List<Double>>()
            var i = 0
            while (i < xy.size) {
                out.add(listOf(xy[i], xy[i + 1]))
                i += 2
            }
            return out
        }
    }

    // --- ambiguity ratio -----------------------------------------------------------------------

    @Test
    fun ratioIsBestOverSecondBest() {
        assertEquals(0.25, TagAmbiguityMath.ambiguityRatio(doubleArrayOf(0.5, 2.0)), TOL)
    }

    @Test
    fun ratioIgnoresOrderingOfInput() {
        // Must pick the two smallest regardless of input order.
        assertEquals(0.25, TagAmbiguityMath.ambiguityRatio(doubleArrayOf(2.0, 0.5)), TOL)
        assertEquals(0.1, TagAmbiguityMath.ambiguityRatio(doubleArrayOf(5.0, 0.1, 1.0, 9.0)), TOL)
    }

    @Test
    fun equalErrorsAreMaximallyAmbiguous() {
        assertEquals(1.0, TagAmbiguityMath.ambiguityRatio(doubleArrayOf(1.0, 1.0)), TOL)
    }

    @Test
    fun singleSolutionIsUnambiguous() {
        assertEquals(0.0, TagAmbiguityMath.ambiguityRatio(doubleArrayOf(0.7)), TOL)
        assertEquals(0.0, TagAmbiguityMath.ambiguityRatio(doubleArrayOf()), TOL)
    }

    @Test
    fun nonFiniteAndNegativeErrorsAreSkipped() {
        // Only 0.4 is a valid second solution; NaN/inf/negative are dropped, leaving <2 -> 0.
        assertEquals(
            0.0,
            TagAmbiguityMath.ambiguityRatio(doubleArrayOf(0.4, Double.NaN, -1.0)),
            TOL,
        )
        // 0.2 and 0.8 survive -> 0.25.
        assertEquals(
            0.25,
            TagAmbiguityMath.ambiguityRatio(
                doubleArrayOf(0.8, Double.POSITIVE_INFINITY, 0.2, -3.0)
            ),
            TOL,
        )
    }

    @Test
    fun secondBestZeroClampsToOne() {
        assertEquals(1.0, TagAmbiguityMath.ambiguityRatio(doubleArrayOf(0.0, 0.0)), TOL)
    }

    // --- object points -------------------------------------------------------------------------

    @Test
    fun objectPointsAreCenteredSquareInIppeOrder() {
        val o = TagAmbiguityMath.squareObjectPoints(0.2)
        assertArrayEquals(doubleArrayOf(-0.1, 0.1, 0.0), o[0], TOL) // top-left
        assertArrayEquals(doubleArrayOf(0.1, 0.1, 0.0), o[1], TOL) // top-right
        assertArrayEquals(doubleArrayOf(0.1, -0.1, 0.0), o[2], TOL) // bottom-right
        assertArrayEquals(doubleArrayOf(-0.1, -0.1, 0.0), o[3], TOL) // bottom-left
    }

    // --- camera-matrix validation --------------------------------------------------------------

    @Test
    fun acceptsAWellFormedCameraMatrix() {
        assertTrue(
            TagAmbiguityMath.isUsableCameraMatrix(
                doubleArrayOf(800.0, 0.0, 640.0, 0.0, 800.0, 480.0, 0.0, 0.0, 1.0)
            )
        )
    }

    @Test
    fun rejectsBadCameraMatrices() {
        assertFalse(TagAmbiguityMath.isUsableCameraMatrix(null))
        assertFalse(
            TagAmbiguityMath.isUsableCameraMatrix(doubleArrayOf(800.0, 0.0, 640.0))
        ) // too short
        // Non-positive focal lengths (e.g. an empty/zeroed calibration).
        assertFalse(
            TagAmbiguityMath.isUsableCameraMatrix(
                doubleArrayOf(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            )
        )
        // Non-finite entry.
        assertFalse(
            TagAmbiguityMath.isUsableCameraMatrix(
                doubleArrayOf(800.0, 0.0, 640.0, 0.0, Double.NaN, 480.0, 0.0, 0.0, 1.0)
            )
        )
    }

    // --- corner validation / flattening --------------------------------------------------------

    @Test
    fun flattensFourCornersInOrder() {
        val c = corners(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0)
        val out = TagAmbiguityMath.flattenCorners(c)
        assertArrayEquals(doubleArrayOf(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0), out, TOL)
    }

    @Test
    fun rejectsWrongCount() {
        assertNull(TagAmbiguityMath.flattenCorners(corners(1.0, 2.0, 3.0, 4.0))) // 2 pts
        assertNull(TagAmbiguityMath.flattenCorners(null))
    }

    @Test
    fun rejectsMalformedOrNonFinitePoints() {
        val shortPt = ArrayList(corners(10.0, 20.0, 30.0, 40.0, 50.0, 60.0))
        shortPt.add(listOf(70.0)) // only one coord
        assertNull(TagAmbiguityMath.flattenCorners(shortPt))

        val nan = corners(10.0, 20.0, 30.0, 40.0, 50.0, 60.0, Double.NaN, 80.0)
        assertNull(TagAmbiguityMath.flattenCorners(nan))
    }
}
