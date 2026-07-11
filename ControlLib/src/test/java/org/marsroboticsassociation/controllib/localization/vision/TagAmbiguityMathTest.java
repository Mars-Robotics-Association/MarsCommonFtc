package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Unit tests for the OpenCV-free ambiguity math. The actual {@code solvePnPGeneric} call (the
 * injected {@link PlanarPnpSolver}) needs the OpenCV native lib (Android-only) and is validated
 * on-robot; this pins everything around it: object-point geometry, corner validation/reordering, and
 * the reprojection-error ratio.
 */
class TagAmbiguityMathTest {

    private static final double TOL = 1e-9;

    // --- ambiguity ratio -----------------------------------------------------------------------

    @Test
    void ratioIsBestOverSecondBest() {
        assertEquals(0.25, TagAmbiguityMath.ambiguityRatio(new double[] {0.5, 2.0}), TOL);
    }

    @Test
    void ratioIgnoresOrderingOfInput() {
        // Must pick the two smallest regardless of input order.
        assertEquals(0.25, TagAmbiguityMath.ambiguityRatio(new double[] {2.0, 0.5}), TOL);
        assertEquals(0.1, TagAmbiguityMath.ambiguityRatio(new double[] {5.0, 0.1, 1.0, 9.0}), TOL);
    }

    @Test
    void equalErrorsAreMaximallyAmbiguous() {
        assertEquals(1.0, TagAmbiguityMath.ambiguityRatio(new double[] {1.0, 1.0}), TOL);
    }

    @Test
    void singleSolutionIsUnambiguous() {
        assertEquals(0.0, TagAmbiguityMath.ambiguityRatio(new double[] {0.7}), TOL);
        assertEquals(0.0, TagAmbiguityMath.ambiguityRatio(new double[] {}), TOL);
    }

    @Test
    void nonFiniteAndNegativeErrorsAreSkipped() {
        // Only 0.4 is a valid second solution; NaN/inf/negative are dropped, leaving <2 -> 0.
        assertEquals(0.0, TagAmbiguityMath.ambiguityRatio(new double[] {0.4, Double.NaN, -1.0}), TOL);
        // 0.2 and 0.8 survive -> 0.25.
        assertEquals(
                0.25,
                TagAmbiguityMath.ambiguityRatio(
                        new double[] {0.8, Double.POSITIVE_INFINITY, 0.2, -3.0}),
                TOL);
    }

    @Test
    void secondBestZeroClampsToOne() {
        assertEquals(1.0, TagAmbiguityMath.ambiguityRatio(new double[] {0.0, 0.0}), TOL);
    }

    // --- object points -------------------------------------------------------------------------

    @Test
    void objectPointsAreCenteredSquareInIppeOrder() {
        double[][] o = TagAmbiguityMath.squareObjectPoints(0.2);
        assertArrayEquals(new double[] {-0.1, 0.1, 0.0}, o[0], TOL); // top-left
        assertArrayEquals(new double[] {0.1, 0.1, 0.0}, o[1], TOL); // top-right
        assertArrayEquals(new double[] {0.1, -0.1, 0.0}, o[2], TOL); // bottom-right
        assertArrayEquals(new double[] {-0.1, -0.1, 0.0}, o[3], TOL); // bottom-left
    }

    // --- camera-matrix validation --------------------------------------------------------------

    @Test
    void acceptsAWellFormedCameraMatrix() {
        assertTrue(
                TagAmbiguityMath.isUsableCameraMatrix(
                        new double[] {800, 0, 640, 0, 800, 480, 0, 0, 1}));
    }

    @Test
    void rejectsBadCameraMatrices() {
        assertFalse(TagAmbiguityMath.isUsableCameraMatrix(null));
        assertFalse(TagAmbiguityMath.isUsableCameraMatrix(new double[] {800, 0, 640})); // too short
        // Non-positive focal lengths (e.g. an empty/zeroed calibration).
        assertFalse(TagAmbiguityMath.isUsableCameraMatrix(new double[] {0, 0, 0, 0, 0, 0, 0, 0, 0}));
        // Non-finite entry.
        assertFalse(
                TagAmbiguityMath.isUsableCameraMatrix(
                        new double[] {800, 0, 640, 0, Double.NaN, 480, 0, 0, 1}));
    }

    // --- corner validation / flattening --------------------------------------------------------

    private static List<List<Double>> corners(double... xy) {
        List<List<Double>> out = new ArrayList<>();
        for (int i = 0; i < xy.length; i += 2) {
            out.add(Arrays.asList(xy[i], xy[i + 1]));
        }
        return out;
    }

    @Test
    void flattensFourCornersInOrder() {
        List<List<Double>> c = corners(10, 20, 30, 40, 50, 60, 70, 80);
        double[] out = TagAmbiguityMath.flattenCorners(c);
        assertArrayEquals(new double[] {10, 20, 30, 40, 50, 60, 70, 80}, out, TOL);
    }

    @Test
    void rejectsWrongCount() {
        assertNull(TagAmbiguityMath.flattenCorners(corners(1, 2, 3, 4))); // 2 pts
        assertNull(TagAmbiguityMath.flattenCorners(null));
    }

    @Test
    void rejectsMalformedOrNonFinitePoints() {
        List<List<Double>> shortPt = new ArrayList<>(corners(10, 20, 30, 40, 50, 60));
        shortPt.add(Arrays.asList(70.0)); // only one coord
        assertNull(TagAmbiguityMath.flattenCorners(shortPt));

        List<List<Double>> nan = corners(10, 20, 30, 40, 50, 60, Double.NaN, 80);
        assertNull(TagAmbiguityMath.flattenCorners(nan));
    }
}
