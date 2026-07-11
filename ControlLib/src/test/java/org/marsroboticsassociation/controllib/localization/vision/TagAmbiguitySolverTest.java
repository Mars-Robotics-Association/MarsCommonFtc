package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Tests the pure {@link TagAmbiguitySolver} against a canned {@link PlanarPnpSolver} fake — proving
 * the OpenCV-extraction refactor works end-to-end with no native dependency: the solver prepares the
 * points, the (fake) seam returns solutions, and the solver selects best/alt and computes the
 * ambiguity ratio. On-robot the same solver is wired to the OpenCV-backed {@code
 * OpenCvPlanarPnpSolver} instead.
 */
class TagAmbiguitySolverTest {

    private static final double[] CAM = {800, 0, 640, 0, 800, 480, 0, 0, 1};
    private static final double TOL = 1e-9;

    /** A PnP fake that returns preset solutions, ignoring its inputs. */
    private static final class FakePnp implements PlanarPnpSolver {
        private final List<PnpSolution> solutions;

        FakePnp(List<PnpSolution> solutions) {
            this.solutions = solutions;
        }

        @Override
        public List<PnpSolution> solveIppeSquare(
                double[] objectPoints, double[] imagePoints, double[] cameraMatrix, double[] distCoeffs) {
            return solutions;
        }
    }

    private static List<List<Double>> fourCorners() {
        List<List<Double>> c = new ArrayList<>();
        c.add(Arrays.asList(10.0, 20.0));
        c.add(Arrays.asList(30.0, 20.0));
        c.add(Arrays.asList(30.0, 40.0));
        c.add(Arrays.asList(10.0, 40.0));
        return c;
    }

    @Test
    void selectsBestAndAltByReprojectionError() {
        double[] rBest = {0.1, 0.2, 0.3}, tBest = {1, 2, 3};
        double[] rAlt = {0.4, 0.5, 0.6}, tAlt = {4, 5, 6};
        // Supply worst-first to prove selection is by error, not input order.
        List<PlanarPnpSolver.PnpSolution> sols =
                Arrays.asList(
                        new PlanarPnpSolver.PnpSolution(rAlt, tAlt, 2.0),
                        new PlanarPnpSolver.PnpSolution(rBest, tBest, 0.5));
        TagAmbiguitySolver solver = new TagAmbiguitySolver(new FakePnp(sols), CAM, null, 0.1651);

        TagAmbiguitySolver.PnpSolutions s = solver.solve(fourCorners());
        assertNotNull(s);
        assertEquals(0.25, s.ratio, TOL); // 0.5 / 2.0
        assertEquals(0.5, s.reprojErrBest, TOL);
        assertArrayEquals(rBest, s.rvecBest, TOL);
        assertArrayEquals(tBest, s.tvecBest, TOL);
        assertArrayEquals(rAlt, s.rvecAlt, TOL);
        assertArrayEquals(tAlt, s.tvecAlt, TOL);
    }

    @Test
    void singleSolutionHasNullAltAndZeroAmbiguity() {
        List<PlanarPnpSolver.PnpSolution> sols =
                Arrays.asList(
                        new PlanarPnpSolver.PnpSolution(
                                new double[] {0, 0, 0.3}, new double[] {1, 2, 3}, 0.7));
        TagAmbiguitySolver solver = new TagAmbiguitySolver(new FakePnp(sols), CAM, null, 0.1651);

        TagAmbiguitySolver.PnpSolutions s = solver.solve(fourCorners());
        assertNotNull(s);
        assertEquals(0.0, s.ratio, TOL);
        assertNull(s.rvecAlt);
        assertNull(s.tvecAlt);
    }

    @Test
    void ambiguityConvenienceReturnsRatio() {
        List<PlanarPnpSolver.PnpSolution> sols =
                Arrays.asList(
                        new PlanarPnpSolver.PnpSolution(new double[] {0, 0, 0}, new double[] {0, 0, 1}, 0.5),
                        new PlanarPnpSolver.PnpSolution(new double[] {0, 0, 1}, new double[] {0, 0, 1}, 2.0));
        TagAmbiguitySolver solver = new TagAmbiguitySolver(new FakePnp(sols), CAM, null, 0.1651);
        assertEquals(0.25, solver.ambiguity(fourCorners()), TOL);
    }

    @Test
    void nullOnEmptySolutionsOrMalformedCorners() {
        TagAmbiguitySolver empty =
                new TagAmbiguitySolver(new FakePnp(new ArrayList<>()), CAM, null, 0.1651);
        assertNull(empty.solve(fourCorners()), "no solutions → null");

        // Malformed corners (wrong count) short-circuit before the PnP is even called.
        List<List<Double>> threeCorners = new ArrayList<>(fourCorners());
        threeCorners.remove(3);
        assertNull(empty.solve(threeCorners), "bad corners → null");
        assertNull(empty.ambiguity(null), "null corners → null");
    }
}
