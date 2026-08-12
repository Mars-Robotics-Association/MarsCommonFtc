package org.marsroboticsassociation.controllib.localization.vision

import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Test

/**
 * Tests the pure [TagAmbiguitySolver] against a canned [PlanarPnpSolver] fake, so the full path
 * runs with no native dependency: the solver prepares the points, the (fake) seam returns
 * solutions, and the solver selects best/alt and computes the ambiguity ratio. On-robot the same
 * solver is wired to the OpenCV-backed `OpenCvPlanarPnpSolver`.
 */
class TagAmbiguitySolverTest {

    companion object {
        private val CAM = doubleArrayOf(800.0, 0.0, 640.0, 0.0, 800.0, 480.0, 0.0, 0.0, 1.0)
        private const val TOL = 1e-9

        private fun fourCorners(): List<List<Double>> {
            return listOf(
                listOf(10.0, 20.0),
                listOf(30.0, 20.0),
                listOf(30.0, 40.0),
                listOf(10.0, 40.0),
            )
        }
    }

    /** A PnP fake that returns preset solutions, ignoring its inputs. */
    private class FakePnp(private val solutions: List<PlanarPnpSolver.PnpSolution>) :
        PlanarPnpSolver {
        override fun solveIppeSquare(
            objectPoints: DoubleArray,
            imagePoints: DoubleArray,
            cameraMatrix: DoubleArray,
            distCoeffs: DoubleArray?,
        ): List<PlanarPnpSolver.PnpSolution> = solutions
    }

    @Test
    fun selectsBestAndAltByReprojectionError() {
        val rBest = doubleArrayOf(0.1, 0.2, 0.3)
        val tBest = doubleArrayOf(1.0, 2.0, 3.0)
        val rAlt = doubleArrayOf(0.4, 0.5, 0.6)
        val tAlt = doubleArrayOf(4.0, 5.0, 6.0)
        // Supply worst-first to prove selection is by error, not input order.
        val sols =
            listOf(
                PlanarPnpSolver.PnpSolution(rAlt, tAlt, 2.0),
                PlanarPnpSolver.PnpSolution(rBest, tBest, 0.5),
            )
        val solver = TagAmbiguitySolver(FakePnp(sols), CAM, null, 0.1651)

        val s = solver.solve(fourCorners())
        assertNotNull(s)
        assertEquals(0.25, s!!.ratio, TOL) // 0.5 / 2.0
        assertEquals(0.5, s.reprojErrBest, TOL)
        assertArrayEquals(rBest, s.rvecBest, TOL)
        assertArrayEquals(tBest, s.tvecBest, TOL)
        assertArrayEquals(rAlt, s.rvecAlt, TOL)
        assertArrayEquals(tAlt, s.tvecAlt, TOL)
    }

    @Test
    fun singleSolutionHasNullAltAndZeroAmbiguity() {
        val sols =
            listOf(
                PlanarPnpSolver.PnpSolution(
                    doubleArrayOf(0.0, 0.0, 0.3),
                    doubleArrayOf(1.0, 2.0, 3.0),
                    0.7,
                )
            )
        val solver = TagAmbiguitySolver(FakePnp(sols), CAM, null, 0.1651)

        val s = solver.solve(fourCorners())
        assertNotNull(s)
        assertEquals(0.0, s!!.ratio, TOL)
        assertNull(s.rvecAlt)
        assertNull(s.tvecAlt)
    }

    @Test
    fun ambiguityConvenienceReturnsRatio() {
        val sols =
            listOf(
                PlanarPnpSolver.PnpSolution(
                    doubleArrayOf(0.0, 0.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 1.0),
                    0.5,
                ),
                PlanarPnpSolver.PnpSolution(
                    doubleArrayOf(0.0, 0.0, 1.0),
                    doubleArrayOf(0.0, 0.0, 1.0),
                    2.0,
                ),
            )
        val solver = TagAmbiguitySolver(FakePnp(sols), CAM, null, 0.1651)
        assertEquals(0.25, solver.ambiguity(fourCorners())!!, TOL)
    }

    @Test
    fun nullOnEmptySolutionsOrMalformedCorners() {
        val empty = TagAmbiguitySolver(FakePnp(ArrayList()), CAM, null, 0.1651)
        assertNull(empty.solve(fourCorners()), "no solutions → null")

        // Malformed corners (wrong count) short-circuit before the PnP is even called.
        val threeCorners = ArrayList(fourCorners())
        threeCorners.removeAt(3)
        assertNull(empty.solve(threeCorners), "bad corners → null")
        assertNull(empty.ambiguity(null), "null corners → null")
    }
}
