package org.marsroboticsassociation.controllib.localization.vision.replay

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import java.io.IOException
import java.nio.file.Files
import java.nio.file.Path
import kotlin.math.hypot
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertArrayEquals
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Assumptions.assumeTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.io.TempDir
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer
import org.marsroboticsassociation.controllib.localization.vision.Transform3D
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame
import org.marsroboticsassociation.controllib.localization.vision.VisionFrameCsv
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolver
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolverConfig

/**
 * Tests the replay contract end to end: (1) the [VisionFrameCsv] schema round-trips a frame through
 * a map and through a real CSV + [CsvVisionSource]; (2) [BankReplay] drives the pure localizer
 * deterministically off a recorded CSV, honoring the pre-commit seed. Plus an opt-in replay of a
 * real external recording (skipped unless `-Dbank.replay.csv=...` is set), which is the mechanism
 * for testing different approaches against logged data.
 */
class VisionReplayTest {

    companion object {
        private const val TOL = 1e-9

        private val CAM = doubleArrayOf(800.0, 800.0, 640.0, 480.0) // fx, fy, cx, cy
        private val DIST = doubleArrayOf(0.1, -0.2, 0.001, 0.002, 0.05)

        /**
         * Curiosity DECODE competition geometry (fixture only — not a library default): that
         * robot's Limelight mount plus the DECODE goal-tag map. Used so replay tests that touch the
         * vision chain have a consistent, documented solver.
         */
        private fun fixtureSolver(): VisionPoseSolver {
            val tags = HashMap<Int, Transform3D>()
            // ftc2025DECODE.fmap tags 20 / 24
            tags[20] =
                VisionPoseSolverConfig.fromRowMajor4x4(
                    doubleArrayOf(
                        0.5877852522924731,
                        -0.8090169943749473,
                        0.0,
                        -1.4827,
                        0.8090169943749473,
                        0.5877852522924731,
                        0.0,
                        -1.4133,
                        0.0,
                        0.0,
                        1.0,
                        0.7493,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    )
                )
            tags[24] =
                VisionPoseSolverConfig.fromRowMajor4x4(
                    doubleArrayOf(
                        0.5877852522924731,
                        0.8090169943749473,
                        0.0,
                        -1.4827,
                        -0.8090169943749473,
                        0.5877852522924731,
                        0.0,
                        1.4133,
                        0.0,
                        0.0,
                        1.0,
                        0.7493,
                        0.0,
                        0.0,
                        0.0,
                        1.0,
                    )
                )
            return VisionPoseSolverConfig(
                    VisionPoseSolverConfig.robotFromCameraFromLimelightRs(
                        0.0816,
                        -0.0400,
                        0.3835,
                        180.0,
                        0.23,
                        -0.78,
                    ),
                    tags,
                )
                .solver()
        }

        /** A frame carrying the replay-relevant fields, for the round-trip assertions. */
        private fun sampleFrame(): VisionFrame {
            val f = VisionFrame()
            f.valid = true
            f.tagCount = 2
            f.avgDistM = 1.5
            f.latencySec = 0.08
            f.mt1Pose = Pose2d(10.0, -5.0, Rotation2d(0.3))
            f.ambiguity = 0.2
            f.reprojErrPx = 0.7
            f.focusMetric = 123.0
            f.skew = 0.01
            f.solTagId = 20
            f.solBestRvec = doubleArrayOf(0.1, 0.2, 0.3)
            f.solBestTvec = doubleArrayOf(1.0, 2.0, 3.0)
            f.solAltRvec = doubleArrayOf(0.4, 0.5, 0.6)
            f.solAltTvec = doubleArrayOf(4.0, 5.0, 6.0)
            f.t6tCs = doubleArrayOf(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
            f.t6tRs = doubleArrayOf(7.0, 8.0, 9.0, 10.0, 11.0, 12.0)
            f.t6rFs = doubleArrayOf(13.0, 14.0, 15.0, 16.0, 17.0, 18.0)
            f.tagTxDeg = 1.1
            f.tagTyDeg = -2.2
            f.tagMinXPx = 100.0
            f.tagMaxXPx = 240.0
            f.tagMinYPx = 120.0
            f.tagMaxYPx = 250.0
            f.allTagIds = intArrayOf(20, 24)
            f.allTagCorners =
                arrayOf(
                    doubleArrayOf(100.0, 120.0, 240.0, 120.0, 240.0, 250.0, 100.0, 250.0),
                    doubleArrayOf(300.0, 130.0, 420.0, 130.0, 420.0, 255.0, 300.0, 255.0),
                )
            return f
        }

        @Throws(IOException::class)
        private fun writeRun(path: Path, odo: List<DoubleArray>, fused: Array<DoubleArray>) {
            val header = ArrayList<String>()
            header.add("t_ms")
            header.add("loop_ms")
            header.add("odo_x")
            header.add("odo_y")
            header.add("odo_headingDeg")
            header.add("ang_vel_deg_s")
            header.addAll(VisionFrameCsv.COLUMNS)
            header.add("fused_x")
            header.add("fused_y")
            header.add("fused_headingDeg")

            val sb = StringBuilder(header.joinToString(",")).append('\n')
            val frame =
                VisionFrameCsv.toMap(VisionFrame(), DoubleArray(4), DoubleArray(5)) // all invalid
            for (i in odo.indices) {
                val row = ArrayList<Any?>()
                row.add(i * 20.0) // t_ms
                row.add(20.0) // loop_ms
                row.add(odo[i][0])
                row.add(odo[i][1])
                row.add(odo[i][2])
                row.add(0.0) // ang_vel_deg_s
                for (c in VisionFrameCsv.COLUMNS) {
                    row.add(frame[c])
                }
                row.add(fused[i][0])
                row.add(fused[i][1])
                row.add(fused[i][2])
                for (c in row.indices) {
                    if (c > 0) sb.append(',')
                    sb.append(row[c])
                }
                sb.append('\n')
            }
            Files.writeString(path, sb.toString())
        }
    }

    @Test
    fun codecRoundTripsFrameThroughMap() {
        val f = sampleFrame()
        val m = VisionFrameCsv.toMap(f, CAM, DIST)
        val p = VisionFrameCsv.parse { c -> m.getOrDefault(c, Double.NaN) }

        assertEquals(f.tagCount, p.frame.tagCount)
        assertEquals(f.avgDistM, p.frame.avgDistM, TOL)
        assertEquals(f.latencySec, p.frame.latencySec, TOL)
        assertEquals(f.mt1Pose!!.x, p.frame.mt1Pose!!.x, TOL)
        assertEquals(f.mt1Pose!!.y, p.frame.mt1Pose!!.y, TOL)
        assertEquals(
            f.mt1Pose!!.rotation.radians,
            p.frame.mt1Pose!!.rotation.radians,
            1e-9,
        )
        assertEquals(f.ambiguity, p.frame.ambiguity, TOL)
        assertEquals(f.reprojErrPx, p.frame.reprojErrPx, TOL)
        assertEquals(f.solTagId, p.frame.solTagId)
        assertArrayEquals(f.solBestRvec, p.frame.solBestRvec, TOL)
        assertArrayEquals(f.solBestTvec, p.frame.solBestTvec, TOL)
        assertArrayEquals(f.solAltRvec, p.frame.solAltRvec, TOL)
        assertArrayEquals(f.solAltTvec, p.frame.solAltTvec, TOL)
        assertArrayEquals(f.t6tCs, p.frame.t6tCs, TOL)
        assertArrayEquals(f.t6tRs, p.frame.t6tRs, TOL)
        assertArrayEquals(f.t6rFs, p.frame.t6rFs, TOL)
        assertEquals(f.tagMaxXPx, p.frame.tagMaxXPx, TOL)
        assertArrayEquals(f.allTagIds, p.frame.allTagIds)
        assertArrayEquals(f.allTagCorners!![0], p.frame.allTagCorners!![0], TOL)
        assertArrayEquals(f.allTagCorners!![1], p.frame.allTagCorners!![1], TOL)
        // Intrinsics: cameraMatrix is row-major [fx,0,cx, 0,fy,cy, 0,0,1].
        assertEquals(CAM[0], p.cameraMatrix[0], TOL)
        assertEquals(CAM[1], p.cameraMatrix[4], TOL)
        assertEquals(CAM[2], p.cameraMatrix[2], TOL)
        assertEquals(CAM[3], p.cameraMatrix[5], TOL)
        assertArrayEquals(DIST, p.distCoeffs, TOL)
    }

    @Test
    fun csvRoundTripsThroughCsvVisionSource(@TempDir dir: Path) {
        val f = sampleFrame()
        val m = VisionFrameCsv.toMap(f, CAM, DIST)

        val csvPath = dir.resolve("frame.csv")
        val sb = StringBuilder(VisionFrameCsv.HEADER).append('\n')
        for (i in VisionFrameCsv.COLUMNS.indices) {
            if (i > 0) sb.append(',')
            sb.append(m[VisionFrameCsv.COLUMNS[i]])
        }
        sb.append('\n')
        Files.writeString(csvPath, sb.toString())

        val csv = TelemetryCsv.read(csvPath)
        assertEquals(1, csv.size())
        val src = CsvVisionSource()
        src.prepare(csv, 0, 42L)
        val g = src.latest()

        assertTrue(g.valid, "vis_newFrame==1 -> valid")
        assertEquals(42L, g.receiptNanos)
        assertEquals(f.solTagId, g.solTagId)
        assertArrayEquals(f.solBestRvec, g.solBestRvec, TOL)
        assertArrayEquals(f.solAltTvec, g.solAltTvec, TOL)
        assertArrayEquals(f.allTagCorners!![0], g.allTagCorners!![0], TOL)
        assertEquals(CAM[0], src.getCalFx(), TOL)
        assertEquals(CAM[3], src.getCalCy(), TOL)
        assertArrayEquals(DIST, src.getCalDistCoeffs(), TOL)
    }

    @Test
    fun bankReplayHonorsSeedAndIsDeterministic(@TempDir dir: Path) {
        // Three rows of pure odometry motion (no usable vision: valid=false frames). The bank never
        // commits, so getPose rides the pre-commit seed = impliedDatum(fused0, odo0) ∘ odo.
        val csvPath = dir.resolve("run.csv")
        val odo =
            listOf(
                doubleArrayOf(0.0, 0.0, 0.0),
                doubleArrayOf(4.0, 0.0, 0.0),
                doubleArrayOf(4.0, 0.0, 90.0),
            )
        val fused =
            arrayOf(
                doubleArrayOf(10.0, 5.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0),
                doubleArrayOf(0.0, 0.0, 0.0),
            ) // only row 0's fused seeds the replay
        writeRun(csvPath, odo, fused)

        val replay = BankReplay(HypothesisBankLocalizer.Params(), fixtureSolver())
        val a = replay.replay(csvPath)
        val b = replay.replay(csvPath)

        // Deterministic.
        assertArrayEquals(a.fusedX, b.fusedX, 0.0)
        assertArrayEquals(a.fusedY, b.fusedY, 0.0)
        assertArrayEquals(a.fusedHeadingDeg, b.fusedHeadingDeg, 0.0)

        // No vision -> never commits, bank stays empty.
        assertEquals(0, a.committedRows)
        assertEquals(0, a.maxBankSize)

        // Row 0: seed ∘ odo0 == fused0. Row 1: datum (10,5,0) ∘ (4,0,0) == (14,5,0).
        assertEquals(10.0, a.fusedX[0], 1e-6)
        assertEquals(5.0, a.fusedY[0], 1e-6)
        assertEquals(14.0, a.fusedX[1], 1e-6)
        assertEquals(5.0, a.fusedY[1], 1e-6)
        // Row 2: odo rotated 90°, datum heading 0 -> field heading 90°, position still (14,5).
        assertEquals(90.0, a.fusedHeadingDeg[2], 1e-6)
        assertEquals(14.0, a.fusedX[2], 1e-6)
    }

    /**
     * Opt-in: replay a real recording end to end and assert the pure chain reproduces the logged
     * fused pose on the committed rows. Skipped unless `-Dbank.replay.csv=<path>` points at a
     * recorded CSV. This is the hook for testing different params/solvers/PnP on logged data.
     */
    @Test
    fun replaysExternalRecordingIfProvided() {
        val path = System.getProperty("bank.replay.csv")
        assumeTrue(path != null && path.isNotBlank(), "set -Dbank.replay.csv to run")

        // External CSVs are robot-specific: pass the mount that matches the recording (Curiosity
        // fixture below is the historical DECODE competition mount; swap for other robots).
        val replay = BankReplay(HypothesisBankLocalizer.Params(), fixtureSolver())
        val r = replay.replay(Path.of(path))
        assertTrue(r.rows > 0, "recording had rows")

        var compared = 0
        var maxErr = 0.0
        for (i in 0 until r.rows) {
            if (!r.recX[i].isFinite() || !r.fusedX[i].isFinite()) {
                continue
            }
            val e = hypot(r.fusedX[i] - r.recX[i], r.fusedY[i] - r.recY[i])
            maxErr = max(maxErr, e)
            compared++
        }
        assertTrue(compared > 0, "no comparable rows")
        // Same params as the recording -> the replayed fused trajectory should track it closely.
        assertTrue(maxErr < 2.0, "replayed fused pose diverged from recorded by $maxErr in")
    }
}
