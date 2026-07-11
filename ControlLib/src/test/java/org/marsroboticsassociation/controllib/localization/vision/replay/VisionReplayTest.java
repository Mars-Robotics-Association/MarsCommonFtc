package org.marsroboticsassociation.controllib.localization.vision.replay;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assumptions.assumeTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrameCsv;
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolverConfig;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

/**
 * Tests the replay contract end to end: (1) the {@link VisionFrameCsv} schema round-trips a frame
 * through a map and through a real CSV + {@link CsvVisionSource}; (2) {@link BankReplay} drives the
 * pure localizer deterministically off a recorded CSV, honoring the pre-commit seed. Plus an opt-in
 * replay of a real external recording (skipped unless {@code -Dbank.replay.csv=...} is set), which
 * is the mechanism for testing different approaches against logged data.
 */
class VisionReplayTest {

    private static final double TOL = 1e-9;

    /** A frame carrying the replay-relevant fields, for the round-trip assertions. */
    private static VisionFrame sampleFrame() {
        VisionFrame f = new VisionFrame();
        f.valid = true;
        f.tagCount = 2;
        f.avgDistM = 1.5;
        f.latencySec = 0.08;
        f.mt1Pose = new Pose2d(10, -5, new Rotation2d(0.3));
        f.ambiguity = 0.2;
        f.reprojErrPx = 0.7;
        f.focusMetric = 123.0;
        f.skew = 0.01;
        f.solTagId = 20;
        f.solBestRvec = new double[] {0.1, 0.2, 0.3};
        f.solBestTvec = new double[] {1, 2, 3};
        f.solAltRvec = new double[] {0.4, 0.5, 0.6};
        f.solAltTvec = new double[] {4, 5, 6};
        f.t6tCs = new double[] {1, 2, 3, 4, 5, 6};
        f.t6tRs = new double[] {7, 8, 9, 10, 11, 12};
        f.t6rFs = new double[] {13, 14, 15, 16, 17, 18};
        f.tagTxDeg = 1.1;
        f.tagTyDeg = -2.2;
        f.tagMinXPx = 100;
        f.tagMaxXPx = 240;
        f.tagMinYPx = 120;
        f.tagMaxYPx = 250;
        f.allTagIds = new int[] {20, 24};
        f.allTagCorners =
                new double[][] {
                    {100, 120, 240, 120, 240, 250, 100, 250},
                    {300, 130, 420, 130, 420, 255, 300, 255}
                };
        return f;
    }

    private static final double[] CAM = {800, 800, 640, 480}; // fx, fy, cx, cy
    private static final double[] DIST = {0.1, -0.2, 0.001, 0.002, 0.05};

    @Test
    void codecRoundTripsFrameThroughMap() {
        VisionFrame f = sampleFrame();
        LinkedHashMap<String, Double> m = VisionFrameCsv.toMap(f, CAM, DIST);
        VisionFrameCsv.Parsed p = VisionFrameCsv.parse(c -> m.getOrDefault(c, Double.NaN));

        assertEquals(f.tagCount, p.frame.tagCount);
        assertEquals(f.avgDistM, p.frame.avgDistM, TOL);
        assertEquals(f.latencySec, p.frame.latencySec, TOL);
        assertEquals(f.mt1Pose.getX(), p.frame.mt1Pose.getX(), TOL);
        assertEquals(f.mt1Pose.getY(), p.frame.mt1Pose.getY(), TOL);
        assertEquals(
                f.mt1Pose.getRotation().getRadians(),
                p.frame.mt1Pose.getRotation().getRadians(),
                1e-9);
        assertEquals(f.ambiguity, p.frame.ambiguity, TOL);
        assertEquals(f.reprojErrPx, p.frame.reprojErrPx, TOL);
        assertEquals(f.solTagId, p.frame.solTagId);
        assertArrayEquals(f.solBestRvec, p.frame.solBestRvec, TOL);
        assertArrayEquals(f.solBestTvec, p.frame.solBestTvec, TOL);
        assertArrayEquals(f.solAltRvec, p.frame.solAltRvec, TOL);
        assertArrayEquals(f.solAltTvec, p.frame.solAltTvec, TOL);
        assertArrayEquals(f.t6tCs, p.frame.t6tCs, TOL);
        assertArrayEquals(f.t6tRs, p.frame.t6tRs, TOL);
        assertArrayEquals(f.t6rFs, p.frame.t6rFs, TOL);
        assertEquals(f.tagMaxXPx, p.frame.tagMaxXPx, TOL);
        assertArrayEquals(f.allTagIds, p.frame.allTagIds);
        assertArrayEquals(f.allTagCorners[0], p.frame.allTagCorners[0], TOL);
        assertArrayEquals(f.allTagCorners[1], p.frame.allTagCorners[1], TOL);
        // Intrinsics: cameraMatrix is row-major [fx,0,cx, 0,fy,cy, 0,0,1].
        assertEquals(CAM[0], p.cameraMatrix[0], TOL);
        assertEquals(CAM[1], p.cameraMatrix[4], TOL);
        assertEquals(CAM[2], p.cameraMatrix[2], TOL);
        assertEquals(CAM[3], p.cameraMatrix[5], TOL);
        assertArrayEquals(DIST, p.distCoeffs, TOL);
    }

    @Test
    void csvRoundTripsThroughCsvVisionSource(@TempDir Path dir) throws IOException {
        VisionFrame f = sampleFrame();
        LinkedHashMap<String, Double> m = VisionFrameCsv.toMap(f, CAM, DIST);

        Path csvPath = dir.resolve("frame.csv");
        StringBuilder sb = new StringBuilder(VisionFrameCsv.HEADER).append('\n');
        for (int i = 0; i < VisionFrameCsv.COLUMNS.size(); i++) {
            if (i > 0) sb.append(',');
            sb.append(m.get(VisionFrameCsv.COLUMNS.get(i)));
        }
        sb.append('\n');
        Files.writeString(csvPath, sb.toString());

        TelemetryCsv csv = TelemetryCsv.read(csvPath);
        assertEquals(1, csv.size());
        CsvVisionSource src = new CsvVisionSource();
        src.prepare(csv, 0, 42L);
        VisionFrame g = src.latest();

        assertTrue(g.valid, "vis_newFrame==1 -> valid");
        assertEquals(42L, g.receiptNanos);
        assertEquals(f.solTagId, g.solTagId);
        assertArrayEquals(f.solBestRvec, g.solBestRvec, TOL);
        assertArrayEquals(f.solAltTvec, g.solAltTvec, TOL);
        assertArrayEquals(f.allTagCorners[0], g.allTagCorners[0], TOL);
        assertEquals(CAM[0], src.getCalFx(), TOL);
        assertEquals(CAM[3], src.getCalCy(), TOL);
        assertArrayEquals(DIST, src.getCalDistCoeffs(), TOL);
    }

    @Test
    void bankReplayHonorsSeedAndIsDeterministic(@TempDir Path dir) throws IOException {
        // Three rows of pure odometry motion (no usable vision: valid=false frames). The bank never
        // commits, so getPose rides the pre-commit seed = impliedDatum(fused0, odo0) ∘ odo.
        Path csvPath = dir.resolve("run.csv");
        List<double[]> odo =
                List.of(
                        new double[] {0, 0, 0},
                        new double[] {4, 0, 0},
                        new double[] {4, 0, 90});
        double[][] fused = {{10, 5, 0}, {0, 0, 0}, {0, 0, 0}}; // only row 0's fused seeds the replay
        writeRun(csvPath, odo, fused);

        BankReplay replay =
                new BankReplay(
                        new HypothesisBankLocalizer.Params(), new VisionPoseSolverConfig().solver());
        BankReplay.Result a = replay.replay(csvPath);
        BankReplay.Result b = replay.replay(csvPath);

        // Deterministic.
        assertArrayEquals(a.fusedX, b.fusedX, 0.0);
        assertArrayEquals(a.fusedY, b.fusedY, 0.0);
        assertArrayEquals(a.fusedHeadingDeg, b.fusedHeadingDeg, 0.0);

        // No vision -> never commits, bank stays empty.
        assertEquals(0, a.committedRows);
        assertEquals(0, a.maxBankSize);

        // Row 0: seed ∘ odo0 == fused0. Row 1: datum (10,5,0) ∘ (4,0,0) == (14,5,0).
        assertEquals(10, a.fusedX[0], 1e-6);
        assertEquals(5, a.fusedY[0], 1e-6);
        assertEquals(14, a.fusedX[1], 1e-6);
        assertEquals(5, a.fusedY[1], 1e-6);
        // Row 2: odo rotated 90°, datum heading 0 -> field heading 90°, position still (14,5).
        assertEquals(90, a.fusedHeadingDeg[2], 1e-6);
        assertEquals(14, a.fusedX[2], 1e-6);
    }

    /**
     * Opt-in: replay a real recording end to end and assert the pure chain reproduces the logged
     * fused pose on the committed rows. Skipped unless {@code -Dbank.replay.csv=<path>} points at a
     * recorded CSV. This is the hook for testing different params/solvers/PnP on logged data.
     */
    @Test
    void replaysExternalRecordingIfProvided() throws IOException {
        String path = System.getProperty("bank.replay.csv");
        assumeTrue(path != null && !path.isBlank(), "set -Dbank.replay.csv to run");

        BankReplay replay =
                new BankReplay(
                        new HypothesisBankLocalizer.Params(), new VisionPoseSolverConfig().solver());
        BankReplay.Result r = replay.replay(Path.of(path));
        assertTrue(r.rows > 0, "recording had rows");

        int compared = 0;
        double maxErr = 0;
        for (int i = 0; i < r.rows; i++) {
            if (!Double.isFinite(r.recX[i]) || !Double.isFinite(r.fusedX[i])) {
                continue;
            }
            double e = Math.hypot(r.fusedX[i] - r.recX[i], r.fusedY[i] - r.recY[i]);
            maxErr = Math.max(maxErr, e);
            compared++;
        }
        assertTrue(compared > 0, "no comparable rows");
        // Same params as the recording -> the replayed fused trajectory should track it closely.
        assertTrue(maxErr < 2.0, "replayed fused pose diverged from recorded by " + maxErr + " in");
    }

    // --- helpers --------------------------------------------------------------------------------

    private static void writeRun(Path path, List<double[]> odo, double[][] fused) throws IOException {
        List<String> header = new ArrayList<>();
        header.add("t_ms");
        header.add("loop_ms");
        header.add("odo_x");
        header.add("odo_y");
        header.add("odo_headingDeg");
        header.add("ang_vel_deg_s");
        header.addAll(VisionFrameCsv.COLUMNS);
        header.add("fused_x");
        header.add("fused_y");
        header.add("fused_headingDeg");

        StringBuilder sb = new StringBuilder(String.join(",", header)).append('\n');
        LinkedHashMap<String, Double> frame =
                VisionFrameCsv.toMap(new VisionFrame(), new double[4], new double[5]); // all invalid
        for (int i = 0; i < odo.size(); i++) {
            List<Object> row = new ArrayList<>();
            row.add(i * 20.0); // t_ms
            row.add(20.0); // loop_ms
            row.add(odo.get(i)[0]);
            row.add(odo.get(i)[1]);
            row.add(odo.get(i)[2]);
            row.add(0.0); // ang_vel_deg_s
            for (String c : VisionFrameCsv.COLUMNS) {
                row.add(frame.get(c));
            }
            row.add(fused[i][0]);
            row.add(fused[i][1]);
            row.add(fused[i][2]);
            for (int c = 0; c < row.size(); c++) {
                if (c > 0) sb.append(',');
                sb.append(row.get(c));
            }
            sb.append('\n');
        }
        Files.writeString(path, sb.toString());
    }
}
