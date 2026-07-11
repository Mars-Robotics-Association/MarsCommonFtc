package org.marsroboticsassociation.controllib.localization.vision.replay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer;
import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver;
import org.marsroboticsassociation.controllib.localization.vision.TagAmbiguitySolver;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame;
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolver;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Replays one recorded telemetry CSV through the pure {@link HypothesisBankLocalizer} off-robot:
 * each row reconstructs the {@link VisionFrame} (via {@link CsvVisionSource}) and feeds the recorded
 * absolute odometry straight into {@code update()} — no odometry-integration replay is needed
 * because the bank consumes field-frame odometry directly. Returns the replayed vs. recorded fused
 * trajectory so a harness can (a) assert byte-fidelity under the recording's own params, or (b)
 * measure how a <i>different</i> configuration (bank params, solver, or PnP method) would have
 * behaved on the same sensor stream.
 *
 * <h3>Testing a different PnP method</h3>
 *
 * Pass a {@link PlanarPnpSolver} + tag size to the re-solving constructor and the driver re-runs the
 * winning tag's PnP from the logged corners + intrinsics (replacing the logged {@code sol_*} rvec/
 * tvec) before feeding the bank — so you can compare, e.g., a different IPPE backend against the
 * recorded one on identical corner observations.
 */
public final class BankReplay {

    /** Replayed vs. recorded fused trajectory, plus bank-health summaries. */
    public static final class Result {
        public final int rows;
        public final double[] tMs;
        public final double[] fusedX, fusedY, fusedHeadingDeg; // replayed
        public final double[] recX, recY, recHeadingDeg; // recorded
        public final int maxBankSize;
        public final long committedRows;

        Result(
                int rows,
                double[] tMs,
                double[] fusedX,
                double[] fusedY,
                double[] fusedHeadingDeg,
                double[] recX,
                double[] recY,
                double[] recHeadingDeg,
                int maxBankSize,
                long committedRows) {
            this.rows = rows;
            this.tMs = tMs;
            this.fusedX = fusedX;
            this.fusedY = fusedY;
            this.fusedHeadingDeg = fusedHeadingDeg;
            this.recX = recX;
            this.recY = recY;
            this.recHeadingDeg = recHeadingDeg;
            this.maxBankSize = maxBankSize;
            this.committedRows = committedRows;
        }
    }

    private final HypothesisBankLocalizer.Params params;
    private final VisionPoseSolver solver;
    private final PlanarPnpSolver pnp; // null = replay logged rvec/tvec; non-null = re-solve
    private final double tagSizeMeters;

    /** Replay from the logged PnP solutions (no re-solve). */
    public BankReplay(HypothesisBankLocalizer.Params params, VisionPoseSolver solver) {
        this(params, solver, null, 0);
    }

    /**
     * @param pnp if non-null, re-solve each frame's winning tag from its logged corners + intrinsics
     *     with this solver before feeding the bank
     * @param tagSizeMeters AprilTag side length for the re-solve
     */
    public BankReplay(
            HypothesisBankLocalizer.Params params,
            VisionPoseSolver solver,
            PlanarPnpSolver pnp,
            double tagSizeMeters) {
        this.params = params;
        this.solver = solver;
        this.pnp = pnp;
        this.tagSizeMeters = tagSizeMeters;
    }

    public Result replay(Path input) throws IOException {
        TelemetryCsv csv = TelemetryCsv.read(input);
        int n = csv.size();

        HypothesisBankLocalizer core = new HypothesisBankLocalizer(params, solver);
        CsvVisionSource vision = new CsvVisionSource();

        // Seed the pre-commit fallback from the recording's first fused pose, so the replayed
        // trajectory matches the recording before the bank commits (it rides odometry from there).
        if (n > 0) {
            Pose2d fused0 = pose(csv, 0, "fused_x", "fused_y", "fused_headingDeg");
            Pose2d odo0 = pose(csv, 0, "odo_x", "odo_y", "odo_headingDeg");
            if (fused0 != null && odo0 != null) {
                core.setPose(fused0, odo0);
            }
        }

        double[] tMs = new double[n];
        double[] fx = new double[n], fy = new double[n], fh = new double[n];
        double[] rx = new double[n], ry = new double[n], rh = new double[n];
        int maxBankSize = 0;
        long committedRows = 0;

        TagAmbiguitySolver resolver = null; // built lazily once intrinsics are available

        for (int i = 0; i < n; i++) {
            long nowNanos = (long) (csv.get(i, "t_ms") * 1e6);
            Pose2d odo = pose(csv, i, "odo_x", "odo_y", "odo_headingDeg");
            if (odo == null) {
                odo = Pose2d.kZero;
            }
            double yawRate = Math.toRadians(nz(csv.get(i, "ang_vel_deg_s")));

            vision.prepare(csv, i, nowNanos);
            VisionFrame f = vision.latest();

            if (pnp != null && f.valid) {
                if (resolver == null && Double.isFinite(vision.getCalFx())) {
                    resolver =
                            new TagAmbiguitySolver(
                                    pnp, vision.cameraMatrix(), vision.getCalDistCoeffs(), tagSizeMeters);
                }
                if (resolver != null) {
                    resolveWinningTag(f, resolver);
                }
            }

            core.update(nowNanos, odo, yawRate, f);
            Pose2d fused = core.getPose(odo);

            tMs[i] = csv.get(i, "t_ms");
            fx[i] = fused.getX();
            fy[i] = fused.getY();
            fh[i] = Math.toDegrees(fused.getRotation().getRadians());
            rx[i] = csv.get(i, "fused_x");
            ry[i] = csv.get(i, "fused_y");
            rh[i] = csv.get(i, "fused_headingDeg");

            maxBankSize = Math.max(maxBankSize, core.bank().size());
            if (core.isCommitted()) {
                committedRows++;
            }
        }

        return new Result(n, tMs, fx, fy, fh, rx, ry, rh, maxBankSize, committedRows);
    }

    /** Re-solve the winning tag's PnP from its logged corners and overwrite the frame's sol_*. */
    private static void resolveWinningTag(VisionFrame f, TagAmbiguitySolver resolver) {
        if (f.solTagId < 0 || f.allTagIds == null || f.allTagCorners == null) {
            return;
        }
        int idx = -1;
        for (int k = 0; k < f.allTagIds.length; k++) {
            if (f.allTagIds[k] == f.solTagId) {
                idx = k;
                break;
            }
        }
        if (idx < 0 || idx >= f.allTagCorners.length || f.allTagCorners[idx] == null) {
            return;
        }
        TagAmbiguitySolver.PnpSolutions s = resolver.solve(toCornerList(f.allTagCorners[idx]));
        if (s == null) {
            return;
        }
        f.solBestRvec = s.rvecBest;
        f.solBestTvec = s.tvecBest;
        f.solAltRvec = s.rvecAlt;
        f.solAltTvec = s.tvecAlt;
        f.ambiguity = s.ratio;
        f.reprojErrPx = s.reprojErrBest;
    }

    /** Flat {@code [x0,y0,...,x3,y3]} corners to the {@code List<List<Double>>} the solver takes. */
    private static List<List<Double>> toCornerList(double[] flat) {
        List<List<Double>> out = new ArrayList<>(4);
        for (int j = 0; j < 4; j++) {
            out.add(Arrays.asList(flat[2 * j], flat[2 * j + 1]));
        }
        return out;
    }

    /** Reads three columns as a pose (x, y inches; heading degrees), or null if x/y is not finite. */
    private static Pose2d pose(TelemetryCsv csv, int row, String xc, String yc, String hc) {
        double x = csv.get(row, xc), y = csv.get(row, yc), h = csv.get(row, hc);
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return null;
        }
        return new Pose2d(x, y, new Rotation2d(Math.toRadians(nz(h))));
    }

    private static double nz(double v) {
        return Double.isFinite(v) ? v : 0.0;
    }
}
