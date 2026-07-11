package org.marsroboticsassociation.controllib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * The CSV serialization contract for a {@link VisionFrame} plus its camera calibration — the raw
 * per-frame inputs the localizer chain consumes. It is the single source of truth for the
 * <i>replay</i> schema: the on-robot logger writes these columns, and the off-robot replay harness
 * reads them back to reconstruct the exact {@link VisionFrame} and re-run the PnP&nbsp;&rarr;
 * solver&nbsp;&rarr; bank chain (optionally re-solving the PnP from the logged corners with a
 * different method).
 *
 * <p>Both sides are driven from one ordered map ({@link #toMap}), so the written column order and
 * the parsed names can never drift apart. Every column is numeric (booleans/ids are written as
 * {@code 0/1}/{@code -1}); an absent value is {@link Double#NaN}. A round-trip
 * ({@code toMap} &rarr; CSV &rarr; {@link #parse}) reproduces the frame's replay-relevant fields
 * exactly.
 *
 * <p>What is <b>not</b> here: {@code timestamp}/{@code receiptNanos} (the replay clock synthesizes
 * them) and the purely-diagnostic fields the chain never reads (MT2 botpose, MT1 stddevs, out-of-
 * plane z/roll/pitch). The odometry, loop timing, and fused/bank <i>outputs</i> are the logger's
 * own columns wrapped around this frame block, not part of this codec.
 */
public final class VisionFrameCsv {

    private VisionFrameCsv() {}

    private static final String[] T6_SUFFIX = {"_x", "_y", "_z", "_yaw", "_pitch", "_roll"};

    /**
     * The frame's replay columns and their values, in write order, from a {@link VisionFrame} plus
     * the camera intrinsics ({@code cam = [fx,fy,cx,cy]}) and distortion (OpenCV order
     * {@code [k1,k2,p1,p2,k3]}) the on-robot PnP used. Insertion order defines {@link #COLUMNS}.
     */
    public static LinkedHashMap<String, Double> toMap(VisionFrame f, double[] cam, double[] dist) {
        LinkedHashMap<String, Double> m = new LinkedHashMap<>();

        // Frame meta.
        m.put("vis_newFrame", f.valid ? 1.0 : 0.0);
        m.put("vis_tagCount", (double) f.tagCount);
        m.put("vis_avgDistM", f.avgDistM);
        m.put("vis_latencyMs", f.latencySec * 1e3);

        // MegaTag1 botpose (field inches, heading deg) — reference; the chain only null-checks it.
        m.put("vision_x", f.mt1Pose != null ? f.mt1Pose.getX() : Double.NaN);
        m.put("vision_y", f.mt1Pose != null ? f.mt1Pose.getY() : Double.NaN);
        m.put(
                "vision_headingDeg",
                f.mt1Pose != null ? Math.toDegrees(f.mt1Pose.getRotation().getRadians()) : Double.NaN);

        // Detection quality.
        m.put("vis_ambiguity", f.ambiguity);
        m.put("vis_reprojErrPx", f.reprojErrPx);
        m.put("focus_metric", f.focusMetric);
        m.put("tag_skew", f.skew);

        // The winning tag's PnP solutions (the replay-critical raw inputs): tag id + both IPPE
        // (rvec, tvec) candidates, best-first.
        m.put("sol_tagId", (double) f.solTagId);
        putVec3(m, "sol_best_r", f.solBestRvec);
        putVec3(m, "sol_best_t", f.solBestTvec);
        putVec3(m, "sol_alt_r", f.solAltRvec);
        putVec3(m, "sol_alt_t", f.solAltTvec);

        // Limelight's own per-tag / botpose 6-DOF (reference for comparison).
        putVec6(m, "t6tCs", f.t6tCs);
        putVec6(m, "t6tRs", f.t6tRs);
        putVec6(m, "t6rFs", f.t6rFs);

        // Winning tag bounding box + centering angles (the sigma(span) regressor).
        m.put("vis_tagTxDeg", f.tagTxDeg);
        m.put("vis_tagTyDeg", f.tagTyDeg);
        m.put("vis_tagMinXPx", f.tagMinXPx);
        m.put("vis_tagMaxXPx", f.tagMaxXPx);
        m.put("vis_tagMinYPx", f.tagMinYPx);
        m.put("vis_tagMaxYPx", f.tagMaxYPx);

        // ALL detected tags (up to 4 slots): id + 4 image corners each — the raw per-landmark
        // observations an offline PnP re-solve consumes. Empty slots are id -1 / NaN corners.
        for (int k = 0; k < 4; k++) {
            int id = (f.allTagIds != null && k < f.allTagIds.length) ? f.allTagIds[k] : -1;
            double[] c =
                    (f.allTagCorners != null && k < f.allTagCorners.length) ? f.allTagCorners[k] : null;
            m.put("tag" + k + "_id", (double) id);
            for (int j = 0; j < 4; j++) {
                m.put("tag" + k + "_c" + j + "x", c != null && 2 * j < c.length ? c[2 * j] : Double.NaN);
                m.put(
                        "tag" + k + "_c" + j + "y",
                        c != null && 2 * j + 1 < c.length ? c[2 * j + 1] : Double.NaN);
            }
        }

        // Camera intrinsics + distortion actually used on-robot (so an offline re-solve matches).
        m.put("vis_calFx", at(cam, 0));
        m.put("vis_calFy", at(cam, 1));
        m.put("vis_calCx", at(cam, 2));
        m.put("vis_calCy", at(cam, 3));
        String[] distNames = {"vis_dist_k1", "vis_dist_k2", "vis_dist_p1", "vis_dist_p2", "vis_dist_k3"};
        for (int i = 0; i < distNames.length; i++) {
            m.put(distNames[i], at(dist, i));
        }
        return m;
    }

    /** The frame column names, in write order (derived from an empty {@link #toMap}). */
    public static final List<String> COLUMNS =
            new ArrayList<>(toMap(new VisionFrame(), new double[4], new double[5]).keySet());

    /** The frame columns joined as a CSV header fragment. */
    public static final String HEADER = String.join(",", COLUMNS);

    /** The reconstructed frame plus the camera calibration read from a row. */
    public static final class Parsed {
        public final VisionFrame frame;
        public final double[] cameraMatrix; // row-major 3x3 [fx,0,cx, 0,fy,cy, 0,0,1], NaN if absent
        public final double[] distCoeffs; // [k1,k2,p1,p2,k3]

        Parsed(VisionFrame frame, double[] cameraMatrix, double[] distCoeffs) {
            this.frame = frame;
            this.cameraMatrix = cameraMatrix;
            this.distCoeffs = distCoeffs;
        }
    }

    /**
     * Reconstruct a {@link VisionFrame} (and its calibration) from a row, read by column name via
     * {@code get} ({@link Double#NaN} for absent columns). {@code valid}/{@code timestamp} are NOT
     * set here — the replay source assigns them from {@code vis_newFrame} and the frame clock.
     */
    public static Parsed parse(Function<String, Double> get) {
        VisionFrame f = new VisionFrame();
        f.tagCount = (int) Math.round(nz(get.apply("vis_tagCount")));
        f.avgDistM = get.apply("vis_avgDistM");
        f.latencySec = get.apply("vis_latencyMs") / 1e3;

        double vx = get.apply("vision_x");
        double vy = get.apply("vision_y");
        double vh = get.apply("vision_headingDeg");
        f.mt1Pose =
                Double.isFinite(vx) ? new Pose2d(vx, vy, new Rotation2d(Math.toRadians(vh))) : null;

        f.ambiguity = get.apply("vis_ambiguity");
        f.reprojErrPx = get.apply("vis_reprojErrPx");
        f.focusMetric = get.apply("focus_metric");
        f.skew = get.apply("tag_skew");

        f.solTagId = (int) Math.round(nz(get.apply("sol_tagId"), -1));
        f.solBestRvec = vec3(get, "sol_best_r");
        f.solBestTvec = vec3(get, "sol_best_t");
        f.solAltRvec = vec3(get, "sol_alt_r");
        f.solAltTvec = vec3(get, "sol_alt_t");

        f.t6tCs = vec6(get, "t6tCs");
        f.t6tRs = vec6(get, "t6tRs");
        f.t6rFs = vec6(get, "t6rFs");

        f.tagTxDeg = get.apply("vis_tagTxDeg");
        f.tagTyDeg = get.apply("vis_tagTyDeg");
        f.tagMinXPx = get.apply("vis_tagMinXPx");
        f.tagMaxXPx = get.apply("vis_tagMaxXPx");
        f.tagMinYPx = get.apply("vis_tagMinYPx");
        f.tagMaxYPx = get.apply("vis_tagMaxYPx");

        List<Integer> ids = new ArrayList<>();
        List<double[]> corners = new ArrayList<>();
        for (int k = 0; k < 4; k++) {
            double id = get.apply("tag" + k + "_id");
            if (!Double.isFinite(id) || id < 0) {
                continue;
            }
            ids.add((int) Math.round(id));
            double[] c = new double[8];
            for (int j = 0; j < 4; j++) {
                c[2 * j] = get.apply("tag" + k + "_c" + j + "x");
                c[2 * j + 1] = get.apply("tag" + k + "_c" + j + "y");
            }
            corners.add(c);
        }
        if (!ids.isEmpty()) {
            f.allTagIds = ids.stream().mapToInt(Integer::intValue).toArray();
            f.allTagCorners = corners.toArray(new double[0][]);
        }

        double fx = get.apply("vis_calFx");
        double fy = get.apply("vis_calFy");
        double cx = get.apply("vis_calCx");
        double cy = get.apply("vis_calCy");
        double[] cameraMatrix = {fx, 0, cx, 0, fy, cy, 0, 0, 1};
        double[] dist = {
            get.apply("vis_dist_k1"),
            get.apply("vis_dist_k2"),
            get.apply("vis_dist_p1"),
            get.apply("vis_dist_p2"),
            get.apply("vis_dist_k3"),
        };
        return new Parsed(f, cameraMatrix, dist);
    }

    // --- helpers --------------------------------------------------------------------------------

    private static void putVec3(Map<String, Double> m, String prefix, double[] v) {
        for (int i = 0; i < 3; i++) {
            m.put(prefix + i, v != null && i < v.length ? v[i] : Double.NaN);
        }
    }

    private static void putVec6(Map<String, Double> m, String prefix, double[] v) {
        for (int i = 0; i < 6; i++) {
            m.put(prefix + T6_SUFFIX[i], v != null && i < v.length ? v[i] : Double.NaN);
        }
    }

    /** Reads {@code <prefix>0..2}, or null if all three are NaN. */
    private static double[] vec3(Function<String, Double> get, String prefix) {
        double x = get.apply(prefix + "0"), y = get.apply(prefix + "1"), z = get.apply(prefix + "2");
        return (Double.isFinite(x) || Double.isFinite(y) || Double.isFinite(z))
                ? new double[] {x, y, z}
                : null;
    }

    /** Reads the six {@code <prefix>{_x,_y,_z,_yaw,_pitch,_roll}} columns, or null if all NaN. */
    private static double[] vec6(Function<String, Double> get, String prefix) {
        double[] v = new double[6];
        boolean any = false;
        for (int i = 0; i < 6; i++) {
            v[i] = get.apply(prefix + T6_SUFFIX[i]);
            any |= Double.isFinite(v[i]);
        }
        return any ? v : null;
    }

    private static double at(double[] a, int i) {
        return a != null && i < a.length ? a[i] : Double.NaN;
    }

    private static double nz(double v) {
        return Double.isFinite(v) ? v : 0.0;
    }

    private static double nz(double v, double fallback) {
        return Double.isFinite(v) ? v : fallback;
    }
}
