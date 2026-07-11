package org.marsroboticsassociation.controllib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * One camera frame's already-extracted features — the seam between vision <i>acquisition</i>
 * (turning a Limelight {@code LLResult} into numbers, or reading them back from a telemetry CSV)
 * and the localizer policy in {@link HypothesisBankLocalizer}. The policy reads only this struct, so
 * it runs identically against a live camera and against a recording.
 *
 * <p>Plain mutable fields, no logic: it is built once per frame by a {@link VisionSource} and
 * consumed immediately. Field names and units mirror the diagnostics they feed (and the CSV columns
 * they are logged as), so the mapping in both directions is one-to-one.
 *
 * <p>All the {@code sol*}/{@code t6t*}/{@code t6r*}/{@code tag*Px}/{@code focusMetric}/{@code
 * stddevMt1}/{@code skew} fields are pure diagnostics the policy does not gate on (except {@code
 * t6tRs} which drives the sweep-rate computation); they are carried so the replay output CSV
 * reproduces the original schema. A {@code double[]} diagnostic that was unavailable is left {@code
 * null}; a scalar is {@link Double#NaN}.
 */
public final class VisionFrame {

    /**
     * A vision result was present and parseable this loop (LLResult.isValid, or a CSV row exists).
     */
    public boolean valid;

    /**
     * Monotonic frame timestamp used only for fresh-frame de-duplication: the policy processes a
     * frame when this differs from the previous one. On a live camera this is the Limelight result
     * timestamp; in replay it is a synthetic counter that advances once per logged {@code
     * vis_newFrame == 1} row.
     */
    public double timestamp;

    /**
     * Receipt time of this result in the {@link System#nanoTime()} clock domain (back-date anchor).
     */
    public long receiptNanos;

    public int tagCount;
    public double avgDistM = Double.NaN;

    /** Total pipeline latency (capture + targeting), seconds. */
    public double latencySec = Double.NaN;

    /** MegaTag1 botpose (field inches, heading radians), or null when no botpose is present. */
    public Pose2d mt1Pose;

    /** Out-of-plane DOF of the raw botpose the 2D fusion discards (height in, roll/pitch deg). */
    public double visionZIn = Double.NaN;

    public double visionRollDeg = Double.NaN;
    public double visionPitchDeg = Double.NaN;

    /** Raw, un-consumed MegaTag2 botpose (field inches, heading radians), or null. */
    public Pose2d mt2Pose;

    /** Pose PnP ambiguity (our own re-solve), NaN when uncomputable. Higher = more flip-prone. */
    public double ambiguity = Double.NaN;

    /**
     * Absolute reprojection error (px) of the winning tag's best PnP solution — a prior-free
     * detection-quality signal (rises with blur / occlusion / far / oblique), distinct from the
     * {@link #ambiguity} ratio. NaN when uncomputable (no intrinsics/corners) or in replay (the raw
     * corners aren't logged, so it can't be re-solved — it round-trips from the recorded column).
     */
    public double reprojErrPx = Double.NaN;

    // --- Pure diagnostics (carried for CSV schema parity; only t6tRs feeds the policy, via sweep)
    // ---
    public double focusMetric = Double.NaN;
    public double[] stddevMt1; // LL MT1 pose stddev [x,y,z,roll,pitch,yaw]
    public double skew = Double.NaN;
    public int solTagId = -1;
    public double[] solBestRvec;
    public double[] solBestTvec;
    public double[] solAltRvec;
    public double[] solAltTvec;
    public double[] t6tCs; // target-in-camera [x,y,z(m),yaw,pitch,roll(deg)]
    public double[] t6tRs; // target-in-robot   (drives the sweep-rate computation)
    public double[] t6rFs; // robot-in-field
    public double tagTxDeg = Double.NaN;
    public double tagTyDeg = Double.NaN;
    public double tagMinXPx = Double.NaN;
    public double tagMaxXPx = Double.NaN;
    public double tagMinYPx = Double.NaN;
    public double tagMaxYPx = Double.NaN;
    // ALL AprilTags detected this frame (not just the winning one): parallel arrays of fiducial id
    // and that tag's 4 image corners flattened [x0,y0,x1,y1,x2,y2,x3,y3] (px). Null when no
    // detection. These are the raw per-landmark observations a multi-tag SLAM / factor-graph
    // optimizes over; with intrinsics + distortion + the tag map the whole PnP front-end is
    // replayable off-robot. The winning tag (which drove sol_*/ambiguity) is the entry whose id
    // equals solTagId.
    public int[] allTagIds;
    public double[][] allTagCorners;
}
