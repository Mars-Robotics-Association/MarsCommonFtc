package org.marsroboticsassociation.controllib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Map;
import java.util.TreeMap;

/**
 * Pure, library-agnostic policy core that drives a {@link PoseHypothesisBank} from a stream of
 * {@link VisionFrame}s and odometry poses — the host-agnostic half of the multi-hypothesis
 * localizer. It owns the bank, the vision-pose solver, the per-frame gating (motion-blur reject,
 * fresh-frame de-duplication), the latency compensation (fitting each frame's datum against the
 * odometry that was true at the vision <i>capture</i> instant), and the commit latch.
 *
 * <p>Everything here is in WPILib {@link Pose2d} (field inches, radians) with no Road Runner, no FTC
 * SDK, no {@code @Config}, and no clock of its own — the host adapter supplies the current odometry
 * pose, the loop timestamp (a {@link System#nanoTime()}-domain value), the yaw rate (for the blur
 * gate), and the {@link VisionFrame}. That keeps the whole gating + fusion path desktop-testable and
 * reusable across drive libraries; a thin adapter binds it to a specific localizer interface.
 *
 * <h3>Usage</h3>
 *
 * <ol>
 *   <li>Optionally {@link #setPose} to seed a trusted starting field pose.</li>
 *   <li>Call {@link #update} every loop with {@code (nowNanos, odoPose, yawRateRadPerSec, frame)}.</li>
 *   <li>Read {@link #getPose} for the fused field pose, and {@link #isCommitted} /
 *       {@link #dominantWeight} for the commitment level (gate auto-aim on these).</li>
 * </ol>
 *
 * <h3>Pre-commit behavior</h3>
 *
 * Commit is <b>vision-driven</b>: {@link #isCommitted} latches only when the bank's dominant weight
 * crosses {@link Params#commitWeight}, i.e. vision has confirmed a branch. Before that, {@link
 * #getPose} rides a seed datum from {@link #setPose} composed with live odometry (the start pose the
 * host handed in), or raw odometry if nothing was seeded. This is the bank-only path: there is no
 * EKF pre-commit fallback — the bank is the whole estimator.
 */
public final class HypothesisBankLocalizer {

    private static final double M_TO_IN = 1.0 / 0.0254;

    /** Tuning for the policy core (the bank has its own nested {@link PoseHypothesisBank.Params}). */
    public static final class Params {
        /** The multi-hypothesis bank's own tuning. */
        public PoseHypothesisBank.Params bank = new PoseHypothesisBank.Params();

        /**
         * Dominant-hypothesis weight at/above which the bank is "committed": {@link #getPose}
         * switches to the bank's MAP and stays there (the bank's own reweighting handles later
         * flips, so this never needs to un-latch).
         */
        public double commitWeight = 0.7;

        /**
         * Motion-blur reject: frames captured while the robot's yaw rate exceeds this are skipped
         * (a fast pan smears the tag corners and corrupts the PnP). Radians/sec.
         */
        public double maxYawRateRadPerSec = Math.toRadians(90.0);

        /**
         * Differential-latency correction (seconds): the effective vision capture time is {@code
         * receiptNanos - (frame.latencySec - odomLatencySec)}. Back-dates vision <i>less</i> by the
         * odometry computer's own lag so the datum isn't fit against stale odometry. Measure offline;
         * 0 = full back-date. See the source project's latency-calibration notes.
         */
        public double odomLatencySec = 0.035;

        /** How long odometry history is retained for latency compensation (seconds). */
        public double odometryHistorySec = 1.5;
    }

    private final Params params;
    private final PoseHypothesisBank bank;
    private final VisionPoseSolver solver;

    /** Odometry history (nanoTime → field-frame odometry pose) for capture-time latency alignment. */
    private final TreeMap<Long, Pose2d> odometryBuffer = new TreeMap<>();

    private double bankLastSampleTs = Double.NaN;
    private boolean committed = false;

    // Pre-commit fallback: the datum implied by the last seeded (fieldPose, odoPose). getPose rides
    // this composed with live odometry until vision commits the bank. Null until setPose is called.
    private Pose2d fallbackDatum = null;

    // --- Per-update diagnostics (for logging / debugging; see the last* accessors) ---
    private VisionFrame lastFrame;          // the frame handed to the most recent update()
    private double lastYawRateRadPerSec = Double.NaN;
    private boolean lastFoldedNewFrame = false; // did the last update fold a fresh frame into the bank
    private Pose2d[] lastBranchPoses;       // the branch field poses that update fed the bank, or null

    public HypothesisBankLocalizer(Params params, VisionPoseSolver solver) {
        this.params = params;
        this.solver = solver;
        this.bank = new PoseHypothesisBank(params.bank);
    }

    /**
     * Seed a trusted starting field pose. Sets the pre-commit fallback datum (implied by {@code
     * (fieldPose, odoPose)}), so {@link #getPose} reports {@code fieldPose} riding odometry until
     * vision commits the bank. Does <b>not</b> mark the localizer committed — commit stays
     * vision-driven — and does not inject a hypothesis, so a wrong prior cannot bias branch
     * selection.
     *
     * @param fieldPose the trusted field pose (inches, radians)
     * @param odoPose the current odometry pose at the same instant
     */
    public void setPose(Pose2d fieldPose, Pose2d odoPose) {
        fallbackDatum = PoseHypothesisBank.impliedDatum(fieldPose, odoPose);
    }

    /**
     * Feed one loop of odometry + vision.
     *
     * @param nowNanos the current time in the {@link System#nanoTime()} domain
     * @param odoPose the current field-frame odometry pose (inches, radians)
     * @param yawRateRadPerSec the current yaw rate (for the motion-blur gate)
     * @param frame this loop's vision frame (never null; {@code valid=false} means no usable result)
     */
    public void update(long nowNanos, Pose2d odoPose, double yawRateRadPerSec, VisionFrame frame) {
        lastFrame = frame;
        lastYawRateRadPerSec = yawRateRadPerSec;
        lastFoldedNewFrame = false;
        lastBranchPoses = null;

        odometryBuffer.put(nowNanos, odoPose);
        long cutoff = nowNanos - (long) (params.odometryHistorySec * 1e9);
        while (!odometryBuffer.isEmpty() && odometryBuffer.firstKey() < cutoff) {
            odometryBuffer.pollFirstEntry();
        }
        serviceHypothesisBank(frame, yawRateRadPerSec, odoPose);
    }

    /**
     * The fused field pose. Once committed, the dominant hypothesis applied to the current odometry
     * (it rides odometry between frames and self-corrects internally). Before commit, the seed datum
     * from {@link #setPose} riding odometry, or raw odometry if nothing was seeded.
     */
    public Pose2d getPose(Pose2d odoPose) {
        if (committed) {
            Pose2d mapPose = bank.mapPose(odoPose);
            if (mapPose != null) {
                return mapPose;
            }
        }
        if (fallbackDatum != null) {
            return PoseHypothesisBank.composeDatumOnOdo(fallbackDatum, odoPose);
        }
        return odoPose;
    }

    /** Whether the bank has committed (dominant weight crossed {@link Params#commitWeight}). */
    public boolean isCommitted() {
        return committed;
    }

    /** Commitment level: dominant-hypothesis weight in {@code (0,1]} (0 if no vision seen). */
    public double dominantWeight() {
        return bank.dominantWeight();
    }

    /** The underlying bank (diagnostics / advanced use). */
    public PoseHypothesisBank bank() {
        return bank;
    }

    /** The {@link VisionFrame} handed to the most recent {@link #update}, or null before any. */
    public VisionFrame lastFrame() {
        return lastFrame;
    }

    /** Whether the most recent {@link #update} folded a fresh valid frame into the bank. */
    public boolean wasNewFrameFolded() {
        return lastFoldedNewFrame;
    }

    /**
     * The 1–2 branch field poses the most recent {@link #update} fed the bank (best-first), or null
     * if no frame was folded — the raw multi-hypothesis input, for debugging branch separation.
     */
    public Pose2d[] lastBranchPoses() {
        return lastBranchPoses;
    }

    /** The yaw rate passed to the most recent {@link #update} (rad/s), or NaN before any. */
    public double lastYawRateRadPerSec() {
        return lastYawRateRadPerSec;
    }

    private void serviceHypothesisBank(VisionFrame frame, double yawRate, Pose2d currentOdo) {
        boolean blurOk = Math.abs(yawRate) <= params.maxYawRateRadPerSec;
        if (frame.valid
                && frame.timestamp != bankLastSampleTs
                && frame.tagCount > 0
                && frame.avgDistM > 0
                && frame.mt1Pose != null
                && blurOk) {
            Pose2d[] branchPoses = branchPosesFromFrame(frame);
            if (branchPoses != null) {
                bankLastSampleTs = frame.timestamp;
                double spanPx = tagSpanPx(frame);
                double[] branchConf = branchConfFromFrame(frame, branchPoses.length);
                // Latency compensation (the "rewind"): the branch poses describe the robot at the
                // vision CAPTURE instant, so fit the datum against the odometry that was true THEN,
                // not the current odometry — otherwise the (constant) datum absorbs the odometry
                // motion over the vision latency and jitters with speed. No forward replay is needed
                // because the datum is constant and getPose() already rides live odometry. Falls
                // back to current odometry if the capture time is outside the history window.
                long captureNanos =
                        frame.receiptNanos
                                - (long) ((frame.latencySec - params.odomLatencySec) * 1e9);
                Pose2d odoAtCapture = odometryPoseAt(captureNanos);
                Pose2d odoForDatum = odoAtCapture != null ? odoAtCapture : currentOdo;
                bank.observe(branchPoses, branchConf, odoForDatum, spanPx);
                lastFoldedNewFrame = true;
                lastBranchPoses = branchPoses;
            }
        }

        // Commit latch: once the dominant hypothesis is a confident majority, getPose() switches to
        // the bank's MAP and stays there — the bank's own reweighting handles later flips, so this
        // never needs to un-latch (no reseed event).
        if (!committed && bank.dominantWeight() >= params.commitWeight) {
            committed = true;
        }
    }

    /**
     * The 1–2 PnP branch <i>full field poses</i> (inches) for this frame, via {@link
     * VisionPoseSolver#solveBranches}. Null when the tag is unknown or no candidate is usable.
     */
    private Pose2d[] branchPosesFromFrame(VisionFrame frame) {
        if (frame.solTagId < 0 || frame.solBestRvec == null || frame.solBestTvec == null) {
            return null;
        }
        Transform3D[] cands =
                frame.solAltRvec != null && frame.solAltTvec != null
                        ? new Transform3D[] {
                            Transform3D.fromRodrigues(frame.solBestRvec, frame.solBestTvec),
                            Transform3D.fromRodrigues(frame.solAltRvec, frame.solAltTvec)
                        }
                        : new Transform3D[] {
                            Transform3D.fromRodrigues(frame.solBestRvec, frame.solBestTvec)
                        };
        VisionPoseSolver.Branch[] branches = solver.solveBranches(frame.solTagId, cands);
        if (branches == null) {
            return null;
        }
        Pose2d[] poses = new Pose2d[branches.length];
        for (int i = 0; i < branches.length; i++) {
            // Solver works in metres; bank poses are inches.
            poses[i] =
                    new Pose2d(
                            branches[i].fieldFromRobot.x() * M_TO_IN,
                            branches[i].fieldFromRobot.y() * M_TO_IN,
                            new Rotation2d(branches[i].yawRad));
        }
        return poses;
    }

    /**
     * Apparent tag size (px) of the winning tag this frame — {@code sqrt(boxW*boxH)} of its corner
     * bounding box, the regressor for the bank's sigma(span) shoulders. NaN when the corner extents
     * weren't recorded.
     */
    private static double tagSpanPx(VisionFrame frame) {
        double w = frame.tagMaxXPx - frame.tagMinXPx;
        double h = frame.tagMaxYPx - frame.tagMinYPx;
        if (!(w > 0) || !(h > 0)) {
            return Double.NaN;
        }
        return Math.sqrt(w * h);
    }

    /**
     * Per-branch confidence for the bank's ambiguity weighting. {@link #branchPosesFromFrame} orders
     * the branches {@code [best, alt]} (best = lower reprojection), so the best branch gets full
     * confidence and the alternative gets the frame's {@code ambiguity} ratio, floored by {@link
     * PoseHypothesisBank.Params#ambiguityFloor} so the alt is never fully starved. Returns null
     * (equal confidence) when weighting is off, the ambiguity is unknown, or there's only one branch.
     */
    private double[] branchConfFromFrame(VisionFrame frame, int n) {
        PoseHypothesisBank.Params bp = params.bank;
        if (!bp.ambiguityWeightingEnabled || n < 2 || !Double.isFinite(frame.ambiguity)) {
            return null;
        }
        double[] conf = new double[n];
        java.util.Arrays.fill(conf, 1.0);
        conf[1] = Math.max(bp.ambiguityFloor, Math.min(1.0, frame.ambiguity));
        return conf;
    }

    /**
     * The odometry pose as it was at {@code timestampNanos}, interpolated from the retained history,
     * or null if that time is outside the window. Uses WPILib's SE(2) geodesic interpolation.
     */
    private Pose2d odometryPoseAt(long timestampNanos) {
        if (odometryBuffer.isEmpty()) {
            return null;
        }
        if (timestampNanos < odometryBuffer.firstKey() || timestampNanos > odometryBuffer.lastKey()) {
            return null;
        }
        Pose2d exact = odometryBuffer.get(timestampNanos);
        if (exact != null) {
            return exact;
        }
        Map.Entry<Long, Pose2d> lower = odometryBuffer.floorEntry(timestampNanos);
        Map.Entry<Long, Pose2d> upper = odometryBuffer.ceilingEntry(timestampNanos);
        if (lower == null) {
            return upper.getValue();
        }
        if (upper == null) {
            return lower.getValue();
        }
        double t =
                (double) (timestampNanos - lower.getKey())
                        / (double) (upper.getKey() - lower.getKey());
        return lower.getValue().interpolate(upper.getValue(), t);
    }
}
