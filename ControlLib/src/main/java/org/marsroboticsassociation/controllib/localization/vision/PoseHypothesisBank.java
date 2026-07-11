package org.marsroboticsassociation.controllib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Pure, host-agnostic multi-hypothesis pose core. Where a single-pose localizer commits to one IPPE
 * branch and throws the other away, this bank refuses to choose: it keeps both branches alive as
 * parallel weighted hypotheses and lets evidence accumulated over time prune the wrong one, only
 * once the sensors actually separate them.
 *
 * <h3>State = a bank of weighted datums, not poses</h3>
 *
 * The load-bearing representation choice: a hypothesis is its <b>datum</b> {@code T = fieldPose ∘
 * odoPose⁻¹} — the unknown rigid SE(2) offset between the odometry frame and the field — <i>not</i>
 * a field pose. Everything good falls out of that:
 *
 * <ul>
 *   <li><b>Predict is free and exact.</b> The true datum is constant (odometry deltas are the
 *       reliable rigid backbone), so a hypothesis needs no propagation between frames — its pose at
 *       any instant is just {@code T ∘ odoNow} ({@link #mapPose}). There is no predict step to grow
 *       covariance on; odometry motion is absorbed exactly.
 *   <li><b>"Never guess while parked" falls out.</b> Each vision frame yields, per IPPE branch, an
 *       <i>implied datum</i> {@code branchPose ∘ odo⁻¹}. Parked, the mirror branch's implied datum
 *       is just as constant as the true one, so the mirror hypothesis keeps matching its own
 *       observation and its weight never decays — the two stay tied. No motion ⇒ no commit, with no
 *       special-case gate.
 *   <li><b>Recovery is reseed-free.</b> Under motion the mirror datum drifts (heading drifts under
 *       rotation, translation under translation), so a persistent mirror hypothesis stops matching
 *       incoming implied datums and decays, while the true datum recurs and accumulates. A wrong
 *       commit self-corrects: the true hypothesis is still in the bank, so its weight reasserts once
 *       motion separates them — no discrete reseed event, no downstream guard defending a bad seed.
 * </ul>
 *
 * <h3>Per-frame update (Gaussian-sum / IMM in datum space)</h3>
 *
 * {@link #observe} does, per vision frame:
 *
 * <ol>
 *   <li><b>Reweight.</b> Each existing hypothesis is multiplied by the likelihood of its nearest
 *       implied datum this frame, {@code exp(-½·min(r², gate²))} with {@code r} the combined
 *       heading/position datum residual in σ units. The gate makes the penalty redescending: a
 *       hypothesis the data has moved away from decays at a bounded geometric rate per frame (so it
 *       can recover — reseed-free), rather than being annihilated by one outlier.
 *   <li><b>Birth.</b> Any implied datum not already explained by an existing hypothesis spawns a
 *       new one at low weight. This is what keeps the mirror alive at all times, not just at
 *       startup.
 *   <li><b>Merge + cap.</b> Near-duplicate datums are merged (weight-summed, datum weight-averaged)
 *       and the bank is pruned below a weight floor and capped at {@code maxHypotheses}, so the 2ᵏ
 *       split growth stays bounded.
 * </ol>
 *
 * <p>Consumers read {@link #mapPose} (the dominant hypothesis applied to current odometry) and
 * {@link #dominantWeight} (how committed the bank is — a value near {@code 1/size} means unresolved
 * ambiguity, near 1 means a confident commit). The alternative stays in the bank until the evidence
 * kills it.
 *
 * <h3>Purity</h3>
 *
 * JDK + WPILib {@link Pose2d} only — no Android, no hardware, no {@code @Config}, no clock, no I/O.
 * Deterministic and unit-tested in isolation ({@code PoseHypothesisBankTest}). It works in inches
 * and radians ({@code field = T ∘ odo}). Wiring to live intake/consumers is a separate adapter
 * ({@link HypothesisBankLocalizer}); this is the engine only.
 */
public final class PoseHypothesisBank {

    /** Tuning for the bank. All distances in inches, angles in radians. */
    public static final class Params {
        /**
         * Heading scale (σ) of the datum residual — how far off in heading still counts as a match.
         */
        public double datumHeadingSigmaRad = Math.toRadians(4.0);

        /** Position scale (σ) of the datum residual (inches). */
        public double datumPosSigmaIn = 4.0;

        /**
         * Redescending gate on the per-frame residual, in σ units. A hypothesis whose nearest
         * implied datum is beyond this is charged a fixed {@code exp(-½·gate²)} per frame
         * (geometric decay) rather than an unbounded one — so a lone outlier can't annihilate an
         * otherwise-good hypothesis, and a decayed-but-correct one can recover.
         */
        public double residualGate = 3.0;

        /** Two datums within both tolerances are merged (heading). */
        public double mergeHeadingTolRad = Math.toRadians(8.0);

        /** Two datums within both tolerances are merged (position, inches). */
        public double mergePosTolIn = 4.0;

        /** Weight (relative to the bank's seed weight 1.0) of a newly born hypothesis. */
        public double birthWeight = 0.15;

        /** Prune any hypothesis whose normalized weight falls below this. */
        public double minWeight = 0.01;

        /** Hard cap on the number of hypotheses kept (top-weighted survive). */
        public int maxHypotheses = 12;

        // --- Distance / apparent-size weighting (the sigma(span) shoulders) ----------------------
        // Vision quality falls off nonlinearly with range; the governing variable is the tag's
        // apparent size in pixels (range, tag size, viewing angle all collapse into corner-pixel
        // span). Above the FINE knee a frame is trusted at the base sigma; below the USELESS knee its
        // sigma is inflated to farSigmaInflation x base (near-uniform likelihood -> it barely votes)
        // and its births are suppressed (so far garbage can't spawn confident hypotheses); the iffy
        // zone between interpolates. Keyed on the frame's own apparent size, never on the held
        // estimate, so it does not privilege the incumbent (no re-latching). Knees are
        // camera/field-dependent; calibrate from the logged datum residual. NaN span (size
        // unavailable) disables weighting for that frame (inflation 1, full births).

        /** Master switch for apparent-size weighting; off => fixed base sigma, full births. */
        public boolean spanWeightingEnabled = true;

        /** Apparent tag size (px) at/above which a frame is fully trusted (base sigma). */
        public double fineKneePx = 130.0;

        /** Apparent tag size (px) at/below which sigma is maxed and births are suppressed. */
        public double uselessKneePx = 60.0;

        /** Sigma multiplier at/below the useless knee (>=1): how far a far frame is discounted. */
        public double farSigmaInflation = 9.0;

        /**
         * Heading-sigma multiplier at/below the useless knee (>=1), split from {@link
         * #farSigmaInflation} (which governs position only) because the two channels degrade
         * differently with range: position error scales with range while heading error carries no
         * range multiplier. Kept at parity with the position value (9.0) by default — see the
         * project history for the sweep that measured splitting it apart as net-negative between
         * branches.
         */
        public double farSigmaInflationHead = 9.0;

        // --- Per-frame ambiguity weighting (single-frame branch confidence) ----------------------
        // The motion-only reweight discards the firmware's per-frame branch-quality signal (the PnP
        // reprojection separation / ambiguity ratio). Folding it back in lets a confident frame —
        // one branch fitting far better, the close low-ambiguity case — favor that branch, so the
        // bank can commit even while stationary (where motion can't separate the mirror), while a
        // genuinely ambiguous (far / head-on) frame stays motion-reliant. The caller
        // ({@link HypothesisBankLocalizer}) derives the per-branch confidence from these knobs and
        // passes it to observe(); the bank consumes the confidence directly.

        /** Master switch for per-frame ambiguity weighting (else equal-confidence, motion-only). */
        public boolean ambiguityWeightingEnabled = true;

        /**
         * Floor on a non-best branch's confidence (its ambiguity ratio is clamped up to this). Caps
         * how hard a single confident frame favors its better branch — and keeps the alternative
         * alive rather than instantly starved, so a confidently-wrong commit can still
         * self-correct.
         */
        public double ambiguityFloor = 0.15;
    }

    /**
     * One hypothesis: a datum {@code T} (the field←odometry rigid offset) and its current
     * normalized weight. The field pose it asserts right now is {@code composeDatumOnOdo(datum,
     * odoNow)}.
     */
    public static final class Hypothesis {
        private final Pose2d datum;
        private double weight;

        private Hypothesis(Pose2d datum, double weight) {
            this.datum = datum;
            this.weight = weight;
        }

        /** The datum {@code T} such that {@code fieldPose = T ∘ odoPose}. */
        public Pose2d datum() {
            return datum;
        }

        /** Normalized weight in {@code (0, 1]}; the bank's weights sum to 1. */
        public double weight() {
            return weight;
        }
    }

    private final Params params;
    private final List<Hypothesis> hyps = new ArrayList<>();

    // Per-observe diagnostics for in-situ self-calibration: the most recent frame's datum residual
    // against the dominant hypothesis (the innovation, computed BEFORE the frame updates the bank),
    // its apparent size, and the sigma inflation applied. NaN until the first observe with a
    // dominant hypothesis. Logged so sigma(span) can be tuned from the bank's own residuals.
    private double lastResidPosIn = Double.NaN;
    private double lastResidHeadDeg = Double.NaN;
    private double lastSpanPx = Double.NaN;
    private double lastSigmaInflation = Double.NaN;

    public PoseHypothesisBank(Params params) {
        this.params = params;
    }

    /**
     * Fold one vision frame into the bank: reweight existing hypotheses by their nearest implied
     * datum, birth new ones for unexplained datums, merge, prune, cap, renormalize. A null/empty
     * {@code branchPoses} (a frame with no usable vision) is a no-op — the bank simply rides
     * odometry, no propagation needed.
     *
     * @param branchPoses the 1–2 IPPE branch field poses for this frame (x,y in inches), already
     *     floor-admissibility filtered by the caller; null entries are skipped
     * @param odoPose the odometry pose at this frame (the unknown-datum relative pose)
     */
    public void observe(Pose2d[] branchPoses, Pose2d odoPose) {
        observe(branchPoses, odoPose, Double.NaN);
    }

    /**
     * As {@link #observe(Pose2d[], Pose2d)} but with the frame's apparent tag size (corner-pixel
     * span). The span drives the sigma(span) shoulders — a far/small-tag frame is discounted toward
     * a near-uniform likelihood and its births suppressed (see {@link Params}). Pass {@link
     * Double#NaN} when size is unavailable (weighting disabled for that frame).
     *
     * @param tagSpanPx apparent tag size in pixels (e.g. {@code sqrt(boxW*boxH)}), or NaN
     */
    public void observe(Pose2d[] branchPoses, Pose2d odoPose, double tagSpanPx) {
        observe(branchPoses, null, odoPose, tagSpanPx);
    }

    /**
     * As {@link #observe(Pose2d[], Pose2d, double)} but with a per-branch confidence — the
     * firmware's single-frame branch-quality signal (reprojection separation / PnP ambiguity),
     * which the motion-only reweight otherwise discards. A confident frame (one branch fits far
     * better) favors that branch in <i>both</i> the reweight (a confidence-weighted mixture over
     * branches, not just the nearest) and the births, so the bank can <b>commit while
     * stationary</b> when the geometry is unambiguous — the close, low-ambiguity case where
     * MegaTag1 is plainly correct. An ambiguous frame (branches fit nearly equally) leaves
     * confidences ~equal, so it stays motion-reliant and order-immune. {@code branchConf} is
     * parallel to {@code branchPoses} (null entries skipped with their pose); null ⇒ all-equal
     * confidence (pure motion separation).
     *
     * @param branchConf relative confidence per branch (e.g. 1.0 for the lower-reprojection branch,
     *     the ambiguity ratio for the other), or null for equal confidence
     */
    public void observe(
            Pose2d[] branchPoses, double[] branchConf, Pose2d odoPose, double tagSpanPx) {
        if (branchPoses == null) {
            return;
        }
        // Implied datum + confidence per usable branch: T = branchPose ∘ odo⁻¹.
        List<Pose2d> implied = new ArrayList<>(2);
        List<Double> conf = new ArrayList<>(2);
        for (int i = 0; i < branchPoses.length; i++) {
            if (branchPoses[i] != null) {
                implied.add(impliedDatum(branchPoses[i], odoPose));
                conf.add(branchConf != null && i < branchConf.length ? branchConf[i] : 1.0);
            }
        }
        if (implied.isEmpty()) {
            return;
        }

        // Apparent-size weighting: inflate sigma (and damp births) for far/small-tag frames.
        // Split per channel — position degrades with range, heading barely does (see Params).
        double infl = sigmaInflation(tagSpanPx, params.farSigmaInflation);
        double sigH =
                params.datumHeadingSigmaRad
                        * sigmaInflation(tagSpanPx, params.farSigmaInflationHead);
        double sigP = params.datumPosSigmaIn * infl;
        double gate2 = params.residualGate * params.residualGate;

        // Diagnostic innovation: this frame's residual vs the dominant hypothesis, captured before
        // it updates the bank (for logging / sigma(span) self-calibration). Uses base sigmas so the
        // logged residual is a stable quality measure independent of the inflation just applied.
        recordResidual(implied, tagSpanPx, infl);

        // 1. Reweight: multiply each hypothesis by a confidence-weighted mixture over the branches,
        // Σⱼ confⱼ·exp(-½·min(rⱼ², gate²)). With equal confidence this ≈ the nearest-branch
        // likelihood (the branches sit at well-separated datums), so motion-only behavior is
        // preserved; with a confident frame the better branch's term dominates, so the hypothesis
        // matching it outgrows its mirror every frame — committing even with no motion.
        for (Hypothesis h : hyps) {
            double like = 0.0;
            for (int j = 0; j < implied.size(); j++) {
                double r2 = residualSq(h.datum, implied.get(j), sigH, sigP);
                double g = Math.exp(-0.5 * Math.min(r2, gate2));
                like += conf.get(j) * g;
            }
            h.weight *= like;
        }

        // 2. Birth: any implied datum not already explained by a hypothesis seeds a new one, its
        // weight scaled by apparent size (far suppressed) AND by branch confidence (so a confident
        // frame seeds its better branch heavier). Empty bank (bootstrap) seeds at unit weight ×
        // conf.
        boolean bootstrap = hyps.isEmpty();
        double birthW = params.birthWeight * birthFactor(tagSpanPx);
        for (int j = 0; j < implied.size(); j++) {
            Pose2d d = implied.get(j);
            double w = (bootstrap ? 1.0 : birthW) * conf.get(j);
            if (!matchesExisting(d) && (bootstrap || w > 0.0)) {
                hyps.add(new Hypothesis(d, w));
            }
        }

        // 3. Merge near-duplicate datums, 4. normalize, 5. prune + cap.
        merge();
        normalize();
        pruneAndCap();
        normalize();
    }

    /**
     * The dominant (MAP) hypothesis's field pose applied to {@code odoPose}, or {@code null} if the
     * bank is empty (no vision seen yet).
     */
    public Pose2d mapPose(Pose2d odoPose) {
        Hypothesis best = dominant();
        return best == null ? null : composeDatumOnOdo(best.datum, odoPose);
    }

    /** The dominant hypothesis's datum, or {@code null} if the bank is empty. */
    public Pose2d mapDatum() {
        Hypothesis best = dominant();
        return best == null ? null : best.datum;
    }

    /**
     * Weight of the dominant hypothesis in {@code (0, 1]}: near {@code 1/size()} means the bank is
     * undecided (ambiguity unresolved — e.g. parked), near 1 means a confident commit. The honest
     * "how sure am I which branch" scalar.
     */
    public double dominantWeight() {
        Hypothesis best = dominant();
        return best == null ? 0.0 : best.weight;
    }

    /** Number of live hypotheses. */
    public int size() {
        return hyps.size();
    }

    /**
     * Position residual (in) of the most recent frame's nearest branch against the dominant
     * hypothesis — the pre-update innovation, for logging / sigma(span) self-calibration. NaN
     * before the first frame with a dominant hypothesis.
     */
    public double lastResidPosIn() {
        return lastResidPosIn;
    }

    /** Heading residual (deg) counterpart of {@link #lastResidPosIn()}. */
    public double lastResidHeadDeg() {
        return lastResidHeadDeg;
    }

    /** Apparent tag size (px) of the most recent frame, or NaN if none/unavailable. */
    public double lastSpanPx() {
        return lastSpanPx;
    }

    /** σ inflation applied to the most recent frame by the apparent-size shoulders (1 = none). */
    public double lastSigmaInflation() {
        return lastSigmaInflation;
    }

    /** Immutable snapshot of the live hypotheses (for diagnostics/tests). */
    public List<Hypothesis> hypotheses() {
        return Collections.unmodifiableList(new ArrayList<>(hyps));
    }

    /**
     * Seed a hypothesis directly (for tests, or to inject a known prior such as a
     * deliberately-wrong latch to exercise reseed-free recovery). Weight is renormalized into the
     * bank.
     */
    public void seed(Pose2d datum, double weight) {
        hyps.add(new Hypothesis(datum, weight));
        normalize();
    }

    // --------------------------------------------------------------------------------------------
    // Internals
    // --------------------------------------------------------------------------------------------

    private Hypothesis dominant() {
        Hypothesis best = null;
        for (Hypothesis h : hyps) {
            if (best == null || h.weight > best.weight) {
                best = h;
            }
        }
        return best;
    }

    /** Combined heading/position datum residual², in σ units (Mahalanobis with diagonal scales). */
    private static double residualSq(Pose2d a, Pose2d b, double sigH, double sigP) {
        double dh = headingDelta(a.getRotation().getRadians(), b.getRotation().getRadians()) / sigH;
        double dp = Math.hypot(a.getX() - b.getX(), a.getY() - b.getY()) / sigP;
        return dh * dh + dp * dp;
    }

    /**
     * σ multiplier for a frame of apparent size {@code spanPx}: 1 above the fine knee, {@code
     * maxInflation} at/below the useless knee, linear between. NaN span (or weighting off) ⇒ 1.
     * Callers pass {@link Params#farSigmaInflation} (position) or {@link
     * Params#farSigmaInflationHead} (heading) — the two channels degrade differently with range.
     */
    private double sigmaInflation(double spanPx, double maxInflation) {
        if (!params.spanWeightingEnabled || Double.isNaN(spanPx)) {
            return 1.0;
        }
        if (spanPx >= params.fineKneePx) {
            return 1.0;
        }
        if (spanPx <= params.uselessKneePx || params.fineKneePx <= params.uselessKneePx) {
            return maxInflation;
        }
        double f = (params.fineKneePx - spanPx) / (params.fineKneePx - params.uselessKneePx);
        return 1.0 + f * (maxInflation - 1.0);
    }

    /**
     * Birth-weight scale for a frame of apparent size {@code spanPx}: 1 above the fine knee, 0
     * at/below the useless knee (births suppressed), linear between. NaN span / weighting off ⇒ 1.
     */
    private double birthFactor(double spanPx) {
        if (!params.spanWeightingEnabled || Double.isNaN(spanPx)) {
            return 1.0;
        }
        if (spanPx >= params.fineKneePx) {
            return 1.0;
        }
        if (spanPx <= params.uselessKneePx || params.fineKneePx <= params.uselessKneePx) {
            return 0.0;
        }
        return (spanPx - params.uselessKneePx) / (params.fineKneePx - params.uselessKneePx);
    }

    /** Stash this frame's residual against the dominant hypothesis (the pre-update innovation). */
    private void recordResidual(List<Pose2d> implied, double spanPx, double infl) {
        lastSpanPx = spanPx;
        lastSigmaInflation = infl;
        Hypothesis dom = dominant();
        if (dom == null) {
            lastResidPosIn = Double.NaN;
            lastResidHeadDeg = Double.NaN;
            return;
        }
        double best = Double.POSITIVE_INFINITY, bp = Double.NaN, bh = Double.NaN;
        for (Pose2d d : implied) {
            double comb =
                    residualSq(dom.datum, d, params.datumHeadingSigmaRad, params.datumPosSigmaIn);
            if (comb < best) {
                best = comb;
                bp = Math.hypot(dom.datum.getX() - d.getX(), dom.datum.getY() - d.getY());
                bh =
                        Math.toDegrees(
                                Math.abs(
                                        headingDelta(
                                                dom.datum.getRotation().getRadians(),
                                                d.getRotation().getRadians())));
            }
        }
        lastResidPosIn = bp;
        lastResidHeadDeg = bh;
    }

    /** Whether some existing hypothesis is within the merge tolerance of datum {@code d}. */
    private boolean matchesExisting(Pose2d d) {
        for (Hypothesis h : hyps) {
            if (withinMergeTol(h.datum, d)) {
                return true;
            }
        }
        return false;
    }

    private boolean withinMergeTol(Pose2d a, Pose2d b) {
        return Math.abs(headingDelta(a.getRotation().getRadians(), b.getRotation().getRadians()))
                        <= params.mergeHeadingTolRad
                && Math.hypot(a.getX() - b.getX(), a.getY() - b.getY()) <= params.mergePosTolIn;
    }

    /**
     * Greedily merge hypotheses within the merge tolerance: heaviest absorbs its neighbours,
     * summing weights and weight-averaging the datum (circular mean heading). Keeps the 2ᵏ split
     * bounded.
     */
    private void merge() {
        hyps.sort((p, q) -> Double.compare(q.weight, p.weight));
        List<Hypothesis> merged = new ArrayList<>();
        boolean[] taken = new boolean[hyps.size()];
        for (int i = 0; i < hyps.size(); i++) {
            if (taken[i]) {
                continue;
            }
            double wSum = hyps.get(i).weight;
            double sx = hyps.get(i).datum.getX() * wSum;
            double sy = hyps.get(i).datum.getY() * wSum;
            double sSin = Math.sin(hyps.get(i).datum.getRotation().getRadians()) * wSum;
            double sCos = Math.cos(hyps.get(i).datum.getRotation().getRadians()) * wSum;
            for (int j = i + 1; j < hyps.size(); j++) {
                if (taken[j] || !withinMergeTol(hyps.get(i).datum, hyps.get(j).datum)) {
                    continue;
                }
                taken[j] = true;
                double w = hyps.get(j).weight;
                wSum += w;
                sx += hyps.get(j).datum.getX() * w;
                sy += hyps.get(j).datum.getY() * w;
                sSin += Math.sin(hyps.get(j).datum.getRotation().getRadians()) * w;
                sCos += Math.cos(hyps.get(j).datum.getRotation().getRadians()) * w;
            }
            Pose2d datum =
                    new Pose2d(sx / wSum, sy / wSum, new Rotation2d(Math.atan2(sSin / wSum, sCos / wSum)));
            merged.add(new Hypothesis(datum, wSum));
        }
        hyps.clear();
        hyps.addAll(merged);
    }

    private void normalize() {
        double total = 0.0;
        for (Hypothesis h : hyps) {
            total += h.weight;
        }
        if (total <= 0.0) {
            return;
        }
        for (Hypothesis h : hyps) {
            h.weight /= total;
        }
    }

    /** Drop hypotheses below the weight floor, then cap to the heaviest {@code maxHypotheses}. */
    private void pruneAndCap() {
        for (int i = hyps.size() - 1; i >= 0; i--) {
            if (hyps.get(i).weight < params.minWeight) {
                hyps.remove(i);
            }
        }
        if (hyps.size() > params.maxHypotheses) {
            hyps.sort((p, q) -> Double.compare(q.weight, p.weight));
            hyps.subList(params.maxHypotheses, hyps.size()).clear();
        }
    }

    // --- datum algebra (self-contained) ---------------------------------------------------------

    /** Smallest signed angle {@code a − b}, wrapped to (−π, π]. */
    static double headingDelta(double a, double b) {
        double d = a - b;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d <= -Math.PI) d += 2 * Math.PI;
        return d;
    }

    /** Datum implied by a field pose riding an odometry pose: {@code T = field ∘ odo⁻¹}. */
    static Pose2d impliedDatum(Pose2d field, Pose2d odo) {
        double th = headingDelta(field.getRotation().getRadians(), odo.getRotation().getRadians());
        double c = Math.cos(th), s = Math.sin(th);
        return new Pose2d(
                field.getX() - (c * odo.getX() - s * odo.getY()),
                field.getY() - (s * odo.getX() + c * odo.getY()),
                new Rotation2d(th));
    }

    /**
     * Apply an SE(2) datum to an odometry pose: {@code datum ∘ odo} (the field pose it implies).
     */
    static Pose2d composeDatumOnOdo(Pose2d datum, Pose2d odo) {
        double th = datum.getRotation().getRadians();
        double c = Math.cos(th), s = Math.sin(th);
        return new Pose2d(
                c * odo.getX() - s * odo.getY() + datum.getX(),
                s * odo.getX() + c * odo.getY() + datum.getY(),
                new Rotation2d(headingDelta(odo.getRotation().getRadians() + th, 0.0)));
    }
}
