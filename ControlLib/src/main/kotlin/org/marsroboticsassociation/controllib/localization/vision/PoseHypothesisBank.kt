package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import java.util.ArrayList
import java.util.Collections
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Pure, host-agnostic multi-hypothesis pose core. Where a single-pose localizer commits to one IPPE
 * branch and throws the other away, this bank refuses to choose: it keeps both branches alive as
 * parallel weighted hypotheses and lets evidence accumulated over time prune the wrong one, only
 * once the sensors actually separate them.
 *
 * <h3>State = a bank of weighted datums, not poses</h3>
 *
 * The load-bearing representation choice: a hypothesis is its **datum** `T = fieldPose ∘ odoPose⁻¹`
 * — the unknown rigid SE(2) offset between the odometry frame and the field — *not* a field pose.
 * Everything good falls out of that:
 * <ul>
 * <li><b>Predict is free and exact.</b> The true datum is constant (odometry deltas are the
 *   reliable rigid backbone), so a hypothesis needs no propagation between frames — its pose at any
 *   instant is just `T ∘ odoNow` ([mapPose]). There is no predict step to grow covariance on;
 *   odometry motion is absorbed exactly.
 * <li><b>"Never guess while parked" falls out.</b> Each vision frame yields, per IPPE branch, an
 *   *implied datum* `branchPose ∘ odo⁻¹`. Parked, the mirror branch's implied datum is just as
 *   constant as the true one, so the mirror hypothesis keeps matching its own observation and its
 *   weight never decays — the two stay tied. No motion ⇒ no commit, with no special-case gate.
 * <li><b>Recovery is reseed-free.</b> Under motion the mirror datum drifts (heading drifts under
 *   rotation, translation under translation), so a persistent mirror hypothesis stops matching
 *   incoming implied datums and decays, while the true datum recurs and accumulates. A wrong commit
 *   self-corrects: the true hypothesis is still in the bank, so its weight reasserts once motion
 *   separates them — no discrete reseed event, no downstream guard defending a bad seed.
 * </ul>
 *
 * <h3>Per-frame update (Gaussian-sum / IMM in datum space)</h3>
 *
 * [observe] does, per vision frame:
 * <ol>
 * <li><b>Reweight.</b> Each existing hypothesis is multiplied by the likelihood of its nearest
 *   implied datum this frame, `exp(-½·min(r², gate²))` with `r` the combined heading/position datum
 *   residual in σ units. The gate makes the penalty redescending: a hypothesis the data has moved
 *   away from decays at a bounded geometric rate per frame (so it can recover — reseed-free),
 *   rather than being annihilated by one outlier.
 * <li><b>Birth.</b> Any implied datum not already explained by an existing hypothesis spawns a new
 *   one at low weight. This is what keeps the mirror alive at all times, not just at startup.
 * <li><b>Merge + cap.</b> Near-duplicate datums are merged (weight-summed, datum weight-averaged)
 *   and the bank is pruned below a weight floor and capped at `maxHypotheses`, so the 2ᵏ split
 *   growth stays bounded.
 * </ol>
 *
 * <p>Consumers read [mapPose] (the dominant hypothesis applied to current odometry) and
 * [dominantWeight] (how committed the bank is — a value near `1/size` means unresolved ambiguity,
 * near 1 means a confident commit). The alternative stays in the bank until the evidence kills it.
 *
 * <h3>Purity</h3>
 *
 * JDK + WPILib [Pose2d] only — no Android, no hardware, no `@Config`, no clock, no I/O.
 * Deterministic and unit-tested in isolation (`PoseHypothesisBankTest`). It works in inches and
 * radians (`field = T ∘ odo`). Wiring to live intake/consumers is a separate adapter
 * ([HypothesisBankLocalizer]); this is the engine only.
 */
class PoseHypothesisBank(private val params: Params) {

    /** Tuning for the bank. All distances in inches, angles in radians. */
    class Params {
        /**
         * Heading scale (σ) of the datum residual — how far off in heading still counts as a match.
         */
        @JvmField var datumHeadingSigmaRad: Double = Math.toRadians(4.0)

        /** Position scale (σ) of the datum residual (inches). */
        @JvmField var datumPosSigmaIn: Double = 4.0

        /**
         * Redescending gate on the per-frame residual, in σ units. A hypothesis whose nearest
         * implied datum is beyond this is charged a fixed `exp(-½·gate²)` per frame (geometric
         * decay) rather than an unbounded one — so a lone outlier can't annihilate an
         * otherwise-good hypothesis, and a decayed-but-correct one can recover.
         */
        @JvmField var residualGate: Double = 3.0

        /** Two datums within both tolerances are merged (heading). */
        @JvmField var mergeHeadingTolRad: Double = Math.toRadians(8.0)

        /** Two datums within both tolerances are merged (position, inches). */
        @JvmField var mergePosTolIn: Double = 4.0

        /** Weight (relative to the bank's seed weight 1.0) of a newly born hypothesis. */
        @JvmField var birthWeight: Double = 0.15

        /** Prune any hypothesis whose normalized weight falls below this. */
        @JvmField var minWeight: Double = 0.01

        /** Hard cap on the number of hypotheses kept (top-weighted survive). */
        @JvmField var maxHypotheses: Int = 12

        // --- Distance / apparent-size weighting (the sigma(span) shoulders) ----------------------
        // Vision quality falls off nonlinearly with range; the governing variable is the tag's
        // apparent size in pixels (range, tag size, viewing angle all collapse into corner-pixel
        // span). Above the FINE knee a frame is trusted at the base sigma; below the USELESS knee
        // its
        // sigma is inflated to farSigmaInflation x base (near-uniform likelihood -> it barely
        // votes)
        // and its births are suppressed (so far garbage can't spawn confident hypotheses); the iffy
        // zone between interpolates. Keyed on the frame's own apparent size, never on the held
        // estimate, so it does not privilege the incumbent (no re-latching). Knees are
        // camera/field-dependent; calibrate from the logged datum residual. NaN span (size
        // unavailable) disables weighting for that frame (inflation 1, full births).

        /** Master switch for apparent-size weighting; off => fixed base sigma, full births. */
        @JvmField var spanWeightingEnabled: Boolean = true

        /** Apparent tag size (px) at/above which a frame is fully trusted (base sigma). */
        @JvmField var fineKneePx: Double = 130.0

        /** Apparent tag size (px) at/below which sigma is maxed and births are suppressed. */
        @JvmField var uselessKneePx: Double = 60.0

        /** Sigma multiplier at/below the useless knee (>=1): how far a far frame is discounted. */
        @JvmField var farSigmaInflation: Double = 9.0

        /**
         * Heading-sigma multiplier at/below the useless knee (>=1), split from [farSigmaInflation]
         * (which governs position only) because the two channels degrade differently with range:
         * position error scales with range while heading error carries no range multiplier. Kept at
         * parity with the position value (9.0) by default — see the project history for the sweep
         * that measured splitting it apart as net-negative between branches.
         */
        @JvmField var farSigmaInflationHead: Double = 9.0

        // --- Per-frame ambiguity weighting (single-frame branch confidence) ----------------------
        // The motion-only reweight discards the firmware's per-frame branch-quality signal (the PnP
        // reprojection separation / ambiguity ratio). Folding it back in lets a confident frame —
        // one branch fitting far better, the close low-ambiguity case — favor that branch, so the
        // bank can commit even while stationary (where motion can't separate the mirror), while a
        // genuinely ambiguous (far / head-on) frame stays motion-reliant. The caller
        // ([HypothesisBankLocalizer]) derives the per-branch confidence from these knobs and
        // passes it to observe(); the bank consumes the confidence directly.

        /** Master switch for per-frame ambiguity weighting (else equal-confidence, motion-only). */
        @JvmField var ambiguityWeightingEnabled: Boolean = true

        /**
         * Floor on a non-best branch's confidence (its ambiguity ratio is clamped up to this). Caps
         * how hard a single confident frame favors its better branch — and keeps the alternative
         * alive rather than instantly starved, so a confidently-wrong commit can still
         * self-correct.
         */
        @JvmField var ambiguityFloor: Double = 0.15
    }

    /**
     * One hypothesis: a datum `T` (the field←odometry rigid offset) and its current normalized
     * weight. The field pose it asserts right now is `composeDatumOnOdo(datum, odoNow)`.
     */
    class Hypothesis
    internal constructor(
        private val datumPose: Pose2d,
        weight: Double,
    ) {
        private var weightValue: Double = weight

        /** The datum `T` such that `fieldPose = T ∘ odoPose`. */
        fun datum(): Pose2d = datumPose

        /** Normalized weight in `(0, 1]`; the bank's weights sum to 1. */
        fun weight(): Double = weightValue

        internal fun setWeight(w: Double) {
            weightValue = w
        }

        internal fun multiplyWeight(factor: Double) {
            weightValue *= factor
        }
    }

    private val hyps = ArrayList<Hypothesis>()

    // Per-observe diagnostics for in-situ self-calibration: the most recent frame's datum residual
    // against the dominant hypothesis (the innovation, computed BEFORE the frame updates the bank),
    // its apparent size, and the sigma inflation applied. NaN until the first observe with a
    // dominant hypothesis. Logged so sigma(span) can be tuned from the bank's own residuals.
    private var lastResidPosIn: Double = Double.NaN
    private var lastResidHeadDeg: Double = Double.NaN
    private var lastSpanPx: Double = Double.NaN
    private var lastSigmaInflation: Double = Double.NaN

    /**
     * Fold one vision frame into the bank: reweight existing hypotheses by their nearest implied
     * datum, birth new ones for unexplained datums, merge, prune, cap, renormalize. A null/empty
     * `branchPoses` (a frame with no usable vision) is a no-op — the bank simply rides odometry, no
     * propagation needed.
     *
     * @param branchPoses the 1–2 IPPE branch field poses for this frame (x,y in inches), already
     *   floor-admissibility filtered by the caller; null entries are skipped
     * @param odoPose the odometry pose at this frame (the unknown-datum relative pose)
     */
    fun observe(branchPoses: Array<out Pose2d?>?, odoPose: Pose2d) {
        observe(branchPoses, odoPose, Double.NaN)
    }

    /**
     * As [observe] but with the frame's apparent tag size (corner-pixel span). The span drives the
     * sigma(span) shoulders — a far/small-tag frame is discounted toward a near-uniform likelihood
     * and its births suppressed (see [Params]). Pass [Double.NaN] when size is unavailable
     * (weighting disabled for that frame).
     *
     * @param tagSpanPx apparent tag size in pixels (e.g. `sqrt(boxW*boxH)`), or NaN
     */
    fun observe(branchPoses: Array<out Pose2d?>?, odoPose: Pose2d, tagSpanPx: Double) {
        observe(branchPoses, null, odoPose, tagSpanPx)
    }

    /**
     * As the span overload but with a per-branch confidence — the firmware's single-frame
     * branch-quality signal (reprojection separation / PnP ambiguity), which the motion-only
     * reweight otherwise discards. A confident frame (one branch fits far better) favors that
     * branch in *both* the reweight (a confidence-weighted mixture over branches, not just the
     * nearest) and the births, so the bank can **commit while stationary** when the geometry is
     * unambiguous — the close, low-ambiguity case where MegaTag1 is plainly correct. An ambiguous
     * frame (branches fit nearly equally) leaves confidences ~equal, so it stays motion-reliant and
     * order-immune. `branchConf` is parallel to `branchPoses` (null entries skipped with their
     * pose); null ⇒ all-equal confidence (pure motion separation).
     *
     * @param branchConf relative confidence per branch (e.g. 1.0 for the lower-reprojection branch,
     *   the ambiguity ratio for the other), or null for equal confidence
     */
    fun observe(
        branchPoses: Array<out Pose2d?>?,
        branchConf: DoubleArray?,
        odoPose: Pose2d,
        tagSpanPx: Double,
    ) {
        if (branchPoses == null) {
            return
        }
        // Implied datum + confidence per usable branch: T = branchPose ∘ odo⁻¹.
        val implied = ArrayList<Pose2d>(2)
        val conf = ArrayList<Double>(2)
        for (i in branchPoses.indices) {
            if (branchPoses[i] != null) {
                implied.add(impliedDatum(branchPoses[i]!!, odoPose))
                conf.add(if (branchConf != null && i < branchConf.size) branchConf[i] else 1.0)
            }
        }
        if (implied.isEmpty()) {
            return
        }

        // Apparent-size weighting: inflate sigma (and damp births) for far/small-tag frames.
        // Split per channel — position degrades with range, heading barely does (see Params).
        val infl = sigmaInflation(tagSpanPx, params.farSigmaInflation)
        val sigH =
            params.datumHeadingSigmaRad * sigmaInflation(tagSpanPx, params.farSigmaInflationHead)
        val sigP = params.datumPosSigmaIn * infl
        val gate2 = params.residualGate * params.residualGate

        // Diagnostic innovation: this frame's residual vs the dominant hypothesis, captured before
        // it updates the bank (for logging / sigma(span) self-calibration). Uses base sigmas so the
        // logged residual is a stable quality measure independent of the inflation just applied.
        recordResidual(implied, tagSpanPx, infl)

        // 1. Reweight: multiply each hypothesis by a confidence-weighted mixture over the branches,
        // Σⱼ confⱼ·exp(-½·min(rⱼ², gate²)). With equal confidence this ≈ the nearest-branch
        // likelihood (the branches sit at well-separated datums), so motion-only behavior is
        // preserved; with a confident frame the better branch's term dominates, so the hypothesis
        // matching it outgrows its mirror every frame — committing even with no motion.
        for (h in hyps) {
            var like = 0.0
            for (j in implied.indices) {
                val r2 = residualSq(h.datum(), implied[j], sigH, sigP)
                val g = exp(-0.5 * minOf(r2, gate2))
                like += conf[j] * g
            }
            h.multiplyWeight(like)
        }

        // 2. Birth: any implied datum not already explained by a hypothesis seeds a new one, its
        // weight scaled by apparent size (far suppressed) AND by branch confidence (so a confident
        // frame seeds its better branch heavier). Empty bank (bootstrap) seeds at unit weight ×
        // conf.
        val bootstrap = hyps.isEmpty()
        val birthW = params.birthWeight * birthFactor(tagSpanPx)
        for (j in implied.indices) {
            val d = implied[j]
            val w = (if (bootstrap) 1.0 else birthW) * conf[j]
            if (!matchesExisting(d) && (bootstrap || w > 0.0)) {
                hyps.add(Hypothesis(d, w))
            }
        }

        // 3. Merge near-duplicate datums, 4. normalize, 5. prune + cap.
        merge()
        normalize()
        pruneAndCap()
        normalize()
    }

    /**
     * The dominant (MAP) hypothesis's field pose applied to `odoPose`, or `null` if the bank is
     * empty (no vision seen yet).
     */
    fun mapPose(odoPose: Pose2d): Pose2d? {
        val best = dominant()
        return if (best == null) null else composeDatumOnOdo(best.datum(), odoPose)
    }

    /** The dominant hypothesis's datum, or `null` if the bank is empty. */
    fun mapDatum(): Pose2d? {
        val best = dominant()
        return best?.datum()
    }

    /**
     * Weight of the dominant hypothesis in `(0, 1]`: near `1/size()` means the bank is undecided
     * (ambiguity unresolved — e.g. parked), near 1 means a confident commit. The honest "how sure
     * am I which branch" scalar.
     */
    fun dominantWeight(): Double {
        val best = dominant()
        return best?.weight() ?: 0.0
    }

    /** Number of live hypotheses. */
    fun size(): Int = hyps.size

    /**
     * Position residual (in) of the most recent frame's nearest branch against the dominant
     * hypothesis — the pre-update innovation, for logging / sigma(span) self-calibration. NaN
     * before the first frame with a dominant hypothesis.
     */
    fun lastResidPosIn(): Double = lastResidPosIn

    /** Heading residual (deg) counterpart of [lastResidPosIn]. */
    fun lastResidHeadDeg(): Double = lastResidHeadDeg

    /** Apparent tag size (px) of the most recent frame, or NaN if none/unavailable. */
    fun lastSpanPx(): Double = lastSpanPx

    /** σ inflation applied to the most recent frame by the apparent-size shoulders (1 = none). */
    fun lastSigmaInflation(): Double = lastSigmaInflation

    /** Immutable snapshot of the live hypotheses (for diagnostics/tests). */
    fun hypotheses(): List<Hypothesis> = Collections.unmodifiableList(ArrayList(hyps))

    /**
     * Seed a hypothesis directly (for tests, or to inject a known prior such as a
     * deliberately-wrong latch to exercise reseed-free recovery). Weight is renormalized into the
     * bank.
     */
    fun seed(datum: Pose2d, weight: Double) {
        hyps.add(Hypothesis(datum, weight))
        normalize()
    }

    // --------------------------------------------------------------------------------------------
    // Internals
    // --------------------------------------------------------------------------------------------

    private fun dominant(): Hypothesis? {
        var best: Hypothesis? = null
        for (h in hyps) {
            if (best == null || h.weight() > best.weight()) {
                best = h
            }
        }
        return best
    }

    /** Combined heading/position datum residual², in σ units (Mahalanobis with diagonal scales). */
    private fun residualSq(a: Pose2d, b: Pose2d, sigH: Double, sigP: Double): Double {
        val dh = headingDelta(a.rotation.radians, b.rotation.radians) / sigH
        val dp = hypot(a.x - b.x, a.y - b.y) / sigP
        return dh * dh + dp * dp
    }

    /**
     * σ multiplier for a frame of apparent size `spanPx`: 1 above the fine knee, `maxInflation`
     * at/below the useless knee, linear between. NaN span (or weighting off) ⇒ 1. Callers pass
     * [Params.farSigmaInflation] (position) or [Params.farSigmaInflationHead] (heading) — the two
     * channels degrade differently with range.
     */
    private fun sigmaInflation(spanPx: Double, maxInflation: Double): Double {
        if (!params.spanWeightingEnabled || spanPx.isNaN()) {
            return 1.0
        }
        if (spanPx >= params.fineKneePx) {
            return 1.0
        }
        if (spanPx <= params.uselessKneePx || params.fineKneePx <= params.uselessKneePx) {
            return maxInflation
        }
        val f = (params.fineKneePx - spanPx) / (params.fineKneePx - params.uselessKneePx)
        return 1.0 + f * (maxInflation - 1.0)
    }

    /**
     * Birth-weight scale for a frame of apparent size `spanPx`: 1 above the fine knee, 0 at/below
     * the useless knee (births suppressed), linear between. NaN span / weighting off ⇒ 1.
     */
    private fun birthFactor(spanPx: Double): Double {
        if (!params.spanWeightingEnabled || spanPx.isNaN()) {
            return 1.0
        }
        if (spanPx >= params.fineKneePx) {
            return 1.0
        }
        if (spanPx <= params.uselessKneePx || params.fineKneePx <= params.uselessKneePx) {
            return 0.0
        }
        return (spanPx - params.uselessKneePx) / (params.fineKneePx - params.uselessKneePx)
    }

    /** Stash this frame's residual against the dominant hypothesis (the pre-update innovation). */
    private fun recordResidual(implied: List<Pose2d>, spanPx: Double, infl: Double) {
        lastSpanPx = spanPx
        lastSigmaInflation = infl
        val dom = dominant()
        if (dom == null) {
            lastResidPosIn = Double.NaN
            lastResidHeadDeg = Double.NaN
            return
        }
        var best = Double.POSITIVE_INFINITY
        var bp = Double.NaN
        var bh = Double.NaN
        for (d in implied) {
            val comb =
                residualSq(
                    dom.datum(),
                    d,
                    params.datumHeadingSigmaRad,
                    params.datumPosSigmaIn,
                )
            if (comb < best) {
                best = comb
                bp = hypot(dom.datum().x - d.x, dom.datum().y - d.y)
                bh =
                    Math.toDegrees(
                        abs(
                            headingDelta(
                                dom.datum().rotation.radians,
                                d.rotation.radians,
                            )
                        )
                    )
            }
        }
        lastResidPosIn = bp
        lastResidHeadDeg = bh
    }

    /** Whether some existing hypothesis is within the merge tolerance of datum `d`. */
    private fun matchesExisting(d: Pose2d): Boolean {
        for (h in hyps) {
            if (withinMergeTol(h.datum(), d)) {
                return true
            }
        }
        return false
    }

    private fun withinMergeTol(a: Pose2d, b: Pose2d): Boolean {
        return abs(headingDelta(a.rotation.radians, b.rotation.radians)) <=
            params.mergeHeadingTolRad && hypot(a.x - b.x, a.y - b.y) <= params.mergePosTolIn
    }

    /**
     * Greedily merge hypotheses within the merge tolerance: heaviest absorbs its neighbours,
     * summing weights and weight-averaging the datum (circular mean heading). Keeps the 2ᵏ split
     * bounded.
     */
    private fun merge() {
        hyps.sortWith { p, q -> q.weight().compareTo(p.weight()) }
        val merged = ArrayList<Hypothesis>()
        val taken = BooleanArray(hyps.size)
        for (i in hyps.indices) {
            if (taken[i]) {
                continue
            }
            var wSum = hyps[i].weight()
            var sx = hyps[i].datum().x * wSum
            var sy = hyps[i].datum().y * wSum
            var sSin = sin(hyps[i].datum().rotation.radians) * wSum
            var sCos = cos(hyps[i].datum().rotation.radians) * wSum
            for (j in i + 1 until hyps.size) {
                if (taken[j] || !withinMergeTol(hyps[i].datum(), hyps[j].datum())) {
                    continue
                }
                taken[j] = true
                val w = hyps[j].weight()
                wSum += w
                sx += hyps[j].datum().x * w
                sy += hyps[j].datum().y * w
                sSin += sin(hyps[j].datum().rotation.radians) * w
                sCos += cos(hyps[j].datum().rotation.radians) * w
            }
            val datum = Pose2d(sx / wSum, sy / wSum, Rotation2d(atan2(sSin / wSum, sCos / wSum)))
            merged.add(Hypothesis(datum, wSum))
        }
        hyps.clear()
        hyps.addAll(merged)
    }

    private fun normalize() {
        var total = 0.0
        for (h in hyps) {
            total += h.weight()
        }
        if (total <= 0.0) {
            return
        }
        for (h in hyps) {
            h.setWeight(h.weight() / total)
        }
    }

    /** Drop hypotheses below the weight floor, then cap to the heaviest `maxHypotheses`. */
    private fun pruneAndCap() {
        for (i in hyps.size - 1 downTo 0) {
            if (hyps[i].weight() < params.minWeight) {
                hyps.removeAt(i)
            }
        }
        if (hyps.size > params.maxHypotheses) {
            hyps.sortWith { p, q -> q.weight().compareTo(p.weight()) }
            hyps.subList(params.maxHypotheses, hyps.size).clear()
        }
    }

    companion object {
        // --- datum algebra (self-contained) -----------------------------------------------------

        /** Smallest signed angle `a − b`, wrapped to (−π, π]. */
        @JvmStatic
        internal fun headingDelta(a: Double, b: Double): Double {
            var d = a - b
            while (d > PI) d -= 2 * PI
            while (d <= -PI) d += 2 * PI
            return d
        }

        /** Datum implied by a field pose riding an odometry pose: `T = field ∘ odo⁻¹`. */
        @JvmStatic
        fun impliedDatum(field: Pose2d, odo: Pose2d): Pose2d {
            val th = headingDelta(field.rotation.radians, odo.rotation.radians)
            val c = cos(th)
            val s = sin(th)
            return Pose2d(
                field.x - (c * odo.x - s * odo.y),
                field.y - (s * odo.x + c * odo.y),
                Rotation2d(th),
            )
        }

        /** Apply an SE(2) datum to an odometry pose: `datum ∘ odo` (the field pose it implies). */
        @JvmStatic
        fun composeDatumOnOdo(datum: Pose2d, odo: Pose2d): Pose2d {
            val th = datum.rotation.radians
            val c = cos(th)
            val s = sin(th)
            return Pose2d(
                c * odo.x - s * odo.y + datum.x,
                s * odo.x + c * odo.y + datum.y,
                Rotation2d(headingDelta(odo.rotation.radians + th, 0.0)),
            )
        }
    }
}
