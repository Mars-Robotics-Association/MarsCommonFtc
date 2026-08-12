package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import java.util.TreeMap
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Pure, library-agnostic policy core that drives a [PoseHypothesisBank] from a stream of
 * [VisionFrame]s and odometry poses — the host-agnostic half of the multi-hypothesis localizer. It
 * owns the bank, the vision-pose solver, the per-frame gating (motion-blur reject, fresh-frame
 * de-duplication), the latency compensation (fitting each frame's datum against the odometry that
 * was true at the vision *capture* instant), and the commit latch.
 *
 * <p>Everything here is in WPILib [Pose2d] (field inches, radians) with no Road Runner, no FTC SDK,
 * no `@Config`, and no clock of its own — the host adapter supplies the current odometry pose, the
 * loop timestamp (a [System.nanoTime]-domain value), the yaw rate (for the blur gate), and the
 * [VisionFrame]. That keeps the whole gating + fusion path desktop-testable and reusable across
 * drive libraries; a thin adapter binds it to a specific localizer interface.
 *
 * <h3>Usage</h3>
 * <ol>
 * <li>Optionally [setPose] to seed a trusted starting field pose.</li>
 * <li>Call [update] every loop with `(nowNanos, odoPose, yawRateRadPerSec, frame)`.</li>
 * <li>Read [getPose] for the fused field pose, and [isCommitted] / [dominantWeight] for the
 *   commitment level (gate auto-aim on these).</li>
 * </ol>
 *
 * <h3>Pre-commit behavior</h3>
 *
 * Commit is **vision-driven**: [isCommitted] latches only when the bank's dominant weight crosses
 * [Params.commitWeight], i.e. vision has confirmed a branch. Before that, [getPose] rides a seed
 * datum from [setPose] composed with live odometry (the start pose the host handed in), or raw
 * odometry if nothing was seeded. This is the bank-only path: there is no EKF pre-commit fallback —
 * the bank is the whole estimator.
 */
class HypothesisBankLocalizer(private val params: Params, private val solver: VisionPoseSolver) {

    /** Tuning for the policy core (the bank has its own nested [PoseHypothesisBank.Params]). */
    class Params {
        /** The multi-hypothesis bank's own tuning. */
        @JvmField var bank: PoseHypothesisBank.Params = PoseHypothesisBank.Params()

        /**
         * Dominant-hypothesis weight at/above which the bank is "committed": [getPose] switches to
         * the bank's MAP and stays there (the bank's own reweighting handles later flips, so this
         * never needs to un-latch).
         */
        @JvmField var commitWeight: Double = 0.7

        /**
         * Motion-blur reject: frames captured while the robot's yaw rate exceeds this are skipped
         * (a fast pan smears the tag corners and corrupts the PnP). Radians/sec.
         */
        @JvmField var maxYawRateRadPerSec: Double = Math.toRadians(90.0)

        /**
         * Differential-latency correction (seconds): the effective vision capture time is
         * `receiptNanos - (frame.latencySec - odomLatencySec)`. Back-dates vision *less* by the
         * odometry computer's own lag so the datum isn't fit against stale odometry. Measure
         * offline; 0 = full back-date. See the source project's latency-calibration notes.
         */
        @JvmField var odomLatencySec: Double = 0.035

        /** How long odometry history is retained for latency compensation (seconds). */
        @JvmField var odometryHistorySec: Double = 1.5
    }

    private val bank: PoseHypothesisBank = PoseHypothesisBank(params.bank)

    /**
     * Odometry history (nanoTime → field-frame odometry pose) for capture-time latency alignment.
     */
    private val odometryBuffer = TreeMap<Long, Pose2d>()

    private var bankLastSampleTs: Double = Double.NaN
    private var committed: Boolean = false

    // Pre-commit fallback: the datum implied by the last seeded (fieldPose, odoPose). getPose rides
    // this composed with live odometry until vision commits the bank. Null until setPose is called.
    private var fallbackDatum: Pose2d? = null

    // --- Per-update diagnostics (for logging / debugging; see the last* accessors) ---
    private var lastFrame: VisionFrame? = null // the frame handed to the most recent update()
    private var lastYawRateRadPerSec: Double = Double.NaN
    private var lastFoldedNewFrame: Boolean =
        false // did the last update fold a fresh frame into the bank
    private var lastBranchPoses: Array<Pose2d>? =
        null // the branch field poses that update fed the bank, or null

    /**
     * Seed a trusted starting field pose. Sets the pre-commit fallback datum (implied by
     * `(fieldPose, odoPose)`), so [getPose] reports `fieldPose` riding odometry until vision
     * commits the bank. Does **not** mark the localizer committed — commit stays vision-driven —
     * and does not inject a hypothesis, so a wrong prior cannot bias branch selection.
     *
     * @param fieldPose the trusted field pose (inches, radians)
     * @param odoPose the current odometry pose at the same instant
     */
    fun setPose(fieldPose: Pose2d, odoPose: Pose2d) {
        fallbackDatum = PoseHypothesisBank.impliedDatum(fieldPose, odoPose)
    }

    /**
     * Feed one loop of odometry + vision.
     *
     * @param nowNanos the current time in the [System.nanoTime] domain
     * @param odoPose the current field-frame odometry pose (inches, radians)
     * @param yawRateRadPerSec the current yaw rate (for the motion-blur gate)
     * @param frame this loop's vision frame (never null; `valid=false` means no usable result)
     */
    fun update(nowNanos: Long, odoPose: Pose2d, yawRateRadPerSec: Double, frame: VisionFrame) {
        lastFrame = frame
        lastYawRateRadPerSec = yawRateRadPerSec
        lastFoldedNewFrame = false
        lastBranchPoses = null

        odometryBuffer[nowNanos] = odoPose
        val cutoff = nowNanos - (params.odometryHistorySec * 1e9).toLong()
        while (!odometryBuffer.isEmpty() && odometryBuffer.firstKey() < cutoff) {
            odometryBuffer.pollFirstEntry()
        }
        serviceHypothesisBank(frame, yawRateRadPerSec, odoPose)
    }

    /**
     * The fused field pose. Once committed, the dominant hypothesis applied to the current odometry
     * (it rides odometry between frames and self-corrects internally). Before commit, the seed
     * datum from [setPose] riding odometry, or raw odometry if nothing was seeded.
     */
    fun getPose(odoPose: Pose2d): Pose2d {
        if (committed) {
            val mapPose = bank.mapPose(odoPose)
            if (mapPose != null) {
                return mapPose
            }
        }
        val fallback = fallbackDatum
        if (fallback != null) {
            return PoseHypothesisBank.composeDatumOnOdo(fallback, odoPose)
        }
        return odoPose
    }

    /** Whether the bank has committed (dominant weight crossed [Params.commitWeight]). */
    fun isCommitted(): Boolean = committed

    /** Commitment level: dominant-hypothesis weight in `(0,1]` (0 if no vision seen). */
    fun dominantWeight(): Double = bank.dominantWeight()

    /** The underlying bank (diagnostics / advanced use). */
    fun bank(): PoseHypothesisBank = bank

    /** The [VisionFrame] handed to the most recent [update], or null before any. */
    fun lastFrame(): VisionFrame? = lastFrame

    /** Whether the most recent [update] folded a fresh valid frame into the bank. */
    fun wasNewFrameFolded(): Boolean = lastFoldedNewFrame

    /**
     * The 1–2 branch field poses the most recent [update] fed the bank (best-first), or null if no
     * frame was folded — the raw multi-hypothesis input, for debugging branch separation.
     */
    fun lastBranchPoses(): Array<Pose2d>? = lastBranchPoses

    /** The yaw rate passed to the most recent [update] (rad/s), or NaN before any. */
    fun lastYawRateRadPerSec(): Double = lastYawRateRadPerSec

    private fun serviceHypothesisBank(frame: VisionFrame, yawRate: Double, currentOdo: Pose2d) {
        val blurOk = abs(yawRate) <= params.maxYawRateRadPerSec
        if (
            frame.valid &&
                frame.timestamp != bankLastSampleTs &&
                frame.tagCount > 0 &&
                frame.avgDistM > 0 &&
                frame.mt1Pose != null &&
                blurOk
        ) {
            val branchPoses = branchPosesFromFrame(frame)
            if (branchPoses != null) {
                bankLastSampleTs = frame.timestamp
                val spanPx = tagSpanPx(frame)
                val branchConf = branchConfFromFrame(frame, branchPoses.size)
                // Latency compensation (the "rewind"): the branch poses describe the robot at the
                // vision CAPTURE instant, so fit the datum against the odometry that was true THEN,
                // not the current odometry — otherwise the (constant) datum absorbs the odometry
                // motion over the vision latency and jitters with speed. No forward replay is
                // needed
                // because the datum is constant and getPose() already rides live odometry. Falls
                // back to current odometry if the capture time is outside the history window.
                val captureNanos =
                    frame.receiptNanos - ((frame.latencySec - params.odomLatencySec) * 1e9).toLong()
                val odoAtCapture = odometryPoseAt(captureNanos)
                val odoForDatum = odoAtCapture ?: currentOdo
                bank.observe(branchPoses, branchConf, odoForDatum, spanPx)
                lastFoldedNewFrame = true
                lastBranchPoses = branchPoses
            }
        }

        // Commit latch: once the dominant hypothesis is a confident majority, getPose() switches to
        // the bank's MAP and stays there — the bank's own reweighting handles later flips, so this
        // never needs to un-latch (no reseed event).
        if (!committed && bank.dominantWeight() >= params.commitWeight) {
            committed = true
        }
    }

    /**
     * The 1–2 PnP branch *full field poses* (inches) for this frame, via
     * [VisionPoseSolver.solveBranches]. Null when the tag is unknown or no candidate is usable.
     */
    private fun branchPosesFromFrame(frame: VisionFrame): Array<Pose2d>? {
        if (frame.solTagId < 0 || frame.solBestRvec == null || frame.solBestTvec == null) {
            return null
        }
        val bestR = frame.solBestRvec ?: return null
        val bestT = frame.solBestTvec ?: return null
        val cands: Array<Transform3D> =
            if (frame.solAltRvec != null && frame.solAltTvec != null) {
                arrayOf(
                    Transform3D.fromRodrigues(bestR, bestT),
                    Transform3D.fromRodrigues(frame.solAltRvec!!, frame.solAltTvec!!),
                )
            } else {
                arrayOf(Transform3D.fromRodrigues(bestR, bestT))
            }
        val branches = solver.solveBranches(frame.solTagId, cands) ?: return null
        val poses =
            Array(branches.size) { i ->
                // Solver works in metres; bank poses are inches.
                Pose2d(
                    branches[i].fieldFromRobot.x() * M_TO_IN,
                    branches[i].fieldFromRobot.y() * M_TO_IN,
                    Rotation2d(branches[i].yawRad),
                )
            }
        return poses
    }

    /**
     * Apparent tag size (px) of the winning tag this frame — `sqrt(boxW*boxH)` of its corner
     * bounding box, the regressor for the bank's sigma(span) shoulders. NaN when the corner extents
     * weren't recorded.
     */
    private fun tagSpanPx(frame: VisionFrame): Double {
        val w = frame.tagMaxXPx - frame.tagMinXPx
        val h = frame.tagMaxYPx - frame.tagMinYPx
        if (!(w > 0) || !(h > 0)) {
            return Double.NaN
        }
        return sqrt(w * h)
    }

    /**
     * Per-branch confidence for the bank's ambiguity weighting. [branchPosesFromFrame] orders the
     * branches `[best, alt]` (best = lower reprojection), so the best branch gets full confidence
     * and the alternative gets the frame's `ambiguity` ratio, floored by
     * [PoseHypothesisBank.Params.ambiguityFloor] so the alt is never fully starved. Returns null
     * (equal confidence) when weighting is off, the ambiguity is unknown, or there's only one
     * branch.
     */
    private fun branchConfFromFrame(frame: VisionFrame, n: Int): DoubleArray? {
        val bp = params.bank
        if (!bp.ambiguityWeightingEnabled || n < 2 || !frame.ambiguity.isFinite()) {
            return null
        }
        val conf = DoubleArray(n) { 1.0 }
        conf[1] = max(bp.ambiguityFloor, min(1.0, frame.ambiguity))
        return conf
    }

    /**
     * The odometry pose as it was at `timestampNanos`, interpolated from the retained history, or
     * null if that time is outside the window. Uses WPILib's SE(2) geodesic interpolation.
     */
    private fun odometryPoseAt(timestampNanos: Long): Pose2d? {
        if (odometryBuffer.isEmpty()) {
            return null
        }
        if (
            timestampNanos < odometryBuffer.firstKey() || timestampNanos > odometryBuffer.lastKey()
        ) {
            return null
        }
        val exact = odometryBuffer[timestampNanos]
        if (exact != null) {
            return exact
        }
        val lower = odometryBuffer.floorEntry(timestampNanos)
        val upper = odometryBuffer.ceilingEntry(timestampNanos)
        if (lower == null) {
            return upper.value
        }
        if (upper == null) {
            return lower.value
        }
        val t = (timestampNanos - lower.key).toDouble() / (upper.key - lower.key).toDouble()
        return lower.value.interpolate(upper.value, t)
    }

    companion object {
        private const val M_TO_IN = 1.0 / 0.0254
    }
}
