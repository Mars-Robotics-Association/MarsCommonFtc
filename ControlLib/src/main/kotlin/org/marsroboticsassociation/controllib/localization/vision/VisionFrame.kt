package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d

/**
 * One camera frame's already-extracted features — the seam between vision *acquisition* (turning a
 * Limelight `LLResult` into numbers, or reading them back from a telemetry CSV) and the localizer
 * policy in [HypothesisBankLocalizer]. The policy reads only this struct, so it runs identically
 * against a live camera and against a recording.
 *
 * <p>Plain mutable fields, no logic: it is built once per frame by a [VisionSource] and consumed
 * immediately. Field names and units mirror the diagnostics they feed (and the CSV columns they are
 * logged as), so the mapping in both directions is one-to-one.
 *
 * <p>All the `sol*`/`t6t*`/`t6r*`/`tag*Px`/`focusMetric`/`stddevMt1`/`skew` fields are pure
 * diagnostics the policy does not gate on (except `t6tRs` which drives the sweep-rate computation);
 * they are carried so the replay output CSV reproduces the original schema. A `double[]` diagnostic
 * that was unavailable is left `null`; a scalar is [Double.NaN].
 */
class VisionFrame {

    /**
     * A vision result was present and parseable this loop (LLResult.isValid, or a CSV row exists).
     */
    @JvmField var valid: Boolean = false

    /**
     * Monotonic frame timestamp used only for fresh-frame de-duplication: the policy processes a
     * frame when this differs from the previous one. On a live camera this is the Limelight result
     * timestamp; in replay it is a synthetic counter that advances once per logged `vis_newFrame ==
     * 1` row.
     */
    @JvmField var timestamp: Double = 0.0

    /** Receipt time of this result in the [System.nanoTime] clock domain (back-date anchor). */
    @JvmField var receiptNanos: Long = 0L

    @JvmField var tagCount: Int = 0
    @JvmField var avgDistM: Double = Double.NaN

    /** Total pipeline latency (capture + targeting), seconds. */
    @JvmField var latencySec: Double = Double.NaN

    /** MegaTag1 botpose (field inches, heading radians), or null when no botpose is present. */
    @JvmField var mt1Pose: Pose2d? = null

    /** Out-of-plane DOF of the raw botpose the 2D fusion discards (height in, roll/pitch deg). */
    @JvmField var visionZIn: Double = Double.NaN

    @JvmField var visionRollDeg: Double = Double.NaN
    @JvmField var visionPitchDeg: Double = Double.NaN

    /** Raw, un-consumed MegaTag2 botpose (field inches, heading radians), or null. */
    @JvmField var mt2Pose: Pose2d? = null

    /** Pose PnP ambiguity (our own re-solve), NaN when uncomputable. Higher = more flip-prone. */
    @JvmField var ambiguity: Double = Double.NaN

    /**
     * Absolute reprojection error (px) of the winning tag's best PnP solution — a prior-free
     * detection-quality signal (rises with blur / occlusion / far / oblique), distinct from the
     * [ambiguity] ratio. NaN when uncomputable (no intrinsics/corners) or in replay (the raw
     * corners aren't logged, so it can't be re-solved — it round-trips from the recorded column).
     */
    @JvmField var reprojErrPx: Double = Double.NaN

    // --- Pure diagnostics (carried for CSV schema parity; only t6tRs feeds the policy, via sweep)
    // ---
    @JvmField var focusMetric: Double = Double.NaN
    @JvmField var stddevMt1: DoubleArray? = null // LL MT1 pose stddev [x,y,z,roll,pitch,yaw]
    @JvmField var skew: Double = Double.NaN
    @JvmField var solTagId: Int = -1
    @JvmField var solBestRvec: DoubleArray? = null
    @JvmField var solBestTvec: DoubleArray? = null
    @JvmField var solAltRvec: DoubleArray? = null
    @JvmField var solAltTvec: DoubleArray? = null
    @JvmField var t6tCs: DoubleArray? = null // target-in-camera [x,y,z(m),yaw,pitch,roll(deg)]
    @JvmField
    var t6tRs: DoubleArray? = null // target-in-robot   (drives the sweep-rate computation)
    @JvmField var t6rFs: DoubleArray? = null // robot-in-field
    @JvmField var tagTxDeg: Double = Double.NaN
    @JvmField var tagTyDeg: Double = Double.NaN
    @JvmField var tagMinXPx: Double = Double.NaN
    @JvmField var tagMaxXPx: Double = Double.NaN
    @JvmField var tagMinYPx: Double = Double.NaN
    @JvmField var tagMaxYPx: Double = Double.NaN
    // ALL AprilTags detected this frame (not just the winning one): parallel arrays of fiducial id
    // and that tag's 4 image corners flattened [x0,y0,x1,y1,x2,y2,x3,y3] (px). Null when no
    // detection. These are the raw per-landmark observations a multi-tag SLAM / factor-graph
    // optimizes over; with intrinsics + distortion + the tag map the whole PnP front-end is
    // replayable off-robot. The winning tag (which drove sol_*/ambiguity) is the entry whose id
    // equals solTagId.
    @JvmField var allTagIds: IntArray? = null
    @JvmField var allTagCorners: Array<DoubleArray>? = null
}
