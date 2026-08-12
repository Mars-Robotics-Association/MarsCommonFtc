package org.marsroboticsassociation.controllib.localization.vision

/**
 * The vision-acquisition seam consumed by [HypothesisBankLocalizer]: it turns "whatever the camera
 * produced this loop" into a [VisionFrame] of extracted features, so the localizer policy never
 * touches a hardware type. Implementations live in the host project (the quickstart's
 * `LimelightVisionSource` wraps a live `Limelight3A`; a replay `CsvVisionSource` reconstructs a
 * [VisionFrame] from a recorded telemetry row so the exact production gating + fusion runs
 * off-robot).
 */
interface VisionSource {

    /**
     * The current frame's extracted features. Never null — a [VisionFrame] with `valid = false`
     * means "no usable result this loop". The same frame may be returned across several loops (the
     * control loop runs faster than the camera); the policy de-duplicates on
     * [VisionFrame.timestamp].
     */
    fun latest(): VisionFrame

    /** Feed the current robot heading back to the source (e.g. the Limelight's MegaTag2 seed). */
    fun updateRobotOrientation(headingDeg: Double)

    /**
     * Load any up-front calibration the source needs (the live source fetches camera intrinsics for
     * the ambiguity PnP). Returns true when ready; a source that needs none returns true
     * immediately.
     */
    fun prefetchCalibration(timeoutMs: Long): Boolean

    /**
     * Focal length `fx` of the loaded camera-calibration matrix, or [Double.NaN] until the
     * intrinsics are read. A persistent proof in the log that calibration loaded; the replay source
     * passes the recorded value straight through.
     */
    fun getCalFx(): Double

    /**
     * The remaining camera-intrinsic terms `fy`, `cx`, `cy` (NaN until loaded). Logged alongside
     * `fx` so an offline harness has the full matrix to re-run the PnP solve from the logged tag
     * corners; the replay source passes the recorded values straight through.
     */
    fun getCalFy(): Double

    fun getCalCx(): Double

    fun getCalCy(): Double

    /**
     * The camera distortion coefficients actually passed to the on-robot PnP solve (OpenCV order
     * `[k1,k2,p1,p2,k3,...]`), or null until loaded. Logged so an offline re-solve matches the
     * robot exactly rather than assuming zero distortion; the replay source passes them through.
     */
    fun getCalDistCoeffs(): DoubleArray?
}
