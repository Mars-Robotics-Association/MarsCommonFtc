package org.marsroboticsassociation.controllib.localization.vision;

/**
 * The vision-acquisition seam consumed by {@link HypothesisBankLocalizer}: it turns "whatever the
 * camera produced this loop" into a {@link VisionFrame} of extracted features, so the localizer
 * policy never touches a hardware type. Implementations live in the host project (the quickstart's
 * {@code LimelightVisionSource} wraps a live {@code Limelight3A}; a replay {@code CsvVisionSource}
 * reconstructs a {@link VisionFrame} from a recorded telemetry row so the exact production gating +
 * fusion runs off-robot).
 */
public interface VisionSource {

    /**
     * The current frame's extracted features. Never null — a {@link VisionFrame} with {@code valid
     * = false} means "no usable result this loop". The same frame may be returned across several
     * loops (the control loop runs faster than the camera); the policy de-duplicates on {@link
     * VisionFrame#timestamp}.
     */
    VisionFrame latest();

    /** Feed the current robot heading back to the source (e.g. the Limelight's MegaTag2 seed). */
    void updateRobotOrientation(double headingDeg);

    /**
     * Load any up-front calibration the source needs (the live source fetches camera intrinsics for
     * the ambiguity PnP). Returns true when ready; a source that needs none returns true
     * immediately.
     */
    boolean prefetchCalibration(long timeoutMs);

    /**
     * Focal length {@code fx} of the loaded camera-calibration matrix, or {@link Double#NaN} until
     * the intrinsics are read. A persistent proof in the log that calibration loaded; the replay
     * source passes the recorded value straight through.
     */
    double getCalFx();

    /**
     * The remaining camera-intrinsic terms {@code fy}, {@code cx}, {@code cy} (NaN until loaded).
     * Logged alongside {@code fx} so an offline harness has the full matrix to re-run the PnP solve
     * from the logged tag corners; the replay source passes the recorded values straight through.
     */
    double getCalFy();

    double getCalCx();

    double getCalCy();

    /**
     * The camera distortion coefficients actually passed to the on-robot PnP solve (OpenCV order
     * {@code [k1,k2,p1,p2,k3,...]}), or null until loaded. Logged so an offline re-solve matches
     * the robot exactly rather than assuming zero distortion; the replay source passes them
     * through.
     */
    double[] getCalDistCoeffs();
}
