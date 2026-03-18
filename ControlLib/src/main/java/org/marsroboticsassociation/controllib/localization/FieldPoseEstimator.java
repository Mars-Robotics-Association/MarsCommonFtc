package org.marsroboticsassociation.controllib.localization;

import edu.wpi.first.math.MathUtil;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/**
 * Fuses field-frame odometry poses with vision measurements using latency-compensated Kalman
 * filtering.
 *
 * <p>This is a simplified reimplementation of WPILib's {@code PoseEstimator} that works directly
 * with field-frame poses. Unlike the WPILib version, which was designed for wheel-encoder odometry
 * and requires converting through robot-frame twists, this class accepts field-centric poses
 * directly (as output by odometry computers like the GoBilda Pinpoint or SparkFun OTOS).
 *
 * <p>The algorithm is identical to WPILib's: an odometry history buffer enables latency-compensated
 * vision corrections, and a per-axis Kalman gain controls the trust balance between odometry and
 * vision. The simplification is that {@code compensate()} uses field-frame addition instead of
 * WPILib's robot-frame {@code Transform2d} round-trip.
 *
 * <h2>Usage</h2>
 * <ol>
 *   <li>Call {@link #update} every loop with the current timestamp and odometry pose</li>
 *   <li>Call {@link #addVisionMeasurement} when vision data is available</li>
 *   <li>Call {@link #getEstimatedPose} to get the fused estimate</li>
 * </ol>
 *
 * <p>All coordinates should use consistent units (e.g., inches + radians or meters + radians).
 * The estimator is unit-agnostic; just ensure odometry and vision use the same units.
 */
public class FieldPoseEstimator {

    /**
     * An immutable field-frame pose: (x, y, heading).
     *
     * <p>Heading is stored as-is from the source. For odometry poses this is typically cumulative
     * (unwrapped); for vision poses it may be wrapped. The {@link #interpolate} method uses
     * shortest-path heading interpolation, which is correct for both cases when interpolating
     * between temporally adjacent samples.
     */
    public static class FieldPose {
        public final double x;
        public final double y;
        public final double heading;

        public FieldPose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        /**
         * Linearly interpolates between this pose and {@code other}.
         *
         * @param other the target pose
         * @param t     interpolation parameter in [0, 1]
         * @return the interpolated pose
         */
        public FieldPose interpolate(FieldPose other, double t) {
            double headingDiff = Math.IEEEremainder(other.heading - this.heading, 2 * Math.PI);
            return new FieldPose(
                    this.x + t * (other.x - this.x),
                    this.y + t * (other.y - this.y),
                    this.heading + t * headingDiff
            );
        }
    }

    /**
     * A vision correction record storing the corrected pose and the odometry pose at the same
     * timestamp. {@link #compensate} replays the odometry delta since that timestamp onto the
     * corrected pose to produce a current estimate.
     */
    private static final class VisionUpdate {
        private final FieldPose correctedPose;
        private final FieldPose odometryPose;

        private VisionUpdate(FieldPose correctedPose, FieldPose odometryPose) {
            this.correctedPose = correctedPose;
            this.odometryPose = odometryPose;
        }

        /**
         * Returns the vision-compensated version of a current odometry pose.
         *
         * <p>This is the field-frame equivalent of WPILib's {@code visionPose.plus(pose.minus(
         * odometryPose))}. Because both poses are in field coordinates, the delta is simple
         * subtraction and the compensation is simple addition. Heading deltas are NOT wrapped
         * because odometry heading is cumulative/unwrapped.
         */
        public FieldPose compensate(FieldPose currentOdometry) {
            return new FieldPose(
                    correctedPose.x + (currentOdometry.x - odometryPose.x),
                    correctedPose.y + (currentOdometry.y - odometryPose.y),
                    correctedPose.heading + (currentOdometry.heading - odometryPose.heading)
            );
        }
    }

    private static final double BUFFER_DURATION = 1.5;

    private final double[] q = new double[3];
    private final double[] kalmanGain = new double[3];

    private final TreeMap<Double, FieldPose> odometryBuffer = new TreeMap<>();
    private final NavigableMap<Double, VisionUpdate> visionUpdates = new TreeMap<>();

    private FieldPose estimatedPose;

    /**
     * Constructs a FieldPoseEstimator.
     *
     * @param stateStdDevX              odometry X standard deviation. Increase to trust odometry less.
     * @param stateStdDevY              odometry Y standard deviation.
     * @param stateStdDevHeading        odometry heading standard deviation (radians).
     * @param visionStdDevX             initial vision X standard deviation.
     *                                  Typically overwritten per-measurement via
     *                                  {@link #setVisionStdDevs}.
     * @param visionStdDevY             initial vision Y standard deviation.
     * @param visionStdDevHeading       initial vision heading standard deviation (radians).
     */
    public FieldPoseEstimator(
            double stateStdDevX, double stateStdDevY, double stateStdDevHeading,
            double visionStdDevX, double visionStdDevY, double visionStdDevHeading) {
        q[0] = stateStdDevX * stateStdDevX;
        q[1] = stateStdDevY * stateStdDevY;
        q[2] = stateStdDevHeading * stateStdDevHeading;
        setVisionStdDevs(visionStdDevX, visionStdDevY, visionStdDevHeading);
        estimatedPose = new FieldPose(0, 0, 0);
    }

    /**
     * Updates the Kalman gain for vision measurements.
     *
     * <p>Uses the closed-form solution for a continuous Kalman filter with A = 0 and C = I:
     * {@code K_i = q_i / (q_i + sqrt(q_i * r_i))}
     *
     * <p>It is safe to pass different values for {@code stdDevX} and {@code stdDevY} (e.g.,
     * Limelight's per-axis standard deviations directly). Because this estimator computes the
     * innovation and applies corrections in field frame, anisotropic gains correctly scale the
     * correction along field X and Y axes independently. This was not the case with the WPILib
     * {@code PoseEstimator}, which applied gains in the robot's local frame via
     * {@code Transform2d}, causing anisotropic gains to correct along robot-body axes instead
     * of toward the vision pose.
     *
     * @param stdDevX       vision X standard deviation, same units as the state stddev
     * @param stdDevY       vision Y standard deviation
     * @param stdDevHeading vision heading standard deviation (radians)
     */
    public void setVisionStdDevs(double stdDevX, double stdDevY, double stdDevHeading) {
        double[] r = {stdDevX * stdDevX, stdDevY * stdDevY, stdDevHeading * stdDevHeading};
        for (int i = 0; i < 3; i++) {
            if (q[i] == 0.0) {
                kalmanGain[i] = 0.0;
            } else {
                kalmanGain[i] = q[i] / (q[i] + Math.sqrt(q[i] * r[i]));
            }
        }
    }

    /**
     * Resets the estimator to the given pose, clearing all history buffers.
     */
    public void resetPose(FieldPose pose) {
        odometryBuffer.clear();
        visionUpdates.clear();
        estimatedPose = pose;
    }

    /**
     * Returns the current fused pose estimate.
     */
    public FieldPose getEstimatedPose() {
        return estimatedPose;
    }

    /**
     * Feeds a new odometry reading and updates the fused estimate.
     *
     * <p>Must be called every loop iteration. The timestamp should use the same epoch as the
     * timestamps passed to {@link #addVisionMeasurement} (typically {@code System.nanoTime() / 1e9}).
     *
     * @param timestampSec time of the odometry reading in seconds
     * @param odometryPose the field-frame odometry pose
     */
    public void update(double timestampSec, FieldPose odometryPose) {
        odometryBuffer.put(timestampSec, odometryPose);

        // Trim buffer to BUFFER_DURATION
        while (!odometryBuffer.isEmpty()
                && odometryBuffer.lastKey() - odometryBuffer.firstKey() > BUFFER_DURATION) {
            odometryBuffer.pollFirstEntry();
        }

        if (visionUpdates.isEmpty()) {
            estimatedPose = odometryPose;
        } else {
            VisionUpdate latestVision = visionUpdates.get(visionUpdates.lastKey());
            estimatedPose = latestVision.compensate(odometryPose);
        }
    }

    /**
     * Adds a latency-compensated vision measurement.
     *
     * <p>The vision pose is blended with the current estimate using the Kalman gain, and the
     * correction is stored so that subsequent odometry updates are compensated.
     *
     * @param visionPose   the vision-measured field pose (same units as odometry)
     * @param timestampSec the capture time of the vision measurement (same epoch as
     *                     {@link #update})
     */
    public void addVisionMeasurement(FieldPose visionPose, double timestampSec) {
        // Step 0: Skip if too old for the buffer
        if (odometryBuffer.isEmpty()
                || odometryBuffer.lastKey() - BUFFER_DURATION > timestampSec) {
            return;
        }

        // Step 1: Clean up old vision entries
        cleanUpVisionUpdates();

        // Step 2: Interpolate odometry at vision capture time
        FieldPose odometrySample = sampleOdometry(timestampSec);
        if (odometrySample == null) return;

        // Step 3: Get the vision-compensated estimate at vision capture time
        FieldPose visionSample = sampleAt(timestampSec);
        if (visionSample == null) return;

        // Step 4: Compute innovation in field frame
        double dx = visionPose.x - visionSample.x;
        double dy = visionPose.y - visionSample.y;
        double dh = Math.IEEEremainder(visionPose.heading - visionSample.heading, 2 * Math.PI);

        // Step 5: Scale by Kalman gain
        double scaledDx = kalmanGain[0] * dx;
        double scaledDy = kalmanGain[1] * dy;
        double scaledDh = kalmanGain[2] * dh;

        // Step 6: Create corrected pose and store vision update
        FieldPose corrected = new FieldPose(
                visionSample.x + scaledDx,
                visionSample.y + scaledDy,
                visionSample.heading + scaledDh
        );
        VisionUpdate visionUpdate = new VisionUpdate(corrected, odometrySample);
        visionUpdates.put(timestampSec, visionUpdate);

        // Step 7: Remove later vision updates (they were based on stale data)
        visionUpdates.tailMap(timestampSec, false).clear();

        // Step 8: Update the current estimate
        FieldPose latestOdometry = odometryBuffer.lastEntry().getValue();
        estimatedPose = visionUpdate.compensate(latestOdometry);
    }

    /**
     * Interpolates the odometry buffer at the given timestamp.
     */
    private FieldPose sampleOdometry(double timestampSec) {
        if (odometryBuffer.isEmpty()) return null;

        // Clamp to buffer range
        double first = odometryBuffer.firstKey();
        double last = odometryBuffer.lastKey();
        timestampSec = MathUtil.clamp(timestampSec, first, last);

        // Exact match
        FieldPose exact = odometryBuffer.get(timestampSec);
        if (exact != null) return exact;

        // Interpolate between bracketing entries
        Map.Entry<Double, FieldPose> lower = odometryBuffer.floorEntry(timestampSec);
        Map.Entry<Double, FieldPose> upper = odometryBuffer.ceilingEntry(timestampSec);

        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue();

        double t = (timestampSec - lower.getKey()) / (upper.getKey() - lower.getKey());
        return lower.getValue().interpolate(upper.getValue(), t);
    }

    /**
     * Returns the vision-compensated pose at a given timestamp (mirrors WPILib's sampleAt).
     */
    private FieldPose sampleAt(double timestampSec) {
        if (odometryBuffer.isEmpty()) return null;

        double first = odometryBuffer.firstKey();
        double last = odometryBuffer.lastKey();
        timestampSec = MathUtil.clamp(timestampSec, first, last);

        // If no vision updates apply, use raw odometry
        if (visionUpdates.isEmpty() || timestampSec < visionUpdates.firstKey()) {
            return sampleOdometry(timestampSec);
        }

        // Find the most recent vision update at or before this timestamp
        double floorTimestamp = visionUpdates.floorKey(timestampSec);
        VisionUpdate visionUpdate = visionUpdates.get(floorTimestamp);

        FieldPose odometryPose = sampleOdometry(timestampSec);
        if (odometryPose == null) return null;

        return visionUpdate.compensate(odometryPose);
    }

    /**
     * Removes stale vision updates that can no longer affect sampling.
     */
    private void cleanUpVisionUpdates() {
        if (odometryBuffer.isEmpty()) return;

        double oldestOdometryTimestamp = odometryBuffer.firstKey();

        if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) return;

        Double newestNeeded = visionUpdates.floorKey(oldestOdometryTimestamp);
        if (newestNeeded != null) {
            visionUpdates.headMap(newestNeeded, false).clear();
        }
    }
}
