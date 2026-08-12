package org.marsroboticsassociation.controllib.localization

import edu.wpi.first.math.MathUtil
import java.util.NavigableMap
import java.util.TreeMap
import kotlin.math.PI
import kotlin.math.sqrt

/**
 * Fuses field-frame odometry poses with vision measurements using latency-compensated Kalman
 * filtering.
 *
 * <p>This is a simplified reimplementation of WPILib's `PoseEstimator` that works directly with
 * field-frame poses. Unlike the WPILib version, which was designed for wheel-encoder odometry and
 * requires converting through robot-frame twists, this class accepts field-centric poses directly
 * (as output by odometry computers like the GoBilda Pinpoint or SparkFun OTOS).
 *
 * <p>The algorithm is identical to WPILib's: an odometry history buffer enables latency-compensated
 * vision corrections, and a per-axis Kalman gain controls the trust balance between odometry and
 * vision. The simplification is that `compensate()` uses field-frame addition instead of WPILib's
 * robot-frame `Transform2d` round-trip.
 *
 * <h2>Usage</h2>
 * <ol>
 * <li>Call [update] every loop with the current timestamp and odometry pose</li>
 * <li>Call [addVisionMeasurement] when vision data is available</li>
 * <li>Call [getEstimatedPose] to get the fused estimate</li>
 * </ol>
 *
 * <p>All coordinates should use consistent units (e.g., inches + radians or meters + radians). The
 * estimator is unit-agnostic; just ensure odometry and vision use the same units.
 */
class FieldPoseEstimator(
    stateStdDevX: Double,
    stateStdDevY: Double,
    stateStdDevHeading: Double,
    visionStdDevX: Double,
    visionStdDevY: Double,
    visionStdDevHeading: Double,
) {

    /**
     * An immutable field-frame pose: (x, y, heading).
     *
     * <p>Heading is stored as-is from the source. For odometry poses this is typically cumulative
     * (unwrapped); for vision poses it may be wrapped. The [interpolate] method uses shortest-path
     * heading interpolation, which is correct for both cases when interpolating between temporally
     * adjacent samples.
     */
    class FieldPose(
        @JvmField val x: Double,
        @JvmField val y: Double,
        @JvmField val heading: Double,
    ) {
        /**
         * Linearly interpolates between this pose and [other].
         *
         * @param other the target pose
         * @param t interpolation parameter in [0, 1]
         * @return the interpolated pose
         */
        fun interpolate(other: FieldPose, t: Double): FieldPose {
            val headingDiff = Math.IEEEremainder(other.heading - this.heading, 2 * PI)
            return FieldPose(
                this.x + t * (other.x - this.x),
                this.y + t * (other.y - this.y),
                this.heading + t * headingDiff,
            )
        }
    }

    /**
     * A vision correction record storing the corrected pose and the odometry pose at the same
     * timestamp. [compensate] replays the odometry delta since that timestamp onto the corrected
     * pose to produce a current estimate.
     */
    private class VisionUpdate(
        private val correctedPose: FieldPose,
        private val odometryPose: FieldPose,
    ) {
        /**
         * Returns the vision-compensated version of a current odometry pose.
         *
         * <p>This is the field-frame equivalent of WPILib's `visionPose.plus(pose.minus(
         * odometryPose))`. Because both poses are in field coordinates, the delta is simple
         * subtraction and the compensation is simple addition. Heading deltas are NOT wrapped
         * because odometry heading is cumulative/unwrapped.
         */
        fun compensate(currentOdometry: FieldPose): FieldPose {
            return FieldPose(
                correctedPose.x + (currentOdometry.x - odometryPose.x),
                correctedPose.y + (currentOdometry.y - odometryPose.y),
                correctedPose.heading + (currentOdometry.heading - odometryPose.heading),
            )
        }
    }

    private val q = DoubleArray(3)
    private val kalmanGain = DoubleArray(3)

    private val odometryBuffer = TreeMap<Double, FieldPose>()
    private val visionUpdates: NavigableMap<Double, VisionUpdate> = TreeMap()

    private var estimatedPose: FieldPose

    init {
        q[0] = stateStdDevX * stateStdDevX
        q[1] = stateStdDevY * stateStdDevY
        q[2] = stateStdDevHeading * stateStdDevHeading
        setVisionStdDevs(visionStdDevX, visionStdDevY, visionStdDevHeading)
        estimatedPose = FieldPose(0.0, 0.0, 0.0)
    }

    /**
     * Updates the Kalman gain for vision measurements.
     *
     * <p>Uses the closed-form solution for a continuous Kalman filter with A = 0 and C = I: `K_i =
     * q_i / (q_i + sqrt(q_i * r_i))`
     *
     * <p>It is safe to pass different values for `stdDevX` and `stdDevY` (e.g., Limelight's
     * per-axis standard deviations directly). Because this estimator computes the innovation and
     * applies corrections in field frame, anisotropic gains correctly scale the correction along
     * field X and Y axes independently. This was not the case with the WPILib `PoseEstimator`,
     * which applied gains in the robot's local frame via `Transform2d`, causing anisotropic gains
     * to correct along robot-body axes instead of toward the vision pose.
     *
     * @param stdDevX vision X standard deviation, same units as the state stddev
     * @param stdDevY vision Y standard deviation
     * @param stdDevHeading vision heading standard deviation (radians)
     */
    fun setVisionStdDevs(stdDevX: Double, stdDevY: Double, stdDevHeading: Double) {
        val r = doubleArrayOf(stdDevX * stdDevX, stdDevY * stdDevY, stdDevHeading * stdDevHeading)
        for (i in 0 until 3) {
            if (q[i] == 0.0) {
                kalmanGain[i] = 0.0
            } else {
                kalmanGain[i] = q[i] / (q[i] + sqrt(q[i] * r[i]))
            }
        }
    }

    /** Resets the estimator to the given pose, clearing all history buffers. */
    fun resetPose(pose: FieldPose) {
        odometryBuffer.clear()
        visionUpdates.clear()
        estimatedPose = pose
    }

    /** Returns the current fused pose estimate. */
    fun getEstimatedPose(): FieldPose = estimatedPose

    /**
     * Feeds a new odometry reading and updates the fused estimate.
     *
     * <p>Must be called every loop iteration. The timestamp should use the same epoch as the
     * timestamps passed to [addVisionMeasurement] (typically `System.nanoTime() / 1e9`).
     *
     * @param timestampSec time of the odometry reading in seconds
     * @param odometryPose the field-frame odometry pose
     */
    fun update(timestampSec: Double, odometryPose: FieldPose) {
        odometryBuffer[timestampSec] = odometryPose

        // Trim buffer to BUFFER_DURATION
        while (
            !odometryBuffer.isEmpty() &&
                odometryBuffer.lastKey() - odometryBuffer.firstKey() > BUFFER_DURATION
        ) {
            odometryBuffer.pollFirstEntry()
        }

        if (visionUpdates.isEmpty()) {
            estimatedPose = odometryPose
        } else {
            val latestVision = visionUpdates[visionUpdates.lastKey()]!!
            estimatedPose = latestVision.compensate(odometryPose)
        }
    }

    /**
     * Adds a latency-compensated vision measurement.
     *
     * <p>The vision pose is blended with the current estimate using the Kalman gain, and the
     * correction is stored so that subsequent odometry updates are compensated.
     *
     * @param visionPose the vision-measured field pose (same units as odometry)
     * @param timestampSec the capture time of the vision measurement (same epoch as [update])
     */
    fun addVisionMeasurement(visionPose: FieldPose, timestampSec: Double) {
        // Step 0: Skip if too old for the buffer
        if (odometryBuffer.isEmpty() || odometryBuffer.lastKey() - BUFFER_DURATION > timestampSec) {
            return
        }

        // Step 1: Clean up old vision entries
        cleanUpVisionUpdates()

        // Step 2: Interpolate odometry at vision capture time
        val odometrySample = sampleOdometry(timestampSec) ?: return

        // Step 3: Get the vision-compensated estimate at vision capture time
        val visionSample = sampleAt(timestampSec) ?: return

        // Step 4: Compute innovation in field frame
        val dx = visionPose.x - visionSample.x
        val dy = visionPose.y - visionSample.y
        val dh = Math.IEEEremainder(visionPose.heading - visionSample.heading, 2 * PI)

        // Step 5: Scale by Kalman gain
        val scaledDx = kalmanGain[0] * dx
        val scaledDy = kalmanGain[1] * dy
        val scaledDh = kalmanGain[2] * dh

        // Step 6: Create corrected pose and store vision update
        val corrected =
            FieldPose(
                visionSample.x + scaledDx,
                visionSample.y + scaledDy,
                visionSample.heading + scaledDh,
            )
        val visionUpdate = VisionUpdate(corrected, odometrySample)
        visionUpdates[timestampSec] = visionUpdate

        // Step 7: Remove later vision updates (they were based on stale data)
        visionUpdates.tailMap(timestampSec, false).clear()

        // Step 8: Update the current estimate
        val latestOdometry = odometryBuffer.lastEntry().value
        estimatedPose = visionUpdate.compensate(latestOdometry)
    }

    /** Interpolates the odometry buffer at the given timestamp. */
    private fun sampleOdometry(timestampSec: Double): FieldPose? {
        if (odometryBuffer.isEmpty()) return null

        // Clamp to buffer range
        val first = odometryBuffer.firstKey()
        val last = odometryBuffer.lastKey()
        val ts = MathUtil.clamp(timestampSec, first, last)

        // Exact match
        val exact = odometryBuffer[ts]
        if (exact != null) return exact

        // Interpolate between bracketing entries
        val lower = odometryBuffer.floorEntry(ts)
        val upper = odometryBuffer.ceilingEntry(ts)

        if (lower == null) return upper.value
        if (upper == null) return lower.value

        val t = (ts - lower.key) / (upper.key - lower.key)
        return lower.value.interpolate(upper.value, t)
    }

    /** Returns the vision-compensated pose at a given timestamp (mirrors WPILib's sampleAt). */
    private fun sampleAt(timestampSec: Double): FieldPose? {
        if (odometryBuffer.isEmpty()) return null

        val first = odometryBuffer.firstKey()
        val last = odometryBuffer.lastKey()
        val ts = MathUtil.clamp(timestampSec, first, last)

        // If no vision updates apply, use raw odometry
        if (visionUpdates.isEmpty() || ts < visionUpdates.firstKey()) {
            return sampleOdometry(ts)
        }

        // Find the most recent vision update at or before this timestamp
        val floorTimestamp = visionUpdates.floorKey(ts)
        val visionUpdate = visionUpdates[floorTimestamp]!!

        val odometryPose = sampleOdometry(ts) ?: return null

        return visionUpdate.compensate(odometryPose)
    }

    /** Removes stale vision updates that can no longer affect sampling. */
    private fun cleanUpVisionUpdates() {
        if (odometryBuffer.isEmpty()) return

        val oldestOdometryTimestamp = odometryBuffer.firstKey()

        if (visionUpdates.isEmpty() || oldestOdometryTimestamp < visionUpdates.firstKey()) return

        val newestNeeded = visionUpdates.floorKey(oldestOdometryTimestamp)
        if (newestNeeded != null) {
            visionUpdates.headMap(newestNeeded, false).clear()
        }
    }

    companion object {
        private const val BUFFER_DURATION = 1.5
    }
}
