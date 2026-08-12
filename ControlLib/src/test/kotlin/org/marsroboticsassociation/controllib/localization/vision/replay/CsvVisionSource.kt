package org.marsroboticsassociation.controllib.localization.vision.replay

import org.marsroboticsassociation.controllib.localization.vision.VisionFrame
import org.marsroboticsassociation.controllib.localization.vision.VisionFrameCsv
import org.marsroboticsassociation.controllib.localization.vision.VisionSource

/**
 * A [VisionSource] that replays recorded telemetry rows: it reconstructs the per-frame
 * [VisionFrame] the localizer consumes from the logged columns (via [VisionFrameCsv.parse]), so the
 * exact production chain runs off-robot. The driver calls [prepare] once per row before
 * `HypothesisBankLocalizer.update()`, then the localizer pulls [latest].
 *
 * <p>A `vis_newFrame == 1` row gets a fresh, incrementing [VisionFrame.timestamp] (and `valid =
 * true`); every other row carries `valid = false`, which the localizer treats as "the camera
 * returned the same cached frame again" — it does not re-process. Intrinsics are constant within a
 * run, so the last finite values are held across no-frame loops.
 */
class CsvVisionSource : VisionSource {

    private var current = VisionFrame() // valid == false until the first prepared row
    private var calFx = Double.NaN
    private var calFy = Double.NaN
    private var calCx = Double.NaN
    private var calCy = Double.NaN
    private val calDist = doubleArrayOf(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN)
    private var tsCounter = 0L

    /**
     * Build the frame for one telemetry row.
     *
     * @param csv the run
     * @param row row index
     * @param receiptNanos the loop's clock time (the back-date anchor), shared with the replay
     *   clock
     */
    fun prepare(csv: TelemetryCsv, row: Int, receiptNanos: Long) {
        val r = row
        val parsed = VisionFrameCsv.parse { col -> csv.get(r, col) }
        val f = parsed.frame

        val newFrame = csv.get(row, "vis_newFrame") == 1.0
        f.valid = newFrame // a fresh camera frame this loop (else a held/no result: not processed)
        if (newFrame) {
            tsCounter++
        }
        f.timestamp =
            tsCounter.toDouble() // identical across non-new rows -> the ts dedup skips them
        f.receiptNanos = receiptNanos
        current = f

        // Intrinsics are constant within a run; hold the last finite values across no-frame loops.
        if (parsed.cameraMatrix[0].isFinite()) calFx = parsed.cameraMatrix[0]
        if (parsed.cameraMatrix[4].isFinite()) calFy = parsed.cameraMatrix[4]
        if (parsed.cameraMatrix[2].isFinite()) calCx = parsed.cameraMatrix[2]
        if (parsed.cameraMatrix[5].isFinite()) calCy = parsed.cameraMatrix[5]
        for (i in calDist.indices) {
            if (parsed.distCoeffs[i].isFinite()) calDist[i] = parsed.distCoeffs[i]
        }
    }

    override fun latest(): VisionFrame = current

    override fun updateRobotOrientation(headingDeg: Double) {
        // No-op in replay: the recorded frame already carries whatever the seed produced.
    }

    override fun prefetchCalibration(timeoutMs: Long): Boolean {
        return true // nothing to fetch; intrinsics come from the recorded columns
    }

    override fun getCalFx(): Double = calFx

    override fun getCalFy(): Double = calFy

    override fun getCalCx(): Double = calCx

    override fun getCalCy(): Double = calCy

    override fun getCalDistCoeffs(): DoubleArray = calDist

    /** The full row-major 3x3 camera matrix currently held (`[fx,0,cx, 0,fy,cy, 0,0,1]`). */
    fun cameraMatrix(): DoubleArray {
        return doubleArrayOf(calFx, 0.0, calCx, 0.0, calFy, calCy, 0.0, 0.0, 1.0)
    }
}
