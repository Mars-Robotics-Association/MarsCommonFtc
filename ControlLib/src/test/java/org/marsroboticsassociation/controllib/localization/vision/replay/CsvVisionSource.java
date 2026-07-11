package org.marsroboticsassociation.controllib.localization.vision.replay;

import org.marsroboticsassociation.controllib.localization.vision.VisionFrame;
import org.marsroboticsassociation.controllib.localization.vision.VisionFrameCsv;
import org.marsroboticsassociation.controllib.localization.vision.VisionSource;

/**
 * A {@link VisionSource} that replays recorded telemetry rows: it reconstructs the per-frame {@link
 * VisionFrame} the localizer consumes from the logged columns (via {@link VisionFrameCsv#parse}), so
 * the exact production chain runs off-robot. The driver calls {@link #prepare} once per row before
 * {@code HypothesisBankLocalizer.update()}, then the localizer pulls {@link #latest()}.
 *
 * <p>A {@code vis_newFrame == 1} row gets a fresh, incrementing {@link VisionFrame#timestamp} (and
 * {@code valid = true}); every other row carries {@code valid = false}, which the localizer treats
 * as "the camera returned the same cached frame again" — it does not re-process. Intrinsics are
 * constant within a run, so the last finite values are held across no-frame loops.
 */
public final class CsvVisionSource implements VisionSource {

    private VisionFrame current = new VisionFrame(); // valid == false until the first prepared row
    private double calFx = Double.NaN, calFy = Double.NaN, calCx = Double.NaN, calCy = Double.NaN;
    private final double[] calDist = {Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN};
    private long tsCounter = 0;

    /**
     * Build the frame for one telemetry row.
     *
     * @param csv the run
     * @param row row index
     * @param receiptNanos the loop's clock time (the back-date anchor), shared with the replay clock
     */
    public void prepare(TelemetryCsv csv, int row, long receiptNanos) {
        final int r = row;
        VisionFrameCsv.Parsed parsed = VisionFrameCsv.parse(col -> csv.get(r, col));
        VisionFrame f = parsed.frame;

        boolean newFrame = csv.get(row, "vis_newFrame") == 1.0;
        f.valid = newFrame; // a fresh camera frame this loop (else a held/no result: not processed)
        if (newFrame) {
            tsCounter++;
        }
        f.timestamp = tsCounter; // identical across non-new rows -> the ts dedup skips them
        f.receiptNanos = receiptNanos;
        current = f;

        // Intrinsics are constant within a run; hold the last finite values across no-frame loops.
        if (Double.isFinite(parsed.cameraMatrix[0])) calFx = parsed.cameraMatrix[0];
        if (Double.isFinite(parsed.cameraMatrix[4])) calFy = parsed.cameraMatrix[4];
        if (Double.isFinite(parsed.cameraMatrix[2])) calCx = parsed.cameraMatrix[2];
        if (Double.isFinite(parsed.cameraMatrix[5])) calCy = parsed.cameraMatrix[5];
        for (int i = 0; i < calDist.length; i++) {
            if (Double.isFinite(parsed.distCoeffs[i])) calDist[i] = parsed.distCoeffs[i];
        }
    }

    @Override
    public VisionFrame latest() {
        return current;
    }

    @Override
    public void updateRobotOrientation(double headingDeg) {
        // No-op in replay: the recorded frame already carries whatever the seed produced.
    }

    @Override
    public boolean prefetchCalibration(long timeoutMs) {
        return true; // nothing to fetch; intrinsics come from the recorded columns
    }

    @Override
    public double getCalFx() {
        return calFx;
    }

    @Override
    public double getCalFy() {
        return calFy;
    }

    @Override
    public double getCalCx() {
        return calCx;
    }

    @Override
    public double getCalCy() {
        return calCy;
    }

    @Override
    public double[] getCalDistCoeffs() {
        return calDist;
    }

    /** The full row-major 3x3 camera matrix currently held ({@code [fx,0,cx, 0,fy,cy, 0,0,1]}). */
    public double[] cameraMatrix() {
        return new double[] {calFx, 0, calCx, 0, calFy, calCy, 0, 0, 1};
    }
}
