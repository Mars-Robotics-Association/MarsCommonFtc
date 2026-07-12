package org.marsroboticsassociation.controllab.arm;

/**
 * Rolling, backlash-focused metrics for the Arm tab, computed against the load ground truth. Fed one
 * sample per control tick plus a notification on each target change (which resets the per-move
 * latches). All public angles are reported in degrees.
 *
 * <ul>
 *   <li><b>Steady-state error</b> — instantaneous |target − load|.
 *   <li><b>Overshoot</b> — max excursion of the load past the target in the direction of the move.
 *   <li><b>Settling time</b> — time from the last target change until the load first entered the
 *       position + velocity tolerance band.
 *   <li><b>Lost motion on reversal</b> — the lash gap |motor − load| the load lags across, tracked
 *       as the peak since the last commanded-direction reversal.
 *   <li><b>% time engaged</b> — an EMA of the plant's {@code isEngaged} flag.
 * </ul>
 */
public class ArmMetrics {

    private static final double SETTLE_POS_TOL_RAD = Math.toRadians(2.0);
    private static final double SETTLE_VEL_TOL_RAD = Math.toRadians(5.0);
    private static final double ENGAGED_EMA_ALPHA = 0.02; // ~ last few seconds at 60 Hz

    // Per-move latches
    private double moveStartTime = 0.0;
    private double moveDir = 0.0;          // sign(target - load) at last target change
    private double overshootDeg = 0.0;
    private double settleTimeSec = Double.NaN;
    private double lostMotionDeg = 0.0;    // peak lash gap since the last reversal

    // Live values
    private double targetRad = 0.0;
    private double lastErrorDeg = 0.0;
    private double lashGapDeg = 0.0;
    private double engagedEma = 1.0;
    private boolean sawTarget = false;

    /** Reset the per-move latches for a new commanded target. */
    public void onTargetChanged(double newTargetRad, double loadRad, double tSec) {
        double newDir = Math.signum(newTargetRad - loadRad);
        // A reversal is a direction flip relative to the previous move; reset the lash-gap peak.
        if (sawTarget && newDir != 0 && moveDir != 0 && newDir != moveDir) {
            lostMotionDeg = 0.0;
        }
        moveDir = newDir;
        targetRad = newTargetRad;
        moveStartTime = tSec;
        overshootDeg = 0.0;
        settleTimeSec = Double.NaN;
        sawTarget = true;
    }

    /** Feed one control-tick sample. */
    public void update(double tSec, double loadRad, double loadVelRad,
                       double motorRad, boolean engaged) {
        double errorRad = targetRad - loadRad;
        lastErrorDeg = Math.abs(Math.toDegrees(errorRad));
        lashGapDeg = Math.abs(Math.toDegrees(motorRad - loadRad));

        // Overshoot: how far past the target, in the direction of travel.
        if (moveDir != 0) {
            double past = Math.toDegrees((loadRad - targetRad) * moveDir);
            if (past > overshootDeg) overshootDeg = past;
        }

        // Settling time: first entry into the tolerance band after the target change.
        if (Double.isNaN(settleTimeSec)
                && Math.abs(errorRad) < SETTLE_POS_TOL_RAD
                && Math.abs(loadVelRad) < SETTLE_VEL_TOL_RAD) {
            settleTimeSec = tSec - moveStartTime;
        }

        // Lost motion / lash gap: peak since the last reversal.
        if (lashGapDeg > lostMotionDeg) lostMotionDeg = lashGapDeg;

        engagedEma += ENGAGED_EMA_ALPHA * ((engaged ? 1.0 : 0.0) - engagedEma);
    }

    /** Clear all state (used on engine reset). */
    public void reset() {
        moveStartTime = 0.0;
        moveDir = 0.0;
        overshootDeg = 0.0;
        settleTimeSec = Double.NaN;
        lostMotionDeg = 0.0;
        targetRad = 0.0;
        lastErrorDeg = 0.0;
        lashGapDeg = 0.0;
        engagedEma = 1.0;
        sawTarget = false;
    }

    public double steadyStateErrorDeg() { return lastErrorDeg; }
    public double overshootDeg()        { return overshootDeg; }
    public double settleTimeSec()       { return settleTimeSec; }
    public double lashGapDeg()          { return lashGapDeg; }
    public double lostMotionDeg()       { return lostMotionDeg; }
    public double pctEngaged()          { return engagedEma * 100.0; }
}
