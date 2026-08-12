package org.marsroboticsassociation.controllab.arm

import kotlin.math.abs
import kotlin.math.sign

/**
 * Rolling, backlash-focused metrics for the Arm tab, computed against the load ground truth. Fed
 * one sample per control tick plus a notification on each target change (which resets the per-move
 * latches). All public angles are reported in degrees.
 *
 * - **Steady-state error** — instantaneous |target − load|.
 * - **Overshoot** — max excursion of the load past the target in the direction of the move.
 * - **Settling time** — time from the last target change until the load first entered the
 *   position + velocity tolerance band.
 * - **Lost motion on reversal** — the lash gap |motor − load| the load lags across, tracked as the
 *   peak since the last commanded-direction reversal.
 * - **% time engaged** — an EMA of the plant's `isEngaged` flag.
 */
class ArmMetrics {
    // Per-move latches
    private var moveStartTime = 0.0
    private var moveDir = 0.0 // sign(target - load) at last target change
    private var overshootDegField = 0.0
    private var settleTimeSecField = Double.NaN
    private var lostMotionDegField = 0.0 // peak lash gap since the last reversal

    // Live values
    private var targetRad = 0.0
    private var lastErrorDeg = 0.0
    private var lashGapDegField = 0.0
    private var engagedEma = 1.0
    private var sawTarget = false

    /** Reset the per-move latches for a new commanded target. */
    fun onTargetChanged(newTargetRad: Double, loadRad: Double, tSec: Double) {
        val newDir = sign(newTargetRad - loadRad)
        // A reversal is a direction flip relative to the previous move; reset the lash-gap peak.
        if (sawTarget && newDir != 0.0 && moveDir != 0.0 && newDir != moveDir) {
            lostMotionDegField = 0.0
        }
        moveDir = newDir
        targetRad = newTargetRad
        moveStartTime = tSec
        overshootDegField = 0.0
        settleTimeSecField = Double.NaN
        sawTarget = true
    }

    /** Feed one control-tick sample. */
    fun update(
        tSec: Double,
        loadRad: Double,
        loadVelRad: Double,
        motorRad: Double,
        engaged: Boolean,
    ) {
        val errorRad = targetRad - loadRad
        lastErrorDeg = abs(Math.toDegrees(errorRad))
        lashGapDegField = abs(Math.toDegrees(motorRad - loadRad))

        // Overshoot: how far past the target, in the direction of travel.
        if (moveDir != 0.0) {
            val past = Math.toDegrees((loadRad - targetRad) * moveDir)
            if (past > overshootDegField) overshootDegField = past
        }

        // Settling time: first entry into the tolerance band after the target change.
        if (
            settleTimeSecField.isNaN() &&
                abs(errorRad) < SETTLE_POS_TOL_RAD &&
                abs(loadVelRad) < SETTLE_VEL_TOL_RAD
        ) {
            settleTimeSecField = tSec - moveStartTime
        }

        // Lost motion / lash gap: peak since the last reversal.
        if (lashGapDegField > lostMotionDegField) lostMotionDegField = lashGapDegField

        engagedEma += ENGAGED_EMA_ALPHA * ((if (engaged) 1.0 else 0.0) - engagedEma)
    }

    /** Clear all state (used on engine reset). */
    fun reset() {
        moveStartTime = 0.0
        moveDir = 0.0
        overshootDegField = 0.0
        settleTimeSecField = Double.NaN
        lostMotionDegField = 0.0
        targetRad = 0.0
        lastErrorDeg = 0.0
        lashGapDegField = 0.0
        engagedEma = 1.0
        sawTarget = false
    }

    fun steadyStateErrorDeg(): Double = lastErrorDeg

    fun overshootDeg(): Double = overshootDegField

    fun settleTimeSec(): Double = settleTimeSecField

    fun lashGapDeg(): Double = lashGapDegField

    fun lostMotionDeg(): Double = lostMotionDegField

    fun pctEngaged(): Double = engagedEma * 100.0

    companion object {
        private val SETTLE_POS_TOL_RAD = Math.toRadians(2.0)
        private val SETTLE_VEL_TOL_RAD = Math.toRadians(5.0)
        private const val ENGAGED_EMA_ALPHA = 0.02 // ~ last few seconds at 60 Hz
    }
}
