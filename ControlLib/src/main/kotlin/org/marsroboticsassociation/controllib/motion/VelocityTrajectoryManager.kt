package org.marsroboticsassociation.controllib.motion

import java.util.function.LongSupplier
import kotlin.math.abs
import kotlin.math.sign
import org.marsroboticsassociation.controllib.util.SetOnChange
import org.marsroboticsassociation.controllib.util.TelemetryAddData

class VelocityTrajectoryManager
@JvmOverloads
constructor(
    aMax: Double,
    jMax: Double,
    vChangeTolerance: Double,
    private val telemetry: TelemetryAddData,
    private val clock: LongSupplier = LongSupplier { System.nanoTime() },
) {
    private var aMax: Double = 0.0
    private var jInc: Double = 0.0
    private var jDec: Double = 0.0

    private val targetVelocity: SetOnChange<Double>
    lateinit var currentTrajectory: VelocityTrajectory
        private set

    private var startTime: Long = 0
    private var lastV: Double = 0.0
    private var lastA: Double = 0.0
    private var pendingTarget: Double = Double.NaN

    init {
        pendingTarget = Double.NaN
        updateConfig(aMax, jMax, jMax)
        lastV = 0.0
        lastA = 0.0
        startTime = clock.asLong
        changeTrajectory(0.0)
        targetVelocity =
            SetOnChange.ofDouble(0.0, vChangeTolerance) { v ->
                pendingTarget = v
                update()
            }
    }

    fun updateConfig(aMax: Double, jInc: Double, jDec: Double) {
        this.aMax = abs(aMax)
        this.jInc = abs(jInc)
        this.jDec = abs(jDec)
    }

    /**
     * Create a new SCurveVelocity trajectory from the current state to the given target velocity.
     * This method safely handles opposing accelerations and tiny Δv.
     */
    private fun changeTrajectory(vTarget: Double) {
        val dv = vTarget - lastV

        // If dv is effectively zero, create a trivial trajectory
        if (abs(dv) < 1e-6) {
            currentTrajectory = SCurveVelocity(lastV, lastV, 0.0, aMax, jInc, jDec)
        } else {
            // Compute effective starting acceleration in the direction of the target
            val dir = sign(dv)
            var effectiveA0 = lastA

            // Clamp initial acceleration to not overshoot the new target immediately
            if (dir * effectiveA0 > aMax) {
                effectiveA0 = dir * aMax
            }

            currentTrajectory = SCurveVelocity(lastV, vTarget, effectiveA0, aMax, jInc, jDec)
        }

        startTime = clock.asLong
        pendingTarget = Double.NaN
    }

    /** Update current velocity/acceleration (call once per loop) */
    fun update() {
        val seconds = (clock.asLong - startTime) / 1e9

        lastV = currentTrajectory.getVelocity(seconds)
        lastA = currentTrajectory.getAcceleration(seconds)

        telemetry.addData("trajectory velocity", "%.3f", lastV)
        telemetry.addData("trajectory acceleration", "%.1f", lastA)

        // Always update trajectory if target changed
        if (!pendingTarget.isNaN()) {
            this.changeTrajectory(pendingTarget)
        }
    }

    /**
     * Immediately discard the current trajectory and restart from a measured velocity. Acceleration
     * is assumed zero.
     */
    fun resetFromMeasurement(measuredV: Double) {
        resetFromMeasurement(measuredV, 0.0)
    }

    /**
     * Immediately discard the current trajectory and restart from measured velocity & acceleration.
     */
    fun resetFromMeasurement(measuredV: Double, measuredA: Double) {
        this.lastV = measuredV
        this.lastA = measuredA
        this.pendingTarget = Double.NaN
        changeTrajectory(target)
    }

    /** Command a new target velocity at any time */
    fun setTarget(vTarget: Double) {
        targetVelocity.setDouble(vTarget)
    }

    val target: Double
        get() = targetVelocity.get()

    val velocity: Double
        get() = lastV

    val acceleration: Double
        get() = lastA
}
