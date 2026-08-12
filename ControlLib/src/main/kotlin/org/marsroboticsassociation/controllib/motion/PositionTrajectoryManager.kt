package org.marsroboticsassociation.controllib.motion

import java.util.function.LongSupplier
import kotlin.math.abs
import org.marsroboticsassociation.controllib.util.SetOnChange
import org.marsroboticsassociation.controllib.util.TelemetryAddData

class PositionTrajectoryManager {

    /**
     * Factory for creating [PositionTrajectory] instances. Implement this to swap in a different
     * trajectory type (e.g. [SinCurvePosition]) without changing the manager.
     *
     * <p>Example usage:
     * <pre>
     *   PositionTrajectoryManager(..., ::SinCurvePosition)
     * </pre>
     */
    fun interface TrajectoryFactory {
        fun create(
            p0: Double,
            pTarget: Double,
            v0: Double,
            a0: Double,
            vMax: Double,
            aMaxAccel: Double,
            aMaxDecel: Double,
            jMax: Double,
        ): PositionTrajectory
    }

    private var vMax: Double = 0.0
    private var aMaxAccel: Double = 0.0
    private var aMaxDecel: Double = 0.0
    private var jMax: Double = 0.0

    private val clock: LongSupplier
    private val telemetry: TelemetryAddData
    private val factory: TrajectoryFactory
    private val targetPosition: SetOnChange<Double>
    private lateinit var currentTrajectory: PositionTrajectory
    private var startTime: Long = 0
    private var lastP: Double = 0.0
    private var lastV: Double = 0.0
    private var lastA: Double = 0.0
    private var pendingTarget: Double = Double.NaN

    /** Production constructor — uses System.nanoTime as the clock and SCurvePosition. */
    constructor(
        vMax: Double,
        aMaxAccel: Double,
        aMaxDecel: Double,
        jMax: Double,
        pChangeTolerance: Double,
        telemetry: TelemetryAddData,
    ) : this(
        vMax,
        aMaxAccel,
        aMaxDecel,
        jMax,
        pChangeTolerance,
        telemetry,
        LongSupplier { System.nanoTime() },
        TrajectoryFactory { p0, pTarget, v0, a0, vMax, aMaxAccel, aMaxDecel, jMax ->
            SCurvePosition(p0, pTarget, v0, a0, vMax, aMaxAccel, aMaxDecel, jMax)
        },
    )

    /** Test constructor — allows injecting a simulated clock. Uses SCurvePosition. */
    constructor(
        vMax: Double,
        aMaxAccel: Double,
        aMaxDecel: Double,
        jMax: Double,
        pChangeTolerance: Double,
        telemetry: TelemetryAddData,
        clock: LongSupplier,
    ) : this(
        vMax,
        aMaxAccel,
        aMaxDecel,
        jMax,
        pChangeTolerance,
        telemetry,
        clock,
        TrajectoryFactory { p0, pTarget, v0, a0, vMax, aMaxAccel, aMaxDecel, jMax ->
            SCurvePosition(p0, pTarget, v0, a0, vMax, aMaxAccel, aMaxDecel, jMax)
        },
    )

    /** Production constructor with custom trajectory factory. Uses System.nanoTime as the clock. */
    constructor(
        vMax: Double,
        aMaxAccel: Double,
        aMaxDecel: Double,
        jMax: Double,
        pChangeTolerance: Double,
        telemetry: TelemetryAddData,
        factory: TrajectoryFactory,
    ) : this(
        vMax,
        aMaxAccel,
        aMaxDecel,
        jMax,
        pChangeTolerance,
        telemetry,
        LongSupplier { System.nanoTime() },
        factory,
    )

    /** Base constructor — all others delegate here. */
    constructor(
        vMax: Double,
        aMaxAccel: Double,
        aMaxDecel: Double,
        jMax: Double,
        pChangeTolerance: Double,
        telemetry: TelemetryAddData,
        clock: LongSupplier,
        factory: TrajectoryFactory,
    ) {
        this.clock = clock
        this.telemetry = telemetry
        this.factory = factory
        pendingTarget = Double.NaN
        updateConfig(vMax, aMaxAccel, aMaxDecel, jMax)
        lastP = 0.0
        lastV = 0.0
        lastA = 0.0
        startTime = clock.asLong
        changeTrajectory(0.0)
        targetPosition =
            SetOnChange.ofDouble(0.0, pChangeTolerance) { p ->
                pendingTarget = p
                update()
            }
    }

    /** Update motion limits. Takes effect on the next trajectory plan. */
    fun updateConfig(vMax: Double, aMaxAccel: Double, aMaxDecel: Double, jMax: Double) {
        this.vMax = abs(vMax)
        this.aMaxAccel = abs(aMaxAccel)
        this.aMaxDecel = abs(aMaxDecel)
        this.jMax = abs(jMax)
    }

    /**
     * Plan a new trajectory from the currently sampled state to the given target position.
     *
     * <p>The manager carries forward position, velocity, and acceleration only. This makes
     * mid-motion replans continuous in `p/v/a`, but not necessarily in jerk.
     */
    private fun changeTrajectory(pTarget: Double) {
        currentTrajectory =
            factory.create(lastP, pTarget, lastV, lastA, vMax, aMaxAccel, aMaxDecel, jMax)
        startTime = clock.asLong
        pendingTarget = Double.NaN
    }

    /**
     * Sample the trajectory at the current time and cache position/velocity/acceleration. Call once
     * per control loop.
     */
    fun update() {
        val seconds = (clock.asLong - startTime) / 1e9

        lastP = currentTrajectory.getPosition(seconds)
        lastV = currentTrajectory.getVelocity(seconds)
        lastA = currentTrajectory.getAcceleration(seconds)

        telemetry.addData("trajectory position", "%.3f", lastP)
        telemetry.addData("trajectory velocity", "%.3f", lastV)
        telemetry.addData("trajectory acceleration", "%.1f", lastA)

        if (!pendingTarget.isNaN()) {
            changeTrajectory(pendingTarget)
        }
    }

    /**
     * Discard the current trajectory and restart from a directly measured state. Acceleration is
     * assumed zero.
     */
    fun resetFromMeasurement(measuredP: Double, measuredV: Double) {
        resetFromMeasurement(measuredP, measuredV, 0.0)
    }

    /**
     * Discard the current trajectory and restart from directly measured position, velocity, and
     * acceleration.
     */
    fun resetFromMeasurement(measuredP: Double, measuredV: Double, measuredA: Double) {
        this.lastP = measuredP
        this.lastV = measuredV
        this.lastA = measuredA
        this.pendingTarget = Double.NaN
        changeTrajectory(targetPosition.get())
    }

    /**
     * Command a new target position.
     *
     * <p>If the target changes while a trajectory is already in progress, the next plan begins from
     * the sampled current `p/v/a` state. That preserves position, velocity, and acceleration
     * continuity across the replan, while still allowing the new trajectory to choose its own jerk
     * profile from that state.
     */
    fun setTarget(pTarget: Double) {
        targetPosition.setDouble(pTarget)
    }

    fun getTarget(): Double = targetPosition.get()

    fun getPosition(): Double = lastP

    fun getVelocity(): Double = lastV

    fun getAcceleration(): Double = lastA

    fun getCurrentTrajectory(): PositionTrajectory = currentTrajectory
}
