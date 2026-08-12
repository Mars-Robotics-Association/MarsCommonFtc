package org.marsroboticsassociation.controllab.trajectory

import java.awt.BasicStroke
import java.awt.Color
import java.util.function.LongSupplier
import kotlin.math.abs
import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment
import org.marsroboticsassociation.controllib.motion.PositionTrajectoryManager
import org.marsroboticsassociation.controllib.motion.SCurvePosition
import org.marsroboticsassociation.controllib.motion.SCurveVelocity
import org.marsroboticsassociation.controllib.motion.SinCurvePosition
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment
import org.marsroboticsassociation.controllib.motion.VelocityTrajectoryManager
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Simulation engine that drives one trajectory planner at 20 ms per tick. Designed to be called
 * from the Swing EDT by a javax.swing.Timer.
 *
 * <p>Params are staged via setXxxParams() and applied to the underlying planner only when
 * applyParamsAndGoTo() is called (button press).
 */
class TrajectoryEngine(type: TrajectoryType) {

    // ------------------------------------------------------------------
    // State
    // ------------------------------------------------------------------
    private var type: TrajectoryType = type

    // --- SCurvePosition ---
    private var posManager: PositionTrajectoryManager? = null
    private var pendingVMax = 10.0
    private var pendingAAccel = 5.0
    private var pendingADecel = 5.0
    private var pendingJMax = 50.0

    // --- SCurveVelocity ---
    private var velManager: VelocityTrajectoryManager? = null
    private var pendingAMax = 1197.0
    private var pendingJInc = 2669.0
    private var pendingJDec = 800.0

    // --- Simulated wall clock (nanoseconds) ---
    private var clockNs = 0L

    // --- Cached outputs ---
    private var lastP = 0.0
    private var lastV = 0.0
    private var lastA = 0.0
    private var currentTarget = 0.0
    private var moving = false

    init {
        buildPlanner()
    }

    // ------------------------------------------------------------------
    // Param staging (call any time; applied only on next button press)
    // ------------------------------------------------------------------

    fun setPositionParams(vMax: Double, aAccel: Double, aDecel: Double, jMax: Double) {
        pendingVMax = vMax
        pendingAAccel = aAccel
        pendingADecel = aDecel
        pendingJMax = jMax
    }

    fun setVelocityParams(aMax: Double, jInc: Double, jDec: Double) {
        pendingAMax = aMax
        pendingJInc = jInc
        pendingJDec = jDec
    }

    // ------------------------------------------------------------------
    // Button-press action: apply staged params + command new target
    // ------------------------------------------------------------------

    fun applyParamsAndGoTo(target: Double) {
        currentTarget = target
        moving = true
        when (type) {
            TrajectoryType.SCURVE_POSITION,
            TrajectoryType.SIN_CURVE_POSITION -> {
                val pos = posManager!!
                pos.updateConfig(pendingVMax, pendingAAccel, pendingADecel, pendingJMax)
                pos.setTarget(target)
                pos.update()
                lastP = pos.getPosition()
                lastV = pos.getVelocity()
                lastA = pos.getAcceleration()
            }
            TrajectoryType.SCURVE_VELOCITY -> {
                val vel = velManager!!
                vel.updateConfig(pendingAMax, pendingJInc, pendingJDec)
                vel.setTarget(target)
                vel.update()
                lastP = 0.0
                lastV = vel.getVelocity()
                lastA = vel.getAcceleration()
            }
        }
    }

    fun reset() {
        lastP = 0.0
        lastV = 0.0
        lastA = 0.0
        currentTarget = 0.0
        moving = false
        clockNs = 0
        val vel = velManager
        if (vel != null) {
            vel.updateConfig(pendingAMax, pendingJInc, pendingJDec)
            vel.resetFromMeasurement(0.0, 0.0)
            lastV = vel.getVelocity()
            lastA = vel.getAcceleration()
        }
    }

    // ------------------------------------------------------------------
    // Simulation tick (call every 20 ms from Swing Timer)
    // ------------------------------------------------------------------

    fun tick() {
        if (!moving) return
        clockNs += CYCLE_NS

        when (type) {
            TrajectoryType.SCURVE_POSITION,
            TrajectoryType.SIN_CURVE_POSITION -> {
                val pos = posManager!!
                pos.update()
                lastP = pos.getPosition()
                lastV = pos.getVelocity()
                lastA = pos.getAcceleration()
                if (abs(lastP - currentTarget) < 0.01 && abs(lastV) < 0.01) {
                    moving = false
                }
            }
            TrajectoryType.SCURVE_VELOCITY -> {
                val vel = velManager!!
                vel.update()
                lastP = 0.0
                lastV = vel.getVelocity()
                lastA = vel.getAcceleration()
                if (abs(lastV - currentTarget) < 1.0 && abs(lastA) < 1.0) {
                    moving = false
                }
            }
        }
    }

    // ------------------------------------------------------------------
    // Lifecycle
    // ------------------------------------------------------------------

    /** Release native resources. Call when the application is shutting down. */
    fun dispose() {
        tearDownPlanner()
    }

    // ------------------------------------------------------------------
    // Accessors
    // ------------------------------------------------------------------

    fun getPosition(): Double = lastP

    fun getVelocity(): Double = lastV

    fun getAcceleration(): Double = lastA

    fun isMoving(): Boolean = moving

    fun getTarget(): Double = currentTarget

    fun hasPosition(): Boolean = type != TrajectoryType.SCURVE_VELOCITY

    fun getType(): TrajectoryType = type

    fun supportsExactSvgExport(): Boolean =
        type == TrajectoryType.SCURVE_POSITION ||
            type == TrajectoryType.SCURVE_VELOCITY ||
            type == TrajectoryType.SIN_CURVE_POSITION

    fun buildExactSvgModel(): TrajectorySvgModel? {
        if (!supportsExactSvgExport()) return null

        return when (type) {
            TrajectoryType.SCURVE_POSITION -> buildPositionSvgModel()
            TrajectoryType.SIN_CURVE_POSITION -> buildSinPositionSvgModel()
            TrajectoryType.SCURVE_VELOCITY -> buildVelocitySvgModel()
        }
    }

    // ------------------------------------------------------------------
    // Type switch: tear down old planner, build new one
    // ------------------------------------------------------------------

    fun switchType(newType: TrajectoryType) {
        if (type == newType) return
        tearDownPlanner()
        type = newType
        moving = false
        lastP = 0.0
        lastV = 0.0
        lastA = 0.0
        clockNs = 0
        buildPlanner()
    }

    // ------------------------------------------------------------------
    // Internal planner lifecycle
    // ------------------------------------------------------------------

    private fun buildPlanner() {
        when (type) {
            TrajectoryType.SCURVE_POSITION -> {
                posManager =
                    PositionTrajectoryManager(
                        pendingVMax,
                        pendingAAccel,
                        pendingADecel,
                        pendingJMax,
                        0.001,
                        NO_OP,
                        LongSupplier { clockNs },
                    )
            }
            TrajectoryType.SIN_CURVE_POSITION -> {
                posManager =
                    PositionTrajectoryManager(
                        pendingVMax,
                        pendingAAccel,
                        pendingADecel,
                        pendingJMax,
                        0.001,
                        NO_OP,
                        LongSupplier { clockNs },
                        PositionTrajectoryManager.TrajectoryFactory(::SinCurvePosition),
                    )
            }
            TrajectoryType.SCURVE_VELOCITY -> {
                val vel =
                    VelocityTrajectoryManager(
                        pendingAMax,
                        pendingJInc,
                        1.0,
                        NO_OP,
                        LongSupplier { clockNs },
                    )
                vel.updateConfig(pendingAMax, pendingJInc, pendingJDec)
                velManager = vel
            }
        }
    }

    private fun tearDownPlanner() {
        posManager = null
        velManager = null
    }

    private fun buildPositionSvgModel(): TrajectorySvgModel? {
        val profile = posManager!!.getCurrentTrajectory() as? SCurvePosition ?: return null
        val series = ArrayList<TrajectorySvgSeries>()
        series.add(
            TrajectorySvgSeries(
                "Position (units)",
                asCurveSegments(profile.positionSegments()),
                chartColor(0),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Velocity (units/s)",
                asCurveSegments(profile.velocitySegments()),
                chartColor(1),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Acceleration (units/s\u00b2)",
                asCurveSegments(profile.accelerationSegments()),
                chartColor(2),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Target",
                horizontalSegments(profile.getTotalTime(), currentTarget),
                chartColor(3),
                1.5f,
                BasicStroke(
                    1.5f,
                    BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,
                    10.0f,
                    floatArrayOf(6.0f, 4.0f),
                    0.0f,
                ),
            )
        )
        return TrajectorySvgModel(0.0, profile.getTotalTime(), minY(series), maxY(series), series)
    }

    private fun buildVelocitySvgModel(): TrajectorySvgModel? {
        val profile = velManager!!.getCurrentTrajectory() as? SCurveVelocity ?: return null
        val series = ArrayList<TrajectorySvgSeries>()
        series.add(
            TrajectorySvgSeries(
                "Velocity (units/s)",
                asCurveSegments(profile.velocitySegments()),
                chartColor(1),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Acceleration (units/s\u00b2)",
                asCurveSegments(profile.accelerationSegments()),
                chartColor(2),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Target",
                horizontalSegments(profile.getTotalTime(), currentTarget),
                chartColor(3),
                1.5f,
                BasicStroke(
                    1.5f,
                    BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,
                    10.0f,
                    floatArrayOf(6.0f, 4.0f),
                    0.0f,
                ),
            )
        )
        return TrajectorySvgModel(0.0, profile.getTotalTime(), minY(series), maxY(series), series)
    }

    private fun buildSinPositionSvgModel(): TrajectorySvgModel? {
        val profile = posManager!!.getCurrentTrajectory() as? SinCurvePosition ?: return null
        val series = ArrayList<TrajectorySvgSeries>()
        series.add(
            TrajectorySvgSeries(
                "Position (units)",
                profile.positionSegments(),
                chartColor(0),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Velocity (units/s)",
                profile.velocitySegments(),
                chartColor(1),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Acceleration (units/s\u00b2)",
                profile.accelerationSegments(),
                chartColor(2),
                2.0f,
                null,
            )
        )
        series.add(
            TrajectorySvgSeries(
                "Target",
                horizontalSegments(profile.getTotalTime(), currentTarget),
                chartColor(3),
                1.5f,
                BasicStroke(
                    1.5f,
                    BasicStroke.CAP_BUTT,
                    BasicStroke.JOIN_MITER,
                    10.0f,
                    floatArrayOf(6.0f, 4.0f),
                    0.0f,
                ),
            )
        )
        return TrajectorySvgModel(0.0, profile.getTotalTime(), minY(series), maxY(series), series)
    }

    companion object {
        private val NO_OP = TelemetryAddData { _, _, _ -> }

        @JvmField val CYCLE_S = 0.020 // 20 ms

        private val CYCLE_NS = (CYCLE_S * 1e9).toLong()

        private fun horizontalSegments(
            totalTime: Double,
            value: Double,
        ): List<TrajectoryCurveSegment> {
            if (totalTime <= 0) return emptyList()
            return listOf(PolynomialCurveSegment(0.0, totalTime, value, 0.0, 0.0, 0.0))
        }

        private fun asCurveSegments(
            segments: List<TrajectoryCurveSegment>
        ): List<TrajectoryCurveSegment> = ArrayList(segments)

        private fun minY(seriesList: List<TrajectorySvgSeries>): Double {
            var min = Double.POSITIVE_INFINITY
            for (series in seriesList) {
                for (segment in series.segments) {
                    min = minOf(min, segment.minValue())
                }
            }
            return if (min.isFinite()) min else -1.0
        }

        private fun maxY(seriesList: List<TrajectorySvgSeries>): Double {
            var max = Double.NEGATIVE_INFINITY
            for (series in seriesList) {
                for (segment in series.segments) {
                    max = maxOf(max, segment.maxValue())
                }
            }
            return if (max.isFinite()) max else 1.0
        }

        private fun chartColor(index: Int): Color {
            val colors =
                arrayOf(
                    Color(0x2E, 0x86, 0xDE),
                    Color(0xE6, 0x7E, 0x22),
                    Color(0x27, 0xAE, 0x60),
                    Color(0x95, 0xA5, 0xA6),
                )
            return colors[Math.floorMod(index, colors.size)]
        }
    }
}
