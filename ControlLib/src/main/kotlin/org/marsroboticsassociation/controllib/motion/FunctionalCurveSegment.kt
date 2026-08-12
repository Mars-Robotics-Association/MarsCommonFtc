package org.marsroboticsassociation.controllib.motion

import java.util.function.DoubleUnaryOperator
import kotlin.math.max
import kotlin.math.min

/** Analytic curve segment backed by value and slope functions over an absolute time interval. */
class FunctionalCurveSegment(
    private val startTime: Double,
    private val endTime: Double,
    private val valueFunction: DoubleUnaryOperator,
    private val slopeFunction: DoubleUnaryOperator,
) : TrajectoryCurveSegment {

    override fun startTime(): Double = startTime

    override fun endTime(): Double = endTime

    override fun valueAt(time: Double): Double = valueFunction.applyAsDouble(time)

    override fun slopeAt(time: Double): Double = slopeFunction.applyAsDouble(time)

    override fun minValue(): Double = sampledExtremum(false)

    override fun maxValue(): Double = sampledExtremum(true)

    override fun shiftedBy(deltaTime: Double): TrajectoryCurveSegment {
        return FunctionalCurveSegment(
            startTime + deltaTime,
            endTime + deltaTime,
            { time -> valueAt(time - deltaTime) },
            { time -> slopeAt(time - deltaTime) },
        )
    }

    override fun clippedTo(newStartTime: Double, newEndTime: Double): TrajectoryCurveSegment? {
        val clippedStart = max(startTime, newStartTime)
        val clippedEnd = min(endTime, newEndTime)
        if (clippedEnd <= clippedStart) return null
        return FunctionalCurveSegment(clippedStart, clippedEnd, this::valueAt, this::slopeAt)
    }

    private fun sampledExtremum(wantMax: Boolean): Double {
        if (duration() <= 0) return valueAt(startTime)
        var best = valueAt(startTime)
        val endValue = valueAt(endTime)
        best = if (wantMax) max(best, endValue) else min(best, endValue)
        for (i in 1 until EXTREMA_SAMPLES) {
            val t = startTime + duration() * i / EXTREMA_SAMPLES
            val value = valueAt(t)
            best = if (wantMax) max(best, value) else min(best, value)
        }
        return best
    }

    companion object {
        private const val EXTREMA_SAMPLES = 128
    }
}
