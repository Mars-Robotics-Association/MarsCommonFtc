package org.marsroboticsassociation.controllib.motion

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

/**
 * Piecewise polynomial segment over an absolute time interval.
 *
 * <p>The polynomial is evaluated in local time `dt = t - startTime`:
 * <pre>
 * value(dt) = c0 + c1*dt + c2*dt^2 + c3*dt^3
 * </pre>
 */
data class PolynomialCurveSegment(
    private val startTime: Double,
    private val endTime: Double,
    val c0: Double,
    val c1: Double,
    val c2: Double,
    val c3: Double,
) : TrajectoryCurveSegment {

    override fun startTime(): Double = startTime

    override fun endTime(): Double = endTime

    override fun duration(): Double = endTime - startTime

    override fun valueAt(time: Double): Double {
        val dt = time - startTime
        return ((c3 * dt + c2) * dt + c1) * dt + c0
    }

    override fun slopeAt(time: Double): Double {
        val dt = time - startTime
        return (3.0 * c3 * dt + 2.0 * c2) * dt + c1
    }

    override fun minValue(): Double = extremum(false)

    override fun maxValue(): Double = extremum(true)

    override fun shiftedBy(deltaTime: Double): PolynomialCurveSegment {
        return PolynomialCurveSegment(startTime + deltaTime, endTime + deltaTime, c0, c1, c2, c3)
    }

    override fun clippedTo(newStartTime: Double, newEndTime: Double): PolynomialCurveSegment? {
        val clippedStart = max(startTime, newStartTime)
        val clippedEnd = min(endTime, newEndTime)
        if (clippedEnd <= clippedStart) return null

        val h = clippedStart - startTime
        val newC0 = valueAt(clippedStart)
        val newC1 = slopeAt(clippedStart)
        val newC2 = c2 + 3.0 * c3 * h
        return PolynomialCurveSegment(clippedStart, clippedEnd, newC0, newC1, newC2, c3)
    }

    private fun extremum(wantMax: Boolean): Double {
        var best = valueAt(startTime)
        val endValue = valueAt(endTime)
        best = if (wantMax) max(best, endValue) else min(best, endValue)

        val duration = duration()
        if (duration <= 0) return best

        if (abs(c3) < 1e-12) {
            if (abs(c2) < 1e-12) return best
            val dt = -c1 / (2.0 * c2)
            if (dt > 0 && dt < duration) {
                val value = ((c3 * dt + c2) * dt + c1) * dt + c0
                best = if (wantMax) max(best, value) else min(best, value)
            }
            return best
        }

        val a = 3.0 * c3
        val b = 2.0 * c2
        val c = c1
        val disc = b * b - 4.0 * a * c
        if (disc < 0) return best
        val sqrtDisc = sqrt(disc)
        val dt1 = (-b + sqrtDisc) / (2.0 * a)
        val dt2 = (-b - sqrtDisc) / (2.0 * a)
        if (dt1 > 0 && dt1 < duration) {
            val value = ((c3 * dt1 + c2) * dt1 + c1) * dt1 + c0
            best = if (wantMax) max(best, value) else min(best, value)
        }
        if (dt2 > 0 && dt2 < duration) {
            val value = ((c3 * dt2 + c2) * dt2 + c1) * dt2 + c0
            best = if (wantMax) max(best, value) else min(best, value)
        }
        return best
    }
}
