package org.marsroboticsassociation.controllib.motion

interface TrajectoryCurveSegment {
    fun startTime(): Double

    fun endTime(): Double

    fun duration(): Double = endTime() - startTime()

    fun valueAt(time: Double): Double

    fun slopeAt(time: Double): Double

    fun minValue(): Double

    fun maxValue(): Double

    fun shiftedBy(deltaTime: Double): TrajectoryCurveSegment

    fun clippedTo(newStartTime: Double, newEndTime: Double): TrajectoryCurveSegment?
}
