package org.marsroboticsassociation.controllib.declarativeinterlock

import java.util.function.Consumer
import java.util.function.ToDoubleFunction
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Commands semantic targets and tracks conservative completion deadlines.
 *
 * This engine type intentionally does not block or suspend. Callers command targets, then observe
 * [isSettled] on subsequent OpMode ticks.
 */
class TimedServo<T : Any> @JvmOverloads constructor(
    private val clock: SecondsClock,
    private val params: Params,
    initiallySettled: Boolean = true,
    private val commandHardware: Consumer<T>,
    private val targetPosition: ToDoubleFunction<T>? = null,
) {
    /** Mutable calibration held by reference so Dashboard edits apply to subsequent commands. */
    class Params {
        @JvmField
        var travelSeconds = 0.0

        @JvmField
        var maxVelocity = 0.0

        @JvmField
        var maxAcceleration = 0.0

        @JvmField
        var positionTolerance = 0.0
    }

    var target: T? = null
        private set

    val modeledPosition: Double?
        get() = motion.modeledPosition(clock.nowSeconds())

    private var motion =
        if (initiallySettled) {
            MotionProfile.settled(null, clock.nowSeconds())
        } else {
            MotionProfile.unmodeled(clock.nowSeconds(), currentTravelSeconds())
        }

    private var settledAt =
        if (initiallySettled) clock.nowSeconds() else clock.nowSeconds() + currentTravelSeconds()

    val isSettled: Boolean
        get() = clock.nowSeconds() >= settledAt

    @JvmOverloads
    fun command(newTarget: T, force: Boolean = false) {
        if (!force && target == newTarget) return

        val nowSeconds = clock.nowSeconds()
        motion = nextMotion(newTarget, nowSeconds)
        commandHardware.accept(newTarget)
        target = newTarget
        settledAt = nowSeconds + motion.durationSeconds
    }

    private fun currentTravelSeconds(): Double =
        params.travelSeconds.also { seconds ->
            require(seconds.isFinite() && seconds >= 0.0) {
                "travelSeconds must be finite and non-negative"
            }
        }

    private fun nextMotion(newTarget: T, nowSeconds: Double): MotionProfile {
        val positionResolver = targetPosition ?: return MotionProfile.unmodeled(nowSeconds, currentTravelSeconds())
        val endPosition = positionResolver.applyAsDouble(newTarget)
        require(endPosition.isFinite()) { "Servo target position must be finite" }
        val startPosition =
            modeledPosition ?: target?.let {
                positionResolver.applyAsDouble(it).also { position ->
                    require(position.isFinite()) { "Servo start position must be finite" }
                }
            } ?: return MotionProfile.unmodeled(nowSeconds, currentTravelSeconds(), endPosition)
        return MotionProfile.trapezoidal(
            startPosition = startPosition,
            endPosition = endPosition,
            startTimeSeconds = nowSeconds,
            params = params,
        )
    }

    private class MotionProfile private constructor(
        private val startPosition: Double?,
        private val endPosition: Double?,
        private val startTimeSeconds: Double,
        val durationSeconds: Double,
        private val accelerationTimeSeconds: Double,
        private val cruiseTimeSeconds: Double,
        private val direction: Double,
        private val maxVelocity: Double,
        private val maxAcceleration: Double,
    ) {
        fun modeledPosition(nowSeconds: Double): Double? {
            val end = endPosition ?: return null
            val start = startPosition ?: return if (nowSeconds >= startTimeSeconds + durationSeconds) end else null
            if (durationSeconds <= 0.0) return end

            val elapsed = (nowSeconds - startTimeSeconds).coerceIn(0.0, durationSeconds)
            val distance =
                when {
                    elapsed <= accelerationTimeSeconds ->
                        0.5 * maxAcceleration * elapsed * elapsed
                    elapsed <= accelerationTimeSeconds + cruiseTimeSeconds -> {
                        val cruiseElapsed = elapsed - accelerationTimeSeconds
                        0.5 * maxAcceleration * accelerationTimeSeconds * accelerationTimeSeconds +
                            maxVelocity * cruiseElapsed
                    }
                    else -> {
                        val decelElapsed = elapsed - accelerationTimeSeconds - cruiseTimeSeconds
                        val accelDistance =
                            0.5 * maxAcceleration * accelerationTimeSeconds * accelerationTimeSeconds
                        val cruiseDistance = maxVelocity * cruiseTimeSeconds
                        accelDistance + cruiseDistance +
                            maxVelocity * decelElapsed -
                            0.5 * maxAcceleration * decelElapsed * decelElapsed
                    }
                }
            return start + direction * distance
        }

        companion object {
            fun settled(position: Double?, nowSeconds: Double): MotionProfile =
                MotionProfile(position, position, nowSeconds, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0)

            fun unmodeled(
                nowSeconds: Double,
                durationSeconds: Double,
                endPosition: Double? = null,
            ): MotionProfile =
                MotionProfile(null, endPosition, nowSeconds, durationSeconds, 0.0, 0.0, 1.0, 0.0, 0.0)

            fun trapezoidal(
                startPosition: Double,
                endPosition: Double,
                startTimeSeconds: Double,
                params: Params,
            ): MotionProfile {
                val tolerance = checkedNonNegative(params.positionTolerance, "positionTolerance")
                val distance = abs(endPosition - startPosition)
                if (distance <= tolerance) {
                    return settled(endPosition, startTimeSeconds)
                }

                val maxVelocity = checkedPositive(params.maxVelocity, "maxVelocity")
                val maxAcceleration = checkedPositive(params.maxAcceleration, "maxAcceleration")
                val accelerationTime = maxVelocity / maxAcceleration
                val accelerationDistance = 0.5 * maxAcceleration * accelerationTime * accelerationTime
                val direction = if (endPosition >= startPosition) 1.0 else -1.0

                return if (distance <= 2.0 * accelerationDistance) {
                    val triangularAccelerationTime = sqrt(distance / maxAcceleration)
                    MotionProfile(
                        startPosition,
                        endPosition,
                        startTimeSeconds,
                        2.0 * triangularAccelerationTime,
                        triangularAccelerationTime,
                        0.0,
                        direction,
                        maxAcceleration * triangularAccelerationTime,
                        maxAcceleration,
                    )
                } else {
                    val cruiseDistance = distance - 2.0 * accelerationDistance
                    val cruiseTime = cruiseDistance / maxVelocity
                    MotionProfile(
                        startPosition,
                        endPosition,
                        startTimeSeconds,
                        2.0 * accelerationTime + cruiseTime,
                        accelerationTime,
                        cruiseTime,
                        direction,
                        maxVelocity,
                        maxAcceleration,
                    )
                }
            }

            private fun checkedPositive(value: Double, name: String): Double {
                require(value.isFinite() && value > 0.0) { "$name must be finite and positive" }
                return value
            }

            private fun checkedNonNegative(value: Double, name: String): Double {
                require(value.isFinite() && value >= 0.0) { "$name must be finite and non-negative" }
                return value
            }
        }
    }
}
