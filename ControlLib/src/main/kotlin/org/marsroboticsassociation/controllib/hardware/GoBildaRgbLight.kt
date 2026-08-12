package org.marsroboticsassociation.controllib.hardware

import java.util.OptionalDouble
import java.util.function.DoubleConsumer
import kotlin.math.max
import kotlin.math.min
import org.marsroboticsassociation.controllib.util.LinInterpTable
import org.marsroboticsassociation.controllib.util.SetOnChange

/**
 * Hardware-agnostic RGB LED state machine for servo-controlled lights. Supports solid colors,
 * overrides, and timed flash sequences.
 *
 * <p>Inject a [DoubleConsumer] that maps [0.0, 1.0] servo positions to the hardware output. Call
 * [update] every loop iteration to advance flash state.
 */
class GoBildaRgbLight(setPosition: DoubleConsumer) {

    enum class Color(val servo: Double) {
        OFF(0.0),
        RED(0.279),
        ORANGE(0.333),
        YELLOW(0.388),
        SAGE(0.444),
        GREEN(0.5),
        AZURE(0.555),
        BLUE(0.611),
        INDIGO(0.666),
        VIOLET(0.722),
        WHITE(1.0),
    }

    private val lightColor = SetOnChange.ofDouble(0.0, 0.02, setPosition)

    private enum class FlashState {
        IDLE,
        ON,
        OFF,
    }

    private var flashState = FlashState.IDLE
    private var flashStartNanos = 0L

    private var overrideServo: OptionalDouble = OptionalDouble.empty()
    private var flashServo = 0.0
    private var baseServo = 0.0
    private var flashesRemaining = 0

    fun setBase(color: Color) {
        setBase(color.servo)
    }

    fun setOverride(color: Color) {
        setOverride(color.servo)
    }

    fun flash(color: Color, times: Int) {
        flash(color.servo, times)
    }

    /** @param hue color wheel angle in degrees [0, 270], where 0 is red and 270 is violet */
    fun setBaseHue(hue: Double) {
        setBase(hueToServoPosition(hue))
    }

    /** @param hue color wheel angle in degrees [0, 270], where 0 is red and 270 is violet */
    fun setOverrideHue(hue: Double) {
        setOverride(hueToServoPosition(hue))
    }

    /** @param hue color wheel angle in degrees [0, 270], where 0 is red and 270 is violet */
    fun flashHue(hue: Double, times: Int) {
        flash(hueToServoPosition(hue), times)
    }

    private fun hueToServoPosition(hue: Double): Double {
        require(hue in 0.0..270.0) { "hue must be in [0, 270], got $hue" }
        return LinInterpTable.linearInterpolation(
            min(270.0, max(0.0, hue)),
            0.0,
            270.0,
            Color.RED.servo,
            Color.VIOLET.servo,
        )
    }

    /**
     * Sets the steady-state servo position. During a flash sequence, this updates the position
     * shown between flashes. When no flash is active, it takes effect immediately.
     *
     * @param servoPos servo position in [0.0, 1.0]
     */
    fun setBase(servoPos: Double) {
        baseServo = servoPos
        if (flashState == FlashState.IDLE) {
            lightColor.setDouble(baseServo)
        }
    }

    /**
     * Overrides the output immediately, bypassing any active flash sequence. The override persists
     * until [unsetOverride] is called.
     *
     * @param servoPos servo position in [0.0, 1.0]
     */
    fun setOverride(servoPos: Double) {
        overrideServo = OptionalDouble.of(servoPos)
        lightColor.setDouble(overrideServo.asDouble)
    }

    /** Clears the override, restoring normal base/flash behavior on the next [update]. */
    fun unsetOverride() {
        overrideServo = OptionalDouble.empty()
    }

    /**
     * Starts a flash sequence. Requires [update] every loop to advance state.
     *
     * @param servoPos servo position to flash in [0.0, 1.0]
     * @param times number of flashes
     */
    fun flash(servoPos: Double, times: Int) {
        flashServo = servoPos
        flashesRemaining = times
        flashState = FlashState.ON
        flashStartNanos = System.nanoTime()
        lightColor.setDouble(flashServo)
    }

    /**
     * Advances the flash state machine and updates the output. Must be called every loop iteration
     * for flash sequences to work correctly.
     */
    fun update() {
        if (flashState == FlashState.IDLE) {
            lightColor.setDouble(if (overrideServo.isEmpty) baseServo else overrideServo.asDouble)
            return
        }

        val elapsedMs = (System.nanoTime() - flashStartNanos) / 1_000_000.0

        if (flashState == FlashState.ON && elapsedMs >= FLASH_PHASE_MS) {
            lightColor.setDouble(if (overrideServo.isEmpty) baseServo else overrideServo.asDouble)
            flashesRemaining--
            if (flashesRemaining <= 0) {
                flashState = FlashState.IDLE
            } else {
                flashState = FlashState.OFF
                flashStartNanos = System.nanoTime()
            }
        } else if (flashState == FlashState.OFF && elapsedMs >= FLASH_PHASE_MS) {
            lightColor.setDouble(if (overrideServo.isEmpty) flashServo else overrideServo.asDouble)
            flashState = FlashState.ON
            flashStartNanos = System.nanoTime()
        }
    }

    companion object {
        private const val FLASH_PHASE_MS = 175.0
    }
}
