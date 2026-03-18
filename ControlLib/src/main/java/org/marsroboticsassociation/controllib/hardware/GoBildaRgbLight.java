package org.marsroboticsassociation.controllib.hardware;

import org.marsroboticsassociation.controllib.util.LinInterpTable;
import org.marsroboticsassociation.controllib.util.SetOnChange;

import java.util.OptionalDouble;
import java.util.function.DoubleConsumer;

/**
 * Hardware-agnostic RGB LED state machine for servo-controlled lights.
 * Supports solid colors, overrides, and timed flash sequences.
 *
 * <p>Inject a {@code DoubleConsumer} that maps [0.0, 1.0] servo positions to the
 * hardware output. Call {@link #update()} every loop iteration to advance flash state.
 */
public class GoBildaRgbLight {

    public enum Color {
        OFF(0.0), RED(0.279), ORANGE(0.333), YELLOW(0.388), SAGE(0.444),
        GREEN(0.5), AZURE(0.555), BLUE(0.611), INDIGO(0.666), VIOLET(0.722),
        WHITE(1.0);

        public final double servo;

        Color(double servo) {
            this.servo = servo;
        }
    }

    private final SetOnChange<Double> lightColor;

    private enum FlashState { IDLE, ON, OFF }

    private FlashState flashState = FlashState.IDLE;
    private long flashStartNanos = 0;

    private OptionalDouble overrideServo = OptionalDouble.empty();
    private double flashServo = 0.0;
    private double baseServo = 0.0;
    private int flashesRemaining = 0;

    private static final double FLASH_PHASE_MS = 175;

    public GoBildaRgbLight(DoubleConsumer setPosition) {
        lightColor = SetOnChange.ofDouble(0.0, 0.02, setPosition);
    }

    public void setBase(Color color) { setBase(color.servo); }
    public void setOverride(Color color) { setOverride(color.servo); }
    public void flash(Color color, int times) { flash(color.servo, times); }

    /** @param hue color wheel angle in degrees [0, 270], where 0 is red and 270 is violet */
    public void setBaseHue(double hue) { setBase(hueToServoPosition(hue)); }

    /** @param hue color wheel angle in degrees [0, 270], where 0 is red and 270 is violet */
    public void setOverrideHue(double hue) { setOverride(hueToServoPosition(hue)); }

    /** @param hue color wheel angle in degrees [0, 270], where 0 is red and 270 is violet */
    public void flashHue(double hue, int times) { flash(hueToServoPosition(hue), times); }

    private double hueToServoPosition(double hue) {
        if (hue < 0 || hue > 270) {
            throw new IllegalArgumentException("hue must be in [0, 270], got " + hue);
        }
        return LinInterpTable.linearInterpolation(
                java.lang.Math.min(270.0, java.lang.Math.max(0.0, hue)),
                0.0, 270.0, Color.RED.servo, Color.VIOLET.servo);
    }

    /**
     * Sets the steady-state servo position. During a flash sequence, this updates the
     * position shown between flashes. When no flash is active, it takes effect immediately.
     *
     * @param servoPos servo position in [0.0, 1.0]
     */
    public void setBase(double servoPos) {
        baseServo = servoPos;
        if (flashState == FlashState.IDLE) {
            lightColor.set(baseServo);
        }
    }

    /**
     * Overrides the output immediately, bypassing any active flash sequence.
     * The override persists until {@link #unsetOverride()} is called.
     *
     * @param servoPos servo position in [0.0, 1.0]
     */
    public void setOverride(double servoPos) {
        overrideServo = OptionalDouble.of(servoPos);
        lightColor.set(overrideServo.getAsDouble());
    }

    /**
     * Clears the override, restoring normal base/flash behavior on the next {@link #update()}.
     */
    public void unsetOverride() {
        overrideServo = OptionalDouble.empty();
    }

    /**
     * Starts a flash sequence. Requires {@link #update()} every loop to advance state.
     *
     * @param servoPos servo position to flash in [0.0, 1.0]
     * @param times    number of flashes
     */
    public void flash(double servoPos, int times) {
        flashServo = servoPos;
        flashesRemaining = times;
        flashState = FlashState.ON;
        flashStartNanos = System.nanoTime();
        lightColor.set(flashServo);
    }

    /**
     * Advances the flash state machine and updates the output.
     * Must be called every loop iteration for flash sequences to work correctly.
     */
    public void update() {
        if (flashState == FlashState.IDLE) {
            lightColor.set(overrideServo.isEmpty() ? baseServo : overrideServo.getAsDouble());
            return;
        }

        double elapsedMs = (System.nanoTime() - flashStartNanos) / 1_000_000.0;

        if (flashState == FlashState.ON && elapsedMs >= FLASH_PHASE_MS) {
            lightColor.set(overrideServo.isEmpty() ? baseServo : overrideServo.getAsDouble());
            flashesRemaining--;
            if (flashesRemaining <= 0) {
                flashState = FlashState.IDLE;
            } else {
                flashState = FlashState.OFF;
                flashStartNanos = System.nanoTime();
            }
        } else if (flashState == FlashState.OFF && elapsedMs >= FLASH_PHASE_MS) {
            lightColor.set(overrideServo.isEmpty() ? flashServo : overrideServo.getAsDouble());
            flashState = FlashState.ON;
            flashStartNanos = System.nanoTime();
        }
    }
}
