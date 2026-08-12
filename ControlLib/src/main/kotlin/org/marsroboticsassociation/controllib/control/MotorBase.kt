package org.marsroboticsassociation.controllib.control

import edu.wpi.first.math.MathUtil
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Abstract base class for controlling a motor (or motor group via `LinkedMotorGroup`).
 *
 * Accepts a single [IMotor] — multi-motor grouping is handled externally. Provides quantized power
 * updates to reduce unnecessary motor commands.
 */
abstract class MotorBase(
    protected val telemetry: TelemetryAddData,
    protected val gearRatio: Double,
    protected val motorPPR: Double,
    motorPowerChangeTolerance: Double,
    protected val motor: IMotor,
) {
    protected val name: String = motor.name
    private var lastPower: Double = Double.NaN
    private val quantStep: Double = motorPowerChangeTolerance

    /**
     * Sets power to the motor, only if the quantized value changes from the last command. Power is
     * clamped to [-1, 1] and quantized to reduce command noise.
     */
    open fun setPower(power: Double) {
        var p = power
        if (p.isNaN()) {
            telemetry.addData("power", "%s", "NaN")
            return
        } else if (p.isInfinite()) {
            telemetry.addData("power", "%s", "Infinite")
            return
        } else {
            telemetry.addData("power", "%.2f", p)
        }

        p = MathUtil.clamp(p, -1.0, 1.0)
        val quantPower = Math.round(p / quantStep) * quantStep
        if (lastPower.isNaN() || quantPower != lastPower) {
            motor.setPower(quantPower)
            lastPower = quantPower
        }
    }

    fun getVoltage(): Double = motor.hubVoltage

    fun update(dt: Double) {
        if (dt < 1e-6) return
        updateInternal(dt)
    }

    protected abstract fun updateInternal(dt: Double)

    abstract fun writeTelemetry()

    abstract fun stop()
}
