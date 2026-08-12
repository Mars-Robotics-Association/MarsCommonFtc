package org.marsroboticsassociation.controllib.control

import edu.wpi.first.math.MathUtil
import java.util.function.LongSupplier
import kotlin.math.abs
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.util.SetOnChange
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Velocity-controlled motor using the SDK's built-in PIDF controller.
 *
 * The motor must be set to `RUN_USING_ENCODER` mode before being passed to this constructor. Motor
 * direction and zero-power behavior are configured externally on the [IMotor].
 *
 * [setPower] is not supported; use [setRPM]/[setTPS] instead.
 */
class VelocityMotorSdkPidf : VelocityMotorBase {

    class MotorPIDFConfig(maxTPS: Double) {
        @JvmField var Kv: Double = 32767.0 / maxTPS
        @JvmField var nominalVoltage: Double = 12.0
        @JvmField var Kp: Double = 0.1 * Kv
        @JvmField var Ki: Double = 0.1 * Kp
        @JvmField var Kd: Double = 0.01 * Kp
        @JvmField var maxSettableVelocity: Double = 0.8 * maxTPS
        @JvmField var maxAccel: Double = 1196.7
        @JvmField var jerkIncreasing: Double = 2669.2
        @JvmField var jerkDecreasing: Double = 800.0
        @JvmField var lpfCutoff: Double = 4.0
    }

    @JvmField val config: MotorPIDFConfig

    private val motorVelocitySetpoint: SetOnChange<Double>
    private val voltageFactor: SetOnChange<Double>
    private val Kp: SetOnChange<Double>
    private val Ki: SetOnChange<Double>
    private val Kd: SetOnChange<Double>
    private val Kv: SetOnChange<Double>

    constructor(
        telemetry: TelemetryAddData,
        gearRatio: Double,
        motorPPR: Double,
        config: MotorPIDFConfig,
        motor: IMotor,
    ) : super(
        telemetry,
        gearRatio,
        motorPPR,
        0.05,
        motor,
        config.maxAccel,
        config.jerkIncreasing,
        config.jerkDecreasing,
        1.0,
    ) {
        this.config = config
        motorVelocitySetpoint = SetOnChange.ofDouble(0.0, 1.0, motor::setVelocity)
        val updatePIDF = Runnable {
            motor.setVelocityPIDFCoefficients(
                config.Kp,
                config.Ki,
                config.Kd,
                config.Kv * config.nominalVoltage / getVoltage(),
            )
        }
        voltageFactor =
            SetOnChange.ofDouble(config.nominalVoltage / getVoltage(), 0.025) { updatePIDF.run() }
        Kp = SetOnChange.ofDouble(config.Kp) { updatePIDF.run() }
        Ki = SetOnChange.ofDouble(config.Ki) { updatePIDF.run() }
        Kd = SetOnChange.ofDouble(config.Kd) { updatePIDF.run() }
        Kv = SetOnChange.ofDouble(config.Kv) { updatePIDF.run() }
    }

    /** Package-visible constructor for tests — injects a custom clock. */
    internal constructor(
        telemetry: TelemetryAddData,
        gearRatio: Double,
        motorPPR: Double,
        config: MotorPIDFConfig,
        motor: IMotor,
        clock: LongSupplier,
    ) : super(
        telemetry,
        gearRatio,
        motorPPR,
        0.05,
        motor,
        config.maxAccel,
        config.jerkIncreasing,
        config.jerkDecreasing,
        1.0,
        clock,
    ) {
        this.config = config
        motorVelocitySetpoint = SetOnChange.ofDouble(0.0, 1.0, motor::setVelocity)
        val updatePIDF = Runnable {
            motor.setVelocityPIDFCoefficients(
                config.Kp,
                config.Ki,
                config.Kd,
                config.Kv * config.nominalVoltage / getVoltage(),
            )
        }
        voltageFactor =
            SetOnChange.ofDouble(config.nominalVoltage / getVoltage(), 0.025) { updatePIDF.run() }
        Kp = SetOnChange.ofDouble(config.Kp) { updatePIDF.run() }
        Ki = SetOnChange.ofDouble(config.Ki) { updatePIDF.run() }
        Kd = SetOnChange.ofDouble(config.Kd) { updatePIDF.run() }
        Kv = SetOnChange.ofDouble(config.Kv) { updatePIDF.run() }
    }

    override fun setTPS(tps: Double) {
        val clamped = MathUtil.clamp(tps, -config.maxSettableVelocity, config.maxSettableVelocity)
        trajectory.setTarget(clamped)
    }

    override fun isAtTargetSpeed(): Boolean {
        return trajectory.getAcceleration() == 0.0 &&
            abs(getTpsFiltered() - trajectory.getTarget()) < 30
    }

    override fun setPower(power: Double) {
        throw UnsupportedOperationException(
            "VelocityMotorSdkPidf is velocity-only; use setRPM/setTPS"
        )
    }

    override fun updateInternal(dt: Double) {
        setFilterCutoff(config.lpfCutoff)
        super.updateInternal(dt)
        trajectory.updateConfig(config.maxAccel, config.jerkIncreasing, config.jerkDecreasing)
        Kp.set(config.Kp)
        Ki.set(config.Ki)
        Kd.set(config.Kd)
        Kv.set(config.Kv)
        voltageFactor.set(config.nominalVoltage / getVoltage())
        trajectory.update()
        if (abs(trajectory.getTarget()) < 1e-6) {
            stop()
        } else {
            motorVelocitySetpoint.set(trajectory.getVelocity())
        }
    }

    override fun writeTelemetry() {
        super.writeTelemetry()
        telemetry.addData(name + " Voltage Factor", "%.3f", voltageFactor.get())
        telemetry.addData(name + " TPS LPF", "%.1f", getTpsFiltered())
        telemetry.addData(name + " RPM LPF", "%.1f", tpsToRpm(getTpsFiltered()))
    }

    override fun stop() {
        trajectory.setTarget(0.0)
        motor.setVelocity(0.0)
    }
}
