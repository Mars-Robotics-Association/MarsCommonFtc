package org.marsroboticsassociation.controllib.control

import java.util.function.LongSupplier
import kotlin.math.sqrt
import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt
import org.marsroboticsassociation.controllib.filter.LowPassFilter
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.motion.VelocityTrajectoryManager
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Abstract base class for velocity-controlled motors with encoder feedback.
 *
 * Extends [MotorBase] to add velocity measurement and filtering. Subclasses implement specific
 * control strategies (e.g., PF, SDK PIDF).
 */
abstract class VelocityMotorBase : MotorBase {

    protected val trajectory: VelocityTrajectoryManager

    private var tpsActual: Double = 0.0
    private val tpsLpf: LowPassFilter

    constructor(
        telemetry: TelemetryAddData,
        gearRatio: Double,
        motorPPR: Double,
        motorPowerChangeTolerance: Double,
        motor: IMotor,
        aMax: Double,
        jInc: Double,
        jDec: Double,
        vChangeTolerance: Double,
    ) : super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor) {
        tpsLpf = BiquadLowPassVarDt(12.0, 1.0 / sqrt(2.0))
        trajectory = VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry)
        trajectory.updateConfig(aMax, jInc, jDec)
    }

    /** Public constructor for simulations — injects a custom clock. */
    constructor(
        telemetry: TelemetryAddData,
        gearRatio: Double,
        motorPPR: Double,
        motorPowerChangeTolerance: Double,
        motor: IMotor,
        aMax: Double,
        jInc: Double,
        jDec: Double,
        vChangeTolerance: Double,
        clock: LongSupplier,
    ) : super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor) {
        tpsLpf = BiquadLowPassVarDt(12.0, 1.0 / sqrt(2.0))
        trajectory = VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry, clock)
        trajectory.updateConfig(aMax, jInc, jDec)
    }

    fun tpsToRpm(tps: Double): Double = tps * 60.0 / motorPPR / gearRatio

    fun rpmToTps(rpm: Double): Double = rpm * gearRatio * motorPPR / 60.0

    abstract fun setTPS(tps: Double)

    fun getProfiledVelocity(): Double = trajectory.getVelocity()

    fun setRPM(rpm: Double) {
        setTPS(rpmToTps(rpm))
    }

    fun getTpsSetpoint(): Double = trajectory.getTarget()

    fun getRpmSetpoint(): Double = tpsToRpm(getTpsSetpoint())

    protected fun setFilterCutoff(cutoffHz: Double) {
        tpsLpf.setCutoffHz(cutoffHz)
    }

    abstract fun isAtTargetSpeed(): Boolean

    override fun updateInternal(dt: Double) {
        tpsActual = motor.velocity
        tpsLpf.update(tpsActual, dt)
    }

    override fun writeTelemetry() {
        telemetry.addData(name + " TPS setpoint", "%.0f", getTpsSetpoint())
        telemetry.addData(name + " RPM setpoint", "%.0f", getRpmSetpoint())
        telemetry.addData(name + " TPS measured", "%.0f", tpsActual)
        telemetry.addData(name + " RPM measured", "%.0f", tpsToRpm(tpsActual))
    }

    fun getTpsMeasurement(): Double = tpsActual

    fun getTpsFiltered(): Double = tpsLpf.value
}
