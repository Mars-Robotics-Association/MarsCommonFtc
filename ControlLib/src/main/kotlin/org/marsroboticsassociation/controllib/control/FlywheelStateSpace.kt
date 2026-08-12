package org.marsroboticsassociation.controllib.control

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.LinearSystemLoop
import edu.wpi.first.math.system.plant.LinearSystemId
import kotlin.math.PI
import kotlin.math.abs
import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt
import org.marsroboticsassociation.controllib.filter.LowPassFilter
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Flywheel velocity controller using a state-space LQR + Kalman filter observer.
 *
 * Uses `LinearSystemId.identifyVelocitySystem(kV, kA)` to build the plant from characterization
 * constants, avoiding the need for FTC motor specs that are absent from `DCMotor.java`. [Params.kV]
 * and [Params.kA] are stored in WPILib SI units (V·s/rad and V·s²/rad); see [Params] for the
 * conversion from characterization data.
 *
 * The Kalman filter runs at a nominal 20 ms dt for gain computation but accepts the real measured
 * dt in [update] for accurate state propagation. FTC's non-deterministic loop timing (typically
 * 10–30 ms) is handled automatically.
 *
 * Voltage normalization: the loop outputs a voltage command which is divided by the live hub
 * voltage to produce a motor power in [−1, 1], compensating for battery sag. Battery voltage is
 * sampled each control loop; pass a pre-sampled value via [update] if voltage was already read
 * elsewhere in the OpMode.
 *
 * Usage:
 * ```
 *   val flywheel = FlywheelStateSpace(motor, telemetry::addData)
 *   flywheel.setTps(2000.0)
 *   // each loop iteration:
 *   flywheel.update(dt)
 * ```
 */
class FlywheelStateSpace(
    private val motor: IMotor,
    private val telemetry: TelemetryAddData,
) {

    /**
     * Tuning parameters. Shared between instances.
     *
     * Plant constants ([kV], [kA]) are in WPILib SI units. To convert from characterization data
     * (voltage, max TPS, ticksPerRev):
     * ```
     *   kV = measuredVoltage * ticksPerRev / (maxTps * 2π)   // V·s/rad
     *   kA = measuredVoltage * ticksPerRev / (maxAccelTps * 2π) // V·s²/rad
     * ```
     *
     * Kalman std-dev values trade off responsiveness vs. noise rejection: higher
     * modelStdDevRadPerSec trusts measurements more; lower measurementStdDevRadPerSec trusts the
     * encoder more.
     *
     * LQR values: smaller qVelocityRadPerSec → more aggressive velocity tracking; larger rVoltage →
     * penalizes voltage use more heavily.
     */
    class Params {
        // Plant — WPILib SI units (V·s/rad, V·s²/rad)
        // Defaults derived from: 12.5 V characterization run, 28 ticks/rev
        @JvmField var kV: Double = 12.5 * 28 / (2632.1 * 2 * PI) // V·s/rad
        @JvmField var kA: Double = 12.5 * 28 / (2087.9 * 2 * PI) // V·s²/rad
        @JvmField var ticksPerRev: Int = 28 // encoder ticks per revolution at output shaft

        // Kalman observer noise model
        @JvmField
        var modelStdDevRadPerSec: Double = 3.0 // process noise (higher → trust sensor more)
        @JvmField
        var measurementStdDevRadPerSec: Double =
            0.01 // measurement noise (lower → trust encoder more)

        // LQR cost weights
        @JvmField
        var qVelocityRadPerSec: Double =
            8.0 // velocity tolerance in rad/s (tighter → more aggressive)
        @JvmField var rVoltage: Double = 12.0 // voltage penalty

        @JvmField var dtSeconds: Double = 0.020 // nominal loop period for Kalman gain computation

        /** Estimated velocity must be within this many TPS of the setpoint for [isReady]. */
        @JvmField var readyThresholdTps: Double = 30.0
        /** Estimated acceleration must be below this (TPS/s) for [isReady]. */
        @JvmField var readyAccelToleranceTps2: Double = 50.0
        /** Biquad low-pass cutoff frequency for smoothing the acceleration estimate (Hz). */
        @JvmField var accelLpfCutoffHz: Double = 2.0
    }

    private val primaryName: String = motor.name
    private val loop: LinearSystemLoop<N1, N1, N1>

    private var targetTps: Double = 0.0
    private var lastVoltageCmded: Double = 0.0
    private var lastPower: Double = 0.0
    private var prevEstimatedTps: Double = 0.0
    private val accelLpf: LowPassFilter

    init {
        val startupVoltage = motor.hubVoltage
        loop = buildLoop(PARAMS, startupVoltage)
        accelLpf = BiquadLowPassVarDt(PARAMS.accelLpfCutoffHz, 0.5)
        // Seed observer to current velocity so the first correction isn't a large jump
        val initialRadPerSec = motor.velocity * 2 * PI / PARAMS.ticksPerRev
        loop.reset(VecBuilder.fill(initialRadPerSec))
    }

    // ---------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------

    /** Set the target flywheel velocity in ticks per second. Pass 0 to coast the motor. */
    fun setTps(tps: Double) {
        targetTps = tps
    }

    /**
     * Run one control cycle, reading battery voltage from the motor adapter.
     *
     * Must be called once per loop iteration. Pass the elapsed time since the last call so the
     * Kalman predictor uses the real dt rather than the nominal value.
     *
     * If battery voltage was already sampled elsewhere in the OpMode this loop, use [update] with
     * hubVoltage to avoid a redundant hub read.
     *
     * @param dt elapsed time in seconds since the last `update()` call
     */
    fun update(dt: Double) {
        update(dt, motor.hubVoltage)
    }

    /**
     * Run one control cycle with a pre-sampled battery voltage.
     *
     * Use this overload when battery voltage has already been read elsewhere in the OpMode to avoid
     * a redundant I²C read.
     *
     * @param dt elapsed time in seconds since the last `update()` call
     * @param hubVoltage battery voltage in volts, sampled this loop iteration
     */
    fun update(dt: Double, hubVoltage: Double) {
        if (dt < 1e-6) return // likely a duplicate call in the same frame

        val twoPI = 2.0 * PI
        val measuredRadPerSec = motor.velocity * twoPI / PARAMS.ticksPerRev
        val targetRadPerSec = targetTps * twoPI / PARAMS.ticksPerRev

        loop.setNextR(VecBuilder.fill(targetRadPerSec))
        loop.correct(VecBuilder.fill(measuredRadPerSec))
        loop.predict(dt) // uses real dt for accurate propagation

        lastVoltageCmded = loop.getU(0) // already clamped to ±startupVoltage by the loop
        val voltage = MathUtil.clamp(lastVoltageCmded, -hubVoltage, hubVoltage)
        lastPower = MathUtil.clamp(voltage / hubVoltage, -1.0, 1.0)

        motor.setPower(if (targetTps == 0.0) 0.0 else lastPower)

        val currentEstimatedTps = getEstimatedTps()
        val rawAccel = (currentEstimatedTps - prevEstimatedTps) / dt
        accelLpf.update(rawAccel, dt)
        prevEstimatedTps = currentEstimatedTps
    }

    /**
     * Returns the Kalman-estimated flywheel velocity in ticks per second. This is smoother than the
     * raw encoder reading and typically more accurate than a fixed-cutoff low-pass filter.
     */
    fun getEstimatedTps(): Double {
        return loop.getXHat(0) * PARAMS.ticksPerRev / (2.0 * PI)
    }

    /**
     * Returns true when the flywheel is spinning, the Kalman-estimated velocity is within
     * [Params.readyThresholdTps] of the setpoint, and the estimated acceleration is below
     * [Params.readyAccelToleranceTps2] (indicating the velocity has settled rather than just
     * passing through the threshold). Returns false when the setpoint is zero.
     */
    fun isReady(): Boolean {
        return targetTps != 0.0 &&
            abs(getEstimatedTps() - targetTps) < PARAMS.readyThresholdTps &&
            abs(accelLpf.value) < PARAMS.readyAccelToleranceTps2
    }

    /**
     * Returns the low pass filtered acceleration in TPS/s, derived from consecutive Kalman
     * estimates.
     */
    fun getEstimatedAccelTps2(): Double = accelLpf.value

    /**
     * Reset the observer to the current measured velocity. Call this when restarting after a long
     * idle period to avoid a transient where the observer state is far from reality.
     */
    fun reset() {
        val currentRadPerSec = motor.velocity * 2.0 * PI / PARAMS.ticksPerRev
        loop.reset(VecBuilder.fill(currentRadPerSec))
    }

    /** Add state-space flywheel telemetry to the driver station. */
    fun writeTelemetry() {
        telemetry.addData(primaryName + " ss target TPS", "%.0f", targetTps)
        telemetry.addData(primaryName + " ss estimated TPS", "%.1f", getEstimatedTps())
        telemetry.addData(primaryName + " ss measured TPS", "%.1f", motor.velocity)
        telemetry.addData(primaryName + " ss voltage cmd", "%.2f V", lastVoltageCmded)
        telemetry.addData(primaryName + " ss power", "%.3f", lastPower)
    }

    companion object {
        @JvmField var PARAMS: Params = Params()

        private fun buildLoop(p: Params, maxVoltage: Double): LinearSystemLoop<N1, N1, N1> {
            val plant: LinearSystem<N1, N1, N1> = LinearSystemId.identifyVelocitySystem(p.kV, p.kA)

            val observer =
                KalmanFilter(
                    Nat.N1(),
                    Nat.N1(),
                    plant,
                    VecBuilder.fill(p.modelStdDevRadPerSec),
                    VecBuilder.fill(p.measurementStdDevRadPerSec),
                    p.dtSeconds,
                )

            val controller =
                LinearQuadraticRegulator(
                    plant,
                    VecBuilder.fill(p.qVelocityRadPerSec),
                    VecBuilder.fill(p.rVoltage),
                    p.dtSeconds,
                )

            return LinearSystemLoop(plant, controller, observer, maxVoltage, p.dtSeconds)
        }
    }
}
