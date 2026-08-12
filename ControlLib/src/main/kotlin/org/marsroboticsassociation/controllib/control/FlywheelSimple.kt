package org.marsroboticsassociation.controllib.control

import edu.wpi.first.math.MathUtil
import java.util.function.LongSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Simplified flywheel control with voltage-based feedforward + proportional feedback, battery
 * voltage compensation, and a clamped-exponential motion profile.
 *
 * Coasting / spin-up behavior:
 * - When tps is 0, the motor is set to power = 0 (coast).
 * - On resume from zero, the velocity filter is re-seeded from the current encoder reading and the
 *   profile resumes from the current (coasting) speed.
 */
class FlywheelSimple {

    /** Parameters for tuning */
    class Params {
        /** Low-pass filter cutoff frequency for measured velocity, in Hz. */
        @JvmField var velLpfCutoffHz: Double = 4.0

        /**
         * Feedforward voltage constant (voltage per unit velocity). From
         * FlywheelsFeedforwardTuning.
         */
        @JvmField var kV: Double = 12.5 / 2632.1
        /**
         * Feedforward voltage constant (voltage per unit acceleration). From
         * FlywheelsFeedforwardTuning.
         */
        @JvmField var kA: Double = 12.5 / 2087.9
        /** Feedforward voltage constant (Coulomb friction). From FlywheelsFeedforwardTuning. */
        @JvmField var kS: Double = 0.8931

        /** Proportional feedback gain (volts per tps error). */
        @JvmField var kP: Double = 0.010
        /** Maximum feedback as a fraction of hub voltage. */
        @JvmField var fbMax: Double = 0.2

        /**
         * Maximum duration of a single profile step, in seconds. Caps dt to prevent gap overshoot.
         */
        @JvmField var maxProfileDt: Double = 0.060

        /**
         * Maximum acceleration cap for the motion profile, in TPS^2.
         *
         * The actual acceleration limit at any point is: `min(maxAccel, (V - kS - kV *
         * profiledVelocity) / kA)`, accounting for back-EMF stealing voltage at higher speeds.
         */
        @JvmField var maxAccel: Double = (12.0 - kS) / kA

        /** Velocity threshold (TPS) within which isReady() returns true. */
        @JvmField var readyThreshold: Double = 40.0
        /** Velocity error (TPS) below which the profile snaps to the setpoint. */
        @JvmField var snapThresholdTPS: Double = 10.0
    }

    private val motor: IMotor
    private val primaryName: String
    private val hubVoltage: Double
    private val telemetry: TelemetryAddData
    private val clock: LongSupplier
    private var smoothVelocity: Double = 0.0

    private var profiledVelocity: Double = 0.0
    private var stopped: Boolean = true
    private var lastTimeNanos: Long = 0

    private var rawSetpoint: Double = 0.0
    private var rawVelocity: Double = 0.0
    private var lastPower: Double = 0.0
    private var powerTooLowForTarget: Boolean = false

    /**
     * @param telemetry typically telemetry::addData
     * @param motor motor providing both encoder feedback and power output
     */
    constructor(telemetry: TelemetryAddData, motor: IMotor) {
        this.motor = motor
        this.hubVoltage = motor.hubVoltage
        this.primaryName = motor.name
        this.telemetry = telemetry
        this.clock = LongSupplier { System.nanoTime() }
    }

    /**
     * Returns the current profiled (commanded) velocity in ticks per second. This is the velocity
     * the controller is actively targeting via feedforward, which may differ from the measured
     * velocity during spin-up or disturbance recovery.
     */
    fun getProfiledVelocity(): Double = profiledVelocity

    /** Returns the low-pass filtered measured velocity in ticks per second. */
    fun getFilteredVelocity(): Double = smoothVelocity

    /** For unit tests and simulations — allows injecting a custom clock. */
    constructor(telemetry: TelemetryAddData, clock: LongSupplier, motor: IMotor) {
        this.motor = motor
        this.hubVoltage = motor.hubVoltage
        this.primaryName = motor.name
        this.telemetry = telemetry
        this.clock = clock
    }

    /** Command a new target velocity (ticks per second). */
    fun setTps(tps: Double) {
        rawSetpoint = max(0.0, tps)
    }

    /** Update the flywheel controller. Should be called once on each control loop. */
    fun update() {
        // Timing
        val now = clock.asLong
        if (lastTimeNanos == 0L) {
            lastTimeNanos = now
            return
        }
        val dt = (now - lastTimeNanos) / 1e9
        lastTimeNanos = now
        if (dt < 1e-6) return

        // Velocity filter (always runs so readings stay fresh)
        val filterTau = 1.0 / (2.0 * PI * PARAMS.velLpfCutoffHz)
        val alpha = 1.0 - exp(-dt / filterTau)
        rawVelocity = motor.velocity
        smoothVelocity = alpha * rawVelocity + (1.0 - alpha) * smoothVelocity

        // Coast when target is zero
        if (rawSetpoint == 0.0) {
            motor.setPower(0.0)
            lastPower = 0.0
            stopped = true
            return
        }

        // Seed profile on resume from stopped
        // Reset the filter so the profile and feedback start from a clean
        // encoder reading, not a stale smoothed value from before the gap.
        if (stopped) {
            smoothVelocity = rawVelocity
            profiledVelocity = rawVelocity
            stopped = false
        }

        // Velocity ramp profile
        // Ramp toward target at accelLimit, decelerate when approaching.
        // accelLimit accounts for back-EMF: (V - kS - kV * profiledVelocity) / kA.
        val profileDt = min(dt, PARAMS.maxProfileDt)
        var accelLimit = (hubVoltage - PARAMS.kS - PARAMS.kV * profiledVelocity) / PARAMS.kA
        accelLimit = min(PARAMS.maxAccel, max(accelLimit, 0.0))
        val error = rawSetpoint - profiledVelocity
        var accel = sign(error) * accelLimit
        profiledVelocity += accel * profileDt

        // Snap to target once within one quantization step (avoids asymptotic creep)
        if (abs(rawSetpoint - profiledVelocity) < PARAMS.snapThresholdTPS) {
            profiledVelocity = rawSetpoint
            accel = 0.0
        }
        // Voltage-based feedforward + proportional feedback
        val ffVoltage =
            PARAMS.kS * sign(profiledVelocity) + PARAMS.kV * profiledVelocity + PARAMS.kA * accel
        var fbVoltage = PARAMS.kP * (profiledVelocity - smoothVelocity)
        fbVoltage = MathUtil.clamp(fbVoltage, -PARAMS.fbMax * hubVoltage, PARAMS.fbMax * hubVoltage)

        lastPower = MathUtil.clamp((ffVoltage + fbVoltage) / hubVoltage, -1.0, 1.0)
        motor.setPower(lastPower)

        // Update powerTooLowForTarget
        val ffFraction = (PARAMS.kS + PARAMS.kV * rawSetpoint) / hubVoltage
        powerTooLowForTarget = ffFraction > 0.85
    }

    /**
     * Returns true when both the profile has settled and measured velocity is within
     * [Params.readyThreshold] of the target.
     */
    fun isReady(): Boolean {
        return rawSetpoint != 0.0 &&
            profiledVelocity == rawSetpoint &&
            abs(smoothVelocity - rawSetpoint) < PARAMS.readyThreshold
    }

    /**
     * Returns true if the steady-state feedforward at current battery voltage leaves insufficient
     * headroom for feedback to reliably reach the target velocity.
     */
    fun isPowerTooLowForTargetVelocity(): Boolean = powerTooLowForTarget

    /** Add flywheel telemetry to driver station. */
    fun writeTelemetry() {
        telemetry.addData(primaryName + " setpoint", "%.0f", rawSetpoint)
        telemetry.addData(primaryName + " profiled", "%.0f", profiledVelocity)
        telemetry.addData(primaryName + " velocity (raw)", "%.0f", rawVelocity)
        telemetry.addData(primaryName + " velocity (smooth)", "%.1f", smoothVelocity)
        telemetry.addData(primaryName + " power", "%.2f", lastPower)
        telemetry.addData(primaryName + " isReady", "%b", isReady())
    }

    companion object {
        /** Global tuning parameters. Shared between instances. */
        @JvmField var PARAMS: Params = Params()
    }
}
