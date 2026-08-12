package org.marsroboticsassociation.controllib.sim

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sign

/**
 * Physics mock for a single-joint arm motor with gravity.
 *
 * <p>Simulates second-order nonlinear arm dynamics:
 * <pre>
 *   dθ/dt = ω
 *   dω/dt = −(kV/kA)·ω + (1/kA)·(u − kS·sign(ω) − kG·cos(θ))
 * </pre>
 * where θ is the arm angle measured from horizontal (positive = above horizontal), ω is angular
 * velocity in rad/s, and u is applied voltage.
 *
 * <p>Integrated with 4th-order Runge-Kutta. Hard stops clamp position and zero velocity.
 *
 * <p>Encoder model: an [EncoderSim] provides a live integer tick position, while its ring buffer
 * models the 50 ms velocity window (6-entry buffer, 20 TPS quantization).
 *
 * <p>Typical use:
 * <pre>
 *   ArmMotorSim sim = new ArmMotorSim(kS, kG, kV, kA, ticksPerRev, gearRatio,
 *                                      encoderZeroOffsetRad, minAngleRad, maxAngleRad);
 *   for (int i = 0; i &lt; 300; i++) {
 *       int    posTicks = sim.getPositionTicks();
 *       double velTps   = sim.getVelocityTps();
 *       double power    = controller.update(dt);
 *       sim.step(dt, power, 12.0);
 *   }
 * </pre>
 */
class ArmMotorSim(
    private val kS: Double, // static friction voltage
    private val kG: Double, // gravity voltage at horizontal
    private val kV: Double, // V/(rad/s)
    private val kA: Double, // V/(rad/s^2)
    ticksPerRev: Int,
    gearRatio: Double,
    private val encoderZeroOffsetRad: Double, // angle (from horizontal) when encoder reads 0
    private val minAngleRad: Double,
    private val maxAngleRad: Double,
    initialAngleRad: Double,
) {

    // Derived motor dynamics coefficients
    private val aCoeff = -kV / kA // −kV/kA  [1/s]
    private val bCoeff = 1.0 / kA //  1/kA   [rad/s per V·s]

    // Encoder conversion
    private val ticksPerRad =
        (ticksPerRev * gearRatio) / (2.0 * PI) // motor encoder ticks per output radian

    private lateinit var encoder: EncoderSim

    // True state (output shaft, radians from horizontal)
    private var truePositionRad = initialAngleRad
    private var trueVelocityRadPerSec = 0.0
    private var disturbanceVoltage = 0.0

    init {
        // Seed encoder: set fractionalTicks to initial position, then write enough
        // samples at zero velocity to fill the ring buffer so getPosition/getVelocity work.
        installEncoder(EncoderSim(), truePositionRad)
    }

    /**
     * Advance the simulation by one time step.
     *
     * @param dt time step in seconds
     * @param normalizedPower motor power in [-1, 1]
     * @param hubVoltage bus voltage in volts (typically 12.0)
     */
    fun step(dt: Double, normalizedPower: Double, hubVoltage: Double) {
        val u = normalizedPower * hubVoltage + disturbanceVoltage
        rk4Step(u, dt)
        enforceHardStops()
        // Feed encoder in motor ticks per second
        val velocityTps = trueVelocityRadPerSec * ticksPerRad
        encoder.advance(dt, velocityTps)
    }

    /**
     * Inject a voltage-equivalent disturbance into the plant.
     *
     * @param v disturbance voltage in volts
     */
    fun setDisturbanceVoltage(v: Double) {
        disturbanceVoltage = v
    }

    /**
     * Replaces the encoder model (e.g. [EncoderSim.controlHub] or [EncoderSim.expansionHub] for
     * read-timing jitter). Re-seeds the new encoder to the current position and fills its ring
     * buffer at zero velocity, so [getPositionTicks] and [getVelocityTps] work immediately.
     *
     * @param encoder non-null encoder model
     * @throws IllegalArgumentException if [encoder] is null
     */
    fun setEncoder(encoder: EncoderSim?) {
        installEncoder(encoder, truePositionRad)
    }

    /**
     * Install an encoder model seeded at the given output-shaft angle with a full zero-velocity
     * ring buffer so position and windowed velocity reads work immediately.
     */
    private fun installEncoder(encoder: EncoderSim?, positionRad: Double) {
        if (encoder == null) {
            throw IllegalArgumentException("encoder must not be null")
        }
        this.encoder = encoder
        val initialTicksDouble = (positionRad - encoderZeroOffsetRad) * ticksPerRad
        this.encoder.setState(0, initialTicksDouble)
        // Write 6 samples (fills the buffer) at zero velocity, spaced 10 ms apart.
        for (i in 0 until 6) {
            this.encoder.advance(0.010, 0.0)
        }
    }

    /** Returns the most recent integer tick position from the encoder ring buffer. */
    fun getPositionTicks(): Int {
        return encoder.getPosition()
    }

    /** Returns the velocity in TPS from the encoder ring buffer. */
    fun getVelocityTps(): Double {
        return encoder.getVelocityTps()
    }

    /** Returns the true (noiseless) position in radians from horizontal. */
    fun getTruePositionRad(): Double {
        return truePositionRad
    }

    /** Returns the true (noiseless) angular velocity in rad/s. */
    fun getTrueVelocityRadPerSec(): Double {
        return trueVelocityRadPerSec
    }

    // -------------------------------------------------------------------------
    // 4th-order Runge-Kutta integration of the arm dynamics
    //
    //   dθ/dt = ω
    //   dω/dt = aCoeff·ω + bCoeff·(u − kS·sign(ω) − kG·cos(θ))
    // -------------------------------------------------------------------------

    private fun dTheta(omega: Double): Double {
        return omega
    }

    private fun dOmega(theta: Double, omega: Double, u: Double): Double {
        val friction = kS * sign(omega)
        val gravity = kG * cos(theta)
        return aCoeff * omega + bCoeff * (u - friction - gravity)
    }

    private fun rk4Step(u: Double, dt: Double) {
        val th = truePositionRad
        val om = trueVelocityRadPerSec

        val k1_th = dTheta(om)
        val k1_om = dOmega(th, om, u)

        val k2_th = dTheta(om + 0.5 * dt * k1_om)
        val k2_om = dOmega(th + 0.5 * dt * k1_th, om + 0.5 * dt * k1_om, u)

        val k3_th = dTheta(om + 0.5 * dt * k2_om)
        val k3_om = dOmega(th + 0.5 * dt * k2_th, om + 0.5 * dt * k2_om, u)

        val k4_th = dTheta(om + dt * k3_om)
        val k4_om = dOmega(th + dt * k3_th, om + dt * k3_om, u)

        truePositionRad += (dt / 6.0) * (k1_th + 2 * k2_th + 2 * k3_th + k4_th)
        trueVelocityRadPerSec += (dt / 6.0) * (k1_om + 2 * k2_om + 2 * k3_om + k4_om)
    }

    private fun enforceHardStops() {
        if (truePositionRad <= minAngleRad) {
            truePositionRad = minAngleRad
            if (trueVelocityRadPerSec < 0) trueVelocityRadPerSec = 0.0
        }
        if (truePositionRad >= maxAngleRad) {
            truePositionRad = maxAngleRad
            if (trueVelocityRadPerSec > 0) trueVelocityRadPerSec = 0.0
        }
    }
}
