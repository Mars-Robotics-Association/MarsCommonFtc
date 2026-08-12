package org.marsroboticsassociation.controllib.sim

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sign

/**
 * Physics mock for a single-joint arm motor with gravity <b>and gearbox backlash</b>.
 *
 * <p>Where [ArmMotorSim] treats the motor and arm as one rigid body, this models them as <b>two
 * inertias</b> coupled through the gear teeth, with a dead band between them:
 * <ul>
 * <li><b>Motor side</b> ({@code θ_m}, {@code ω_m}) &mdash; the rotor and everything ahead of the
 *   backlash. This is what the motor encoder measures. The reflected rotor inertia dominates a
 *   high-ratio FTC gearbox, so this side carries the bulk of {@code kA}.</li>
 * <li><b>Load side</b> ({@code θ_L}, {@code ω_L}) &mdash; the arm itself, on the far side of the
 *   lash. <b>Gravity acts here.</b> Its own inertia is a fraction of the motor side.</li>
 * </ul>
 *
 * <p>All angles are output-shaft (arm) radians from horizontal (positive = above horizontal).
 *
 * <p><b>Dead-band spring contact.</b> Let {@code δ = θ_m − θ_L} and {@code h = backlash/2}. The
 * gear teeth only push when the relative position leaves the dead band:
 * <pre>
 *   δ &gt;  h :  penetration = δ − h   (forward face in contact)
 *   δ &lt; −h :  penetration = δ + h   (reverse face in contact)
 *   |δ| ≤ h :  penetration = 0       (free &mdash; teeth not touching)
 *   τ_contact = k_contact · penetration  (+ c_contact · (ω_m − ω_L) while engaged)
 * </pre>
 * <p>The consequences are the ones that wreck arm controllers:
 * <ul>
 * <li><b>Lost motion on reversal.</b> After a direction change the motor must travel the full
 *   backlash before the arm responds &mdash; the encoder moves while the arm does not.</li>
 * <li><b>Gravity free-fall across the gap.</b> While the teeth are separated the arm is unsupported
 *   and accelerates under gravity (only its own small inertia), then re-engages with an
 *   impact.</li>
 * <li><b>The encoder is blind to the load.</b> [getPositionTicks] reflects the <em>motor</em> side;
 *   the true arm angle can differ by up to the backlash. This is the "measured through the
 *   backlash" problem &mdash; a motor-encoder-only controller cannot see where the arm actually is
 *   across the gap.</li>
 * </ul>
 *
 * <p><b>Equations of motion</b> (voltage-equivalent units, matching [ArmMotorSim]; "torque" is in
 * volts, "inertia" is {@code kA} in V·s²/rad, "damping" is {@code kV} in V·s/rad):
 * <pre>
 *   kA_m · dω_m/dt = u − kV·ω_m − kS·sign(ω_m) − τ_contact
 *   kA_L · dω_L/dt = τ_contact − kG·cos(θ_L) − kV_L·ω_L − kS_L·sign(ω_L)
 * </pre>
 * <p>Integrated with 4th-order Runge-Kutta on a small internal sub-step (the contact spring is
 * stiff relative to a 16 ms control loop, so a single 16 ms RK4 step would ring or diverge). Hard
 * stops clamp the <b>load</b> position and zero its velocity. The encoder is fed the motor-side
 * velocity.
 *
 * <p><b>Note on defaults.</b> The split of inertia between motor and load, the contact stiffness,
 * and the contact damping are not yet known from sysid; the defaults here are chosen to be
 * qualitatively realistic and numerically stable, and are exposed via setters so they can be swept
 * (e.g. in ControlLab). Treat them as a starting point, not measured values.
 *
 * <p>Typical use:
 * <pre>
 *   BacklashArmMotorSim sim = new BacklashArmMotorSim(kS, kG, kV, kA, ticksPerRev, gearRatio,
 *           encoderZeroOffsetRad, minAngleRad, maxAngleRad, initialAngleRad, Math.toRadians(5));
 *   for (int i = 0; i &lt; 300; i++) {
 *       int    posTicks = sim.getPositionTicks();   // motor side — what a real controller sees
 *       double velTps   = sim.getVelocityTps();
 *       double power    = controller.update(dt);
 *       sim.step(dt, power, 12.0);
 *       double trueArm  = sim.getTruePositionRad();  // load side — ground truth for scoring
 *   }
 * </pre>
 */
class BacklashArmMotorSim(
    private val kS: Double, // motor-side static friction voltage
    private val kG: Double, // gravity voltage at horizontal (acts on the load)
    private val kV: Double, // motor-side viscous damping, V/(rad/s)
    kA: Double,
    ticksPerRev: Int,
    gearRatio: Double,
    private val encoderZeroOffsetRad: Double, // angle (from horizontal) when encoder reads 0
    private val minAngleRad: Double,
    private val maxAngleRad: Double,
    initialAngleRad: Double,
    backlashRad: Double,
) {

    // Reflected rotor inertia dominates a high-ratio gearbox, so the motor side carries most of
    // kA. The default load fraction is a stable, qualitatively reasonable guess (pending sysid).
    private val kAMotor: Double // motor-side inertia, V/(rad/s^2)
    private val kALoad: Double // load-side inertia, V/(rad/s^2)

    // Load-side friction (the arm bearing). Defaults to zero so the gravity free-fall through the
    // gap stays pronounced; set via setters if the real joint has meaningful bearing drag.
    private var kVLoad = 0.0
    private var kSLoad = 0.0

    // Dead-band spring contact between motor and load.
    private val halfBacklashRad: Double // h = backlash / 2
    private var contactStiffness: Double // k_contact, V per rad of tooth penetration
    private var contactDamping: Double // c_contact, V per rad/s of relative velocity while engaged

    // Encoder conversion
    private val ticksPerRad: Double // motor encoder ticks per output radian

    // Largest internal integration sub-step. The stiff contact spring needs this to stay stable and
    // non-ringing at typical control-loop dt's (~16 ms).
    private var maxInternalDtSec = 0.0005

    private lateinit var encoder: EncoderSim

    // True state (output shaft, radians from horizontal)
    private var motorPositionRad: Double
    private var motorVelocityRadPerSec: Double
    private var loadPositionRad: Double
    private var loadVelocityRadPerSec: Double
    private var disturbanceVoltage = 0.0

    init {
        val defaultLoadFraction = 0.2
        kAMotor = kA * (1.0 - defaultLoadFraction)
        kALoad = kA * defaultLoadFraction

        ticksPerRad = (ticksPerRev * gearRatio) / (2.0 * PI)
        halfBacklashRad = maxOf(0.0, backlashRad) / 2.0

        // Default contact: stiff enough that the static gravity penetration is a small fraction of
        // a
        // degree, lightly under-damped so re-engagement has a little bounce. Tunable via setters.
        contactStiffness = 500.0
        contactDamping = 2.0

        loadPositionRad = clampToStops(initialAngleRad)
        loadVelocityRadPerSec = 0.0

        // Seed the motor side resting on the gravity-loaded contact face so there is no start-up
        // transient: at rest the contact must supply τ = kG·cos(θ_L) to hold the arm.
        val holdTorque = kG * cos(loadPositionRad)
        val penetration = holdTorque / contactStiffness
        motorPositionRad = loadPositionRad + penetration + sign(penetration) * halfBacklashRad
        motorVelocityRadPerSec = 0.0

        seedEncoder()
    }

    private fun seedEncoder() {
        val initialTicksDouble = (motorPositionRad - encoderZeroOffsetRad) * ticksPerRad
        encoder = EncoderSim()
        encoder.setState(0, initialTicksDouble)
        // Fill the ring buffer at zero velocity so getPosition/getVelocity work immediately.
        for (i in 0 until 6) {
            encoder.advance(0.010, 0.0)
        }
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

        var remaining = dt
        while (remaining > 1e-12) {
            val subDt = min(maxInternalDtSec, remaining)
            rk4Step(u, subDt)
            enforceHardStops()
            // Feed the encoder the motor-side velocity (the shaft the encoder is on), in TPS.
            encoder.advance(subDt, motorVelocityRadPerSec * ticksPerRad)
            remaining -= subDt
        }
    }

    /**
     * Inject a voltage-equivalent disturbance into the motor side of the plant.
     *
     * @param v disturbance voltage in volts
     */
    fun setDisturbanceVoltage(v: Double) {
        disturbanceVoltage = v
    }

    // ── tuning setters (sweepable, e.g. in ControlLab) ───────────────────────────

    /** Sets the contact spring stiffness in volts per radian of tooth penetration. */
    fun setContactStiffness(vPerRad: Double) {
        this.contactStiffness = vPerRad
    }

    /** Sets the contact damping in volts per (rad/s) of relative velocity while engaged. */
    fun setContactDamping(vPerRadPerSec: Double) {
        this.contactDamping = vPerRadPerSec
    }

    /** Sets the load-side bearing friction: viscous (V per rad/s) and static (V). */
    fun setLoadFriction(viscous: Double, staticVolts: Double) {
        this.kVLoad = viscous
        this.kSLoad = staticVolts
    }

    /**
     * Overrides the largest internal RK4 sub-step (seconds). Smaller = stiffer contact stays
     * stable.
     */
    fun setMaxInternalDt(sec: Double) {
        this.maxInternalDtSec = sec
    }

    /**
     * Replaces the encoder model (e.g. [EncoderSim.expansionHub] for RS485 jitter). Re-seeds at the
     * current motor-side angle with a zero-velocity ring buffer.
     *
     * @param encoder non-null encoder model
     * @throws IllegalArgumentException if [encoder] is null
     */
    fun setEncoder(encoder: EncoderSim?) {
        if (encoder == null) {
            throw IllegalArgumentException("encoder must not be null")
        }
        this.encoder = encoder
        val initialTicksDouble = (motorPositionRad - encoderZeroOffsetRad) * ticksPerRad
        this.encoder.setState(0, initialTicksDouble)
        for (i in 0 until 6) {
            this.encoder.advance(0.010, 0.0)
        }
    }

    // ── sensor outputs (what a real controller sees) ─────────────────────────────

    /**
     * Returns the most recent integer tick position from the encoder (motor side, through the
     * lash).
     */
    fun getPositionTicks(): Int {
        return encoder.getPosition()
    }

    /** Returns the velocity in TPS from the encoder ring buffer (motor side). */
    fun getVelocityTps(): Double {
        return encoder.getVelocityTps()
    }

    // ── ground truth (for scoring in sim only) ───────────────────────────────────

    /** Returns the true arm (load) position in radians from horizontal. */
    fun getTruePositionRad(): Double {
        return loadPositionRad
    }

    /** Returns the true arm (load) angular velocity in rad/s. */
    fun getTrueVelocityRadPerSec(): Double {
        return loadVelocityRadPerSec
    }

    /** Returns the true motor-side position in radians (output-referred). */
    fun getMotorPositionRad(): Double {
        return motorPositionRad
    }

    /** Returns the true motor-side angular velocity in rad/s (output-referred). */
    fun getMotorVelocityRadPerSec(): Double {
        return motorVelocityRadPerSec
    }

    /** True if the gear teeth are currently in contact (outside the dead band). */
    fun isEngaged(): Boolean {
        return abs(motorPositionRad - loadPositionRad) > halfBacklashRad
    }

    /** Total backlash at the output shaft, in radians. */
    fun getBacklashRad(): Double {
        return 2.0 * halfBacklashRad
    }

    /**
     * The load's static rest compliance: radians of motor-to-load droop per volt of gravity
     * hold-voltage, from the contact-spring penetration. The total rest offset between motor and
     * load is {@code halfBacklash·sign(g) + compliance·g}.
     */
    fun getRestComplianceRadPerVolt(): Double {
        return 1.0 / contactStiffness
    }

    // ── dynamics ─────────────────────────────────────────────────────────────────
    //
    //   state = [θ_m, ω_m, θ_L, ω_L]
    //   kA_m · dω_m/dt = u − kV·ω_m − kS·sign(ω_m) − τ_contact
    //   kA_L · dω_L/dt = τ_contact − kG·cos(θ_L) − kV_L·ω_L − kS_L·sign(ω_L)

    /** Contact (gear-tooth) torque on the load, in volts, from the relative state. */
    private fun contactTorque(
        thetaM: Double,
        omegaM: Double,
        thetaL: Double,
        omegaL: Double,
    ): Double {
        val delta = thetaM - thetaL
        if (delta > halfBacklashRad) {
            return contactStiffness * (delta - halfBacklashRad) + contactDamping * (omegaM - omegaL)
        }
        if (delta < -halfBacklashRad) {
            return contactStiffness * (delta + halfBacklashRad) + contactDamping * (omegaM - omegaL)
        }
        return 0.0 // teeth separated — no coupling
    }

    private fun dOmegaMotor(
        thetaM: Double,
        omegaM: Double,
        thetaL: Double,
        omegaL: Double,
        u: Double,
    ): Double {
        val tau = contactTorque(thetaM, omegaM, thetaL, omegaL)
        return (u - kV * omegaM - kS * sign(omegaM) - tau) / kAMotor
    }

    private fun dOmegaLoad(thetaM: Double, omegaM: Double, thetaL: Double, omegaL: Double): Double {
        val tau = contactTorque(thetaM, omegaM, thetaL, omegaL)
        val gravity = kG * cos(thetaL)
        return (tau - gravity - kVLoad * omegaL - kSLoad * sign(omegaL)) / kALoad
    }

    private fun rk4Step(u: Double, dt: Double) {
        val tm = motorPositionRad
        val wm = motorVelocityRadPerSec
        val tl = loadPositionRad
        val wl = loadVelocityRadPerSec

        // k1
        val k1_tm = wm
        val k1_wm = dOmegaMotor(tm, wm, tl, wl, u)
        val k1_tl = wl
        val k1_wl = dOmegaLoad(tm, wm, tl, wl)

        // k2
        val tm2 = tm + 0.5 * dt * k1_tm
        val wm2 = wm + 0.5 * dt * k1_wm
        val tl2 = tl + 0.5 * dt * k1_tl
        val wl2 = wl + 0.5 * dt * k1_wl
        val k2_tm = wm2
        val k2_wm = dOmegaMotor(tm2, wm2, tl2, wl2, u)
        val k2_tl = wl2
        val k2_wl = dOmegaLoad(tm2, wm2, tl2, wl2)

        // k3
        val tm3 = tm + 0.5 * dt * k2_tm
        val wm3 = wm + 0.5 * dt * k2_wm
        val tl3 = tl + 0.5 * dt * k2_tl
        val wl3 = wl + 0.5 * dt * k2_wl
        val k3_tm = wm3
        val k3_wm = dOmegaMotor(tm3, wm3, tl3, wl3, u)
        val k3_tl = wl3
        val k3_wl = dOmegaLoad(tm3, wm3, tl3, wl3)

        // k4
        val tm4 = tm + dt * k3_tm
        val wm4 = wm + dt * k3_wm
        val tl4 = tl + dt * k3_tl
        val wl4 = wl + dt * k3_wl
        val k4_tm = wm4
        val k4_wm = dOmegaMotor(tm4, wm4, tl4, wl4, u)
        val k4_tl = wl4
        val k4_wl = dOmegaLoad(tm4, wm4, tl4, wl4)

        motorPositionRad += (dt / 6.0) * (k1_tm + 2 * k2_tm + 2 * k3_tm + k4_tm)
        motorVelocityRadPerSec += (dt / 6.0) * (k1_wm + 2 * k2_wm + 2 * k3_wm + k4_wm)
        loadPositionRad += (dt / 6.0) * (k1_tl + 2 * k2_tl + 2 * k3_tl + k4_tl)
        loadVelocityRadPerSec += (dt / 6.0) * (k1_wl + 2 * k2_wl + 2 * k3_wl + k4_wl)
    }

    private fun clampToStops(angle: Double): Double {
        if (angle < minAngleRad) return minAngleRad
        if (angle > maxAngleRad) return maxAngleRad
        return angle
    }

    private fun enforceHardStops() {
        if (loadPositionRad <= minAngleRad) {
            loadPositionRad = minAngleRad
            if (loadVelocityRadPerSec < 0) loadVelocityRadPerSec = 0.0
        }
        if (loadPositionRad >= maxAngleRad) {
            loadPositionRad = maxAngleRad
            if (loadVelocityRadPerSec > 0) loadVelocityRadPerSec = 0.0
        }
    }
}
