package org.marsroboticsassociation.controllib.sim

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Physics mock for a single-joint arm motor with gravity, gearbox backlash, <b>and structural arm
 * flex</b>.
 *
 * <p>Where [BacklashArmMotorSim] splits the plant into a motor and a rigid load across the
 * gear-tooth dead band, this splits the load itself in two: the <b>hub</b> (the shaft, hub, and the
 * near end of the arm) and the <b>tip</b> (the far end of the arm plus the end effector), joined by
 * a torsional spring that stands in for the arm tube's first bending mode. Three inertias total:
 * <pre>
 *   motor (θ_m) ──[dead-band gear contact]── hub (θ_h) ──[flex spring k_f, c_f]── tip (θ_t)
 * </pre>
 * <ul>
 * <li><b>Motor side</b> — rotor plus gearbox ahead of the lash. The encoder lives here, as do
 *   back-EMF ({@code kV}) and gearbox friction ({@code kS}).</li>
 * <li><b>Hub</b> — light: just the structure near the pivot. Joint bearing friction (settable) acts
 *   here.</li>
 * <li><b>Tip</b> — carries most of the load inertia and most of the gravity torque. Only the flex
 *   spring's own tiny structural damping acts on it.</li>
 * </ul>
 *
 * <p>All angles are output-shaft (arm) radians from horizontal (positive = above horizontal).
 *
 * <p><b>Why this exists.</b> The two-inertia backlash plant cannot reproduce the classic
 * long-heavy-arm behavior of bouncing all the way down: with a rigid load, gravity presses the
 * teeth onto one face for the whole descent and the pair moves as one body. The bounce is the arm's
 * own bending resonance — a few Hz, nearly undamped, and <em>behind</em> the lash where neither
 * back-EMF, gearbox friction, nor a motor-encoder controller can reach it. Each swing of the tip
 * unloads the gear mesh (easiest where gravity preload is light), the arm free-falls through the
 * lash, and the re-impact pumps gravity energy back into the resonance. On the way up, drive torque
 * and gravity load the same tooth face, the mesh never unloads, and the mode stays quiet — the
 * down/up asymmetry seen on real robots.
 *
 * <p><b>Flex parameterization.</b> Rather than a raw spring constant, the flex is tuned by the
 * tip-on-spring natural frequency {@code flexHz} and damping ratio {@code flexZeta}, the two
 * numbers you can eyeball on a real arm (pluck the tip, count the wobble):
 * <pre>
 *   k_f = (2π·flexHz)² · kA_tip        c_f = 2·flexZeta·sqrt(k_f · kA_tip)
 * </pre>
 * A long heavy FTC arm rings at roughly 2–5 Hz with ζ ≈ 0.02–0.05.
 *
 * <p><b>Equations of motion</b> (voltage-equivalent units matching [ArmMotorSim]: "torque" in
 * volts, "inertia" is {@code kA} in V·s²/rad):
 * <pre>
 *   kA_m · dω_m/dt = u − kV·ω_m − kS·sign(ω_m) − τ_contact
 *   kA_h · dω_h/dt = τ_contact − τ_flex − (1−g_tip)·kG·cos(θ_h) − kV_L·ω_h − kS_L·sign(ω_h)
 *   kA_t · dω_t/dt = τ_flex − g_tip·kG·cos(θ_t)
 *
 *   τ_contact = dead-band spring on (θ_m − θ_h), as in BacklashArmMotorSim
 *   τ_flex    = k_f·(θ_h − θ_t) + c_f·(ω_h − ω_t)
 * </pre>
 * <p>Integrated with 4th-order Runge-Kutta on a small internal sub-step. Hard stops clamp the hub
 * and tip positions and zero their velocities. The encoder is fed the motor-side velocity.
 *
 * <p><b>Note on defaults.</b> The inertia split (motor / hub / tip), the tip's share of gravity,
 * and the contact parameters are not known from sysid; the defaults are chosen to be qualitatively
 * realistic for a long heavy arm and numerically stable, and are exposed via setters so they can be
 * swept (e.g. in ControlLab). Treat them as a starting point, not measured values.
 *
 * <p>Typical use mirrors [BacklashArmMotorSim]; [getTruePositionRad] is the <em>tip</em> — the
 * thing you actually watch bounce.
 */
class FlexArmMotorSim(
    private val kS: Double, // motor-side static friction voltage
    private val kG: Double, // gravity voltage at horizontal (split hub/tip)
    private val kV: Double, // motor-side viscous damping, V/(rad/s)
    kA: Double,
    ticksPerRev: Int,
    gearRatio: Double,
    private val encoderZeroOffsetRad: Double,
    private val minAngleRad: Double,
    private val maxAngleRad: Double,
    initialAngleRad: Double,
    backlashRad: Double,
    flexHz: Double,
    flexZeta: Double,
) {

    private val kATotal: Double = kA
    // Inertia split. Reflected rotor inertia is still large in a high-ratio gearbox, but a "long
    // and heavy" arm puts most of the load inertia at the tip.
    private var motorInertiaFraction = 0.50
    private var hubInertiaFraction = 0.05 // tip gets the remainder
    private var tipGravityShare = 0.85

    // Hub (joint bearing) friction. Defaults to zero, like BacklashArmMotorSim's load friction.
    private var kVLoad = 0.0
    private var kSLoad = 0.0

    // Dead-band spring contact between motor and hub.
    private val halfBacklashRad: Double = maxOf(0.0, backlashRad) / 2.0
    private var contactStiffness = 500.0 // V per rad of tooth penetration
    private var contactDamping = 2.0 // V per rad/s of relative velocity while engaged

    // Structural flex between hub and tip, parameterized by natural frequency + damping ratio.
    private var flexHzState = 0.0
    private var flexZetaState = 0.0
    private var kFlex = 0.0 // V per rad, derived
    private var cFlex = 0.0 // V per rad/s, derived

    // Encoder conversion
    private val ticksPerRad: Double = (ticksPerRev * gearRatio) / (2.0 * PI)

    // Largest internal integration sub-step (stiff contact spring, light hub inertia).
    private var maxInternalDtSec = 0.0005

    private lateinit var encoder: EncoderSim

    // True state (output shaft, radians from horizontal)
    private var motorPositionRad = 0.0
    private var motorVelocityRadPerSec = 0.0
    private var hubPositionRad = 0.0
    private var hubVelocityRadPerSec = 0.0
    private var tipPositionRad = 0.0
    private var tipVelocityRadPerSec = 0.0
    private var disturbanceVoltage = 0.0

    init {
        setFlex(flexHz, flexZeta)
        seedAtRest(clampToStops(initialAngleRad))
        seedEncoder()
    }

    /**
     * Seed all three bodies in static equilibrium at the given tip angle: the flex spring deflects
     * to hold the tip's gravity, and the motor rests on the gravity-loaded contact face, so there
     * is no start-up transient.
     */
    private fun seedAtRest(tipAngleRad: Double) {
        tipPositionRad = tipAngleRad
        tipVelocityRadPerSec = 0.0

        val tipHold = tipGravityShare * kG * cos(tipPositionRad)
        hubPositionRad = tipPositionRad + tipHold / kFlex
        hubVelocityRadPerSec = 0.0

        // The contact carries the whole arm's gravity at rest.
        val holdTorque = tipHold + (1.0 - tipGravityShare) * kG * cos(hubPositionRad)
        val penetration = holdTorque / contactStiffness
        motorPositionRad = hubPositionRad + penetration + sign(penetration) * halfBacklashRad
        motorVelocityRadPerSec = 0.0
    }

    private fun seedEncoder() {
        encoder = EncoderSim()
        seedEncoderState()
    }

    private fun seedEncoderState() {
        val initialTicksDouble = (motorPositionRad - encoderZeroOffsetRad) * ticksPerRad
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

    /**
     * Sets the arm's structural flex mode.
     *
     * @param hz natural frequency of the tip on the flex spring, in Hz
     * @param zeta damping ratio (dimensionless)
     * @throws IllegalArgumentException if [hz] or [zeta] is not positive
     */
    fun setFlex(hz: Double, zeta: Double) {
        if (hz <= 0 || zeta <= 0) {
            throw IllegalArgumentException("flexHz and flexZeta must be positive")
        }
        this.flexHzState = hz
        this.flexZetaState = zeta
        recomputeFlex()
    }

    /**
     * Sets the inertia split. The tip receives {@code 1 − motorFraction − hubFraction} of the total
     * {@code kA}.
     *
     * @param motorFraction fraction of total inertia on the motor side, in (0, 1)
     * @param hubFraction fraction of total inertia on the hub, in (0, 1)
     * @throws IllegalArgumentException if any resulting fraction is not positive
     */
    fun setInertiaFractions(motorFraction: Double, hubFraction: Double) {
        val tip = 1.0 - motorFraction - hubFraction
        if (motorFraction <= 0 || hubFraction <= 0 || tip <= 0) {
            throw IllegalArgumentException("all three inertia fractions must be positive")
        }
        this.motorInertiaFraction = motorFraction
        this.hubInertiaFraction = hubFraction
        recomputeFlex() // k_f is tied to the tip inertia at the requested flexHz
    }

    /**
     * Sets the tip's share of the gravity torque, in [0, 1]; the hub carries the rest.
     *
     * @param share tip gravity share
     * @throws IllegalArgumentException if outside [0, 1]
     */
    fun setTipGravityShare(share: Double) {
        if (share < 0 || share > 1) {
            throw IllegalArgumentException("tip gravity share must be in [0, 1]")
        }
        this.tipGravityShare = share
    }

    /** Sets the contact spring stiffness in volts per radian of tooth penetration. */
    fun setContactStiffness(vPerRad: Double) {
        this.contactStiffness = vPerRad
    }

    /** Sets the contact damping in volts per (rad/s) of relative velocity while engaged. */
    fun setContactDamping(vPerRadPerSec: Double) {
        this.contactDamping = vPerRadPerSec
    }

    /** Sets the hub (joint bearing) friction: viscous (V per rad/s) and static (V). */
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
        seedEncoderState()
    }

    private fun recomputeFlex() {
        val omegaN = 2.0 * PI * flexHzState
        val kATip = kATotal * (1.0 - motorInertiaFraction - hubInertiaFraction)
        this.kFlex = omegaN * omegaN * kATip
        this.cFlex = 2.0 * flexZetaState * sqrt(kFlex * kATip)
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

    /**
     * Returns the true arm tip position in radians from horizontal — the thing you watch bounce.
     */
    fun getTruePositionRad(): Double {
        return tipPositionRad
    }

    /** Returns the true arm tip angular velocity in rad/s. */
    fun getTrueVelocityRadPerSec(): Double {
        return tipVelocityRadPerSec
    }

    /** Returns the true hub position in radians (between the lash and the flex spring). */
    fun getHubPositionRad(): Double {
        return hubPositionRad
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
        return abs(motorPositionRad - hubPositionRad) > halfBacklashRad
    }

    /** Total backlash at the output shaft, in radians. */
    fun getBacklashRad(): Double {
        return 2.0 * halfBacklashRad
    }

    /**
     * The tip's static rest compliance: radians of motor-to-tip droop per volt of gravity
     * hold-voltage — contact-spring penetration plus flex-spring sag (the tip carries {@code
     * tipGravityShare} of the gravity across the flex spring). The total rest offset between motor
     * and tip is {@code halfBacklash·sign(g) + compliance·g}.
     */
    fun getRestComplianceRadPerVolt(): Double {
        return 1.0 / contactStiffness + tipGravityShare / kFlex
    }

    /** The arm structural natural frequency in Hz. */
    fun getFlexHz(): Double {
        return flexHzState
    }

    /** The arm structural damping ratio. */
    fun getFlexZeta(): Double {
        return flexZetaState
    }

    // ── dynamics ─────────────────────────────────────────────────────────────────
    //
    //   state = [θ_m, ω_m, θ_h, ω_h, θ_t, ω_t]

    /** Contact (gear-tooth) torque on the hub, in volts, from the relative state. */
    private fun contactTorque(
        thetaM: Double,
        omegaM: Double,
        thetaH: Double,
        omegaH: Double,
    ): Double {
        val delta = thetaM - thetaH
        if (delta > halfBacklashRad) {
            return contactStiffness * (delta - halfBacklashRad) + contactDamping * (omegaM - omegaH)
        }
        if (delta < -halfBacklashRad) {
            return contactStiffness * (delta + halfBacklashRad) + contactDamping * (omegaM - omegaH)
        }
        return 0.0 // teeth separated — no coupling
    }

    private fun derivatives(s: DoubleArray, u: Double, out: DoubleArray) {
        val thetaM = s[0]
        val omegaM = s[1]
        val thetaH = s[2]
        val omegaH = s[3]
        val thetaT = s[4]
        val omegaT = s[5]

        val kAMotor = kATotal * motorInertiaFraction
        val kAHub = kATotal * hubInertiaFraction
        val kATip = kATotal * (1.0 - motorInertiaFraction - hubInertiaFraction)

        val tauContact = contactTorque(thetaM, omegaM, thetaH, omegaH)
        val tauFlex = kFlex * (thetaH - thetaT) + cFlex * (omegaH - omegaT)

        out[0] = omegaM
        out[1] = (u - kV * omegaM - kS * sign(omegaM) - tauContact) / kAMotor
        out[2] = omegaH
        out[3] =
            (tauContact -
                tauFlex -
                (1.0 - tipGravityShare) * kG * cos(thetaH) -
                kVLoad * omegaH -
                kSLoad * sign(omegaH)) / kAHub
        out[4] = omegaT
        out[5] = (tauFlex - tipGravityShare * kG * cos(thetaT)) / kATip
    }

    private fun rk4Step(u: Double, dt: Double) {
        val s =
            doubleArrayOf(
                motorPositionRad,
                motorVelocityRadPerSec,
                hubPositionRad,
                hubVelocityRadPerSec,
                tipPositionRad,
                tipVelocityRadPerSec,
            )
        val n = s.size
        val k1 = DoubleArray(n)
        val k2 = DoubleArray(n)
        val k3 = DoubleArray(n)
        val k4 = DoubleArray(n)
        val tmp = DoubleArray(n)

        derivatives(s, u, k1)
        for (i in 0 until n) tmp[i] = s[i] + 0.5 * dt * k1[i]
        derivatives(tmp, u, k2)
        for (i in 0 until n) tmp[i] = s[i] + 0.5 * dt * k2[i]
        derivatives(tmp, u, k3)
        for (i in 0 until n) tmp[i] = s[i] + dt * k3[i]
        derivatives(tmp, u, k4)
        for (i in 0 until n) {
            s[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i])
        }

        motorPositionRad = s[0]
        motorVelocityRadPerSec = s[1]
        hubPositionRad = s[2]
        hubVelocityRadPerSec = s[3]
        tipPositionRad = s[4]
        tipVelocityRadPerSec = s[5]
    }

    private fun clampToStops(angle: Double): Double {
        if (angle < minAngleRad) return minAngleRad
        if (angle > maxAngleRad) return maxAngleRad
        return angle
    }

    private fun enforceHardStops() {
        // The stop is a physical bar the arm structure hits; clamp both load bodies against it.
        if (hubPositionRad <= minAngleRad) {
            hubPositionRad = minAngleRad
            if (hubVelocityRadPerSec < 0) hubVelocityRadPerSec = 0.0
        }
        if (hubPositionRad >= maxAngleRad) {
            hubPositionRad = maxAngleRad
            if (hubVelocityRadPerSec > 0) hubVelocityRadPerSec = 0.0
        }
        if (tipPositionRad <= minAngleRad) {
            tipPositionRad = minAngleRad
            if (tipVelocityRadPerSec < 0) tipVelocityRadPerSec = 0.0
        }
        if (tipPositionRad >= maxAngleRad) {
            tipPositionRad = maxAngleRad
            if (tipVelocityRadPerSec > 0) tipVelocityRadPerSec = 0.0
        }
    }
}
