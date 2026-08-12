package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

/**
 * PIDF position controller for a DC-motor driven mechanism, the companion to [MotorMechanismEkf]:
 * the filter estimates where the mechanism is, this drives it where you want it to go. Both take
 * the same [MechanismModel].
 *
 * <p>Each loop it does four things:
 * <ol>
 * <li><b>Profiles the target</b> through a [ModelAwareSetpointProfile] (a
 *   [ModelAwareRuckigProfiler] unless the caller supplies one), so the setpoint it chases is a
 *   planned, jerk-limited trajectory that lands at rest exactly on the target.
 * <li><b>Feeds forward</b> the voltage the model says the motion needs (the F in PIDF), including
 *   the back-EMF term.
 * <li><b>Corrects with PID</b> on the gap between the profile and the filter's estimate.
 * <li><b>Clamps</b> the result to the bus voltage.
 * </ol>
 *
 * <p>The measured <b>bus voltage</b> is passed in every loop, not fixed at construction, because an
 * FTC battery is only nominally 12 V: a fresh pack is a bit over 14 V and it sags under load and
 * drains over a match. Every limit keyed off voltage (the clamp and the back-EMF ceilings) tracks
 * that live value. The output is a voltage; convert it to a motor power with the same reading
 * (`power = volts / busVolts`) so sag does not change the behavior, and feed that applied voltage
 * back into the filter's predict step.
 *
 * <p><b>Back-EMF ceilings.</b> The profile owns them: before every replan it evaluates the shared
 * model at its own state and caps its velocity, acceleration, and deceleration limits to what the
 * motor can actually sustain there (see [ModelAwareRuckigProfiler] for how gravity and braking are
 * charged). The controller's only per-loop contribution is telling the profile how much voltage
 * this loop has — the bus voltage minus the feedback margin.
 */
class MotorMechanismController {

    private val model: MechanismModel

    // PID feedback gains, in volts per unit of error.
    private val kP: Double
    private val kI: Double
    private val kD: Double

    private val feedbackVoltageMargin: Double

    private val profile: ModelAwareSetpointProfile
    private var halfBacklash = 0.0
    private var backlashTaperVolts = 0.0
    private var restComplianceRadPerVolt = 0.0
    private var staticFrictionTaperVelocity = 0.0
    private var integral = 0.0
    private var lastVoltage = 0.0

    /**
     * Build with the default profiler, a [ModelAwareRuckigProfiler] under symmetric accel/decel
     * caps.
     *
     * @param model the shared mechanism model (arm or lift)
     * @param kP proportional gain, volts per unit of error
     * @param kI integral gain, volts per (unit-second)
     * @param kD derivative gain, volts per (unit/sec) of velocity error
     * @param maxVelocity velocity limit
     * @param maxAcceleration acceleration limit (the mechanical cap for both speeding up and
     *   braking; back-EMF throttles acceleration below it at speed)
     * @param maxJerk jerk limit
     * @param feedbackVoltageMargin volts held back from the profile's feedforward so the PID has
     *   room to correct (e.g. 1.5); the bus voltage comes in per loop
     * @param initialPosition the mechanism's position right now
     */
    constructor(
        model: MechanismModel,
        kP: Double,
        kI: Double,
        kD: Double,
        maxVelocity: Double,
        maxAcceleration: Double,
        maxJerk: Double,
        feedbackVoltageMargin: Double,
        initialPosition: Double,
    ) : this(
        model,
        kP,
        kI,
        kD,
        maxVelocity,
        maxAcceleration,
        maxAcceleration,
        maxJerk,
        feedbackVoltageMargin,
        initialPosition,
    )

    /**
     * As the symmetric constructor, but with independent mechanical caps for speeding up and
     * braking. Both are motion-frame limits ([SetpointProfile]): `maxAcceleration` bounds gaining
     * speed, `maxDeceleration` bounds braking, regardless of direction. Pass `maxAcceleration >
     * maxDeceleration` to let a mechanism launch harder than it stops (e.g. a flexible arm whose
     * arrival swing limits braking, not starting).
     *
     * @param maxAcceleration mechanical cap on acceleration that increases speed
     * @param maxDeceleration mechanical cap on braking
     */
    constructor(
        model: MechanismModel,
        kP: Double,
        kI: Double,
        kD: Double,
        maxVelocity: Double,
        maxAcceleration: Double,
        maxDeceleration: Double,
        maxJerk: Double,
        feedbackVoltageMargin: Double,
        initialPosition: Double,
    ) : this(
        model,
        kP,
        kI,
        kD,
        feedbackVoltageMargin,
        ModelAwareRuckigProfiler(
            model,
            maxVelocity,
            maxAcceleration,
            maxDeceleration,
            maxJerk,
            initialPosition,
        ),
    )

    /**
     * Construct with a caller-supplied setpoint profiler, e.g. a [ModelAwareRuckigProfiler] built
     * with non-default limits. The profile computes its own back-EMF ceilings at plan time from the
     * shared model; the controller only forwards the available voltage each loop.
     *
     * @param profile the setpoint profiler, already configured with its limits and initial position
     */
    constructor(
        model: MechanismModel,
        kP: Double,
        kI: Double,
        kD: Double,
        feedbackVoltageMargin: Double,
        profile: ModelAwareSetpointProfile,
    ) {
        this.model = model
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.feedbackVoltageMargin = feedbackVoltageMargin
        this.profile = profile
    }

    /**
     * Enable rest-only backlash compensation. A motor-encoder controller can only servo the motor
     * side of the gearbox; the load settles a half-backlash away from it, hanging on the
     * gravity-loaded tooth face. Biasing the target a half-backlash *against* gravity puts the
     * load, not the motor, on the stated target. Only the endpoint moves — the profile plans to the
     * biased target from the start, so the motion is unchanged and there is no correction hop at
     * arrival.
     *
     * <p>The bias is scaled by `gravityVoltage(target) / taperVolts`, clamped to ±1, so it carries
     * gravity's sign (an arm past vertical rests on the other face) and fades to zero near a crest,
     * where gravity is too weak to pin the resting face and a full bias would be a coin flip that
     * can double the error.
     *
     * <p>The biased target sits up to a half-backlash past the stated one: keep stated targets at
     * least that far inside any hard stop. Pass `backlashRad` of 0 to disable.
     *
     * @param backlash total lash at the output, in the mechanism's position units
     * @param taperVolts gravity hold-voltage below which the bias tapers toward zero (must be
     *   positive; e.g. the gravity voltage a few degrees off the crest)
     */
    fun setBacklashCompensation(backlash: Double, taperVolts: Double) {
        setBacklashCompensation(backlash, taperVolts, 0.0)
    }

    /**
     * As [setBacklashCompensation], plus a <b>rest compliance</b> term for elastic droop:
     * gear-tooth contact penetration and structural (arm-tube) sag both let the load rest
     * `compliance · gravityVoltage(target)` beyond the half-lash, proportionally to gravity. Unlike
     * the lash term this needs no crest taper — it vanishes with gravity on its own. On a real
     * mechanism, measure the rest droop at two angles: the constant part is the half-lash, the
     * gravity-proportional slope is this compliance.
     *
     * @param backlash total lash at the output, in the mechanism's position units
     * @param taperVolts gravity hold-voltage below which the lash bias tapers toward zero
     * @param complianceRadPerVolt elastic rest droop per volt of gravity hold-voltage (0 for a
     *   stiff mechanism)
     */
    fun setBacklashCompensation(
        backlash: Double,
        taperVolts: Double,
        complianceRadPerVolt: Double,
    ) {
        if (backlash < 0) {
            throw IllegalArgumentException("backlash must be >= 0, got $backlash")
        }
        if (backlash > 0 && taperVolts <= 0) {
            throw IllegalArgumentException("taperVolts must be > 0, got $taperVolts")
        }
        if (complianceRadPerVolt < 0) {
            throw IllegalArgumentException(
                "complianceRadPerVolt must be >= 0, got $complianceRadPerVolt"
            )
        }
        this.halfBacklash = backlash / 2.0
        this.backlashTaperVolts = taperVolts
        this.restComplianceRadPerVolt = complianceRadPerVolt
    }

    /**
     * Taper the static-friction feedforward toward zero as the setpoint decelerates below this
     * speed, so an over-estimated `kS` cannot fight the brake into an arrival. The model's `kS` is
     * the y-intercept of the voltage-versus-velocity line from the moving regime; that
     * extrapolation is least valid near a zero-crossing (stiction governs there), and motor- side
     * identification through a lashy/flexy drivetrain biases `kS` high (the direction- flipping
     * friction-flex deflection is collinear with the `sign(v)` regressor and cannot be separated
     * from it). The taper only bites while braking: full `kS` is kept when gaining speed (where it
     * helps break free), and it ramps linearly to zero as |setpoint velocity| falls from this
     * threshold to a stop.
     *
     * @param velocity speed below which a decelerating setpoint's `kS` feedforward tapers; 0 (the
     *   default) disables the taper (full `kS·sign(v)` at all times)
     */
    fun setStaticFrictionTaperVelocity(velocity: Double) {
        if (velocity < 0) {
            throw IllegalArgumentException("taper velocity must be >= 0, got $velocity")
        }
        this.staticFrictionTaperVelocity = velocity
    }

    /** Restart the profile from a known position at rest and clear the integrator. */
    fun reset(position: Double) {
        profile.reset(position)
        integral = 0.0
        lastVoltage = 0.0
    }

    /**
     * Compute the commanded motor voltage for this loop.
     *
     * @param targetPosition where you want the mechanism to end up
     * @param measuredPosition the filter's position estimate
     * @param measuredVelocity the filter's velocity estimate
     * @param busVoltage the measured battery/bus voltage this loop
     * @param dt seconds since the previous call
     * @return commanded voltage, clamped to +/- busVoltage
     */
    fun calculate(
        targetPosition: Double,
        measuredPosition: Double,
        measuredVelocity: Double,
        busVoltage: Double,
        dt: Double,
    ): Double {
        // Rest-only backlash compensation (see setBacklashCompensation): the profile chases the
        // biased endpoint so the load comes to rest on the stated one.
        val target = compensatedTarget(targetPosition)

        // Voltage available to the feedforward, leaving a margin for the PID to correct with. The
        // profile computes its own back-EMF ceilings from the shared model at plan time; it only
        // needs to know how much voltage this loop has.
        val availableVoltage = max(0.0, busVoltage - feedbackVoltageMargin)
        profile.setAvailableVoltage(availableVoltage)

        profile.update(target, dt)
        val setpointPosition = profile.getPosition()
        val setpointVelocity = profile.getVelocity()
        val setpointAcceleration = profile.getAcceleration()

        // F: feedforward from the model (back-EMF aware), with the static-friction term tapered
        // through an arrival so an over-estimated kS cannot fight the brake (see
        // setStaticFrictionTaperVelocity).
        val staticFrictionScale = staticFrictionScale(setpointVelocity, setpointAcceleration)
        val feedforward =
            model.feedforwardVoltage(
                setpointPosition,
                setpointVelocity,
                setpointAcceleration,
                staticFrictionScale,
            )

        // PID feedback on the profile-versus-estimate gap.
        val positionError = setpointPosition - measuredPosition
        val velocityError = setpointVelocity - measuredVelocity
        val tentativeIntegral = integral + positionError * dt
        val feedback = kP * positionError + kI * tentativeIntegral + kD * velocityError

        val voltage = feedforward + feedback
        val clamped = clamp(voltage, -busVoltage, busVoltage)
        // Anti-windup: only keep the new integral if the command was not saturated.
        if (clamped == voltage) {
            integral = tentativeIntegral
        }

        lastVoltage = clamped
        return clamped
    }

    /** The voltage returned by the most recent [calculate]. */
    fun getLastVoltage(): Double = lastVoltage

    fun getSetpointPosition(): Double = profile.getPosition()

    fun getSetpointVelocity(): Double = profile.getVelocity()

    fun getSetpointAcceleration(): Double = profile.getAcceleration()

    /**
     * The target the profile actually chases for a stated target: the stated position plus the
     * rest-only backlash bias (identity when compensation is disabled). Exposed so callers can
     * score the profile against its true endpoint, e.g. in telemetry or arrival checks.
     */
    fun compensatedTarget(targetPosition: Double): Double {
        return targetPosition + backlashBias(targetPosition)
    }

    /**
     * The rest bias for a stated target: a half-backlash signed by which tooth face gravity loads
     * there (tapered where the gravity hold-voltage is below `backlashTaperVolts`), plus the
     * gravity-proportional elastic droop.
     */
    private fun backlashBias(target: Double): Double {
        if (halfBacklash == 0.0 && restComplianceRadPerVolt == 0.0) {
            return 0.0
        }
        val gravity = model.gravityVoltage(target)
        var bias = restComplianceRadPerVolt * gravity
        if (halfBacklash > 0.0) {
            bias += halfBacklash * clamp(gravity / backlashTaperVolts, -1.0, 1.0)
        }
        return bias
    }

    /**
     * The [0, 1] scale for the static-friction feedforward this loop (see
     * [setStaticFrictionTaperVelocity]). Full `kS` unless the taper is enabled and the setpoint is
     * braking (velocity and acceleration opposite in sign); while braking it ramps linearly with
     * |velocity| from 0 at rest to 1 at the taper velocity.
     */
    private fun staticFrictionScale(velocity: Double, acceleration: Double): Double {
        if (staticFrictionTaperVelocity <= 0.0) {
            return 1.0
        }
        val braking = velocity * acceleration < 0.0
        if (!braking) {
            return 1.0
        }
        return clamp(abs(velocity) / staticFrictionTaperVelocity, 0.0, 1.0)
    }

    companion object {
        private fun clamp(value: Double, lo: Double, hi: Double): Double {
            return max(lo, min(hi, value))
        }
    }
}
