package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.abs
import kotlin.math.sign

/**
 * The physics model of a DC-motor driven mechanism, shared by the estimator ([MotorMechanismEkf])
 * and the controller ([MotorMechanismController]). Building one model and handing it to both
 * guarantees they agree on the constants.
 *
 * <p>The feedforward relationship between motion and voltage is:
 * <pre>
 *   voltage = kS*sign(velocity) + kV*velocity + kA*acceleration + gravityVoltage(position)
 * </pre>
 *
 * Read forward it predicts acceleration from an applied voltage (what the filter's predict step
 * needs); read backward it predicts the voltage a desired motion needs (the F in the controller's
 * PIDF). The `kV*velocity` term is the motor's back-EMF: the faster it spins, the more applied
 * voltage is spent balancing the voltage the motor itself generates.
 *
 * <p>The only thing a concrete model supplies is gravity: a constant for a lift, an angle-dependent
 * term for an arm. See [ArmModel] and [LiftModel].
 *
 * <p>Units are the mechanism's own (radians and volts for an arm, length units and volts for a
 * lift); stay consistent.
 */
abstract class MechanismModel
protected constructor(
    private val kS: Double,
    private val kV: Double, // back-EMF / viscous term
    private val kA: Double,
) {

    /** Voltage needed to hold against gravity at this position. */
    abstract fun gravityVoltage(position: Double): Double

    /** Derivative of [gravityVoltage] with respect to position. Zero for a lift. */
    abstract fun gravityVoltageDerivative(position: Double): Double

    /** Forward dynamics: the acceleration produced by an applied voltage. */
    open fun acceleration(position: Double, velocity: Double, voltage: Double): Double {
        return (voltage - kS * sign(velocity) - kV * velocity - gravityVoltage(position)) / kA
    }

    /** Inverse dynamics: the voltage a desired motion needs (the feedforward term). */
    open fun feedforwardVoltage(position: Double, velocity: Double, acceleration: Double): Double {
        return feedforwardVoltage(position, velocity, acceleration, 1.0)
    }

    /**
     * Inverse dynamics with the static-friction term scaled by `staticFrictionScale` (in [0, 1]).
     * `kS` is the y-intercept of the voltage-versus-velocity line extrapolated from the moving
     * regime; near a zero-crossing that extrapolation stops describing the mechanism (stiction, not
     * the kinetic intercept, governs there), and on a lashy/flexy drivetrain motor- side
     * identification biases `kS` high. Scaling it toward zero as the setpoint decelerates into an
     * arrival keeps an over-estimate from fighting the brake. The rest of the feedforward is
     * unchanged. See [MotorMechanismController] for the taper policy.
     */
    open fun feedforwardVoltage(
        position: Double,
        velocity: Double,
        acceleration: Double,
        staticFrictionScale: Double,
    ): Double {
        return staticFrictionScale * kS * sign(velocity) +
            kV * velocity +
            kA * acceleration +
            gravityVoltage(position)
    }

    /** Partial derivative of acceleration with respect to position, for the EKF Jacobian. */
    open fun accelerationSlopeByPosition(position: Double): Double {
        return -gravityVoltageDerivative(position) / kA
    }

    /** Partial derivative of acceleration with respect to velocity (the back-EMF term). */
    open fun accelerationSlopeByVelocity(): Double {
        return -kV / kA
    }

    /**
     * The steady velocity the motor can hold while travelling in `travelDirection` at this
     * position, given the available voltage: `(availableVoltage - kS - gravityAlongTravel) / kV`,
     * where `gravityAlongTravel` is the gravity voltage signed by the direction of travel. Going
     * faster would need more voltage than exists, because back-EMF grows with speed. Used as a
     * velocity ceiling.
     *
     * <p>Gravity is charged in the direction of travel rather than at worst case: climbing, gravity
     * opposes the motion and lowers the ceiling; descending, it aids the motion and raises it, so a
     * mechanism is allowed to run down faster than it can drive up. `travelDirection` is any value
     * whose sign is the direction of intended motion (e.g. target minus current position).
     */
    open fun maxSustainableVelocity(
        availableVoltage: Double,
        position: Double,
        travelDirection: Double,
    ): Double {
        val gravityAlongTravel = sign(travelDirection) * gravityVoltage(position)
        return (availableVoltage - kS - gravityAlongTravel) / kV
    }

    /**
     * The acceleration the motor can still deliver at this position and velocity given the
     * available voltage: `(availableVoltage - kS - kV*|velocity| - |gravity|) / kA`.
     *
     * <p>This is the more immediate face of the back-EMF limit. The `kV*|velocity|` term means the
     * headroom for acceleration shrinks the instant the mechanism moves, not just at top speed; it
     * reaches zero exactly at [maxSustainableVelocity]. Friction is taken at worst case (opposing
     * the motion), but gravity is charged in the direction of travel: descending, it aids the
     * acceleration and opens up headroom the worst case would have hidden. `travelDirection` is any
     * value whose sign is the direction of intended motion.
     */
    open fun maxSustainableAcceleration(
        availableVoltage: Double,
        position: Double,
        velocity: Double,
        travelDirection: Double,
    ): Double {
        val gravityAlongTravel = sign(travelDirection) * gravityVoltage(position)
        return (availableVoltage - kS - kV * abs(velocity) - gravityAlongTravel) / kA
    }

    /**
     * The deceleration the motor can deliver while braking at this position and velocity: `(
     * availableVoltage + kV*|velocity| - kS - |gravity|) / kA`.
     *
     * <p>The key difference from [maxSustainableAcceleration] is the sign of the back-EMF term:
     * when braking, the voltage the spinning motor generates *aids* the brake, so the available
     * deceleration *grows* with speed instead of shrinking.
     *
     * <p>Unlike the acceleration and velocity ceilings, gravity is held at worst case (opposing the
     * brake) here on purpose, so this ceiling takes no direction. The braking limit is what stops
     * the setpoint at the target, so an optimistic value overshoots; and worst case is also the
     * true value for the case that matters most, braking a gravity-driven descent.
     */
    open fun maxSustainableDeceleration(
        availableVoltage: Double,
        position: Double,
        velocity: Double,
    ): Double {
        return (availableVoltage + kV * abs(velocity) - kS - abs(gravityVoltage(position))) / kA
    }

    fun getKS(): Double = kS

    fun getKV(): Double = kV

    fun getKA(): Double = kA
}
