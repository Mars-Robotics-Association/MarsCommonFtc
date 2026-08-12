package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.max
import kotlin.math.min

/**
 * [RuckigProfiler] that owns its back-EMF ceilings: before every replan it evaluates the
 * [MechanismModel] at its *own* current state and rewrites its velocity,
 * acceleration, and deceleration limits itself. [MotorMechanismController] drives this
 * [ModelAwareSetpointProfile] surface and only forwards the available voltage each loop.
 *
 * <p>Owning the ceilings lets each plan use limits evaluated at the exact state it plans from
 * (see [ModelAwareSetpointProfile] for why an outside caller cannot). Two ceilings are
 * handled specially:
 *
 * <ul>
 *   <li><b>Velocity, with one-step lookahead.</b> The sustainable-velocity ceiling depends on
 *       position through gravity, so a profile cruising exactly at the ceiling can find itself a
 *       hair *above* the ceiling one step later wherever the ceiling falls along the
 *       motion — a regime that forces the clamp-and-replan fallback every loop. The
 *       ceiling used for each plan is the minimum of its value here and at the position one step
 *       ahead along the current motion, so the plan never cruises into a falling ceiling.
 *   <li><b>Braking, at zero speed and at the target.</b> Back-EMF aids braking, so the model's
 *       deceleration ceiling shrinks as a stop progresses — and it also varies with position
 *       through gravity, collapsing toward the target on a gravity-loaded descent. A planner
 *       assumes its limits hold for the whole remaining move, so the stop is planned at the
 *       ceiling's minimum over the stop: evaluated at rest, at the worse of the current and the
 *       target position.
 *   </li>
 * </ul>
 *
 * <p>The configured limits passed at construction remain the mechanical caps; the model ceilings
 * only ever lower them. [setAvailableVoltage] must be called before each [update]
 * (the controller does); until it is called there is no voltage, hence no authority, and the
 * setpoint holds.
 */
class ModelAwareRuckigProfiler : RuckigProfiler, ModelAwareSetpointProfile {

    private val model: MechanismModel
    private val configuredMaxVelocity: Double
    private val configuredMaxAcceleration: Double
    private val configuredMaxDeceleration: Double

    private var availableVoltage = 0.0

    /**
     * @param model the shared mechanism model the ceilings are computed from
     * @param maxVelocity mechanical velocity cap
     * @param maxAcceleration mechanical acceleration cap (both speeding up and braking; the
     *     back-EMF ceilings throttle each below it independently)
     * @param maxJerk jerk limit ([UNLIMITED_JERK] for second-order profiles)
     * @param initialPosition the mechanism's position right now
     */
    constructor(
            model: MechanismModel,
            maxVelocity: Double,
            maxAcceleration: Double,
            maxJerk: Double,
            initialPosition: Double,
    ) : this(model, maxVelocity, maxAcceleration, maxAcceleration, maxJerk, initialPosition)

    /**
     * As the symmetric constructor, but with independent mechanical caps for speeding up and
     * braking (motion-frame, per the [SetpointProfile] contract). Pass
     * `maxAcceleration > maxDeceleration` to let a mechanism launch harder than it stops; the
     * back-EMF ceilings still throttle each cap independently below its configured value.
     *
     * @param maxAcceleration mechanical cap on acceleration that increases speed
     * @param maxDeceleration mechanical cap on braking
     */
    constructor(
            model: MechanismModel,
            maxVelocity: Double,
            maxAcceleration: Double,
            maxDeceleration: Double,
            maxJerk: Double,
            initialPosition: Double,
    ) : super(maxVelocity, maxAcceleration, maxDeceleration, maxJerk, initialPosition) {
        this.model = model
        this.configuredMaxVelocity = maxVelocity
        this.configuredMaxAcceleration = maxAcceleration
        this.configuredMaxDeceleration = maxDeceleration
    }

    override fun setAvailableVoltage(availableVoltage: Double) {
        this.availableVoltage = availableVoltage
    }

    override fun update(targetPosition: Double, dt: Double) {
        // Let the base class handle degenerate inputs with its usual no-op semantics.
        if (!(dt > 0) || targetPosition.isNaN() || targetPosition.isInfinite()) {
            super.update(targetPosition, dt)
            return
        }

        val position = getPosition()
        val velocity = getVelocity()
        val travelDirection = targetPosition - position
        // The acceleration ceiling models speeding up along the actual motion (its kV·|v| term
        // assumes the push and the motion agree), so gravity must be charged along the motion,
        // not along the travel: when the setpoint is moving the wrong way, "accelerating" means
        // gaining speed away from the target. Only at rest does the direction of travel decide.
        val motionDirection = if (velocity != 0.0) velocity else travelDirection

        val accelCeiling =
                model.maxSustainableAcceleration(
                        availableVoltage, position, velocity, motionDirection)
        setMaxAcceleration(min(configuredMaxAcceleration, max(0.0, accelCeiling)))

        // Braking planned at the ceiling's minimum over the stop: at rest (back-EMF aid gone),
        // and at the worse of here and the target position (on a gravity-loaded descent the
        // braking ceiling collapses toward the target; a plan that promises braking the end of
        // the stop will not have overshoots into the wrong-way state).
        val decelCeiling =
                min(
                        model.maxSustainableDeceleration(availableVoltage, position, 0.0),
                        model.maxSustainableDeceleration(availableVoltage, targetPosition, 0.0))
        setMaxDeceleration(min(configuredMaxDeceleration, max(0.0, decelCeiling)))

        // Velocity ceiling with one-step lookahead along the current motion, so a plan that
        // cruises at the ceiling is still inside the band when the next replan happens.
        val ceilingHere =
                model.maxSustainableVelocity(availableVoltage, position, travelDirection)
        val ceilingAhead =
                model.maxSustainableVelocity(
                        availableVoltage, position + velocity * dt, travelDirection)
        val velocityCeiling = min(ceilingHere, ceilingAhead)
        setMaxVelocity(min(configuredMaxVelocity, max(0.0, velocityCeiling)))

        super.update(targetPosition, dt)
    }
}
