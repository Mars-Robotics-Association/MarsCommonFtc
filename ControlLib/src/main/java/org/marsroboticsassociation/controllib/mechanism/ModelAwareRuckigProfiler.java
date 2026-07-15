package org.marsroboticsassociation.controllib.mechanism;

/**
 * {@link RuckigProfiler} that owns its back-EMF ceilings: before every replan it evaluates the
 * {@link MechanismModel} at its <em>own</em> current state and rewrites its velocity,
 * acceleration, and deceleration limits itself. {@link MotorMechanismController} detects the
 * {@link ModelAwareSetpointProfile} interface and only forwards the available voltage each loop.
 *
 * <p>This removes the lag seam of the externally-rewritten-limits design, where the controller
 * evaluated the ceilings at the profile's pre-update state (the algebraic-loop workaround). Two
 * ceilings are handled specially:
 *
 * <ul>
 *   <li><b>Velocity, with one-step lookahead.</b> The sustainable-velocity ceiling depends on
 *       position through gravity, so a profile cruising exactly at the ceiling can find itself a
 *       hair <em>above</em> the ceiling one step later wherever the ceiling falls along the
 *       motion — the regime that used to force the clamp-and-replan fallback every loop. The
 *       ceiling used for each plan is the minimum of its value here and at the position one step
 *       ahead along the current motion, so the plan never cruises into a falling ceiling.
 *   <li><b>Braking, at zero speed and at the target.</b> Back-EMF aids braking, so the model's
 *       deceleration ceiling shrinks as a stop progresses — and it also varies with position
 *       through gravity, collapsing toward the target on a gravity-loaded descent. A planner
 *       assumes its limits hold for the whole remaining move, so the stop is planned at the
 *       ceiling's minimum over the stop: evaluated at rest, at the worse of the current and the
 *       target position. (The speed part is what {@code MotorMechanismController}'s
 *       conservative-braking option did; here both are inherent.)
 *   </li>
 * </ul>
 *
 * <p>The configured limits passed at construction remain the mechanical caps; the model ceilings
 * only ever lower them. {@link #setAvailableVoltage} must be called before each {@link #update}
 * (the controller does); until it is called there is no voltage, hence no authority, and the
 * setpoint holds.
 */
public class ModelAwareRuckigProfiler extends RuckigProfiler implements ModelAwareSetpointProfile {

    private final MechanismModel model;
    private final double configuredMaxVelocity;
    private final double configuredMaxAcceleration;

    private double availableVoltage = 0.0;

    /**
     * @param model the shared mechanism model the ceilings are computed from
     * @param maxVelocity mechanical velocity cap
     * @param maxAcceleration mechanical acceleration cap (both speeding up and braking; the
     *     back-EMF ceilings throttle each below it independently)
     * @param maxJerk jerk limit ({@link #UNLIMITED_JERK} for second-order profiles)
     * @param initialPosition the mechanism's position right now
     */
    public ModelAwareRuckigProfiler(
            MechanismModel model,
            double maxVelocity,
            double maxAcceleration,
            double maxJerk,
            double initialPosition) {
        super(maxVelocity, maxAcceleration, maxAcceleration, maxJerk, initialPosition);
        this.model = model;
        this.configuredMaxVelocity = maxVelocity;
        this.configuredMaxAcceleration = maxAcceleration;
    }

    @Override
    public void setAvailableVoltage(double availableVoltage) {
        this.availableVoltage = availableVoltage;
    }

    @Override
    public void update(double targetPosition, double dt) {
        // Let the base class handle degenerate inputs with its usual no-op semantics.
        if (!(dt > 0) || Double.isNaN(targetPosition) || Double.isInfinite(targetPosition)) {
            super.update(targetPosition, dt);
            return;
        }

        double position = getPosition();
        double velocity = getVelocity();
        double travelDirection = targetPosition - position;
        // The acceleration ceiling models speeding up along the actual motion (its kV·|v| term
        // assumes the push and the motion agree), so gravity must be charged along the motion,
        // not along the travel: when the setpoint is moving the wrong way, "accelerating" means
        // gaining speed away from the target. Only at rest does the direction of travel decide.
        double motionDirection = velocity != 0 ? velocity : travelDirection;

        double accelCeiling =
                model.maxSustainableAcceleration(
                        availableVoltage, position, velocity, motionDirection);
        setMaxAcceleration(Math.min(configuredMaxAcceleration, Math.max(0, accelCeiling)));

        // Braking planned at the ceiling's minimum over the stop: at rest (back-EMF aid gone),
        // and at the worse of here and the target position (on a gravity-loaded descent the
        // braking ceiling collapses toward the target; a plan must not promise braking the end
        // of the stop will not have — that is what caused overshoot into the wrong-way state).
        double decelCeiling =
                Math.min(
                        model.maxSustainableDeceleration(availableVoltage, position, 0.0),
                        model.maxSustainableDeceleration(availableVoltage, targetPosition, 0.0));
        setMaxDeceleration(Math.min(configuredMaxAcceleration, Math.max(0, decelCeiling)));

        // Velocity ceiling with one-step lookahead along the current motion, so a plan that
        // cruises at the ceiling is still inside the band when the next replan happens.
        double ceilingHere =
                model.maxSustainableVelocity(availableVoltage, position, travelDirection);
        double ceilingAhead =
                model.maxSustainableVelocity(
                        availableVoltage, position + velocity * dt, travelDirection);
        double velocityCeiling = Math.min(ceilingHere, ceilingAhead);
        setMaxVelocity(Math.min(configuredMaxVelocity, Math.max(0, velocityCeiling)));

        super.update(targetPosition, dt);
    }
}
