package org.marsroboticsassociation.controllib.mechanism;

/**
 * A {@link SetpointProfile} that computes its own back-EMF ceilings from a {@link MechanismModel}
 * at plan time, instead of having {@link MotorMechanismController} rewrite its limits from the
 * outside each loop.
 *
 * <p>The externally-rewritten-limits interface has a structural flaw: the controller can only
 * evaluate the ceilings at the profile's <em>pre-update</em> state (evaluating after the update
 * would be an algebraic loop), so a profile cruising at the velocity ceiling is judged against a
 * ceiling computed one step behind where the plan actually runs. A profile that owns the model
 * evaluates its ceilings at its own state at the moment it plans — and can look ahead along its
 * own motion — which removes that lag entirely.
 *
 * <p>The controller detects this interface and, instead of rewriting limits, only forwards the
 * voltage available for feedforward each loop via {@link #setAvailableVoltage}.
 */
public interface ModelAwareSetpointProfile extends SetpointProfile {

    /**
     * The voltage available to motion this loop (bus voltage minus the feedback margin). Must be
     * called before each {@link #update}; the ceilings are computed from it at plan time.
     */
    void setAvailableVoltage(double availableVoltage);
}
