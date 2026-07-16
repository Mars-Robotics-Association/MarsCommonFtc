package org.marsroboticsassociation.controllib.mechanism;

/**
 * A {@link SetpointProfile} that computes its own back-EMF ceilings from a {@link MechanismModel}
 * at plan time. This is the surface {@link MotorMechanismController} drives.
 *
 * <p>The profile owns the ceilings because only it can evaluate them without lag. The ceilings
 * depend on the profile's state, and the post-update state is itself the result of applying them
 * — an algebraic loop — so any outside caller could only evaluate them at the <em>pre-update</em>
 * state, one step behind where the plan actually runs. A profile that owns the model evaluates
 * its ceilings at its own state at the moment it plans, and can look ahead along its own motion.
 *
 * <p>The controller never rewrites limits; it only forwards the
 * voltage available for feedforward each loop via {@link #setAvailableVoltage}.
 */
public interface ModelAwareSetpointProfile extends SetpointProfile {

    /**
     * The voltage available to motion this loop (bus voltage minus the feedback margin). Must be
     * called before each {@link #update}; the ceilings are computed from it at plan time.
     */
    void setAvailableVoltage(double availableVoltage);
}
