package org.marsroboticsassociation.controllib.mechanism

/**
 * A [SetpointProfile] that computes its own back-EMF ceilings from a [MechanismModel]
 * at plan time. This is the surface [MotorMechanismController] drives.
 *
 * <p>The profile owns the ceilings because only it can evaluate them without lag. The ceilings
 * depend on the profile's state, and the post-update state is itself the result of applying them
 * — an algebraic loop — so any outside caller could only evaluate them at the *pre-update*
 * state, one step behind where the plan actually runs. A profile that owns the model evaluates
 * its ceilings at its own state at the moment it plans, and can look ahead along its own motion.
 *
 * <p>The controller never rewrites limits; it only forwards the
 * voltage available for feedforward each loop via [setAvailableVoltage].
 */
interface ModelAwareSetpointProfile : SetpointProfile {

    /**
     * The voltage available to motion this loop (bus voltage minus the feedback margin). Must be
     * called before each [update]; the ceilings are computed from it at plan time.
     */
    fun setAvailableVoltage(availableVoltage: Double)
}
