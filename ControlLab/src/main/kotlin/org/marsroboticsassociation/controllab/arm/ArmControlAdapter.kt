package org.marsroboticsassociation.controllab.arm

/**
 * App-level seam that hides the two controller shapes in ControlLib (Lineage A's
 * `void update(dt)` over an IMotor, and Lineage B's pure `calculate(...)->voltage` with its own
 * EKF) behind a single interface the [ArmEngine] can drive uniformly. ControlLib is not
 * modified; this abstraction lives only in ControlLab.
 *
 * Each control period the engine calls [step], which runs the wrapped controller (reading
 * the current plant's sensors) and stashes the commanded power; the engine then reads
 * [commandedPower] to advance the plant. All angles are radians from horizontal.
 */
interface ArmControlAdapter {
    /** Set the target arm angle (radians from horizontal). Immediate; used for live user input. */
    fun setTargetRad(rad: Double)

    /**
     * Run one control cycle at the given loop period and bus voltage. Reads plant sensors and stashes
     * the commanded power (retrievable via [commandedPower]).
     */
    fun step(dt: Double, hubVoltage: Double)

    /** The normalized motor power in [-1, 1] commanded by the most recent [step]. */
    fun commandedPower(): Double

    /** Point the adapter at a (possibly hot-swapped) plant to read sensors from. */
    fun setPlant(plant: ArmPlant)

    // --- plotting getters ---

    /**
     * The endpoint the controller's profile actually drives to: the stated target plus any
     * rest-only backlash bias, clamped to the hard stops. Equals the stated target when the live
     * plant has no lash.
     */
    fun profileTargetRad(): Double

    /** The controller's estimated arm position (radians). */
    fun estimatedPosRad(): Double

    /** The controller's estimated arm velocity (rad/s). */
    fun estimatedVelRad(): Double

    /** The controller's trajectory/profile setpoint position (radians). */
    fun trajPosRad(): Double

    /** The controller's trajectory/profile setpoint velocity (rad/s). */
    fun trajVelRad(): Double

    /**
     * The controller's profile setpoint acceleration (rad/s²), for setpoint-smoothness metrics.
     * NaN when the wrapped controller does not expose one (Lineage A).
     */
    fun trajAccelRad(): Double = Double.NaN

    /** Short human-readable mode/type label for the metrics readout. */
    fun modeLabel(): String
}
