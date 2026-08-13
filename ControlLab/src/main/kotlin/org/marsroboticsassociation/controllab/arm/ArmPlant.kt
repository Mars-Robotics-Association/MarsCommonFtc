package org.marsroboticsassociation.controllab.arm

/**
 * Uniform view over the arm plant simulations so the renderer, engine, and metrics do not care
 * which one is live. [org.marsroboticsassociation.controllib.sim.ArmMotorSim] (rigid),
 * [org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim] (two-inertia), and
 * [org.marsroboticsassociation.controllib.sim.FlexArmMotorSim] (three-inertia, backlash +
 * structural flex) are all hidden behind this.
 *
 * All angles are output-shaft radians measured from horizontal (positive = above horizontal),
 * matching the controllers and sims.
 *
 * The rigid plant reports `motorPos == loadPos` and `isEngaged == true` so the arm renderer can
 * always draw both a load link and a motor-side link uniformly; only the lashy plants make them
 * diverge. The flex plant reports the *tip* as the true load — the thing that visibly bounces.
 */
interface ArmPlant {
    /**
     * Advance the plant one control period.
     *
     * @param dt loop period in seconds (physics sub-steps internally)
     * @param power normalized motor power in [-1, 1]
     * @param hubVoltage bus voltage in volts
     */
    fun step(dt: Double, power: Double, hubVoltage: Double)

    /** Motor-side encoder integer tick position (what a real controller reads). */
    val positionTicks: Int

    /** Motor-side windowed velocity in ticks per second (what a real controller reads). */
    val velocityTps: Double

    /** True arm (load) angle in radians from horizontal — ground truth for scoring/drawing. */
    val truePositionRad: Double

    /** True arm (load) angular velocity in rad/s. */
    val trueVelocityRadPerSec: Double

    /**
     * True motor-side angle in radians (output-referred). Equals the load angle for a rigid plant.
     */
    val motorPositionRad: Double

    /** Whether the gear teeth are currently in contact. Always true for a rigid plant. */
    val isEngaged: Boolean

    /** Total backlash at the output shaft in radians. Zero for a rigid plant. */
    val backlashRad: Double

    /**
     * The load's static rest compliance: radians of motor-to-load droop per volt of gravity
     * hold-voltage, from gear-contact and structural elasticity. Zero for a rigid plant. Together
     * with [getBacklashRad], this describes where the load rests relative to the motor:
     * `halfBacklash·sign(g) + compliance·g` away, in the direction gravity pulls.
     */
    fun restComplianceRadPerVolt(): Double

    /**
     * Rebuild the plant at the given load pose, preserving the current tuning config. Used on
     * hot-swap and on structural param edits so the arm keeps its pose (no jump home). The sims
     * always seed at rest, so `loadVel` is advisory and the new plant starts with zero velocity.
     */
    fun seedFrom(loadRad: Double, loadVel: Double)

    /**
     * Re-read the live-tunable config fields (disturbance, contact stiffness/damping, load
     * friction, encoder model) into the underlying sim without rebuilding. Structural params
     * (backlash) still require [seedFrom].
     */
    fun applyLiveParams()
}
