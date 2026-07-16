package org.marsroboticsassociation.controllab.arm;

/**
 * Uniform view over the arm plant simulations so the renderer, engine, and metrics do not care
 * which one is live. {@link org.marsroboticsassociation.controllib.sim.ArmMotorSim} (rigid),
 * {@link org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim} (two-inertia), and
 * {@link org.marsroboticsassociation.controllib.sim.FlexArmMotorSim} (three-inertia, backlash +
 * structural flex) are all hidden behind this.
 *
 * <p>All angles are output-shaft radians measured from horizontal (positive = above horizontal),
 * matching the controllers and sims.
 *
 * <p>The rigid plant reports {@code motorPos == loadPos} and {@code isEngaged == true} so the arm
 * renderer can always draw both a load link and a motor-side link uniformly; only the lashy plants
 * make them diverge. The flex plant reports the <em>tip</em> as the true load — the thing that
 * visibly bounces.
 */
public interface ArmPlant {

    /**
     * Advance the plant one control period.
     *
     * @param dt         loop period in seconds (physics sub-steps internally)
     * @param power      normalized motor power in [-1, 1]
     * @param hubVoltage bus voltage in volts
     */
    void step(double dt, double power, double hubVoltage);

    /** Motor-side encoder integer tick position (what a real controller reads). */
    int getPositionTicks();

    /** Motor-side windowed velocity in ticks per second (what a real controller reads). */
    double getVelocityTps();

    /** True arm (load) angle in radians from horizontal — ground truth for scoring/drawing. */
    double getTruePositionRad();

    /** True arm (load) angular velocity in rad/s. */
    double getTrueVelocityRadPerSec();

    /** True motor-side angle in radians (output-referred). Equals the load angle for a rigid plant. */
    double getMotorPositionRad();

    /** Whether the gear teeth are currently in contact. Always true for a rigid plant. */
    boolean isEngaged();

    /** Total backlash at the output shaft in radians. Zero for a rigid plant. */
    double getBacklashRad();

    /**
     * The load's static rest compliance: radians of motor-to-load droop per volt of gravity
     * hold-voltage, from gear-contact and structural elasticity. Zero for a rigid plant. Together
     * with {@link #getBacklashRad}, this describes where the load rests relative to the motor:
     * {@code halfBacklash·sign(g) + compliance·g} away, in the direction gravity pulls.
     */
    double restComplianceRadPerVolt();

    /**
     * Rebuild the plant at the given load pose, preserving the current tuning config. Used on
     * hot-swap and on structural param edits so the arm keeps its pose (no jump home). The sims
     * always seed at rest, so {@code loadVel} is advisory and the new plant starts with zero
     * velocity.
     */
    void seedFrom(double loadRad, double loadVel);

    /**
     * Re-read the live-tunable config fields (disturbance, contact stiffness/damping, load friction,
     * encoder model) into the underlying sim without rebuilding. Structural params (backlash) still
     * require {@link #seedFrom}.
     */
    void applyLiveParams();
}
