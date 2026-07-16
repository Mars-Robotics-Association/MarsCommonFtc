package org.marsroboticsassociation.controllab.arm;

/**
 * App-level seam that hides the two controller shapes in ControlLib (Lineage A's {@code
 * void update(dt)} over an IMotor, and Lineage B's pure {@code calculate(...)->voltage} with its own
 * EKF) behind a single interface the {@link ArmEngine} can drive uniformly. ControlLib is not
 * modified; this abstraction lives only in ControlLab.
 *
 * <p>Each control period the engine calls {@link #step}, which runs the wrapped controller (reading
 * the current plant's sensors) and stashes the commanded power; the engine then reads
 * {@link #commandedPower()} to advance the plant. All angles are radians from horizontal.
 */
public interface ArmControlAdapter {

    /** Set the target arm angle (radians from horizontal). Immediate; used for live user input. */
    void setTargetRad(double rad);

    /**
     * Run one control cycle at the given loop period and bus voltage. Reads plant sensors and stashes
     * the commanded power (retrievable via {@link #commandedPower()}).
     */
    void step(double dt, double hubVoltage);

    /** The normalized motor power in [-1, 1] commanded by the most recent {@link #step}. */
    double commandedPower();

    /** Point the adapter at a (possibly hot-swapped) plant to read sensors from. */
    void setPlant(ArmPlant plant);

    // --- plotting getters ---

    /**
     * The endpoint the controller's profile actually drives to: the stated target plus any
     * rest-only backlash bias, clamped to the hard stops. Equals the stated target when the live
     * plant has no lash.
     */
    double profileTargetRad();

    /** The controller's estimated arm position (radians). */
    double estimatedPosRad();

    /** The controller's estimated arm velocity (rad/s). */
    double estimatedVelRad();

    /** The controller's trajectory/profile setpoint position (radians). */
    double trajPosRad();

    /** The controller's trajectory/profile setpoint velocity (rad/s). */
    double trajVelRad();

    /**
     * The controller's profile setpoint acceleration (rad/s²), for setpoint-smoothness metrics.
     * NaN when the wrapped controller does not expose one (Lineage A).
     */
    default double trajAccelRad() { return Double.NaN; }

    /** Short human-readable mode/type label for the metrics readout. */
    String modeLabel();
}
