package org.marsroboticsassociation.controllab.arm;

/**
 * Mutable bag of the arm plant's physical parameters, shared by the engine and both {@link ArmPlant}
 * implementations. Live edits from the sidebar mutate these fields; the engine then either calls
 * {@link ArmPlant#applyLiveParams()} (for fields the sim can change in place) or rebuilds the plant
 * via {@link ArmPlant#seedFrom} (for structural fields like backlash).
 *
 * <p>All angles are radians from horizontal. Voltages are volts.
 */
public class ArmPlantConfig {

    // --- Motor / encoder / geometry (structural; changing needs a rebuild) ---
    public int ticksPerRev = 28;
    public double gearRatio = 100.0;
    /** Angle from horizontal when the encoder reads 0. */
    public double encoderZeroOffsetRad = 0.0;
    /** Back hard stop (radians from horizontal). */
    public double minAngleRad = Math.toRadians(-100);
    /** Front hard stop (radians from horizontal). */
    public double maxAngleRad = Math.toRadians(45);

    // --- Feedforward / dynamics (structural; matches the "real robot") ---
    // Defaults model a deliberately heavy end-effector: gravity torque (kG) and inertia (kA) are
    // large, so the arm is genuinely hard to control. This sharpens the differences between the
    // controllers — the integral-bearing, model-based ones hold and settle noticeably better than
    // the fixed-gain PD, and the gravity free-fall across the backlash gap is dramatic.
    public double kS = 0.3;   // static friction voltage
    public double kG = 3.5;   // gravity voltage at horizontal (heavy end-effector)
    public double kV = 1.2;   // V/(rad/s)
    public double kA = 0.35;  // V/(rad/s^2) (high inertia)

    // --- Backlash (structural) ---
    public double backlashRad = Math.toRadians(5.0);

    // --- Backlash contact + load friction (live-settable) ---
    public double contactStiffness = 500.0;  // V per rad of tooth penetration
    public double contactDamping = 2.0;       // V per (rad/s) while engaged
    public double loadViscousFriction = 0.0;  // V per (rad/s) at the load bearing
    public double loadStaticFriction = 0.0;   // V at the load bearing

    // --- Disturbance (live-settable) ---
    public double disturbanceVoltage = 0.0;

    // --- Encoder read-timing jitter model (structural: rebuild to swap) ---
    public EncoderKind encoderKind = EncoderKind.CONTROL_HUB;
    public long encoderSeed = 1L;

    /** Ticks per output-shaft radian, derived from ticksPerRev/gearRatio. */
    public double ticksPerRad() {
        return ticksPerRev * gearRatio / (2.0 * Math.PI);
    }

    /** The encoder read-timing jitter models available in the sim. */
    public enum EncoderKind {
        /** No read-timing jitter (exact live position). */
        NONE,
        /** Small Control-Hub jitter (~0.3 ms). */
        CONTROL_HUB,
        /** Larger Expansion-Hub RS485 jitter (~2 ms). */
        EXPANSION_HUB
    }
}
