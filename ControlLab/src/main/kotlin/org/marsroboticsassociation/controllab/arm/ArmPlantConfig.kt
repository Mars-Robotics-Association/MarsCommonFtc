package org.marsroboticsassociation.controllab.arm

import kotlin.math.PI

/**
 * Mutable bag of the arm plant's physical parameters, shared by the engine and both [ArmPlant]
 * implementations. Live edits from the sidebar mutate these fields; the engine then either calls
 * [ArmPlant.applyLiveParams] (for fields the sim can change in place) or rebuilds the plant
 * via [ArmPlant.seedFrom] (for structural fields like backlash).
 *
 * All angles are radians from horizontal. Voltages are volts.
 */
class ArmPlantConfig {
    // --- Motor / encoder / geometry (structural; changing needs a rebuild) ---
    @JvmField var ticksPerRev: Int = 28
    @JvmField var gearRatio: Double = 100.0
    /** Angle from horizontal when the encoder reads 0. */
    @JvmField var encoderZeroOffsetRad: Double = 0.0
    /**
     * Front hard stop (radians from horizontal): 45° below horizontal in front. Gravity loads the
     * arm into this stop.
     */
    @JvmField var minAngleRad: Double = Math.toRadians(-45.0)
    /**
     * Back hard stop (radians from horizontal), continuous over the top from [minAngleRad].
     * +225° is the same ray as −135° principal (45° below horizontal behind). Gravity loads the arm
     * into this stop. Span is 270° via upright, not the short 90° arc under the robot.
     */
    @JvmField var maxAngleRad: Double = Math.toRadians(225.0)

    // --- Feedforward / dynamics (structural; matches the "real robot") ---
    // Defaults model a deliberately heavy end-effector: gravity torque (kG) and inertia (kA) are
    // large, so the arm is genuinely hard to control. This sharpens the differences between the
    // controllers — the integral-bearing, model-based ones hold and settle noticeably better than
    // the fixed-gain PD, and the gravity free-fall across the backlash gap is dramatic.
    @JvmField var kS: Double = 0.3 // static friction voltage
    @JvmField var kG: Double = 3.5 // gravity voltage at horizontal (heavy end-effector)
    @JvmField var kV: Double = 1.2 // V/(rad/s)
    @JvmField var kA: Double = 0.35 // V/(rad/s^2) (high inertia)

    // --- Backlash (structural) ---
    @JvmField var backlashRad: Double = Math.toRadians(5.0)

    // --- Backlash contact + load friction (live-settable) ---
    @JvmField var contactStiffness: Double = 500.0 // V per rad of tooth penetration
    @JvmField var contactDamping: Double = 2.0 // V per (rad/s) while engaged
    @JvmField var loadViscousFriction: Double = 0.0 // V per (rad/s) at the load bearing
    @JvmField var loadStaticFriction: Double = 0.0 // V at the load bearing

    // --- Arm structural flex (live-settable; used by the flex plant only) ---
    // A long heavy FTC arm's first bending mode: a few Hz, nearly undamped. This is what makes the
    // arm bouncy on the way down — the mode lives behind the lash where nothing can damp it.
    @JvmField var flexHz: Double = 3.0 // tip-on-spring natural frequency
    @JvmField var flexZeta: Double = 0.03 // structural damping ratio

    // --- Disturbance (live-settable) ---
    @JvmField var disturbanceVoltage: Double = 0.0

    // --- Encoder read-timing jitter model (structural: rebuild to swap) ---
    @JvmField var encoderKind: EncoderKind = EncoderKind.CONTROL_HUB
    @JvmField var encoderSeed: Long = 1L

    /** Ticks per output-shaft radian, derived from ticksPerRev/gearRatio. */
    fun ticksPerRad(): Double = ticksPerRev * gearRatio / (2.0 * PI)

    /** The encoder read-timing jitter models available in the sim. */
    enum class EncoderKind {
        /** No read-timing jitter (exact live position). */
        NONE,
        /** Small Control-Hub jitter (~0.3 ms). */
        CONTROL_HUB,
        /** Larger Expansion-Hub RS485 jitter (~2 ms). */
        EXPANSION_HUB,
    }
}
