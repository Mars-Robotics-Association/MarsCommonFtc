package org.marsroboticsassociation.controllib.mechanism

/**
 * Model-based PD feedback synthesis for a voltage-driven mechanism whose plant has already been
 * identified (SysID). Given the linearised error dynamics with good feedforward
 * <pre>
 *   kA · ë + kV · ė = −kP · e − kD · ė
 * </pre>
 *
 * (position error `e`, voltage-domain kV/kA from SysID), the closed loop is the standard
 * second-order form with
 * <pre>
 *   ωₙ² = kP / kA
 *   2 ζ ωₙ = (kV + kD) / kA
 * </pre>
 *
 * so the gains that realise a chosen natural frequency and damping ratio are
 * <pre>
 *   kP = kA · ωₙ²
 *   kD = 2 ζ ωₙ · kA − kV   (clamped to ≥ 0 if the plant is already overdamped at ωₙ)
 * </pre>
 * <p>This is *not* a second SysID of kP/kD from closed-loop data — it is plant SysID plus a
 * pole-placement design. Integral gain is left to the user (hold bias); this class only suggests
 * proportional and derivative.
 */
class FeedbackGainSynthesis private constructor() {

    /** Suggested PD gains plus the design inputs they came from. */
    class PdSuggestion
    internal constructor(
        /** Proportional gain, volts per unit position error. */
        @JvmField val kP: Double,
        /** Derivative gain, volts per (unit/sec) velocity error. Non-negative. */
        @JvmField val kD: Double,
        /** Plant kV used in the design, volts per (unit/sec). */
        @JvmField val kV: Double,
        /** Plant kA used in the design, volts per (unit/sec²). */
        @JvmField val kA: Double,
        /** Requested natural frequency, rad/s. */
        @JvmField val omegaN: Double,
        /** Requested damping ratio (dimensionless). */
        @JvmField val zeta: Double,
        /**
         * True when `2 ζ ωₙ kA < kV`, so kD was clamped to 0 — the open-loop plant already supplies
         * more damping than the design asked for at this ωₙ.
         */
        @JvmField val kDClampedToZero: Boolean,
    )

    companion object {
        /**
         * Suggest PD gains for the identified plant `(kV, kA)` and second-order specs `(omegaN,
         * zeta)`.
         *
         * @param kV back-EMF / viscous coefficient from SysID, volts per (unit/sec); must be ≥ 0
         * @param kA inertia coefficient from SysID, volts per (unit/sec²); must be > 0
         * @param omegaN desired closed-loop natural frequency, rad/s; must be > 0
         * @param zeta desired damping ratio (1 ≈ critically damped); must be > 0
         */
        @JvmStatic
        fun suggestPd(kV: Double, kA: Double, omegaN: Double, zeta: Double): PdSuggestion {
            if (!(kA > 0) || kA.isNaN()) {
                throw IllegalArgumentException("kA must be positive; got $kA")
            }
            if (!(kV >= 0) || kV.isNaN()) {
                throw IllegalArgumentException("kV must be non-negative; got $kV")
            }
            if (!(omegaN > 0) || omegaN.isNaN()) {
                throw IllegalArgumentException("omegaN must be positive; got $omegaN")
            }
            if (!(zeta > 0) || zeta.isNaN()) {
                throw IllegalArgumentException("zeta must be positive; got $zeta")
            }

            val kP = kA * omegaN * omegaN
            val kDRaw = 2.0 * zeta * omegaN * kA - kV
            val clamped = kDRaw < 0
            val kD = if (clamped) 0.0 else kDRaw
            return PdSuggestion(kP, kD, kV, kA, omegaN, zeta, clamped)
        }

        /** Same as [suggestPd] using the model's kV and kA (SysID-applied or hand-entered). */
        @JvmStatic
        fun suggestPd(model: MechanismModel, omegaN: Double, zeta: Double): PdSuggestion {
            return suggestPd(model.getKV(), model.getKA(), omegaN, zeta)
        }
    }
}
