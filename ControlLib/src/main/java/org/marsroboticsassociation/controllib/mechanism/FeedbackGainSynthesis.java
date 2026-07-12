package org.marsroboticsassociation.controllib.mechanism;

/**
 * Model-based PD feedback synthesis for a voltage-driven mechanism whose plant has already been
 * identified (SysID). Given the linearised error dynamics with good feedforward
 *
 * <pre>
 *   kA · ë + kV · ė = −kP · e − kD · ė
 * </pre>
 *
 * (position error {@code e}, voltage-domain kV/kA from SysID), the closed loop is the standard
 * second-order form with
 *
 * <pre>
 *   ωₙ² = kP / kA
 *   2 ζ ωₙ = (kV + kD) / kA
 * </pre>
 *
 * so the gains that realise a chosen natural frequency and damping ratio are
 *
 * <pre>
 *   kP = kA · ωₙ²
 *   kD = 2 ζ ωₙ · kA − kV   (clamped to ≥ 0 if the plant is already overdamped at ωₙ)
 * </pre>
 *
 * <p>This is <em>not</em> a second SysID of kP/kD from closed-loop data — it is plant SysID plus a
 * pole-placement design. Integral gain is left to the user (hold bias); this class only suggests
 * proportional and derivative.
 */
public final class FeedbackGainSynthesis {

    private FeedbackGainSynthesis() {}

    /** Suggested PD gains plus the design inputs they came from. */
    public static final class PdSuggestion {
        /** Proportional gain, volts per unit position error. */
        public final double kP;
        /** Derivative gain, volts per (unit/sec) velocity error. Non-negative. */
        public final double kD;
        /** Plant kV used in the design, volts per (unit/sec). */
        public final double kV;
        /** Plant kA used in the design, volts per (unit/sec²). */
        public final double kA;
        /** Requested natural frequency, rad/s. */
        public final double omegaN;
        /** Requested damping ratio (dimensionless). */
        public final double zeta;
        /**
         * True when {@code 2 ζ ωₙ kA < kV}, so kD was clamped to 0 — the open-loop plant already
         * supplies more damping than the design asked for at this ωₙ.
         */
        public final boolean kDClampedToZero;

        PdSuggestion(
                double kP,
                double kD,
                double kV,
                double kA,
                double omegaN,
                double zeta,
                boolean kDClampedToZero) {
            this.kP = kP;
            this.kD = kD;
            this.kV = kV;
            this.kA = kA;
            this.omegaN = omegaN;
            this.zeta = zeta;
            this.kDClampedToZero = kDClampedToZero;
        }
    }

    /**
     * Suggest PD gains for the identified plant {@code (kV, kA)} and second-order specs {@code
     * (omegaN, zeta)}.
     *
     * @param kV back-EMF / viscous coefficient from SysID, volts per (unit/sec); must be ≥ 0
     * @param kA inertia coefficient from SysID, volts per (unit/sec²); must be &gt; 0
     * @param omegaN desired closed-loop natural frequency, rad/s; must be &gt; 0
     * @param zeta desired damping ratio (1 ≈ critically damped); must be &gt; 0
     */
    public static PdSuggestion suggestPd(double kV, double kA, double omegaN, double zeta) {
        if (!(kA > 0) || Double.isNaN(kA)) {
            throw new IllegalArgumentException("kA must be positive; got " + kA);
        }
        if (!(kV >= 0) || Double.isNaN(kV)) {
            throw new IllegalArgumentException("kV must be non-negative; got " + kV);
        }
        if (!(omegaN > 0) || Double.isNaN(omegaN)) {
            throw new IllegalArgumentException("omegaN must be positive; got " + omegaN);
        }
        if (!(zeta > 0) || Double.isNaN(zeta)) {
            throw new IllegalArgumentException("zeta must be positive; got " + zeta);
        }

        double kP = kA * omegaN * omegaN;
        double kDRaw = 2.0 * zeta * omegaN * kA - kV;
        boolean clamped = kDRaw < 0;
        double kD = clamped ? 0.0 : kDRaw;
        return new PdSuggestion(kP, kD, kV, kA, omegaN, zeta, clamped);
    }

    /**
     * Same as {@link #suggestPd(double, double, double, double)} using the model's kV and kA
     * (SysID-applied or hand-entered).
     */
    public static PdSuggestion suggestPd(MechanismModel model, double omegaN, double zeta) {
        return suggestPd(model.getKV(), model.getKA(), omegaN, zeta);
    }
}
