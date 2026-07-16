package org.marsroboticsassociation.controllib.sim;

/**
 * Physics mock for a single-joint arm motor with gravity, gearbox backlash, <b>and structural arm
 * flex</b>.
 *
 * <p>Where {@link BacklashArmMotorSim} splits the plant into a motor and a rigid load across the
 * gear-tooth dead band, this splits the load itself in two: the <b>hub</b> (the shaft, hub, and the
 * near end of the arm) and the <b>tip</b> (the far end of the arm plus the end effector), joined by
 * a torsional spring that stands in for the arm tube's first bending mode. Three inertias total:
 *
 * <pre>
 *   motor (θ_m) ──[dead-band gear contact]── hub (θ_h) ──[flex spring k_f, c_f]── tip (θ_t)
 * </pre>
 *
 * <ul>
 *   <li><b>Motor side</b> — rotor plus gearbox ahead of the lash. The encoder lives here, as do
 *       back-EMF ({@code kV}) and gearbox friction ({@code kS}).</li>
 *   <li><b>Hub</b> — light: just the structure near the pivot. Joint bearing friction (settable)
 *       acts here.</li>
 *   <li><b>Tip</b> — carries most of the load inertia and most of the gravity torque. Only the
 *       flex spring's own tiny structural damping acts on it.</li>
 * </ul>
 *
 * <p>All angles are output-shaft (arm) radians from horizontal (positive = above horizontal).
 *
 * <p><b>Why this exists.</b> The two-inertia backlash plant cannot reproduce the classic
 * long-heavy-arm behavior of bouncing all the way down: with a rigid load, gravity presses the
 * teeth onto one face for the whole descent and the pair moves as one body. The bounce is the arm's
 * own bending resonance — a few Hz, nearly undamped, and <em>behind</em> the lash where neither
 * back-EMF, gearbox friction, nor a motor-encoder controller can reach it. Each swing of the tip
 * unloads the gear mesh (easiest where gravity preload is light), the arm free-falls through the
 * lash, and the re-impact pumps gravity energy back into the resonance. On the way up, drive torque
 * and gravity load the same tooth face, the mesh never unloads, and the mode stays quiet — the
 * down/up asymmetry seen on real robots.
 *
 * <p><b>Flex parameterization.</b> Rather than a raw spring constant, the flex is tuned by the
 * tip-on-spring natural frequency {@code flexHz} and damping ratio {@code flexZeta}, the two
 * numbers you can eyeball on a real arm (pluck the tip, count the wobble):
 * <pre>
 *   k_f = (2π·flexHz)² · kA_tip        c_f = 2·flexZeta·sqrt(k_f · kA_tip)
 * </pre>
 * A long heavy FTC arm rings at roughly 2–5 Hz with ζ ≈ 0.02–0.05.
 *
 * <p><b>Equations of motion</b> (voltage-equivalent units matching {@link ArmMotorSim}: "torque" in
 * volts, "inertia" is {@code kA} in V·s²/rad):
 * <pre>
 *   kA_m · dω_m/dt = u − kV·ω_m − kS·sign(ω_m) − τ_contact
 *   kA_h · dω_h/dt = τ_contact − τ_flex − (1−g_tip)·kG·cos(θ_h) − kV_L·ω_h − kS_L·sign(ω_h)
 *   kA_t · dω_t/dt = τ_flex − g_tip·kG·cos(θ_t)
 *
 *   τ_contact = dead-band spring on (θ_m − θ_h), as in BacklashArmMotorSim
 *   τ_flex    = k_f·(θ_h − θ_t) + c_f·(ω_h − ω_t)
 * </pre>
 *
 * <p>Integrated with 4th-order Runge-Kutta on a small internal sub-step. Hard stops clamp the hub
 * and tip positions and zero their velocities. The encoder is fed the motor-side velocity.
 *
 * <p><b>Note on defaults.</b> The inertia split (motor / hub / tip), the tip's share of gravity,
 * and the contact parameters are not known from sysid; the defaults are chosen to be qualitatively
 * realistic for a long heavy arm and numerically stable, and are exposed via setters so they can be
 * swept (e.g. in ControlLab). Treat them as a starting point, not measured values.
 *
 * <p>Typical use mirrors {@link BacklashArmMotorSim}; {@link #getTruePositionRad()} is the
 * <em>tip</em> — the thing you actually watch bounce.
 */
public class FlexArmMotorSim {

    // Feedforward / dynamics constants (output-shaft units, radians; volts)
    private final double kS;   // motor-side static friction voltage
    private final double kG;   // gravity voltage at horizontal (split hub/tip)
    private final double kV;   // motor-side viscous damping, V/(rad/s)

    private final double kATotal;
    // Inertia split. Reflected rotor inertia is still large in a high-ratio gearbox, but a "long
    // and heavy" arm puts most of the load inertia at the tip.
    private double motorInertiaFraction = 0.50;
    private double hubInertiaFraction = 0.05;   // tip gets the remainder
    private double tipGravityShare = 0.85;

    // Hub (joint bearing) friction. Defaults to zero, like BacklashArmMotorSim's load friction.
    private double kVLoad = 0.0;
    private double kSLoad = 0.0;

    // Dead-band spring contact between motor and hub.
    private final double halfBacklashRad;
    private double contactStiffness = 500.0;  // V per rad of tooth penetration
    private double contactDamping = 2.0;      // V per rad/s of relative velocity while engaged

    // Structural flex between hub and tip, parameterized by natural frequency + damping ratio.
    private double flexHz;
    private double flexZeta;
    private double kFlex;   // V per rad, derived
    private double cFlex;   // V per rad/s, derived

    // Encoder conversion
    private final double ticksPerRad;
    private final double encoderZeroOffsetRad;

    // Hard stops (radians from horizontal), enforced on hub and tip
    private final double minAngleRad;
    private final double maxAngleRad;

    // Largest internal integration sub-step (stiff contact spring, light hub inertia).
    private double maxInternalDtSec = 0.0005;

    private EncoderSim encoder;

    // True state (output shaft, radians from horizontal)
    private double motorPositionRad;
    private double motorVelocityRadPerSec;
    private double hubPositionRad;
    private double hubVelocityRadPerSec;
    private double tipPositionRad;
    private double tipVelocityRadPerSec;
    private double disturbanceVoltage = 0.0;

    /**
     * Construct a flexible-arm plant simulation with default contact and inertia-split parameters.
     *
     * @param kS                   motor-side static friction voltage
     * @param kG                   gravity voltage at horizontal (split between hub and tip)
     * @param kV                   motor-side velocity constant V/(rad/s) at output shaft
     * @param kA                   total acceleration constant V/(rad/s^2) at output shaft; split
     *                             between motor, hub, and tip by the default fractions
     * @param ticksPerRev          motor encoder ticks per motor revolution (e.g. 28)
     * @param gearRatio            output:input gear ratio (e.g. 100.0)
     * @param encoderZeroOffsetRad angle from horizontal when encoder reads 0
     * @param minAngleRad          minimum angle hard stop (radians from horizontal)
     * @param maxAngleRad          maximum angle hard stop (radians from horizontal)
     * @param initialAngleRad      initial arm (tip) angle (radians from horizontal)
     * @param backlashRad          total gearbox backlash at the output shaft (e.g. toRadians(5))
     * @param flexHz               arm structural (first bending) natural frequency in Hz (e.g. 3.0)
     * @param flexZeta             structural damping ratio (e.g. 0.03; real arms are barely damped)
     * @throws IllegalArgumentException if {@code flexHz} or {@code flexZeta} is not positive
     */
    public FlexArmMotorSim(double kS, double kG, double kV, double kA,
                           int ticksPerRev, double gearRatio,
                           double encoderZeroOffsetRad,
                           double minAngleRad, double maxAngleRad,
                           double initialAngleRad, double backlashRad,
                           double flexHz, double flexZeta) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kATotal = kA;

        this.ticksPerRad = (ticksPerRev * gearRatio) / (2.0 * Math.PI);
        this.encoderZeroOffsetRad = encoderZeroOffsetRad;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.halfBacklashRad = Math.max(0.0, backlashRad) / 2.0;

        setFlex(flexHz, flexZeta);
        seedAtRest(clampToStops(initialAngleRad));
        seedEncoder();
    }

    /**
     * Seed all three bodies in static equilibrium at the given tip angle: the flex spring deflects
     * to hold the tip's gravity, and the motor rests on the gravity-loaded contact face, so there
     * is no start-up transient.
     */
    private void seedAtRest(double tipAngleRad) {
        tipPositionRad = tipAngleRad;
        tipVelocityRadPerSec = 0.0;

        double tipHold = tipGravityShare * kG * Math.cos(tipPositionRad);
        hubPositionRad = tipPositionRad + tipHold / kFlex;
        hubVelocityRadPerSec = 0.0;

        // The contact carries the whole arm's gravity at rest.
        double holdTorque = tipHold + (1.0 - tipGravityShare) * kG * Math.cos(hubPositionRad);
        double penetration = holdTorque / contactStiffness;
        motorPositionRad = hubPositionRad + penetration
                + Math.signum(penetration) * halfBacklashRad;
        motorVelocityRadPerSec = 0.0;
    }

    private void seedEncoder() {
        this.encoder = new EncoderSim();
        seedEncoderState();
    }

    private void seedEncoderState() {
        double initialTicksDouble = (motorPositionRad - encoderZeroOffsetRad) * ticksPerRad;
        this.encoder.setState(0, initialTicksDouble);
        // Fill the ring buffer at zero velocity so getPosition/getVelocity work immediately.
        for (int i = 0; i < 6; i++) {
            this.encoder.advance(0.010, 0.0);
        }
    }

    /**
     * Advance the simulation by one time step.
     *
     * @param dt              time step in seconds
     * @param normalizedPower motor power in [-1, 1]
     * @param hubVoltage      bus voltage in volts (typically 12.0)
     */
    public void step(double dt, double normalizedPower, double hubVoltage) {
        double u = normalizedPower * hubVoltage + disturbanceVoltage;

        double remaining = dt;
        while (remaining > 1e-12) {
            double subDt = Math.min(maxInternalDtSec, remaining);
            rk4Step(u, subDt);
            enforceHardStops();
            encoder.advance(subDt, motorVelocityRadPerSec * ticksPerRad);
            remaining -= subDt;
        }
    }

    /**
     * Inject a voltage-equivalent disturbance into the motor side of the plant.
     *
     * @param v disturbance voltage in volts
     */
    public void setDisturbanceVoltage(double v) {
        disturbanceVoltage = v;
    }

    // ── tuning setters (sweepable, e.g. in ControlLab) ───────────────────────────

    /**
     * Sets the arm's structural flex mode.
     *
     * @param hz   natural frequency of the tip on the flex spring, in Hz
     * @param zeta damping ratio (dimensionless)
     * @throws IllegalArgumentException if {@code hz} or {@code zeta} is not positive
     */
    public void setFlex(double hz, double zeta) {
        if (hz <= 0 || zeta <= 0) {
            throw new IllegalArgumentException("flexHz and flexZeta must be positive");
        }
        this.flexHz = hz;
        this.flexZeta = zeta;
        recomputeFlex();
    }

    /**
     * Sets the inertia split. The tip receives {@code 1 − motorFraction − hubFraction} of the total
     * {@code kA}.
     *
     * @param motorFraction fraction of total inertia on the motor side, in (0, 1)
     * @param hubFraction   fraction of total inertia on the hub, in (0, 1)
     * @throws IllegalArgumentException if any resulting fraction is not positive
     */
    public void setInertiaFractions(double motorFraction, double hubFraction) {
        double tip = 1.0 - motorFraction - hubFraction;
        if (motorFraction <= 0 || hubFraction <= 0 || tip <= 0) {
            throw new IllegalArgumentException("all three inertia fractions must be positive");
        }
        this.motorInertiaFraction = motorFraction;
        this.hubInertiaFraction = hubFraction;
        recomputeFlex(); // k_f is tied to the tip inertia at the requested flexHz
    }

    /**
     * Sets the tip's share of the gravity torque, in [0, 1]; the hub carries the rest.
     *
     * @param share tip gravity share
     * @throws IllegalArgumentException if outside [0, 1]
     */
    public void setTipGravityShare(double share) {
        if (share < 0 || share > 1) {
            throw new IllegalArgumentException("tip gravity share must be in [0, 1]");
        }
        this.tipGravityShare = share;
    }

    /** Sets the contact spring stiffness in volts per radian of tooth penetration. */
    public void setContactStiffness(double vPerRad) {
        this.contactStiffness = vPerRad;
    }

    /** Sets the contact damping in volts per (rad/s) of relative velocity while engaged. */
    public void setContactDamping(double vPerRadPerSec) {
        this.contactDamping = vPerRadPerSec;
    }

    /** Sets the hub (joint bearing) friction: viscous (V per rad/s) and static (V). */
    public void setLoadFriction(double viscous, double staticVolts) {
        this.kVLoad = viscous;
        this.kSLoad = staticVolts;
    }

    /** Overrides the largest internal RK4 sub-step (seconds). Smaller = stiffer contact stays stable. */
    public void setMaxInternalDt(double sec) {
        this.maxInternalDtSec = sec;
    }

    /**
     * Replaces the encoder model (e.g. {@link EncoderSim#expansionHub(long)} for RS485 jitter).
     * Re-seeds at the current motor-side angle with a zero-velocity ring buffer.
     *
     * @param encoder non-null encoder model
     * @throws IllegalArgumentException if {@code encoder} is null
     */
    public void setEncoder(EncoderSim encoder) {
        if (encoder == null) {
            throw new IllegalArgumentException("encoder must not be null");
        }
        this.encoder = encoder;
        seedEncoderState();
    }

    private void recomputeFlex() {
        double omegaN = 2.0 * Math.PI * flexHz;
        double kATip = kATotal * (1.0 - motorInertiaFraction - hubInertiaFraction);
        this.kFlex = omegaN * omegaN * kATip;
        this.cFlex = 2.0 * flexZeta * Math.sqrt(kFlex * kATip);
    }

    // ── sensor outputs (what a real controller sees) ─────────────────────────────

    /** Returns the most recent integer tick position from the encoder (motor side, through the lash). */
    public int getPositionTicks() {
        return encoder.getPosition();
    }

    /** Returns the velocity in TPS from the encoder ring buffer (motor side). */
    public double getVelocityTps() {
        return encoder.getVelocityTps();
    }

    // ── ground truth (for scoring in sim only) ───────────────────────────────────

    /** Returns the true arm tip position in radians from horizontal — the thing you watch bounce. */
    public double getTruePositionRad() {
        return tipPositionRad;
    }

    /** Returns the true arm tip angular velocity in rad/s. */
    public double getTrueVelocityRadPerSec() {
        return tipVelocityRadPerSec;
    }

    /** Returns the true hub position in radians (between the lash and the flex spring). */
    public double getHubPositionRad() {
        return hubPositionRad;
    }

    /** Returns the true motor-side position in radians (output-referred). */
    public double getMotorPositionRad() {
        return motorPositionRad;
    }

    /** Returns the true motor-side angular velocity in rad/s (output-referred). */
    public double getMotorVelocityRadPerSec() {
        return motorVelocityRadPerSec;
    }

    /** True if the gear teeth are currently in contact (outside the dead band). */
    public boolean isEngaged() {
        return Math.abs(motorPositionRad - hubPositionRad) > halfBacklashRad;
    }

    /** Total backlash at the output shaft, in radians. */
    public double getBacklashRad() {
        return 2.0 * halfBacklashRad;
    }

    /**
     * The tip's static rest compliance: radians of motor-to-tip droop per volt of gravity
     * hold-voltage — contact-spring penetration plus flex-spring sag (the tip carries {@code
     * tipGravityShare} of the gravity across the flex spring). The total rest offset between motor
     * and tip is {@code halfBacklash·sign(g) + compliance·g}.
     */
    public double getRestComplianceRadPerVolt() {
        return 1.0 / contactStiffness + tipGravityShare / kFlex;
    }

    /** The arm structural natural frequency in Hz. */
    public double getFlexHz() {
        return flexHz;
    }

    /** The arm structural damping ratio. */
    public double getFlexZeta() {
        return flexZeta;
    }

    // ── dynamics ─────────────────────────────────────────────────────────────────
    //
    //   state = [θ_m, ω_m, θ_h, ω_h, θ_t, ω_t]

    /** Contact (gear-tooth) torque on the hub, in volts, from the relative state. */
    private double contactTorque(double thetaM, double omegaM, double thetaH, double omegaH) {
        double delta = thetaM - thetaH;
        if (delta > halfBacklashRad) {
            return contactStiffness * (delta - halfBacklashRad)
                    + contactDamping * (omegaM - omegaH);
        }
        if (delta < -halfBacklashRad) {
            return contactStiffness * (delta + halfBacklashRad)
                    + contactDamping * (omegaM - omegaH);
        }
        return 0.0;  // teeth separated — no coupling
    }

    private void derivatives(double[] s, double u, double[] out) {
        double thetaM = s[0], omegaM = s[1];
        double thetaH = s[2], omegaH = s[3];
        double thetaT = s[4], omegaT = s[5];

        double kAMotor = kATotal * motorInertiaFraction;
        double kAHub = kATotal * hubInertiaFraction;
        double kATip = kATotal * (1.0 - motorInertiaFraction - hubInertiaFraction);

        double tauContact = contactTorque(thetaM, omegaM, thetaH, omegaH);
        double tauFlex = kFlex * (thetaH - thetaT) + cFlex * (omegaH - omegaT);

        out[0] = omegaM;
        out[1] = (u - kV * omegaM - kS * Math.signum(omegaM) - tauContact) / kAMotor;
        out[2] = omegaH;
        out[3] = (tauContact - tauFlex
                - (1.0 - tipGravityShare) * kG * Math.cos(thetaH)
                - kVLoad * omegaH - kSLoad * Math.signum(omegaH)) / kAHub;
        out[4] = omegaT;
        out[5] = (tauFlex - tipGravityShare * kG * Math.cos(thetaT)) / kATip;
    }

    private void rk4Step(double u, double dt) {
        double[] s = {motorPositionRad, motorVelocityRadPerSec,
                hubPositionRad, hubVelocityRadPerSec,
                tipPositionRad, tipVelocityRadPerSec};
        int n = s.length;
        double[] k1 = new double[n], k2 = new double[n], k3 = new double[n], k4 = new double[n];
        double[] tmp = new double[n];

        derivatives(s, u, k1);
        for (int i = 0; i < n; i++) tmp[i] = s[i] + 0.5 * dt * k1[i];
        derivatives(tmp, u, k2);
        for (int i = 0; i < n; i++) tmp[i] = s[i] + 0.5 * dt * k2[i];
        derivatives(tmp, u, k3);
        for (int i = 0; i < n; i++) tmp[i] = s[i] + dt * k3[i];
        derivatives(tmp, u, k4);
        for (int i = 0; i < n; i++) {
            s[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
        }

        motorPositionRad = s[0];
        motorVelocityRadPerSec = s[1];
        hubPositionRad = s[2];
        hubVelocityRadPerSec = s[3];
        tipPositionRad = s[4];
        tipVelocityRadPerSec = s[5];
    }

    private double clampToStops(double angle) {
        if (angle < minAngleRad) return minAngleRad;
        if (angle > maxAngleRad) return maxAngleRad;
        return angle;
    }

    private void enforceHardStops() {
        // The stop is a physical bar the arm structure hits; clamp both load bodies against it.
        if (hubPositionRad <= minAngleRad) {
            hubPositionRad = minAngleRad;
            if (hubVelocityRadPerSec < 0) hubVelocityRadPerSec = 0;
        }
        if (hubPositionRad >= maxAngleRad) {
            hubPositionRad = maxAngleRad;
            if (hubVelocityRadPerSec > 0) hubVelocityRadPerSec = 0;
        }
        if (tipPositionRad <= minAngleRad) {
            tipPositionRad = minAngleRad;
            if (tipVelocityRadPerSec < 0) tipVelocityRadPerSec = 0;
        }
        if (tipPositionRad >= maxAngleRad) {
            tipPositionRad = maxAngleRad;
            if (tipVelocityRadPerSec > 0) tipVelocityRadPerSec = 0;
        }
    }
}
