package org.marsroboticsassociation.controllib.sim;

/**
 * Physics mock for a single-joint arm motor with gravity <b>and gearbox backlash</b>.
 *
 * <p>Where {@link ArmMotorSim} treats the motor and arm as one rigid body, this models them as
 * <b>two inertias</b> coupled through the gear teeth, with a dead band between them:
 *
 * <ul>
 *   <li><b>Motor side</b> ({@code θ_m}, {@code ω_m}) &mdash; the rotor and everything ahead of the
 *       backlash. This is what the motor encoder measures. The reflected rotor inertia dominates a
 *       high-ratio FTC gearbox, so this side carries the bulk of {@code kA}.</li>
 *   <li><b>Load side</b> ({@code θ_L}, {@code ω_L}) &mdash; the arm itself, on the far side of the
 *       lash. <b>Gravity acts here.</b> Its own inertia is a fraction of the motor side.</li>
 * </ul>
 *
 * <p>All angles are output-shaft (arm) radians from horizontal (positive = above horizontal).
 *
 * <p><b>Dead-band spring contact.</b> Let {@code δ = θ_m − θ_L} and {@code h = backlash/2}. The gear
 * teeth only push when the relative position leaves the dead band:
 * <pre>
 *   δ &gt;  h :  penetration = δ − h   (forward face in contact)
 *   δ &lt; −h :  penetration = δ + h   (reverse face in contact)
 *   |δ| ≤ h :  penetration = 0       (free &mdash; teeth not touching)
 *   τ_contact = k_contact · penetration  (+ c_contact · (ω_m − ω_L) while engaged)
 * </pre>
 *
 * <p>The consequences are the ones that wreck arm controllers:
 * <ul>
 *   <li><b>Lost motion on reversal.</b> After a direction change the motor must travel the full
 *       backlash before the arm responds &mdash; the encoder moves while the arm does not.</li>
 *   <li><b>Gravity free-fall across the gap.</b> While the teeth are separated the arm is
 *       unsupported and accelerates under gravity (only its own small inertia), then re-engages with
 *       an impact.</li>
 *   <li><b>The encoder is blind to the load.</b> {@link #getPositionTicks()} reflects the
 *       <em>motor</em> side; the true arm angle can differ by up to the backlash. This is the
 *       "measured through the backlash" problem &mdash; a motor-encoder-only controller cannot see
 *       where the arm actually is across the gap.</li>
 * </ul>
 *
 * <p><b>Equations of motion</b> (voltage-equivalent units, matching {@link ArmMotorSim}; "torque"
 * is in volts, "inertia" is {@code kA} in V·s²/rad, "damping" is {@code kV} in V·s/rad):
 * <pre>
 *   kA_m · dω_m/dt = u − kV·ω_m − kS·sign(ω_m) − τ_contact
 *   kA_L · dω_L/dt = τ_contact − kG·cos(θ_L) − kV_L·ω_L − kS_L·sign(ω_L)
 * </pre>
 *
 * <p>Integrated with 4th-order Runge-Kutta on a small internal sub-step (the contact spring is stiff
 * relative to a 16 ms control loop, so a single 16 ms RK4 step would ring or diverge). Hard stops
 * clamp the <b>load</b> position and zero its velocity. The encoder is fed the motor-side velocity.
 *
 * <p><b>Note on defaults.</b> The split of inertia between motor and load, the contact stiffness,
 * and the contact damping are not yet known from sysid; the defaults here are chosen to be
 * qualitatively realistic and numerically stable, and are exposed via setters so they can be swept
 * (e.g. in ControlLab). Treat them as a starting point, not measured values.
 *
 * <p>Typical use:
 * <pre>
 *   BacklashArmMotorSim sim = new BacklashArmMotorSim(kS, kG, kV, kA, ticksPerRev, gearRatio,
 *           encoderZeroOffsetRad, minAngleRad, maxAngleRad, initialAngleRad, Math.toRadians(5));
 *   for (int i = 0; i &lt; 300; i++) {
 *       int    posTicks = sim.getPositionTicks();   // motor side — what a real controller sees
 *       double velTps   = sim.getVelocityTps();
 *       double power    = controller.update(dt);
 *       sim.step(dt, power, 12.0);
 *       double trueArm  = sim.getTruePositionRad();  // load side — ground truth for scoring
 *   }
 * </pre>
 */
public class BacklashArmMotorSim {

    // Feedforward / dynamics constants (output-shaft units, radians; volts)
    private final double kS;   // motor-side static friction voltage
    private final double kG;   // gravity voltage at horizontal (acts on the load)
    private final double kV;   // motor-side viscous damping, V/(rad/s)
    private final double kAMotor;  // motor-side inertia, V/(rad/s^2)
    private final double kALoad;   // load-side inertia, V/(rad/s^2)

    // Load-side friction (the arm bearing). Defaults to zero so the gravity free-fall through the
    // gap stays pronounced; set via setters if the real joint has meaningful bearing drag.
    private double kVLoad = 0.0;
    private double kSLoad = 0.0;

    // Dead-band spring contact between motor and load.
    private final double halfBacklashRad;   // h = backlash / 2
    private double contactStiffness;        // k_contact, V per rad of tooth penetration
    private double contactDamping;          // c_contact, V per rad/s of relative velocity while engaged

    // Encoder conversion
    private final double ticksPerRad;           // motor encoder ticks per output radian
    private final double encoderZeroOffsetRad;  // angle (from horizontal) when encoder reads 0

    // Hard stops (radians from horizontal), enforced on the load
    private final double minAngleRad;
    private final double maxAngleRad;

    // Largest internal integration sub-step. The stiff contact spring needs this to stay stable and
    // non-ringing at typical control-loop dt's (~16 ms).
    private double maxInternalDtSec = 0.0005;

    private EncoderSim encoder;

    // True state (output shaft, radians from horizontal)
    private double motorPositionRad;
    private double motorVelocityRadPerSec;
    private double loadPositionRad;
    private double loadVelocityRadPerSec;
    private double disturbanceVoltage = 0.0;

    /**
     * Construct a backlash arm plant simulation with default contact parameters.
     *
     * @param kS                   motor-side static friction voltage
     * @param kG                   gravity voltage at horizontal (acts on the load)
     * @param kV                   motor-side velocity constant V/(rad/s) at output shaft
     * @param kA                   total acceleration constant V/(rad/s^2) at output shaft; split
     *                             between motor and load by the default load-inertia fraction
     * @param ticksPerRev          motor encoder ticks per motor revolution (e.g. 28)
     * @param gearRatio            output:input gear ratio (e.g. 100.0)
     * @param encoderZeroOffsetRad angle from horizontal when encoder reads 0
     * @param minAngleRad          minimum angle hard stop (radians from horizontal)
     * @param maxAngleRad          maximum angle hard stop (radians from horizontal)
     * @param initialAngleRad      initial arm (load) angle (radians from horizontal)
     * @param backlashRad          total gearbox backlash at the output shaft (e.g. toRadians(5))
     */
    public BacklashArmMotorSim(double kS, double kG, double kV, double kA,
                               int ticksPerRev, double gearRatio,
                               double encoderZeroOffsetRad,
                               double minAngleRad, double maxAngleRad,
                               double initialAngleRad, double backlashRad) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;

        // Reflected rotor inertia dominates a high-ratio gearbox, so the motor side carries most of
        // kA. The default load fraction is a stable, qualitatively reasonable guess (pending sysid).
        double defaultLoadFraction = 0.2;
        this.kAMotor = kA * (1.0 - defaultLoadFraction);
        this.kALoad = kA * defaultLoadFraction;

        this.ticksPerRad = (ticksPerRev * gearRatio) / (2.0 * Math.PI);
        this.encoderZeroOffsetRad = encoderZeroOffsetRad;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.halfBacklashRad = Math.max(0.0, backlashRad) / 2.0;

        // Default contact: stiff enough that the static gravity penetration is a small fraction of a
        // degree, lightly under-damped so re-engagement has a little bounce. Tunable via setters.
        this.contactStiffness = 500.0;
        this.contactDamping = 2.0;

        this.loadPositionRad = clampToStops(initialAngleRad);
        this.loadVelocityRadPerSec = 0.0;

        // Seed the motor side resting on the gravity-loaded contact face so there is no start-up
        // transient: at rest the contact must supply τ = kG·cos(θ_L) to hold the arm.
        double holdTorque = kG * Math.cos(loadPositionRad);
        double penetration = holdTorque / contactStiffness;
        this.motorPositionRad = loadPositionRad + penetration
                + Math.signum(penetration) * halfBacklashRad;
        this.motorVelocityRadPerSec = 0.0;

        seedEncoder();
    }

    private void seedEncoder() {
        double initialTicksDouble = (motorPositionRad - encoderZeroOffsetRad) * ticksPerRad;
        this.encoder = new EncoderSim();
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
            // Feed the encoder the motor-side velocity (the shaft the encoder is on), in TPS.
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

    /** Sets the contact spring stiffness in volts per radian of tooth penetration. */
    public void setContactStiffness(double vPerRad) {
        this.contactStiffness = vPerRad;
    }

    /** Sets the contact damping in volts per (rad/s) of relative velocity while engaged. */
    public void setContactDamping(double vPerRadPerSec) {
        this.contactDamping = vPerRadPerSec;
    }

    /** Sets the load-side bearing friction: viscous (V per rad/s) and static (V). */
    public void setLoadFriction(double viscous, double staticVolts) {
        this.kVLoad = viscous;
        this.kSLoad = staticVolts;
    }

    /** Overrides the largest internal RK4 sub-step (seconds). Smaller = stiffer contact stays stable. */
    public void setMaxInternalDt(double sec) {
        this.maxInternalDtSec = sec;
    }

    /** Replaces the encoder model (e.g. {@link EncoderSim#expansionHub(long)} for RS485 jitter). */
    public void setEncoder(EncoderSim encoder) {
        this.encoder = encoder;
        double initialTicksDouble = (motorPositionRad - encoderZeroOffsetRad) * ticksPerRad;
        this.encoder.setState(0, initialTicksDouble);
        for (int i = 0; i < 6; i++) {
            this.encoder.advance(0.010, 0.0);
        }
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

    /** Returns the true arm (load) position in radians from horizontal. */
    public double getTruePositionRad() {
        return loadPositionRad;
    }

    /** Returns the true arm (load) angular velocity in rad/s. */
    public double getTrueVelocityRadPerSec() {
        return loadVelocityRadPerSec;
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
        return Math.abs(motorPositionRad - loadPositionRad) > halfBacklashRad;
    }

    /** Total backlash at the output shaft, in radians. */
    public double getBacklashRad() {
        return 2.0 * halfBacklashRad;
    }

    // ── dynamics ─────────────────────────────────────────────────────────────────
    //
    //   state = [θ_m, ω_m, θ_L, ω_L]
    //   kA_m · dω_m/dt = u − kV·ω_m − kS·sign(ω_m) − τ_contact
    //   kA_L · dω_L/dt = τ_contact − kG·cos(θ_L) − kV_L·ω_L − kS_L·sign(ω_L)

    /** Contact (gear-tooth) torque on the load, in volts, from the relative state. */
    private double contactTorque(double thetaM, double omegaM, double thetaL, double omegaL) {
        double delta = thetaM - thetaL;
        if (delta > halfBacklashRad) {
            return contactStiffness * (delta - halfBacklashRad)
                    + contactDamping * (omegaM - omegaL);
        }
        if (delta < -halfBacklashRad) {
            return contactStiffness * (delta + halfBacklashRad)
                    + contactDamping * (omegaM - omegaL);
        }
        return 0.0;  // teeth separated — no coupling
    }

    private double dOmegaMotor(double thetaM, double omegaM, double thetaL, double omegaL, double u) {
        double tau = contactTorque(thetaM, omegaM, thetaL, omegaL);
        return (u - kV * omegaM - kS * Math.signum(omegaM) - tau) / kAMotor;
    }

    private double dOmegaLoad(double thetaM, double omegaM, double thetaL, double omegaL) {
        double tau = contactTorque(thetaM, omegaM, thetaL, omegaL);
        double gravity = kG * Math.cos(thetaL);
        return (tau - gravity - kVLoad * omegaL - kSLoad * Math.signum(omegaL)) / kALoad;
    }

    private void rk4Step(double u, double dt) {
        double tm = motorPositionRad, wm = motorVelocityRadPerSec;
        double tl = loadPositionRad, wl = loadVelocityRadPerSec;

        // k1
        double k1_tm = wm;
        double k1_wm = dOmegaMotor(tm, wm, tl, wl, u);
        double k1_tl = wl;
        double k1_wl = dOmegaLoad(tm, wm, tl, wl);

        // k2
        double tm2 = tm + 0.5 * dt * k1_tm, wm2 = wm + 0.5 * dt * k1_wm;
        double tl2 = tl + 0.5 * dt * k1_tl, wl2 = wl + 0.5 * dt * k1_wl;
        double k2_tm = wm2;
        double k2_wm = dOmegaMotor(tm2, wm2, tl2, wl2, u);
        double k2_tl = wl2;
        double k2_wl = dOmegaLoad(tm2, wm2, tl2, wl2);

        // k3
        double tm3 = tm + 0.5 * dt * k2_tm, wm3 = wm + 0.5 * dt * k2_wm;
        double tl3 = tl + 0.5 * dt * k2_tl, wl3 = wl + 0.5 * dt * k2_wl;
        double k3_tm = wm3;
        double k3_wm = dOmegaMotor(tm3, wm3, tl3, wl3, u);
        double k3_tl = wl3;
        double k3_wl = dOmegaLoad(tm3, wm3, tl3, wl3);

        // k4
        double tm4 = tm + dt * k3_tm, wm4 = wm + dt * k3_wm;
        double tl4 = tl + dt * k3_tl, wl4 = wl + dt * k3_wl;
        double k4_tm = wm4;
        double k4_wm = dOmegaMotor(tm4, wm4, tl4, wl4, u);
        double k4_tl = wl4;
        double k4_wl = dOmegaLoad(tm4, wm4, tl4, wl4);

        motorPositionRad += (dt / 6.0) * (k1_tm + 2 * k2_tm + 2 * k3_tm + k4_tm);
        motorVelocityRadPerSec += (dt / 6.0) * (k1_wm + 2 * k2_wm + 2 * k3_wm + k4_wm);
        loadPositionRad += (dt / 6.0) * (k1_tl + 2 * k2_tl + 2 * k3_tl + k4_tl);
        loadVelocityRadPerSec += (dt / 6.0) * (k1_wl + 2 * k2_wl + 2 * k3_wl + k4_wl);
    }

    private double clampToStops(double angle) {
        if (angle < minAngleRad) return minAngleRad;
        if (angle > maxAngleRad) return maxAngleRad;
        return angle;
    }

    private void enforceHardStops() {
        if (loadPositionRad <= minAngleRad) {
            loadPositionRad = minAngleRad;
            if (loadVelocityRadPerSec < 0) loadVelocityRadPerSec = 0;
        }
        if (loadPositionRad >= maxAngleRad) {
            loadPositionRad = maxAngleRad;
            if (loadVelocityRadPerSec > 0) loadVelocityRadPerSec = 0;
        }
    }
}
