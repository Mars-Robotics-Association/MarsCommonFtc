package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.control.ArmController;
import org.marsroboticsassociation.controllib.control.VerticalArmController;

import java.util.Random;

/**
 * Simulation engine for the Arm tab: owns the active {@link ArmPlant} and {@link ArmControlAdapter},
 * a virtual clock, and the {@link ArmMetrics}. Modeled on {@code FlywheelEngine} — each {@link #tick}
 * advances the clock, runs the controller (which reads the plant and stashes power), then steps the
 * plant with that power.
 *
 * <p>The backlash plant is selected by default even though none of the controllers model backlash;
 * the rigid/backlash toggle hot-swaps the plant mid-run (seeding the new plant from the current load
 * pose) so the degradation is visible instantly. Structural plant edits and plant swaps also rebuild
 * the controller adapter so the estimator/profile cannot keep flying with a plant that was just
 * reseeded at rest.
 *
 * <p><b>Lineage A params lock.</b> {@link ArmController#PARAMS} and {@link
 * VerticalArmController#PARAMS} are process-wide static bags. All Lineage A configure/build/tick
 * work holds {@link #LINEAGE_A_PARAMS_LOCK} and re-publishes this engine's params before each step so
 * concurrent engines (or tests) do not silently share a half-written bag. Prefer a fixed {@linkplain
 * #ArmEngine(ArmControllerType, long) RNG seed} in tests for reproducibility under dt jitter.
 */
public class ArmEngine {

    public static final double HUB_VOLTAGE = 12.0;
    private static final double NOMINAL_DT = 0.016; // 16 ms loop, like FlywheelEngine
    private static final double DT_JITTER = 0.008;  // +/- 4 ms

    /**
     * Guards all reads/writes of the Lineage A static {@code PARAMS} bags used by {@link
     * ArmController} and {@link VerticalArmController}.
     */
    static final Object LINEAGE_A_PARAMS_LOCK = new Object();

    private final Random random;

    private final ArmPlantConfig cfg = new ArmPlantConfig();
    private boolean backlashEnabled = true;

    private ArmPlant plant;
    private ArmControllerType type;
    private ArmControlAdapter adapter;

    private long elapsedNanos = 0;
    private double elapsedSec = 0;
    private double targetRad;

    private final ArmMetrics metrics = new ArmMetrics();

    // --- Lineage A controller gains (editable) ---
    // Feedforward defaults match the (heavy) plant so the controllers know the model; the visible
    // ranking then comes from feedback structure, not model error.
    private double ffKs = 0.3, ffKg = 3.5, ffKv = 1.2, ffKa = 0.35;
    private double kP = 15.0, kD = 1.0;                 // ARM_PD feedback
    private double lqrQPos = 0.5, lqrQVel = 5.0, lqrR = 12.0; // ARM_LQR weights

    // --- Lineage B (mechanism) gains ---
    private final MechanismArmAdapter.Gains mechGains = new MechanismArmAdapter.Gains();

    /** Construct with a non-deterministic dt-jitter stream (interactive ControlLab use). */
    public ArmEngine(ArmControllerType type) {
        this(type, System.nanoTime());
    }

    /**
     * Construct with a fixed RNG seed so loop-dt jitter is reproducible (smoke tests, bisects).
     *
     * @param seed seed for the per-tick dt jitter generator
     */
    public ArmEngine(ArmControllerType type, long seed) {
        this.random = new Random(seed);
        this.type = type;
        this.targetRad = cfg.maxAngleRad; // park at the back hard stop (over-the-top end)
        this.plant = backlashEnabled
                ? new BacklashArmPlant(cfg, cfg.maxAngleRad)
                : new RigidArmPlant(cfg, cfg.maxAngleRad);
        buildAdapter();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Simulation loop
    // ─────────────────────────────────────────────────────────────────────────────

    public void tick() {
        // Always apply realistic loop timing (16 ms ± 4 ms). The rate limiter is hardened against
        // dt jitter, so the profile no longer needs an exactly-periodic mode for clean plots.
        double dt = NOMINAL_DT + (random.nextDouble() - 0.5) * DT_JITTER;
        elapsedNanos += (long) (dt * 1e9);

        if (type == ArmControllerType.MECHANISM_PIDF) {
            stepControlAndPlant(dt);
        } else {
            // Re-publish this engine's params and hold the lock for the whole Lineage A step so
            // another engine cannot overwrite PARAMS mid-update.
            synchronized (LINEAGE_A_PARAMS_LOCK) {
                configureParams();
                stepControlAndPlant(dt);
            }
        }
    }

    private void stepControlAndPlant(double dt) {
        adapter.step(dt, HUB_VOLTAGE);
        plant.step(dt, adapter.commandedPower(), HUB_VOLTAGE);
        elapsedSec += dt;

        metrics.update(elapsedSec, plant.getTruePositionRad(), plant.getTrueVelocityRadPerSec(),
                plant.getMotorPositionRad(), plant.isEngaged());
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Controller / plant selection
    // ─────────────────────────────────────────────────────────────────────────────

    private void configureParams() {
        if (type == ArmControllerType.ARM_PD) {
            ArmController.PARAMS = armParams();
        } else if (type == ArmControllerType.ARM_LQR) {
            VerticalArmController.PARAMS = verticalParams();
        }
        // MECHANISM carries its gains in mechGains; nothing static to set.
    }

    private ArmController.Params armParams() {
        ArmController.Params p = new ArmController.Params();
        p.ticksPerRev = cfg.ticksPerRev;
        p.gearRatio = cfg.gearRatio;
        p.encoderZeroOffsetRad = cfg.encoderZeroOffsetRad;
        p.minAngleRad = cfg.minAngleRad;
        p.maxAngleRad = cfg.maxAngleRad;
        p.ks = ffKs; p.kg = ffKg; p.kv = ffKv; p.ka = ffKa;
        p.kP = kP;   p.kD = kD;
        p.maxVelRad = 3.0; p.maxAccelRad = 6.0; p.maxDecelRad = 8.0; p.maxJerkRad = 30.0;
        return p;
    }

    private VerticalArmController.Params verticalParams() {
        VerticalArmController.Params p = new VerticalArmController.Params();
        p.ticksPerRev = cfg.ticksPerRev;
        p.gearRatio = cfg.gearRatio;
        p.encoderZeroOffsetRad = cfg.encoderZeroOffsetRad;
        p.minAngleRad = cfg.minAngleRad;
        p.maxAngleRad = cfg.maxAngleRad;
        p.ks = ffKs; p.kg = ffKg; p.kv = ffKv; p.ka = ffKa;
        p.qPosition = lqrQPos; p.qVelocity = lqrQVel; p.rVoltage = lqrR;
        p.maxVelRad = 3.0; p.maxAccelRad = 6.0; p.maxDecelRad = 8.0; p.maxJerkRad = 30.0;
        return p;
    }

    private void buildAdapter() {
        if (type == ArmControllerType.MECHANISM_PIDF) {
            MechanismArmAdapter m = new MechanismArmAdapter(mechGains, plant,
                    cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad);
            m.setTargetRad(targetRad);
            adapter = m;
            return;
        }
        // Lineage A constructors and live kP/kD reads both touch the static PARAMS bags.
        synchronized (LINEAGE_A_PARAMS_LOCK) {
            configureParams();
            LineageAArmAdapter a =
                    new LineageAArmAdapter(type, plant, () -> elapsedNanos, HUB_VOLTAGE);
            a.deferTarget(targetRad);
            adapter = a;
        }
    }

    /**
     * Rebuild the controller adapter from the current plant pose and re-apply the active target.
     * Used after any structural plant change that zeros plant velocity or swaps the sim instance, so
     * the estimator/profile cannot keep a stale flying state against a plant that just reseeded at
     * rest.
     */
    private void reseedAdapterFromPlant() {
        buildAdapter();
    }

    public void setControllerType(ArmControllerType newType) {
        if (newType == type) return;
        this.type = newType;
        buildAdapter(); // reseeds from current plant pose; target re-applied without a jump
    }

    public ArmControllerType getControllerType() { return type; }

    /** Hot-swap rigid <-> backlash, seeding the new plant from the current load pose (no jump). */
    public void setBacklashEnabled(boolean enabled) {
        if (enabled == backlashEnabled) return;
        backlashEnabled = enabled;
        // The sims seed at rest, so the pose is preserved but velocity resets to zero on swap.
        double loadRad = plant.getTruePositionRad();
        plant = enabled ? new BacklashArmPlant(cfg, loadRad) : new RigidArmPlant(cfg, loadRad);
        reseedAdapterFromPlant();
    }

    public boolean isBacklashEnabled() { return backlashEnabled; }

    // ─────────────────────────────────────────────────────────────────────────────
    // Targets
    // ─────────────────────────────────────────────────────────────────────────────

    public void setTargetRad(double rad) {
        targetRad = clampAngle(rad);
        adapter.setTargetRad(targetRad);
        metrics.onTargetChanged(targetRad, plant.getTruePositionRad(), elapsedSec);
    }

    public void setTargetDeg(double deg) {
        setTargetRad(Math.toRadians(deg));
    }

    private double clampAngle(double rad) {
        return Math.max(cfg.minAngleRad, Math.min(cfg.maxAngleRad, rad));
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Live controller-gain edits
    // ─────────────────────────────────────────────────────────────────────────────

    public void setFeedforwardGains(double ks, double kg, double kv, double ka) {
        ffKs = ks; ffKg = kg; ffKv = kv; ffKa = ka;
        if (type == ArmControllerType.ARM_PD || type == ArmControllerType.ARM_LQR) {
            synchronized (LINEAGE_A_PARAMS_LOCK) {
                configureParams();
                ((LineageAArmAdapter) adapter).rebuild(); // FF/kalman baked at construction
            }
        }
    }

    public void setPdGains(double kP, double kD) {
        this.kP = kP; this.kD = kD;
        if (type == ArmControllerType.ARM_PD) {
            // ArmController reads PARAMS.kP/kD live, so mutate in place (no rebuild).
            synchronized (LINEAGE_A_PARAMS_LOCK) {
                ArmController.PARAMS.kP = kP;
                ArmController.PARAMS.kD = kD;
            }
        }
    }

    public void setLqrWeights(double qPos, double qVel, double r) {
        this.lqrQPos = qPos; this.lqrQVel = qVel; this.lqrR = r;
        if (type == ArmControllerType.ARM_LQR) {
            synchronized (LINEAGE_A_PARAMS_LOCK) {
                configureParams();
                ((LineageAArmAdapter) adapter).rebuild(); // LQR gains baked at construction
            }
        }
    }

    public void setMechanismGains(double kP, double kI, double kD,
                                  double kS, double kV, double kA,
                                  double kCos, double kSin,
                                  double maxVel, double maxAccel, double maxJerk) {
        mechGains.kP = kP; mechGains.kI = kI; mechGains.kD = kD;
        mechGains.kS = kS; mechGains.kV = kV; mechGains.kA = kA;
        mechGains.kCos = kCos; mechGains.kSin = kSin;
        mechGains.maxVel = maxVel; mechGains.maxAccel = maxAccel; mechGains.maxJerk = maxJerk;
        if (type == ArmControllerType.MECHANISM_PIDF) {
            ((MechanismArmAdapter) adapter).rebuild(); // controller + EKF reseeded from pose
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Live plant-param edits
    // ─────────────────────────────────────────────────────────────────────────────

    /** Structural plant dynamics: rebuild the plant at the current pose. */
    public void setPlantDynamics(double kS, double kG, double kV, double kA) {
        cfg.kS = kS; cfg.kG = kG; cfg.kV = kV; cfg.kA = kA;
        reseedPlantInPlace();
    }

    /** Structural: rebuild the plant at the current pose with the new backlash. */
    public void setBacklashRad(double rad) {
        cfg.backlashRad = rad;
        reseedPlantInPlace();
    }

    public void setContact(double stiffness, double damping) {
        cfg.contactStiffness = stiffness;
        cfg.contactDamping = damping;
        plant.applyLiveParams();
    }

    public void setLoadFriction(double viscous, double staticVolts) {
        cfg.loadViscousFriction = viscous;
        cfg.loadStaticFriction = staticVolts;
        plant.applyLiveParams();
    }

    public void setDisturbanceVoltage(double v) {
        cfg.disturbanceVoltage = v;
        plant.applyLiveParams();
    }

    /** Structural: swap the encoder read-timing model (rebuilds the plant at the current pose). */
    public void setEncoderKind(ArmPlantConfig.EncoderKind kind) {
        cfg.encoderKind = kind;
        reseedPlantInPlace();
    }

    private void reseedPlantInPlace() {
        double loadRad = plant.getTruePositionRad();
        double loadVel = plant.getTrueVelocityRadPerSec();
        plant.seedFrom(loadRad, loadVel);
        // Sims always seed at rest; rebuild the controller so the profile/EKF match.
        reseedAdapterFromPlant();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // System identification
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Run the simulated sysid against the currently-selected plant. With backlash enabled it
     * identifies through the motor-side encoder of the two-inertia plant — the realistic case for a
     * real robot with an unavoidable lashy gearbox. The routine's steady one-directional runs keep
     * the gear teeth engaged, so the motor encoder sees the full coupled inertia and gravity.
     */
    public ArmSysId.Result runSysId() {
        return ArmSysId.characterize(cfg, backlashEnabled);
    }

    /**
     * Push identified feedforward gains into both controller models (Lineage-A feedforward and the
     * mechanism model) and rebuild the active controller so they take effect immediately.
     */
    public void applyIdentifiedGains(ArmSysId.Result r) {
        ffKs = r.kS; ffKg = r.kCos; ffKv = r.kV; ffKa = r.kA;
        mechGains.kS = r.kS; mechGains.kV = r.kV; mechGains.kA = r.kA;
        mechGains.kCos = r.kCos; mechGains.kSin = r.kSin;
        switch (type) {
            case ARM_PD:
            case ARM_LQR:
                synchronized (LINEAGE_A_PARAMS_LOCK) {
                    configureParams();
                    ((LineageAArmAdapter) adapter).rebuild();
                }
                break;
            case MECHANISM_PIDF:
                ((MechanismArmAdapter) adapter).rebuild();
                break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Reset
    // ─────────────────────────────────────────────────────────────────────────────

    public void reset() {
        elapsedNanos = 0;
        elapsedSec = 0;
        targetRad = cfg.maxAngleRad;
        plant = backlashEnabled
                ? new BacklashArmPlant(cfg, cfg.maxAngleRad)
                : new RigidArmPlant(cfg, cfg.maxAngleRad);
        buildAdapter();
        metrics.reset();
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Accessors for the canvas / chart / metrics
    // ─────────────────────────────────────────────────────────────────────────────

    public double getElapsedSec()          { return elapsedSec; }
    public double getTargetRad()           { return targetRad; }
    public double getTrueLoadRad()         { return plant.getTruePositionRad(); }
    public double getTrueLoadVelRad()      { return plant.getTrueVelocityRadPerSec(); }
    public double getMotorRad()            { return plant.getMotorPositionRad(); }
    public boolean isEngaged()             { return plant.isEngaged(); }
    public double getBacklashRad()         { return plant.getBacklashRad(); }
    public double getMinAngleRad()         { return cfg.minAngleRad; }
    public double getMaxAngleRad()         { return cfg.maxAngleRad; }

    /** Motor-side encoder angle the controller actually sees (ticks -> rad). */
    public double getMeasuredEncoderRad() {
        return plant.getPositionTicks() / cfg.ticksPerRad() + cfg.encoderZeroOffsetRad;
    }

    public double getEstimatedPosRad()     { return adapter.estimatedPosRad(); }
    public double getEstimatedVelRad()     { return adapter.estimatedVelRad(); }
    public double getTrajPosRad()          { return adapter.trajPosRad(); }
    public double getTrajVelRad()          { return adapter.trajVelRad(); }
    public String getModeLabel()           { return adapter.modeLabel(); }

    public ArmMetrics getMetrics()         { return metrics; }

    // Editable-param initial values for the sidebar
    public double getFfKs() { return ffKs; }
    public double getFfKg() { return ffKg; }
    public double getFfKv() { return ffKv; }
    public double getFfKa() { return ffKa; }
    public double getKP()   { return kP; }
    public double getKD()   { return kD; }
    public double getLqrQPos() { return lqrQPos; }
    public double getLqrQVel() { return lqrQVel; }
    public double getLqrR()    { return lqrR; }
    public MechanismArmAdapter.Gains getMechGains() { return mechGains; }

    public double getPlantKs() { return cfg.kS; }
    public double getPlantKg() { return cfg.kG; }
    public double getPlantKv() { return cfg.kV; }
    public double getPlantKa() { return cfg.kA; }
    public double getBacklashRadCfg()   { return cfg.backlashRad; }
    public double getContactStiffness() { return cfg.contactStiffness; }
    public double getContactDamping()   { return cfg.contactDamping; }
    public double getLoadViscous()      { return cfg.loadViscousFriction; }
    public double getLoadStatic()       { return cfg.loadStaticFriction; }
    public double getDisturbanceVoltage() { return cfg.disturbanceVoltage; }
    public ArmPlantConfig.EncoderKind getEncoderKind() { return cfg.encoderKind; }
}
