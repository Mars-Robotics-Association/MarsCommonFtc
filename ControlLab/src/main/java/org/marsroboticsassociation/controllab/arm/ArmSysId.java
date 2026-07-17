package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.sim.ArmMotorSim;
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim;
import org.marsroboticsassociation.controllib.sim.EncoderSim;
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

/**
 * Simulated system-identification for the arm. It drives a fresh plant built from the config, logs
 * the motor-side encoder, and recovers the arm feedforward model via
 * {@link org.marsroboticsassociation.controllib.mechanism.ArmSysId}.
 *
 * <pre>
 *   V = kS*sign(w) + kV*w + kA*(dw/dt) + kCos*cos(theta) + kSin*sin(theta)
 * </pre>
 *
 * <p><b>Two-stage battery.</b> Friction and gravity ({@code kS, kV, kCos, kSin}) come from
 * constant-velocity holds and inertia ({@code kA}) from constant-power runs, combined by
 * {@link org.marsroboticsassociation.controllib.mechanism.ArmSysId#solveTwoStage}. The holds are
 * what make {@code kS} survive a lashy/flexy drivetrain: at a steady speed the acceleration is ~ 0,
 * so the flex sits at its quasi-static deflection and the direction-flipping flex/lash bias that a
 * hard-accelerating run pumps into {@code kS} is starved. (A single-stage constant-power sysid
 * through the lash + flex instead inflates {@code kS}, and that over-estimate is an anti-braking
 * feedforward term that reintroduces arrival overshoot.) The holds here are driven by a PI velocity
 * loop over the sim; on a real robot the {@code ArmSysIdTuning} OpMode drives the same holds and
 * feeds {@link org.marsroboticsassociation.controllib.mechanism.ArmSysId#accumulateHold} the same
 * angle + voltage log.
 *
 * <p><b>Plant choice.</b> {@link #characterize(ArmPlantConfig)} uses a rigid {@link ArmMotorSim}.
 * {@link #characterize(ArmPlantConfig, ArmEngine.PlantKind)} selects the two-inertia
 * {@link BacklashArmMotorSim} or the three-inertia {@link FlexArmMotorSim} and logs the
 * <em>motor-side</em> encoder — the realistic case for a robot with an unavoidable lashy (and
 * flexy) drivetrain.
 */
public final class ArmSysId {

    private ArmSysId() {}

    /**
     * Identified feedforward coefficients plus a fit-quality score.
     *
     * <p>Delegates field layout to {@link org.marsroboticsassociation.controllib.mechanism.ArmSysId.Result}
     * so ControlLab and the on-robot OpMode share one type family; this thin wrapper keeps existing
     * ControlLab call sites ({@code ArmSysId.Result}) compiling.
     */
    public static final class Result {
        public final double kS, kV, kA, kCos, kSin, rSquared;
        public final int samples;
        /** Hold-stage kV (quasi-static, flex-immune); NaN when the holds could not pin it. */
        public final double kVHold;
        /** Run-stage kV cross-check from the moving fit. */
        public final double kVRun;
        /** Backlash half-angle (rad) implied by the direction-split hold fit; NaN if unavailable. */
        public final double halfLashRad;

        Result(org.marsroboticsassociation.controllib.mechanism.ArmSysId.Result r) {
            this.kS = r.kS;
            this.kV = r.kV;
            this.kA = r.kA;
            this.kCos = r.kCos;
            this.kSin = r.kSin;
            this.rSquared = r.rSquared;
            this.samples = r.samples;
            this.kVHold = r.kVHold;
            this.kVRun = r.kVRun;
            this.halfLashRad = r.halfLashRad;
        }

        /** |kVHold − kVRun| / kV; beyond ~10% flags flex/lash contamination of the moving runs. */
        public double kVDisagreement() {
            return Math.abs(kVHold - kVRun) / Math.max(Math.abs(kV), 1e-9);
        }
    }

    private static final double DT = 0.01;           // 100 Hz logging
    private static final int RUN_STEPS = 90;         // 0.9 s per run
    private static final double HUB = ArmEngine.HUB_VOLTAGE;

    // Constant-velocity holds (identify kS/kV/kCos/kSin quasi-statically; see velocitySweep).
    private static final double[] SWEEP_SPEEDS = {0.75, 1.5, 2.25, 3.0}; // rad/s, both directions
    private static final int SWEEP_MAX_STEPS = 500;  // 5 s cap per hold
    private static final double SWEEP_KP = 8.0;      // velocity-hold P gain, V per rad/s: saturates
    // warmup so the hold reaches speed early and sweeps a wide angle band (heavy inertia otherwise
    // burns the whole travel getting up to the higher speeds); steady samples are picked by
    // accumulateHold's acceleration gate, so a bit of warmup in the log is harmless
    private static final double SWEEP_KI = 20.0;     // velocity-hold I gain, V per rad

    /** Run the characterization against a rigid plant built from {@code cfg}. */
    public static Result characterize(ArmPlantConfig cfg) {
        return characterize(cfg, false);
    }

    /**
     * Run the characterization against the plant built from {@code cfg}, optionally through the
     * two-inertia backlash plant (i.e. logging the motor-side encoder, exactly what a real robot with
     * an unavoidable lashy gearbox exposes) rather than a rigid plant.
     *
     * @param throughBacklash true to identify through the backlash plant's motor encoder
     */
    public static Result characterize(ArmPlantConfig cfg, boolean throughBacklash) {
        return characterize(cfg,
                throughBacklash ? ArmEngine.PlantKind.BACKLASH : ArmEngine.PlantKind.RIGID);
    }

    /**
     * Run the characterization against the given plant kind built from {@code cfg}, logging the
     * motor-side encoder (the only thing a real robot exposes).
     *
     * @param kind which plant simulation to identify through
     */
    public static Result characterize(ArmPlantConfig cfg, ArmEngine.PlantKind kind) {
        double ticksPerRad = cfg.ticksPerRad();
        double lo = cfg.minAngleRad, hi = cfg.maxAngleRad, span = hi - lo;
        double bottom = lo + 0.05 * span, top = hi - 0.05 * span, mid = lo + 0.55 * span;
        // Flex-aware fit windows: through the flex plant the structural period is known from the
        // config, so run intervals span whole periods and the ring cancels out of the kA fit.
        var fit = new org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams();
        if (kind == ArmEngine.PlantKind.FLEX && cfg.flexHz > 0) {
            fit.flexPeriodSec = 1.0 / cfg.flexHz;
        }
        // Gravity peaks at horizontals (cos θ = ±1). On an over-the-top workspace those are 0 and
        // +π, not the span midpoint (often upright). Seed extra runs from every horizontal that
        // sits clear of the hard stops so kS/kCos stay identifiable.
        double margin = 0.08 * span;
        double[] gravityStarts = gravityRichStarts(lo, hi, margin, mid);

        // Stage 2 data: constant-power runs (rich angle sweep at near-terminal velocity) in both
        // directions, plus hard steps from rest whose onset makes Δw large (exciting kA).
        List<double[]> movingRows = new ArrayList<>();
        List<Double> movingRhs = new ArrayList<>();
        for (double p : new double[]{0.45, 0.60, 0.80, 0.95}) {
            runRun(cfg, ticksPerRad, bottom, p, kind, fit, movingRows, movingRhs);
        }
        for (double p : new double[]{-0.45, -0.60, -0.80, -0.95}) {
            runRun(cfg, ticksPerRad, top, p, kind, fit, movingRows, movingRhs);
        }
        for (double start : gravityStarts) {
            runRun(cfg, ticksPerRad, start, 0.70, kind, fit, movingRows, movingRhs);
            runRun(cfg, ticksPerRad, start, -0.70, kind, fit, movingRows, movingRhs);
        }

        // Stage 1 data: constant-velocity holds at several speeds, both directions.
        List<double[]> holdRows = new ArrayList<>();
        List<Double> holdRhs = new ArrayList<>();
        for (double speed : SWEEP_SPEEDS) {
            velocityHold(cfg, ticksPerRad, kind, +speed, fit, holdRows, holdRhs);
            velocityHold(cfg, ticksPerRad, kind, -speed, fit, holdRows, holdRhs);
        }

        return new Result(
                org.marsroboticsassociation.controllib.mechanism.ArmSysId.solveTwoStage(
                        fit, holdRows, holdRhs, movingRows, movingRhs));
    }

    /** One constant-power run: log position, then stack integrated-dynamics equations. */
    private static void runRun(ArmPlantConfig cfg, double ticksPerRad, double startRad, double power,
                               ArmEngine.PlantKind kind,
                               org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams fit,
                               List<double[]> rows, List<Double> rhs) {
        int n = RUN_STEPS;
        double[] theta = new double[n];
        // Clean encoders throughout: isolate the plant (lash/flex) effect from read-timing jitter.
        SimHandle sim = newSim(cfg, ticksPerRad, kind, startRad);
        for (int i = 0; i < n; i++) {
            sim.step(DT, power);
            theta[i] = sim.posRad();
        }
        double v = power * HUB;
        org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateRun(
                theta, v, DT, cfg.minAngleRad, cfg.maxAngleRad, fit, rows, rhs);
    }

    /**
     * One constant-velocity hold: a PI loop on velocity error sweeps the arm across the travel at a
     * held {@code targetVel}, logging motor-encoder angle and the applied voltage that sustains it.
     * The log is handed to {@link org.marsroboticsassociation.controllib.mechanism.ArmSysId#accumulateHold},
     * which keeps only the steady (α ≈ 0) samples. Mirrors what the on-robot OpMode does with a real
     * arm: same angle + voltage log, same accumulate call.
     */
    private static void velocityHold(ArmPlantConfig cfg, double ticksPerRad, ArmEngine.PlantKind kind,
                                     double targetVel,
                                     org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams fit,
                                     List<double[]> rows, List<Double> rhs) {
        double start = targetVel > 0 ? cfg.minAngleRad + fit.stopMarginRad
                : cfg.maxAngleRad - fit.stopMarginRad;
        SimHandle sim = newSim(cfg, ticksPerRad, kind, start);
        List<Double> theta = new ArrayList<>();
        List<Double> volts = new ArrayList<>();
        double integral = 0;
        for (int i = 0; i < SWEEP_MAX_STEPS; i++) {
            double err = targetVel - sim.velRad();
            integral += err * DT;
            double power = Math.max(-1.0, Math.min(1.0, (SWEEP_KP * err + SWEEP_KI * integral) / HUB));
            sim.step(DT, power);
            double pos = sim.posRad();
            // Stop once the sweep reaches the far hard-stop margin (accumulateHold discards those
            // samples anyway); until then log every step.
            if (targetVel > 0 ? pos > cfg.maxAngleRad - fit.stopMarginRad
                    : pos < cfg.minAngleRad + fit.stopMarginRad) {
                if (!theta.isEmpty()) break;
            }
            theta.add(pos);
            volts.add(power * HUB);
        }
        double[] times = new double[theta.size()];
        for (int i = 0; i < times.length; i++) times[i] = i * DT; // sim runs at a fixed step
        org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateHold(
                toArray(theta), toArray(volts), times,
                cfg.minAngleRad, cfg.maxAngleRad, fit, rows, rhs);
    }

    private static double[] toArray(List<Double> list) {
        double[] a = new double[list.size()];
        for (int i = 0; i < a.length; i++) a[i] = list.get(i);
        return a;
    }

    /** A uniform driver over the three plant sims, in the mechanism's motor-encoder frame. */
    private interface SimHandle {
        void step(double dt, double power);
        double posRad();
        double velRad();
    }

    /** A sim's {@code step(dt, power, hubVoltage)}. */
    private interface Stepper {
        void step(double dt, double power, double hubVoltage);
    }

    /** Build a {@link SimHandle} for the plant kind, seeded at {@code startRad} with a clean encoder. */
    private static SimHandle newSim(ArmPlantConfig cfg, double ticksPerRad,
                                    ArmEngine.PlantKind kind, double startRad) {
        switch (kind) {
            case FLEX: {
                FlexArmMotorSim sim = new FlexArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                        cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                        cfg.minAngleRad, cfg.maxAngleRad, startRad, cfg.backlashRad,
                        cfg.flexHz, cfg.flexZeta);
                sim.setEncoder(new EncoderSim());
                return handle(sim::step, sim::getPositionTicks, sim::getVelocityTps, ticksPerRad,
                        cfg.encoderZeroOffsetRad);
            }
            case BACKLASH: {
                BacklashArmMotorSim sim = new BacklashArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                        cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                        cfg.minAngleRad, cfg.maxAngleRad, startRad, cfg.backlashRad);
                sim.setEncoder(new EncoderSim());
                return handle(sim::step, sim::getPositionTicks, sim::getVelocityTps, ticksPerRad,
                        cfg.encoderZeroOffsetRad);
            }
            case RIGID:
            default: {
                ArmMotorSim sim = new ArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                        cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                        cfg.minAngleRad, cfg.maxAngleRad, startRad);
                sim.setEncoder(new EncoderSim());
                return handle(sim::step, sim::getPositionTicks, sim::getVelocityTps, ticksPerRad,
                        cfg.encoderZeroOffsetRad);
            }
        }
    }

    /** Adapt a sim's step/read methods to a {@link SimHandle} in radians. */
    private static SimHandle handle(Stepper stepper, IntSupplier ticks, DoubleSupplier velTps,
                                    double ticksPerRad, double offsetRad) {
        return new SimHandle() {
            @Override public void step(double dt, double power) { stepper.step(dt, power, HUB); }
            @Override public double posRad() { return ticks.getAsInt() / ticksPerRad + offsetRad; }
            @Override public double velRad() { return velTps.getAsDouble() / ticksPerRad; }
        };
    }

    /**
     * Start angles that excite gravity for OLS: horizontals in range (cos θ = ±1), plus the span
     * midpoint as a fallback when neither horizontal is available.
     */
    private static double[] gravityRichStarts(double lo, double hi, double margin, double mid) {
        List<Double> starts = new ArrayList<>();
        for (double h : new double[]{0.0, Math.PI, -Math.PI}) {
            if (h > lo + margin && h < hi - margin) {
                starts.add(h);
            }
        }
        if (starts.isEmpty()) {
            starts.add(mid);
        }
        double[] out = new double[starts.size()];
        for (int i = 0; i < starts.size(); i++) out[i] = starts.get(i);
        return out;
    }
}
