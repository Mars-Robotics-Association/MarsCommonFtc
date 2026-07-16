package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.sim.ArmMotorSim;
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim;
import org.marsroboticsassociation.controllib.sim.EncoderSim;
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim;

import java.util.ArrayList;
import java.util.List;

/**
 * Simulated system-identification for the arm. It drives a fresh plant built from the config with a
 * battery of known-voltage runs across the travel, logs the motor-side encoder, and recovers the arm
 * feedforward model via {@link org.marsroboticsassociation.controllib.mechanism.ArmSysId}
 * (integrated equation of motion).
 *
 * <pre>
 *   V = kS*sign(w) + kV*w + kA*(dw/dt) + kCos*cos(theta) + kSin*sin(theta)
 * </pre>
 *
 * <p><b>Plant choice.</b> {@link #characterize(ArmPlantConfig)} uses a rigid {@link ArmMotorSim}.
 * {@link #characterize(ArmPlantConfig, ArmEngine.PlantKind)} selects the two-inertia
 * {@link BacklashArmMotorSim} or the three-inertia {@link FlexArmMotorSim} and logs the
 * <em>motor-side</em> encoder — the realistic case for a robot with an unavoidable lashy (and
 * flexy) drivetrain. Steady one-directional runs keep the teeth engaged and the flex spring
 * quasi-static, so the motor encoder still sees the coupled inertia and gravity and the fit holds.
 * Identified {@code kA} is what the model-based controller needs when the true plant is heavier or
 * lashier than the hand-tuned model.
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

        Result(org.marsroboticsassociation.controllib.mechanism.ArmSysId.Result r) {
            this.kS = r.kS;
            this.kV = r.kV;
            this.kA = r.kA;
            this.kCos = r.kCos;
            this.kSin = r.kSin;
            this.rSquared = r.rSquared;
            this.samples = r.samples;
        }
    }

    private static final double DT = 0.01;           // 100 Hz logging
    private static final int RUN_STEPS = 90;         // 0.9 s per run
    private static final double HUB = ArmEngine.HUB_VOLTAGE;
    private static final org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams FIT =
            org.marsroboticsassociation.controllib.mechanism.ArmSysId.DEFAULT_PARAMS;

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
        // Gravity peaks at horizontals (cos θ = ±1). On an over-the-top workspace those are 0 and
        // +π, not the span midpoint (often upright). Seed extra runs from every horizontal that
        // sits clear of the hard stops so kS/kCos stay identifiable.
        double margin = 0.08 * span;
        double[] gravityStarts = gravityRichStarts(lo, hi, margin, mid);

        List<double[]> rows = new ArrayList<>();
        List<Double> rhs = new ArrayList<>();

        // Moderate runs (rich angle sweep at near-terminal velocity) in both directions, plus hard
        // steps from rest whose onset makes Δw large (exciting kA), plus gravity-rich starts.
        for (double p : new double[]{0.45, 0.60, 0.80, 0.95}) {
            runRun(cfg, ticksPerRad, bottom, p, kind, rows, rhs);
        }
        for (double p : new double[]{-0.45, -0.60, -0.80, -0.95}) {
            runRun(cfg, ticksPerRad, top, p, kind, rows, rhs);
        }
        for (double start : gravityStarts) {
            runRun(cfg, ticksPerRad, start, 0.70, kind, rows, rhs);
            runRun(cfg, ticksPerRad, start, -0.70, kind, rows, rhs);
        }

        return new Result(
                org.marsroboticsassociation.controllib.mechanism.ArmSysId.solve(rows, rhs));
    }

    /** One constant-power run: log position, then stack integrated-dynamics equations. */
    private static void runRun(ArmPlantConfig cfg, double ticksPerRad, double startRad, double power,
                               ArmEngine.PlantKind kind, List<double[]> rows, List<Double> rhs) {
        int n = RUN_STEPS;
        double[] theta = new double[n];
        // Clean encoders throughout: isolate the plant (lash/flex) effect from read-timing jitter.
        switch (kind) {
            case FLEX: {
                FlexArmMotorSim sim = new FlexArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                        cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                        cfg.minAngleRad, cfg.maxAngleRad, startRad, cfg.backlashRad,
                        cfg.flexHz, cfg.flexZeta);
                sim.setEncoder(new EncoderSim());
                for (int i = 0; i < n; i++) {
                    sim.step(DT, power, HUB);
                    theta[i] = sim.getPositionTicks() / ticksPerRad + cfg.encoderZeroOffsetRad;
                }
                break;
            }
            case BACKLASH: {
                BacklashArmMotorSim sim = new BacklashArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                        cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                        cfg.minAngleRad, cfg.maxAngleRad, startRad, cfg.backlashRad);
                sim.setEncoder(new EncoderSim());
                for (int i = 0; i < n; i++) {
                    sim.step(DT, power, HUB);
                    theta[i] = sim.getPositionTicks() / ticksPerRad + cfg.encoderZeroOffsetRad;
                }
                break;
            }
            case RIGID:
            default: {
                ArmMotorSim sim = new ArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                        cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                        cfg.minAngleRad, cfg.maxAngleRad, startRad);
                sim.setEncoder(new EncoderSim());
                for (int i = 0; i < n; i++) {
                    sim.step(DT, power, HUB);
                    theta[i] = sim.getPositionTicks() / ticksPerRad + cfg.encoderZeroOffsetRad;
                }
                break;
            }
        }
        double v = power * HUB;
        org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateRun(
                theta, v, DT, cfg.minAngleRad, cfg.maxAngleRad, FIT, rows, rhs);
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
