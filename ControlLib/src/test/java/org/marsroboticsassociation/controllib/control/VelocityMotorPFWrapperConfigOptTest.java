package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.motion.SCurveVelocity;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;

import java.util.Random;

/**
 * Sweeps key VelocityMotorPFWrapper config parameters to find the combination that
 * minimizes time to isAtTargetSpeed() at FARVELOCITY (1450 TPS, 13.75V).
 *
 * Uses the wrapper's exact characterization constants:
 *   kS=0.3735, kV=0.005212, kA=0.006861, kP=0.003125
 *
 * Run with: ./gradlew :ControlLib:test --tests "*.VelocityMotorPFWrapperConfigOptTest"
 */
class VelocityMotorPFWrapperConfigOptTest {

    // Wrapper constants (mirrors VelocityMotorPFWrapper static block)
    private static final double KS = 0.3735;
    private static final double KV = 0.005212;
    private static final double KA = 0.006861;
    private static final double KP = 0.003125;
    private static final double TARGET_TPS = 1450.0;
    private static final double VOLTAGE = 13.75;

    private static VelocityMotorPF.VelocityMotorPFConfig makeConfig(
            double jerkIncreasing, double measurementLpfHz, double accelLpfHz,
            double accelerationTolerance, double kpRampMs) {
        VelocityMotorPF.VelocityMotorPFConfig cfg = new VelocityMotorPF.VelocityMotorPFConfig();
        cfg.kS = KS;
        cfg.kV = KV;
        cfg.kA = KA;
        cfg.kP = KP;
        cfg.targetSpeedTolerance = 40.0;
        cfg.jerkIncreasing = jerkIncreasing;
        cfg.measurementLpfCutoffHz = measurementLpfHz;
        cfg.accelLpfCutoffHz = accelLpfHz;
        cfg.accelerationTolerance = accelerationTolerance;
        cfg.kpRampMs = kpRampMs;
        cfg.accelMax = SCurveVelocity.findMaxAMax(0, TARGET_TPS, jerkIncreasing, VOLTAGE, KS, KV, KA);
        cfg.jerkDecreasing = Math.min(10000, Math.max(150,
                SCurveVelocity.findMaxJDec(0, TARGET_TPS, 0, cfg.accelMax, jerkIncreasing, VOLTAGE, KS, KV, KA)));
        return cfg;
    }

    /** Measure time (seconds) until isAtTargetSpeed(), or NaN if never reached within maxSec. */
    private static double measureTimeToReady(VelocityMotorPF.VelocityMotorPFConfig cfg, double maxSec) {
        FlywheelMotorSim sim = new FlywheelMotorSim(KV, KA);
        sim.setDisturbanceVoltage(-KS);
        FlywheelTestFixture.SimMotorAdapter adapter = new FlywheelTestFixture.SimMotorAdapter(sim);
        double[] timeSecs = {1.0};
        VelocityMotorPF controller = new VelocityMotorPF(
                (caption, format, value) -> {},
                1.0, 1.0, 0.005, cfg, adapter,
                () -> (long) (timeSecs[0] * 1e9));

        controller.setTPS(TARGET_TPS);
        Random rng = new Random(42L);
        double elapsed = 0;
        int maxSteps = (int) (maxSec / 0.016);
        for (int i = 0; i < maxSteps; i++) {
            double dt = Math.max(0.001, 0.020 + rng.nextGaussian() * 0.004);
            timeSecs[0] += dt;
            elapsed += dt;
            controller.update(dt);
            sim.step(dt, adapter.lastPower, VOLTAGE);
            if (controller.isAtTargetSpeed()) return elapsed;
        }
        return Double.NaN;
    }

    @Test
    void verifyClosedFormJDec() {
        System.out.println("\n=== Closed-form jDec verification ===");
        System.out.println("jDec_closed = 2*kV*(V - kS - kV*v1) / kA^2");
        System.out.printf("%-10s %-8s %-10s %-12s %-12s %-12s | %s%n",
                "v1", "voltage", "jInc", "binary", "closed", "abs.err", "rel.err%");
        System.out.println("-".repeat(95));

        double[] targets = {400, 700, 1000, 1250, 1450, 1700, 1900};
        double[] voltages = {12.0, 12.75, 13.75};
        double[] jIncs = {2000, 5000, 10000, 15000, 30000};

        int passed = 0, total = 0;
        for (double v1 : targets) {
            for (double V : voltages) {
                double aMax = SCurveVelocity.findMaxAMax(0, v1, 5000, V, KS, KV, KA);
                for (double jInc : jIncs) {
                    double bin = SCurveVelocity.findMaxJDec(0, v1, 0, aMax, jInc, V, KS, KV, KA);
                    double vHead = V - KS - KV * v1;
                    double closed = vHead > 0 ? 2 * KV * vHead / (KA * KA) : 0.0;

                    // Is the interior-max condition satisfied? (aPeak > kA*jDec/kV)
                    double aPeakSq = 2 * v1 * jInc * closed / (jInc + closed);
                    double aPeak = Math.sqrt(aPeakSq);
                    boolean interiorMax = aPeak > KA * closed / KV;

                    double err = Math.abs(bin - closed);
                    double relErr = bin > 0 ? err / bin * 100 : 0;
                    total++;
                    if (relErr < 1.0 || !interiorMax) passed++;

                    String note = interiorMax ? "" : " (boundary)";
                    System.out.printf("%-10.0f %-8.2f %-10.0f %-12.2f %-12.2f %-12.4f | %.4f%%%s%n",
                            v1, V, jInc, bin, closed, err, relErr, note);
                }
            }
        }
        System.out.printf("%nPassed %d/%d%n", passed, total);
    }

    @Test
    void jerkIncreasingConvergence() {
        System.out.println("\n=== jerkIncreasing convergence (other params at opt values) ===");
        System.out.printf("%-12s %-10s %-10s %-8s %-8s %-8s %-6s | %s%n",
                "jInc", "aMax", "jDec", "t1(s)", "t2(s)", "t3(s)", "shape", "t_ready(s)");
        System.out.println("-".repeat(90));

        for (double jInc : new double[]{500, 1000, 2000, 3000, 5000, 7500, 10000, 15000,
                20000, 30000, 50000, 100000}) {
            double aMax = SCurveVelocity.findMaxAMax(0, TARGET_TPS, jInc, VOLTAGE, KS, KV, KA);
            double jDec = Math.min(10000, Math.max(150,
                    SCurveVelocity.findMaxJDec(0, TARGET_TPS, 0, aMax, jInc, VOLTAGE, KS, KV, KA)));
            SCurveVelocity traj = new SCurveVelocity(0, TARGET_TPS, 0, aMax, jInc, jDec);
            double t1 = traj.t1;
            double t2 = traj.t2;
            double t3 = traj.t3;
            String shape = t2 > 1e-6 ? "trap" : "tri";

            VelocityMotorPF.VelocityMotorPFConfig cfg = makeConfig(jInc, 10.0, 5.0, 125, 250);
            double tReady = measureTimeToReady(cfg, 20.0);
            System.out.printf("%-12.0f %-10.1f %-10.1f %-8.4f %-8.4f %-8.4f %-6s | %.3f%n",
                    jInc, aMax, jDec, t1, t2, t3, shape, tReady);
        }
    }

    @Test
    void sweepAllParameters() {
        System.out.println("\n=== VelocityMotorPFWrapper Config Sweep (target=" + TARGET_TPS + " TPS) ===");
        System.out.printf("%-18s %-14s %-12s %-22s %-12s | %s%n",
                "jerkInc", "measLpfHz", "accelLpfHz", "accelTolerance", "kpRampMs", "t_ready (s)");
        System.out.println("-".repeat(100));

        // baseline (current wrapper values, except kpRampMs default 250)
        printRow(5000,  10.0, 2.0,  75,  250, "← baseline");

        // --- jerkIncreasing sweep ---
        for (double jInc : new double[]{2500, 7500, 10000, 15000}) {
            printRow(jInc,  10.0, 2.0,  75, 250, "");
        }

        System.out.println();

        // --- measurementLpfCutoffHz sweep ---
        for (double mHz : new double[]{6.0, 8.0, 12.0, 15.0, 20.0}) {
            printRow(5000, mHz, 2.0, 75, 250, "");
        }

        System.out.println();

        // --- accelLpfCutoffHz sweep ---
        for (double aHz : new double[]{1.0, 3.0, 4.0, 5.0, 8.0, 10.0}) {
            printRow(5000, 10.0, aHz, 75, 250, "");
        }

        System.out.println();

        // --- accelerationTolerance sweep ---
        for (double tol : new double[]{50, 100, 125, 150, 200}) {
            printRow(5000, 10.0, 2.0, tol, 250, "");
        }

        System.out.println();

        // --- kpRampMs sweep ---
        for (double ramp : new double[]{0, 50, 100, 150, 500}) {
            printRow(5000, 10.0, 2.0, 75, ramp, "");
        }

        System.out.println();

        // --- promising combinations ---
        printRow(10000, 12.0, 4.0, 100, 100, "← candidate A");
        printRow(10000, 10.0, 5.0, 125, 100, "← candidate B");
        printRow(10000, 12.0, 5.0, 125,  50, "← candidate C");
        printRow(15000, 12.0, 5.0, 125,  50, "← candidate D");
    }

    private static void printRow(double jInc, double mHz, double aHz,
                                  double tol, double ramp, String label) {
        VelocityMotorPF.VelocityMotorPFConfig cfg = makeConfig(jInc, mHz, aHz, tol, ramp);
        double t = measureTimeToReady(cfg, 20.0);
        String tStr = Double.isNaN(t) ? "TIMEOUT" : String.format("%.3f", t);
        System.out.printf("%-18.0f %-14.1f %-12.1f %-22.0f %-12.0f | %s  %s%n",
                jInc, mHz, aHz, tol, ramp, tStr, label);
    }
}
