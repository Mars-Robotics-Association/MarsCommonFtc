package org.marsroboticsassociation.controllib.mechanism;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;
import org.marsroboticsassociation.controllib.sim.ArmMotorSim;
import org.marsroboticsassociation.controllib.sim.EncoderSim;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * ControlLib arm sysid must recover rigid-plant feedforward coefficients from encoder logs alone,
 * using the same integrated-EOM battery as ControlLab.
 */
@Execution(ExecutionMode.SAME_THREAD)
class ArmSysIdTest {

    private static final double DT = 0.01;
    private static final int RUN_STEPS = 90;
    private static final double HUB = 12.0;

    private static final double KS = 0.3;
    private static final double KG = 3.5;
    private static final double KV = 1.2;
    private static final double KA = 0.35;
    private static final int TICKS_PER_REV = 28;
    private static final double GEAR = 100.0;
    private static final double ZERO = 0.0;
    private static final double MIN = Math.toRadians(-45);
    private static final double MAX = Math.toRadians(225);

    @Test
    void recoversDefaultHeavyArmParams() {
        ArmSysId.Result r = runBattery(KS, KG, KV, KA);
        System.out.printf("sysid: kS=%.3f kV=%.3f kA=%.3f kCos=%.3f kSin=%.3f  R2=%.4f  n=%d%n",
                r.kS, r.kV, r.kA, r.kCos, r.kSin, r.rSquared, r.samples);

        assertTrue(r.rSquared > 0.99, "fit should be excellent, R2=" + r.rSquared);
        assertEquals(KA, r.kA, KA * 0.15, "kA within 15%");
        assertEquals(KG, r.kCos, KG * 0.10, "kCos (gravity) within 10%");
        assertEquals(KV, r.kV, KV * 0.15, "kV within 15%");
        assertEquals(KS, r.kS, 0.20, "kS within 0.2 V");
        assertEquals(0.0, r.kSin, 0.25, "kSin ~ 0 (plant gravity is pure cosine)");
        assertEquals(KG, r.kG(), KG * 0.10, "kG() matches hypot(kCos,kSin)");
        assertEquals(0.0, r.phiRad(), 0.1, "phi ~ 0 when pure cosine");
    }

    @Test
    void recoversAModifiedInertia() {
        double kA = 0.6;
        double kG = 2.5;
        ArmSysId.Result r = runBattery(KS, kG, KV, kA);
        System.out.printf("sysid(kA=0.6,kG=2.5): kA=%.3f kCos=%.3f R2=%.4f%n", r.kA, r.kCos, r.rSquared);
        assertEquals(kA, r.kA, kA * 0.15, "kA within 15%");
        assertEquals(kG, r.kCos, kG * 0.10, "kCos within 10%");
    }

    @Test
    void perSampleVoltageMatchesConstantVoltage() {
        // Battery-sag path (per-sample V) with constant V should match the constant-V overload.
        List<double[]> rowsA = new ArrayList<>();
        List<Double> rhsA = new ArrayList<>();
        List<double[]> rowsB = new ArrayList<>();
        List<Double> rhsB = new ArrayList<>();

        double[] theta = logRun(MIN + 0.05 * (MAX - MIN), 0.80, KS, KG, KV, KA);
        double v = 0.80 * HUB;
        double[] voltages = new double[theta.length];
        for (int i = 0; i < voltages.length; i++) {
            voltages[i] = v;
        }

        ArmSysId.accumulateRun(theta, v, DT, MIN, MAX, ArmSysId.DEFAULT_PARAMS, rowsA, rhsA);
        ArmSysId.accumulateRun(theta, voltages, DT, MIN, MAX, ArmSysId.DEFAULT_PARAMS, rowsB, rhsB);

        assertEquals(rowsA.size(), rowsB.size());
        for (int i = 0; i < rhsA.size(); i++) {
            assertEquals(rhsA.get(i), rhsB.get(i), 1e-9);
        }
    }

    /** Same constant-power battery as ControlLab (rigid plant, clean encoder). */
    private static ArmSysId.Result runBattery(double kS, double kG, double kV, double kA) {
        double span = MAX - MIN;
        double bottom = MIN + 0.05 * span;
        double top = MAX - 0.05 * span;
        double mid = MIN + 0.55 * span;
        double margin = 0.08 * span;

        List<double[]> rows = new ArrayList<>();
        List<Double> rhs = new ArrayList<>();

        for (double p : new double[] {0.45, 0.60, 0.80, 0.95}) {
            accumulate(bottom, p, kS, kG, kV, kA, rows, rhs);
        }
        for (double p : new double[] {-0.45, -0.60, -0.80, -0.95}) {
            accumulate(top, p, kS, kG, kV, kA, rows, rhs);
        }
        // Gravity-rich: horizontals in range
        List<Double> gravityStarts = new ArrayList<>();
        for (double h : new double[] {0.0, Math.PI, -Math.PI}) {
            if (h > MIN + margin && h < MAX - margin) {
                gravityStarts.add(h);
            }
        }
        if (gravityStarts.isEmpty()) {
            gravityStarts.add(mid);
        }
        for (double start : gravityStarts) {
            accumulate(start, 0.70, kS, kG, kV, kA, rows, rhs);
            accumulate(start, -0.70, kS, kG, kV, kA, rows, rhs);
        }

        return ArmSysId.solve(rows, rhs);
    }

    private static void accumulate(
            double startRad,
            double power,
            double kS,
            double kG,
            double kV,
            double kA,
            List<double[]> rows,
            List<Double> rhs) {
        double[] theta = logRun(startRad, power, kS, kG, kV, kA);
        ArmSysId.accumulateRun(
                theta, power * HUB, DT, MIN, MAX, ArmSysId.DEFAULT_PARAMS, rows, rhs);
    }

    private static double[] logRun(
            double startRad, double power, double kS, double kG, double kV, double kA) {
        double ticksPerRad = TICKS_PER_REV * GEAR / (2.0 * Math.PI);
        ArmMotorSim sim = new ArmMotorSim(
                kS, kG, kV, kA, TICKS_PER_REV, GEAR, ZERO, MIN, MAX, startRad);
        sim.setEncoder(new EncoderSim());
        double[] theta = new double[RUN_STEPS];
        for (int i = 0; i < RUN_STEPS; i++) {
            sim.step(DT, power, HUB);
            theta[i] = sim.getPositionTicks() / ticksPerRad + ZERO;
        }
        return theta;
    }
}
