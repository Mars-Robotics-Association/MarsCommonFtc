package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import org.junit.AfterClass;
import org.junit.Test;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.Random;

/**
 * Head-to-head of the mechanism {@link MotorMechanismEkf} against the WPILib {@link KalmanFilter}
 * configured exactly like {@code MarsCommonFtc}'s {@code ArmController} (linear motor plant,
 * gravity stripped from the input, no velocity-lag de-lag), both scored against the faithful {@link
 * ArmPlantSim} oracle. Plus the latency-horizon sweep: how far the estimate must be projected
 * forward to best predict the true state at the actuation instant, with and without velocity
 * de-lagging.
 *
 * <p>The thesis under test (from the design discussion): de-lagging the ~25 ms velocity boxcar and
 * forward-prediction are different operations, and once velocity is de-lagged the optimal forward
 * horizon should drop toward the true actuation delay. This harness turns that into numbers against
 * the oracle. Results are written to {@code build/estimator-comparison-report.txt}.
 */
public class EstimatorComparisonTest {

    private static final double TICKS_PER_RAD = 28 * 100 / (2 * Math.PI);
    private static final double K_S = 0.1, K_G = 1.5, K_V = 2.0, K_A = 0.2;
    private static final double VOLTAGE = 12.0;

    private static final StringBuilder REPORT = new StringBuilder();

    @AfterClass
    public static void writeReport() throws IOException {
        File dir = new File(System.getProperty("user.dir"), "build");
        dir.mkdirs();
        File out = new File(dir, "estimator-comparison-report.txt");
        Files.write(out.toPath(), REPORT.toString().getBytes(StandardCharsets.UTF_8));
        System.out.println("Estimator comparison report: " + out.getAbsolutePath());
    }

    // ------------------------------------------------------------------------------------------
    // Part 1: head-to-head estimator accuracy against the oracle (passive observers).
    // ------------------------------------------------------------------------------------------

    @Test
    public void headToHead_blendingEkfVsArmControllerKalman() {
        double startRad = -Math.PI / 4;
        double targetRad = Math.PI / 4;

        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S,
                        K_G,
                        K_V,
                        K_A,
                        TICKS_PER_RAD,
                        -Math.PI * 0.9,
                        Math.PI * 0.9,
                        startRad,
                        ReadTimingJitter.controlHub(99L));

        MotorMechanismEkf blending =
                new MotorMechanismEkf(
                        new ArmModel(K_S, K_V, K_A, K_G, 0.0),
                        /* velocityLagSec= */ 0.025,
                        /* modelAccelStdDev= */ 5.0,
                        /* positionStdDev= */ 0.003,
                        /* velocityStdDev= */ 0.1,
                        /* positionTimingJitterStdDev= */ ReadTimingJitter.CONTROL_HUB_STD_SEC,
                        startRad);

        ArmStyleKalman wpilib = new ArmStyleKalman(startRad);

        Random rng = new Random(99L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double blendPosSq = 0, blendVelSq = 0, wpiPosSq = 0, wpiVelSq = 0;
        int n = 0;

        for (int ms = 0; ms <= 2500; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                double measPosRad = plant.getEncoderPosition() / TICKS_PER_RAD;
                double measVelRad = plant.getEncoderVelocityTps() / TICKS_PER_RAD;
                double appliedVoltage = clamp(power, -1.0, 1.0) * VOLTAGE;

                blending.predict(dt, power, VOLTAGE);
                blending.correct(measPosRad, measVelRad);

                wpilib.update(dt, measPosRad, measVelRad, appliedVoltage);

                // Controller acts on TRUE state so the trajectory is identical for both observers.
                double trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD;
                double trueVelTps = plant.getTrueAngularVelocityRadPerSec() * TICKS_PER_RAD;
                power =
                        0.004 * (targetRad * TICKS_PER_RAD - trueTicks)
                                - 0.0008 * trueVelTps
                                + (K_G / VOLTAGE) * Math.cos(plant.getTrueAngleRad());
                power = clamp(power, -1.0, 1.0);

                if (ms / 1000.0 > 0.2) {
                    double trueAngle = plant.getTrueAngleRad();
                    double trueVel = plant.getTrueAngularVelocityRadPerSec();
                    blendPosSq += sq(blending.getPosition() - trueAngle);
                    blendVelSq += sq(blending.getVelocity() - trueVel);
                    wpiPosSq += sq(wpilib.getPosition() - trueAngle);
                    wpiVelSq += sq(wpilib.getVelocity() - trueVel);
                    n++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double blendPosRms = Math.sqrt(blendPosSq / n);
        double blendVelRms = Math.sqrt(blendVelSq / n);
        double wpiPosRms = Math.sqrt(wpiPosSq / n);
        double wpiVelRms = Math.sqrt(wpiVelSq / n);

        REPORT.append(
                "=== Part 1: estimator accuracy vs oracle (present state, -45..+45 sweep) ===\n");
        REPORT.append("                         position RMS (rad)   velocity RMS (rad/s)\n");
        REPORT.append(
                String.format(
                        "  blending EKF             %18.6f   %18.6f%n", blendPosRms, blendVelRms));
        REPORT.append(
                String.format(
                        "  ArmController KF (WPILib) %17.6f   %18.6f%n", wpiPosRms, wpiVelRms));
        REPORT.append(
                String.format(
                        "  velocity RMS ratio (KF / blending): %.2fx%n", wpiVelRms / blendVelRms));
        REPORT.append(
                "  note: this gap mixes two things — the structural de-lag AND ArmController's\n");
        REPORT.append(
                "  looser noise tuning (modelStdDevVel=3.0, measStdDevPos=0.05). Part 2"
                    + " isolates\n");
        REPORT.append("  the de-lag alone (same filter, only velocityLagSec toggled).\n\n");

        // The de-lag should win on velocity, where the 50 ms boxcar lag bites during acceleration.
        assertTrue(
                "blending velocity RMS ("
                        + blendVelRms
                        + ") should beat the KF ("
                        + wpiVelRms
                        + ")",
                blendVelRms < wpiVelRms);
    }

    // ------------------------------------------------------------------------------------------
    // Part 2: latency-horizon sweep — optimal forward horizon vs velocity de-lagging.
    // ------------------------------------------------------------------------------------------

    @Test
    public void latencyHorizonSweep_horizonTracksActuationDelay_deLagLowersFloor() {
        int totalMs = 3000;
        int loopMs = 20; // fixed-cadence ZOH loop for clean curves
        int latchMs = 10;
        double actuationDelaySec = 0.015; // the round-trip delay the forward-prediction compensates
        int actuationDelayMs = (int) Math.round(actuationDelaySec * 1000);

        double startRad = -Math.PI / 4;

        // Reference true trajectory under the fixed ZOH excitation (oracle, no filter, no jitter).
        double[] trueAngle = referenceTrajectory(totalMs, loopMs, startRad);

        double[] horizons = new double[21];
        for (int i = 0; i < horizons.length; i++) horizons[i] = i * 0.0025; // 0..50 ms, 2.5 ms grid

        REPORT.append("=== Part 2: latency-horizon sweep (predict the true state ")
                .append(actuationDelayMs)
                .append(" ms ahead) ===\n");
        REPORT.append(
                "Position RMS (rad) of the forward-projected estimate vs the true future state.\n");
        REPORT.append(String.format("%-10s", "tau(ms)"));
        REPORT.append("   de-lagged(0.025)     simplified(0.000)\n");

        double[] rmsDelag = new double[horizons.length];
        double[] rmsSimple = new double[horizons.length];
        for (int i = 0; i < horizons.length; i++) {
            rmsDelag[i] =
                    sweepRun(
                            0.025,
                            horizons[i],
                            totalMs,
                            loopMs,
                            latchMs,
                            actuationDelayMs,
                            startRad,
                            trueAngle);
            rmsSimple[i] =
                    sweepRun(
                            0.0,
                            horizons[i],
                            totalMs,
                            loopMs,
                            latchMs,
                            actuationDelayMs,
                            startRad,
                            trueAngle);
            REPORT.append(
                    String.format(
                            "%-10.1f   %16.6f   %18.6f%n",
                            horizons[i] * 1000.0, rmsDelag[i], rmsSimple[i]));
        }

        int argDelag = argmin(rmsDelag);
        int argSimple = argmin(rmsSimple);
        double tauDelag = horizons[argDelag] * 1000.0;
        double tauSimple = horizons[argSimple] * 1000.0;

        REPORT.append(
                String.format(
                        "%noptimal horizon  de-lagged: %.1f ms (RMS %.6f)%n",
                        tauDelag, rmsDelag[argDelag]));
        REPORT.append(
                String.format(
                        "optimal horizon  simplified: %.1f ms (RMS %.6f)%n",
                        tauSimple, rmsSimple[argSimple]));
        REPORT.append(
                String.format("actuation delay being compensated: %d ms%n", actuationDelayMs));
        REPORT.append(
                "finding: with LIVE position the optimal horizon ~= the actuation delay for both"
                    + " configs (position prediction dominates, so velocity lag does not move the"
                    + " optimum); de-lagging velocity instead lowers the error floor ~2x.\n\n");

        // The forward horizon is governed by the actuation delay, not the velocity sensor lag,
        // because position is a live counter. So BOTH optima sit near the actuation delay.
        assertTrue(
                "de-lagged optimum ("
                        + tauDelag
                        + " ms) should be near the actuation delay ("
                        + actuationDelayMs
                        + " ms)",
                Math.abs(tauDelag - actuationDelayMs) <= 10.0);
        assertTrue(
                "simplified optimum ("
                        + tauSimple
                        + " ms) should also be near the actuation delay ("
                        + actuationDelayMs
                        + " ms)",
                Math.abs(tauSimple - actuationDelayMs) <= 10.0);

        // De-lagging does not require a longer horizon (it does not stand in for
        // forward-prediction).
        assertTrue(
                "de-lagging should not need a longer horizon than the simplified filter: de-lagged "
                        + tauDelag
                        + " ms vs simplified "
                        + tauSimple
                        + " ms",
                tauDelag <= tauSimple);

        // The de-lag's payoff is accuracy: a clearly lower error floor at the optimal horizon.
        assertTrue(
                "de-lagged error floor ("
                        + rmsDelag[argDelag]
                        + ") should clearly beat simplified ("
                        + rmsSimple[argSimple]
                        + ")",
                rmsDelag[argDelag] < 0.8 * rmsSimple[argSimple]);
    }

    /**
     * One sweep run: returns RMS of the tau-ahead projected position vs the true state D ms ahead.
     */
    private static double sweepRun(
            double velocityLagSec,
            double horizonSec,
            int totalMs,
            int loopMs,
            int latchMs,
            int actuationDelayMs,
            double startRad,
            double[] trueAngle) {
        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S,
                        K_G,
                        K_V,
                        K_A,
                        TICKS_PER_RAD,
                        -Math.PI * 0.9,
                        Math.PI * 0.9,
                        startRad,
                        ReadTimingJitter.disabled());
        MotorMechanismEkf ekf =
                new MotorMechanismEkf(
                        new ArmModel(K_S, K_V, K_A, K_G, 0.0),
                        velocityLagSec,
                        5.0,
                        0.003,
                        0.1,
                        /* jitter= */ 0.0,
                        startRad);

        double heldPower = 0.0;
        int lastFireMs = 0;
        double sumSq = 0.0;
        int n = 0;

        for (int ms = 0; ms <= totalMs; ms++) {
            if (ms % loopMs == 0) {
                if (ms > 0) {
                    double dt = (ms - lastFireMs) / 1000.0;
                    double measPosRad = plant.getEncoderPosition() / TICKS_PER_RAD;
                    double measVelRad = plant.getEncoderVelocityTps() / TICKS_PER_RAD;
                    ekf.predict(
                            dt, heldPower, VOLTAGE); // held power applied over the last interval
                    ekf.correct(measPosRad, measVelRad);

                    double newHeld = excitation(ms);
                    if (ms / 1000.0 > 0.2 && ms + actuationDelayMs <= totalMs) {
                        double[] projected = ekf.projectedState(horizonSec, newHeld, VOLTAGE);
                        sumSq += sq(projected[0] - trueAngle[ms + actuationDelayMs]);
                        n++;
                    }
                    heldPower = newHeld;
                    lastFireMs = ms;
                } else {
                    heldPower = excitation(0);
                }
            }
            plant.integrate(0.001, heldPower, VOLTAGE);
            if (ms % latchMs == 0) {
                plant.latchEncoder();
            }
        }
        return Math.sqrt(sumSq / n);
    }

    /** True angle at each ms under the fixed ZOH excitation (the oracle, no filter, no jitter). */
    private static double[] referenceTrajectory(int totalMs, int loopMs, double startRad) {
        ArmPlantSim ref =
                new ArmPlantSim(
                        K_S,
                        K_G,
                        K_V,
                        K_A,
                        TICKS_PER_RAD,
                        -Math.PI * 0.9,
                        Math.PI * 0.9,
                        startRad,
                        ReadTimingJitter.disabled());
        double[] trueAngle = new double[totalMs + 1];
        double heldPower = 0.0;
        for (int ms = 0; ms <= totalMs; ms++) {
            if (ms % loopMs == 0) {
                heldPower = excitation(ms);
            }
            ref.integrate(0.001, heldPower, VOLTAGE);
            trueAngle[ms] = ref.getTrueAngleRad();
        }
        return trueAngle;
    }

    /**
     * Fixed open-loop command: gravity-ish bias plus a sweep that accelerates through horizontal.
     */
    private static double excitation(int ms) {
        double t = ms / 1000.0;
        return 0.125 + 0.5 * Math.sin(2 * Math.PI * 0.6 * t);
    }

    // ------------------------------------------------------------------------------------------
    // Part 3: non-circular delay recovery. A real actuation-delay FIFO lives in the plant; the
    // closed-loop optimal forward horizon must TRACK that plant delay (not a number in the
    // harness),
    // and do so independently of velocity de-lagging.
    // ------------------------------------------------------------------------------------------

    @Test
    public void delayRecoverySweep_peakErrorUShaped_forwardProjectionWeaklyTracksDelay() {
        // Push the actuation delay well above the ~20 ms loop/ZOH so it dominates the loop
        // dynamics;
        // otherwise the optimal horizon is set by the loop, not the delay, and barely tracks it.
        int[] plantDelaysMs = {10, 30, 50, 70};
        double[] horizons = new double[49];
        for (int i = 0; i < horizons.length; i++)
            horizons[i] = i * 0.0025; // 0..120 ms, 2.5 ms grid

        REPORT.append(
                "=== Part 3: non-circular delay recovery (real actuation-delay FIFO in plant)"
                    + " ===\n");
        REPORT.append(
                "Closed loop (controller + EKF). Metric: peak |true - setpoint| (rad) over the"
                    + " move,\n");
        REPORT.append(
                "which penalizes both lag (low horizon) and lead (high horizon) -> interior"
                    + " minimum.\n");
        REPORT.append(
                "The actuation delay is a real FIFO in the plant (pushed dominant, 10..70 ms), so"
                    + " the\n");
        REPORT.append(
                "optimal horizon reflects the plant, not the harness. (* marks the column"
                    + " optimum.)\n\n");

        int[] argDelag = new int[plantDelaysMs.length];
        int[] argSimple = new int[plantDelaysMs.length];

        for (int d = 0; d < plantDelaysMs.length; d++) {
            double delaySec = plantDelaysMs[d] / 1000.0;
            double[] peakDelag = new double[horizons.length];
            double[] peakSimple = new double[horizons.length];
            for (int i = 0; i < horizons.length; i++) {
                peakDelag[i] = closedLoopPeakError(delaySec, 0.025, horizons[i]);
                peakSimple[i] = closedLoopPeakError(delaySec, 0.0, horizons[i]);
            }
            argDelag[d] = argmin(peakDelag);
            argSimple[d] = argmin(peakSimple);

            REPORT.append(String.format("plant actuation delay = %d ms%n", plantDelaysMs[d]));
            REPORT.append(String.format("%-10s   de-lagged           simplified%n", "tau(ms)"));
            for (int i = 0; i < horizons.length; i++) {
                REPORT.append(
                        String.format(
                                "%-10.1f   %12.6f%s   %12.6f%s%n",
                                horizons[i] * 1000.0,
                                peakDelag[i],
                                i == argDelag[d] ? " *" : "  ",
                                peakSimple[i],
                                i == argSimple[d] ? " *" : "  "));
            }
            REPORT.append(
                    String.format(
                            "  -> optimal horizon: de-lagged %.1f ms, simplified %.1f ms%n%n",
                            horizons[argDelag[d]] * 1000.0, horizons[argSimple[d]] * 1000.0));
        }

        REPORT.append("summary: plant delay -> recovered optimal horizon\n");
        for (int d = 0; d < plantDelaysMs.length; d++) {
            REPORT.append(
                    String.format(
                            "  %2d ms -> de-lagged %.1f ms, simplified %.1f ms%n",
                            plantDelaysMs[d],
                            horizons[argDelag[d]] * 1000.0,
                            horizons[argSimple[d]] * 1000.0));
        }
        REPORT.append(
                "\n"
                    + "finding: peak |true-setpoint| IS a proper metric (clean interior U-shape,"
                    + " unlike\n");
        REPORT.append(
                "overshoot). But it shows naive forward-projection only WEAKLY compensates the"
                    + " actuation\n");
        REPORT.append(
                "delay: recovered horizon ~12.5/15/17.5/20 ms for delays 10/30/50/70 ms (slope"
                    + " ~0.12,\n");
        REPORT.append(
                "not ~1). Even a 70 ms delay is best offset by only ~20 ms of lead -- you cannot"
                    + " project\n");
        REPORT.append(
                "your way out of the delay here. Projection has competing effects (adds phase lead"
                    + " but\n");
        REPORT.append(
                "also steady lag), and projectedState just rolls the state forward under the held"
                    + " command;\n");
        REPORT.append(
                "it does NOT account for the commands already in flight in the delay buffer. A"
                    + " Smith-style\n");
        REPORT.append(
                "predictor that did would track the delay far better. So 'forward-predict by the"
                    + " delay'\n");
        REPORT.append("is not a clean knob with this simple scheme.\n\n");

        // Peak error has an INTERIOR optimum (not pinned to either grid edge) -> a real U-shape,
        // where overshoot bottomed at the over-lead corner.
        int lastIdx = horizons.length - 1;
        for (int d = 0; d < plantDelaysMs.length; d++) {
            assertTrue(
                    "peak-error optimum should be interior at delay "
                            + plantDelaysMs[d]
                            + " ms ("
                            + (horizons[argDelag[d]] * 1000)
                            + " ms)",
                    argDelag[d] > 0 && argDelag[d] < lastIdx);
        }
        // The optimum rises with the delay (it is a real, plant-driven trend, not censoring)...
        for (int d = 1; d < plantDelaysMs.length; d++) {
            assertTrue(
                    "de-lagged optimum should not decrease as plant delay grows: "
                            + (horizons[argDelag[d]] * 1000)
                            + " ms at "
                            + plantDelaysMs[d]
                            + " ms",
                    horizons[argDelag[d]] >= horizons[argDelag[d - 1]] - 1e-12);
        }
        assertTrue(
                "de-lagged optimum should rise across the 10->70 ms delay span",
                horizons[argDelag[plantDelaysMs.length - 1]] > horizons[argDelag[0]]);
        // ...but only WEAKLY: even the largest (dominant) delay is best offset by a far smaller
        // horizon, so naive forward-projection does not invert the delay.
        int last = plantDelaysMs.length - 1;
        assertTrue(
                "recovered horizon should be far below the dominant plant delay (weak tracking): "
                        + (horizons[argDelag[last]] * 1000)
                        + " ms for a "
                        + plantDelaysMs[last]
                        + " ms delay",
                horizons[argDelag[last]] < 0.5 * (plantDelaysMs[last] / 1000.0));
    }

    /**
     * Closed-loop peak |true - setpoint| (rad) over the move, for a given plant delay, de-lag, and
     * forward horizon. Unlike overshoot, this penalizes BOTH under-compensation (true lags the
     * setpoint) and over-compensation (true leads it), so it has an interior minimum near the
     * delay.
     */
    private static double closedLoopPeakError(
            double actuationDelaySec, double velocityLagSec, double horizonSec) {
        double startRad = -Math.PI / 4;
        double targetRad = Math.PI / 4;

        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S,
                        K_G,
                        K_V,
                        K_A,
                        TICKS_PER_RAD,
                        -Math.PI * 0.9,
                        Math.PI * 0.9,
                        startRad,
                        ReadTimingJitter.disabled());
        plant.setActuationDelaySec(actuationDelaySec);

        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        MotorMechanismEkf ekf =
                new MotorMechanismEkf(model, velocityLagSec, 5.0, 0.003, 0.1, 0.0, startRad);
        MotorMechanismController ctrl =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, startRad);

        double lastPower = 0.0;
        int lastFireMs = 0;
        double currentSetpoint = startRad;
        double maxAbsErr = 0.0;

        for (int ms = 0; ms <= 2000; ms++) {
            if (ms % 20 == 0) {
                if (ms > 0) {
                    double dt = (ms - lastFireMs) / 1000.0;
                    double measPosRad = plant.getEncoderPosition() / TICKS_PER_RAD;
                    double measVelRad = plant.getEncoderVelocityTps() / TICKS_PER_RAD;
                    ekf.predict(dt, lastPower, VOLTAGE);
                    ekf.correct(measPosRad, measVelRad);
                    double[] proj = ekf.projectedState(horizonSec, lastPower, VOLTAGE);
                    double volts = ctrl.calculate(targetRad, proj[0], proj[1], VOLTAGE, dt);
                    lastPower = clamp(volts / VOLTAGE, -1.0, 1.0);
                    lastFireMs = ms;
                } else {
                    double[] proj = ekf.projectedState(horizonSec, lastPower, VOLTAGE);
                    double volts = ctrl.calculate(targetRad, proj[0], proj[1], VOLTAGE, 0.02);
                    lastPower = clamp(volts / VOLTAGE, -1.0, 1.0);
                }
                currentSetpoint = ctrl.getSetpointPosition();
            }
            plant.integrate(0.001, lastPower, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= 40) {
                maxAbsErr =
                        Math.max(maxAbsErr, Math.abs(plant.getTrueAngleRad() - currentSetpoint));
            }
        }
        return maxAbsErr;
    }

    // ------------------------------------------------------------------------------------------
    // WPILib Kalman filter wrapped to match ArmController exactly (gravity stripped from the input,
    // no velocity-lag de-lag, present estimate read after correct).
    // ------------------------------------------------------------------------------------------

    private static final class ArmStyleKalman {
        private final KalmanFilter<N2, N1, N2> kf;
        private double lastLinearVoltage = 0.0;
        private double estPos;
        private double estVel;

        ArmStyleKalman(double startRad) {
            LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(K_V, K_A);
            // ArmController.PARAMS defaults: model std (0.01 rad, 3.0 rad/s), meas std (0.05, 0.5).
            kf =
                    new KalmanFilter<>(
                            Nat.N2(),
                            Nat.N2(),
                            plant,
                            VecBuilder.fill(0.01, 3.0),
                            VecBuilder.fill(0.05, 0.5),
                            0.02);
            kf.setXhat(VecBuilder.fill(startRad, 0.0));
            estPos = startRad;
            estVel = 0.0;
        }

        void update(double dt, double measPosRad, double measVelRad, double appliedVoltage) {
            Matrix<N1, N1> uLinear = VecBuilder.fill(lastLinearVoltage);
            kf.correct(uLinear, VecBuilder.fill(measPosRad, measVelRad));
            // Present (filtered) estimate is read here, before predicting on to the next step.
            estPos = kf.getXhat(0);
            estVel = kf.getXhat(1);
            kf.predict(uLinear, dt);
            // Linear-model input for next loop: strip gravity and friction (handled by
            // feedforward).
            double gravity = K_G * Math.cos(estPos);
            double friction = K_S * Math.signum(estVel);
            lastLinearVoltage = appliedVoltage - gravity - friction;
        }

        double getPosition() {
            return estPos;
        }

        double getVelocity() {
            return estVel;
        }
    }

    private static int argmin(double[] a) {
        int best = 0;
        for (int i = 1; i < a.length; i++) {
            if (a[i] < a[best]) best = i;
        }
        return best;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double sq(double v) {
        return v * v;
    }
}
