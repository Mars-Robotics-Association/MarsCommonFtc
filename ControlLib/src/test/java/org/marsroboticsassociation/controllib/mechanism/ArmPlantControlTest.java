package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * Closed-loop test: a {@link MotorMechanismEkf} on an {@link ArmModel} driving a real second-order
 * arm plant ({@link ArmPlantSim}) through a PIDF controller. The controller never sees the true
 * state; it uses only the filter's estimated angle (P and gravity feedforward) and estimated
 * velocity (D), and feeds the filter the voltage it commanded. This exercises the EKF the way it
 * would actually be used on a robot.
 *
 * <p>The move sweeps the arm from -45 deg, down through horizontal where gravity torque peaks, up
 * to +45 deg, then holds.
 */
public class ArmPlantControlTest {

    private static final double TICKS_PER_RAD = 28 * 100 / (2 * Math.PI);
    private static final double K_S = 0.1; // static friction, volts
    private static final double K_G = 1.5; // gravity, volts at horizontal
    private static final double K_V = 2.0; // volts per rad/s (back-EMF)
    private static final double K_A = 0.2; // volts per rad/s^2
    private static final double VOLTAGE = 12.0;

    // PIDF gains (normalized power, tick units) and filter tuning, from calibration.
    private static final double K_P = 0.004;
    private static final double K_D = 0.0008;
    private static final double K_F = K_G / VOLTAGE;

    private static final double START_RAD = -Math.PI / 4;
    private static final double TARGET_RAD = Math.PI / 4;

    @Test
    public void armReachesAndHoldsTargetUsingOnlyFilterEstimates() {
        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S,
                        K_G,
                        K_V,
                        K_A,
                        TICKS_PER_RAD,
                        -Math.PI * 0.9,
                        Math.PI * 0.9,
                        START_RAD);
        MotorMechanismEkf filter =
                new MotorMechanismEkf(
                        new ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0),
                        /* velocityLagSec= */ 0.025,
                        /* modelAccelStdDev= */ 5.0,
                        /* positionStdDev= */ 0.003,
                        /* velocityStdDev= */ 0.1,
                        /* positionTimingJitterStdDev= */ 0.0,
                        START_RAD);

        double targetTicks = TARGET_RAD * TICKS_PER_RAD;
        Random rng = new Random(2024L);

        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double maxOvershoot = 0.0;
        double settleMs = -1.0;
        double sumAngleErrSq = 0.0;
        int angleSamples = 0;
        double sumSteadyAbsErr = 0.0;
        int steadySamples = 0;

        for (int ms = 0; ms <= 2500; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                int position = plant.getEncoderPosition();
                double velocityTps = plant.getEncoderVelocityTps();

                // The voltage held over the last interval was the previously commanded power.
                filter.predict(dt, power, VOLTAGE);
                filter.correct(position / TICKS_PER_RAD, velocityTps / TICKS_PER_RAD);

                double estAngleRad = filter.getPosition();
                double estAngleTicks = estAngleRad * TICKS_PER_RAD;
                double estVelocityTps = filter.getVelocity() * TICKS_PER_RAD;

                power =
                        K_P * (targetTicks - estAngleTicks)
                                - K_D * estVelocityTps
                                + K_F * Math.cos(estAngleRad);
                power = Math.max(-1.0, Math.min(1.0, power));

                double trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD;
                double t = ms / 1000.0;
                double overshoot = trueTicks - targetTicks;
                if (overshoot > maxOvershoot) {
                    maxOvershoot = overshoot;
                }
                if (settleMs < 0 && Math.abs(trueTicks - targetTicks) < 10.0) {
                    settleMs = ms;
                }
                if (t > 0.3) {
                    sumAngleErrSq += sq(estAngleTicks - trueTicks);
                    angleSamples++;
                }
                if (t > 1.5) {
                    sumSteadyAbsErr += Math.abs(trueTicks - targetTicks);
                    steadySamples++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double finalErrTicks = plant.getTrueAngleRad() * TICKS_PER_RAD - targetTicks;
        double steadyAbsErr = sumSteadyAbsErr / steadySamples;
        double filterAngleRms = Math.sqrt(sumAngleErrSq / angleSamples);

        assertTrue(
                "did not reach target, final error " + finalErrTicks + " ticks",
                Math.abs(finalErrTicks) < 10.0);
        assertTrue("steady-state error too high: " + steadyAbsErr + " ticks", steadyAbsErr < 8.0);
        assertTrue("overshoot too high: " + maxOvershoot + " ticks", maxOvershoot < 30.0);
        assertTrue("settled too slowly: " + settleMs + " ms", settleMs >= 0 && settleMs < 2000.0);
        assertTrue(
                "filter angle estimate drifted from truth: " + filterAngleRms + " ticks RMS",
                filterAngleRms < 4.0);
    }

    private static double sq(double v) {
        return v * v;
    }
}
