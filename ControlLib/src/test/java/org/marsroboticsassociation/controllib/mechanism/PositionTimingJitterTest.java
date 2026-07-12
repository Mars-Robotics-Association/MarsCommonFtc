package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * Exercises the live-position read-timing jitter model: the dominant real noise once position is a
 * live counter instead of a 10 ms latch. Two things should hold &mdash; the read error grows with
 * speed (the {@code v * delta} signature) and with the hub (Expansion's RS485 hop is noisier than a
 * Control Hub) &mdash; and the EKF's repurposed timing-jitter noise term should keep the estimate
 * stable under it. A fixed seed makes every assertion reproducible.
 */
public class PositionTimingJitterTest {

    private static final double TICKS_PER_RAD = 28 * 100 / (2 * Math.PI);
    private static final double K_S = 0.1, K_G = 1.5, K_V = 2.0, K_A = 0.2;
    private static final double VOLTAGE = 12.0;

    @Test
    public void readNoiseScalesWithVelocityAndHub() {
        // Same seed, same hub: a faster sweep reads noisier (error is velocity * delta).
        double slow = rawReadNoiseRms(ReadTimingJitter.controlHub(1L), 1.0);
        double fast = rawReadNoiseRms(ReadTimingJitter.controlHub(1L), 6.0);
        assertTrue(
                "faster motion should read noisier: " + slow + " vs " + fast + " ticks RMS",
                fast > slow);

        // Same speed, noisier hub: the Expansion-Hub hop beats the Control Hub.
        double fastControl = rawReadNoiseRms(ReadTimingJitter.controlHub(1L), 6.0);
        double fastExpansion = rawReadNoiseRms(ReadTimingJitter.expansionHub(1L), 6.0);
        assertTrue(
                "expansion hub should be noisier than control: "
                        + fastControl
                        + " vs "
                        + fastExpansion
                        + " ticks RMS",
                fastExpansion > fastControl);
    }

    @Test
    public void stationaryEncoderHasNoReadError() {
        // delta only bites through velocity, so a stationary mechanism reads exactly.
        double still = rawReadNoiseRms(ReadTimingJitter.expansionHub(3L), 0.0);
        assertTrue("a stationary encoder should have no read error, got " + still, still == 0.0);
    }

    @Test
    public void ekfStaysStableUnderControlHubJitter() {
        // The same gravity sweep as ArmEkfTest, but with Control-Hub jitter on the position reads
        // and a matching timing-jitter noise term in the EKF. The estimate should stay tight.
        double angleRms = sweepAngleRms(ReadTimingJitter.CONTROL_HUB_STD_SEC, 1234L);
        assertTrue("EKF angle RMS too high under jitter: " + angleRms + " rad", angleRms < 0.03);
    }

    /** RMS of the jittered position read against the exact moving count, in ticks. */
    private static double rawReadNoiseRms(ReadTimingJitter jitter, double velRadPerSec) {
        double velTps = velRadPerSec * TICKS_PER_RAD;
        double sumSq = 0.0;
        int n = 0;
        for (int i = 1; i <= 500; i++) {
            double t = i * 0.005;
            double trueTicks = velTps * t;
            double err = jitter.read(trueTicks, velTps) - trueTicks;
            sumSq += err * err;
            n++;
        }
        return Math.sqrt(sumSq / n);
    }

    /** Runs the -45..+45 deg sweep with jittered reads and returns the EKF angle RMS in radians. */
    private static double sweepAngleRms(double jitterStdSec, long seed) {
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
                        new ReadTimingJitter(jitterStdSec, seed));
        MotorMechanismEkf ekf =
                new MotorMechanismEkf(
                        new ArmModel(K_S, K_V, K_A, K_G, 0.0),
                        /* velocityLagSec= */ 0.025,
                        /* modelAccelStdDev= */ 5.0,
                        /* positionStdDev= */ 0.003,
                        /* velocityStdDev= */ 0.1,
                        /* positionTimingJitterStdDev= */ jitterStdSec,
                        startRad);

        Random rng = new Random(seed);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;
        double sumSq = 0.0;
        int n = 0;

        for (int ms = 0; ms <= 2500; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(
                        plant.getEncoderPosition() / TICKS_PER_RAD,
                        plant.getEncoderVelocityTps() / TICKS_PER_RAD);

                double trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD;
                double trueVelTps = plant.getTrueAngularVelocityRadPerSec() * TICKS_PER_RAD;
                power =
                        0.004 * (targetRad * TICKS_PER_RAD - trueTicks)
                                - 0.0008 * trueVelTps
                                + (K_G / VOLTAGE) * Math.cos(plant.getTrueAngleRad());
                power = Math.max(-1.0, Math.min(1.0, power));

                if (ms / 1000.0 > 0.2) {
                    double err = ekf.getPosition() - plant.getTrueAngleRad();
                    sumSq += err * err;
                    n++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
        return Math.sqrt(sumSq / n);
    }
}
