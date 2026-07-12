package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * Measures how well a {@link MotorMechanismEkf} on a {@link LiftModel} tracks over a lift move,
 * driven through the realistic {@link EncoderSim} and an unsynced, jittery control loop. As with
 * the arm test, the controller runs on the plant's true state so the motion is fixed and the filter
 * rides along as a passive observer. The lift's gravity is constant, so its EKF is really a linear
 * Kalman filter with a control input, but it still benefits from back-EMF-aware prediction and
 * velocity-lag compensation.
 */
public class LiftEkfTest {

    private static final double K_S = 0.1, K_G = 2.0, K_V = 0.006, K_A = 0.0016;
    private static final double VOLTAGE = 12.0;

    @Test
    public void tracksPositionAndVelocityThroughMove() {
        double start = 0.0;
        double targetTicks = 1500.0;

        LiftPlantSim plant = new LiftPlantSim(K_S, K_G, K_V, K_A, -100, 3000, start);

        MotorMechanismEkf ekf =
                new MotorMechanismEkf(
                        new LiftModel(K_S, K_V, K_A, K_G),
                        /* velocityLagSec= */ 0.025,
                        /* modelAccelStdDev= */ 1500.0,
                        /* positionStdDev= */ 1.0,
                        /* velocityStdDev= */ 30.0,
                        /* positionTimingJitterStdDev= */ 0.0,
                        start);

        Random rng = new Random(5L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double ekfVelSq = 0, ekfPosSq = 0;
        int samples = 0;

        for (int ms = 0; ms <= 2500; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                int pos = plant.getEncoderPosition();
                double velTps = plant.getEncoderVelocityTps();

                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(pos, velTps);

                double truePos = plant.getTruePositionTicks();
                double trueVel = plant.getTrueVelocityTps();
                power = 0.002 * (targetTicks - truePos) - 0.0006 * trueVel + (K_G / VOLTAGE);
                power = Math.max(-1.0, Math.min(1.0, power));

                double t = ms / 1000.0;
                if (t > 0.2) {
                    ekfVelSq += sq(ekf.getVelocity() - trueVel);
                    ekfPosSq += sq(ekf.getPosition() - truePos);
                    samples++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double velRms = Math.sqrt(ekfVelSq / samples);
        double posRms = Math.sqrt(ekfPosSq / samples);

        assertTrue("velocity RMS too high: " + velRms + " TPS", velRms < 25.0);
        assertTrue("position RMS too high: " + posRms + " ticks", posRms < 6.0);
    }

    private static double sq(double v) {
        return v * v;
    }
}
