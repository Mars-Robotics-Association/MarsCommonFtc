package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * Measures how well a {@link MotorMechanismEkf} on an {@link ArmModel} tracks as the arm sweeps
 * from -45 deg through horizontal to +45 deg, driven through the realistic {@link EncoderSim} and
 * an unsynced, jittery control loop.
 *
 * <p>To isolate estimator quality from the controller, the controller runs on the plant's true
 * state, so the motion is fixed; the filter rides along as a passive observer on the same encoder
 * stream and the same commanded voltage. The model and the velocity-lag compensation should keep
 * velocity error low even while the arm is accelerating, where the 50 ms velocity reading lags
 * hardest.
 */
public class ArmEkfTest {

    private static final double TICKS_PER_RAD = 28 * 100 / (2 * Math.PI);
    private static final double K_S = 0.1, K_G = 1.5, K_V = 2.0, K_A = 0.2;
    private static final double VOLTAGE = 12.0;

    @Test
    public void tracksAngleAndVelocityThroughGravitySweep() {
        double startRad = -Math.PI / 4;
        double targetRad = Math.PI / 4;

        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S, K_G, K_V, K_A, TICKS_PER_RAD, -Math.PI * 0.9, Math.PI * 0.9, startRad);

        MotorMechanismEkf ekf =
                new MotorMechanismEkf(
                        new ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0),
                        /* velocityLagSec= */ 0.025,
                        /* modelAccelStdDev= */ 5.0,
                        /* positionStdDev= */ 0.003,
                        /* velocityStdDev= */ 0.1,
                        /* positionTimingJitterStdDev= */ 0.0,
                        startRad);

        Random rng = new Random(99L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double ekfVelSqAll = 0, ekfVelSqMoving = 0, ekfAngleSq = 0;
        int allSamples = 0, movingSamples = 0;

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
                ekf.correct(pos / TICKS_PER_RAD, velTps / TICKS_PER_RAD);

                // Controller acts on TRUE state so the trajectory is fixed for the measurement.
                double trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD;
                double trueVelTps = plant.getTrueAngularVelocityRadPerSec() * TICKS_PER_RAD;
                power =
                        0.004 * (targetRad * TICKS_PER_RAD - trueTicks)
                                - 0.0008 * trueVelTps
                                + (K_G / VOLTAGE) * Math.cos(plant.getTrueAngleRad());
                power = Math.max(-1.0, Math.min(1.0, power));

                double trueVelRad = plant.getTrueAngularVelocityRadPerSec();
                double velErr = ekf.getVelocity() - trueVelRad;

                double t = ms / 1000.0;
                if (t > 0.2) {
                    ekfVelSqAll += velErr * velErr;
                    ekfAngleSq += sq(ekf.getPosition() - plant.getTrueAngleRad());
                    allSamples++;
                    if (Math.abs(trueVelRad) > 0.5) {
                        ekfVelSqMoving += velErr * velErr;
                        movingSamples++;
                    }
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double velRmsAll = Math.sqrt(ekfVelSqAll / allSamples);
        double velRmsMoving = Math.sqrt(ekfVelSqMoving / movingSamples);
        double angleRms = Math.sqrt(ekfAngleSq / allSamples);

        assertTrue("velocity RMS too high: " + velRmsAll + " rad/s", velRmsAll < 0.04);
        assertTrue(
                "velocity RMS while moving too high: " + velRmsMoving + " rad/s",
                velRmsMoving < 0.07);
        assertTrue("angle RMS too high: " + angleRms + " rad", angleRms < 0.02);
    }

    private static double sq(double v) {
        return v * v;
    }
}
