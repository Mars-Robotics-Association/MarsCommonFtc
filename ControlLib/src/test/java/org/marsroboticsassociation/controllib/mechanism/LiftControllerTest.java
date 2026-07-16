package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * Closed-loop test of a {@link MotorMechanismController} (built on a {@link LiftModel}) driving
 * {@link LiftPlantSim}, with a {@link MotorMechanismEkf} on the same model estimating the state.
 * Besides reaching and holding the target, this exercises the back-EMF limits: the configured
 * velocity limit (2500) is set higher than the motor can sustain, so the controller throttles the
 * cruise speed down toward the voltage ceiling {@code (12 - margin - kS - kG)/kV ~= 1400} instead.
 * The velocity-dependent acceleration ceiling does most of that throttling on the way up.
 */
public class LiftControllerTest {

    private static final double K_S = 0.1, K_G = 2.0, K_V = 0.006, K_A = 0.0016;
    private static final double VOLTAGE = 12.0;
    private static final double FEEDBACK_MARGIN = 1.5;
    private static final double CONFIGURED_MAX_VELOCITY = 2500.0;
    // The cruise ceiling climbing, once the feedback margin is held back from the feedforward.
    // Gravity opposes the motion, so it is subtracted.
    private static final double BACK_EMF_CEILING =
            (VOLTAGE - FEEDBACK_MARGIN - K_S - K_G) / K_V; // ~1400 TPS
    // The cruise ceiling descending: gravity aids the motion, so it is added instead.
    private static final double DESCENT_CEILING =
            (VOLTAGE - FEEDBACK_MARGIN - K_S + K_G) / K_V; // ~2067 TPS

    @Test
    public void reachesTargetAndObeysBackEmfVelocityCeiling() {
        double start = 0.0;
        double target = 1500.0;

        LiftPlantSim plant = new LiftPlantSim(K_S, K_G, K_V, K_A, -100, 3000, start);
        LiftModel model = new LiftModel(K_S, K_V, K_A, K_G);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 1500.0, 1.0, 30.0, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model,
                        /* kP= */ 0.10,
                        /* kI= */ 0.15,
                        /* kD= */ 0.02,
                        CONFIGURED_MAX_VELOCITY,
                        /* maxAcceleration= */ 6000.0,
                        /* maxJerk= */ 40000.0,
                        FEEDBACK_MARGIN,
                        start);

        Random rng = new Random(3L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double peakVelocity = 0.0;
        double maxVoltage = 0.0;
        double settleMs = -1.0;

        for (int ms = 0; ms <= 3000; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(plant.getEncoderPosition(), plant.getEncoderVelocityTps());

                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                maxVoltage = Math.max(maxVoltage, Math.abs(voltage));
                power = voltage / VOLTAGE;

                double truePos = plant.getTruePositionTicks();
                peakVelocity = Math.max(peakVelocity, Math.abs(plant.getTrueVelocityTps()));
                if (settleMs < 0 && Math.abs(truePos - target) < 10.0) {
                    settleMs = ms;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double finalPos = plant.getTruePositionTicks();

        assertTrue("did not reach target, final " + finalPos, Math.abs(finalPos - target) < 25.0);
        assertTrue("settled too slowly: " + settleMs + " ms", settleMs >= 0 && settleMs < 2000.0);
        assertTrue("voltage exceeded the clamp: " + maxVoltage, maxVoltage <= VOLTAGE + 1e-6);
        // The back-EMF ceiling, not the configured limit, governed the cruise speed.
        assertTrue(
                "exceeded the back-EMF ceiling: " + peakVelocity,
                peakVelocity < BACK_EMF_CEILING * 1.05);
        assertTrue(
                "back-EMF ceiling did not actually bind: " + peakVelocity,
                peakVelocity < 0.8 * CONFIGURED_MAX_VELOCITY
                        && peakVelocity > 0.85 * BACK_EMF_CEILING);
    }

    @Test
    public void descendsFasterThanItCouldClimb() {
        // Same lift, but now dropping from the top to near the bottom. Gravity aids the descent, so
        // the direction-aware ceiling lets the setpoint run past the climb ceiling it could never
        // exceed going up.
        double start = 2000.0;
        double target = 200.0;

        LiftPlantSim plant = new LiftPlantSim(K_S, K_G, K_V, K_A, -100, 3000, start);
        LiftModel model = new LiftModel(K_S, K_V, K_A, K_G);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 1500.0, 1.0, 30.0, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model,
                        /* kP= */ 0.10,
                        /* kI= */ 0.15,
                        /* kD= */ 0.02,
                        CONFIGURED_MAX_VELOCITY,
                        /* maxAcceleration= */ 6000.0,
                        /* maxJerk= */ 40000.0,
                        FEEDBACK_MARGIN,
                        start);

        Random rng = new Random(5L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double peakSpeed = 0.0;
        double maxVoltage = 0.0;

        for (int ms = 0; ms <= 4000; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(plant.getEncoderPosition(), plant.getEncoderVelocityTps());

                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                maxVoltage = Math.max(maxVoltage, Math.abs(voltage));
                power = voltage / VOLTAGE;

                peakSpeed = Math.max(peakSpeed, Math.abs(plant.getTrueVelocityTps()));

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double finalPos = plant.getTruePositionTicks();

        assertTrue("did not reach target, final " + finalPos, Math.abs(finalPos - target) < 25.0);
        assertTrue("voltage exceeded the clamp: " + maxVoltage, maxVoltage <= VOLTAGE + 1e-6);
        // The whole point of charging gravity in the direction of travel: the descent runs past
        // the climb ceiling, where worst-case gravity charging would clamp it.
        assertTrue(
                "descent did not exploit gravity, peak " + peakSpeed,
                peakSpeed > 1.05 * BACK_EMF_CEILING);
        // But still within the descent ceiling, where gravity assist runs out.
        assertTrue(
                "exceeded the descent ceiling: " + peakSpeed, peakSpeed < DESCENT_CEILING * 1.05);
    }
}
