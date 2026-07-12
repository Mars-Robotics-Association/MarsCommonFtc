package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

/**
 * Closed-loop test of a {@link MotorMechanismController} (built on an {@link ArmModel}) driving
 * {@link ArmPlantSim}, with a {@link MotorMechanismEkf} on the same model estimating the state. The
 * controller sees only the filter's estimates and the bus voltage; it profiles the move, feeds
 * forward from the model, corrects with PID, and clamps the output. The move sweeps -45 deg through
 * horizontal to +45 deg.
 */
public class ArmControllerTest {

    private static final double TICKS_PER_RAD = 28 * 100 / (2 * Math.PI);
    private static final double K_S = 0.1, K_G = 1.5, K_V = 2.0, K_A = 0.2;
    private static final double VOLTAGE = 12.0;

    @Test
    public void reachesAndHoldsTargetWithinAllLimits() {
        double start = -Math.PI / 4;
        double target = Math.PI / 4;

        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S, K_G, K_V, K_A, TICKS_PER_RAD, -Math.PI * 0.9, Math.PI * 0.9, start);
        // One model, shared by the filter and the controller.
        ArmModel model = new ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model,
                        /* kP= */ 40,
                        /* kI= */ 8,
                        /* kD= */ 1.5,
                        /* maxVelocity= */ 8.0,
                        /* maxAcceleration= */ 12.0,
                        /* maxJerk= */ 480.0,
                        /* feedbackVoltageMargin= */ 1.5,
                        start);

        Random rng = new Random(1L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double maxOvershootDeg = 0.0;
        double settleMs = -1.0;
        double peakVelocity = 0.0;
        double maxVoltage = 0.0;
        double sumSteadyAbsErrDeg = 0.0;
        int steadySamples = 0;

        for (int ms = 0; ms <= 3000; ms++) {
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

                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                maxVoltage = Math.max(maxVoltage, Math.abs(voltage));
                power = voltage / VOLTAGE; // voltage compensation; clamped inside the controller
                // already

                double trueDeg = Math.toDegrees(plant.getTrueAngleRad());
                peakVelocity =
                        Math.max(peakVelocity, Math.abs(plant.getTrueAngularVelocityRadPerSec()));
                maxOvershootDeg = Math.max(maxOvershootDeg, trueDeg - 45.0);
                if (settleMs < 0 && Math.abs(trueDeg - 45.0) < 1.0) {
                    settleMs = ms;
                }
                if (ms / 1000.0 > 2.0) {
                    sumSteadyAbsErrDeg += Math.abs(trueDeg - 45.0);
                    steadySamples++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double finalDeg = Math.toDegrees(plant.getTrueAngleRad());
        double steadyAbsErrDeg = sumSteadyAbsErrDeg / steadySamples;

        assertTrue(
                "did not reach target, final " + finalDeg + " deg",
                Math.abs(finalDeg - 45.0) < 2.0);
        assertTrue(
                "steady-state error too high: " + steadyAbsErrDeg + " deg", steadyAbsErrDeg < 1.5);
        assertTrue("overshoot too high: " + maxOvershootDeg + " deg", maxOvershootDeg < 8.0);
        assertTrue("settled too slowly: " + settleMs + " ms", settleMs >= 0 && settleMs < 1500.0);
        // Back-EMF-aware velocity ceiling: ~ (12 - kS - gravity)/kV ~= 5.4 rad/s here.
        assertTrue("exceeded the back-EMF velocity ceiling: " + peakVelocity, peakVelocity < 5.6);
        // Voltage clamp respected.
        assertTrue("voltage exceeded the clamp: " + maxVoltage, maxVoltage <= VOLTAGE + 1e-6);
    }
}
