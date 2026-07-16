package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Asymmetric configured accel/decel caps: a mechanism may be allowed to launch harder than it
 * stops (maxAcceleration > maxDeceleration), e.g. a flexible arm whose arrival swing constrains
 * braking but not starting. Verifies the configured deceleration cap — not the acceleration cap —
 * bounds the braking phase, both through the convenience constructor (which builds the
 * model-aware Ruckig profiler itself) and with a caller-supplied profiler.
 *
 * <p>The model is deliberately weak (tiny kV/kA, no gravity) so its back-EMF ceilings sit far
 * above the configured caps and the configured caps are what bind.
 */
public class AsymmetricAccelLimitTest {

    private static final double MAX_VEL = 2.0;
    private static final double MAX_ACCEL = 12.0;
    private static final double MAX_DECEL = 4.0;
    private static final double MAX_JERK = 1000.0;
    private static final double VOLTAGE = 12.0;
    private static final double TARGET = 2.0;
    private static final double DT = 0.005;

    @Test
    public void convenienceConstructorBrakesAtDecelCapAndLaunchesAboveIt() {
        ArmModel model = new ArmModel(0.05, 0.1, 0.1, /* kCos= */ 0.0, /* kSin= */ 0.0);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model, 40, 8, 1.5,
                        MAX_VEL, MAX_ACCEL, MAX_DECEL, MAX_JERK,
                        /* feedbackVoltageMargin= */ 1.5, /* initialPosition= */ 0.0);
        assertAsymmetricShaping(controller);
    }

    @Test
    public void suppliedProfilerBrakesAtDecelCapAndLaunchesAboveIt() {
        ArmModel model = new ArmModel(0.05, 0.1, 0.1, /* kCos= */ 0.0, /* kSin= */ 0.0);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model, 40, 8, 1.5,
                        /* feedbackVoltageMargin= */ 1.5,
                        new ModelAwareRuckigProfiler(
                                model, MAX_VEL, MAX_ACCEL, MAX_DECEL, MAX_JERK, 0.0));
        assertAsymmetricShaping(controller);
    }

    /**
     * Drive the controller with a perfect measurement (the profile's own state) and check the
     * setpoint trajectory: speeding up may exceed the decel cap (proving the launch is not pinned
     * to it), braking never does.
     */
    private static void assertAsymmetricShaping(MotorMechanismController controller) {
        double peakSpeedUpAccel = 0.0;
        double peakBrakingAccel = 0.0;
        for (int i = 0; i < 4000; i++) {
            controller.calculate(
                    TARGET,
                    controller.getSetpointPosition(),
                    controller.getSetpointVelocity(),
                    VOLTAGE,
                    DT);
            double v = controller.getSetpointVelocity();
            double a = controller.getSetpointAcceleration();
            if (v * a > 0) {
                peakSpeedUpAccel = Math.max(peakSpeedUpAccel, Math.abs(a));
            } else if (v * a < 0) {
                peakBrakingAccel = Math.max(peakBrakingAccel, Math.abs(a));
            }
        }
        assertEquals("did not land on the target", TARGET, controller.getSetpointPosition(), 1e-6);
        assertTrue(
                "launch never used the headroom above the decel cap: " + peakSpeedUpAccel,
                peakSpeedUpAccel > MAX_DECEL * 1.5);
        assertTrue(
                "launch exceeded the accel cap: " + peakSpeedUpAccel,
                peakSpeedUpAccel <= MAX_ACCEL + 1e-6);
        assertTrue("braking never approached its cap: " + peakBrakingAccel,
                peakBrakingAccel > MAX_DECEL * 0.9);
        assertTrue(
                "braking exceeded the decel cap: " + peakBrakingAccel,
                peakBrakingAccel <= MAX_DECEL + 1e-6);
    }
}
