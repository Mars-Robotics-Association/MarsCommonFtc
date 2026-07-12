package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Focused tests for {@link MotorMechanismEkf} behaviors not covered by the closed-loop sweeps,
 * notably saturation awareness in the predict step. Uses a {@link LiftModel} (constant gravity, so
 * the EKF is linear and the arithmetic is easy to reason about).
 */
public class MotorMechanismEkfTest {

    private static final double K_S = 0.1, K_G = 2.0, K_V = 0.006, K_A = 0.0016;

    private static MotorMechanismEkf newEkf() {
        return new MotorMechanismEkf(
                new LiftModel(K_S, K_V, K_A, K_G),
                /* velocityLagSec= */ 0.025,
                /* modelAccelStdDev= */ 1500.0,
                /* positionStdDev= */ 1.0,
                /* velocityStdDev= */ 30.0,
                /* positionTimingJitterStdDev= */ 0.0,
                /* initialPosition= */ 0.0);
    }

    @Test
    public void overRangePowerClampsToRail() {
        double dt = 0.02;
        double busVoltage = 12.0;

        // An over-command (power 100) cannot outrun full power (1.0): both apply 12 V, so the
        // filter
        // predicts identical motion. Saturation awareness means an over-command can never inflate
        // the
        // estimate the way trusting the raw request would.
        MotorMechanismEkf over = newEkf();
        over.predict(dt, 100.0, busVoltage);

        MotorMechanismEkf full = newEkf();
        full.predict(dt, 1.0, busVoltage);

        assertEquals(full.getPosition(), over.getPosition(), 1e-9);
        assertEquals(full.getVelocity(), over.getVelocity(), 1e-9);
        // Guard against a vacuous pass: full power should actually move the mechanism.
        assertTrue(
                "full power should produce motion, got " + full.getVelocity(),
                full.getVelocity() > 0.0);

        // The clamp is symmetric: an over-command the other way matches full reverse power.
        MotorMechanismEkf overReverse = newEkf();
        overReverse.predict(dt, -100.0, busVoltage);
        MotorMechanismEkf fullReverse = newEkf();
        fullReverse.predict(dt, -1.0, busVoltage);
        assertEquals(fullReverse.getVelocity(), overReverse.getVelocity(), 1e-9);
    }

    @Test
    public void appliedVoltageIsPowerTimesBusVoltage() {
        double dt = 0.02;

        // Half power on a 12 V bus is the same 6 V the motor would see at full power on a 6 V bus,
        // so
        // the prediction must match — confirming the power-times-bus scaling.
        MotorMechanismEkf halfOnFull = newEkf();
        halfOnFull.predict(dt, 0.5, 12.0);

        MotorMechanismEkf fullOnHalf = newEkf();
        fullOnHalf.predict(dt, 1.0, 6.0);

        assertEquals(fullOnHalf.getPosition(), halfOnFull.getPosition(), 1e-12);
        assertEquals(fullOnHalf.getVelocity(), halfOnFull.getVelocity(), 1e-12);
    }
}
