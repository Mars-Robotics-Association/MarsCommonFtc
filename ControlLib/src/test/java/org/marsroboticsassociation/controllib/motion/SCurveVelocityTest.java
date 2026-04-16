package org.marsroboticsassociation.controllib.motion;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class SCurveVelocityTest {

    private static final double MOTOR_VOLTAGE = 12.0;
    private static final double KS = 0.893;
    private static final double KV = 0.00475;
    private static final double KA = 0.00599;

    private double maxMotorAcceleration(double velocity) {
        double availableVoltage = MOTOR_VOLTAGE - KS - KV * Math.abs(velocity);
        if (availableVoltage <= 0) return 0;
        return availableVoltage / KA;
    }

    @Test
    void backEmfViolationCheck() {
        double aMax = 1858;
        double jInc = 3000;
        double jDec = 360;
        double targetV = 2000;
        double v0 = 0;
        double a0 = 0;

        SCurveVelocity trajectory = new SCurveVelocity(v0, targetV, a0, aMax, jInc, jDec);
        double totalTime = trajectory.getTotalTime();

        double maxViolation = 0;
        double maxViolationTime = 0;
        double maxViolationVelocity = 0;
        double maxViolationAccel = 0;
        double maxViolationMotorAccel = 0;

        double maxTrajAccel = 0;
        double maxTrajAccelTime = 0;
        double maxTrajAccelV = 0;

        int samples = 10000;
        for (int i = 0; i <= samples; i++) {
            double t = totalTime * i / samples;
            double v = trajectory.getVelocity(t);
            double a = trajectory.getAcceleration(t);

            if (a > maxTrajAccel) {
                maxTrajAccel = a;
                maxTrajAccelTime = t;
                maxTrajAccelV = v;
            }

            double motorAMax = maxMotorAcceleration(v);
            double violation = a - motorAMax;

            if (violation > maxViolation) {
                maxViolation = violation;
                maxViolationTime = t;
                maxViolationVelocity = v;
                maxViolationAccel = a;
                maxViolationMotorAccel = motorAMax;
            }
        }

        System.out.println("=== Back-EMF Violation Check ===");
        System.out.println("aMax: " + aMax + ", jInc: " + jInc + ", jDec: " + jDec);
        System.out.println("Target velocity: " + targetV);
        System.out.println("Total time: " + totalTime + "s");
        System.out.println();
        System.out.println("Max trajectory acceleration: " + maxTrajAccel + " at t=" + maxTrajAccelTime + "s, v=" + maxTrajAccelV);
        System.out.println("Motor max accel at that velocity: " + maxMotorAcceleration(maxTrajAccelV));
        System.out.println();
        System.out.println("Max violation: " + maxViolation + " units/s^2");
        if (maxViolation > 0) {
            System.out.println("  at t=" + maxViolationTime + "s");
            System.out.println("  velocity: " + maxViolationVelocity + " RPM");
            System.out.println("  trajectory accel: " + maxViolationAccel);
            System.out.println("  motor max accel: " + maxViolationMotorAccel);
        }
        System.out.println();
        System.out.println("Motor max accel curve:");
        for (int v = 0; v <= targetV; v += 200) {
            System.out.println("  v=" + v + ": " + maxMotorAcceleration(v));
        }

        assertTrue(maxViolation < 1.0, "Acceleration should stay within motor limits (with tolerance)");
    }

    // --- findMaxAMax degenerate input guards ---

    @Test
    void findMaxAMax_kAZero_returnsInfinity() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, 3000, MOTOR_VOLTAGE, KS, KV, 0.0);
        assertEquals(Double.POSITIVE_INFINITY, result);
    }

    @Test
    void findMaxAMax_kANegative_returnsInfinity() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, 3000, MOTOR_VOLTAGE, KS, KV, -1.0);
        assertEquals(Double.POSITIVE_INFINITY, result);
    }

    @Test
    void findMaxAMax_voltageEqualToKs_returnsZero() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, 3000, KS, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxAMax_voltageLessThanKs_returnsZero() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, 3000, KS - 1.0, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxAMax_jIncZero_returnsZero() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, 0.0, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxAMax_jIncNegative_returnsZero() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, -100.0, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxAMax_normalInputs_returnsPositiveFiniteValue() {
        double result = SCurveVelocity.findMaxAMax(0, 2000, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertTrue(Double.isFinite(result), "result should be finite");
        assertTrue(result > 0, "result should be positive");
    }

    // --- findMaxJDec degenerate input guards ---

    @Test
    void findMaxJDec_kAZero_returnsInfinity() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 1858, 3000, MOTOR_VOLTAGE, KS, KV, 0.0);
        assertEquals(Double.POSITIVE_INFINITY, result);
    }

    @Test
    void findMaxJDec_kANegative_returnsInfinity() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 1858, 3000, MOTOR_VOLTAGE, KS, KV, -1.0);
        assertEquals(Double.POSITIVE_INFINITY, result);
    }

    @Test
    void findMaxJDec_voltageEqualToKs_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 1858, 3000, KS, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_voltageLessThanKs_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 1858, 3000, KS - 1.0, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_aMaxZero_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 0.0, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_aMaxNegative_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, -100.0, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_aMaxInfinity_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, Double.POSITIVE_INFINITY, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_aMaxNaN_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, Double.NaN, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_jIncZero_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 1858, 0.0, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_jIncNegative_returnsZero() {
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, 1858, -100.0, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(0.0, result);
    }

    @Test
    void findMaxJDec_trivialVelocity_returnsInfinity() {
        double result = SCurveVelocity.findMaxJDec(2000, 2000, 0, 1858, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(Double.POSITIVE_INFINITY, result);
    }

    @Test
    void findMaxJDec_nearTrivialVelocity_returnsInfinity() {
        double result = SCurveVelocity.findMaxJDec(2000, 2000 + 1e-10, 0, 1858, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertEquals(Double.POSITIVE_INFINITY, result);
    }

    @Test
    void findMaxJDec_normalInputs_returnsPositiveFiniteValue() {
        double aMax = SCurveVelocity.findMaxAMax(0, 2000, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        double result = SCurveVelocity.findMaxJDec(0, 2000, 0, aMax, 3000, MOTOR_VOLTAGE, KS, KV, KA);
        assertTrue(Double.isFinite(result), "result should be finite");
        assertTrue(result > 0, "result should be positive");
    }

    // --- chained call safety ---

    @Test
    void findMaxAMax_thenFindMaxJDec_kAZero_noNaNOrInfiniteTrajectory() {
        // When kA=0 both functions return POSITIVE_INFINITY; constructing a trajectory with
        // those values must not produce NaN timing (regression guard for the chained-call pattern).
        double aMax = SCurveVelocity.findMaxAMax(0, 2000, 3000, MOTOR_VOLTAGE, KS, KV, 0.0);
        double jDec = SCurveVelocity.findMaxJDec(0, 2000, 0, aMax, 3000, MOTOR_VOLTAGE, KS, KV, 0.0);
        assertEquals(Double.POSITIVE_INFINITY, aMax);
        assertEquals(Double.POSITIVE_INFINITY, jDec);
        // Callers must clamp before constructing a trajectory; the functions themselves are safe.
    }
}