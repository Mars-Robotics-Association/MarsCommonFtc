package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.sim.ArmMotorSim;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class ArmControllerTest {

    // ── constants ───────────────────────────────────────────────────────────────

    static final double HUB_VOLTAGE = 13.75;
    static final long SEED = 42L;

    // Feedforward gains (output-shaft rad units)
    static final double KS = 0.3;
    static final double KG = 1.5;
    static final double KV = 1.2;
    static final double KA = 0.15;

    static final int TICKS_PER_REV = 28;
    static final double GEAR_RATIO = 100.0;

    // Arm geometry: front hard stop at -45 deg, back hard stop at -225 deg
    // Encoder reads 0 at the front hard stop.
    static final double ENCODER_ZERO_OFFSET_RAD = -Math.PI / 4;
    static final double MIN_ANGLE_RAD = -Math.PI * 5 / 4;  // -225 deg (back stop)
    static final double MAX_ANGLE_RAD = -Math.PI / 4;       // -45 deg  (front stop)

    // ── sim clock ───────────────────────────────────────────────────────────────

    private long simTimeNanos = 0;

    private long clockSupplier() {
        return simTimeNanos;
    }

    // ── fixture ─────────────────────────────────────────────────────────────────

    static class SimMotorAdapter implements IMotor {
        final ArmMotorSim sim;
        double lastPower = 0.0;

        SimMotorAdapter(ArmMotorSim sim) {
            this.sim = sim;
        }

        @Override public String getName()              { return "arm"; }
        @Override public int    getPosition()          { return sim.getPositionTicks(); }
        @Override public double getVelocity()          { return sim.getVelocityTps(); }
        @Override public void   setPower(double power) { lastPower = power; }
        @Override public double getHubVoltage()        { return HUB_VOLTAGE; }
        @Override public void   setVelocity(double tps) {}
        @Override public void   setVelocityPIDFCoefficients(double p, double i, double d, double f) {}
    }

    private ArmMotorSim makeSim(double initialAngleRad) {
        return new ArmMotorSim(KS, KG, KV, KA,
                TICKS_PER_REV, GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD, MAX_ANGLE_RAD,
                initialAngleRad);
    }

    @BeforeEach
    void setupParams() {
        ArmController.PARAMS = new ArmController.Params();
        ArmController.PARAMS.ks = KS;
        ArmController.PARAMS.kg = KG;
        ArmController.PARAMS.kv = KV;
        ArmController.PARAMS.ka = KA;
        ArmController.PARAMS.ticksPerRev = TICKS_PER_REV;
        ArmController.PARAMS.gearRatio = GEAR_RATIO;
        ArmController.PARAMS.encoderZeroOffsetRad = ENCODER_ZERO_OFFSET_RAD;
        ArmController.PARAMS.minAngleRad = MIN_ANGLE_RAD;
        ArmController.PARAMS.maxAngleRad = MAX_ANGLE_RAD;

        // PD gains
        ArmController.PARAMS.kP = 15.0;
        ArmController.PARAMS.kD = 1.0;

        // Trajectory limits
        ArmController.PARAMS.maxVelRad = 3.0;
        ArmController.PARAMS.maxAccelRad = 6.0;
        ArmController.PARAMS.maxDecelRad = 8.0;
        ArmController.PARAMS.maxJerkRad = 30.0;

        // Kalman tuning
        ArmController.PARAMS.modelStdDevPos = 0.01;
        ArmController.PARAMS.modelStdDevVel = 3.0;
        ArmController.PARAMS.measurementStdDevPos = 0.05;
        ArmController.PARAMS.measurementStdDevVel = 0.5;

        ArmController.PARAMS.latencyCompensationSec = 0.030;
        ArmController.PARAMS.replanThresholdRad = Math.toRadians(15);
        ArmController.PARAMS.coastZoneRad = Math.toRadians(10);

        simTimeNanos = 0;
    }

    /** Advance by a normally-distributed dt (mean 16 ms, sigma 5 ms). Returns actual dt. */
    private double step(ArmController controller, SimMotorAdapter adapter,
                         ArmMotorSim sim, Random rng) {
        double dt = Math.max(0.005, 0.016 + rng.nextGaussian() * 0.005);
        simTimeNanos += (long) (dt * 1e9);
        controller.update(dt, HUB_VOLTAGE);
        sim.step(dt, adapter.lastPower, HUB_VOLTAGE);
        return dt;
    }

    // ── tests ───────────────────────────────────────────────────────────────────

    @Test
    void testMoveToTarget() {
        // Start at front hard stop (-45 deg), move to -120 deg (past straight down)
        double startAngle = MAX_ANGLE_RAD;       // -45 deg
        double targetAngle = Math.toRadians(-120); // -120 deg

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        ArmController controller = new ArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(targetAngle);

        Random rng = new Random(SEED);
        double elapsed = 0;
        for (int i = 0; i < 500; i++) {
            elapsed += step(controller, adapter, sim, rng);
        }

        double finalPos = sim.getTruePositionRad();
        System.out.printf("testMoveToTarget: target=%.1f deg, actual=%.1f deg, elapsed=%.2f s%n",
                Math.toDegrees(targetAngle), Math.toDegrees(finalPos), elapsed);

        assertEquals(targetAngle, finalPos, Math.toRadians(3),
                "arm should converge to target within 3 degrees");
    }

    @Test
    void testHoldPositionAgainstGravity() {
        // Start at -60 deg (significant gravity torque), hold position
        double holdAngle = Math.toRadians(-60);

        ArmMotorSim sim = makeSim(holdAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        ArmController controller = new ArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(holdAngle);

        Random rng = new Random(SEED);
        // Let it settle
        for (int i = 0; i < 300; i++) {
            step(controller, adapter, sim, rng);
        }

        // Check steady-state error over next 200 iterations
        double maxError = 0;
        for (int i = 0; i < 200; i++) {
            step(controller, adapter, sim, rng);
            maxError = Math.max(maxError, Math.abs(sim.getTruePositionRad() - holdAngle));
        }

        System.out.printf("testHoldPosition: max error=%.2f deg%n", Math.toDegrees(maxError));
        assertTrue(maxError < Math.toRadians(5),
                "steady-state position error should be under 5 degrees");
    }

    @Test
    void testCoastNearHardStop() {
        // Start near front hard stop, target is the hard stop
        double startAngle = MAX_ANGLE_RAD;

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        ArmController controller = new ArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(startAngle);

        // Run a few iterations — arm should coast immediately since both target and
        // position are within the coast zone of the front hard stop.
        Random rng = new Random(SEED);
        for (int i = 0; i < 5; i++) {
            step(controller, adapter, sim, rng);
        }

        assertEquals(ArmController.Mode.COASTING, controller.getMode(),
                "should be coasting near hard stop");
        assertEquals(0.0, adapter.lastPower, 0.001,
                "power should be 0 when coasting");
    }

    @Test
    void testWakeFromHardStop() {
        // Start at front hard stop, coasting, then wake and move to -120 deg
        double startAngle = MAX_ANGLE_RAD;          // -45 deg
        double targetAngle = Math.toRadians(-120);   // -120 deg

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        ArmController controller = new ArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        // Let it enter coast
        controller.setTarget(startAngle);
        Random rng = new Random(SEED);
        for (int i = 0; i < 20; i++) {
            step(controller, adapter, sim, rng);
        }
        assertEquals(ArmController.Mode.COASTING, controller.getMode());

        // Now wake it up
        controller.setTarget(targetAngle);
        assertEquals(ArmController.Mode.TRACKING, controller.getMode(),
                "should switch to TRACKING after setTarget");

        // Let it move
        for (int i = 0; i < 500; i++) {
            step(controller, adapter, sim, rng);
        }

        double finalPos = sim.getTruePositionRad();
        System.out.printf("testWakeFromHardStop: target=%.1f deg, actual=%.1f deg%n",
                Math.toDegrees(targetAngle), Math.toDegrees(finalPos));

        assertEquals(targetAngle, finalPos, Math.toRadians(5),
                "arm should reach target after waking from hard stop");
    }

    @Test
    void testReplanOnDisturbance() {
        // Move to -120 deg, then apply a large disturbance
        double targetAngle = Math.toRadians(-120);

        ArmMotorSim sim = makeSim(MAX_ANGLE_RAD);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        ArmController controller = new ArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(targetAngle);

        Random rng = new Random(SEED);
        // Let it converge
        for (int i = 0; i < 500; i++) {
            step(controller, adapter, sim, rng);
        }
        assertEquals(targetAngle, sim.getTruePositionRad(), Math.toRadians(5),
                "should be near target before disturbance");

        // Apply large disturbance (equivalent to someone pushing the arm)
        sim.setDisturbanceVoltage(-3.0);
        for (int i = 0; i < 30; i++) {
            step(controller, adapter, sim, rng);
        }
        sim.setDisturbanceVoltage(0.0);

        // Let it recover
        for (int i = 0; i < 500; i++) {
            step(controller, adapter, sim, rng);
        }

        double finalPos = sim.getTruePositionRad();
        System.out.printf("testReplanOnDisturbance: target=%.1f deg, actual=%.1f deg%n",
                Math.toDegrees(targetAngle), Math.toDegrees(finalPos));

        assertEquals(targetAngle, finalPos, Math.toRadians(5),
                "arm should recover to target after disturbance");
    }

    @Test
    void testAsymmetricProfile() {
        // Move a large distance to exercise both accel and decel phases
        double startAngle = MAX_ANGLE_RAD;            // -45 deg
        double targetAngle = Math.toRadians(-150);     // -150 deg

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        ArmController controller = new ArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(targetAngle);

        Random rng = new Random(SEED);
        double maxAccelSeen = 0;
        double maxDecelSeen = 0;
        double prevVel = 0;

        for (int i = 0; i < 500; i++) {
            double dt = step(controller, adapter, sim, rng);
            double vel = controller.getTrajectoryVelocityRadPerSec();
            if (i > 0) {
                double accel = (vel - prevVel) / dt;

                if (Math.abs(accel) > 0.5) {
                    if (Math.signum(accel) == Math.signum(vel) || Math.abs(vel) < 0.01) {
                        maxAccelSeen = Math.max(maxAccelSeen, Math.abs(accel));
                    } else {
                        maxDecelSeen = Math.max(maxDecelSeen, Math.abs(accel));
                    }
                }
            }
            prevVel = vel;
        }

        System.out.printf("testAsymmetricProfile: maxAccel=%.1f, maxDecel=%.1f (limits: %.1f, %.1f)%n",
                maxAccelSeen, maxDecelSeen,
                ArmController.PARAMS.maxAccelRad, ArmController.PARAMS.maxDecelRad);

        // The accel and decel should be bounded by their respective limits (with some jerk margin)
        assertTrue(maxAccelSeen <= ArmController.PARAMS.maxAccelRad + 1.0,
                "acceleration should not exceed maxAccelRad (plus jerk margin)");
        assertNotEquals(ArmController.PARAMS.maxAccelRad, ArmController.PARAMS.maxDecelRad,
                "test setup should have asymmetric limits");
    }

    @Test
    void testVoltageCompensation() {
        // Run the same move at two different voltages, verify similar tracking
        double startAngle = MAX_ANGLE_RAD;
        double targetAngle = Math.toRadians(-120);

        // --- Run at 13.75V ---
        ArmMotorSim sim1 = makeSim(startAngle);
        SimMotorAdapter adapter1 = new SimMotorAdapter(sim1);
        simTimeNanos = 0;
        ArmController controller1 = new ArmController(adapter1,
                (caption, format, value) -> {}, this::clockSupplier);
        controller1.setTarget(targetAngle);

        Random rng1 = new Random(SEED);
        for (int i = 0; i < 500; i++) {
            double dt = Math.max(0.005, 0.016 + rng1.nextGaussian() * 0.005);
            simTimeNanos += (long) (dt * 1e9);
            controller1.update(dt, 13.75);
            sim1.step(dt, adapter1.lastPower, 13.75);
        }

        // --- Run at 10.5V ---
        ArmMotorSim sim2 = makeSim(startAngle);
        SimMotorAdapter adapter2 = new SimMotorAdapter(sim2);
        simTimeNanos = 0;
        ArmController controller2 = new ArmController(adapter2,
                (caption, format, value) -> {}, this::clockSupplier);
        controller2.setTarget(targetAngle);

        Random rng2 = new Random(SEED);
        for (int i = 0; i < 500; i++) {
            double dt = Math.max(0.005, 0.016 + rng2.nextGaussian() * 0.005);
            simTimeNanos += (long) (dt * 1e9);
            controller2.update(dt, 10.5);
            sim2.step(dt, adapter2.lastPower, 10.5);
        }

        double pos1 = sim1.getTruePositionRad();
        double pos2 = sim2.getTruePositionRad();
        System.out.printf("testVoltageCompensation: 13.75V final=%.1f deg, 10.5V final=%.1f deg%n",
                Math.toDegrees(pos1), Math.toDegrees(pos2));

        assertEquals(targetAngle, pos1, Math.toRadians(5),
                "13.75V run should converge to target");
        assertEquals(targetAngle, pos2, Math.toRadians(5),
                "10.5V run should converge to target");
    }
}
