package org.marsroboticsassociation.controllib.control;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.sim.ArmMotorSim;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class VerticalArmControllerTest {

    // ── constants ───────────────────────────────────────────────────────────────

    static final double HUB_VOLTAGE = 13.75;
    static final long SEED = 42L;

    static final double KS = 0.3;
    static final double KG = 1.5;
    static final double KV = 1.2;
    static final double KA = 0.15;

    static final int TICKS_PER_REV = 28;
    static final double GEAR_RATIO = 100.0;

    static final double ENCODER_ZERO_OFFSET_RAD = -Math.PI / 4;
    static final double MIN_ANGLE_RAD = -Math.PI * 5 / 4;
    static final double MAX_ANGLE_RAD = -Math.PI / 4;

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
        VerticalArmController.PARAMS = new VerticalArmController.Params();
        VerticalArmController.PARAMS.ks = KS;
        VerticalArmController.PARAMS.kg = KG;
        VerticalArmController.PARAMS.kv = KV;
        VerticalArmController.PARAMS.ka = KA;
        VerticalArmController.PARAMS.ticksPerRev = TICKS_PER_REV;
        VerticalArmController.PARAMS.gearRatio = GEAR_RATIO;
        VerticalArmController.PARAMS.encoderZeroOffsetRad = ENCODER_ZERO_OFFSET_RAD;
        VerticalArmController.PARAMS.minAngleRad = MIN_ANGLE_RAD;
        VerticalArmController.PARAMS.maxAngleRad = MAX_ANGLE_RAD;

        // LQR cost weights — targeting K ≈ [15, 1] to match existing PD behavior
        VerticalArmController.PARAMS.qPosition = 0.5;
        VerticalArmController.PARAMS.qVelocity = 5.0;
        VerticalArmController.PARAMS.rVoltage = 12.0;

        // Trajectory limits
        VerticalArmController.PARAMS.maxVelRad = 3.0;
        VerticalArmController.PARAMS.maxAccelRad = 6.0;
        VerticalArmController.PARAMS.maxDecelRad = 8.0;
        VerticalArmController.PARAMS.maxJerkRad = 30.0;

        // Kalman tuning
        VerticalArmController.PARAMS.modelStdDevPos = 0.01;
        VerticalArmController.PARAMS.modelStdDevVel = 3.0;
        VerticalArmController.PARAMS.measurementStdDevPos = 0.05;
        VerticalArmController.PARAMS.measurementStdDevVel = 0.5;

        VerticalArmController.PARAMS.latencyCompensationSec = 0.030;
        VerticalArmController.PARAMS.replanThresholdRad = Math.toRadians(15);
        VerticalArmController.PARAMS.coastZoneRad = Math.toRadians(10);

        simTimeNanos = 0;
    }

    private double step(VerticalArmController controller, SimMotorAdapter adapter,
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
        double startAngle = MAX_ANGLE_RAD;
        double targetAngle = Math.toRadians(-120);

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
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
        double holdAngle = Math.toRadians(-60);

        ArmMotorSim sim = makeSim(holdAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(holdAngle);

        Random rng = new Random(SEED);
        for (int i = 0; i < 300; i++) {
            step(controller, adapter, sim, rng);
        }

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
        double startAngle = MAX_ANGLE_RAD;

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(startAngle);

        Random rng = new Random(SEED);
        for (int i = 0; i < 5; i++) {
            step(controller, adapter, sim, rng);
        }

        assertEquals(VerticalArmController.Mode.COASTING, controller.getMode(),
                "should be coasting near hard stop");
        assertEquals(0.0, adapter.lastPower, 0.001,
                "power should be 0 when coasting");
    }

    @Test
    void testWakeFromHardStop() {
        double startAngle = MAX_ANGLE_RAD;
        double targetAngle = Math.toRadians(-120);

        ArmMotorSim sim = makeSim(startAngle);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(startAngle);
        Random rng = new Random(SEED);
        for (int i = 0; i < 20; i++) {
            step(controller, adapter, sim, rng);
        }
        assertEquals(VerticalArmController.Mode.COASTING, controller.getMode());

        controller.setTarget(targetAngle);
        assertEquals(VerticalArmController.Mode.TRACKING, controller.getMode(),
                "should switch to TRACKING after setTarget");

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
        double targetAngle = Math.toRadians(-120);

        ArmMotorSim sim = makeSim(MAX_ANGLE_RAD);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        controller.setTarget(targetAngle);

        Random rng = new Random(SEED);
        for (int i = 0; i < 500; i++) {
            step(controller, adapter, sim, rng);
        }
        assertEquals(targetAngle, sim.getTruePositionRad(), Math.toRadians(5),
                "should be near target before disturbance");

        sim.setDisturbanceVoltage(-3.0);
        for (int i = 0; i < 30; i++) {
            step(controller, adapter, sim, rng);
        }
        sim.setDisturbanceVoltage(0.0);

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
    void testVoltageCompensation() {
        double startAngle = MAX_ANGLE_RAD;
        double targetAngle = Math.toRadians(-120);

        // --- Run at 13.75V ---
        ArmMotorSim sim1 = makeSim(startAngle);
        SimMotorAdapter adapter1 = new SimMotorAdapter(sim1);
        simTimeNanos = 0;
        VerticalArmController ctrl1 = new VerticalArmController(adapter1,
                (caption, format, value) -> {}, this::clockSupplier);
        ctrl1.setTarget(targetAngle);

        Random rng1 = new Random(SEED);
        for (int i = 0; i < 500; i++) {
            double dt = Math.max(0.005, 0.016 + rng1.nextGaussian() * 0.005);
            simTimeNanos += (long) (dt * 1e9);
            ctrl1.update(dt, 13.75);
            sim1.step(dt, adapter1.lastPower, 13.75);
        }

        // --- Run at 10.5V ---
        ArmMotorSim sim2 = makeSim(startAngle);
        SimMotorAdapter adapter2 = new SimMotorAdapter(sim2);
        simTimeNanos = 0;
        VerticalArmController ctrl2 = new VerticalArmController(adapter2,
                (caption, format, value) -> {}, this::clockSupplier);
        ctrl2.setTarget(targetAngle);

        Random rng2 = new Random(SEED);
        for (int i = 0; i < 500; i++) {
            double dt = Math.max(0.005, 0.016 + rng2.nextGaussian() * 0.005);
            simTimeNanos += (long) (dt * 1e9);
            ctrl2.update(dt, 10.5);
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

    // ── worstCaseAngle tests ────────────────────────────────────────────────────

    @Test
    void testWorstCaseAngle_rangeNotCrossingHorizontal() {
        double result = VerticalArmController.worstCaseAngle(Math.toRadians(-60), Math.toRadians(-120));
        double absCosResult = Math.abs(Math.cos(result));
        assertEquals(0.5, absCosResult, 0.01, "should pick an endpoint with |cos|=0.5");
    }

    @Test
    void testWorstCaseAngle_rangeCrossingHorizontal() {
        double result = VerticalArmController.worstCaseAngle(Math.toRadians(-30), Math.toRadians(30));
        assertEquals(0.0, result, 0.01, "should pick horizontal crossing at 0");
    }

    @Test
    void testWorstCaseAngle_rangeCrossingNegativePi() {
        double result = VerticalArmController.worstCaseAngle(Math.toRadians(-150), Math.toRadians(-210));
        assertEquals(-Math.PI, result, 0.01, "should pick horizontal crossing at -pi");
    }

    @Test
    void testWorstCaseAngle_sameAngle() {
        double angle = Math.toRadians(-90);
        double result = VerticalArmController.worstCaseAngle(angle, angle);
        assertEquals(angle, result, 0.01, "should return the single angle");
    }

    @Test
    void testComputeMoveLimits_limitsVaryWithSweep() {
        ArmMotorSim sim = makeSim(MAX_ANGLE_RAD);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        double[] limitsVertical = controller.computeMoveLimits(
                Math.toRadians(-85), Math.toRadians(-95), HUB_VOLTAGE);

        double[] limitsHorizontal = controller.computeMoveLimits(
                Math.toRadians(-10), Math.toRadians(10), HUB_VOLTAGE);

        assertTrue(limitsVertical[1] >= limitsHorizontal[1],
                "accel limit near vertical should be >= near horizontal");
        assertTrue(limitsVertical[0] >= limitsHorizontal[0],
                "velocity limit near vertical should be >= near horizontal");
    }

    @Test
    void testComputeMoveLimits_cappedAtParamsMaximums() {
        ArmMotorSim sim = makeSim(MAX_ANGLE_RAD);
        SimMotorAdapter adapter = new SimMotorAdapter(sim);
        VerticalArmController controller = new VerticalArmController(adapter,
                (caption, format, value) -> {}, this::clockSupplier);

        double[] limits = controller.computeMoveLimits(
                Math.toRadians(-85), Math.toRadians(-95), HUB_VOLTAGE);

        assertTrue(limits[0] <= VerticalArmController.PARAMS.maxVelRad,
                "velocity should be capped at PARAMS.maxVelRad");
        assertTrue(limits[1] <= VerticalArmController.PARAMS.maxAccelRad,
                "accel should be capped at PARAMS.maxAccelRad");
        assertTrue(limits[2] <= VerticalArmController.PARAMS.maxDecelRad,
                "decel should be capped at PARAMS.maxDecelRad");
    }
}
