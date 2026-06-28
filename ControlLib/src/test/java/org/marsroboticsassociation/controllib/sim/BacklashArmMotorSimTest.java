package org.marsroboticsassociation.controllib.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Behavioral tests for {@link BacklashArmMotorSim}. These assert the qualitative consequences of
 * gearbox backlash — lost motion on reversal, gravity free-fall across the gap, and the encoder
 * being blind to the load — rather than exact numbers, since the contact parameters are defaults
 * pending sysid.
 */
class BacklashArmMotorSimTest {

    // Feedforward gains (output-shaft rad units, volts) — same family as ArmControllerTest.
    static final double KS = 0.3;
    static final double KG = 1.5;
    static final double KV = 1.2;
    static final double KA = 0.15;

    static final int TICKS_PER_REV = 28;
    static final double GEAR_RATIO = 100.0;
    static final double TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);

    // Front hard stop at -45 deg, back hard stop at -225 deg; encoder reads 0 at the front stop.
    static final double ENCODER_ZERO_OFFSET_RAD = -Math.PI / 4;
    static final double MIN_ANGLE_RAD = -Math.PI * 5 / 4;
    static final double MAX_ANGLE_RAD = -Math.PI / 4;

    static final double BACKLASH_RAD = Math.toRadians(5);
    static final double HUB_VOLTAGE = 12.0;
    static final double DT = 0.016;

    private BacklashArmMotorSim makeSim(double initialAngleRad) {
        return new BacklashArmMotorSim(KS, KG, KV, KA,
                TICKS_PER_REV, GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD, MAX_ANGLE_RAD,
                initialAngleRad, BACKLASH_RAD);
    }

    /** Encoder ticks -> motor-side angle in output radians. */
    private double encoderAngleRad(BacklashArmMotorSim sim) {
        return sim.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD;
    }

    // A mid-range angle with strong gravity (cos = -0.707). The arm's range is [-225 deg, -45 deg],
    // so horizontal (0 deg) is out of range — gravity tests must use an in-range angle.
    static final double MID_ANGLE_RAD = Math.toRadians(-135);

    @Test
    void holdsStaticallyUnderGravityFeedforward() {
        // Apply the gravity-canceling holding voltage; the arm should barely move.
        double start = MID_ANGLE_RAD;
        BacklashArmMotorSim sim = makeSim(start);
        double holdPower = (KG * Math.cos(start)) / HUB_VOLTAGE;
        for (int i = 0; i < 200; i++) {
            sim.step(DT, holdPower, HUB_VOLTAGE);
        }
        assertEquals(start, sim.getTruePositionRad(), Math.toRadians(2.0),
                "arm should hold under gravity feedforward");
    }

    @Test
    void lostMotionOnReversal() {
        // Drive up firmly to seat the forward tooth, then command downward. The motor (encoder)
        // must cross the backlash before the load reacts: while crossing, the load moves far less
        // than the motor.
        BacklashArmMotorSim sim = makeSim(MID_ANGLE_RAD);
        for (int i = 0; i < 40; i++) sim.step(DT, 1.0, HUB_VOLTAGE);

        double loadBefore = sim.getTruePositionRad();
        double encBefore = encoderAngleRad(sim);

        // Reverse just long enough to traverse roughly the backlash gap, not far beyond it.
        double encDelta = 0.0, loadDelta = 0.0;
        for (int i = 0; i < 6; i++) {
            sim.step(DT, -1.0, HUB_VOLTAGE);
            encDelta = Math.abs(encoderAngleRad(sim) - encBefore);
            loadDelta = Math.abs(sim.getTruePositionRad() - loadBefore);
            if (encDelta >= BACKLASH_RAD) break;
        }

        assertTrue(encDelta > BACKLASH_RAD * 0.5,
                "motor/encoder should move appreciably during the reversal");
        assertTrue(loadDelta < encDelta * 0.5,
                "load should lag the motor while the lash is being crossed (lost motion): "
                        + "loadDelta=" + loadDelta + " encDelta=" + encDelta);
    }

    @Test
    void freeFallAndBlindEncoderAcrossTheGap() {
        double start = MID_ANGLE_RAD;
        // Direction gravity accelerates the load: dω_L ∝ −cos(θ).
        double gravityDir = -Math.signum(Math.cos(start));

        // Settle on the gravity-loaded contact face (the seed already rests there).
        BacklashArmMotorSim sim = makeSim(start);
        double holdPower = (KG * Math.cos(start)) / HUB_VOLTAGE;
        for (int i = 0; i < 20; i++) sim.step(DT, holdPower, HUB_VOLTAGE);

        double initialLoadVel = sim.getTrueVelocityRadPerSec();

        // Drive the motor across the gap in the gravity direction. The teeth separate and the load,
        // now unsupported, free-falls under gravity until the far face catches it.
        boolean sawSeparation = false;
        double maxDisagreement = 0.0;
        for (int i = 0; i < 12; i++) {
            sim.step(DT, gravityDir, HUB_VOLTAGE);
            if (!sim.isEngaged()) sawSeparation = true;
            double disagreement = Math.abs(encoderAngleRad(sim) - sim.getTruePositionRad());
            maxDisagreement = Math.max(maxDisagreement, disagreement);
        }
        double finalLoadVel = sim.getTrueVelocityRadPerSec();

        assertTrue(sawSeparation, "the gear teeth should separate (a real dead-band gap opens)");
        assertTrue(gravityDir * finalLoadVel > gravityDir * initialLoadVel + 0.2,
                "the unsupported load should accelerate under gravity (free-fall across the gap)");
        assertTrue(maxDisagreement > Math.toRadians(0.5),
                "encoder (motor side) should diverge from the true arm angle across the gap: "
                        + Math.toDegrees(maxDisagreement) + " deg");
        assertTrue(maxDisagreement <= BACKLASH_RAD + Math.toRadians(1.0),
                "divergence should not exceed the backlash by much: "
                        + Math.toDegrees(maxDisagreement) + " deg");
    }

    @Test
    void hardStopsClampTheLoad() {
        // Drive hard toward the front stop; the load must not pass it.
        BacklashArmMotorSim sim = makeSim(MAX_ANGLE_RAD + Math.toRadians(10));
        for (int i = 0; i < 200; i++) sim.step(DT, 1.0, HUB_VOLTAGE);
        assertTrue(sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6,
                "load should not exceed the front hard stop");

        // And toward the back stop.
        BacklashArmMotorSim sim2 = makeSim(MIN_ANGLE_RAD - Math.toRadians(10) + Math.toRadians(20));
        for (int i = 0; i < 400; i++) sim2.step(DT, -1.0, HUB_VOLTAGE);
        assertTrue(sim2.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6,
                "load should not exceed the back hard stop");
    }

    @Test
    void zeroBacklashStaysWellBehaved() {
        // With no backlash the contact is always engaged; the plant should still be stable and
        // hold roughly still under a gravity-canceling command.
        double start = MID_ANGLE_RAD;
        BacklashArmMotorSim sim = new BacklashArmMotorSim(KS, KG, KV, KA,
                TICKS_PER_REV, GEAR_RATIO, ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD, MAX_ANGLE_RAD, start, 0.0);
        double holdPower = (KG * Math.cos(start)) / HUB_VOLTAGE;
        for (int i = 0; i < 200; i++) sim.step(DT, holdPower, HUB_VOLTAGE);
        assertEquals(start, sim.getTruePositionRad(), Math.toRadians(2.0));
        assertTrue(Double.isFinite(sim.getMotorVelocityRadPerSec()));
    }

    @Test
    void statesStayBoundedUnderAggressiveInput() {
        // Slam the input back and forth to excite the contact; nothing should diverge.
        BacklashArmMotorSim sim = makeSim(-Math.PI / 2);
        for (int i = 0; i < 1000; i++) {
            double power = (i % 10 < 5) ? 1.0 : -1.0;
            sim.step(DT, power, HUB_VOLTAGE);
            assertTrue(Double.isFinite(sim.getMotorPositionRad()), "motor pos finite");
            assertTrue(Double.isFinite(sim.getTrueVelocityRadPerSec()), "load vel finite");
            assertTrue(Math.abs(sim.getMotorVelocityRadPerSec()) < 1e4, "motor vel bounded");
        }
        assertTrue(sim.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6
                && sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6);
    }
}
