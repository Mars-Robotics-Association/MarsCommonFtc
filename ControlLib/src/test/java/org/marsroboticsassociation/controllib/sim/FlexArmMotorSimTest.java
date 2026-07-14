package org.marsroboticsassociation.controllib.sim;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Behavioral tests for {@link FlexArmMotorSim}. These assert the qualitative consequences of arm
 * structural flex on top of gearbox backlash — above all the long-heavy-arm signature the rigid
 * plants cannot reproduce: the arm is bouncy on the way down but not on the way up — rather than
 * exact numbers, since the flex and contact parameters are defaults pending sysid.
 */
class FlexArmMotorSimTest {

    // Heavy long-arm plant (matches the ControlLab arm defaults) with an over-the-top workspace.
    static final double KS = 0.3;
    static final double KG = 3.5;
    static final double KV = 1.2;
    static final double KA = 0.35;

    static final int TICKS_PER_REV = 28;
    static final double GEAR_RATIO = 100.0;
    static final double TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);

    static final double ENCODER_ZERO_OFFSET_RAD = 0.0;
    static final double MIN_ANGLE_RAD = Math.toRadians(-45);
    static final double MAX_ANGLE_RAD = Math.toRadians(225);

    static final double BACKLASH_RAD = Math.toRadians(5);
    static final double FLEX_HZ = 3.0;
    static final double FLEX_ZETA = 0.03;
    static final double HUB_VOLTAGE = 12.0;
    static final double DT = 0.016;

    private FlexArmMotorSim makeSim(double initialAngleRad) {
        return new FlexArmMotorSim(KS, KG, KV, KA,
                TICKS_PER_REV, GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD, MAX_ANGLE_RAD,
                initialAngleRad, BACKLASH_RAD, FLEX_HZ, FLEX_ZETA);
    }

    /** Encoder ticks -> motor-side angle in output radians. */
    private double encoderAngleRad(FlexArmMotorSim sim) {
        return sim.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD;
    }

    @Test
    void seedsInStaticEquilibriumAndHoldsUnderGravityFeedforward() {
        double start = Math.toRadians(30); // strong gravity (cos = 0.87)
        FlexArmMotorSim sim = makeSim(start);
        double holdPower = (KG * Math.cos(start)) / HUB_VOLTAGE;
        for (int i = 0; i < 200; i++) {
            sim.step(DT, holdPower, HUB_VOLTAGE);
        }
        assertEquals(start, sim.getTruePositionRad(), Math.toRadians(2.0),
                "tip should hold under gravity feedforward");
        // The flex spring must carry the tip's gravity: hub sits above the tip by the static
        // deflection, on the same side gravity pulls the tip down from.
        assertTrue(sim.getHubPositionRad() > sim.getTruePositionRad(),
                "flex spring should deflect under the tip's gravity");
    }

    @Test
    void lostMotionOnReversal() {
        // Drive up firmly to seat the forward tooth, then reverse: the motor crosses the lash
        // while the tip barely reacts.
        FlexArmMotorSim sim = makeSim(Math.toRadians(90));
        for (int i = 0; i < 40; i++) sim.step(DT, 1.0, HUB_VOLTAGE);

        double tipBefore = sim.getTruePositionRad();
        double encBefore = encoderAngleRad(sim);

        double encDelta = 0.0, tipDelta = 0.0;
        for (int i = 0; i < 6; i++) {
            sim.step(DT, -1.0, HUB_VOLTAGE);
            encDelta = Math.abs(encoderAngleRad(sim) - encBefore);
            tipDelta = Math.abs(sim.getTruePositionRad() - tipBefore);
            if (encDelta >= BACKLASH_RAD) break;
        }

        assertTrue(encDelta > BACKLASH_RAD * 0.5,
                "motor/encoder should move appreciably during the reversal");
        assertTrue(tipDelta < encDelta * 0.5,
                "tip should lag the motor while the lash is being crossed (lost motion): "
                        + "tipDelta=" + tipDelta + " encDelta=" + encDelta);
    }

    /**
     * The reason this sim exists: under the same closed-loop ramp, a long heavy flexible arm is
     * bouncy on the way <b>down</b> (the tip oscillation repeatedly unloads the gear mesh and each
     * re-impact pumps gravity energy back into the flex mode) and comparatively smooth on the way
     * <b>up</b> (drive torque and gravity load the same tooth face, so the mesh never unloads).
     * The two-inertia rigid-load plant shows neither.
     */
    @Test
    void descentIsBouncyButAscentIsNot() {
        double downOsc = trackedMoveOscillation(Math.toRadians(90), Math.toRadians(-40));
        double upOsc = trackedMoveOscillation(Math.toRadians(-40), Math.toRadians(90));

        assertTrue(downOsc > 0.15,
                "descent should visibly bounce (detrended tip-velocity std, rad/s): " + downOsc);
        assertTrue(downOsc > 1.8 * upOsc,
                "descent should be clearly bouncier than ascent: down=" + downOsc + " up=" + upOsc);
    }

    /**
     * Runs a PD + gravity + velocity feedforward controller (on the motor side, like a real
     * motor-encoder controller) along a constant-velocity ramp from {@code from} to {@code to}, and
     * returns the standard deviation of the tip velocity about the commanded ramp rate over the
     * middle of the move — the "bounciness" of the arm itself.
     */
    private double trackedMoveOscillation(double from, double to) {
        FlexArmMotorSim sim = makeSim(from);
        double kP = 12.0, kD = 0.8;
        double spVel = Math.signum(to - from) * 1.2;
        double moveEnd = (to - from) / spVel;

        List<Double> mid = new ArrayList<>();
        for (double t = 0; t < moveEnd; t += DT) {
            double sp = from + spVel * t;
            double measPos = sim.getMotorPositionRad();
            double measVel = sim.getMotorVelocityRadPerSec();
            double u = kP * (sp - measPos) + kD * (0 - measVel)
                    + KG * Math.cos(measPos) + KV * spVel;
            double power = Math.max(-1.0, Math.min(1.0, u / HUB_VOLTAGE));
            sim.step(DT, power, HUB_VOLTAGE);
            if (t > 0.1 * moveEnd && t < 0.9 * moveEnd) {
                mid.add(sim.getTrueVelocityRadPerSec() - spVel);
            }
        }
        double mean = 0;
        for (double v : mid) mean += v;
        mean /= mid.size();
        double var = 0;
        for (double v : mid) var += (v - mean) * (v - mean);
        return Math.sqrt(var / mid.size());
    }

    @Test
    void encoderIsBlindToTheTipAcrossLashAndFlex() {
        // During a hard descent the encoder (motor side) diverges from the tip by more than the
        // lash alone — the flex deflection adds to the lost motion a motor encoder cannot see.
        FlexArmMotorSim sim = makeSim(Math.toRadians(90));
        double maxDisagreement = 0.0;
        for (int i = 0; i < 60; i++) {
            sim.step(DT, -0.6, HUB_VOLTAGE);
            double disagreement = Math.abs(encoderAngleRad(sim) - sim.getTruePositionRad());
            maxDisagreement = Math.max(maxDisagreement, disagreement);
        }
        assertTrue(maxDisagreement > Math.toRadians(1.0),
                "encoder should diverge from the true tip angle: "
                        + Math.toDegrees(maxDisagreement) + " deg");
    }

    @Test
    void hardStopsClampTheArm() {
        FlexArmMotorSim sim = makeSim(MAX_ANGLE_RAD - Math.toRadians(10));
        for (int i = 0; i < 300; i++) sim.step(DT, 1.0, HUB_VOLTAGE);
        assertTrue(sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6,
                "tip should not exceed the back hard stop");

        FlexArmMotorSim sim2 = makeSim(MIN_ANGLE_RAD + Math.toRadians(20));
        for (int i = 0; i < 400; i++) sim2.step(DT, -1.0, HUB_VOLTAGE);
        assertTrue(sim2.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6,
                "tip should not exceed the front hard stop");
    }

    @Test
    void statesStayBoundedUnderAggressiveInput() {
        FlexArmMotorSim sim = makeSim(Math.toRadians(90));
        for (int i = 0; i < 1000; i++) {
            double power = (i % 10 < 5) ? 1.0 : -1.0;
            sim.step(DT, power, HUB_VOLTAGE);
            assertTrue(Double.isFinite(sim.getMotorPositionRad()), "motor pos finite");
            assertTrue(Double.isFinite(sim.getTrueVelocityRadPerSec()), "tip vel finite");
            assertTrue(Math.abs(sim.getMotorVelocityRadPerSec()) < 1e4, "motor vel bounded");
        }
        assertTrue(sim.getTruePositionRad() >= MIN_ANGLE_RAD - 1e-6
                && sim.getTruePositionRad() <= MAX_ANGLE_RAD + 1e-6);
    }

    @Test
    void rejectsNonPositiveFlexParams() {
        assertThrows(IllegalArgumentException.class, () -> makeSim(0).setFlex(0, 0.03));
        assertThrows(IllegalArgumentException.class, () -> makeSim(0).setFlex(3.0, 0));
    }
}
