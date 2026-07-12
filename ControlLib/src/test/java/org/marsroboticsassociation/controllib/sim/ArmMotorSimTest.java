package org.marsroboticsassociation.controllib.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Tests for {@link ArmMotorSim#setEncoder}, the parity with {@link BacklashArmMotorSim#setEncoder}
 * that lets the rigid plant honor a read-timing-jitter encoder model too.
 */
class ArmMotorSimTest {

    static final double KS = 0.3, KG = 1.5, KV = 1.2, KA = 0.15;
    static final int TICKS_PER_REV = 28;
    static final double GEAR_RATIO = 100.0;
    static final double TICKS_PER_RAD = (TICKS_PER_REV * GEAR_RATIO) / (2.0 * Math.PI);
    static final double ENCODER_ZERO_OFFSET_RAD = -Math.PI / 4;
    static final double MIN_ANGLE_RAD = -Math.PI * 5 / 4;
    static final double MAX_ANGLE_RAD = -Math.PI / 4;
    static final double HUB_VOLTAGE = 12.0;
    static final double DT = 0.016;

    private ArmMotorSim makeSim(double initialAngleRad) {
        return new ArmMotorSim(KS, KG, KV, KA, TICKS_PER_REV, GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD, MIN_ANGLE_RAD, MAX_ANGLE_RAD, initialAngleRad);
    }

    @Test
    void setEncoder_reseedsToCurrentPosition() {
        double start = Math.toRadians(-90);
        ArmMotorSim sim = makeSim(start);
        sim.setEncoder(new EncoderSim()); // zero-jitter model

        int expectedTicks = (int) Math.round((start - ENCODER_ZERO_OFFSET_RAD) * TICKS_PER_RAD);
        assertEquals(expectedTicks, sim.getPositionTicks(), 1,
                "swapped encoder should read the current arm position immediately");
        assertEquals(0.0, sim.getVelocityTps(), 1e-9,
                "freshly seeded encoder should report zero velocity");
    }

    @Test
    void setEncoder_rejectsNull() {
        ArmMotorSim sim = makeSim(Math.toRadians(-90));
        assertThrows(IllegalArgumentException.class, () -> sim.setEncoder(null));
    }

    @Test
    void setEncoder_windowedVelocityStillWorksUnderMotion() {
        // Start deep in range (back stop is -225 deg) so the arm keeps moving through the flight.
        ArmMotorSim sim = makeSim(Math.toRadians(-200));
        sim.setEncoder(EncoderSim.expansionHub(7L));

        double peakWindowed = 0, peakTrue = 0;
        for (int i = 0; i < 40; i++) {
            sim.step(DT, 0.5, HUB_VOLTAGE); // drive up, away from the back stop
            peakWindowed = Math.max(peakWindowed, sim.getVelocityTps());
            peakTrue = Math.max(peakTrue, sim.getTrueVelocityRadPerSec() * TICKS_PER_RAD);
        }
        assertTrue(Double.isFinite(peakWindowed));
        assertTrue(peakWindowed > 0, "windowed velocity should track the upward drive");
        // The 50 ms window / 20-TPS quantization keeps the peak near the true peak, not exact.
        assertEquals(peakTrue, peakWindowed, Math.max(60.0, peakTrue * 0.25),
                "peak windowed velocity should be close to the true peak rate");
    }

    @Test
    void expansionHubJitter_stalesPositionAtSpeed() {
        // At speed, the Expansion-Hub read-timing delay stales the live position by velocity*delta,
        // so a jittered encoder generally reads a different integer tick than a zero-jitter one for
        // the same motion. Run both from the same state and confirm they can differ.
        ArmMotorSim clean = makeSim(Math.toRadians(-90));
        clean.setEncoder(new EncoderSim());
        ArmMotorSim jittery = makeSim(Math.toRadians(-90));
        jittery.setEncoder(EncoderSim.expansionHub(1L));

        int maxDiff = 0;
        for (int i = 0; i < 80; i++) {
            clean.step(DT, 1.0, HUB_VOLTAGE);
            jittery.step(DT, 1.0, HUB_VOLTAGE);
            maxDiff = Math.max(maxDiff, Math.abs(clean.getPositionTicks() - jittery.getPositionTicks()));
        }
        assertTrue(maxDiff > 0, "Expansion-Hub jitter should stale the position read at speed");
    }
}
