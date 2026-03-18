package org.marsroboticsassociation.controllib.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class EncoderSimTest {

    @Test
    void beforeAnyAdvance_positionAndVelocityAreZero() {
        EncoderSim enc = new EncoderSim();
        assertEquals(0, enc.getPosition());
        assertEquals(0.0, enc.getVelocityTps());
    }

    @Test
    void singleSample_velocityStillZero() {
        EncoderSim enc = new EncoderSim();
        enc.advance(0.010, 1000.0);

        assertEquals(10, enc.getPosition());
        assertEquals(0.0, enc.getVelocityTps());
    }

    @Test
    void twoSamples_velocityIsCorrect() {
        EncoderSim enc = new EncoderSim();
        enc.advance(0.010, 1000.0);
        enc.advance(0.010, 1000.0);

        assertEquals(20, enc.getPosition());
        // Velocity = (20 - 10) / 0.010 = 1000 TPS
        assertEquals(1000.0, enc.getVelocityTps(), 1e-9);
    }

    @Test
    void bufferFills_velocityUsesFullSpan() {
        EncoderSim enc = new EncoderSim();
        // 6 samples fills the buffer; span = 5 * 10 ms = 50 ms
        for (int i = 0; i < 6; i++) {
            enc.advance(0.010, 1000.0);
        }

        // Positions: 10, 20, 30, 40, 50, 60
        assertEquals(60, enc.getPosition());
        // Velocity = (60 - 10) / 0.050 = 1000 TPS
        assertEquals(1000.0, enc.getVelocityTps(), 1e-9);
    }

    @Test
    void bufferFills_quantizedTo20Tps() {
        EncoderSim enc = new EncoderSim();
        // At 505 TPS, each 10 ms sample advances 5.05 ticks → rounds to 5 each time
        // Positions: 5, 10, 15, 20, 25, 30
        // Velocity = (30 - 5) / 0.050 = 500 TPS — a multiple of 20 TPS
        for (int i = 0; i < 6; i++) {
            enc.advance(0.010, 505.0);
        }
        double v = enc.getVelocityTps();
        assertEquals(0.0, v % 20.0, 1e-9, "velocity should be a multiple of 20 TPS");
    }

    @Test
    void largeDt_fillsBuffer() {
        EncoderSim enc = new EncoderSim();
        // 60 ms at 1000 TPS should produce 6 samples in one call
        enc.advance(0.060, 1000.0);

        assertEquals(60, enc.getPosition());
        // 6 samples → span = 5 * 0.010 = 0.050 s
        // Velocity = (60 - 10) / 0.050 = 1000 TPS
        assertEquals(1000.0, enc.getVelocityTps(), 1e-9);
    }

    @Test
    void zeroVelocity_positionStaysZero() {
        EncoderSim enc = new EncoderSim();
        for (int i = 0; i < 10; i++) {
            enc.advance(0.010, 0.0);
        }
        assertEquals(0, enc.getPosition());
        assertEquals(0.0, enc.getVelocityTps());
    }

    @Test
    void reset_clearsAllState() {
        EncoderSim enc = new EncoderSim();
        for (int i = 0; i < 6; i++) {
            enc.advance(0.010, 1000.0);
        }
        assertNotEquals(0, enc.getPosition());

        enc.reset();
        assertEquals(0, enc.getPosition());
        assertEquals(0.0, enc.getVelocityTps());
    }

    @Test
    void exactIntegerMath() {
        EncoderSim enc = new EncoderSim();
        // 200 TPS → 2 ticks per 10 ms sample
        for (int i = 1; i <= 6; i++) {
            enc.advance(0.010, 200.0);
            assertEquals(2 * i, enc.getPosition(),
                    "position after sample " + i + " should be exact");
        }
        // Full buffer: velocity = (12 - 2) / 0.050 = 200 TPS
        assertEquals(200.0, enc.getVelocityTps(), 1e-9);
    }

    @Test
    void partialStep_noSampleUntilBoundary() {
        EncoderSim enc = new EncoderSim();
        enc.advance(0.005, 1000.0);
        assertEquals(0, enc.getPosition(), "no sample should be written before 10 ms boundary");

        enc.advance(0.005, 1000.0);
        assertEquals(10, enc.getPosition(), "sample should be written at 10 ms boundary");
    }
}
