package org.marsroboticsassociation.controllib.sim

import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class EncoderSimTest {

    @Test
    fun beforeAnyAdvance_positionAndVelocityAreZero() {
        val enc = EncoderSim()
        assertEquals(0, enc.getPosition())
        assertEquals(0.0, enc.getVelocityTps())
    }

    @Test
    fun singleSample_velocityStillZero() {
        val enc = EncoderSim()
        enc.advance(0.010, 1000.0)

        assertEquals(10, enc.getPosition())
        assertEquals(0.0, enc.getVelocityTps())
    }

    @Test
    fun twoSamples_velocityIsCorrect() {
        val enc = EncoderSim()
        enc.advance(0.010, 1000.0)
        enc.advance(0.010, 1000.0)

        assertEquals(20, enc.getPosition())
        // Velocity = (20 - 10) / 0.010 = 1000 TPS
        assertEquals(1000.0, enc.getVelocityTps(), 1e-9)
    }

    @Test
    fun bufferFills_velocityUsesFullSpan() {
        val enc = EncoderSim()
        // 6 samples fills the buffer; span = 5 * 10 ms = 50 ms
        for (i in 0 until 6) {
            enc.advance(0.010, 1000.0)
        }

        // Positions: 10, 20, 30, 40, 50, 60
        assertEquals(60, enc.getPosition())
        // Velocity = (60 - 10) / 0.050 = 1000 TPS
        assertEquals(1000.0, enc.getVelocityTps(), 1e-9)
    }

    @Test
    fun bufferFills_quantizedTo20Tps() {
        val enc = EncoderSim()
        // At 505 TPS, each 10 ms sample advances 5.05 ticks → rounds to 5 each time
        // Positions: 5, 10, 15, 20, 25, 30
        // Velocity = (30 - 5) / 0.050 = 500 TPS — a multiple of 20 TPS
        for (i in 0 until 6) {
            enc.advance(0.010, 505.0)
        }
        val v = enc.getVelocityTps()
        assertEquals(0.0, v % 20.0, 1e-9, "velocity should be a multiple of 20 TPS")
    }

    @Test
    fun largeDt_fillsBuffer() {
        val enc = EncoderSim()
        // 60 ms at 1000 TPS should produce 6 samples in one call
        enc.advance(0.060, 1000.0)

        assertEquals(60, enc.getPosition())
        // 6 samples → span = 5 * 0.010 = 0.050 s
        // Velocity = (60 - 10) / 0.050 = 1000 TPS
        assertEquals(1000.0, enc.getVelocityTps(), 1e-9)
    }

    @Test
    fun zeroVelocity_positionStaysZero() {
        val enc = EncoderSim()
        for (i in 0 until 10) {
            enc.advance(0.010, 0.0)
        }
        assertEquals(0, enc.getPosition())
        assertEquals(0.0, enc.getVelocityTps())
    }

    @Test
    fun reset_clearsAllState() {
        val enc = EncoderSim()
        for (i in 0 until 6) {
            enc.advance(0.010, 1000.0)
        }
        assertNotEquals(0, enc.getPosition())

        enc.reset()
        assertEquals(0, enc.getPosition())
        assertEquals(0.0, enc.getVelocityTps())
    }

    @Test
    fun exactIntegerMath() {
        val enc = EncoderSim()
        // 200 TPS → 2 ticks per 10 ms sample
        for (i in 1..6) {
            enc.advance(0.010, 200.0)
            assertEquals(2 * i, enc.getPosition(), "position after sample $i should be exact")
        }
        // Full buffer: velocity = (12 - 2) / 0.050 = 200 TPS
        assertEquals(200.0, enc.getVelocityTps(), 1e-9)
    }

    @Test
    fun positionIsLive_advancesWithinTheVelocityWindow() {
        val enc = EncoderSim()
        // Position is a live counter: it moves on every read, not only at 10 ms latch boundaries.
        enc.advance(0.003, 1000.0)
        assertEquals(3, enc.getPosition(), "position should advance ~3 ticks after 3 ms")

        enc.advance(0.003, 1000.0)
        assertEquals(6, enc.getPosition(), "position should keep advancing mid-window")

        enc.advance(0.003, 1000.0)
        assertEquals(9, enc.getPosition(), "position should keep advancing mid-window")
    }

    @Test
    fun positionIsLive_neverPlateausAcrossSuccessiveReads() {
        val enc = EncoderSim()
        var previous = enc.getPosition()
        // March forward in 1 ms steps through more than one latch window; the live count must
        // increase essentially every read, never plateauing between 10 ms latches.
        var increases = 0
        for (i in 0 until 25) {
            enc.advance(0.001, 1000.0)
            val now = enc.getPosition()
            if (now > previous) increases++
            previous = now
        }
        assertTrue(
            increases >= 24,
            "live position should advance almost every read, got $increases",
        )
    }

    @Test
    fun jitter_scalesPositionNoiseWithVelocity_andIsSeededDeterministic() {
        // With the jitter model on, a stationary encoder reads exactly (v = 0 -> no v*delta error),
        // while a fast one is smeared by v*delta. Two sims with the same seed agree exactly.
        val a = EncoderSim.controlHub(42L)
        val b = EncoderSim.controlHub(42L)
        for (i in 0 until 6) {
            a.advance(0.010, 2000.0)
            b.advance(0.010, 2000.0)
        }
        assertEquals(
            a.getPosition(),
            b.getPosition(),
            "same seed must give identical jittered reads",
        )

        // Stationary: no velocity means no read-timing error, so the read is exact.
        val still = EncoderSim.expansionHub(7L)
        for (i in 0 until 6) still.advance(0.010, 0.0)
        assertEquals(0, still.getPosition(), "a stationary encoder has no v*delta error")
    }

    @Test
    fun jitter_expansionHubIsNoisierThanControlHub() {
        val controlErr = readNoiseRms(EncoderSim.controlHub(1L))
        val expansionErr = readNoiseRms(EncoderSim.expansionHub(1L))
        assertTrue(
            expansionErr > controlErr,
            "expansion-hub jitter ($expansionErr) should exceed control-hub ($controlErr)",
        )
    }

    @Test
    fun velocity_staledByDelta_readsPreviousWindowOnlyWhenDeltaReachesPastLastLatch() {
        val enc = EncoderSim()
        // Six 10 ms samples at 1000 TPS, then one more at rest, so the current and previous 50 ms
        // windows differ: window-0 spans the slow-down (800 TPS), window-1 is the full 1000 TPS.
        for (i in 0 until 6) enc.advance(0.010, 1000.0)
        enc.advance(0.010, 0.0)
        // Sit 3 ms past the last latch without writing a new sample.
        enc.advance(0.003, 0.0)

        // delta within the 3 ms since the last latch -> still the current window.
        assertEquals(800.0, enc.velocityTpsStaledBy(0.002), 1e-9)
        // delta reaching back past the last latch -> the previous window.
        assertEquals(1000.0, enc.velocityTpsStaledBy(0.006), 1e-9)
    }

    companion object {
        /** RMS of the position read error against the exact live count over many fast reads. */
        private fun readNoiseRms(enc: EncoderSim): Double {
            val velTps = 3000.0
            var sumSq = 0.0
            var n = 0
            for (i in 0 until 200) {
                enc.advance(0.010, velTps)
                val exact = velTps * 0.010 * (i + 1) // exact live count after (i+1) steps
                val err = enc.getPosition() - exact
                sumSq += err * err
                n++
            }
            return sqrt(sumSq / n)
        }
    }
}
