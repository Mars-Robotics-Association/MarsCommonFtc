package org.marsroboticsassociation.controllib.filter

import kotlin.math.abs
import kotlin.math.sin
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class BiquadLowPassVarDtTest {

    companion object {
        private val BUTTERWORTH_Q = 1.0 / sqrt(2.0)
    }

    @Test
    fun firstSample_passedThrough() {
        val f = BiquadLowPassVarDt(5.0, BUTTERWORTH_Q)
        val y = f.update(42.0, 0.020)
        assertEquals(42.0, y, 1e-12)
    }

    @Test
    fun constantInput_converges() {
        val f = BiquadLowPassVarDt(10.0, BUTTERWORTH_Q)
        val dt = 0.020 // 50 Hz
        var y = 0.0
        for (i in 0 until 200) {
            y = f.update(1.0, dt)
        }
        assertEquals(1.0, y, 1e-6, "should converge to the DC input")
    }

    @Test
    fun getValue_matchesLastOutput() {
        val f = BiquadLowPassVarDt(10.0, BUTTERWORTH_Q)
        var y = f.update(5.0, 0.020)
        assertEquals(y, f.value, 1e-12)

        y = f.update(3.0, 0.020)
        assertEquals(y, f.value, 1e-12)
    }

    @Test
    fun attenuatesHighFrequency() {
        // 5 Hz cutoff, 50 Hz sample rate
        val f = BiquadLowPassVarDt(5.0, BUTTERWORTH_Q)
        val dt = 0.020

        // Feed a 20 Hz sine (well above cutoff) for several cycles
        val samples = 200
        var peakOutput = 0.0
        for (i in 0 until samples) {
            val x = sin(2 * Math.PI * 20.0 * i * dt)
            val y = f.update(x, dt)
            if (i > 50) { // skip transient
                peakOutput = maxOf(peakOutput, abs(y))
            }
        }
        // 20 Hz is 2 octaves above 5 Hz cutoff; 2nd-order rolloff = -40 dB/decade
        // ~24 dB attenuation → amplitude < 0.07. Use generous margin.
        assertTrue(
            peakOutput < 0.15,
            "20 Hz signal should be heavily attenuated; peak was $peakOutput",
        )
    }

    @Test
    fun passesLowFrequency() {
        // 10 Hz cutoff, 50 Hz sample rate
        val f = BiquadLowPassVarDt(10.0, BUTTERWORTH_Q)
        val dt = 0.020

        // Feed a 1 Hz sine (well below cutoff) for several cycles
        val samples = 500
        var peakOutput = 0.0
        for (i in 0 until samples) {
            val x = sin(2 * Math.PI * 1.0 * i * dt)
            val y = f.update(x, dt)
            if (i > 100) {
                peakOutput = maxOf(peakOutput, abs(y))
            }
        }
        // 1 Hz is well below 10 Hz cutoff; should pass with near-unity gain
        assertTrue(
            peakOutput > 0.9,
            "1 Hz signal should pass through nearly unchanged; peak was $peakOutput",
        )
    }

    @Test
    fun variableDt_remainsStable() {
        val f = BiquadLowPassVarDt(5.0, BUTTERWORTH_Q)

        // Alternate between fast and slow loop times (simulating I2C stalls)
        val dts = doubleArrayOf(0.020, 0.020, 0.035, 0.020, 0.040, 0.020)
        for (cycle in 0 until 50) {
            for (dt in dts) {
                val y = f.update(1.0, dt)
                assertTrue(y.isFinite(), "output must remain finite at cycle $cycle")
                // After initial transient settles, output should stay near DC.
                // With a repeating dt pattern the output limit-cycles rather than
                // converging exactly, but should remain bounded near 1.0.
                if (cycle > 5) {
                    assertTrue(abs(y - 1.0) < 0.5, "output should stay near DC; got $y")
                }
            }
        }
    }

    @Test
    fun variableDt_attenuatesHighFrequency() {
        // Even with jittery dt, high-frequency content should be suppressed
        val f = BiquadLowPassVarDt(5.0, BUTTERWORTH_Q)

        var time = 0.0
        var peakOutput = 0.0
        for (i in 0 until 300) {
            // dt jitters between 15-25 ms
            val dt = 0.020 + 0.005 * sin(i * 0.7)
            val x = sin(2 * Math.PI * 20.0 * time)
            val y = f.update(x, dt)
            time += dt
            if (i > 80) {
                peakOutput = maxOf(peakOutput, abs(y))
            }
        }
        assertTrue(
            peakOutput < 0.15,
            "high-frequency signal should still be attenuated with variable dt; peak was $peakOutput",
        )
    }

    @Test
    fun reset_clearsState() {
        val f = BiquadLowPassVarDt(5.0, BUTTERWORTH_Q)
        for (i in 0 until 50) {
            f.update(100.0, 0.020)
        }
        assertEquals(100.0, f.value, 1e-3)

        f.reset()
        assertEquals(0.0, f.value, 1e-12)

        // After reset, first sample should pass through again
        val y = f.update(7.0, 0.020)
        assertEquals(7.0, y, 1e-12)
    }

    @Test
    fun setCutoffHz_takesEffectOnNextUpdate() {
        val f = BiquadLowPassVarDt(5.0, BUTTERWORTH_Q)
        val dt = 0.020

        // Warm up
        for (i in 0 until 100) {
            f.update(0.0, dt)
        }

        // Step input with a very low cutoff — should respond slowly
        f.setCutoffHz(1.0)
        var yLow = 0.0
        for (i in 0 until 5) {
            yLow = f.update(1.0, dt)
        }

        // Reset and repeat with higher cutoff — should respond faster
        f.reset()
        f.setCutoffHz(15.0)
        f.update(0.0, dt) // initialize
        var yHigh = 0.0
        for (i in 0 until 5) {
            yHigh = f.update(1.0, dt)
        }

        assertTrue(
            yHigh > yLow,
            "higher cutoff should respond faster; yHigh=$yHigh yLow=$yLow",
        )
    }
}
