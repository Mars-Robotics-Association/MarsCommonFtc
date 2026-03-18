package org.marsroboticsassociation.controllib.filter;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class BiquadLowPassVarDtTest {

    private static final double BUTTERWORTH_Q = 1.0 / Math.sqrt(2.0);

    @Test
    void firstSample_passedThrough() {
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(5.0, BUTTERWORTH_Q);
        double y = f.update(42.0, 0.020);
        assertEquals(42.0, y, 1e-12);
    }

    @Test
    void constantInput_converges() {
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(10.0, BUTTERWORTH_Q);
        double dt = 0.020; // 50 Hz
        double y = 0;
        for (int i = 0; i < 200; i++) {
            y = f.update(1.0, dt);
        }
        assertEquals(1.0, y, 1e-6, "should converge to the DC input");
    }

    @Test
    void getValue_matchesLastOutput() {
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(10.0, BUTTERWORTH_Q);
        double y = f.update(5.0, 0.020);
        assertEquals(y, f.getValue(), 1e-12);

        y = f.update(3.0, 0.020);
        assertEquals(y, f.getValue(), 1e-12);
    }

    @Test
    void attenuatesHighFrequency() {
        // 5 Hz cutoff, 50 Hz sample rate
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(5.0, BUTTERWORTH_Q);
        double dt = 0.020;

        // Feed a 20 Hz sine (well above cutoff) for several cycles
        int samples = 200;
        double peakOutput = 0;
        for (int i = 0; i < samples; i++) {
            double x = Math.sin(2 * Math.PI * 20.0 * i * dt);
            double y = f.update(x, dt);
            if (i > 50) { // skip transient
                peakOutput = Math.max(peakOutput, Math.abs(y));
            }
        }
        // 20 Hz is 2 octaves above 5 Hz cutoff; 2nd-order rolloff = -40 dB/decade
        // ~24 dB attenuation → amplitude < 0.07. Use generous margin.
        assertTrue(peakOutput < 0.15,
                "20 Hz signal should be heavily attenuated; peak was " + peakOutput);
    }

    @Test
    void passesLowFrequency() {
        // 10 Hz cutoff, 50 Hz sample rate
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(10.0, BUTTERWORTH_Q);
        double dt = 0.020;

        // Feed a 1 Hz sine (well below cutoff) for several cycles
        int samples = 500;
        double peakOutput = 0;
        for (int i = 0; i < samples; i++) {
            double x = Math.sin(2 * Math.PI * 1.0 * i * dt);
            double y = f.update(x, dt);
            if (i > 100) {
                peakOutput = Math.max(peakOutput, Math.abs(y));
            }
        }
        // 1 Hz is well below 10 Hz cutoff; should pass with near-unity gain
        assertTrue(peakOutput > 0.9,
                "1 Hz signal should pass through nearly unchanged; peak was " + peakOutput);
    }

    @Test
    void variableDt_remainsStable() {
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(5.0, BUTTERWORTH_Q);

        // Alternate between fast and slow loop times (simulating I2C stalls)
        double[] dts = {0.020, 0.020, 0.035, 0.020, 0.040, 0.020};
        for (int cycle = 0; cycle < 50; cycle++) {
            for (double dt : dts) {
                double y = f.update(1.0, dt);
                assertTrue(Double.isFinite(y),
                        "output must remain finite at cycle " + cycle);
                // After initial transient settles, output should stay near DC.
                // With a repeating dt pattern the output limit-cycles rather than
                // converging exactly, but should remain bounded near 1.0.
                if (cycle > 5) {
                    assertTrue(Math.abs(y - 1.0) < 0.5,
                            "output should stay near DC; got " + y);
                }
            }
        }
    }

    @Test
    void variableDt_attenuatesHighFrequency() {
        // Even with jittery dt, high-frequency content should be suppressed
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(5.0, BUTTERWORTH_Q);

        double time = 0;
        double peakOutput = 0;
        for (int i = 0; i < 300; i++) {
            // dt jitters between 15-25 ms
            double dt = 0.020 + 0.005 * Math.sin(i * 0.7);
            double x = Math.sin(2 * Math.PI * 20.0 * time);
            double y = f.update(x, dt);
            time += dt;
            if (i > 80) {
                peakOutput = Math.max(peakOutput, Math.abs(y));
            }
        }
        assertTrue(peakOutput < 0.15,
                "high-frequency signal should still be attenuated with variable dt; peak was " + peakOutput);
    }

    @Test
    void reset_clearsState() {
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(5.0, BUTTERWORTH_Q);
        for (int i = 0; i < 50; i++) {
            f.update(100.0, 0.020);
        }
        assertEquals(100.0, f.getValue(), 1e-3);

        f.reset();
        assertEquals(0.0, f.getValue(), 1e-12);

        // After reset, first sample should pass through again
        double y = f.update(7.0, 0.020);
        assertEquals(7.0, y, 1e-12);
    }

    @Test
    void setCutoffHz_takesEffectOnNextUpdate() {
        BiquadLowPassVarDt f = new BiquadLowPassVarDt(5.0, BUTTERWORTH_Q);
        double dt = 0.020;

        // Warm up
        for (int i = 0; i < 100; i++) {
            f.update(0.0, dt);
        }

        // Step input with a very low cutoff — should respond slowly
        f.setCutoffHz(1.0);
        double yLow = 0;
        for (int i = 0; i < 5; i++) {
            yLow = f.update(1.0, dt);
        }

        // Reset and repeat with higher cutoff — should respond faster
        f.reset();
        f.setCutoffHz(15.0);
        f.update(0.0, dt); // initialize
        double yHigh = 0;
        for (int i = 0; i < 5; i++) {
            yHigh = f.update(1.0, dt);
        }

        assertTrue(yHigh > yLow,
                "higher cutoff should respond faster; yHigh=" + yHigh + " yLow=" + yLow);
    }
}
