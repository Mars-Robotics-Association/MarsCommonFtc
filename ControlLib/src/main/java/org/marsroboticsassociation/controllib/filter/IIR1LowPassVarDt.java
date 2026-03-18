package org.marsroboticsassociation.controllib.filter;

/**
 * First-order low-pass filter supporting variable dt.
 * Implements the exact discrete-time solution of a continuous-time resistor-capacitor (RC) filter.
 *
 * <p>Continuous form:
 * <pre>
 * tau * dy/dt + y = x
 * </pre>
 *
 * <p>Exact discrete update:
 * <pre>
 * alpha = 1 - exp(-dt / tau)
 * y = y + alpha * (x - y)
 * </pre>
 *
 * <p>This version should be used when loop timing varies, such as in FTC
 * OpModes or when filtering sensor data with nonuniform timestamps.
 */
public class IIR1LowPassVarDt implements LowPassFilter {
    private double tau;
    private double y = 0;
    private boolean initialized = false;


    public IIR1LowPassVarDt(double tauSeconds) {
        this.tau = tauSeconds;
    }

    public void resetTau(double tau) {
        this.tau = tau;
    }

    public static double tauFromCutoffHz(double cutoffHz) {
        return 1.0 / (2.0 * Math.PI * cutoffHz);
    }

    public static double tauFromAlphaDt(double alpha, double dt) {
        return -dt / Math.log(1.0 - alpha);
    }

    @Override
    public void setCutoffHz(double cutoffHz) {
        resetTau(tauFromCutoffHz(cutoffHz));
    }

    @Override
    public double update(double x, double dtSeconds) {
        if (!initialized) {
            y = x;
            initialized = true;
            return y;
        }
        double alpha = 1.0 - Math.exp(-dtSeconds / tau);
        y = y + alpha * (x - y);
        return y;
    }

    @Override
    public double getValue() {
        return y;
    }

    @Override
    public double getRate() {
        return Double.NaN;
    }


    public void reset() {
        initialized = false;
    }
}
