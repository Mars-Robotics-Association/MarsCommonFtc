package org.marsroboticsassociation.controllib.filter;

/**
 * Second-order low-pass filter supporting variable dt.
 * <p>
 * Uses the exact discretization of the continuous-time state-space form via the
 * matrix exponential, so the filter is stable and accurate regardless of timing
 * jitter or zero-length intervals.
 * <p>
 * Continuous-time transfer function:
 * <pre>
 *   H(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
 * </pre>
 * where wn = 2*pi*fc and zeta = 1/(2*Q).
 * <p>
 * State variables: x1 = filtered value, x2 = its time derivative.
 * The input is assumed constant between samples (zero-order hold).
 */
public class BiquadLowPassVarDt implements LowPassFilter {
    private double cutoffHz;
    private double q;

    // State: x1 = filtered output, x2 = derivative of x1
    private double x1 = 0, x2 = 0;
    private boolean initialized = false;

    public BiquadLowPassVarDt(double cutoffHz, double q) {
        this.cutoffHz = cutoffHz;
        this.q = q;
    }

    @Override
    public double update(double input, double dt) {
        if (!initialized) {
            x1 = input;
            x2 = 0;
            initialized = true;
            return x1;
        }

        if (dt <= 0) {
            return x1;
        }

        double wn = 2.0 * Math.PI * cutoffHz;
        double zeta = 1.0 / (2.0 * q);

        // Error state: how far the filter is from the current input
        double e1 = x1 - input;
        double e2 = x2;

        // Matrix exponential of A*dt, where A = [0, 1; -wn^2, -2*zeta*wn]
        double sigma = zeta * wn;
        double decay = Math.exp(-sigma * dt);

        double ad11, ad12, ad21, ad22;
        double disc = zeta * zeta - 1.0;

        if (disc < -1e-8) {
            // Underdamped (Q > 0.5) — most common case (includes Butterworth Q = 1/sqrt(2))
            double wd = wn * Math.sqrt(-disc);
            double wdDt = wd * dt;
            double cosw = Math.cos(wdDt);
            double sinw = Math.sin(wdDt);
            double sinwOverWd = sinw / wd;

            ad11 = decay * (cosw + sigma * sinwOverWd);
            ad12 = decay * sinwOverWd;
            ad21 = -decay * wn * wn * sinwOverWd;
            ad22 = decay * (cosw - sigma * sinwOverWd);

        } else if (disc > 1e-8) {
            // Overdamped (Q < 0.5)
            double gamma = wn * Math.sqrt(disc);
            double gammaDt = gamma * dt;
            double coshg = Math.cosh(gammaDt);
            double sinhg = Math.sinh(gammaDt);
            double sinhgOverGamma = sinhg / gamma;

            ad11 = decay * (coshg + sigma * sinhgOverGamma);
            ad12 = decay * sinhgOverGamma;
            ad21 = -decay * wn * wn * sinhgOverGamma;
            ad22 = decay * (coshg - sigma * sinhgOverGamma);

        } else {
            // Critically damped (Q ~ 0.5)
            ad11 = decay * (1.0 + sigma * dt);
            ad12 = decay * dt;
            ad21 = -decay * wn * wn * dt;
            ad22 = decay * (1.0 - sigma * dt);
        }

        // Evolve error state under Ad, then add back the input (zero-order hold)
        x1 = ad11 * e1 + ad12 * e2 + input;
        x2 = ad21 * e1 + ad22 * e2;

        return x1;
    }

    @Override
    public void reset() {
        x1 = 0;
        x2 = 0;
        initialized = false;
    }

    @Override
    public void setCutoffHz(double cutoffHz) {
        this.cutoffHz = cutoffHz;
    }

    @Override
    public double getValue() {
        return x1;
    }

    @Override
    public double getRate() {
        return Double.NaN;
    }
}
