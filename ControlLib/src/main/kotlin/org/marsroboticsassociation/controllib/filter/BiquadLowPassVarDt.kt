package org.marsroboticsassociation.controllib.filter

import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.cosh
import kotlin.math.exp
import kotlin.math.sin
import kotlin.math.sinh
import kotlin.math.sqrt

/**
 * Second-order low-pass filter supporting variable dt.
 *
 * <p>Uses the exact discretization of the continuous-time state-space form via the matrix
 * exponential, so the filter is stable and accurate regardless of timing jitter or zero-length
 * intervals.
 *
 * <p>Continuous-time transfer function:
 * <pre>
 *   H(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
 * </pre>
 * where wn = 2*pi*fc and zeta = 1/(2*Q).
 *
 * <p>State variables: x1 = filtered value, x2 = its time derivative. The input is assumed constant
 * between samples (zero-order hold).
 */
class BiquadLowPassVarDt(private var cutoffHz: Double, private val q: Double) : LowPassFilter {
    // State: x1 = filtered output, x2 = derivative of x1
    private var x1 = 0.0
    private var x2 = 0.0
    private var initialized = false

    override fun update(x: Double, dt: Double): Double {
        if (!initialized) {
            x1 = x
            x2 = 0.0
            initialized = true
            return x1
        }

        if (dt <= 0) {
            return x1
        }

        val wn = 2.0 * PI * cutoffHz
        val zeta = 1.0 / (2.0 * q)

        // Error state: how far the filter is from the current input
        val e1 = x1 - x
        val e2 = x2

        // Matrix exponential of A*dt, where A = [0, 1; -wn^2, -2*zeta*wn]
        val sigma = zeta * wn
        val decay = exp(-sigma * dt)

        val ad11: Double
        val ad12: Double
        val ad21: Double
        val ad22: Double
        val disc = zeta * zeta - 1.0

        if (disc < -1e-8) {
            // Underdamped (Q > 0.5) — most common case (includes Butterworth Q = 1/sqrt(2))
            val wd = wn * sqrt(-disc)
            val wdDt = wd * dt
            val cosw = cos(wdDt)
            val sinw = sin(wdDt)
            val sinwOverWd = sinw / wd

            ad11 = decay * (cosw + sigma * sinwOverWd)
            ad12 = decay * sinwOverWd
            ad21 = -decay * wn * wn * sinwOverWd
            ad22 = decay * (cosw - sigma * sinwOverWd)
        } else if (disc > 1e-8) {
            // Overdamped (Q < 0.5)
            val gamma = wn * sqrt(disc)
            val gammaDt = gamma * dt
            val coshg = cosh(gammaDt)
            val sinhg = sinh(gammaDt)
            val sinhgOverGamma = sinhg / gamma

            ad11 = decay * (coshg + sigma * sinhgOverGamma)
            ad12 = decay * sinhgOverGamma
            ad21 = -decay * wn * wn * sinhgOverGamma
            ad22 = decay * (coshg - sigma * sinhgOverGamma)
        } else {
            // Critically damped (Q ~ 0.5)
            ad11 = decay * (1.0 + sigma * dt)
            ad12 = decay * dt
            ad21 = -decay * wn * wn * dt
            ad22 = decay * (1.0 - sigma * dt)
        }

        // Evolve error state under Ad, then add back the input (zero-order hold)
        x1 = ad11 * e1 + ad12 * e2 + x
        x2 = ad21 * e1 + ad22 * e2

        return x1
    }

    override fun reset() {
        x1 = 0.0
        x2 = 0.0
        initialized = false
    }

    override fun setCutoffHz(cutoffHz: Double) {
        this.cutoffHz = cutoffHz
    }

    override val value: Double
        get() = x1

    override val rate: Double
        get() = Double.NaN
}
