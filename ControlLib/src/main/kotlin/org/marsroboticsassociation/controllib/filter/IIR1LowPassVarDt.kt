package org.marsroboticsassociation.controllib.filter

import kotlin.math.PI
import kotlin.math.exp
import kotlin.math.ln

/**
 * First-order low-pass filter supporting variable dt. Implements the exact discrete-time solution
 * of a continuous-time resistor-capacitor (RC) filter.
 *
 * <p>Continuous form:
 * <pre>
 * tau * dy/dt + y = x
 * </pre>
 * <p>Exact discrete update:
 * <pre>
 * alpha = 1 - exp(-dt / tau)
 * y = y + alpha * (x - y)
 * </pre>
 * <p>This version should be used when loop timing varies, such as in FTC OpModes or when filtering
 * sensor data with nonuniform timestamps.
 */
class IIR1LowPassVarDt(private var tau: Double) : LowPassFilter {
    private var y = 0.0
    private var initialized = false

    fun resetTau(tau: Double) {
        this.tau = tau
    }

    override fun setCutoffHz(cutoffHz: Double) {
        resetTau(tauFromCutoffHz(cutoffHz))
    }

    override fun update(x: Double, dt: Double): Double {
        if (!initialized) {
            y = x
            initialized = true
            return y
        }
        val alpha = 1.0 - exp(-dt / tau)
        y = y + alpha * (x - y)
        return y
    }

    override val value: Double
        get() = y

    override val rate: Double
        get() = Double.NaN

    override fun reset() {
        initialized = false
    }

    companion object {
        @JvmStatic
        fun tauFromCutoffHz(cutoffHz: Double): Double {
            return 1.0 / (2.0 * PI * cutoffHz)
        }

        @JvmStatic
        fun tauFromAlphaDt(alpha: Double, dt: Double): Double {
            return -dt / ln(1.0 - alpha)
        }
    }
}
