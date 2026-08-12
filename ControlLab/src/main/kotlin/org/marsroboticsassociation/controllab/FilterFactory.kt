package org.marsroboticsassociation.controllab

import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt
import org.marsroboticsassociation.controllib.filter.Filter
import org.marsroboticsassociation.controllib.filter.IIR1LowPassVarDt

object FilterFactory {
    enum class Type {
        LOWPASS,
        BIQUAD,
        NONE,
    }

    @JvmStatic
    fun create(t: Type, param1: Double, param2: Double, param3: Double): Filter {
        return when (t) {
            Type.LOWPASS -> IIR1LowPassVarDt(IIR1LowPassVarDt.tauFromCutoffHz(param1))
            Type.BIQUAD -> BiquadLowPassVarDt(param1, param2)
            Type.NONE ->
                object : Filter {
                    private var x = 0.0

                    override fun update(x: Double, dt: Double): Double {
                        this.x = x
                        return x
                    }

                    override val rate: Double
                        get() = Double.NaN

                    override val value: Double
                        get() = this.x

                    override fun reset() {}
                }
        }
    }
}
