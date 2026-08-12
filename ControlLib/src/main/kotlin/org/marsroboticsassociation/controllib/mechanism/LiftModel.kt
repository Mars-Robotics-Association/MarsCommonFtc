package org.marsroboticsassociation.controllib.mechanism

/**
 * Physics model of a linear lift. Gravity pulls down with the same force at every height, so the
 * hold voltage is a single constant `kG` and its derivative with respect to position is zero. State
 * is in the lift's length units (or encoder ticks), voltages in volts.
 */
class LiftModel(
    kS: Double,
    kV: Double,
    kA: Double,
    private val kG: Double,
) : MechanismModel(kS, kV, kA) {

    /**
     * @param kS static friction (volts)
     * @param kV back-EMF / viscous term, volts per (unit/sec)
     * @param kA volts per (unit/sec^2)
     * @param kG constant gravity term, volts
     */
    override fun gravityVoltage(position: Double): Double {
        return kG
    }

    override fun gravityVoltageDerivative(position: Double): Double {
        return 0.0
    }
}
