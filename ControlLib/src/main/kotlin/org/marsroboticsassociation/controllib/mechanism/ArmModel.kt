package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.cos
import kotlin.math.sin

/**
 * Physics model of a rotating arm. Gravity's hold voltage varies with angle as `kCos*cos(theta) +
 * kSin*sin(theta)`; the `kSin` term covers an arm whose center of mass is offset from the encoder's
 * zero angle (pass 0 if gravity is purely a cosine). State is in radians, voltages in volts.
 */
open class ArmModel(
    kS: Double,
    kV: Double,
    kA: Double,
    private val kCos: Double,
    private val kSin: Double,
) : MechanismModel(kS, kV, kA) {

    /**
     * @param kS static friction (volts)
     * @param kV back-EMF / viscous term, volts per rad/s
     * @param kA volts per rad/s^2
     * @param kCos gravity term, volts at horizontal
     * @param kSin gravity term for a center-of-mass angular offset, volts (0 if none)
     */
    override fun gravityVoltage(theta: Double): Double {
        return kCos * cos(theta) + kSin * sin(theta)
    }

    override fun gravityVoltageDerivative(theta: Double): Double {
        return -kCos * sin(theta) + kSin * cos(theta)
    }
}
