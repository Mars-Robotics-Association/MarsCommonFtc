package org.marsroboticsassociation.controllib.hardware

interface IMotor {
    val name: String

    /** Encoder ticks. */
    val position: Int

    /** Ticks per second. */
    val velocity: Double

    fun setPower(power: Double)

    /** Volts, live read. */
    val hubVoltage: Double

    fun setVelocity(tps: Double)

    fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double)
}
