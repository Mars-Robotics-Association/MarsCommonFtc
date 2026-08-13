package org.marsroboticsassociation.controllib.hardware

interface IMotor {
    val name: String

    /** Encoder ticks. */
    val position: Int

    /**
     * Encoder ticks per second. Named separately from `DcMotorEx.getVelocity()` so a Kotlin class
     * can implement both without a JVM signature clash (that Java method is overloaded).
     */
    val encoderVelocity: Double

    fun setPower(power: Double)

    /** Volts, live read. */
    val hubVoltage: Double

    fun setVelocity(tps: Double)

    fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double)
}
