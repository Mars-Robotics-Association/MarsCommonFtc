package org.marsroboticsassociation.controllib.control

import java.util.Random
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim

/** Shared simulation infrastructure for flywheel controller unit tests. */
class FlywheelTestFixture {

    /** IMotor stub that bridges [FlywheelMotorSim] to the controller under test. */
    class SimMotorAdapter(private val sim: FlywheelMotorSim) : IMotor {
        var lastPower = 0.0

        override val name: String
            get() = "sim"

        override val position: Int
            get() = sim.getPositionTicks()

        override val encoderVelocity: Double
            get() = sim.getVelocityTps()

        override fun setPower(power: Double) {
            lastPower = power
        }

        override val hubVoltage: Double
            get() = HUB_VOLTAGE

        override fun setVelocity(tps: Double) {}

        override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {}
    }

    companion object {
        /** Nominal hub voltage used across all flywheel sim tests. */
        const val HUB_VOLTAGE = 13.75

        /** RNG seed shared by all flywheel tests for reproducible noise sequences. */
        const val SEED = 42L

        /** Create a new seeded RNG for test-local use. */
        fun makeRng(): Random = Random(SEED)

        /** Create a flywheel plant simulation with the standard characterization constants. */
        fun makeSim(): FlywheelMotorSim =
            FlywheelMotorSim(FlywheelSimple.PARAMS.kV, FlywheelSimple.PARAMS.kA)
    }
}
