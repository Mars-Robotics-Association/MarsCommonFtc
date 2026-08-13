package org.marsroboticsassociation.controllab.arm

import org.marsroboticsassociation.controllib.sim.ArmMotorSim
import org.marsroboticsassociation.controllib.sim.EncoderSim

/**
 * [ArmPlant] backed by the rigid single-inertia [ArmMotorSim]. The motor and load are one body, so
 * the motor-side link tracks the load exactly and the teeth are always "engaged".
 */
internal class RigidArmPlant(
    private val cfg: ArmPlantConfig,
    initialAngleRad: Double,
) : ArmPlant {
    private lateinit var sim: ArmMotorSim

    init {
        rebuild(initialAngleRad)
    }

    private fun rebuild(initialAngleRad: Double) {
        sim =
            ArmMotorSim(
                cfg.kS,
                cfg.kG,
                cfg.kV,
                cfg.kA,
                cfg.ticksPerRev,
                cfg.gearRatio,
                cfg.encoderZeroOffsetRad,
                cfg.minAngleRad,
                cfg.maxAngleRad,
                initialAngleRad,
            )
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage)
        applyEncoder()
    }

    private fun applyEncoder() {
        when (cfg.encoderKind) {
            ArmPlantConfig.EncoderKind.CONTROL_HUB ->
                sim.setEncoder(EncoderSim.controlHub(cfg.encoderSeed))
            ArmPlantConfig.EncoderKind.EXPANSION_HUB ->
                sim.setEncoder(EncoderSim.expansionHub(cfg.encoderSeed))
            ArmPlantConfig.EncoderKind.NONE -> sim.setEncoder(EncoderSim())
        }
    }

    override fun step(dt: Double, power: Double, hubVoltage: Double) {
        sim.step(dt, power, hubVoltage)
    }

    override val positionTicks: Int
        get() = sim.getPositionTicks()

    override val velocityTps: Double
        get() = sim.getVelocityTps()

    override val truePositionRad: Double
        get() = sim.getTruePositionRad()

    override val trueVelocityRadPerSec: Double
        get() = sim.getTrueVelocityRadPerSec()

    // Rigid: the motor side is the load side.
    override val motorPositionRad: Double
        get() = sim.getTruePositionRad()

    override val isEngaged: Boolean
        get() = true

    override val backlashRad: Double
        get() = 0.0

    override fun restComplianceRadPerVolt(): Double = 0.0

    override fun seedFrom(loadRad: Double, loadVel: Double) {
        rebuild(loadRad)
    }

    override fun applyLiveParams() {
        // Only the disturbance is settable in place on the rigid sim.
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage)
    }
}
