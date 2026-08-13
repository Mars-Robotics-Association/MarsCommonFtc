package org.marsroboticsassociation.controllab.arm

import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim
import org.marsroboticsassociation.controllib.sim.EncoderSim

/**
 * [ArmPlant] backed by the two-inertia [BacklashArmMotorSim]. The motor and load are split across a
 * gearbox dead band, so the motor-side link leads or lags the load across the lash and [isEngaged]
 * flips as the teeth separate and re-contact.
 */
internal class BacklashArmPlant(
    private val cfg: ArmPlantConfig,
    initialAngleRad: Double,
) : ArmPlant {
    private lateinit var sim: BacklashArmMotorSim

    init {
        rebuild(initialAngleRad)
    }

    private fun rebuild(initialAngleRad: Double) {
        sim =
            BacklashArmMotorSim(
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
                cfg.backlashRad,
            )
        applyLiveParams()
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

    override val motorPositionRad: Double
        get() = sim.getMotorPositionRad()

    override val isEngaged: Boolean
        get() = sim.isEngaged

    override val backlashRad: Double
        get() = sim.backlashRad

    override fun restComplianceRadPerVolt(): Double = sim.getRestComplianceRadPerVolt()

    override fun seedFrom(loadRad: Double, loadVel: Double) {
        rebuild(loadRad)
    }

    override fun applyLiveParams() {
        sim.setDisturbanceVoltage(cfg.disturbanceVoltage)
        sim.setContactStiffness(cfg.contactStiffness)
        sim.setContactDamping(cfg.contactDamping)
        sim.setLoadFriction(cfg.loadViscousFriction, cfg.loadStaticFriction)
    }
}
