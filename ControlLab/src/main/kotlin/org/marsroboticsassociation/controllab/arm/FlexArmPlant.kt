package org.marsroboticsassociation.controllab.arm

import org.marsroboticsassociation.controllib.sim.EncoderSim
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim

/**
 * [ArmPlant] backed by the three-inertia [FlexArmMotorSim]: motor and hub split across the gearbox
 * dead band (like the backlash plant), plus a lightly damped structural flex spring between the hub
 * and the tip. The tip is reported as the true load — it is what visibly bounces on the way down.
 */
internal class FlexArmPlant(
    private val cfg: ArmPlantConfig,
    initialAngleRad: Double,
) : ArmPlant {
    private lateinit var sim: FlexArmMotorSim

    init {
        rebuild(initialAngleRad)
    }

    private fun rebuild(initialAngleRad: Double) {
        sim =
            FlexArmMotorSim(
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
                cfg.flexHz,
                cfg.flexZeta,
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
        sim.setFlex(cfg.flexHz, cfg.flexZeta)
    }
}
