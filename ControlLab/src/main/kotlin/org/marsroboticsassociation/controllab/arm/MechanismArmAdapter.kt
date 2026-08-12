package org.marsroboticsassociation.controllab.arm

import kotlin.math.PI
import org.marsroboticsassociation.controllib.mechanism.ArmModel
import org.marsroboticsassociation.controllib.mechanism.MotorMechanismController
import org.marsroboticsassociation.controllib.mechanism.MotorMechanismEkf

/**
 * Lineage B adapter: wraps [MotorMechanismController] (pure `calculate(...)->voltage`) plus its own
 * [MotorMechanismEkf] state estimator and a shared [ArmModel].
 *
 * This is the **only** unit-conversion seam in the project: the plant reports the encoder in ticks
 * / TPS, while the mechanism controller and EKF work in radians / rad·s⁻¹. The conversion is
 * isolated in [ticksToRad]/[tpsToRadPerSec] and unit-tested.
 */
class MechanismArmAdapter(
    private val gains: Gains,
    plant: ArmPlant,
    private val ticksPerRev: Int,
    private val gearRatio: Double,
    private val encoderZeroOffsetRad: Double,
) : ArmControlAdapter {
    private var plant: ArmPlant = plant
    private lateinit var model: ArmModel
    private lateinit var controller: MotorMechanismController
    private lateinit var ekf: MotorMechanismEkf

    private var targetRad: Double
    private var lastPower = 0.0

    init {
        val posRad = currentMeasuredPosRad()
        this.targetRad = posRad
        build(posRad)
    }

    /** Editable gains, gathered so a rebuild is a single call. */
    class Gains {
        @JvmField var kP: Double = 40.0
        @JvmField var kI: Double = 8.0
        // kD 4 damps the arm's coupled flex mode through the engaged gear mesh — halves the
        // arrival swing at zero move-time cost (FlexRingTuningTest).
        @JvmField var kD: Double = 4.0
        @JvmField
        var kCos: Double = 3.5 // gravity at horizontal (== plant kG by default, heavy end-effector)
        @JvmField var kSin: Double = 0.0 // center-of-mass angular offset term
        @JvmField var kS: Double = 0.3 // static friction
        @JvmField var kV: Double = 1.2 // back-EMF / viscous
        @JvmField var kA: Double = 0.35 // inertia (matches the heavy plant default)
        @JvmField var maxVel: Double = 8.0
        // Shaped for the flexible arm (FlexRingTuningTest): the jerk ramp aMax/jMax = 0.33 s sits
        // on the coupled flex mode's period band, so the profile cancels its own excitation
        // (arrival swing ~0.55 deg vs ~2.9 deg at a12/j480, for ~0.5 s more on a full move).
        @JvmField var maxAccel: Double = 8.0
        // Braking cap, separate from maxAccel so the arm may launch harder than it stops (the
        // arrival swing constrains braking, not starting). Equal by default.
        @JvmField var maxDecel: Double = 8.0
        @JvmField var maxJerk: Double = 24.0
        // Taper the kS feedforward to zero as the profile decelerates below this speed into an
        // arrival. Motor-side sysid through the lash + flex biases kS high (the friction-flex
        // deflection is collinear with sign(v)), and a too-large kS is an anti-braking term that
        // reintroduces arrival overshoot; the taper caps that. 0 disables it.
        @JvmField var staticFrictionTaperVelocity: Double = 1.0
        @JvmField var feedbackVoltageMargin: Double = 1.5
        @JvmField var velocityLagSec: Double = 0.025
        @JvmField var modelAccelStdDev: Double = 5.0
        @JvmField var positionStdDev: Double = 0.003
        @JvmField var velocityStdDev: Double = 0.1
        @JvmField var positionTimingJitterStdDev: Double = 0.0
    }

    /** (Re)build the model, controller, and EKF seeded from the given pose. */
    private fun build(initialPosRad: Double) {
        model = ArmModel(gains.kS, gains.kV, gains.kA, gains.kCos, gains.kSin)
        controller =
            MotorMechanismController(
                model,
                gains.kP,
                gains.kI,
                gains.kD,
                gains.maxVel,
                gains.maxAccel,
                gains.maxDecel,
                gains.maxJerk,
                gains.feedbackVoltageMargin,
                initialPosRad,
            )
        controller.setStaticFrictionTaperVelocity(gains.staticFrictionTaperVelocity)
        applyRestCompensation()
        ekf =
            MotorMechanismEkf(
                model,
                gains.velocityLagSec,
                gains.modelAccelStdDev,
                gains.positionStdDev,
                gains.velocityStdDev,
                gains.positionTimingJitterStdDev,
                initialPosRad,
            )
        lastPower = 0.0
    }

    /** Rebuild after a gain edit, reseeding controller + EKF from the current pose. */
    fun rebuild() {
        build(currentMeasuredPosRad())
    }

    override fun setPlant(plant: ArmPlant) {
        this.plant = plant
    }

    override fun setTargetRad(rad: Double) {
        this.targetRad = rad
    }

    /**
     * Rest compensation from the live plant's configured lash and rest compliance (both 0 on
     * rigid). Contact stiffness and flex are live-tunable without an adapter rebuild, so this is
     * re-applied every step, mirroring how Lineage A re-publishes its params each tick.
     */
    private fun applyRestCompensation() {
        controller.setBacklashCompensation(
            plant.getBacklashRad(),
            BACKLASH_TAPER_VOLTS,
            plant.restComplianceRadPerVolt(),
        )
    }

    override fun step(dt: Double, hubVoltage: Double) {
        applyRestCompensation()
        val posRad = currentMeasuredPosRad()
        val velRad = tpsToRadPerSec(plant.getVelocityTps())

        ekf.predict(dt, lastPower, hubVoltage)
        ekf.correct(posRad, velRad)

        val volts =
            controller.calculate(targetRad, ekf.getPosition(), ekf.getVelocity(), hubVoltage, dt)
        lastPower = volts / hubVoltage
    }

    override fun commandedPower(): Double = lastPower

    override fun profileTargetRad(): Double = controller.compensatedTarget(targetRad)

    override fun estimatedPosRad(): Double = ekf.getPosition()

    override fun estimatedVelRad(): Double = ekf.getVelocity()

    override fun trajPosRad(): Double = controller.getSetpointPosition()

    override fun trajVelRad(): Double = controller.getSetpointVelocity()

    override fun trajAccelRad(): Double = controller.getSetpointAcceleration()

    override fun modeLabel(): String = "MECHANISM_PIDF"

    private fun currentMeasuredPosRad(): Double = ticksToRad(plant.getPositionTicks())

    private fun ticksToRad(ticks: Int): Double =
        ticksToRad(ticks, ticksPerRev, gearRatio, encoderZeroOffsetRad)

    private fun tpsToRadPerSec(tps: Double): Double = tpsToRadPerSec(tps, ticksPerRev, gearRatio)

    companion object {
        /**
         * Gravity hold-voltage below which the rest-only backlash bias tapers to zero — too little
         * gravity to pin the load onto one tooth face (~5 deg from vertical at the default kG).
         */
        private const val BACKLASH_TAPER_VOLTS = 0.3

        // ── the isolated, unit-tested conversion seam ────────────────────────────────

        /** Convert motor encoder ticks to output-shaft radians from horizontal. */
        @JvmStatic
        fun ticksToRad(
            ticks: Int,
            ticksPerRev: Int,
            gearRatio: Double,
            encoderZeroOffsetRad: Double,
        ): Double = ticks * 2.0 * PI / (ticksPerRev * gearRatio) + encoderZeroOffsetRad

        /** Convert motor encoder ticks-per-second to output-shaft rad/s. */
        @JvmStatic
        fun tpsToRadPerSec(tps: Double, ticksPerRev: Int, gearRatio: Double): Double =
            tps * 2.0 * PI / (ticksPerRev * gearRatio)
    }
}
