package org.marsroboticsassociation.controllab.arm

import java.util.function.LongSupplier
import org.marsroboticsassociation.controllib.control.ArmController
import org.marsroboticsassociation.controllib.control.VerticalArmController
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Lineage A adapter: wraps [ArmController] (PD) or [VerticalArmController] (LQR), both of which
 * drive an [IMotor] port via `void update(dt)`. An inner `IMotor` view reads the current
 * [ArmPlant]'s encoder and stashes the controller's commanded power.
 *
 * The wrapped controllers seed their Kalman + trajectory from `motor.position` at construction, so
 * rebuilding this adapter automatically reseeds from the current arm pose (no jump home). Because
 * those controllers snap their estimate to the nearest hard stop the first time `setTarget` is
 * called while coasting, the target after a rebuild is re-applied via a latch *after* the next
 * `update()` — once the controller has established TRACKING mode mid-range, so a rebuild that
 * happens away from a hard stop does not cause a snap.
 */
internal class LineageAArmAdapter(
    private val type: ArmControllerType,
    plant: ArmPlant,
    private val clock: LongSupplier,
    private val hubVoltageNominal: Double,
) : ArmControlAdapter {
    private var plant: ArmPlant = plant
    private var lastPower = 0.0

    private var armPd: ArmController? = null
    private var armLqr: VerticalArmController? = null

    private var targetRad = 0.0
    private var hasTarget = false
    private var reapplyTargetPending = false

    /** IMotor view over whatever plant is currently installed. */
    private val motorView =
        object : IMotor {
            override val name: String
                get() = "arm"

            override val position: Int
                get() = plant.getPositionTicks()

            override val velocity: Double
                get() = plant.getVelocityTps()

            override fun setPower(power: Double) {
                lastPower = power
            }

            override val hubVoltage: Double
                get() = hubVoltageNominal

            override fun setVelocity(tps: Double) {}

            override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {}
        }

    init {
        require(type == ArmControllerType.ARM_PD || type == ArmControllerType.ARM_LQR) {
            "LineageAArmAdapter handles ARM_PD/ARM_LQR, got $type"
        }
        build()
    }

    /** (Re)construct the wrapped controller, reseeding its estimate from the current plant pose. */
    private fun build() {
        if (type == ArmControllerType.ARM_PD) {
            armPd = ArmController(motorView, NO_OP, clock)
            armLqr = null
        } else {
            armLqr = VerticalArmController(motorView, NO_OP, clock)
            armPd = null
        }
    }

    /**
     * Rebuild after a gain edit: reseed pose from the plant, re-apply the target after next update.
     */
    fun rebuild() {
        build()
        if (hasTarget) reapplyTargetPending = true
    }

    /**
     * Queue a target to be applied after the next `update()` rather than immediately, so a freshly
     * built controller establishes TRACKING mode mid-range before `setTarget` runs and avoids the
     * wake-from-coast hard-stop snap. Used by the engine right after (re)building.
     */
    fun deferTarget(rad: Double) {
        targetRad = rad
        hasTarget = true
        reapplyTargetPending = true
    }

    override fun setPlant(plant: ArmPlant) {
        this.plant = plant
    }

    override fun setTargetRad(rad: Double) {
        targetRad = rad
        hasTarget = true
        reapplyTargetPending = false
        val pd = armPd
        if (pd != null) {
            pd.setTarget(rad, hubVoltageNominal)
        } else {
            armLqr!!.setTarget(rad, hubVoltageNominal)
        }
    }

    override fun step(dt: Double, hubVoltage: Double) {
        val pd = armPd
        if (pd != null) {
            pd.update(dt, hubVoltage)
        } else {
            armLqr!!.update(dt, hubVoltage)
        }

        // Re-apply a target queued by a rebuild, now that the controller has run one cycle and
        // established its mode (TRACKING mid-range, avoiding the wake-from-coast hard-stop snap).
        if (reapplyTargetPending && hasTarget) {
            reapplyTargetPending = false
            if (pd != null) {
                pd.setTarget(targetRad, hubVoltageNominal)
            } else {
                armLqr!!.setTarget(targetRad, hubVoltageNominal)
            }
        }
    }

    override fun commandedPower(): Double = lastPower

    override fun profileTargetRad(): Double {
        val pd = armPd
        return if (pd != null) pd.getTargetAngleRad() else armLqr!!.getTargetAngleRad()
    }

    override fun estimatedPosRad(): Double {
        val pd = armPd
        return if (pd != null) pd.getEstimatedPositionRad() else armLqr!!.getEstimatedPositionRad()
    }

    override fun estimatedVelRad(): Double {
        val pd = armPd
        return if (pd != null) {
            pd.getEstimatedVelocityRadPerSec()
        } else {
            armLqr!!.getEstimatedVelocityRadPerSec()
        }
    }

    override fun trajPosRad(): Double {
        val pd = armPd
        return if (pd != null) pd.getTrajectoryPositionRad()
        else armLqr!!.getTrajectoryPositionRad()
    }

    override fun trajVelRad(): Double {
        val pd = armPd
        return if (pd != null) {
            pd.getTrajectoryVelocityRadPerSec()
        } else {
            armLqr!!.getTrajectoryVelocityRadPerSec()
        }
    }

    override fun modeLabel(): String {
        val mode = if (armPd != null) armPd!!.getMode().name else armLqr!!.getMode().name
        return type.name + " / " + mode
    }

    companion object {
        private val NO_OP = TelemetryAddData { _, _, _ -> }
    }
}
