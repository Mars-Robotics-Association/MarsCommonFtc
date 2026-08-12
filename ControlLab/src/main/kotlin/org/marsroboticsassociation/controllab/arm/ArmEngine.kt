package org.marsroboticsassociation.controllab.arm

import java.nio.file.Path
import java.util.Locale
import java.util.Random
import kotlin.math.max
import kotlin.math.min
import org.marsroboticsassociation.controllib.control.ArmController
import org.marsroboticsassociation.controllib.control.VerticalArmController

/**
 * Simulation engine for the Arm tab: owns the active [ArmPlant] and [ArmControlAdapter], a virtual
 * clock, and the [ArmMetrics]. Modeled on `FlywheelEngine` — each [tick] advances the clock, runs
 * the controller (which reads the plant and stashes power), then steps the plant with that power.
 *
 * The backlash plant is selected by default even though none of the controllers model backlash (or
 * flex); the [PlantKind] selector hot-swaps the plant mid-run (seeding the new plant from the
 * current load pose) so the degradation is visible instantly. Structural plant edits and plant
 * swaps also rebuild the controller adapter so the estimator/profile cannot keep flying with a
 * plant that was just reseeded at rest.
 *
 * **Lineage A params lock.** [ArmController.PARAMS] and [VerticalArmController.PARAMS] are
 * process-wide static bags. All Lineage A configure/build/tick work holds [LINEAGE_A_PARAMS_LOCK]
 * and re-publishes this engine's params before each step so concurrent engines (or tests) do not
 * silently share a half-written bag. Prefer a fixed [ArmEngine] RNG seed in tests for
 * reproducibility under dt jitter.
 */
class ArmEngine {
    /**
     * Which plant simulation is live. The controllers/estimators never know which one they face.
     */
    enum class PlantKind(private val label: String) {
        /** Single rigid inertia (`ArmMotorSim`). */
        RIGID("Rigid"),
        /** Two inertias across the gear-tooth dead band (`BacklashArmMotorSim`). */
        BACKLASH("Backlash"),
        /** Backlash plus a structural flex mode behind the lash (`FlexArmMotorSim`). */
        FLEX("Backlash + flex");

        override fun toString(): String = label
    }

    private val random: Random

    private val cfg = ArmPlantConfig()
    private var plantKind = PlantKind.BACKLASH

    private var plant: ArmPlant
    private var type: ArmControllerType
    private lateinit var adapter: ArmControlAdapter

    private var elapsedNanos = 0L
    private var elapsedSec = 0.0
    private var targetRad: Double

    private val metrics = ArmMetrics()

    /** Optional per-tick CSV log (GUI sessions only; headless tests leave it null). */
    private var recorder: ArmFlightRecorder? = null

    // --- Lineage A controller gains (editable) ---
    // Feedforward defaults match the (heavy) plant so the controllers know the model; the visible
    // ranking then comes from feedback structure, not model error.
    private var ffKs = 0.3
    private var ffKg = 3.5
    private var ffKv = 1.2
    private var ffKa = 0.35
    private var kP = 15.0
    private var kD = 1.0 // ARM_PD feedback
    private var lqrQPos = 0.5
    private var lqrQVel = 5.0
    private var lqrR = 12.0 // ARM_LQR weights

    // --- Lineage B (mechanism) gains ---
    private val mechGains = MechanismArmAdapter.Gains()

    /** Construct with a non-deterministic dt-jitter stream (interactive ControlLab use). */
    constructor(type: ArmControllerType) : this(type, System.nanoTime())

    /**
     * Construct with a fixed RNG seed so loop-dt jitter is reproducible (smoke tests, bisects).
     *
     * @param seed seed for the per-tick dt jitter generator
     */
    constructor(type: ArmControllerType, seed: Long) {
        this.random = Random(seed)
        this.type = type
        this.targetRad = cfg.maxAngleRad // park at the back hard stop (over-the-top end)
        this.plant = buildPlant(cfg.maxAngleRad)
        buildAdapter()
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Simulation loop
    // ─────────────────────────────────────────────────────────────────────────────

    fun tick() {
        // Always apply realistic loop timing (16 ms ± 4 ms). The profiler replans from its own
        // state every loop, so dt jitter does not need a special exactly-periodic mode.
        val dt = NOMINAL_DT + (random.nextDouble() - 0.5) * DT_JITTER
        elapsedNanos += (dt * 1e9).toLong()

        if (type == ArmControllerType.MECHANISM_PIDF) {
            stepControlAndPlant(dt)
        } else {
            // Re-publish this engine's params and hold the lock for the whole Lineage A step so
            // another engine cannot overwrite PARAMS mid-update.
            synchronized(LINEAGE_A_PARAMS_LOCK) {
                configureParams()
                stepControlAndPlant(dt)
            }
        }
    }

    private fun stepControlAndPlant(dt: Double) {
        adapter.step(dt, HUB_VOLTAGE)
        plant.step(dt, adapter.commandedPower(), HUB_VOLTAGE)
        elapsedSec += dt

        metrics.update(
            elapsedSec,
            plant.getTruePositionRad(),
            plant.getTrueVelocityRadPerSec(),
            plant.getMotorPositionRad(),
            plant.isEngaged(),
        )

        val rec = recorder
        if (rec != null) {
            rec.tick(
                elapsedSec,
                dt,
                targetRad,
                adapter.trajPosRad(),
                adapter.trajVelRad(),
                adapter.trajAccelRad(),
                adapter.estimatedPosRad(),
                adapter.estimatedVelRad(),
                plant.getTruePositionRad(),
                plant.getTrueVelocityRadPerSec(),
                plant.getMotorPositionRad(),
                plant.isEngaged(),
                adapter.commandedPower(),
            )
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Flight recorder (per-tick CSV log for offline analysis of live observations)
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Start logging every tick (plus session events) to a timestamped CSV in `dir`. Intended for
     * GUI sessions; headless tests should not enable it.
     *
     * @return the log file path, or `null` if the recorder could not be created
     */
    fun startFlightRecorder(dir: Path): Path? {
        return try {
            val rec = ArmFlightRecorder.createIn(dir)
            recorder = rec
            recordEvent(
                "session start: controller=$type plant=$plantKind mode=${adapter.modeLabel()}"
            )
            rec.getFile()
        } catch (e: java.io.IOException) {
            System.err.println("Flight recorder unavailable: $e")
            recorder = null
            null
        }
    }

    private fun recordEvent(description: String) {
        recorder?.event(description)
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Controller / plant selection
    // ─────────────────────────────────────────────────────────────────────────────

    private fun configureParams() {
        when (type) {
            ArmControllerType.ARM_PD -> ArmController.PARAMS = armParams()
            ArmControllerType.ARM_LQR -> VerticalArmController.PARAMS = verticalParams()
            ArmControllerType.MECHANISM_PIDF -> {
                // MECHANISM carries its gains in mechGains; nothing static to set.
            }
        }
    }

    private fun armParams(): ArmController.Params {
        val p = ArmController.Params()
        p.ticksPerRev = cfg.ticksPerRev
        p.gearRatio = cfg.gearRatio
        p.encoderZeroOffsetRad = cfg.encoderZeroOffsetRad
        p.minAngleRad = cfg.minAngleRad
        p.maxAngleRad = cfg.maxAngleRad
        p.ks = ffKs
        p.kg = ffKg
        p.kv = ffKv
        p.ka = ffKa
        p.kP = kP
        p.kD = kD
        p.maxVelRad = 3.0
        p.maxAccelRad = 6.0
        p.maxDecelRad = 8.0
        p.maxJerkRad = 30.0
        // Rest compensation from the live plant's configured lash and compliance (0 on rigid).
        p.backlashRad = plant.getBacklashRad()
        p.restComplianceRadPerVolt = plant.restComplianceRadPerVolt()
        return p
    }

    private fun verticalParams(): VerticalArmController.Params {
        val p = VerticalArmController.Params()
        p.ticksPerRev = cfg.ticksPerRev
        p.gearRatio = cfg.gearRatio
        p.encoderZeroOffsetRad = cfg.encoderZeroOffsetRad
        p.minAngleRad = cfg.minAngleRad
        p.maxAngleRad = cfg.maxAngleRad
        p.ks = ffKs
        p.kg = ffKg
        p.kv = ffKv
        p.ka = ffKa
        p.qPosition = lqrQPos
        p.qVelocity = lqrQVel
        p.rVoltage = lqrR
        p.maxVelRad = 3.0
        p.maxAccelRad = 6.0
        p.maxDecelRad = 8.0
        p.maxJerkRad = 30.0
        // Rest compensation from the live plant's configured lash and compliance (0 on rigid).
        p.backlashRad = plant.getBacklashRad()
        p.restComplianceRadPerVolt = plant.restComplianceRadPerVolt()
        return p
    }

    private fun buildAdapter() {
        if (type == ArmControllerType.MECHANISM_PIDF) {
            val m =
                MechanismArmAdapter(
                    mechGains,
                    plant,
                    cfg.ticksPerRev,
                    cfg.gearRatio,
                    cfg.encoderZeroOffsetRad,
                )
            m.setTargetRad(targetRad)
            adapter = m
            return
        }
        // Lineage A constructors and live kP/kD reads both touch the static PARAMS bags.
        synchronized(LINEAGE_A_PARAMS_LOCK) {
            configureParams()
            val a = LineageAArmAdapter(type, plant, { elapsedNanos }, HUB_VOLTAGE)
            a.deferTarget(targetRad)
            adapter = a
        }
    }

    /**
     * Rebuild the controller adapter from the current plant pose and re-apply the active target.
     * Used after any structural plant change that zeros plant velocity or swaps the sim instance,
     * so the estimator/profile cannot keep a stale flying state against a plant that just reseeded
     * at rest.
     */
    private fun reseedAdapterFromPlant() {
        buildAdapter()
    }

    fun setControllerType(newType: ArmControllerType) {
        if (newType == type) return
        this.type = newType
        buildAdapter() // reseeds from current plant pose; target re-applied without a jump
        recordEvent("controller=$newType mode=${adapter.modeLabel()}")
    }

    fun getControllerType(): ArmControllerType = type

    private fun buildPlant(initialAngleRad: Double): ArmPlant =
        when (plantKind) {
            PlantKind.RIGID -> RigidArmPlant(cfg, initialAngleRad)
            PlantKind.FLEX -> FlexArmPlant(cfg, initialAngleRad)
            PlantKind.BACKLASH -> BacklashArmPlant(cfg, initialAngleRad)
        }

    /**
     * Hot-swap the plant simulation, seeding the new plant from the current load pose (no jump).
     */
    fun setPlantKind(kind: PlantKind) {
        if (kind == plantKind) return
        plantKind = kind
        // The sims seed at rest, so the pose is preserved but velocity resets to zero on swap.
        plant = buildPlant(plant.getTruePositionRad())
        reseedAdapterFromPlant()
        recordEvent("plant=$kind")
    }

    fun getPlantKind(): PlantKind = plantKind

    /** Boolean shorthand for [setPlantKind], selecting only between BACKLASH and RIGID. */
    fun setBacklashEnabled(enabled: Boolean) {
        setPlantKind(if (enabled) PlantKind.BACKLASH else PlantKind.RIGID)
    }

    /** True when the live plant has a gearbox dead band (backlash or flex). */
    fun isBacklashEnabled(): Boolean = plantKind != PlantKind.RIGID

    // ─────────────────────────────────────────────────────────────────────────────
    // Targets
    // ─────────────────────────────────────────────────────────────────────────────

    fun setTargetRad(rad: Double) {
        targetRad = clampAngle(rad)
        adapter.setTargetRad(targetRad)
        metrics.onTargetChanged(targetRad, plant.getTruePositionRad(), elapsedSec)
        recordEvent(String.format(Locale.US, "target=%.2fdeg", Math.toDegrees(targetRad)))
    }

    fun setTargetDeg(deg: Double) {
        setTargetRad(Math.toRadians(deg))
    }

    private fun clampAngle(rad: Double): Double = max(cfg.minAngleRad, min(cfg.maxAngleRad, rad))

    // ─────────────────────────────────────────────────────────────────────────────
    // Live controller-gain edits
    // ─────────────────────────────────────────────────────────────────────────────

    fun setFeedforwardGains(ks: Double, kg: Double, kv: Double, ka: Double) {
        ffKs = ks
        ffKg = kg
        ffKv = kv
        ffKa = ka
        if (type == ArmControllerType.ARM_PD || type == ArmControllerType.ARM_LQR) {
            synchronized(LINEAGE_A_PARAMS_LOCK) {
                configureParams()
                (adapter as LineageAArmAdapter).rebuild() // FF/kalman baked at construction
            }
        }
    }

    fun setPdGains(kP: Double, kD: Double) {
        this.kP = kP
        this.kD = kD
        if (type == ArmControllerType.ARM_PD) {
            // ArmController reads PARAMS.kP/kD live, so mutate in place (no rebuild).
            synchronized(LINEAGE_A_PARAMS_LOCK) {
                ArmController.PARAMS.kP = kP
                ArmController.PARAMS.kD = kD
            }
        }
    }

    fun setLqrWeights(qPos: Double, qVel: Double, r: Double) {
        this.lqrQPos = qPos
        this.lqrQVel = qVel
        this.lqrR = r
        if (type == ArmControllerType.ARM_LQR) {
            synchronized(LINEAGE_A_PARAMS_LOCK) {
                configureParams()
                (adapter as LineageAArmAdapter).rebuild() // LQR gains baked at construction
            }
        }
    }

    fun setMechanismGains(
        kP: Double,
        kI: Double,
        kD: Double,
        kS: Double,
        kV: Double,
        kA: Double,
        kCos: Double,
        kSin: Double,
        maxVel: Double,
        maxAccel: Double,
        maxDecel: Double,
        maxJerk: Double,
    ) {
        mechGains.kP = kP
        mechGains.kI = kI
        mechGains.kD = kD
        mechGains.kS = kS
        mechGains.kV = kV
        mechGains.kA = kA
        mechGains.kCos = kCos
        mechGains.kSin = kSin
        mechGains.maxVel = maxVel
        mechGains.maxAccel = maxAccel
        mechGains.maxDecel = maxDecel
        mechGains.maxJerk = maxJerk
        if (type == ArmControllerType.MECHANISM_PIDF) {
            (adapter as MechanismArmAdapter).rebuild() // controller + EKF reseeded from pose
        }
        recordEvent(
            String.format(
                Locale.US,
                "mechGains kP=%.2f kI=%.2f kD=%.3f kS=%.3f kV=%.3f kA=%.4f kCos=%.3f kSin=%.3f" +
                    " vMax=%.2f aMax=%.2f dMax=%.2f jMax=%.1f",
                kP,
                kI,
                kD,
                kS,
                kV,
                kA,
                kCos,
                kSin,
                maxVel,
                maxAccel,
                maxDecel,
                maxJerk,
            )
        )
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Live plant-param edits
    // ─────────────────────────────────────────────────────────────────────────────

    /** Structural plant dynamics: rebuild the plant at the current pose. */
    fun setPlantDynamics(kS: Double, kG: Double, kV: Double, kA: Double) {
        cfg.kS = kS
        cfg.kG = kG
        cfg.kV = kV
        cfg.kA = kA
        reseedPlantInPlace()
        recordEvent(
            String.format(
                Locale.US,
                "plantDynamics kS=%.3f kG=%.3f kV=%.3f kA=%.4f",
                kS,
                kG,
                kV,
                kA,
            )
        )
    }

    /** Structural: rebuild the plant at the current pose with the new backlash. */
    fun setBacklashRad(rad: Double) {
        cfg.backlashRad = rad
        reseedPlantInPlace()
    }

    fun setContact(stiffness: Double, damping: Double) {
        cfg.contactStiffness = stiffness
        cfg.contactDamping = damping
        plant.applyLiveParams()
    }

    fun setLoadFriction(viscous: Double, staticVolts: Double) {
        cfg.loadViscousFriction = viscous
        cfg.loadStaticFriction = staticVolts
        plant.applyLiveParams()
    }

    /** Arm structural flex mode (used by the flex plant only; harmless to set on the others). */
    fun setFlexParams(hz: Double, zeta: Double) {
        cfg.flexHz = hz
        cfg.flexZeta = zeta
        plant.applyLiveParams()
    }

    fun setDisturbanceVoltage(v: Double) {
        cfg.disturbanceVoltage = v
        plant.applyLiveParams()
    }

    /** Structural: swap the encoder read-timing model (rebuilds the plant at the current pose). */
    fun setEncoderKind(kind: ArmPlantConfig.EncoderKind) {
        cfg.encoderKind = kind
        reseedPlantInPlace()
    }

    private fun reseedPlantInPlace() {
        val loadRad = plant.getTruePositionRad()
        val loadVel = plant.getTrueVelocityRadPerSec()
        plant.seedFrom(loadRad, loadVel)
        // Sims always seed at rest; rebuild the controller so the profile/EKF match.
        reseedAdapterFromPlant()
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // System identification
    // ─────────────────────────────────────────────────────────────────────────────

    /**
     * Run the simulated sysid against the currently-selected plant. On the backlash and flex plants
     * it identifies through the motor-side encoder — the realistic case for a real robot with an
     * unavoidable lashy (and flexy) drivetrain. The routine's steady one-directional runs keep the
     * gear teeth engaged and the flex spring quasi-static, so the motor encoder sees the full
     * coupled inertia and gravity.
     */
    fun runSysId(): ArmSysId.Result = ArmSysId.characterize(cfg, plantKind)

    /**
     * Push identified feedforward gains into both controller models (Lineage-A feedforward and the
     * mechanism model) and rebuild the active controller so they take effect immediately.
     */
    fun applyIdentifiedGains(r: ArmSysId.Result) {
        ffKs = r.kS
        ffKg = r.kCos
        ffKv = r.kV
        ffKa = r.kA
        mechGains.kS = r.kS
        mechGains.kV = r.kV
        mechGains.kA = r.kA
        mechGains.kCos = r.kCos
        mechGains.kSin = r.kSin
        val event =
            StringBuilder(
                String.format(
                    Locale.US,
                    "sysid applied kS=%.3f kV=%.3f kA=%.4f kCos=%.3f kSin=%.3f",
                    r.kS,
                    r.kV,
                    r.kA,
                    r.kCos,
                    r.kSin,
                )
            )
        if (!r.kVHold.isNaN()) {
            event.append(String.format(Locale.US, " kVhold=%.3f kVrun=%.3f", r.kVHold, r.kVRun))
        }
        if (!r.halfLashRad.isNaN()) {
            event.append(
                String.format(Locale.US, " halfLash=%.2fdeg", Math.toDegrees(r.halfLashRad))
            )
        }
        if (r.kVDisagreement() > 0.10) {
            event.append(" [warn: hold/run kV disagree - flex/lash contamination of runs]")
        }
        recordEvent(event.toString())
        when (type) {
            ArmControllerType.ARM_PD,
            ArmControllerType.ARM_LQR -> {
                synchronized(LINEAGE_A_PARAMS_LOCK) {
                    configureParams()
                    (adapter as LineageAArmAdapter).rebuild()
                }
            }
            ArmControllerType.MECHANISM_PIDF -> (adapter as MechanismArmAdapter).rebuild()
        }
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Reset
    // ─────────────────────────────────────────────────────────────────────────────

    fun reset() {
        elapsedNanos = 0
        elapsedSec = 0.0
        targetRad = cfg.maxAngleRad
        plant = buildPlant(cfg.maxAngleRad)
        buildAdapter()
        metrics.reset()
        recordEvent("reset")
    }

    // ─────────────────────────────────────────────────────────────────────────────
    // Accessors for the canvas / chart / metrics
    // ─────────────────────────────────────────────────────────────────────────────

    fun getElapsedSec(): Double = elapsedSec

    fun getTargetRad(): Double = targetRad

    /** The endpoint the profile actually drives to (stated target plus any backlash bias). */
    fun getProfileTargetRad(): Double = adapter.profileTargetRad()

    fun getTrueLoadRad(): Double = plant.getTruePositionRad()

    fun getTrueLoadVelRad(): Double = plant.getTrueVelocityRadPerSec()

    fun getMotorRad(): Double = plant.getMotorPositionRad()

    fun isEngaged(): Boolean = plant.isEngaged()

    fun getBacklashRad(): Double = plant.getBacklashRad()

    fun getMinAngleRad(): Double = cfg.minAngleRad

    fun getMaxAngleRad(): Double = cfg.maxAngleRad

    /** Motor-side encoder angle the controller actually sees (ticks -> rad). */
    fun getMeasuredEncoderRad(): Double =
        plant.getPositionTicks() / cfg.ticksPerRad() + cfg.encoderZeroOffsetRad

    fun getEstimatedPosRad(): Double = adapter.estimatedPosRad()

    fun getEstimatedVelRad(): Double = adapter.estimatedVelRad()

    fun getTrajPosRad(): Double = adapter.trajPosRad()

    fun getTrajVelRad(): Double = adapter.trajVelRad()

    fun getTrajAccelRad(): Double = adapter.trajAccelRad()

    fun getCommandedPower(): Double = adapter.commandedPower()

    fun getModeLabel(): String = adapter.modeLabel()

    fun getMetrics(): ArmMetrics = metrics

    // Editable-param initial values for the sidebar
    fun getFfKs(): Double = ffKs

    fun getFfKg(): Double = ffKg

    fun getFfKv(): Double = ffKv

    fun getFfKa(): Double = ffKa

    fun getKP(): Double = kP

    fun getKD(): Double = kD

    fun getLqrQPos(): Double = lqrQPos

    fun getLqrQVel(): Double = lqrQVel

    fun getLqrR(): Double = lqrR

    fun getMechGains(): MechanismArmAdapter.Gains = mechGains

    fun getPlantKs(): Double = cfg.kS

    fun getPlantKg(): Double = cfg.kG

    fun getPlantKv(): Double = cfg.kV

    fun getPlantKa(): Double = cfg.kA

    fun getBacklashRadCfg(): Double = cfg.backlashRad

    fun getContactStiffness(): Double = cfg.contactStiffness

    fun getContactDamping(): Double = cfg.contactDamping

    fun getLoadViscous(): Double = cfg.loadViscousFriction

    fun getLoadStatic(): Double = cfg.loadStaticFriction

    fun getFlexHz(): Double = cfg.flexHz

    fun getFlexZeta(): Double = cfg.flexZeta

    fun getDisturbanceVoltage(): Double = cfg.disturbanceVoltage

    fun getEncoderKind(): ArmPlantConfig.EncoderKind = cfg.encoderKind

    companion object {
        @JvmField val HUB_VOLTAGE: Double = 12.0
        private const val NOMINAL_DT = 0.016 // 16 ms loop, like FlywheelEngine
        private const val DT_JITTER = 0.008 // +/- 4 ms

        /**
         * Guards all reads/writes of the Lineage A static `PARAMS` bags used by [ArmController] and
         * [VerticalArmController].
         */
        @JvmField internal val LINEAGE_A_PARAMS_LOCK: Any = Any()
    }
}
