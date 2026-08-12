package org.marsroboticsassociation.controllab.flywheel

import java.util.Random
import java.util.function.LongSupplier
import kotlin.math.max
import org.marsroboticsassociation.controllib.control.FlywheelSimple
import org.marsroboticsassociation.controllib.control.FlywheelStateSpace
import org.marsroboticsassociation.controllib.control.VelocityMotorPF
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.motion.SCurveVelocity
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/** Simulation engine that drives a flywheel motor sim and its controller. */
class FlywheelEngine(private var type: FlywheelControllerType) : IMotor {

    private val random = Random()

    private var sim: FlywheelMotorSim

    // Controllers
    private var simple: FlywheelSimple? = null
    private var pf: VelocityMotorPF? = null
    private var pfConfig: VelocityMotorPF.VelocityMotorPFConfig? = null
    private var ss: FlywheelStateSpace? = null

    // Controller Params
    private var kV = 12.5 / 2632.1
    private var kA = 12.5 / 2087.9
    private var kS = 0.8931
    private var kP = 0.010
    private var velLpfCutoffHz = 4.0

    // FlywheelSimple-specific params (NaN = use PARAMS default)
    private var simpleMaxAccel = Double.NaN

    // VelocityMotorPF-specific params (NaN = use config default)
    private var pfAccelMax = Double.NaN
    private var pfJerkIncreasing = Double.NaN
    private var pfJerkDecreasing = Double.NaN

    // FlywheelStateSpace-specific params
    private var ssModelStdDev = FlywheelStateSpace.PARAMS.modelStdDevRadPerSec
    private var ssMeasurementStdDev = FlywheelStateSpace.PARAMS.measurementStdDevRadPerSec

    // Physical Plant Params (the "Real Robot")
    private var plantKV = 12.5 / 2632.1
    private var plantKA = 12.5 / 2087.9
    private var plantKS = 0.8931

    private var targetTps = 0.0
    private var currentPower = 0.0
    private var elapsedNanos = 0L
    private var elapsedSec = 0.0

    private val noOp = TelemetryAddData { _, _, _ -> }

    init {
        this.sim = FlywheelMotorSim(plantKV, plantKA)
        this.sim.setDisturbanceVoltage(-plantKS)
        rebuildController()
    }

    fun setParams(kV: Double, kA: Double, kS: Double, kP: Double, velLpfCutoffHz: Double) {
        this.kV = kV
        this.kA = kA
        this.kS = kS
        this.kP = kP
        this.velLpfCutoffHz = velLpfCutoffHz

        // Apply to existing controllers if possible
        if (simple != null) {
            FlywheelSimple.PARAMS.kV = kV
            FlywheelSimple.PARAMS.kA = kA
            FlywheelSimple.PARAMS.kS = kS
            FlywheelSimple.PARAMS.kP = kP
            FlywheelSimple.PARAMS.velLpfCutoffHz = velLpfCutoffHz
        }
        val cfg = pfConfig
        if (pf != null && cfg != null) {
            cfg.kV = kV
            cfg.kA = kA
            cfg.kS = kS
            cfg.kP = kP
            cfg.measurementLpfCutoffHz = velLpfCutoffHz
        }
        if (ss != null) {
            FlywheelStateSpace.PARAMS.kV = kV * 28 / (2 * Math.PI) // Convert to rad/s
            FlywheelStateSpace.PARAMS.kA = max(kA, 1e-6) * 28 / (2 * Math.PI)
            rebuildController() // StateSpace plant is defined at construction, so we rebuild.
        }
    }

    fun setPlantParams(plantKV: Double, plantKA: Double, plantKS: Double) {
        this.plantKV = plantKV
        this.plantKA = plantKA
        this.plantKS = plantKS
        // Update the physical simulation plant
        this.sim =
            FlywheelMotorSim(
                plantKV,
                max(plantKA, 1e-6),
                sim.getPositionTicks(),
                sim.getTrueVelocityTps(),
            )
        this.sim.setDisturbanceVoltage(-plantKS)
    }

    fun getPlantKV(): Double = plantKV

    fun getPlantKA(): Double = plantKA

    fun getPlantKS(): Double = plantKS

    private fun rebuildController() {
        when (type) {
            FlywheelControllerType.FLYWHEEL_SIMPLE -> {
                FlywheelSimple.PARAMS.kV = kV
                FlywheelSimple.PARAMS.kA = kA
                FlywheelSimple.PARAMS.kS = kS
                FlywheelSimple.PARAMS.kP = kP
                FlywheelSimple.PARAMS.velLpfCutoffHz = velLpfCutoffHz
                if (!simpleMaxAccel.isNaN()) FlywheelSimple.PARAMS.maxAccel = simpleMaxAccel
                simple = FlywheelSimple(noOp, LongSupplier { elapsedNanos }, this)
                pf = null
                pfConfig = null
                ss = null
            }
            FlywheelControllerType.VELOCITY_MOTOR_PF -> {
                val config = VelocityMotorPF.VelocityMotorPFConfig()
                config.kV = kV
                config.kA = kA
                config.kS = kS
                config.kP = kP
                config.measurementLpfCutoffHz = velLpfCutoffHz
                if (!pfAccelMax.isNaN()) config.accelMax = pfAccelMax
                if (!pfJerkIncreasing.isNaN()) config.jerkIncreasing = pfJerkIncreasing
                if (!pfJerkDecreasing.isNaN()) config.jerkDecreasing = pfJerkDecreasing
                pfConfig = config
                pf =
                    VelocityMotorPF(
                        noOp,
                        1.0,
                        28.0,
                        0.01,
                        config,
                        this,
                        LongSupplier { elapsedNanos },
                    )
                simple = null
                ss = null
            }
            FlywheelControllerType.FLYWHEEL_STATE_SPACE -> {
                FlywheelStateSpace.PARAMS.kV = kV * 28 / (2 * Math.PI)
                FlywheelStateSpace.PARAMS.kA = max(kA, 1e-6) * 28 / (2 * Math.PI)
                FlywheelStateSpace.PARAMS.modelStdDevRadPerSec = ssModelStdDev
                FlywheelStateSpace.PARAMS.measurementStdDevRadPerSec = ssMeasurementStdDev
                ss = FlywheelStateSpace(this, noOp)
                simple = null
                pf = null
                pfConfig = null
            }
        }
    }

    fun setType(type: FlywheelControllerType) {
        if (this.type == type) return
        this.type = type
        rebuildController()
        setTarget(targetTps)
    }

    fun setTarget(tps: Double) {
        this.targetTps = tps
        simple?.setTps(tps)
        pf?.setTPS(tps)
        ss?.setTps(tps)
    }

    fun tick() {
        // Simulate real-world jitter: 16ms +/- 4ms
        val dt = NOMINAL_DT + (random.nextDouble() - 0.5) * 0.008
        elapsedNanos += (dt * 1e9).toLong()

        // 1. Controller update (uses simulated motor readings)
        simple?.update()
        pf?.update(dt)
        ss?.update(dt, 12.0)

        // 2. Sim plant update
        sim.step(dt, currentPower, 12.0)
        elapsedSec += dt
    }

    // --- IMotor implementation ---

    override val position: Int
        get() = sim.getPositionTicks()

    override val velocity: Double
        get() = sim.getVelocityTps()

    override fun setPower(power: Double) {
        this.currentPower = power
    }

    override val hubVoltage: Double
        get() = 12.0

    override val name: String
        get() = "flywheel"

    override fun setVelocity(tps: Double) {
        setTarget(tps)
    }

    override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {
        // Not used in this sim
    }

    fun getPositionTicks(): Int {
        return sim.getPositionTicks()
    }

    // --- Accessors for chart ---

    fun getMeasuredVelocity(): Double {
        return sim.getVelocityTps()
    }

    fun getTrueVelocity(): Double {
        return sim.getTrueVelocityTps()
    }

    fun getProfiledVelocity(): Double {
        simple?.let {
            return it.getProfiledVelocity()
        }
        pf?.let {
            return it.getProfiledVelocity()
        }
        ss?.let {
            return targetTps // SS doesn't have a motion profile in this version
        }
        return 0.0
    }

    fun getFilteredVelocity(): Double {
        simple?.let {
            return it.getFilteredVelocity()
        }
        pf?.let {
            return it.getTpsFiltered()
        }
        ss?.let {
            return it.getEstimatedTps()
        }
        return 0.0
    }

    fun getPower(): Double {
        return currentPower
    }

    fun getTarget(): Double {
        return targetTps
    }

    fun getElapsedSec(): Double {
        return elapsedSec
    }

    fun getKV(): Double = kV

    fun getKA(): Double = kA

    fun getKS(): Double = kS

    fun getKP(): Double = kP

    fun getVelLpfCutoffHz(): Double = velLpfCutoffHz

    fun setSimpleParams(maxAccel: Double) {
        if (!maxAccel.isNaN()) this.simpleMaxAccel = maxAccel
        if (simple != null && !this.simpleMaxAccel.isNaN()) {
            FlywheelSimple.PARAMS.maxAccel = this.simpleMaxAccel
        }
    }

    fun getSimpleMaxAccel(): Double {
        return if (simple != null) FlywheelSimple.PARAMS.maxAccel else simpleMaxAccel
    }

    fun setPFParams(accelMax: Double, jerkIncreasing: Double, jerkDecreasing: Double) {
        this.pfAccelMax = accelMax
        this.pfJerkIncreasing = jerkIncreasing
        this.pfJerkDecreasing = jerkDecreasing
        val cfg = pfConfig
        if (pf != null && cfg != null) {
            cfg.accelMax = accelMax
            cfg.jerkIncreasing = jerkIncreasing
            cfg.jerkDecreasing = jerkDecreasing
        }
    }

    fun getPFBAccelMax(): Double {
        val cfg = pfConfig
        return if (pf != null && cfg != null) cfg.accelMax
        else if (!pfAccelMax.isNaN()) pfAccelMax else 2500.0
    }

    fun getPFJerkIncreasing(): Double {
        val cfg = pfConfig
        return if (pf != null && cfg != null) cfg.jerkIncreasing
        else if (!pfJerkIncreasing.isNaN()) pfJerkIncreasing else 2000.0
    }

    fun getPFJerkDecreasing(): Double {
        val cfg = pfConfig
        return if (pf != null && cfg != null) cfg.jerkDecreasing
        else if (!pfJerkDecreasing.isNaN()) pfJerkDecreasing else 1000.0
    }

    fun setSSParams(modelStdDev: Double, measurementStdDev: Double) {
        this.ssModelStdDev = modelStdDev
        this.ssMeasurementStdDev = measurementStdDev
        if (ss != null) {
            FlywheelStateSpace.PARAMS.modelStdDevRadPerSec = modelStdDev
            FlywheelStateSpace.PARAMS.measurementStdDevRadPerSec = measurementStdDev
            rebuildController()
            ss!!.setTps(targetTps)
        }
    }

    fun getSSModelStdDev(): Double = ssModelStdDev

    fun getSSMeasurementStdDev(): Double = ssMeasurementStdDev

    fun reset() {
        sim.reset(0.0)
        elapsedSec = 0.0
        elapsedNanos = 0
        simple?.setTps(0.0)
        pf?.stop()
        ss?.setTps(0.0)
    }

    /**
     * Randomize plant, zero tuning params, auto-tune profile, reset sim. Profile is tuned for 0 to
     * 75% of max achievable velocity.
     */
    fun newChallenge() {
        val jInc = if (pfJerkIncreasing.isNaN()) 2000.0 else pfJerkIncreasing
        val voltage = 12.0

        // Re-randomize until falling jerk is reasonable (>= 200)
        var aMax: Double
        var jDec: Double
        do {
            // 1. Randomize plant params (FTC-reasonable ranges)
            // Upper bound: GoBilda 5203 1:1 (fastest common FTC motor): 6000 RPM × 28 PPR / 60 =
            // 2800 TPS
            val maxTps = 1000 + random.nextDouble() * 1800 // 1000..2800 TPS
            this.plantKV = 12.5 / maxTps
            // Upper bound: GoBilda 5203 1:1 stall torque 1.47 kg·cm → at R=1.3Ω, light flywheel
            // load
            // gives ~4000 TPS/s²
            val accelDenom = 800 + random.nextDouble() * 3200 // 800..4000 TPS/s²
            this.plantKA = 12.5 / accelDenom
            // GoBilda 5203 1:1 free current 0.25A × R 1.3Ω ≈ 0.33V lower bound; heavy friction
            // motors ~1.5V upper bound
            this.plantKS = 0.3 + random.nextDouble() * 1.2 // 0.3..1.5 V

            // 2. Auto-tune profile params: 0 to 75% of max achievable velocity
            val maxVelocity = (voltage - plantKS) / plantKV
            val v1 = 0.75 * maxVelocity
            aMax = SCurveVelocity.findMaxAMax(0.0, v1, jInc, voltage, plantKS, plantKV, plantKA)
            jDec =
                SCurveVelocity.findMaxJDec(
                    0.0,
                    v1,
                    0.0,
                    aMax,
                    jInc,
                    voltage,
                    plantKS,
                    plantKV,
                    plantKA,
                )
        } while (!(jDec >= 200)) // retry if NaN (infeasible) or below threshold

        // 3. Rebuild sim with new plant
        this.sim = FlywheelMotorSim(plantKV, plantKA)
        this.sim.setDisturbanceVoltage(-plantKS)

        // 4. Zero tuning params (leave cutoff alone)
        this.kS = 0.0
        this.kV = 0.0
        this.kA = 0.0
        this.kP = 0.0

        this.pfAccelMax = aMax
        this.pfJerkDecreasing = jDec

        // 5. Reset sim and rebuild controller with zeroed params
        this.elapsedSec = 0.0
        this.elapsedNanos = 0
        this.currentPower = 0.0
        rebuildController()
        setTarget(0.0) // propagate zero target to newly built controller
    }

    companion object {
        private const val NOMINAL_DT = 0.016 // 16 ms
    }
}
