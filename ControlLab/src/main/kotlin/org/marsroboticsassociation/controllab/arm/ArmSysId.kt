package org.marsroboticsassociation.controllab.arm

import java.util.function.DoubleSupplier
import java.util.function.IntSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import org.marsroboticsassociation.controllib.sim.ArmMotorSim
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim
import org.marsroboticsassociation.controllib.sim.EncoderSim
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim

/**
 * Simulated system-identification for the arm. It drives a fresh plant built from the config, logs
 * the motor-side encoder, and recovers the arm feedforward model via
 * [org.marsroboticsassociation.controllib.mechanism.ArmSysId].
 *
 * ```
 *   V = kS*sign(w) + kV*w + kA*(dw/dt) + kCos*cos(theta) + kSin*sin(theta)
 * ```
 *
 * **Two-stage battery.** Friction and gravity (`kS, kV, kCos, kSin`) come from constant-velocity
 * holds and inertia (`kA`) from constant-power runs, combined by
 * [org.marsroboticsassociation.controllib.mechanism.ArmSysId.solveTwoStage]. The holds are what
 * make `kS` survive a lashy/flexy drivetrain: at a steady speed the acceleration is ~ 0, so the
 * flex sits at its quasi-static deflection and the direction-flipping flex/lash bias that a
 * hard-accelerating run pumps into `kS` is starved. (A single-stage constant-power sysid through
 * the lash + flex instead inflates `kS`, and that over-estimate is an anti-braking feedforward term
 * that reintroduces arrival overshoot.) The holds here are driven by a PI velocity loop over the
 * sim; on a real robot the `ArmSysIdTuning` OpMode drives the same holds and feeds
 * [org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateHold] the same angle +
 * voltage log.
 *
 * **Plant choice.** [characterize] uses a rigid [ArmMotorSim]. [characterize] with a
 * [ArmEngine.PlantKind] selects the two-inertia [BacklashArmMotorSim] or the three-inertia
 * [FlexArmMotorSim] and logs the *motor-side* encoder — the realistic case for a robot with an
 * unavoidable lashy (and flexy) drivetrain.
 */
class ArmSysId private constructor() {
    /**
     * Identified feedforward coefficients plus a fit-quality score.
     *
     * Delegates field layout to [org.marsroboticsassociation.controllib.mechanism.ArmSysId.Result]
     * so ControlLab and the on-robot OpMode share one type family; this thin wrapper keeps existing
     * ControlLab call sites (`ArmSysId.Result`) compiling.
     */
    class Result
    internal constructor(r: org.marsroboticsassociation.controllib.mechanism.ArmSysId.Result) {
        @JvmField val kS: Double = r.kS
        @JvmField val kV: Double = r.kV
        @JvmField val kA: Double = r.kA
        @JvmField val kCos: Double = r.kCos
        @JvmField val kSin: Double = r.kSin
        @JvmField val rSquared: Double = r.rSquared
        @JvmField val samples: Int = r.samples
        /** Hold-stage kV (quasi-static, flex-immune); NaN when the holds could not pin it. */
        @JvmField val kVHold: Double = r.kVHold
        /** Run-stage kV cross-check from the moving fit. */
        @JvmField val kVRun: Double = r.kVRun
        /**
         * Backlash half-angle (rad) implied by the direction-split hold fit; NaN if unavailable.
         */
        @JvmField val halfLashRad: Double = r.halfLashRad

        /** |kVHold − kVRun| / kV; beyond ~10% flags flex/lash contamination of the moving runs. */
        fun kVDisagreement(): Double = abs(kVHold - kVRun) / max(abs(kV), 1e-9)
    }

    /** A uniform driver over the three plant sims, in the mechanism's motor-encoder frame. */
    private interface SimHandle {
        fun step(dt: Double, power: Double)

        fun posRad(): Double

        fun velRad(): Double
    }

    /** A sim's `step(dt, power, hubVoltage)`. */
    private fun interface Stepper {
        fun step(dt: Double, power: Double, hubVoltage: Double)
    }

    companion object {
        private const val DT = 0.01 // 100 Hz logging
        private const val RUN_STEPS = 90 // 0.9 s per run
        private val HUB = ArmEngine.HUB_VOLTAGE

        // Constant-velocity holds (identify kS/kV/kCos/kSin quasi-statically; see velocitySweep).
        private val SWEEP_SPEEDS = doubleArrayOf(0.75, 1.5, 2.25, 3.0) // rad/s, both directions
        private const val SWEEP_MAX_STEPS = 500 // 5 s cap per hold
        private const val SWEEP_KP = 8.0 // velocity-hold P gain, V per rad/s
        // warmup so the hold reaches speed early and sweeps a wide angle band (heavy inertia
        // otherwise
        // burns the whole travel getting up to the higher speeds); steady samples are picked by
        // accumulateHold's acceleration gate, so a bit of warmup in the log is harmless
        private const val SWEEP_KI = 20.0 // velocity-hold I gain, V per rad

        /** Run the characterization against a rigid plant built from `cfg`. */
        @JvmStatic fun characterize(cfg: ArmPlantConfig): Result = characterize(cfg, false)

        /**
         * Run the characterization against the plant built from `cfg`, optionally through the
         * two-inertia backlash plant (i.e. logging the motor-side encoder, exactly what a real
         * robot with an unavoidable lashy gearbox exposes) rather than a rigid plant.
         *
         * @param throughBacklash true to identify through the backlash plant's motor encoder
         */
        @JvmStatic
        fun characterize(cfg: ArmPlantConfig, throughBacklash: Boolean): Result =
            characterize(
                cfg,
                if (throughBacklash) ArmEngine.PlantKind.BACKLASH else ArmEngine.PlantKind.RIGID,
            )

        /**
         * Run the characterization against the given plant kind built from `cfg`, logging the
         * motor-side encoder (the only thing a real robot exposes).
         *
         * @param kind which plant simulation to identify through
         */
        @JvmStatic
        fun characterize(cfg: ArmPlantConfig, kind: ArmEngine.PlantKind): Result {
            val ticksPerRad = cfg.ticksPerRad()
            val lo = cfg.minAngleRad
            val hi = cfg.maxAngleRad
            val span = hi - lo
            val bottom = lo + 0.05 * span
            val top = hi - 0.05 * span
            val mid = lo + 0.55 * span
            // Flex-aware fit windows: through the flex plant the structural period is known from
            // the
            // config, so run intervals span whole periods and the ring cancels out of the kA fit.
            val fit = org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams()
            if (kind == ArmEngine.PlantKind.FLEX && cfg.flexHz > 0) {
                fit.flexPeriodSec = 1.0 / cfg.flexHz
            }
            // Gravity peaks at horizontals (cos θ = ±1). On an over-the-top workspace those are 0
            // and
            // +π, not the span midpoint (often upright). Seed extra runs from every horizontal that
            // sits clear of the hard stops so kS/kCos stay identifiable.
            val margin = 0.08 * span
            val gravityStarts = gravityRichStarts(lo, hi, margin, mid)

            // Stage 2 data: constant-power runs (rich angle sweep at near-terminal velocity) in
            // both
            // directions, plus hard steps from rest whose onset makes Δw large (exciting kA).
            val movingRows = ArrayList<DoubleArray>()
            val movingRhs = ArrayList<Double>()
            for (p in doubleArrayOf(0.45, 0.60, 0.80, 0.95)) {
                runRun(cfg, ticksPerRad, bottom, p, kind, fit, movingRows, movingRhs)
            }
            for (p in doubleArrayOf(-0.45, -0.60, -0.80, -0.95)) {
                runRun(cfg, ticksPerRad, top, p, kind, fit, movingRows, movingRhs)
            }
            for (start in gravityStarts) {
                runRun(cfg, ticksPerRad, start, 0.70, kind, fit, movingRows, movingRhs)
                runRun(cfg, ticksPerRad, start, -0.70, kind, fit, movingRows, movingRhs)
            }

            // Stage 1 data: constant-velocity holds at several speeds, both directions.
            val holdRows = ArrayList<DoubleArray>()
            val holdRhs = ArrayList<Double>()
            for (speed in SWEEP_SPEEDS) {
                velocityHold(cfg, ticksPerRad, kind, +speed, fit, holdRows, holdRhs)
                velocityHold(cfg, ticksPerRad, kind, -speed, fit, holdRows, holdRhs)
            }

            return Result(
                org.marsroboticsassociation.controllib.mechanism.ArmSysId.solveTwoStage(
                    fit,
                    holdRows,
                    holdRhs,
                    movingRows,
                    movingRhs,
                )
            )
        }

        /** One constant-power run: log position, then stack integrated-dynamics equations. */
        private fun runRun(
            cfg: ArmPlantConfig,
            ticksPerRad: Double,
            startRad: Double,
            power: Double,
            kind: ArmEngine.PlantKind,
            fit: org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val n = RUN_STEPS
            val theta = DoubleArray(n)
            // Clean encoders throughout: isolate the plant (lash/flex) effect from read-timing
            // jitter.
            val sim = newSim(cfg, ticksPerRad, kind, startRad)
            for (i in 0 until n) {
                sim.step(DT, power)
                theta[i] = sim.posRad()
            }
            val v = power * HUB
            org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateRun(
                theta,
                v,
                DT,
                cfg.minAngleRad,
                cfg.maxAngleRad,
                fit,
                rows,
                rhs,
            )
        }

        /**
         * One constant-velocity hold: a PI loop on velocity error sweeps the arm across the travel
         * at a held `targetVel`, logging motor-encoder angle and the applied voltage that sustains
         * it. The log is handed to
         * [org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateHold], which keeps
         * only the steady (α ≈ 0) samples. Mirrors what the on-robot OpMode does with a real arm:
         * same angle + voltage log, same accumulate call.
         */
        private fun velocityHold(
            cfg: ArmPlantConfig,
            ticksPerRad: Double,
            kind: ArmEngine.PlantKind,
            targetVel: Double,
            fit: org.marsroboticsassociation.controllib.mechanism.ArmSysId.FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val start =
                if (targetVel > 0) cfg.minAngleRad + fit.stopMarginRad
                else cfg.maxAngleRad - fit.stopMarginRad
            val sim = newSim(cfg, ticksPerRad, kind, start)
            val theta = ArrayList<Double>()
            val volts = ArrayList<Double>()
            var integral = 0.0
            for (i in 0 until SWEEP_MAX_STEPS) {
                val err = targetVel - sim.velRad()
                integral += err * DT
                val power = max(-1.0, min(1.0, (SWEEP_KP * err + SWEEP_KI * integral) / HUB))
                sim.step(DT, power)
                val pos = sim.posRad()
                // Stop once the sweep reaches the far hard-stop margin (accumulateHold discards
                // those
                // samples anyway); until then log every step.
                if (
                    if (targetVel > 0) pos > cfg.maxAngleRad - fit.stopMarginRad
                    else pos < cfg.minAngleRad + fit.stopMarginRad
                ) {
                    if (theta.isNotEmpty()) break
                }
                theta.add(pos)
                volts.add(power * HUB)
            }
            val times = DoubleArray(theta.size) { it * DT } // sim runs at a fixed step
            org.marsroboticsassociation.controllib.mechanism.ArmSysId.accumulateHold(
                toArray(theta),
                toArray(volts),
                times,
                cfg.minAngleRad,
                cfg.maxAngleRad,
                fit,
                rows,
                rhs,
            )
        }

        private fun toArray(list: List<Double>): DoubleArray {
            val a = DoubleArray(list.size)
            for (i in list.indices) a[i] = list[i]
            return a
        }

        /** Build a [SimHandle] for the plant kind, seeded at `startRad` with a clean encoder. */
        private fun newSim(
            cfg: ArmPlantConfig,
            ticksPerRad: Double,
            kind: ArmEngine.PlantKind,
            startRad: Double,
        ): SimHandle {
            return when (kind) {
                ArmEngine.PlantKind.FLEX -> {
                    val sim =
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
                            startRad,
                            cfg.backlashRad,
                            cfg.flexHz,
                            cfg.flexZeta,
                        )
                    sim.setEncoder(EncoderSim())
                    handle(
                        sim::step,
                        sim::getPositionTicks,
                        sim::getVelocityTps,
                        ticksPerRad,
                        cfg.encoderZeroOffsetRad,
                    )
                }
                ArmEngine.PlantKind.BACKLASH -> {
                    val sim =
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
                            startRad,
                            cfg.backlashRad,
                        )
                    sim.setEncoder(EncoderSim())
                    handle(
                        sim::step,
                        sim::getPositionTicks,
                        sim::getVelocityTps,
                        ticksPerRad,
                        cfg.encoderZeroOffsetRad,
                    )
                }
                ArmEngine.PlantKind.RIGID -> {
                    val sim =
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
                            startRad,
                        )
                    sim.setEncoder(EncoderSim())
                    handle(
                        sim::step,
                        sim::getPositionTicks,
                        sim::getVelocityTps,
                        ticksPerRad,
                        cfg.encoderZeroOffsetRad,
                    )
                }
            }
        }

        /** Adapt a sim's step/read methods to a [SimHandle] in radians. */
        private fun handle(
            stepper: Stepper,
            ticks: IntSupplier,
            velTps: DoubleSupplier,
            ticksPerRad: Double,
            offsetRad: Double,
        ): SimHandle =
            object : SimHandle {
                override fun step(dt: Double, power: Double) {
                    stepper.step(dt, power, HUB)
                }

                override fun posRad(): Double = ticks.asInt / ticksPerRad + offsetRad

                override fun velRad(): Double = velTps.asDouble / ticksPerRad
            }

        /**
         * Start angles that excite gravity for OLS: horizontals in range (cos θ = ±1), plus the
         * span midpoint as a fallback when neither horizontal is available.
         */
        private fun gravityRichStarts(
            lo: Double,
            hi: Double,
            margin: Double,
            mid: Double,
        ): DoubleArray {
            val starts = ArrayList<Double>()
            for (h in doubleArrayOf(0.0, PI, -PI)) {
                if (h > lo + margin && h < hi - margin) {
                    starts.add(h)
                }
            }
            if (starts.isEmpty()) {
                starts.add(mid)
            }
            val out = DoubleArray(starts.size)
            for (i in starts.indices) out[i] = starts[i]
            return out
        }
    }
}
