package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Arm system identification via the integrated equation of motion.
 *
 * <p>Recovers the five-parameter feedforward model
 * <pre>
 *   V = kS·sign(ω) + kV·ω + kA·α + kCos·cos(θ) + kSin·sin(θ)
 * </pre>
 *
 * by regressing on integrated intervals rather than differentiating noisy encoder velocity:
 * <pre>
 *   ∫V dt = kS·∫sign(ω) dt + kV·Δθ + kA·Δω + kCos·∫cosθ dt + kSin·∫sinθ dt
 * </pre>
 *
 * Every term is exact or a smoothing integral: `Δθ` comes from encoder position; gravity terms are
 * trapezoidal integrals of measured angle; the only boundary term is `Δω`, obtained from a local
 * quadratic fit of position (noise-tolerant, low-lag). Hard steps / mid-run acceleration excite
 * `kA`; angle sweeps excite gravity.
 *
 * <p>Intervals are kept only where velocity holds a single sign above a minimum speed and stays
 * clear of the hard stops, so `sign(ω)` is constant and contact forces do not pollute the samples.
 *
 * <p>Typical workflow: call [accumulateRun] once per constant-power run (robot or sim), then
 * [solve]. ControlLab's simulated battery and the on-robot `ArmSysIdTuning` OpMode both use this
 * path.
 *
 * <p><b>Through a lashy/flexy drivetrain</b>, prefer the two-stage path: friction, viscous drag,
 * gravity, and the backlash half-angle from constant-velocity holds ([accumulateHold]) plus inertia
 * from the moving runs, combined by [solveTwoStage]. A hard-accelerating run through the lash and
 * flex inflates `kS` — the direction-flipping flex/lash deflection is collinear with the `sign(ω)`
 * regressor — and that over-estimate becomes an anti-braking feedforward term that reintroduces
 * arrival overshoot. Holding a steady speed keeps `α ≈ 0`, starving that bias; direction-split
 * gravity columns absorb the ±half-lash encoder offset the holds still see. Same logging (angle +
 * voltage) as [accumulateRun]; the caller just drives a held speed instead of a constant voltage.
 * When the structural flex period is known (config, or [estimateFlexPeriod] on a ring-down log),
 * set [FitParams.flexPeriodSec] so the moving-run intervals span whole flex periods and the ring
 * cancels out of the `kA` fit.
 *
 * @see ArmModel
 */
class ArmSysId private constructor() {

    /** Identified feedforward coefficients plus a fit-quality score. */
    class Result {
        @JvmField val kS: Double
        @JvmField val kV: Double
        @JvmField val kA: Double
        @JvmField val kCos: Double
        @JvmField val kSin: Double
        @JvmField val rSquared: Double
        @JvmField val samples: Int
        /**
         * Hold-stage `kV` (quasi-static, immune to flex/lash transients). This is the shipped [kV]
         * when the holds cover enough of a speed range to pin it; `NaN` when they do not.
         */
        @JvmField val kVHold: Double
        /**
         * Run-stage `kV` from the two-parameter `(Δθ, Δω)` fit over the moving runs. Kept as a
         * cross-check even when the hold-stage value ships; `NaN` on the no-holds fallback path
         * only when the moving fit itself failed.
         */
        @JvmField val kVRun: Double
        /**
         * Backlash half-angle (rad) implied by the split gravity regressors of the hold fit:
         * gravity acts at the arm angle, offset from the motor encoder by ±half-lash on whichever
         * tooth face carries the load, and that offset shows up as an `offsetSign·sinθ` voltage
         * term of size `kG·halfLash`. The estimate sums the direction-selected and gravity-selected
         * contributions, so it tracks the mechanical lash regardless of which regime dominates the
         * sweep. `NaN` when neither family was identifiable (holds all one direction and never
         * crossing a horizontal).
         */
        @JvmField val halfLashRad: Double

        constructor(
            kS: Double,
            kV: Double,
            kA: Double,
            kCos: Double,
            kSin: Double,
            rSquared: Double,
            samples: Int,
        ) : this(kS, kV, kA, kCos, kSin, rSquared, samples, Double.NaN, kV, Double.NaN)

        constructor(
            kS: Double,
            kV: Double,
            kA: Double,
            kCos: Double,
            kSin: Double,
            rSquared: Double,
            samples: Int,
            kVHold: Double,
            kVRun: Double,
            halfLashRad: Double,
        ) {
            this.kS = kS
            this.kV = kV
            this.kA = kA
            this.kCos = kCos
            this.kSin = kSin
            this.rSquared = rSquared
            this.samples = samples
            this.kVHold = kVHold
            this.kVRun = kVRun
            this.halfLashRad = halfLashRad
        }

        /**
         * Relative disagreement `|kVHold − kVRun| / kV` between the quasi-static and moving
         * estimates of `kV`. The two agree on a rigid drivetrain; a gap beyond ~10% flags flex/lash
         * contamination of the moving runs (the hold-side value is the trustworthy one). `NaN` when
         * either estimate is unavailable.
         */
        fun kVDisagreement(): Double {
            return abs(kVHold - kVRun) / max(abs(kV), 1e-9)
        }

        /**
         * Gravity magnitude `√(kCos² + kSin²)`, the `kG` that a pure-cosine `ArmFeedforward` would
         * use when the encoder zero is corrected by [phiRad].
         */
        fun kG(): Double {
            return hypot(kCos, kSin)
        }

        /**
         * Encoder-zero offset from horizontal implied by the gravity regressors: `V_grav = kG·cos(θ
         * − φ)` with `φ = atan2(kSin, kCos)`. Zero when gravity is a pure cosine of the measured
         * angle.
         */
        fun phiRad(): Double {
            return atan2(kSin, kCos)
        }
    }

    /**
     * Fit hyper-parameters. Defaults match ControlLab's simulated sysid battery.
     *
     * <p>All angles in radians, times in seconds, speeds in rad/s.
     */
    class FitParams {
        /** Half-window (samples) for local quadratic velocity at interval endpoints. */
        @JvmField var velHalf: Int = 3
        /** |ω| must stay above this for an interval to be usable (sign well-defined). */
        @JvmField var minSpeedRad: Double = 0.2
        /** Discard samples within this margin of either hard stop. */
        @JvmField var stopMarginRad: Double = 4.0 * PI / 180.0
        /** Interval lengths in samples (multi-scale windows). */
        @JvmField var intervalLens: IntArray = intArrayOf(8, 16, 30)
        /** Stride between interval starts (samples). */
        @JvmField var stride: Int = 4
        /**
         * A constant-velocity hold sample counts as steady (quasi-static) only when `|α|` is below
         * this, so the flex sits at its quasi-static deflection and there is no `kA·α` term to bias
         * `kS`. See [accumulateHold].
         */
        @JvmField var holdAccelGate: Double = 1.0
        /**
         * Structural-flex fundamental period (seconds); 0 disables flex-aware interval selection.
         * When set, [accumulateRun] spans each integration interval over a whole number of flex
         * periods (1–3), so the near-undamped ring contributes equal phase at both interval
         * boundaries and its `Δθ`/`Δω` contamination largely cancels. Known from the mechanism
         * (ControlLab's plant config) or measured with [estimateFlexPeriod].
         */
        @JvmField var flexPeriodSec: Double = 0.0
        /**
         * With [flexPeriodSec] set, additionally skip samples within this many seconds of a run's
         * start — the voltage step that rings the flex hardest. 0 keeps every sample; runs are
         * short, so large values starve the `kA` fit of its acceleration onset.
         */
        @JvmField var flexSettleSec: Double = 0.0
        /**
         * Minimum spread of hold speeds (`max|ω| − min|ω|`, rad/s) for the hold-stage `kV` to be
         * trusted as the shipped value. Below this the `sign(ω)` and `ω` regressors are nearly
         * collinear (e.g. every hold at one speed), so `kV` falls back to the moving-run estimate.
         */
        @JvmField var minHoldKvSpeedSpread: Double = 0.5
    }

    companion object {
        @JvmField val DEFAULT_PARAMS: FitParams = FitParams()

        /**
         * Stack integrated-dynamics equations from one constant-voltage run into the OLS system.
         *
         * @param thetaRad equally-spaced arm angles (rad from horizontal, or encoder angle with
         *   offset)
         * @param voltage constant applied voltage for the whole run (volts)
         * @param dt sample period (seconds)
         * @param minAngleRad hard-stop lower bound (same units as `thetaRad`)
         * @param maxAngleRad hard-stop upper bound
         * @param params fit hyper-parameters
         * @param rows feature rows `[sign·Δt, Δθ, Δω, ∫cos, ∫sin]` (appended)
         * @param rhs right-hand side `V·Δt` (appended)
         */
        @JvmStatic
        fun accumulateRun(
            thetaRad: DoubleArray,
            voltage: Double,
            dt: Double,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val n = thetaRad.size
            val volts = DoubleArray(n) { voltage }
            accumulateRun(thetaRad, volts, dt, minAngleRad, maxAngleRad, params, rows, rhs)
        }

        /**
         * Like the constant-voltage [accumulateRun] but with a per-sample applied voltage (handles
         * battery sag). Interval RHS uses the mean voltage over the interval times `Δt`.
         */
        @JvmStatic
        fun accumulateRun(
            thetaRad: DoubleArray,
            voltage: DoubleArray,
            dt: Double,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val n = thetaRad.size
            if (n < 2 || voltage.size != n) {
                return
            }
            val timeSec = uniformTimes(n, dt)
            accumulateRun(thetaRad, voltage, timeSec, minAngleRad, maxAngleRad, params, rows, rhs)
        }

        /**
         * Variable-dt form for robot loops: sample every iteration with wall-clock timestamps.
         * Integrals use the actual per-step `dt`; no assumed fixed sample period.
         *
         * @param thetaRad arm angles (rad)
         * @param voltage applied voltage at each sample (volts)
         * @param timeSec monotonically increasing sample times (seconds); any zero origin is fine
         */
        @JvmStatic
        fun accumulateRun(
            thetaRad: DoubleArray,
            voltage: DoubleArray,
            timeSec: DoubleArray,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val n = thetaRad.size
            if (n < 2 * params.velHalf + 2 || voltage.size != n || timeSec.size != n) {
                return
            }
            val w = DoubleArray(n)
            for (i in 0 until n) {
                w[i] = localVelocity(thetaRad, timeSec, i, n, params.velHalf)
            }

            var firstValid = params.velHalf
            val lastValid = n - 1 - params.velHalf
            if (params.flexPeriodSec > 0) {
                // Flex-aware intervals: span whole flex periods so the near-undamped structural
                // ring
                // contributes equal phase at both boundaries and cancels out of Δθ/Δω, and
                // optionally
                // skip the hard-ringing samples right after the voltage step.
                while (
                    firstValid <= lastValid &&
                        timeSec[firstValid] - timeSec[0] < params.flexSettleSec
                ) {
                    firstValid++
                }
                for (periods in 1..3) {
                    val span = periods * params.flexPeriodSec
                    var j = firstValid
                    var s = firstValid
                    while (s <= lastValid) {
                        if (j < s) {
                            j = s
                        }
                        while (j < lastValid && timeSec[j] - timeSec[s] < span) {
                            j++
                        }
                        // Nearest sample to a whole-period span (j is the first at/after it).
                        if (
                            j > s + 1 &&
                                (timeSec[j] - timeSec[s] - span) >
                                    (span - (timeSec[j - 1] - timeSec[s]))
                        ) {
                            j--
                        }
                        if (timeSec[j] - timeSec[s] < 0.5 * span) {
                            break // the log ends before this period count fits
                        }
                        emitRunInterval(
                            thetaRad,
                            voltage,
                            timeSec,
                            w,
                            s,
                            j,
                            minAngleRad,
                            maxAngleRad,
                            params,
                            rows,
                            rhs,
                        )
                        s += params.stride
                    }
                }
                return
            }
            for (len in params.intervalLens) {
                var s = firstValid
                while (s + len <= lastValid) {
                    emitRunInterval(
                        thetaRad,
                        voltage,
                        timeSec,
                        w,
                        s,
                        s + len,
                        minAngleRad,
                        maxAngleRad,
                        params,
                        rows,
                        rhs,
                    )
                    s += params.stride
                }
            }
        }

        /** Append one integrated-dynamics row for the interval [s, j] if it is usable. */
        private fun emitRunInterval(
            thetaRad: DoubleArray,
            voltage: DoubleArray,
            timeSec: DoubleArray,
            w: DoubleArray,
            s: Int,
            j: Int,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            if (
                j <= s + 1 || !intervalUsable(thetaRad, w, s, j, minAngleRad, maxAngleRad, params)
            ) {
                return
            }
            val dtInterval = timeSec[j] - timeSec[s]
            if (dtInterval < 1e-6) {
                return
            }
            val signW = sign(w[(s + j) / 2])
            val dTheta = thetaRad[j] - thetaRad[s]
            val dW = w[j] - w[s]
            val iCos = trapz(thetaRad, timeSec, s, j, true)
            val iSin = trapz(thetaRad, timeSec, s, j, false)
            val iV = trapz(voltage, timeSec, s, j)
            rows.add(doubleArrayOf(signW * dtInterval, dTheta, dW, iCos, iSin))
            rhs.add(iV)
        }

        /**
         * Stack quasi-static rows from one <b>constant-velocity hold</b> into the friction/gravity
         * system. Where [accumulateRun] excites `kA` with acceleration, a hold suppresses it: at a
         * steady speed `α ≈ 0`, so each sample obeys `V = kS·sign(ω) + kV·ω + kCos·cosθ +
         * kSin·sinθ` and the flex spring sits at its quasi-static deflection. That is what keeps a
         * lashy/flexy drivetrain from inflating `kS` — the direction-flipping flex/lash bias a
         * hard-accelerating run pumps into the `sign(ω)` regressor is starved here.
         *
         * <p>Velocity and acceleration are recovered from the position log (local quadratic fit
         * against the actual sample times, then a central difference), so loop-rate jitter does not
         * bias the fit and the caller only logs angle, applied voltage, and wall-clock time —
         * exactly like [accumulateRun], just driven at a held speed instead of a constant voltage.
         * Only steady samples (|α| under [FitParams.holdAccelGate], |ω| above
         * [FitParams.minSpeedRad], clear of the stops) are kept; each becomes a row `[sign(ω), ω,
         * cosθ, sinθ, sign(ω)·cosθ, sign(ω)·sinθ, sign(cosθ)·cosθ, sign(cosθ)·sinθ, α]` with the
         * applied voltage as its right-hand side. The two split-column families absorb the backlash
         * geometry a motor-side encoder cannot see: gravity acts at the arm angle, offset by
         * ±half-lash on whichever tooth face carries the load — a face that flips with *travel
         * direction* when drivetrain wind-up (friction/flex) dominates, and with the *sign of the
         * gravity torque* (at the horizontals) when gravity dominates. Either offset linearizes to
         * a `kG·halfLash·offsetSign·sinθ` voltage term that would otherwise be dumped into `kS` or
         * gravity. Feed the accumulated rows to [solveTwoStage], which also turns those columns
         * into a half-lash estimate ([Result.halfLashRad]).
         *
         * @param thetaRad arm angles over the hold (rad)
         * @param voltage per-sample applied voltage sustaining the hold (volts)
         * @param timeSec monotonically increasing sample times (seconds); any zero origin is fine
         * @param rows feature rows `[sign(ω), ω, cosθ, sinθ, sign·cosθ, sign·sinθ, α]` (appended)
         * @param rhs right-hand side (applied voltage) (appended)
         */
        @JvmStatic
        fun accumulateHold(
            thetaRad: DoubleArray,
            voltage: DoubleArray,
            timeSec: DoubleArray,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val n = thetaRad.size
            if (n < 2 * params.velHalf + 3 || voltage.size != n || timeSec.size != n) {
                return
            }
            val w = DoubleArray(n)
            for (i in 0 until n) {
                w[i] = localVelocity(thetaRad, timeSec, i, n, params.velHalf)
            }
            for (i in (params.velHalf + 1) until (n - params.velHalf - 1)) {
                val dtc = timeSec[i + 1] - timeSec[i - 1]
                if (dtc < 1e-9) {
                    continue
                }
                val accel = (w[i + 1] - w[i - 1]) / dtc
                if (
                    abs(w[i]) < params.minSpeedRad ||
                        abs(accel) > params.holdAccelGate ||
                        thetaRad[i] < minAngleRad + params.stopMarginRad ||
                        thetaRad[i] > maxAngleRad - params.stopMarginRad
                ) {
                    continue
                }
                val signW = sign(w[i])
                val cosTh = cos(thetaRad[i])
                val sinTh = sin(thetaRad[i])
                val faceSign = sign(cosTh) // contact face under a gravity-dominated load
                rows.add(
                    doubleArrayOf(
                        signW,
                        w[i],
                        cosTh,
                        sinTh,
                        signW * cosTh,
                        signW * sinTh,
                        faceSign * cosTh,
                        faceSign * sinTh,
                        accel,
                    )
                )
                rhs.add(voltage[i])
            }
        }

        /**
         * Two-stage fit that splits each parameter to the data that identifies it cleanly.
         *
         * <p><b>Stage 1 — `kS`, `kV`, gravity, and half-lash from the holds.</b> An OLS over the
         * [accumulateHold] rows yields `kS, kV, kCos, kSin` plus the split-column gravity offsets.
         * These are the terms a hard-accelerating run through the lash + flex corrupts: the
         * direction-flipping friction/lash deflection is collinear with the `sign(ω)` regressor, so
         * it lands in `kS` and biases gravity. The quasi-static holds (α ≈ 0) starve that bias, and
         * the two split-column families absorb the ±half-lash offset between the motor encoder and
         * the arm — flipping with travel direction (wind-up-dominated contact) or with the gravity
         * torque at the horizontals (gravity-dominated contact) — so it cannot leak into `kS`
         * either; instead it comes back out as a backlash half-angle estimate
         * ([Result.halfLashRad]). The hold-side `kV` is quasi-static and therefore flex-immune; it
         * ships as the model's `kV` whenever the holds span enough speeds
         * ([FitParams.minHoldKvSpeedSpread]). Each family is fit only where it is identifiable: the
         * direction family needs holds in both travel directions, the gravity-face family needs the
         * sweep to cross a horizontal.
         *
         * <p><b>Stage 2 — `kA` from the moving runs.</b> With `kS`, `kV`, gravity, and the lash
         * offset subtracted, each [accumulateRun] row reduces to `kA·Δω` — a single-parameter fit
         * with nothing left to trade error against. (When the holds could not pin `kV`, stage 2
         * falls back to the two-parameter `(Δθ, Δω)` fit.) The two-parameter run-side `kV` is
         * always computed as a cross-check and reported as [Result.kVRun]; a large
         * [Result.kVDisagreement] flags flex/lash contamination of the runs. The reported
         * `rSquared` is over the moving rows.
         *
         * <p>Falls back to a plain [solve] on the moving rows if there are too few hold rows.
         *
         * @param params fit hyper-parameters (thresholds for trusting the hold-side `kV`)
         * @param holdRows rows from [accumulateHold]
         * @param holdRhs their right-hand sides
         * @param movingRows rows from [accumulateRun]
         * @param movingRhs their right-hand sides
         */
        @JvmStatic
        fun solveTwoStage(
            params: FitParams,
            holdRows: List<DoubleArray>,
            holdRhs: List<Double>,
            movingRows: List<DoubleArray>,
            movingRhs: List<Double>,
        ): Result {
            if (holdRows.size < 5 || movingRows.size < 2) {
                return solve(movingRows, movingRhs)
            }

            // Coverage decides which stage-1 offset columns are identifiable: the wind-up family
            // needs
            // both travel directions (sign(ω) constant otherwise), the gravity-face family needs
            // the
            // sweep to cross a horizontal (sign(cosθ) constant otherwise).
            var posRows = 0
            var negRows = 0
            var posCos = 0
            var negCos = 0
            var minSpeed = Double.POSITIVE_INFINITY
            var maxSpeed = 0.0
            for (row in holdRows) {
                if (row[0] > 0) {
                    posRows++
                } else if (row[0] < 0) {
                    negRows++
                }
                if (row[2] > 0.05) {
                    posCos++
                } else if (row[2] < -0.05) {
                    negCos++
                }
                val speed = abs(row[1])
                minSpeed = min(minSpeed, speed)
                maxSpeed = max(maxSpeed, speed)
            }
            val bothDirections = posRows >= 3 && negRows >= 3
            val crossesHorizontal = posCos >= 3 && negCos >= 3
            val speedSpreadOk = maxSpeed - minSpeed >= params.minHoldKvSpeedSpread

            // Stage-1 design matrix: base columns always, each offset family only when
            // identifiable.
            val cols = ArrayList(listOf(0, 1, 2, 3))
            if (bothDirections) {
                cols.add(4)
                cols.add(5)
            }
            if (crossesHorizontal) {
                cols.add(6)
                cols.add(7)
            }
            cols.add(8) // α nuisance column
            val y1 = toArray(holdRhs)
            val x1 = Array(holdRows.size) { DoubleArray(cols.size) }
            for (i in holdRows.indices) {
                val xi = holdRows[i]
                for (c in cols.indices) {
                    x1[i][c] = xi[cols[c]]
                }
            }
            val b = solveOls(x1, y1)
            val kS = b[0]
            val stage1Kv = b[1]
            val kCos = b[2]
            val kSin = b[3]
            // Gravity at an offset arm angle linearizes to offsetSign·h·(kCos·sinθ − kSin·cosθ), so
            // each family's fitted [·cos, ·sin] pair projects back to a half-lash contribution;
            // their
            // sum estimates the mechanical half-lash however the contact face happens to be
            // selected
            // across the sweep.
            var halfLash = Double.NaN
            val kG2 = kCos * kCos + kSin * kSin
            if (kG2 > 1e-6 && cols.size > 5) {
                halfLash = 0.0
                var next = 4
                if (bothDirections) {
                    halfLash += (b[next + 1] * kCos - b[next] * kSin) / kG2
                    next += 2
                }
                if (crossesHorizontal) {
                    halfLash += (b[next + 1] * kCos - b[next] * kSin) / kG2
                }
            }
            val kVHold = if (speedSpreadOk) stage1Kv else Double.NaN

            // Moving-run residuals with the hold-identified terms (and lash offset) removed.
            val x2 = Array(movingRows.size) { DoubleArray(2) } // [Δθ, Δω]
            val y2 = DoubleArray(movingRows.size)
            for (i in movingRows.indices) {
                val xi = movingRows[i] // [sign·Δt, Δθ, Δω, ∫cos, ∫sin]
                x2[i][0] = xi[1]
                x2[i][1] = xi[2]
                var y = movingRhs[i] - kS * xi[0] - kCos * xi[3] - kSin * xi[4]
                if (!halfLash.isNaN()) {
                    // ∫kG·cos(θ − sign·h) ≈ kG·∫cos + sign·h·(kCos·∫sin − kSin·∫cos)
                    y -= sign(xi[0]) * halfLash * (kCos * xi[4] - kSin * xi[3])
                }
                y2[i] = y
            }
            // Run-side two-parameter fit: the shipped path when holds cannot pin kV, and the
            // cross-check (kVRun) that flags flex contamination when they can.
            val s2 = solveOls(x2, y2)
            val kVRun = s2[0]

            val kV: Double
            val kA: Double
            if (!kVHold.isNaN()) {
                kV = kVHold
                var num = 0.0
                var den = 0.0
                for (i in y2.indices) {
                    val residual = y2[i] - kV * x2[i][0]
                    num += x2[i][1] * residual
                    den += x2[i][1] * x2[i][1]
                }
                kA = if (den > 1e-12) num / den else s2[1]
            } else {
                kV = kVRun
                kA = s2[1]
            }

            val r2 =
                rSquared(
                    movingRows.toTypedArray(),
                    toArray(movingRhs),
                    doubleArrayOf(kS, kV, kA, kCos, kSin),
                )
            return Result(
                kS,
                kV,
                kA,
                kCos,
                kSin,
                r2,
                holdRows.size + movingRows.size,
                kVHold,
                kVRun,
                halfLash,
            )
        }

        /** [solveTwoStage] with [DEFAULT_PARAMS] thresholds. */
        @JvmStatic
        fun solveTwoStage(
            holdRows: List<DoubleArray>,
            holdRhs: List<Double>,
            movingRows: List<DoubleArray>,
            movingRhs: List<Double>,
        ): Result {
            return solveTwoStage(DEFAULT_PARAMS, holdRows, holdRhs, movingRows, movingRhs)
        }

        /**
         * Solve the stacked OLS system for `kS, kV, kA, kCos, kSin`.
         *
         * @return fit result, or a zeroed result with `samples = 0` if there are too few rows
         */
        @JvmStatic
        fun solve(rows: List<DoubleArray>, rhs: List<Double>): Result {
            if (rows.size < 5) {
                return Result(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, rows.size)
            }
            val x = rows.toTypedArray()
            val y = toArray(rhs)
            val b = solveOls(x, y)
            val r2 = rSquared(x, y, b)
            return Result(b[0], b[1], b[2], b[3], b[4], r2, y.size)
        }

        /**
         * Convenience: accumulate many constant-voltage runs then solve.
         *
         * @param thetaRuns each run's angle series
         * @param voltages per-run constant voltages
         */
        @JvmStatic
        fun characterize(
            thetaRuns: List<DoubleArray>,
            voltages: List<Double>,
            dt: Double,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
        ): Result {
            val rows = ArrayList<DoubleArray>()
            val rhs = ArrayList<Double>()
            for (i in thetaRuns.indices) {
                accumulateRun(
                    thetaRuns[i],
                    voltages[i],
                    dt,
                    minAngleRad,
                    maxAngleRad,
                    params,
                    rows,
                    rhs,
                )
            }
            return solve(rows, rhs)
        }

        /**
         * Estimate the structural-flex period from a ring-down log: park the arm, flick or
         * step-release it, and log the free oscillation. The angle series is detrended with a
         * moving-average baseline; the period is the mean spacing of same-direction zero crossings
         * of the residual.
         *
         * <p>Feed the result to [FitParams.flexPeriodSec]. Works from either encoder side — on a
         * motor-side encoder the ring couples through the drivetrain attenuated but at the same
         * frequency.
         *
         * @param thetaRad logged angles (rad) covering at least a few oscillations
         * @param timeSec monotonically increasing sample times (seconds)
         * @return the estimated period in seconds, or 0 when no consistent oscillation is present
         *   (too few crossings, sub-noise amplitude, or irregular spacing)
         */
        @JvmStatic
        fun estimateFlexPeriod(thetaRad: DoubleArray, timeSec: DoubleArray): Double {
            val n = thetaRad.size
            if (n < 16 || timeSec.size != n) {
                return 0.0
            }
            // Moving-average baseline: wide enough to pass the ring through into the residual.
            val half = max(2, n / 16)
            val residual = DoubleArray(n)
            var maxAbs = 0.0
            for (i in 0 until n) {
                val lo = max(0, i - half)
                val hi = min(n - 1, i + half)
                var mean = 0.0
                for (k in lo..hi) {
                    mean += thetaRad[k]
                }
                mean /= (hi - lo + 1)
                residual[i] = thetaRad[i] - mean
                maxAbs = max(maxAbs, abs(residual[i]))
            }
            if (maxAbs < 1e-4) {
                return 0.0 // sub-noise: no ring to measure
            }
            // Rising zero crossings of the residual, linearly interpolated in time.
            val crossings = ArrayList<Double>()
            for (i in 1 until n) {
                if (residual[i - 1] < 0 && residual[i] >= 0) {
                    val frac = residual[i - 1] / (residual[i - 1] - residual[i])
                    crossings.add(timeSec[i - 1] + frac * (timeSec[i] - timeSec[i - 1]))
                }
            }
            if (crossings.size < 3) {
                return 0.0
            }
            val meanGap = (crossings[crossings.size - 1] - crossings[0]) / (crossings.size - 1)
            var variance = 0.0
            for (i in 1 until crossings.size) {
                val gap = crossings[i] - crossings[i - 1]
                variance += (gap - meanGap) * (gap - meanGap)
            }
            val sd = sqrt(variance / (crossings.size - 1))
            return if (sd < 0.35 * meanGap) meanGap else 0.0
        }

        // ── internals ─────────────────────────────────────────────────────────────

        /**
         * An interval is usable if velocity keeps one sign (above threshold) and clears hard stops.
         */
        @JvmStatic
        internal fun intervalUsable(
            theta: DoubleArray,
            w: DoubleArray,
            s: Int,
            j: Int,
            minAngleRad: Double,
            maxAngleRad: Double,
            params: FitParams,
        ): Boolean {
            val signMid = sign(w[(s + j) / 2])
            for (k in s..j) {
                if (abs(w[k]) < params.minSpeedRad || sign(w[k]) != signMid) {
                    return false
                }
                if (theta[k] < minAngleRad + params.stopMarginRad) {
                    return false
                }
                if (theta[k] > maxAngleRad - params.stopMarginRad) {
                    return false
                }
            }
            return true
        }

        /** Trapezoidal integral of cos(theta) (or sin) over [s, j] with fixed step `dt`. */
        @JvmStatic
        internal fun trapz(
            theta: DoubleArray,
            s: Int,
            j: Int,
            dt: Double,
            useCos: Boolean,
        ): Double {
            var sum = 0.0
            for (k in s until j) {
                val a = if (useCos) cos(theta[k]) else sin(theta[k])
                val b = if (useCos) cos(theta[k + 1]) else sin(theta[k + 1])
                sum += 0.5 * (a + b) * dt
            }
            return sum
        }

        /** Trapezoidal integral of cos/sin(theta) over [s, j] using wall-clock sample times. */
        @JvmStatic
        internal fun trapz(
            theta: DoubleArray,
            timeSec: DoubleArray,
            s: Int,
            j: Int,
            useCos: Boolean,
        ): Double {
            var sum = 0.0
            for (k in s until j) {
                val dtk = timeSec[k + 1] - timeSec[k]
                if (dtk < 1e-9) {
                    continue
                }
                val a = if (useCos) cos(theta[k]) else sin(theta[k])
                val b = if (useCos) cos(theta[k + 1]) else sin(theta[k + 1])
                sum += 0.5 * (a + b) * dtk
            }
            return sum
        }

        /** Trapezoidal integral of a scalar series over [s, j] using wall-clock sample times. */
        @JvmStatic
        internal fun trapz(values: DoubleArray, timeSec: DoubleArray, s: Int, j: Int): Double {
            var sum = 0.0
            for (k in s until j) {
                val dtk = timeSec[k + 1] - timeSec[k]
                if (dtk < 1e-9) {
                    continue
                }
                sum += 0.5 * (values[k] + values[k + 1]) * dtk
            }
            return sum
        }

        /** Velocity at index c from a local quadratic fit of position (noise-tolerant, low-lag). */
        @JvmStatic
        internal fun localVelocity(
            theta: DoubleArray,
            c: Int,
            n: Int,
            dt: Double,
            velHalf: Int,
        ): Double {
            val half = min(velHalf, min(c, n - 1 - c))
            if (half < 1) {
                return 0.0
            }
            val m = 2 * half + 1
            val x = Array(m) { DoubleArray(3) }
            val y = DoubleArray(m)
            var r = 0
            for (k in -half..half) {
                val t = k * dt
                x[r][0] = 1.0
                x[r][1] = t
                x[r][2] = t * t
                y[r] = theta[c + k]
                r++
            }
            return solveOls(x, y)[1] // coefficient of t is the velocity at the center
        }

        /**
         * Velocity at index c from a local quadratic fit using actual sample times (variable loop
         * dt).
         */
        @JvmStatic
        internal fun localVelocity(
            theta: DoubleArray,
            timeSec: DoubleArray,
            c: Int,
            n: Int,
            velHalf: Int,
        ): Double {
            val half = min(velHalf, min(c, n - 1 - c))
            if (half < 1) {
                return 0.0
            }
            val m = 2 * half + 1
            val t0 = timeSec[c]
            val x = Array(m) { DoubleArray(3) }
            val y = DoubleArray(m)
            var r = 0
            for (k in -half..half) {
                val t = timeSec[c + k] - t0
                x[r][0] = 1.0
                x[r][1] = t
                x[r][2] = t * t
                y[r] = theta[c + k]
                r++
            }
            return solveOls(x, y)[1]
        }

        private fun uniformTimes(n: Int, dt: Double): DoubleArray {
            val t = DoubleArray(n)
            for (i in 0 until n) {
                t[i] = i * dt
            }
            return t
        }

        // ── ordinary least squares via normal equations (X^T X) beta = X^T y ──────

        @JvmStatic
        fun solveOls(x: Array<DoubleArray>, y: DoubleArray): DoubleArray {
            val n = x.size
            val m = x[0].size
            val ata = Array(m) { DoubleArray(m) }
            val atb = DoubleArray(m)
            for (i in 0 until n) {
                val xi = x[i]
                for (r in 0 until m) {
                    atb[r] += xi[r] * y[i]
                    for (c in 0 until m) {
                        ata[r][c] += xi[r] * xi[c]
                    }
                }
            }
            return gaussianSolve(ata, atb)
        }

        /** Solve a small dense linear system by Gaussian elimination with partial pivoting. */
        @JvmStatic
        internal fun gaussianSolve(a: Array<DoubleArray>, b: DoubleArray): DoubleArray {
            val m = b.size
            for (col in 0 until m) {
                var piv = col
                for (r in (col + 1) until m) {
                    if (abs(a[r][col]) > abs(a[piv][col])) {
                        piv = r
                    }
                }
                val tmp = a[col]
                a[col] = a[piv]
                a[piv] = tmp
                val t = b[col]
                b[col] = b[piv]
                b[piv] = t

                val diag = safeDiag(a[col][col])
                for (r in (col + 1) until m) {
                    val f = a[r][col] / diag
                    for (c in col until m) {
                        a[r][c] -= f * a[col][c]
                    }
                    b[r] -= f * b[col]
                }
            }
            val out = DoubleArray(m)
            for (row in (m - 1) downTo 0) {
                var sum = b[row]
                for (c in (row + 1) until m) {
                    sum -= a[row][c] * out[c]
                }
                out[row] = sum / safeDiag(a[row][row])
            }
            return out
        }

        private fun safeDiag(d: Double): Double {
            return if (abs(d) < 1e-12) 1e-12.withSign(if (d == 0.0) 1.0 else d) else d
        }

        private fun toArray(list: List<Double>): DoubleArray {
            val a = DoubleArray(list.size)
            for (i in a.indices) {
                a[i] = list[i]
            }
            return a
        }

        @JvmStatic
        internal fun rSquared(x: Array<DoubleArray>, y: DoubleArray, beta: DoubleArray): Double {
            var mean = 0.0
            for (v in y) {
                mean += v
            }
            mean /= y.size
            var ssRes = 0.0
            var ssTot = 0.0
            for (i in y.indices) {
                var pred = 0.0
                for (j in beta.indices) {
                    pred += x[i][j] * beta[j]
                }
                ssRes += (y[i] - pred) * (y[i] - pred)
                ssTot += (y[i] - mean) * (y[i] - mean)
            }
            return if (ssTot < 1e-12) 0.0 else 1.0 - ssRes / ssTot
        }
    }
}
