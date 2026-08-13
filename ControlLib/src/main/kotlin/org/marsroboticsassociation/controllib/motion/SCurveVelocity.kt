package org.marsroboticsassociation.controllib.motion

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Generates a jerk-limited (S-curve) velocity trajectory between two velocities, with support for
 * asymmetric jerk (different magnitudes when increasing vs decreasing acceleration).
 *
 * <p>
 * Behavior notes:
 * - Backwards-compatible constructor: SCurveVelocity(v0, v1, a0, aMax, jMax) => symmetric jerk
 * - New constructor: SCurveVelocity(v0, v1, a0, aMax, jInc, jDec) => asymmetric jerk <p> Tuning
 *   hint: to mimic a flywheel motor that ramps up quickly but settles slowly, use jInc >> jDec
 *   (large jerk to speed up, small jerk to slow the change at the end).
 */
class SCurveVelocity : VelocityTrajectory {
    @JvmField val v0: Double
    @JvmField val v1: Double
    @JvmField val a0: Double
    @JvmField val aMax: Double
    @JvmField val jInc: Double
    @JvmField val jDec: Double // jInc = jerk when increasing accel, jDec = when decreasing
    @JvmField val t1: Double
    @JvmField val t2: Double
    @JvmField val t3: Double
    @JvmField val tf: Double
    @JvmField val aPeak: Double
    private val dir: Double // +1 if accelerating (v1>v0), -1 if decelerating
    private val trivial: Boolean // true if v0 ≈ v1
    private val singlePhase: Boolean // true if only one jerk phase used

    /**
     * Symmetric-jerk constructor: `jMax` bounds jerk in both the increasing- and
     * decreasing-acceleration phases.
     */
    constructor(
        v0: Double,
        v1: Double,
        a0: Double,
        aMax: Double,
        jMax: Double,
    ) : this(v0, v1, a0, aMax, jMax, jMax)

    /**
     * Asymmetric-jerk constructor.
     *
     * @param v0 Initial velocity
     * @param v1 Target velocity
     * @param a0 Initial acceleration (may be positive or negative)
     * @param aMax Maximum magnitude of acceleration allowed (>= 0)
     * @param jInc Maximum magnitude of jerk when increasing acceleration (>= 0)
     * @param jDec Maximum magnitude of jerk when decreasing acceleration (>= 0)
     */
    constructor(v0: Double, v1: Double, a0: Double, aMax: Double, jInc: Double, jDec: Double) {
        this.v0 = v0
        this.v1 = v1
        this.a0 = a0
        this.aMax = abs(aMax)
        this.jInc = abs(jInc)
        this.jDec = abs(jDec)

        this.dir = sign(v1 - v0)
        val dv = abs(v1 - v0)

        // --- Trivial case: no velocity change ---
        if (dv < 1e-9) {
            trivial = true
            singlePhase = false
            t1 = 0.0
            t2 = 0.0
            t3 = 0.0
            tf = 0.0
            aPeak = 0.0
            return
        }

        trivial = false

        // Work in magnitude (positive-motion) frame for acceleration math
        val a0pos = abs(a0)

        // --- Determine if it's too small to null a0 within jerk limits ---
        // To reduce the initial acceleration to zero we use the *decreasing* jerk magnitude (jDec)
        val dv_to_zero_accel = 0.5 * a0pos * a0pos / max(this.jDec, 1e-12)

        if (dv < dv_to_zero_accel && a0pos > 1e-9) {
            // Single-phase motion: constant jerk to reduce acceleration magnitude toward zero
            singlePhase = true

            // Solve 0.5 * jDec * t^2 - a0pos * t + dv = 0  (note: jDec used because we're reducing
            // accel)
            var disc = a0pos * a0pos - 2.0 * this.jDec * dv
            if (disc < 0) disc = 0.0
            val tActual = (a0pos - sqrt(disc)) / max(this.jDec, 1e-12)

            this.t1 = tActual
            this.t2 = 0.0
            this.t3 = 0.0
            this.tf = tActual
            this.aPeak = max(0.0, a0pos - this.jDec * tActual)
            return
        }

        singlePhase = false

        // --- Decide whether to jerk up (increase accel) or down (decrease accel) first ---
        val jerkUpFirst = (dv > dv_to_zero_accel)

        // If jerkUpFirst: we will *increase* acceleration toward aMax using jInc,
        // then decrease to zero using jDec.
        // If not jerkUpFirst: we will *decrease* acceleration toward zero first using jDec,
        // then possibly increase later (handled by same equations with aTarget = 0).

        // --- Try full aMax motion (trapezoidal) ---
        val aTarget = if (jerkUpFirst) this.aMax else 0.0

        // t1: ramp from a0pos -> aTarget using jInc if increasing, jDec if decreasing
        var t1tmp: Double
        if (aTarget >= a0pos) {
            // increasing accel phase uses jInc
            t1tmp = (aTarget - a0pos) / max(this.jInc, 1e-12)
        } else {
            // decreasing accel phase uses jDec
            t1tmp = (a0pos - aTarget) / max(this.jDec, 1e-12)
        }
        if (t1tmp < 0) t1tmp = 0.0

        // t3: fall from aTarget -> 0 uses jDec (always decreasing to zero at end)
        val t3tmp = aTarget / max(this.jDec, 1e-12)

        // Velocity change from full (trapezoidal) motion:
        // dvFull = area from ramp up + plateau + ramp down (we'll handle plateau separately)
        val dvFull = 0.5 * (a0pos + aTarget) * abs(t1tmp) + 0.5 * aTarget * t3tmp
        val t2tmp = (dv - dvFull) / max(aTarget, 1e-9)

        val aPeakLocal: Double
        val t1Local: Double
        val t2Local: Double
        val t3Local: Double

        if (t2tmp < 0) {
            // --- Triangular profile (never reach full aMax) ---
            t2Local = 0.0

            // For asymmetric jerk, derive aPeak from:
            // dv = (aPeak^2 - a0pos^2) / (2*jInc) + aPeak^2 / (2*jDec)
            // Solve for aPeak^2:
            // aPeak^2 * (1/jInc + 1/jDec) = 2*dv + a0pos^2 / jInc
            // => aPeak^2 = (2*dv + a0pos^2/jInc) / (1/jInc + 1/jDec)
            val denom = (1.0 / max(this.jInc, 1e-12)) + (1.0 / max(this.jDec, 1e-12))
            val numer = 2.0 * dv + (a0pos * a0pos) / max(this.jInc, 1e-12)
            val aPeakSq = max(0.0, numer / denom)
            aPeakLocal = sqrt(aPeakSq)

            // t1: from a0pos -> aPeakLocal. Use jInc when increasing, jDec when decreasing
            t1Local =
                if (aPeakLocal >= a0pos) (aPeakLocal - a0pos) / max(this.jInc, 1e-12)
                else (a0pos - aPeakLocal) / max(this.jDec, 1e-12)

            // t3: ramp down from aPeakLocal -> 0 uses jDec
            t3Local = aPeakLocal / max(this.jDec, 1e-12)
        } else {
            // --- Full trapezoidal profile ---
            aPeakLocal = aTarget

            t1Local =
                if (aTarget >= a0pos) (aTarget - a0pos) / max(this.jInc, 1e-12)
                else (a0pos - aTarget) / max(this.jDec, 1e-12)

            t2Local = t2tmp
            t3Local = t3tmp
        }

        this.aPeak = aPeakLocal
        this.t1 = t1Local
        this.t2 = t2Local
        this.t3 = t3Local
        this.tf = t1 + t2 + t3
    }

    override fun getAcceleration(t: Double): Double {
        if (trivial) return 0.0

        // magnitude (positive-motion frame)
        val aMag: Double
        if (singlePhase) {
            aMag =
                if (t < 0) abs(a0)
                else if (t < tf) abs(a0) - jDec * t // single-phase reduces accel -> use jDec
                else 0.0
            return dir * aMag
        }

        if (t < 0) {
            aMag = abs(a0)
        } else if (t < t1) {
            // ramp from a0 -> aPeak
            aMag =
                if (aPeak >= abs(a0)) {
                    // increasing accel: use jInc
                    abs(a0) + jInc * t
                } else {
                    // decreasing accel: use jDec
                    abs(a0) - jDec * t
                }
        } else if (t < t1 + t2) {
            aMag = aPeak
        } else if (t < tf) {
            val dt3 = t - t1 - t2
            // final ramp down to zero uses jDec
            var a = aPeak - jDec * dt3
            if (a < 0) a = 0.0
            aMag = a
        } else {
            aMag = 0.0
        }

        return dir * aMag
    }

    override fun getVelocity(t: Double): Double {
        if (trivial) return v1
        if (t <= 0) return v0

        // in magnitude frame
        val v: Double
        if (singlePhase) {
            v =
                if (t < tf) v0 + abs(a0) * t - 0.5 * jDec * t * t
                else v0 + abs(a0) * tf - 0.5 * jDec * tf * tf
            return v0 + dir * abs(v - v0)
        }

        // Multi-phase
        if (t < t1) {
            // first ramp
            v =
                if (aPeak >= abs(a0)) {
                    // increasing accel -> +0.5 * jInc * t^2
                    v0 + abs(a0) * t + 0.5 * jInc * t * t
                } else {
                    // decreasing accel -> -0.5 * jDec * t^2
                    v0 + abs(a0) * t - 0.5 * jDec * t * t
                }
        } else if (t < t1 + t2) {
            // plateau region
            v = v0 + (abs(a0) + aPeak) * t1 / 2.0 + aPeak * (t - t1)
        } else if (t < tf) {
            val dt3 = t - t1 - t2
            val v2 = v0 + (abs(a0) + aPeak) * t1 / 2.0 + aPeak * t2
            // final ramp down uses jDec
            v = v2 + aPeak * dt3 - 0.5 * jDec * dt3 * dt3
        } else {
            v = v1
        }

        return v0 + dir * abs(v - v0)
    }

    override val totalTime: Double
        get() = tf

    override fun isZeroJerk(t: Double): Boolean {
        return trivial || singlePhase || t <= 0 || (t >= t1 && t < t1 + t2) || t > tf
    }

    fun velocitySegments(): List<PolynomialCurveSegment> {
        val segments = mutableListOf<PolynomialCurveSegment>()
        if (trivial || tf <= 0) return segments

        val a0Signed = dir * abs(a0)

        if (singlePhase) {
            segments.add(
                PolynomialCurveSegment(
                    0.0,
                    tf,
                    v0,
                    a0Signed,
                    -0.5 * dir * jDec,
                    0.0,
                )
            )
            return segments
        }

        var tStart = 0.0
        var vStart = v0
        var aStart = a0Signed

        if (t1 > 0) {
            val jerk1 = firstPhaseJerkSigned()
            segments.add(
                PolynomialCurveSegment(
                    tStart,
                    tStart + t1,
                    vStart,
                    aStart,
                    0.5 * jerk1,
                    0.0,
                )
            )
            vStart = getVelocity(tStart + t1)
            aStart = getAcceleration(tStart + t1)
            tStart += t1
        }

        if (t2 > 0) {
            segments.add(
                PolynomialCurveSegment(
                    tStart,
                    tStart + t2,
                    vStart,
                    aStart,
                    0.0,
                    0.0,
                )
            )
            vStart = getVelocity(tStart + t2)
            aStart = getAcceleration(tStart + t2)
            tStart += t2
        }

        if (t3 > 0) {
            val jerk3 = -dir * jDec
            segments.add(
                PolynomialCurveSegment(
                    tStart,
                    tStart + t3,
                    vStart,
                    aStart,
                    0.5 * jerk3,
                    0.0,
                )
            )
        }
        return segments
    }

    fun accelerationSegments(): List<PolynomialCurveSegment> {
        val segments = mutableListOf<PolynomialCurveSegment>()
        if (trivial || tf <= 0) return segments

        val a0Signed = dir * abs(a0)

        if (singlePhase) {
            segments.add(
                PolynomialCurveSegment(
                    0.0,
                    tf,
                    a0Signed,
                    -dir * jDec,
                    0.0,
                    0.0,
                )
            )
            return segments
        }

        var tStart = 0.0
        var aStart = a0Signed

        if (t1 > 0) {
            val jerk1 = firstPhaseJerkSigned()
            segments.add(PolynomialCurveSegment(tStart, tStart + t1, aStart, jerk1, 0.0, 0.0))
            aStart = getAcceleration(tStart + t1)
            tStart += t1
        }

        if (t2 > 0) {
            segments.add(PolynomialCurveSegment(tStart, tStart + t2, aStart, 0.0, 0.0, 0.0))
            tStart += t2
        }

        if (t3 > 0) {
            val jerk3 = -dir * jDec
            segments.add(PolynomialCurveSegment(tStart, tStart + t3, aStart, jerk3, 0.0, 0.0))
        }

        return segments
    }

    private fun firstPhaseJerkSigned(): Double {
        return if (aPeak >= abs(a0)) dir * jInc else -dir * jDec
    }

    companion object {
        /**
         * Find the largest jDec (jerk when decreasing acceleration) that keeps a trajectory within
         * the motor's back-EMF voltage budget.
         *
         * <p>The voltage demand along the trajectory is `P(t) = kS + kV·|v(t)| + kA·a(t)`. Three
         * regimes determine which constraint binds:
         * <ol>
         * <li><b>Interior peak.</b> When the maximum of P during phase 3 is interior (`kV·aPeak >
         *   kA·jDec`), the constraint collapses to `jDec = 2·kV·(voltage − kS − kV·|v1|) /
         *   kA²`.</li>
         * <li><b>Boundary peak with `aMax` cap.</b> The voltage maximum sits at the start of phase
         *   3 and `aPeak = aMax`, giving `jDec = kV·aMax² / (2·(kA·aMax − vHead))`.</li>
         * <li><b>Boundary peak with kinematic cap.</b> `aPeak < aMax` from short Δv. Solve
         *   `kV·aPeak² + 2·kA·jInc·aPeak − 2·jInc·(vHead + kV·Δv) = 0` for aPeak, then `jDec =
         *   aPeak²·jInc / (2·Δv·jInc − aPeak²)`.</li>
         * </ol>
         *
         * <p>Sentinel return values:
         * <ul>
         * <li>[Double.POSITIVE_INFINITY] — voltage budget imposes no constraint (`kA <= 0`, or `v0
         *   ≈ v1` within 1e-9 so jDec has no effect).</li>
         * <li>[Double.NaN] — no jDec satisfies the constraints. Causes are infeasible inputs
         *   (`voltage <= kS`, non-positive or non-finite `aMax`, `jInc <= 0`) or a target velocity
         *   so high that even the kinematic peak cannot fit inside the voltage budget. Callers must
         *   check before use.</li>
         * </ul>
         *
         * @param v0 Initial velocity
         * @param v1 Target velocity
         * @param a0 Initial acceleration (unused — closed form is independent of a0)
         * @param aMax Maximum acceleration magnitude; use result of [findMaxAMax] if available
         * @param jInc Jerk magnitude when increasing acceleration (must be &gt; 0)
         * @param voltage Motor supply voltage (e.g. 12.0 V)
         * @param kS Motor static feedforward (V); voltage needed to overcome friction
         * @param kV Motor velocity feedforward (V per velocity unit)
         * @param kA Motor acceleration feedforward (V per acceleration unit); must be &gt; 0
         * @return Largest jDec in [1, 5000] that doesn't violate back-EMF limits, or a sentinel as
         *   described above
         */
        @JvmStatic
        fun findMaxJDec(
            v0: Double,
            v1: Double,
            a0: Double,
            aMax: Double,
            jInc: Double,
            voltage: Double,
            kS: Double,
            kV: Double,
            kA: Double,
        ): Double {
            if (kA <= 0) return Double.POSITIVE_INFINITY
            if (voltage <= kS) return Double.NaN
            if (aMax <= 0 || !aMax.isFinite()) return Double.NaN
            if (jInc <= 0) return Double.NaN
            if (abs(v1 - v0) < 1e-9) return Double.POSITIVE_INFINITY

            val JDEC_FLOOR = 1.0
            val JDEC_CAP = 5000.0

            val dv = abs(v1 - v0)
            val vHead = voltage - kS - kV * abs(v1)

            // Path A — interior peak of P(τ) during phase 3 (τ* > 0, equivalent to kV·aPeak >
            // kA·jDec).
            // At the peak, a(τ*) = kA·jDec/kV and v(τ*) = |v1| − kA²·jDec/(2·kV²); substituting
            // into
            // the voltage budget cancels all aPeak terms and leaves jDec linear in vHead.
            if (vHead > 0) {
                val jDecA = 2.0 * kV * vHead / (kA * kA)
                val aPeakSqA = 2.0 * dv * jInc * jDecA / (jInc + jDecA)
                val aPeakA = min(sqrt(aPeakSqA), aMax)
                if (kV * aPeakA > kA * jDecA) {
                    return clamp(jDecA, JDEC_FLOOR, JDEC_CAP)
                }
            }

            // Boundary regime: peak of P sits at τ=0 (start of phase 3, end of phase 2).
            // Constraint:
            //   kA·aPeak − kV·aPeak²/(2·jDec) ≤ vHead
            // where aPeak = min(√(2·Δv·jInc·jDec/(jInc+jDec)), aMax).

            // Path B — aMax binds (kinematic peak ≥ aMax).
            val denomB = kA * aMax - vHead
            if (denomB <= 0) {
                // kA·aMax ≤ vHead: boundary constraint slack at any jDec. Return cap.
                return JDEC_CAP
            }
            val jDecB = kV * aMax * aMax / (2.0 * denomB)
            val aKinSqB = 2.0 * dv * jInc * jDecB / (jInc + jDecB)
            if (aKinSqB >= aMax * aMax) {
                return clamp(jDecB, JDEC_FLOOR, JDEC_CAP)
            }

            // Path C — kinematic peak binds (aPeak < aMax). Quadratic in aPeak:
            //   kV·aPeak² + 2·kA·jInc·aPeak − 2·jInc·(vHead + kV·Δv) = 0
            val disc = kA * kA * jInc * jInc + 2.0 * kV * jInc * (vHead + kV * dv)
            if (disc < 0) return Double.NaN // infeasible: no jDec satisfies the constraint
            val aPeakC = (-kA * jInc + sqrt(disc)) / kV
            if (aPeakC <= 0) return Double.NaN
            val denomC = 2.0 * dv * jInc - aPeakC * aPeakC
            if (denomC <= 0) return JDEC_CAP // jDec → ∞
            val jDecC = aPeakC * aPeakC * jInc / denomC
            return clamp(jDecC, JDEC_FLOOR, JDEC_CAP)
        }

        private fun clamp(x: Double, lo: Double, hi: Double): Double {
            return max(lo, min(x, hi))
        }

        /**
         * Find the largest aMax (peak acceleration) that keeps a trajectory within the motor's
         * back-EMF voltage budget. The search starts from the theoretical motor maximum at v=0
         * (`(voltage - kS) / kA`) and binary-searches upward to 5000 using 30 iterations and 1000
         * samples per candidate. The search uses symmetric jerk (`jInc = jDec`) and zero initial
         * acceleration, which is appropriate for finding a conservative peak acceleration limit
         * independent of jerk shape.
         *
         * <p>Degenerate inputs are handled as follows rather than producing NaN or infinite values:
         * <ul>
         * <li>`kA <= 0` — no acceleration constant, voltage limit is irrelevant → returns
         *   [Double.POSITIVE_INFINITY]</li>
         * <li>`voltage <= kS` — motor cannot overcome static friction, so no acceleration is safe →
         *   returns `0.0`</li>
         * <li>`jInc <= 0` — degenerate: the search would construct trajectories with near-infinite
         *   rise times, causing samples to miss violations → returns `0.0`</li>
         * </ul>
         *
         * <p>Typical usage — call this first, then pass the result to [findMaxJDec]:
         * <pre>
         *   double aMax = SCurveVelocity.findMaxAMax(0, targetV, jInc, voltage, kS, kV, kA);
         *   double jDec = SCurveVelocity.findMaxJDec(0, targetV, 0, aMax, jInc, voltage, kS, kV, kA);
         * </pre>
         *
         * @param v0 Initial velocity
         * @param v1 Target velocity
         * @param jInc Jerk magnitude when increasing acceleration (must be &gt; 0)
         * @param voltage Motor supply voltage (e.g. 12.0 V)
         * @param kS Motor static feedforward (V); voltage needed to overcome friction
         * @param kV Motor velocity feedforward (V per velocity unit)
         * @param kA Motor acceleration feedforward (V per acceleration unit); must be &gt; 0
         * @return Largest aMax in [`(voltage-kS)/kA`, 5000] that doesn't violate back-EMF limits,
         *   or a sentinel as described above for degenerate inputs
         */
        @JvmStatic
        fun findMaxAMax(
            v0: Double,
            v1: Double,
            jInc: Double,
            voltage: Double,
            kS: Double,
            kV: Double,
            kA: Double,
        ): Double {
            if (kA <= 0) return Double.POSITIVE_INFINITY
            if (voltage <= kS) return 0.0 // motor can't overcome static friction
            if (jInc <= 0) return 0.0 // degenerate jInc produces near-infinite rise times

            val motorAMaxAtZero = (voltage - kS) / kA
            var low = motorAMaxAtZero
            var high = 5000.0
            var best = low

            for (iter in 0 until 30) {
                val mid = (low + high) / 2.0

                val traj = SCurveVelocity(v0, v1, 0.0, mid, jInc, jInc)
                val totalTime = traj.totalTime

                var violates = false
                val samples = 1000
                for (i in 0..samples) {
                    val t = totalTime * i / samples
                    val v = traj.getVelocity(t)
                    val a = traj.getAcceleration(t)

                    val availableVoltage = voltage - kS - kV * abs(v)
                    if (availableVoltage <= 0) {
                        violates = true
                        break
                    }
                    val motorAMax = availableVoltage / kA
                    if (a > motorAMax) {
                        violates = true
                        break
                    }
                }

                if (violates) {
                    high = mid
                } else {
                    best = mid
                    low = mid
                }
            }

            return best
        }
    }
}
