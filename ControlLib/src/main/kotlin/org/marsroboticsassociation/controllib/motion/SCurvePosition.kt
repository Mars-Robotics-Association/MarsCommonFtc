package org.marsroboticsassociation.controllib.motion

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sqrt

/**
 * Jerk-limited (S-curve) position trajectory from p0 to pTarget, ending at rest. Supports
 * asymmetric acceleration limits: aMaxAccel for speeding up, aMaxDecel for braking. Symmetric jerk:
 * single jMax value.
 *
 * <p>The trajectory has an optional "prefix" phase that nulls out any non-zero initial acceleration
 * (by applying ±jMax until a=0), followed by up to 7 standard phases:
 * <ol>
 * <li>Jerk +jMax: a ramps from 0 → aMaxAccel (or triangular peak)</li>
 * <li>Zero jerk: constant acceleration aMaxAccel</li>
 * <li>Jerk -jMax: a ramps from aMaxAccel → 0</li>
 * <li>Zero jerk: constant velocity vPeak (cruise)</li>
 * <li>Jerk -jMax: a ramps from 0 → -aMaxDecel</li>
 * <li>Zero jerk: constant deceleration -aMaxDecel</li>
 * <li>Jerk +jMax: a ramps from -aMaxDecel → 0</li>
 * </ol>
 */
class SCurvePosition(
    @JvmField val p0: Double,
    @JvmField val pTarget: Double,
    @JvmField val v0: Double,
    @JvmField val a0: Double,
    vMax: Double,
    aMaxAccel: Double,
    aMaxDecel: Double,
    jMax: Double,
) : PositionTrajectory {

    @JvmField val vMax: Double = abs(vMax)
    @JvmField val aMaxAccel: Double = abs(aMaxAccel)
    @JvmField val aMaxDecel: Double = abs(aMaxDecel)
    @JvmField val jMax: Double = abs(jMax)

    // Phase durations (7-phase section only, after prefix)
    @JvmField val T1: Double
    @JvmField val T2: Double
    @JvmField val T3: Double
    @JvmField val T4: Double
    @JvmField val T5: Double
    @JvmField val T6: Double
    @JvmField val T7: Double

    // Peak velocity reached during cruise
    @JvmField val vPeak: Double

    private val trivial: Boolean
    private val dir: Double // +1 if pTarget >= p0, else -1

    // Prefix phase: nulls out a0 before the 7-phase section
    @JvmField val tPrefix: Double // duration (0 if a0 == 0)
    private val jPrefix: Double // world-frame jerk during prefix

    // State at end of a0 prefix (start of braking prefix)
    private val pAfterA0Prefix: Double
    private val vAfterA0Prefix: Double

    // Braking prefix: jerk-limited 3-sub-phase decel to bring wrong-way velocity to 0
    @JvmField val tBrake: Double // total duration (0 if not needed)
    @JvmField
    val aBrakeHandoff: Double // acceleration magnitude at brake→main handoff (0 if no braking)
    private val pBrake: Double // position at end of braking
    private val aMaxBrake: Double // max(aMaxAccel, aMaxDecel) used during braking

    // Brake sub-phase table (up to 3 sub-phases)
    private val brakeSubT = DoubleArray(4) // absolute start times
    private val brakeSubP = DoubleArray(4)
    private val brakeSubV = DoubleArray(4)
    private val brakeSubA = DoubleArray(4)
    private val brakeSubJ = DoubleArray(3) // world-frame jerk per sub-phase

    // Phase table: 8 breakpoints for the 7-phase section.
    // phaseStartT[0] = tPrefix (7-phase section begins after prefix).
    private val phaseStartT = DoubleArray(8)
    private val phaseStartP = DoubleArray(8)
    private val phaseStartV = DoubleArray(8)
    private val phaseStartA = DoubleArray(8)
    private val phaseJerk = DoubleArray(7) // world-frame jerk for each phase

    init {
        val totalDistAbs = abs(pTarget - p0)
        dir = if (pTarget > p0) 1.0 else if (pTarget < p0) -1.0 else 1.0

        if (totalDistAbs < 1e-9) {
            trivial = true
            T1 = 0.0
            T2 = 0.0
            T3 = 0.0
            T4 = 0.0
            T5 = 0.0
            T6 = 0.0
            T7 = 0.0
            vPeak = 0.0
            tPrefix = 0.0
            jPrefix = 0.0
            pAfterA0Prefix = p0
            vAfterA0Prefix = v0
            tBrake = 0.0
            pBrake = p0
            aMaxBrake = 0.0
            aBrakeHandoff = 0.0
            phaseStartP[0] = p0
            phaseStartV[0] = v0
            phaseStartA[0] = a0
            for (i in 1 until 8) {
                phaseStartT[i] = 0.0
                phaseStartP[i] = p0
                phaseStartV[i] = v0
                phaseStartA[i] = 0.0
            }
        } else {
            trivial = false

            // ---------------------------------------------------------------
            // Prefix: apply ±jMax for |a0| / jMax seconds to bring a → 0.
            // For a0pos > 0 (accelerating toward target): jerk = -jMax in pos frame
            // For a0pos < 0 (decelerating / wrong direction): jerk = +jMax in pos frame
            // ---------------------------------------------------------------
            val a0pos = a0 * dir
            val preserveHelpfulBrakeAccel =
                v0 * dir < -1e-9 && a0pos > 1e-9 && a0pos <= this.aMaxAccel + 1e-9
            val tPre: Double
            val jPre: Double
            val pStart: Double
            val vStart: Double

            if (preserveHelpfulBrakeAccel || abs(a0pos) < 1e-9) {
                tPre = 0.0
                jPre = 0.0
                pStart = p0
                vStart = v0
            } else {
                tPre = abs(a0pos) / this.jMax
                // jerk in world frame that nulls a0:
                jPre = if (a0pos > 0) (-dir * this.jMax) else (dir * this.jMax)
                pStart =
                    p0 +
                        v0 * tPre +
                        0.5 * a0 * tPre * tPre +
                        (1.0 / 6.0) * jPre * tPre * tPre * tPre
                vStart = v0 + a0 * tPre + 0.5 * jPre * tPre * tPre
                // a at end of prefix = 0 by construction
            }
            tPrefix = tPre
            jPrefix = jPre
            pAfterA0Prefix = pStart
            vAfterA0Prefix = vStart

            // Braking prefix: if vStart is in the wrong direction, brake to 0 with jerk-limited
            // profile
            val tBrk: Double
            val pBrk: Double
            val aBrakeVal: Double
            var aHandoffActual = 0.0
            if (vStart * dir < -1e-9) {
                val speed = abs(vStart)
                aBrakeVal = max(this.aMaxAccel, this.aMaxDecel)
                val aHandoff = this.aMaxAccel // asymmetric: handoff at aMaxAccel, not aBrakeVal
                val aBrakeStart = if (preserveHelpfulBrakeAccel) a0pos else 0.0
                val triThresh =
                    max(
                        0.0,
                        (aHandoff * aHandoff - aBrakeStart * aBrakeStart) / (2.0 * this.jMax),
                    )
                val tBrk1: Double
                val tBrk2: Double
                val tBrk3: Double
                if (speed > triThresh) {
                    // Trapezoidal: ramp 0→aHandoff, hold until v=0, no ramp-down
                    tBrk1 = (aHandoff - aBrakeStart) / this.jMax
                    tBrk2 = (speed - triThresh) / aHandoff
                    tBrk3 = 0.0
                    aHandoffActual = aHandoff // = aMaxAccel
                } else {
                    // Triangular: single ramp-up until v=0
                    tBrk1 =
                        (sqrt(aBrakeStart * aBrakeStart + 2.0 * this.jMax * speed) - aBrakeStart) /
                            this.jMax
                    tBrk2 = 0.0
                    tBrk3 = 0.0
                    aHandoffActual = aBrakeStart + this.jMax * tBrk1
                }
                tBrk = tBrk1 + tBrk2 + tBrk3
                // Jerk in world frame: braking opposes vStart direction
                val jBrk = -sign(vStart) * this.jMax
                brakeSubJ[0] = jBrk
                brakeSubJ[1] = 0.0
                brakeSubJ[2] = 0.0 // no ramp-down: tBrk3=0 makes this a no-op
                // Forward-chain brake sub-phases from (pStart, vStart, 0) at absolute time tPrefix
                brakeSubT[0] = tPrefix
                brakeSubP[0] = pStart
                brakeSubV[0] = vStart
                brakeSubA[0] = aBrakeStart * dir
                val brkDurs = doubleArrayOf(tBrk1, tBrk2, tBrk3)
                for (i in 0 until 3) {
                    val dt = brkDurs[i]
                    val j = brakeSubJ[i]
                    brakeSubP[i + 1] =
                        brakeSubP[i] +
                            brakeSubV[i] * dt +
                            0.5 * brakeSubA[i] * dt * dt +
                            (1.0 / 6.0) * j * dt * dt * dt
                    brakeSubV[i + 1] = brakeSubV[i] + brakeSubA[i] * dt + 0.5 * j * dt * dt
                    brakeSubA[i + 1] = brakeSubA[i] + j * dt
                    brakeSubT[i + 1] = brakeSubT[i] + dt
                }
                pBrk = brakeSubP[3]
            } else {
                tBrk = 0.0
                aBrakeVal = 0.0
                pBrk = pStart
            }
            tBrake = tBrk
            aMaxBrake = aBrakeVal
            pBrake = pBrk
            aBrakeHandoff = aHandoffActual

            val distRemaining = max(0.0, dir * (pTarget - pBrk))
            val v0adj = if (tBrk > 0) 0.0 else min(max(vStart * dir, 0.0), this.vMax)
            val vStartWorld = v0adj * dir
            val aStart = if (tBrk > 0) aHandoffActual else 0.0

            // ---------------------------------------------------------------
            // Bisect for vPeak. D(v) is strictly increasing in v for v in [v0adj, vMax].
            // ---------------------------------------------------------------
            val dAtVMax =
                accelHalfDistFrom(v0adj, this.vMax, this.aMaxAccel, this.jMax, aStart) +
                    halfDist(this.vMax, this.aMaxDecel, this.jMax)

            val vPeakSolved: Double
            val T4solved: Double

            if (distRemaining >= dAtVMax) {
                vPeakSolved = this.vMax
                T4solved = if (this.vMax > 1e-9) (distRemaining - dAtVMax) / this.vMax else 0.0
            } else {
                T4solved = 0.0
                val dAtV0adj = halfDist(v0adj, this.aMaxDecel, this.jMax)
                if (distRemaining <= dAtV0adj) {
                    // Not enough room to even accelerate; just decel from v0adj
                    vPeakSolved = v0adj
                } else {
                    var lo = v0adj
                    var hi = this.vMax
                    for (i in 0 until 64) {
                        val mid = (lo + hi) * 0.5
                        val d =
                            accelHalfDistFrom(v0adj, mid, this.aMaxAccel, this.jMax, aStart) +
                                halfDist(mid, this.aMaxDecel, this.jMax)
                        if (d < distRemaining) lo = mid else hi = mid
                    }
                    vPeakSolved = (lo + hi) * 0.5
                }
            }
            vPeak = vPeakSolved
            T4 = max(0.0, T4solved)

            // ---------------------------------------------------------------
            // Phase durations from vPeak
            // ---------------------------------------------------------------
            val vAccelMin = this.aMaxAccel * this.aMaxAccel / this.jMax
            val vDecelMin = this.aMaxDecel * this.aMaxDecel / this.jMax
            val dvAccel = vPeakSolved - v0adj

            val t1: Double
            val t2: Double
            val t3: Double
            val trapThresh =
                (2.0 * this.aMaxAccel * this.aMaxAccel - aStart * aStart) / (2.0 * this.jMax)
            if (dvAccel <= 0) {
                t1 = 0.0
                t2 = 0.0
                t3 = 0.0
            } else if (dvAccel >= trapThresh) {
                // Trapezoidal: ramp aStart→aMaxAccel, hold, ramp aMaxAccel→0
                t1 = (this.aMaxAccel - aStart) / this.jMax
                t2 = (dvAccel - trapThresh) / this.aMaxAccel
                t3 = this.aMaxAccel / this.jMax
            } else {
                // Triangular: ramp aStart→aPk→0
                val aPk = sqrt(dvAccel * this.jMax + aStart * aStart / 2.0)
                t1 = (aPk - aStart) / this.jMax
                t2 = 0.0
                t3 = aPk / this.jMax
            }

            val t5: Double
            val t6: Double
            val t7: Double
            if (vPeakSolved >= vDecelMin) {
                t5 = this.aMaxDecel / this.jMax
                t6 = (vPeakSolved - vDecelMin) / this.aMaxDecel
                t7 = t5
            } else {
                t5 = if (vPeakSolved > 0) sqrt(vPeakSolved / this.jMax) else 0.0
                t6 = 0.0
                t7 = t5
            }

            T1 = max(0.0, t1)
            T2 = max(0.0, t2)
            T3 = max(0.0, t3)
            T5 = max(0.0, t5)
            T6 = max(0.0, t6)
            T7 = max(0.0, t7)

            // ---------------------------------------------------------------
            // Build phase table by forward chaining from (pStart, vStartWorld, 0)
            // ---------------------------------------------------------------
            // Phase jerks in world frame (positive frame is dir):
            // Phase 0 (T1): a 0 → aMaxAccel    => +jMax * dir
            // Phase 1 (T2): constant aMaxAccel  => 0
            // Phase 2 (T3): a aMaxAccel → 0     => -jMax * dir
            // Phase 3 (T4): constant velocity   => 0
            // Phase 4 (T5): a 0 → -aMaxDecel   => -jMax * dir
            // Phase 5 (T6): constant -aMaxDecel => 0
            // Phase 6 (T7): a -aMaxDecel → 0   => +jMax * dir
            phaseJerk[0] = dir * this.jMax
            phaseJerk[1] = 0.0
            phaseJerk[2] = -dir * this.jMax
            phaseJerk[3] = 0.0
            phaseJerk[4] = -dir * this.jMax
            phaseJerk[5] = 0.0
            phaseJerk[6] = dir * this.jMax

            phaseStartT[0] = tPre + tBrk
            phaseStartP[0] = pBrk
            phaseStartV[0] = vStartWorld
            phaseStartA[0] = if (tBrk > 0) aHandoffActual * dir else 0.0

            val durations = doubleArrayOf(T1, T2, T3, T4, T5, T6, T7)
            for (i in 0 until 7) {
                val dt = durations[i]
                val j = phaseJerk[i]
                val ps = phaseStartP[i]
                val vs = phaseStartV[i]
                val as0 = phaseStartA[i]
                phaseStartP[i + 1] =
                    ps + vs * dt + 0.5 * as0 * dt * dt + (1.0 / 6.0) * j * dt * dt * dt
                phaseStartV[i + 1] = vs + as0 * dt + 0.5 * j * dt * dt
                phaseStartA[i + 1] = as0 + j * dt
                phaseStartT[i + 1] = phaseStartT[i] + dt
            }
        }
    }

    // ---------------------------------------------------------------
    // PositionTrajectory interface
    // ---------------------------------------------------------------

    override fun getPosition(t: Double): Double {
        if (trivial) return p0
        if (t <= 0) return p0
        val tf = totalTime
        if (t >= tf) return pTarget

        if (tPrefix > 0 && t < tPrefix) {
            return p0 + v0 * t + 0.5 * a0 * t * t + (1.0 / 6.0) * jPrefix * t * t * t
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            val dt = t - brakeSubT[bph]
            val j = brakeSubJ[bph]
            return brakeSubP[bph] +
                brakeSubV[bph] * dt +
                0.5 * brakeSubA[bph] * dt * dt +
                (1.0 / 6.0) * j * dt * dt * dt
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        val j = phaseJerk[ph]
        return phaseStartP[ph] +
            phaseStartV[ph] * dt +
            0.5 * phaseStartA[ph] * dt * dt +
            (1.0 / 6.0) * j * dt * dt * dt
    }

    override fun getVelocity(t: Double): Double {
        if (trivial) return v0
        if (t <= 0) return v0
        val tf = totalTime
        if (t >= tf) return 0.0

        if (tPrefix > 0 && t < tPrefix) {
            return v0 + a0 * t + 0.5 * jPrefix * t * t
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            val dt = t - brakeSubT[bph]
            val j = brakeSubJ[bph]
            return brakeSubV[bph] + brakeSubA[bph] * dt + 0.5 * j * dt * dt
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        val j = phaseJerk[ph]
        return phaseStartV[ph] + phaseStartA[ph] * dt + 0.5 * j * dt * dt
    }

    override fun getAcceleration(t: Double): Double {
        if (trivial) return a0
        if (t <= 0) return a0
        val tf = totalTime
        if (t >= tf) return 0.0

        if (tPrefix > 0 && t < tPrefix) {
            return a0 + jPrefix * t
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            val dt = t - brakeSubT[bph]
            return brakeSubA[bph] + brakeSubJ[bph] * dt
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        return phaseStartA[ph] + phaseJerk[ph] * dt
    }

    override val totalTime: Double
        get() = if (trivial) 0.0 else phaseStartT[7]

    override fun isZeroJerk(t: Double): Boolean {
        if (trivial) return true
        if (t <= 0 || t >= totalTime) return true
        if (tPrefix > 0 && t < tPrefix) return false // prefix has active jerk
        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) return brakeSubJ[findBrakePhase(t)] == 0.0
        return phaseJerk[findPhase(t)] == 0.0
    }

    /** Returns brake sub-phase index in [0,2] for time t during the braking prefix. */
    private fun findBrakePhase(t: Double): Int {
        if (t >= brakeSubT[2]) return 2
        if (t >= brakeSubT[1]) return 1
        return 0
    }

    /** Returns phase index in [0,6] for time t, assuming t >= phaseStartT[0]. */
    private fun findPhase(t: Double): Int {
        for (i in 6 downTo 1) {
            if (t >= phaseStartT[i]) return i
        }
        return 0
    }

    fun positionSegments(): List<PolynomialCurveSegment> {
        val segments = ArrayList<PolynomialCurveSegment>()
        if (trivial) return segments

        if (tPrefix > 0) {
            segments.add(
                PolynomialCurveSegment(
                    if (tPrefix == 0.0) 0.0 else 0.0,
                    tPrefix,
                    p0,
                    v0,
                    0.5 * a0,
                    jPrefix / 6.0,
                )
            )
        }

        addBrakePositionSegments(segments)

        for (i in 0 until 7) {
            val start = phaseStartT[i]
            val end = phaseStartT[i + 1]
            if (end <= start) continue
            segments.add(
                PolynomialCurveSegment(
                    start,
                    end,
                    phaseStartP[i],
                    phaseStartV[i],
                    0.5 * phaseStartA[i],
                    phaseJerk[i] / 6.0,
                )
            )
        }

        return segments
    }

    fun velocitySegments(): List<PolynomialCurveSegment> {
        val segments = ArrayList<PolynomialCurveSegment>()
        if (trivial) return segments

        if (tPrefix > 0) {
            segments.add(PolynomialCurveSegment(0.0, tPrefix, v0, a0, 0.5 * jPrefix, 0.0))
        }

        addBrakeVelocitySegments(segments)

        for (i in 0 until 7) {
            val start = phaseStartT[i]
            val end = phaseStartT[i + 1]
            if (end <= start) continue
            segments.add(
                PolynomialCurveSegment(
                    start,
                    end,
                    phaseStartV[i],
                    phaseStartA[i],
                    0.5 * phaseJerk[i],
                    0.0,
                )
            )
        }

        return segments
    }

    fun accelerationSegments(): List<PolynomialCurveSegment> {
        val segments = ArrayList<PolynomialCurveSegment>()
        if (trivial) return segments

        if (tPrefix > 0) {
            segments.add(PolynomialCurveSegment(0.0, tPrefix, a0, jPrefix, 0.0, 0.0))
        }

        addBrakeAccelerationSegments(segments)

        for (i in 0 until 7) {
            val start = phaseStartT[i]
            val end = phaseStartT[i + 1]
            if (end <= start) continue
            segments.add(PolynomialCurveSegment(start, end, phaseStartA[i], phaseJerk[i], 0.0, 0.0))
        }

        return segments
    }

    private fun addBrakePositionSegments(segments: MutableList<PolynomialCurveSegment>) {
        if (tBrake <= 0) return
        for (i in 0 until 3) {
            val start = brakeSubT[i]
            val end = brakeSubT[i + 1]
            if (end <= start) continue
            segments.add(
                PolynomialCurveSegment(
                    start,
                    end,
                    brakeSubP[i],
                    brakeSubV[i],
                    0.5 * brakeSubA[i],
                    brakeSubJ[i] / 6.0,
                )
            )
        }
    }

    private fun addBrakeVelocitySegments(segments: MutableList<PolynomialCurveSegment>) {
        if (tBrake <= 0) return
        for (i in 0 until 3) {
            val start = brakeSubT[i]
            val end = brakeSubT[i + 1]
            if (end <= start) continue
            segments.add(
                PolynomialCurveSegment(
                    start,
                    end,
                    brakeSubV[i],
                    brakeSubA[i],
                    0.5 * brakeSubJ[i],
                    0.0,
                )
            )
        }
    }

    private fun addBrakeAccelerationSegments(segments: MutableList<PolynomialCurveSegment>) {
        if (tBrake <= 0) return
        for (i in 0 until 3) {
            val start = brakeSubT[i]
            val end = brakeSubT[i + 1]
            if (end <= start) continue
            segments.add(PolynomialCurveSegment(start, end, brakeSubA[i], brakeSubJ[i], 0.0, 0.0))
        }
    }

    companion object {
        // ---------------------------------------------------------------
        // Distance helpers
        // ---------------------------------------------------------------

        /**
         * Distance covered accelerating (with jerk-limit) from vFrom (a=0) to vPeak (a=0). Returns
         * 0 if vPeak <= vFrom.
         */
        private fun accelHalfDistFrom(
            vFrom: Double,
            vPeak: Double,
            aMax: Double,
            jMax: Double,
        ): Double {
            return accelHalfDistFrom(vFrom, vPeak, aMax, jMax, 0.0)
        }

        /**
         * Distance covered accelerating (with jerk-limit) from vFrom (a=aStart) to vPeak (a=0).
         * Returns 0 if vPeak <= vFrom.
         */
        private fun accelHalfDistFrom(
            vFrom: Double,
            vPeak: Double,
            aMax: Double,
            jMax: Double,
            aStart: Double,
        ): Double {
            val dv = vPeak - vFrom
            if (dv <= 0) return 0.0
            val trapThresh = (2.0 * aMax * aMax - aStart * aStart) / (2.0 * jMax)
            if (dv >= trapThresh) {
                // Trapezoidal: ramp aStart→aMax, hold, ramp aMax→0
                val T1p = (aMax - aStart) / jMax
                val T2p = (dv - trapThresh) / aMax
                val T3 = aMax / jMax
                val vEnd1 = vFrom + aStart * T1p + 0.5 * jMax * T1p * T1p
                val d1 =
                    vFrom * T1p + 0.5 * aStart * T1p * T1p + (1.0 / 6.0) * jMax * T1p * T1p * T1p
                val d2 = vEnd1 * T2p + 0.5 * aMax * T2p * T2p
                val vEnd2 = vEnd1 + aMax * T2p
                val d3 = vEnd2 * T3 + 0.5 * aMax * T3 * T3 - (1.0 / 6.0) * jMax * T3 * T3 * T3
                return d1 + d2 + d3
            } else {
                // Triangular: ramp aStart→aPk→0
                val aPk = sqrt(dv * jMax + aStart * aStart / 2.0)
                val T1p = (aPk - aStart) / jMax
                val T3 = aPk / jMax
                val vEnd1 = vFrom + aStart * T1p + 0.5 * jMax * T1p * T1p
                val d1 =
                    vFrom * T1p + 0.5 * aStart * T1p * T1p + (1.0 / 6.0) * jMax * T1p * T1p * T1p
                val d3 = vEnd1 * T3 + 0.5 * aPk * T3 * T3 - (1.0 / 6.0) * jMax * T3 * T3 * T3
                return d1 + d3
            }
        }

        /** Distance covered decelerating (jerk-limited) from vPeak (a=0) to 0 (a=0). */
        private fun halfDist(vPeak: Double, aMax: Double, jMax: Double): Double {
            return accelHalfDistFrom(0.0, vPeak, aMax, jMax)
        }
    }
}
