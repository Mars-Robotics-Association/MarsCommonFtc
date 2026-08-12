package org.marsroboticsassociation.controllib.motion

import java.util.ArrayList
import java.util.function.DoubleUnaryOperator
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import kotlin.math.withSign

/**
 * Seven-segment sinusoidal (raised-cosine) position trajectory from p0 to pTarget, ending at rest.
 * Supports asymmetric acceleration limits: aMaxAccel for speeding up, aMaxDecel for braking.
 *
 * <p>Instead of piecewise-linear (constant-jerk) acceleration transitions used by [SCurvePosition],
 * each transition is shaped with a raised-cosine acceleration curve, eliminating jerk
 * discontinuities at phase boundaries:
 * <pre>  a(t') = aMax/2 · (1 ± cos(πt'/T))</pre>
 *
 * <p>The seven main phases are:
 * <ol>
 * <li>Sinusoidal onset: a ramps from 0 → aMaxAccel (or triangular peak)
 * <li>Constant accel: a = aMaxAccel
 * <li>Sinusoidal offset: a ramps from aMaxAccel → 0
 * <li>Cruise: constant velocity vPeak
 * <li>Sinusoidal onset: a ramps from 0 → −aMaxDecel
 * <li>Constant decel: a = −aMaxDecel
 * <li>Sinusoidal offset: a ramps from −aMaxDecel → 0
 * </ol>
 *
 * <p>Optional prefix phases (applied before the 7-phase section):
 * <ul>
 * <li><b>a0 prefix</b>: if a0 ≠ 0, a quarter-cosine smoothly brings acceleration to 0.
 * <li><b>Braking prefix</b>: if the post-prefix velocity is in the wrong direction, a symmetric
 *   sinusoidal braking profile brings velocity to 0 before the main phases.
 * </ul>
 *
 * <p>The `jMax` parameter has the same semantics as in [SCurvePosition]: the sinusoidal transition
 * duration equals `aMax/jMax`, matching the linear-jerk transition time. The peak instantaneous
 * jerk is `π·jMax/2`.
 *
 * <p>For a single uninterrupted move, the profile is smooth in position, velocity, and
 * acceleration, and it removes the acceleration corner points of a standard 7-segment polynomial
 * S-curve. When used through [PositionTrajectoryManager], mid-motion replans also preserve `p/v/a`
 * because the next trajectory is seeded from the sampled current state. However, replans do not
 * preserve jerk continuity: rapid target changes may introduce sharp nodes in the acceleration
 * trace even though acceleration itself remains continuous.
 */
class SinCurvePosition(
    p0: Double,
    pTarget: Double,
    v0: Double,
    a0: Double,
    vMax: Double,
    aMaxAccel: Double,
    aMaxDecel: Double,
    jMax: Double,
) : PositionTrajectory {

    @JvmField val p0: Double
    @JvmField val pTarget: Double
    @JvmField val v0: Double
    @JvmField val a0: Double
    @JvmField val vMax: Double
    @JvmField val aMaxAccel: Double
    @JvmField val aMaxDecel: Double
    @JvmField val jMax: Double

    // Phase durations (7-phase section only, after any prefix phases)
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

    // a0 prefix: quarter-cosine from a0 to 0
    @JvmField val tPrefix: Double // duration (0 if a0 == 0)
    private val pAfterPrefix: Double
    private val vAfterPrefix: Double

    // Braking prefix: symmetric sinusoidal decel to bring wrong-way velocity to 0
    @JvmField val tBrake: Double // total duration (0 if not needed)
    private val pBrake: Double // position at end of braking

    // True when a0 != 0 and braking is active: the a0 prefix and braking onset are merged
    // into a single general arc (a0 → brkAmpl). In this case tPrefix stores the arc duration,
    // brakeSubDur[0] = 0 (onset skipped), and the braking constant/offset are recalculated.
    private val prefixMergedWithBrake: Boolean

    // Case B: braking offset + main T1 onset replaced by a single "handoff" arc
    // (brkAmpl→aPkAccel·dir).
    // Braking ends at v=0, a=brkAmpl (no offset phase). The handoff then transitions smoothly
    // to the main acceleration peak without returning to zero.
    @JvmField val handoffCombined: Boolean
    @JvmField val tHandoffStart: Double // absolute time at start of handoff
    @JvmField val tHandoffEnd: Double // absolute time at end of handoff
    @JvmField val aHandoffStart: Double // = brkAmpl (world frame)
    @JvmField val aHandoffEnd: Double // = aPkAccel * dir (world frame)
    @JvmField val pHandoffStart: Double // position at start of handoff (= pBrake)
    @JvmField val pHandoffEnd: Double // position at end of handoff (= phaseStartP[0])
    @JvmField val vHandoffEnd: Double // velocity at end of handoff (= phaseStartV[0])

    // Midpoint combined: when T4=0, T3-offset + T5-onset replaced by a single general arc
    // (accelAmpl → decelAmpl) stored in phase slot 2. Phase slots 3 and 4 have duration 0.
    @JvmField val midpointCombined: Boolean
    private val aMidStart: Double // accelAmpl = aPkAccel * dir (world frame)
    private val aMidEnd: Double // decelAmpl = -aPkDecel * dir (world frame)

    // Brake sub-phase table (3 sub-phases: onset, constant, offset)
    // brakeSubT[0..3] = absolute start times (4 boundaries for 3 sub-phases)
    private val brakeSubT = DoubleArray(4)
    private val brakeSubP = DoubleArray(4)
    private val brakeSubV = DoubleArray(4)
    private val brakeSubAmpl = DoubleArray(3) // signed accel amplitude per sub-phase
    private val brakeSubSign = IntArray(3) // -1 onset, 0 constant, +1 offset
    private val brakeSubDur = DoubleArray(3) // duration per sub-phase

    // Main 7-phase table: 8 breakpoints for 7 phases
    private val phaseStartT = DoubleArray(8) // absolute start times
    private val phaseStartP = DoubleArray(8)
    private val phaseStartV = DoubleArray(8)
    private val phaseAmpl = DoubleArray(7) // signed accel amplitude, world frame
    private val phaseSign = IntArray(7) // -1 onset, 0 constant, +1 offset

    init {
        this.p0 = p0
        this.pTarget = pTarget
        this.v0 = v0
        this.a0 = a0
        this.vMax = abs(vMax)
        this.aMaxAccel = abs(aMaxAccel)
        this.aMaxDecel = abs(aMaxDecel)
        this.jMax = abs(jMax)

        val totalDistAbs = abs(pTarget - p0)
        this.dir = if (pTarget >= p0) 1.0 else -1.0

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
            pAfterPrefix = p0
            vAfterPrefix = v0
            tBrake = 0.0
            pBrake = p0
            prefixMergedWithBrake = false
            handoffCombined = false
            tHandoffStart = 0.0
            tHandoffEnd = 0.0
            aHandoffStart = 0.0
            aHandoffEnd = 0.0
            pHandoffStart = p0
            pHandoffEnd = p0
            vHandoffEnd = 0.0
            midpointCombined = false
            aMidStart = 0.0
            aMidEnd = 0.0
            phaseStartP[0] = p0
            phaseStartV[0] = v0
            for (i in 1 until 8) {
                phaseStartT[i] = 0.0
                phaseStartP[i] = p0
                phaseStartV[i] = v0
            }
        } else {
            trivial = false

            // ---------------------------------------------------------------
            // a0 prefix: quarter-cosine from a0 to 0
            // (replaced by a general arc when Case A applies; see below)
            // ---------------------------------------------------------------
            var tPre: Double
            var pPre: Double
            var vPre: Double
            if (abs(a0) < 1e-9) {
                tPre = 0.0
                pPre = p0
                vPre = v0
            } else {
                tPre = abs(a0) / this.jMax
                vPre = v0 + a0 * (2.0 * tPre / PI)
                pPre = p0 + v0 * tPre + a0 * (4.0 * tPre * tPre / (PI * PI))
            }

            // Determine whether braking is needed (after the a0 prefix)
            val brakingNeeded = (vPre * dir < -1e-9)

            // ---------------------------------------------------------------
            // Case A: if a0 != 0 and braking is needed, merge the a0 prefix
            // and braking onset into a single general arc (a0 → brkAmpl).
            // ---------------------------------------------------------------
            var mergedPrefix = false
            var brkAmplForCaseA = 0.0

            if (abs(a0) > 1e-9 && brakingNeeded) {
                // Determine brkAmpl (sign: opposite to vPre)
                val aBrake = max(this.aMaxAccel, this.aMaxDecel)
                brkAmplForCaseA = -sign(vPre) * aBrake // signed world-frame amplitude

                // Combined arc: a0 → brkAmpl
                var T_A = abs(brkAmplForCaseA - a0) / this.jMax
                var delta_A = brkAmplForCaseA - a0
                var vEnd_A = evalVgen(v0, a0, delta_A, T_A, T_A)
                var pEnd_A = evalPgen(p0, v0, a0, delta_A, T_A, T_A)

                // Prefer preserving already-helpful braking acceleration on replans.
                // If a full-strength merged arc would over-brake, reduce the peak and keep the
                // merged
                // solution instead of bleeding acceleration back to zero and starting braking
                // again.
                if (vEnd_A * dir < -1e-9) {
                    tPre = T_A
                    vPre = vEnd_A
                    pPre = pEnd_A
                    mergedPrefix = true
                } else {
                    val a0Helpful = abs(a0)
                    val aPeakTri = sqrt((2.0 * abs(v0) * this.jMax + a0Helpful * a0Helpful) / 2.0)
                    if (aPeakTri >= a0Helpful + 1e-9) {
                        brkAmplForCaseA = aPeakTri.withSign(brkAmplForCaseA)
                        T_A = abs(brkAmplForCaseA - a0) / this.jMax
                        delta_A = brkAmplForCaseA - a0
                        vEnd_A = evalVgen(v0, a0, delta_A, T_A, T_A)
                        pEnd_A = evalPgen(p0, v0, a0, delta_A, T_A, T_A)
                        tPre = T_A
                        vPre = vEnd_A
                        pPre = pEnd_A
                        mergedPrefix = true
                    }
                }
            }

            this.tPrefix = tPre
            this.pAfterPrefix = pPre
            this.vAfterPrefix = vPre
            this.prefixMergedWithBrake = mergedPrefix

            // ---------------------------------------------------------------
            // Braking prefix: symmetric sinusoidal 3-sub-phase decel if vPre is wrong-way.
            // When Case A is active, the onset sub-phase (brakeSubDur[0]) is set to 0
            // (already absorbed into the prefix arc). tB2 is recalculated from vPre.
            // ---------------------------------------------------------------
            var tBrk: Double
            var pBrk: Double
            if (vPre * dir < -1e-9) {
                val speed = abs(vPre)
                val aBrake = max(this.aMaxAccel, this.aMaxDecel)
                val vBrakeMin = aBrake * aBrake / this.jMax
                var tB1: Double
                var tB2: Double
                var tB3: Double
                var aBrkPeak: Double

                if (mergedPrefix) {
                    // Case A: onset already handled; only need constant + offset to kill vPre.
                    // brkAmplForCaseA is signed; aBrkPeak is the magnitude.
                    aBrkPeak = abs(brkAmplForCaseA)
                    tB1 = 0.0 // onset absorbed into prefix
                    tB3 = aBrkPeak / this.jMax
                    // Solve for tB2: vPre + brkAmpl * tB2 + brkAmpl/2 * tB3 = 0
                    // => tB2 = -(vPre + brkAmpl/2 * tB3) / brkAmpl
                    var brkAmplSigned = brkAmplForCaseA
                    val tB2_candidate = -(vPre + brkAmplSigned / 2.0 * tB3) / brkAmplSigned
                    if (tB2_candidate >= 0) {
                        tB2 = tB2_candidate
                    } else {
                        // Triangular: reduce brkAmpl so tB2=0 is sufficient
                        // Simplified: aBrkPeak_tri = sqrt(|vPre| * jMax)
                        aBrkPeak = sqrt(speed * this.jMax)
                        tB3 = aBrkPeak / this.jMax
                        tB2 = 0.0
                        brkAmplSigned = -sign(vPre) * aBrkPeak
                        brkAmplForCaseA = brkAmplSigned // update for brakeSubAmpl[0]
                    }
                    brakeSubDur[0] = tB1 // = 0
                    brakeSubAmpl[0] = brkAmplSigned
                    brakeSubSign[0] = -1 // onset (zero duration, harmless)
                    brakeSubDur[1] = tB2
                    brakeSubAmpl[1] = brkAmplSigned
                    brakeSubSign[1] = 0
                    brakeSubDur[2] = tB3
                    brakeSubAmpl[2] = brkAmplSigned
                    brakeSubSign[2] = +1
                } else {
                    // Normal braking (no Case A)
                    if (speed >= vBrakeMin) {
                        tB1 = aBrake / this.jMax
                        tB2 = (speed - vBrakeMin) / aBrake
                        tB3 = tB1
                        aBrkPeak = aBrake
                    } else {
                        aBrkPeak = sqrt(speed * this.jMax)
                        tB1 = aBrkPeak / this.jMax
                        tB2 = 0.0
                        tB3 = tB1
                    }
                    val brkAmpl = -sign(vPre) * aBrkPeak
                    brakeSubDur[0] = tB1
                    brakeSubAmpl[0] = brkAmpl
                    brakeSubSign[0] = -1
                    brakeSubDur[1] = tB2
                    brakeSubAmpl[1] = brkAmpl
                    brakeSubSign[1] = 0
                    brakeSubDur[2] = tB3
                    brakeSubAmpl[2] = brkAmpl
                    brakeSubSign[2] = +1
                }

                tBrk = brakeSubDur[0] + brakeSubDur[1] + brakeSubDur[2]
                brakeSubT[0] = tPre
                brakeSubP[0] = pPre
                brakeSubV[0] = vPre
                for (i in 0 until 3) {
                    val end =
                        evalPhaseEnd(
                            brakeSubP[i],
                            brakeSubV[i],
                            brakeSubAmpl[i],
                            brakeSubSign[i],
                            brakeSubDur[i],
                        )
                    brakeSubT[i + 1] = brakeSubT[i] + brakeSubDur[i]
                    brakeSubP[i + 1] = end[0]
                    brakeSubV[i + 1] = end[1]
                }
                pBrk = brakeSubP[3]
            } else {
                tBrk = 0.0
                pBrk = pPre
            }
            // ---------------------------------------------------------------
            // Main 7-phase setup — Pass 1 (original braking structure with offset)
            // ---------------------------------------------------------------
            val distRemaining = max(0.0, dir * (pTarget - pBrk))
            val v0adj = if (tBrk > 0) 0.0 else min(max(vPre * dir, 0.0), this.vMax)

            var vPeakSolved: Double
            var T4solved: Double
            run {
                val dAtVMax =
                    sinHalfDist(v0adj, this.vMax, this.aMaxAccel, this.jMax) +
                        sinHalfDist(0.0, this.vMax, this.aMaxDecel, this.jMax)
                if (distRemaining >= dAtVMax) {
                    vPeakSolved = this.vMax
                    T4solved = if (this.vMax > 1e-9) (distRemaining - dAtVMax) / this.vMax else 0.0
                } else {
                    T4solved = 0.0
                    val dAtV0adj = sinHalfDist(0.0, v0adj, this.aMaxDecel, this.jMax)
                    if (distRemaining <= dAtV0adj) {
                        vPeakSolved = v0adj
                    } else {
                        var lo = v0adj
                        var hi = this.vMax
                        for (i in 0 until 64) {
                            val mid = (lo + hi) * 0.5
                            val d =
                                sinFullDistCombinedMid(
                                    v0adj,
                                    mid,
                                    this.aMaxAccel,
                                    this.aMaxDecel,
                                    this.jMax,
                                )
                            if (d < distRemaining) lo = mid else hi = mid
                        }
                        vPeakSolved = (lo + hi) * 0.5
                    }
                }
            }

            // ---------------------------------------------------------------
            // Case B: check and optional pass 2 with handoff arc
            //
            // When braking is active and the main acceleration is in the same direction as
            // the braking force, replace braking-offset + main-T1-onset with a single smooth
            // "handoff" arc (brkAmpl → aPkAccel·dir). Braking is modified to end at v=0,
            // a=brkAmpl (offset removed; tB2 is adjusted instead).
            // ---------------------------------------------------------------
            var doHandoff = false

            if (tBrk > 0 && vPeakSolved > 0) {
                val dvAccel_0 = vPeakSolved - v0adj
                var aPkAccel_0 = 0.0
                if (dvAccel_0 > 0) {
                    val vAccelMin_0 = this.aMaxAccel * this.aMaxAccel / this.jMax
                    aPkAccel_0 =
                        if (dvAccel_0 >= vAccelMin_0) this.aMaxAccel
                        else sqrt(dvAccel_0 * this.jMax)
                }
                val brkAmplSigned = brakeSubAmpl[0] // e.g. negative for rightward braking
                val brkA = abs(brkAmplSigned)
                val vMinHandoff = brkA * brkA / (2.0 * this.jMax)

                // Case B applies when brkAmpl and aPkAccel·dir have the same sign (both point
                // toward the new target direction), and vPeak is large enough for the handoff.
                if (
                    aPkAccel_0 > 1e-9 &&
                        brkAmplSigned * aPkAccel_0 * dir > 0 &&
                        vPeakSolved >= vMinHandoff
                ) {

                    // Modify braking: remove offset, adjust tB2 so braking ends at v=0, a=brkAmpl.
                    // v_after_onset = brakeSubV[1] (velocity after onset sub-phase)
                    // tB2_new = -v_after_onset / brkAmplSigned  (always >= 0 since they have
                    // opposite
                    // signs)
                    val v_after_onset = brakeSubV[1]
                    val tB2_new = max(0.0, -v_after_onset / brkAmplSigned)
                    brakeSubDur[1] = tB2_new
                    brakeSubDur[2] = 0.0 // offset removed

                    // Re-forward-chain to get new pBrk
                    brakeSubT[0] = tPre
                    brakeSubP[0] = pPre
                    brakeSubV[0] = vPre
                    for (i in 0 until 3) {
                        val end =
                            evalPhaseEnd(
                                brakeSubP[i],
                                brakeSubV[i],
                                brakeSubAmpl[i],
                                brakeSubSign[i],
                                brakeSubDur[i],
                            )
                        brakeSubT[i + 1] = brakeSubT[i] + brakeSubDur[i]
                        brakeSubP[i + 1] = end[0]
                        brakeSubV[i + 1] = end[1]
                    }
                    tBrk = brakeSubDur[0] + brakeSubDur[1] + brakeSubDur[2]
                    pBrk = brakeSubP[3]

                    // Pass 2 bisect using sinHalfDistCombined
                    val distRemaining2 = max(0.0, dir * (pTarget - pBrk))
                    val dAtVMaxCombined =
                        sinHalfDistCombined(v0adj, this.vMax, this.aMaxAccel, this.jMax, brkA) +
                            sinHalfDist(0.0, this.vMax, this.aMaxDecel, this.jMax)
                    if (distRemaining2 >= dAtVMaxCombined) {
                        vPeakSolved = this.vMax
                        T4solved =
                            if (this.vMax > 1e-9) (distRemaining2 - dAtVMaxCombined) / this.vMax
                            else 0.0
                    } else {
                        T4solved = 0.0
                        val dAtV0adjCombined = sinHalfDist(0.0, v0adj, this.aMaxDecel, this.jMax)
                        if (distRemaining2 <= dAtV0adjCombined) {
                            vPeakSolved = v0adj
                        } else {
                            var lo = v0adj
                            var hi = this.vMax
                            for (i in 0 until 64) {
                                val mid = (lo + hi) * 0.5
                                val d =
                                    sinFullDistHandoffAndMid(
                                        v0adj,
                                        mid,
                                        this.aMaxAccel,
                                        this.aMaxDecel,
                                        this.jMax,
                                        brkA,
                                    )
                                if (d < distRemaining2) lo = mid else hi = mid
                            }
                            vPeakSolved = (lo + hi) * 0.5
                        }
                    }

                    // Confirm handoff is still worthwhile with updated vPeak
                    if (vPeakSolved - v0adj > 1e-9) {
                        doHandoff = true
                    }
                }
            }

            this.tBrake = tBrk
            this.pBrake = pBrk
            this.vPeak = vPeakSolved
            this.T4 = max(0.0, T4solved)

            // ---------------------------------------------------------------
            // Accel phase durations
            // ---------------------------------------------------------------
            val dvAccel = vPeakSolved - v0adj
            val vAccelMin = this.aMaxAccel * this.aMaxAccel / this.jMax
            var t1: Double
            var t2: Double
            var t3: Double
            val aPkAccel: Double
            if (dvAccel <= 0) {
                t1 = 0.0
                t2 = 0.0
                t3 = 0.0
                aPkAccel = 0.0
            } else if (dvAccel >= vAccelMin) {
                aPkAccel = this.aMaxAccel
                t1 = this.aMaxAccel / this.jMax
                t2 = (dvAccel - vAccelMin) / this.aMaxAccel
                t3 = t1
            } else {
                aPkAccel = sqrt(dvAccel * this.jMax)
                t1 = aPkAccel / this.jMax
                t2 = 0.0
                t3 = t1
            }

            // Decel phase durations
            val vDecelMin = this.aMaxDecel * this.aMaxDecel / this.jMax
            var t5: Double
            var t6: Double
            var t7: Double
            val aPkDecel: Double
            if (vPeakSolved <= 0) {
                t5 = 0.0
                t6 = 0.0
                t7 = 0.0
                aPkDecel = 0.0
            } else if (vPeakSolved >= vDecelMin) {
                aPkDecel = this.aMaxDecel
                t5 = this.aMaxDecel / this.jMax
                t6 = (vPeakSolved - vDecelMin) / this.aMaxDecel
                t7 = t5
            } else {
                aPkDecel = sqrt(vPeakSolved * this.jMax)
                t5 = aPkDecel / this.jMax
                t6 = 0.0
                t7 = t5
            }

            // When Case B is active, T1 is absorbed into the handoff arc.
            this.T1 = if (doHandoff) 0.0 else max(0.0, t1)
            if (doHandoff && aPkAccel > 1e-9) {
                // T2 must be recomputed: T1=0 so velocity entering T2 is vHEnd (end of handoff
                // arc),
                // not v0adj. Standard formula (dvAccel - vAccelMin)/aPkAccel assumes onset starts
                // at
                // v=0; here the handoff arc already gained vHEnd = T_H*(brkA+aPkAccel)/2.
                val brkA = abs(brakeSubAmpl[0])
                val T_H = abs(aPkAccel - brkA) / this.jMax
                val vHEnd_calc = T_H * (brkA + aPkAccel) / 2.0
                t2 =
                    max(
                        0.0,
                        (vPeakSolved - vHEnd_calc - aPkAccel * aPkAccel / (2.0 * this.jMax)) /
                            aPkAccel,
                    )
            }
            // Midpoint combined: when T4=0 and both accel/decel peaks are nonzero,
            // replace T3-offset + T5-onset with a single general arc.
            val doMidpoint = (T4solved == 0.0) && (aPkAccel > 1e-9) && (aPkDecel > 1e-9)
            if (doMidpoint) {
                t3 = (aPkAccel + aPkDecel) / this.jMax // combined arc stored in T3 slot
                t5 = 0.0 // T5 absorbed into combined arc
            }

            this.T2 = max(0.0, t2)
            this.T3 = max(0.0, t3)
            this.T5 = max(0.0, t5)
            this.T6 = max(0.0, t6)
            this.T7 = max(0.0, t7)

            // ---------------------------------------------------------------
            // Case B handoff arc: compute endpoint state
            // ---------------------------------------------------------------
            var tHStart = 0.0
            var tHEnd = 0.0
            var aHStart = 0.0
            var aHEnd = 0.0
            var pHStart = pBrk
            var pHEnd = pBrk
            var vHEnd = 0.0

            if (doHandoff) {
                aHStart = brakeSubAmpl[0] // brkAmpl (world frame, e.g. negative)
                aHEnd = aPkAccel * dir // aPkAccel·dir (world frame)
                tHStart = tPre + tBrk
                val T_h = abs(aHEnd - aHStart) / this.jMax
                tHEnd = tHStart + T_h
                pHStart = pBrk
                val handoffEnd = evalPhaseEndGen(pBrk, 0.0, aHStart, aHEnd, T_h)
                pHEnd = handoffEnd[0]
                vHEnd = handoffEnd[1]
            }

            this.handoffCombined = doHandoff
            this.tHandoffStart = tHStart
            this.tHandoffEnd = tHEnd
            this.aHandoffStart = aHStart
            this.aHandoffEnd = aHEnd
            this.pHandoffStart = pHStart
            this.pHandoffEnd = pHEnd
            this.vHandoffEnd = vHEnd

            this.midpointCombined = doMidpoint
            if (doMidpoint) {
                this.aMidStart = aPkAccel * dir // accelAmpl (world frame)
                this.aMidEnd = -aPkDecel * dir // decelAmpl (world frame)
            } else {
                this.aMidStart = 0.0
                this.aMidEnd = 0.0
            }

            // ---------------------------------------------------------------
            // Build phase table
            //
            // Phase signs and amplitudes in world frame:
            //   0 (T1): onset,    ampl = +aPkAccel*dir  (a: 0 -> aPkAccel*dir)  [T1=0 when Case B]
            //   1 (T2): constant, ampl = +aPkAccel*dir  (a = aPkAccel*dir)
            //   2 (T3): offset,   ampl = +aPkAccel*dir  (a: aPkAccel*dir -> 0)  [midpoint arc when
            // combined]
            //   3 (T4): constant, ampl = 0              (cruise)
            //   4 (T5): onset,    ampl = -aPkDecel*dir  (a: 0 -> -aPkDecel*dir)
            //   5 (T6): constant, ampl = -aPkDecel*dir  (a = -aPkDecel*dir)
            //   6 (T7): offset,   ampl = -aPkDecel*dir  (a: -aPkDecel*dir -> 0)
            // ---------------------------------------------------------------
            val accelAmpl = aPkAccel * dir
            val decelAmpl = -aPkDecel * dir

            phaseSign[0] = -1
            phaseAmpl[0] = accelAmpl
            phaseSign[1] = 0
            phaseAmpl[1] = accelAmpl
            phaseSign[2] = +1
            phaseAmpl[2] = accelAmpl
            phaseSign[3] = 0
            phaseAmpl[3] = 0.0
            phaseSign[4] = -1
            phaseAmpl[4] = decelAmpl
            phaseSign[5] = 0
            phaseAmpl[5] = decelAmpl
            phaseSign[6] = +1
            phaseAmpl[6] = decelAmpl

            val durations =
                doubleArrayOf(this.T1, this.T2, this.T3, this.T4, this.T5, this.T6, this.T7)
            phaseStartT[0] = tPre + tBrk + (if (doHandoff) (tHEnd - tHStart) else 0.0)
            phaseStartP[0] = if (doHandoff) pHEnd else pBrk
            phaseStartV[0] = if (doHandoff) vHEnd else (v0adj * dir)

            for (i in 0 until 7) {
                val end: DoubleArray
                if (midpointCombined && i == 2) {
                    end =
                        evalPhaseEndGen(
                            phaseStartP[i],
                            phaseStartV[i],
                            aMidStart,
                            aMidEnd,
                            durations[i],
                        )
                } else {
                    end =
                        evalPhaseEnd(
                            phaseStartP[i],
                            phaseStartV[i],
                            phaseAmpl[i],
                            phaseSign[i],
                            durations[i],
                        )
                }
                phaseStartT[i + 1] = phaseStartT[i] + durations[i]
                phaseStartP[i + 1] = end[0]
                phaseStartV[i + 1] = end[1]
            }
        }
    }

    // ---------------------------------------------------------------
    // PositionTrajectory interface
    // ---------------------------------------------------------------

    override fun getPosition(t: Double): Double {
        if (trivial) return p0
        if (t <= 0) return p0
        val tf = getTotalTime()
        if (t >= tf) return pTarget

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                val delta = brakeSubAmpl[0] - a0
                return evalPgen(p0, v0, a0, delta, t, tPrefix)
            }
            // p(t) = p0 + v0*t + a0*(4*tPrefix^2/pi^2)*(1 - cos(pi*t/(2*tPrefix)))
            return p0 +
                v0 * t +
                a0 * (4.0 * tPrefix * tPrefix / (PI * PI)) * (1.0 - cos(PI * t / (2.0 * tPrefix)))
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            val dt = t - brakeSubT[bph]
            return evalP(
                brakeSubP[bph],
                brakeSubV[bph],
                brakeSubAmpl[bph],
                brakeSubSign[bph],
                dt,
                brakeSubDur[bph],
            )
        }

        if (handoffCombined && t < tHandoffEnd) {
            val dt = t - tHandoffStart
            val T_h = tHandoffEnd - tHandoffStart
            return evalPgen(pHandoffStart, 0.0, aHandoffStart, aHandoffEnd - aHandoffStart, dt, T_h)
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        val T = phaseStartT[ph + 1] - phaseStartT[ph]
        if (midpointCombined && ph == 2) {
            return evalPgen(phaseStartP[2], phaseStartV[2], aMidStart, aMidEnd - aMidStart, dt, T)
        }
        return evalP(phaseStartP[ph], phaseStartV[ph], phaseAmpl[ph], phaseSign[ph], dt, T)
    }

    override fun getVelocity(t: Double): Double {
        if (trivial) return v0
        if (t <= 0) return v0
        val tf = getTotalTime()
        if (t >= tf) return 0.0

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                val delta = brakeSubAmpl[0] - a0
                return evalVgen(v0, a0, delta, t, tPrefix)
            }
            // v(t) = v0 + a0*(2*tPrefix/pi)*sin(pi*t/(2*tPrefix))
            return v0 + a0 * (2.0 * tPrefix / PI) * sin(PI * t / (2.0 * tPrefix))
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            val dt = t - brakeSubT[bph]
            return evalV(brakeSubV[bph], brakeSubAmpl[bph], brakeSubSign[bph], dt, brakeSubDur[bph])
        }

        if (handoffCombined && t < tHandoffEnd) {
            val dt = t - tHandoffStart
            val T_h = tHandoffEnd - tHandoffStart
            return evalVgen(0.0, aHandoffStart, aHandoffEnd - aHandoffStart, dt, T_h)
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        val T = phaseStartT[ph + 1] - phaseStartT[ph]
        if (midpointCombined && ph == 2) {
            return evalVgen(phaseStartV[2], aMidStart, aMidEnd - aMidStart, dt, T)
        }
        return evalV(phaseStartV[ph], phaseAmpl[ph], phaseSign[ph], dt, T)
    }

    override fun getAcceleration(t: Double): Double {
        if (trivial) return a0
        if (t <= 0) return a0
        val tf = getTotalTime()
        if (t >= tf) return 0.0

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                val aStart = a0
                val delta = brakeSubAmpl[0] - a0
                return evalAgen(aStart, delta, t, tPrefix)
            }
            // a(t) = a0 * cos(pi*t / (2*tPrefix))
            return a0 * cos(PI * t / (2.0 * tPrefix))
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            val dt = t - brakeSubT[bph]
            return evalA(brakeSubAmpl[bph], brakeSubSign[bph], dt, brakeSubDur[bph])
        }

        if (handoffCombined && t < tHandoffEnd) {
            val dt = t - tHandoffStart
            val T_h = tHandoffEnd - tHandoffStart
            return evalAgen(aHandoffStart, aHandoffEnd - aHandoffStart, dt, T_h)
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        val T = phaseStartT[ph + 1] - phaseStartT[ph]
        if (midpointCombined && ph == 2) {
            return evalAgen(aMidStart, aMidEnd - aMidStart, dt, T)
        }
        return evalA(phaseAmpl[ph], phaseSign[ph], dt, T)
    }

    override fun getTotalTime(): Double {
        return if (trivial) 0.0 else phaseStartT[7]
    }

    override fun isZeroJerk(t: Double): Boolean {
        if (trivial) return true
        if (t <= 0 || t >= getTotalTime()) return true
        if (tPrefix > 0 && t < tPrefix) return false // active sinusoidal prefix
        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            return brakeSubSign[findBrakePhase(t)] == 0
        }
        if (handoffCombined && t < tHandoffEnd) return false // active handoff arc
        if (midpointCombined && findPhase(t) == 2) return false
        return phaseSign[findPhase(t)] == 0
    }

    fun getJerk(t: Double): Double {
        if (trivial) return 0.0
        if (t <= 0 || t >= getTotalTime()) return 0.0

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                return evalJgen(brakeSubAmpl[0] - a0, t, tPrefix)
            }
            return -a0 * PI * sin(PI * t / (2.0 * tPrefix)) / (2.0 * tPrefix)
        }

        val tBrakeEnd = tPrefix + tBrake
        if (tBrake > 0 && t < tBrakeEnd) {
            val bph = findBrakePhase(t)
            return evalJ(brakeSubAmpl[bph], brakeSubSign[bph], t - brakeSubT[bph], brakeSubDur[bph])
        }

        if (handoffCombined && t < tHandoffEnd) {
            val T_h = tHandoffEnd - tHandoffStart
            return evalJgen(aHandoffEnd - aHandoffStart, t - tHandoffStart, T_h)
        }

        val ph = findPhase(t)
        val dt = t - phaseStartT[ph]
        val T = phaseStartT[ph + 1] - phaseStartT[ph]
        if (midpointCombined && ph == 2) {
            return evalJgen(aMidEnd - aMidStart, dt, T)
        }
        return evalJ(phaseAmpl[ph], phaseSign[ph], dt, T)
    }

    fun positionSegments(): List<TrajectoryCurveSegment> {
        return buildCurveSegments(
            DoubleUnaryOperator { getPosition(it) },
            DoubleUnaryOperator { getVelocity(it) },
        )
    }

    fun velocitySegments(): List<TrajectoryCurveSegment> {
        return buildCurveSegments(
            DoubleUnaryOperator { getVelocity(it) },
            DoubleUnaryOperator { getAcceleration(it) },
        )
    }

    fun accelerationSegments(): List<TrajectoryCurveSegment> {
        return buildCurveSegments(
            DoubleUnaryOperator { getAcceleration(it) },
            DoubleUnaryOperator { getJerk(it) },
        )
    }

    private fun buildCurveSegments(
        valueFunction: DoubleUnaryOperator,
        slopeFunction: DoubleUnaryOperator,
    ): List<TrajectoryCurveSegment> {
        val segments = ArrayList<TrajectoryCurveSegment>()
        if (trivial) return segments

        addCurveSegment(segments, 0.0, tPrefix, valueFunction, slopeFunction)
        for (i in 0 until 3) {
            addCurveSegment(segments, brakeSubT[i], brakeSubT[i + 1], valueFunction, slopeFunction)
        }
        if (handoffCombined) {
            addCurveSegment(segments, tHandoffStart, tHandoffEnd, valueFunction, slopeFunction)
        }
        for (i in 0 until 7) {
            addCurveSegment(
                segments,
                phaseStartT[i],
                phaseStartT[i + 1],
                valueFunction,
                slopeFunction,
            )
        }
        return segments
    }

    // ---------------------------------------------------------------
    // Phase lookup
    // ---------------------------------------------------------------

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

    companion object {
        // Integral coefficients for sinusoidal ramp distance contribution over one full phase:
        //   onset  phase: distance += ampl * T^2 * C_ONSET
        //   offset phase: distance += ampl * T^2 * C_OFFSET
        // Derived from integral of the raised-cosine velocity formula:
        //   C_ONSET  = 1/4 - 1/pi^2 = (pi^2 - 4) / (4*pi^2)
        //   C_OFFSET = 1/4 + 1/pi^2 = (pi^2 + 4) / (4*pi^2)
        @JvmField internal val C_ONSET = (PI * PI - 4.0) / (4.0 * PI * PI)
        @JvmField internal val C_OFFSET = (PI * PI + 4.0) / (4.0 * PI * PI)

        private fun addCurveSegment(
            segments: MutableList<TrajectoryCurveSegment>,
            startTime: Double,
            endTime: Double,
            valueFunction: DoubleUnaryOperator,
            slopeFunction: DoubleUnaryOperator,
        ) {
            if (endTime <= startTime) return
            segments.add(FunctionalCurveSegment(startTime, endTime, valueFunction, slopeFunction))
        }

        // ---------------------------------------------------------------
        // Unified sinusoidal phase evaluation helpers
        //
        // All formulas derive from the unified acceleration shape:
        //   a(t') = ampl/2 * (1 + sign * cos(pi*t'/T))
        // which gives:
        //   v(t') = vStar + ampl/2 * (t' + sign * T * sin(pi*t'/T) / pi)
        //   p(t') = pStar + vStar*t' + ampl/2 * (t'^2/2 + sign * T^2 * (1 - cos(pi*t'/T)) / pi^2)
        // For sign=0 (constant phases): reduces to standard polynomial kinematics.
        // ---------------------------------------------------------------

        private fun evalP(
            pStar: Double,
            vStar: Double,
            ampl: Double,
            sign: Int,
            dt: Double,
            T: Double,
        ): Double {
            if (sign == 0) {
                return pStar + vStar * dt + 0.5 * ampl * dt * dt
            }
            val cosVal = if (T > 1e-15) cos(PI * dt / T) else 1.0
            return pStar +
                vStar * dt +
                (ampl / 2.0) * (dt * dt / 2.0 + sign * T * T * (1.0 - cosVal) / (PI * PI))
        }

        private fun evalV(vStar: Double, ampl: Double, sign: Int, dt: Double, T: Double): Double {
            if (sign == 0) {
                return vStar + ampl * dt
            }
            val sinVal = if (T > 1e-15) sin(PI * dt / T) else 0.0
            return vStar + (ampl / 2.0) * (dt + sign * T * sinVal / PI)
        }

        private fun evalA(ampl: Double, sign: Int, dt: Double, T: Double): Double {
            if (sign == 0) {
                return ampl
            }
            val cosVal = if (T > 1e-15) cos(PI * dt / T) else 1.0
            return (ampl / 2.0) * (1.0 + sign * cosVal)
        }

        private fun evalJ(ampl: Double, sign: Int, dt: Double, T: Double): Double {
            if (sign == 0 || T <= 1e-15) {
                return 0.0
            }
            return -(ampl * sign * PI / (2.0 * T)) * sin(PI * dt / T)
        }

        /** Returns [p, v] at the end of a phase. Used for forward-chaining the phase table. */
        private fun evalPhaseEnd(
            pStar: Double,
            vStar: Double,
            ampl: Double,
            sign: Int,
            T: Double,
        ): DoubleArray {
            return doubleArrayOf(
                evalP(pStar, vStar, ampl, sign, T, T),
                evalV(vStar, ampl, sign, T, T),
            )
        }

        /**
         * General sinusoidal arc: a goes from aStart to (aStart+delta) over duration T. Reduces to
         * evalA when aStart=0 (onset, sign=-1) or delta=-aStart (offset, sign=+1).
         */
        private fun evalAgen(aStart: Double, delta: Double, dt: Double, T: Double): Double {
            val cosVal = if (T > 1e-15) cos(PI * dt / T) else 1.0
            return aStart + (delta / 2.0) * (1.0 - cosVal)
        }

        private fun evalJgen(delta: Double, dt: Double, T: Double): Double {
            if (T <= 1e-15) {
                return 0.0
            }
            return (delta * PI / (2.0 * T)) * sin(PI * dt / T)
        }

        private fun evalVgen(
            vStar: Double,
            aStart: Double,
            delta: Double,
            dt: Double,
            T: Double,
        ): Double {
            val sinVal = if (T > 1e-15) sin(PI * dt / T) else 0.0
            return vStar + aStart * dt + (delta / 2.0) * (dt - T * sinVal / PI)
        }

        private fun evalPgen(
            pStar: Double,
            vStar: Double,
            aStart: Double,
            delta: Double,
            dt: Double,
            T: Double,
        ): Double {
            val cosVal = if (T > 1e-15) cos(PI * dt / T) else 1.0
            // p(t') = pStar + vStar*t' + (aStart + delta/2)*t'^2/2 -
            // delta/2*T^2*(1-cos(pi*t'/T))/pi^2
            // Derived by integrating v(t') = vStar + aStart*t' + delta/2*(t' - T*sin(pi*t'/T)/pi)
            return pStar + vStar * dt + (aStart + delta / 2.0) * dt * dt / 2.0 -
                (delta / 2.0) * T * T * (1.0 - cosVal) / (PI * PI)
        }

        /** Returns [p, v] at the end of a general arc. */
        private fun evalPhaseEndGen(
            pStar: Double,
            vStar: Double,
            aStart: Double,
            aEnd: Double,
            T: Double,
        ): DoubleArray {
            val delta = aEnd - aStart
            return doubleArrayOf(
                evalPgen(pStar, vStar, aStart, delta, T, T),
                evalVgen(vStar, aStart, delta, T, T),
            )
        }

        // ---------------------------------------------------------------
        // Sinusoidal distance helper
        // ---------------------------------------------------------------

        /**
         * Distance covered during a sinusoidal acceleration from v0 (a=0) to vPeak (a=0). Uses
         * raised-cosine ramp shape (trapezoidal or triangular depending on Δv vs aMax²/jMax).
         * Returns 0 if vPeak ≤ v0.
         */
        @JvmStatic
        fun sinHalfDist(v0: Double, vPeak: Double, aMax: Double, jMax: Double): Double {
            val dv = vPeak - v0
            if (dv <= 0) return 0.0
            val vMin = aMax * aMax / jMax // triangular threshold
            if (dv >= vMin) {
                // Trapezoidal: onset (0->aMax), constant (aMax), offset (aMax->0)
                val T1 = aMax / jMax
                val T3 = T1
                val T2 = (dv - vMin) / aMax
                val v1 = v0 + aMax * T1 / 2.0
                val v2 = v1 + aMax * T2
                val d1 = v0 * T1 + aMax * T1 * T1 * C_ONSET
                val d2 = v1 * T2 + 0.5 * aMax * T2 * T2
                val d3 = v2 * T3 + aMax * T3 * T3 * C_OFFSET
                return d1 + d2 + d3
            } else {
                // Triangular: onset (0->aPeak), offset (aPeak->0)
                val aPk = sqrt(dv * jMax)
                val T1 = aPk / jMax
                val T3 = T1
                val v1 = v0 + aPk * T1 / 2.0
                val d1 = v0 * T1 + aPk * T1 * T1 * C_ONSET
                val d3 = v1 * T3 + aPk * T3 * T3 * C_OFFSET
                return d1 + d3
            }
        }

        /**
         * Like [sinHalfDist] but for the acceleration half when Case B (braking handoff) is active.
         * Replaces the normal onset (0 → aPkAccel) with a combined arc (brkA → aPkAccel), starting
         * from v = 0 (braking already brought velocity to zero).
         *
         * @param v0adj unused (always 0 when Case B applies; kept for call-site symmetry)
         * @param vPeak target peak velocity
         * @param aMaxAccel maximum acceleration magnitude
         * @param jMax jerk limit
         * @param brkA braking amplitude magnitude (|brkAmpl|)
         */
        @JvmStatic
        internal fun sinHalfDistCombined(
            v0adj: Double,
            vPeak: Double,
            aMaxAccel: Double,
            jMax: Double,
            brkA: Double,
        ): Double {
            if (vPeak <= 0) return 0.0
            val vMin = aMaxAccel * aMaxAccel / jMax
            val vMinHandoff = brkA * brkA / (2.0 * jMax)

            // Triangular case: aPkAccel < aMaxAccel AND aMaxAccel >= brkA (ascending handoff)
            // Threshold: vPeak <= vMin - vMinHandoff  (derived from aPkAccel reaching aMaxAccel at
            // that
            // vPeak)
            val aPkAccel: Double
            val T_H: Double
            val T2: Double
            if (aMaxAccel >= brkA && vPeak <= vMin - vMinHandoff) {
                // Triangular: aPkAccel = sqrt(jMax * vPeak + brkA²/2)
                aPkAccel = sqrt(jMax * vPeak + brkA * brkA / 2.0)
                T_H = (aPkAccel - brkA) / jMax // ascending arc, aPkAccel >= brkA
                T2 = 0.0
            } else {
                // Trapezoidal: aPkAccel = aMaxAccel (ascending or descending handoff)
                aPkAccel = aMaxAccel
                T_H = abs(aPkAccel - brkA) / jMax
                val v_after_H = T_H * (brkA + aPkAccel) / 2.0
                T2 =
                    max(
                        0.0,
                        (vPeak - v_after_H - aPkAccel * aPkAccel / (2.0 * jMax)) / aPkAccel,
                    )
            }
            val T3 = aPkAccel / jMax
            val v_after_H = T_H * (brkA + aPkAccel) / 2.0

            // Distance during handoff arc (a: brkA → aPkAccel), starting from v=0, p=0
            val delta_H = aPkAccel - brkA
            val d_H = evalPgen(0.0, 0.0, brkA, delta_H, T_H, T_H)

            // Distance during constant phase (a = aPkAccel)
            val d_T2 = v_after_H * T2 + 0.5 * aPkAccel * T2 * T2

            // Distance during offset phase (a: aPkAccel → 0)
            val v_start_T3 = v_after_H + aPkAccel * T2
            val d_T3 = v_start_T3 * T3 + aPkAccel * T3 * T3 * C_OFFSET

            return d_H + d_T2 + d_T3
        }

        /** Full no-cruise distance when T3+T5 are replaced by a single midpoint arc. */
        @JvmStatic
        internal fun sinFullDistCombinedMid(
            v0adj: Double,
            vPeak: Double,
            aMaxAccel: Double,
            aMaxDecel: Double,
            jMax: Double,
        ): Double {
            if (vPeak <= v0adj) {
                return sinHalfDist(0.0, vPeak, aMaxDecel, jMax)
            }

            val dvAccel = vPeak - v0adj
            val vAccelMin = aMaxAccel * aMaxAccel / jMax
            val aPkAccel: Double
            val T1: Double
            val T2: Double
            if (dvAccel >= vAccelMin) {
                aPkAccel = aMaxAccel
                T1 = aPkAccel / jMax
                T2 = (dvAccel - vAccelMin) / aPkAccel
            } else {
                aPkAccel = sqrt(dvAccel * jMax)
                T1 = aPkAccel / jMax
                T2 = 0.0
            }

            val vDecelMin = aMaxDecel * aMaxDecel / jMax
            val aPkDecel: Double
            val T6: Double
            val T7: Double
            if (vPeak >= vDecelMin) {
                aPkDecel = aMaxDecel
                T6 = (vPeak - vDecelMin) / aPkDecel
                T7 = aPkDecel / jMax
            } else {
                aPkDecel = sqrt(vPeak * jMax)
                T6 = 0.0
                T7 = aPkDecel / jMax
            }

            var p = 0.0
            var v = v0adj

            var end = evalPhaseEnd(p, v, aPkAccel, -1, T1)
            p = end[0]
            v = end[1]

            end = evalPhaseEnd(p, v, aPkAccel, 0, T2)
            p = end[0]
            v = end[1]

            val Tmid = (aPkAccel + aPkDecel) / jMax
            end = evalPhaseEndGen(p, v, aPkAccel, -aPkDecel, Tmid)
            p = end[0]
            v = end[1]

            end = evalPhaseEnd(p, v, -aPkDecel, 0, T6)
            p = end[0]
            v = end[1]

            end = evalPhaseEnd(p, v, -aPkDecel, +1, T7)
            return end[0]
        }

        /** Full no-cruise distance when both the braking handoff and midpoint arc are combined. */
        @JvmStatic
        internal fun sinFullDistHandoffAndMid(
            v0adj: Double,
            vPeak: Double,
            aMaxAccel: Double,
            aMaxDecel: Double,
            jMax: Double,
            brkA: Double,
        ): Double {
            if (vPeak <= 0) {
                return 0.0
            }

            val vMin = aMaxAccel * aMaxAccel / jMax
            val vMinHandoff = brkA * brkA / (2.0 * jMax)
            val aPkAccel: Double
            val T_H: Double
            val T2: Double
            if (aMaxAccel >= brkA && vPeak <= vMin - vMinHandoff) {
                aPkAccel = sqrt(jMax * vPeak + brkA * brkA / 2.0)
                T_H = (aPkAccel - brkA) / jMax
                T2 = 0.0
            } else {
                aPkAccel = aMaxAccel
                T_H = abs(aPkAccel - brkA) / jMax
                val vAfterH = T_H * (brkA + aPkAccel) / 2.0
                T2 =
                    max(
                        0.0,
                        (vPeak - vAfterH - aPkAccel * aPkAccel / (2.0 * jMax)) / aPkAccel,
                    )
            }

            val vDecelMin = aMaxDecel * aMaxDecel / jMax
            val aPkDecel: Double
            val T6: Double
            val T7: Double
            if (vPeak >= vDecelMin) {
                aPkDecel = aMaxDecel
                T6 = (vPeak - vDecelMin) / aPkDecel
                T7 = aPkDecel / jMax
            } else {
                aPkDecel = sqrt(vPeak * jMax)
                T6 = 0.0
                T7 = aPkDecel / jMax
            }

            var p = 0.0
            var v = 0.0

            var end = evalPhaseEndGen(p, v, brkA, aPkAccel, T_H)
            p = end[0]
            v = end[1]

            end = evalPhaseEnd(p, v, aPkAccel, 0, T2)
            p = end[0]
            v = end[1]

            val Tmid = (aPkAccel + aPkDecel) / jMax
            end = evalPhaseEndGen(p, v, aPkAccel, -aPkDecel, Tmid)
            p = end[0]
            v = end[1]

            end = evalPhaseEnd(p, v, -aPkDecel, 0, T6)
            p = end[0]
            v = end[1]

            end = evalPhaseEnd(p, v, -aPkDecel, +1, T7)
            return end[0]
        }
    }
}
