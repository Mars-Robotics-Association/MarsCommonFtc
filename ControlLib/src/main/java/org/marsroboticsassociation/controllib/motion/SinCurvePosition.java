package org.marsroboticsassociation.controllib.motion;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleUnaryOperator;

/**
 * Seven-segment sinusoidal (raised-cosine) position trajectory from p0 to pTarget, ending at rest.
 * Supports asymmetric acceleration limits: aMaxAccel for speeding up, aMaxDecel for braking.
 *
 * <p>Instead of piecewise-linear (constant-jerk) acceleration transitions used by {@link
 * SCurvePosition}, each transition is shaped with a raised-cosine acceleration curve, eliminating
 * jerk discontinuities at phase boundaries:
 *
 * <pre>  a(t') = aMax/2 &middot; (1 &plusmn; cos(&pi;t'/T))</pre>
 *
 * <p>The seven main phases are:
 *
 * <ol>
 *   <li>Sinusoidal onset: a ramps from 0 &rarr; aMaxAccel (or triangular peak)
 *   <li>Constant accel: a = aMaxAccel
 *   <li>Sinusoidal offset: a ramps from aMaxAccel &rarr; 0
 *   <li>Cruise: constant velocity vPeak
 *   <li>Sinusoidal onset: a ramps from 0 &rarr; &minus;aMaxDecel
 *   <li>Constant decel: a = &minus;aMaxDecel
 *   <li>Sinusoidal offset: a ramps from &minus;aMaxDecel &rarr; 0
 * </ol>
 *
 * <p>Optional prefix phases (applied before the 7-phase section):
 *
 * <ul>
 *   <li><b>a0 prefix</b>: if a0 &ne; 0, a quarter-cosine smoothly brings acceleration to 0.
 *   <li><b>Braking prefix</b>: if the post-prefix velocity is in the wrong direction, a symmetric
 *       sinusoidal braking profile brings velocity to 0 before the main phases.
 * </ul>
 *
 * <p>The {@code jMax} parameter has the same semantics as in {@link SCurvePosition}: the sinusoidal
 * transition duration equals {@code aMax/jMax}, matching the linear-jerk transition time. The peak
 * instantaneous jerk is {@code &pi;&middot;jMax/2}.
 *
 * <p>For a single uninterrupted move, the profile is smooth in position, velocity, and
 * acceleration, and it removes the acceleration corner points of a standard 7-segment polynomial
 * S-curve. When used through {@link PositionTrajectoryManager}, mid-motion replans also preserve
 * {@code p/v/a} because the next trajectory is seeded from the sampled current state. However,
 * replans do not preserve jerk continuity: rapid target changes may introduce sharp nodes in the
 * acceleration trace even though acceleration itself remains continuous.
 */
public class SinCurvePosition implements PositionTrajectory {

    // Integral coefficients for sinusoidal ramp distance contribution over one full phase:
    //   onset  phase: distance += ampl * T^2 * C_ONSET
    //   offset phase: distance += ampl * T^2 * C_OFFSET
    // Derived from integral of the raised-cosine velocity formula:
    //   C_ONSET  = 1/4 - 1/pi^2 = (pi^2 - 4) / (4*pi^2)
    //   C_OFFSET = 1/4 + 1/pi^2 = (pi^2 + 4) / (4*pi^2)
    static final double C_ONSET = (Math.PI * Math.PI - 4.0) / (4.0 * Math.PI * Math.PI);
    static final double C_OFFSET = (Math.PI * Math.PI + 4.0) / (4.0 * Math.PI * Math.PI);

    public final double p0, pTarget, v0, a0, vMax, aMaxAccel, aMaxDecel, jMax;

    // Phase durations (7-phase section only, after any prefix phases)
    public final double T1, T2, T3, T4, T5, T6, T7;

    // Peak velocity reached during cruise
    public final double vPeak;

    private final boolean trivial;
    private final double dir; // +1 if pTarget >= p0, else -1

    // a0 prefix: quarter-cosine from a0 to 0
    public final double tPrefix; // duration (0 if a0 == 0)
    private final double pAfterPrefix;
    private final double vAfterPrefix;

    // Braking prefix: symmetric sinusoidal decel to bring wrong-way velocity to 0
    public final double tBrake; // total duration (0 if not needed)
    private final double pBrake; // position at end of braking

    // True when a0 != 0 and braking is active: the a0 prefix and braking onset are merged
    // into a single general arc (a0 → brkAmpl). In this case tPrefix stores the arc duration,
    // brakeSubDur[0] = 0 (onset skipped), and the braking constant/offset are recalculated.
    private final boolean prefixMergedWithBrake;

    // Case B: braking offset + main T1 onset replaced by a single "handoff" arc
    // (brkAmpl→aPkAccel·dir).
    // Braking ends at v=0, a=brkAmpl (no offset phase). The handoff then transitions smoothly
    // to the main acceleration peak without returning to zero.
    public final boolean handoffCombined;
    public final double tHandoffStart; // absolute time at start of handoff
    public final double tHandoffEnd; // absolute time at end of handoff
    public final double aHandoffStart; // = brkAmpl (world frame)
    public final double aHandoffEnd; // = aPkAccel * dir (world frame)
    public final double pHandoffStart; // position at start of handoff (= pBrake)
    public final double pHandoffEnd; // position at end of handoff (= phaseStartP[0])
    public final double vHandoffEnd; // velocity at end of handoff (= phaseStartV[0])

    // Midpoint combined: when T4=0, T3-offset + T5-onset replaced by a single general arc
    // (accelAmpl → decelAmpl) stored in phase slot 2. Phase slots 3 and 4 have duration 0.
    public final boolean midpointCombined;
    private double aMidStart; // accelAmpl = aPkAccel * dir (world frame)
    private double aMidEnd; // decelAmpl = -aPkDecel * dir (world frame)

    // Brake sub-phase table (3 sub-phases: onset, constant, offset)
    // brakeSubT[0..3] = absolute start times (4 boundaries for 3 sub-phases)
    private final double[] brakeSubT = new double[4];
    private final double[] brakeSubP = new double[4];
    private final double[] brakeSubV = new double[4];
    private final double[] brakeSubAmpl = new double[3]; // signed accel amplitude per sub-phase
    private final int[] brakeSubSign = new int[3]; // -1 onset, 0 constant, +1 offset
    private final double[] brakeSubDur = new double[3]; // duration per sub-phase

    // Main 7-phase table: 8 breakpoints for 7 phases
    private final double[] phaseStartT = new double[8]; // absolute start times
    private final double[] phaseStartP = new double[8];
    private final double[] phaseStartV = new double[8];
    private final double[] phaseAmpl = new double[7]; // signed accel amplitude, world frame
    private final int[] phaseSign = new int[7]; // -1 onset, 0 constant, +1 offset

    public SinCurvePosition(
            double p0,
            double pTarget,
            double v0,
            double a0,
            double vMax,
            double aMaxAccel,
            double aMaxDecel,
            double jMax) {
        this.p0 = p0;
        this.pTarget = pTarget;
        this.v0 = v0;
        this.a0 = a0;
        this.vMax = Math.abs(vMax);
        this.aMaxAccel = Math.abs(aMaxAccel);
        this.aMaxDecel = Math.abs(aMaxDecel);
        this.jMax = Math.abs(jMax);

        double totalDistAbs = Math.abs(pTarget - p0);
        this.dir = (pTarget >= p0) ? 1.0 : -1.0;

        if (totalDistAbs < 1e-9) {
            trivial = true;
            T1 = T2 = T3 = T4 = T5 = T6 = T7 = 0;
            vPeak = 0;
            tPrefix = 0;
            pAfterPrefix = p0;
            vAfterPrefix = v0;
            tBrake = 0;
            pBrake = p0;
            prefixMergedWithBrake = false;
            handoffCombined = false;
            tHandoffStart = 0;
            tHandoffEnd = 0;
            aHandoffStart = 0;
            aHandoffEnd = 0;
            pHandoffStart = p0;
            pHandoffEnd = p0;
            vHandoffEnd = 0;
            midpointCombined = false;
            aMidStart = 0;
            aMidEnd = 0;
            phaseStartP[0] = p0;
            phaseStartV[0] = v0;
            for (int i = 1; i < 8; i++) {
                phaseStartT[i] = 0;
                phaseStartP[i] = p0;
                phaseStartV[i] = v0;
            }
            return;
        }
        trivial = false;

        // ---------------------------------------------------------------
        // a0 prefix: quarter-cosine from a0 to 0
        // (replaced by a general arc when Case A applies; see below)
        // ---------------------------------------------------------------
        double tPre, pPre, vPre;
        if (Math.abs(a0) < 1e-9) {
            tPre = 0;
            pPre = p0;
            vPre = v0;
        } else {
            tPre = Math.abs(a0) / this.jMax;
            vPre = v0 + a0 * (2.0 * tPre / Math.PI);
            pPre = p0 + v0 * tPre + a0 * (4.0 * tPre * tPre / (Math.PI * Math.PI));
        }

        // Determine whether braking is needed (after the a0 prefix)
        final boolean brakingNeeded = (vPre * dir < -1e-9);

        // ---------------------------------------------------------------
        // Case A: if a0 != 0 and braking is needed, merge the a0 prefix
        // and braking onset into a single general arc (a0 → brkAmpl).
        // ---------------------------------------------------------------
        boolean mergedPrefix = false;
        double brkAmplForCaseA = 0;

        if (Math.abs(a0) > 1e-9 && brakingNeeded) {
            // Determine brkAmpl (sign: opposite to vPre)
            double aBrake = Math.max(this.aMaxAccel, this.aMaxDecel);
            brkAmplForCaseA = -Math.signum(vPre) * aBrake; // signed world-frame amplitude

            // Combined arc: a0 → brkAmpl
            double T_A = Math.abs(brkAmplForCaseA - a0) / this.jMax;
            double delta_A = brkAmplForCaseA - a0;
            double vEnd_A = evalVgen(v0, a0, delta_A, T_A, T_A);
            double pEnd_A = evalPgen(p0, v0, a0, delta_A, T_A, T_A);

            // Only merge when the combined arc leaves velocity still in the wrong direction.
            // If the arc over-brakes (small initial speed), fall back to normal prefix.
            if (vEnd_A * dir < -1e-9) {
                tPre = T_A;
                vPre = vEnd_A;
                pPre = pEnd_A;
                mergedPrefix = true;
            }
        }

        this.tPrefix = tPre;
        this.pAfterPrefix = pPre;
        this.vAfterPrefix = vPre;
        this.prefixMergedWithBrake = mergedPrefix;

        // ---------------------------------------------------------------
        // Braking prefix: symmetric sinusoidal 3-sub-phase decel if vPre is wrong-way.
        // When Case A is active, the onset sub-phase (brakeSubDur[0]) is set to 0
        // (already absorbed into the prefix arc). tB2 is recalculated from vPre.
        // ---------------------------------------------------------------
        double tBrk, pBrk;
        if (vPre * dir < -1e-9) {
            double speed = Math.abs(vPre);
            double aBrake = Math.max(this.aMaxAccel, this.aMaxDecel);
            double vBrakeMin = aBrake * aBrake / this.jMax;
            double tB1, tB2, tB3, aBrkPeak;

            if (mergedPrefix) {
                // Case A: onset already handled; only need constant + offset to kill vPre.
                // brkAmplForCaseA is signed; aBrkPeak is the magnitude.
                aBrkPeak = Math.abs(brkAmplForCaseA);
                tB1 = 0; // onset absorbed into prefix
                tB3 = aBrkPeak / this.jMax;
                // Solve for tB2: vPre + brkAmpl * tB2 + brkAmpl/2 * tB3 = 0
                // => tB2 = -(vPre + brkAmpl/2 * tB3) / brkAmpl
                double brkAmplSigned = brkAmplForCaseA;
                double tB2_candidate = -(vPre + brkAmplSigned / 2.0 * tB3) / brkAmplSigned;
                if (tB2_candidate >= 0) {
                    tB2 = tB2_candidate;
                } else {
                    // Triangular: reduce brkAmpl so tB2=0 is sufficient
                    // Simplified: aBrkPeak_tri = sqrt(|vPre| * jMax)
                    aBrkPeak = Math.sqrt(speed * this.jMax);
                    tB3 = aBrkPeak / this.jMax;
                    tB2 = 0;
                    brkAmplSigned = -Math.signum(vPre) * aBrkPeak;
                    brkAmplForCaseA = brkAmplSigned; // update for brakeSubAmpl[0]
                }
                brakeSubDur[0] = tB1; // = 0
                brakeSubAmpl[0] = brkAmplSigned;
                brakeSubSign[0] = -1; // onset (zero duration, harmless)
                brakeSubDur[1] = tB2;
                brakeSubAmpl[1] = brkAmplSigned;
                brakeSubSign[1] = 0;
                brakeSubDur[2] = tB3;
                brakeSubAmpl[2] = brkAmplSigned;
                brakeSubSign[2] = +1;
            } else {
                // Normal braking (no Case A)
                if (speed >= vBrakeMin) {
                    tB1 = aBrake / this.jMax;
                    tB2 = (speed - vBrakeMin) / aBrake;
                    tB3 = tB1;
                    aBrkPeak = aBrake;
                } else {
                    aBrkPeak = Math.sqrt(speed * this.jMax);
                    tB1 = aBrkPeak / this.jMax;
                    tB2 = 0;
                    tB3 = tB1;
                }
                double brkAmpl = -Math.signum(vPre) * aBrkPeak;
                brakeSubDur[0] = tB1;
                brakeSubAmpl[0] = brkAmpl;
                brakeSubSign[0] = -1;
                brakeSubDur[1] = tB2;
                brakeSubAmpl[1] = brkAmpl;
                brakeSubSign[1] = 0;
                brakeSubDur[2] = tB3;
                brakeSubAmpl[2] = brkAmpl;
                brakeSubSign[2] = +1;
            }

            tBrk = brakeSubDur[0] + brakeSubDur[1] + brakeSubDur[2];
            brakeSubT[0] = tPre;
            brakeSubP[0] = pPre;
            brakeSubV[0] = vPre;
            for (int i = 0; i < 3; i++) {
                double[] end =
                        evalPhaseEnd(
                                brakeSubP[i],
                                brakeSubV[i],
                                brakeSubAmpl[i],
                                brakeSubSign[i],
                                brakeSubDur[i]);
                brakeSubT[i + 1] = brakeSubT[i] + brakeSubDur[i];
                brakeSubP[i + 1] = end[0];
                brakeSubV[i + 1] = end[1];
            }
            pBrk = brakeSubP[3];
        } else {
            tBrk = 0;
            pBrk = pPre;
        }
        // ---------------------------------------------------------------
        // Main 7-phase setup — Pass 1 (original braking structure with offset)
        // ---------------------------------------------------------------
        double distRemaining = Math.max(0.0, dir * (pTarget - pBrk));
        double v0adj = (tBrk > 0) ? 0.0 : Math.min(Math.max(vPre * dir, 0.0), this.vMax);

        double vPeakSolved, T4solved;
        {
            double dAtVMax =
                    sinHalfDist(v0adj, this.vMax, this.aMaxAccel, this.jMax)
                            + sinHalfDist(0, this.vMax, this.aMaxDecel, this.jMax);
            if (distRemaining >= dAtVMax) {
                vPeakSolved = this.vMax;
                T4solved = (this.vMax > 1e-9) ? (distRemaining - dAtVMax) / this.vMax : 0;
            } else {
                T4solved = 0;
                double dAtV0adj = sinHalfDist(0, v0adj, this.aMaxDecel, this.jMax);
                if (distRemaining <= dAtV0adj) {
                    vPeakSolved = v0adj;
                } else {
                    double lo = v0adj, hi = this.vMax;
                    for (int i = 0; i < 64; i++) {
                        double mid = (lo + hi) * 0.5;
                        double d =
                                sinFullDistCombinedMid(
                                        v0adj, mid, this.aMaxAccel, this.aMaxDecel, this.jMax);
                        if (d < distRemaining) lo = mid;
                        else hi = mid;
                    }
                    vPeakSolved = (lo + hi) * 0.5;
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
        boolean doHandoff = false;

        if (tBrk > 0 && vPeakSolved > 0) {
            double dvAccel_0 = vPeakSolved - v0adj;
            double aPkAccel_0 = 0;
            if (dvAccel_0 > 0) {
                double vAccelMin_0 = this.aMaxAccel * this.aMaxAccel / this.jMax;
                aPkAccel_0 =
                        (dvAccel_0 >= vAccelMin_0)
                                ? this.aMaxAccel
                                : Math.sqrt(dvAccel_0 * this.jMax);
            }
            double brkAmplSigned = brakeSubAmpl[0]; // e.g. negative for rightward braking
            double brkA = Math.abs(brkAmplSigned);
            double vMinHandoff = brkA * brkA / (2.0 * this.jMax);

            // Case B applies when brkAmpl and aPkAccel·dir have the same sign (both point
            // toward the new target direction), and vPeak is large enough for the handoff.
            if (aPkAccel_0 > 1e-9
                    && brkAmplSigned * aPkAccel_0 * dir > 0
                    && vPeakSolved >= vMinHandoff) {

                // Modify braking: remove offset, adjust tB2 so braking ends at v=0, a=brkAmpl.
                // v_after_onset = brakeSubV[1] (velocity after onset sub-phase)
                // tB2_new = -v_after_onset / brkAmplSigned  (always >= 0 since they have opposite
                // signs)
                double v_after_onset = brakeSubV[1];
                double tB2_new = Math.max(0.0, -v_after_onset / brkAmplSigned);
                brakeSubDur[1] = tB2_new;
                brakeSubDur[2] = 0; // offset removed

                // Re-forward-chain to get new pBrk
                brakeSubT[0] = tPre;
                brakeSubP[0] = pPre;
                brakeSubV[0] = vPre;
                for (int i = 0; i < 3; i++) {
                    double[] end =
                            evalPhaseEnd(
                                    brakeSubP[i],
                                    brakeSubV[i],
                                    brakeSubAmpl[i],
                                    brakeSubSign[i],
                                    brakeSubDur[i]);
                    brakeSubT[i + 1] = brakeSubT[i] + brakeSubDur[i];
                    brakeSubP[i + 1] = end[0];
                    brakeSubV[i + 1] = end[1];
                }
                tBrk = brakeSubDur[0] + brakeSubDur[1] + brakeSubDur[2];
                pBrk = brakeSubP[3];

                // Pass 2 bisect using sinHalfDistCombined
                double distRemaining2 = Math.max(0.0, dir * (pTarget - pBrk));
                double dAtVMaxCombined =
                        sinHalfDistCombined(v0adj, this.vMax, this.aMaxAccel, this.jMax, brkA)
                                + sinHalfDist(0, this.vMax, this.aMaxDecel, this.jMax);
                if (distRemaining2 >= dAtVMaxCombined) {
                    vPeakSolved = this.vMax;
                    T4solved =
                            (this.vMax > 1e-9) ? (distRemaining2 - dAtVMaxCombined) / this.vMax : 0;
                } else {
                    T4solved = 0;
                    double dAtV0adjCombined = sinHalfDist(0, v0adj, this.aMaxDecel, this.jMax);
                    if (distRemaining2 <= dAtV0adjCombined) {
                        vPeakSolved = v0adj;
                    } else {
                        double lo = v0adj, hi = this.vMax;
                        for (int i = 0; i < 64; i++) {
                            double mid = (lo + hi) * 0.5;
                            double d =
                                    sinFullDistHandoffAndMid(
                                            v0adj,
                                            mid,
                                            this.aMaxAccel,
                                            this.aMaxDecel,
                                            this.jMax,
                                            brkA);
                            if (d < distRemaining2) lo = mid;
                            else hi = mid;
                        }
                        vPeakSolved = (lo + hi) * 0.5;
                    }
                }

                // Confirm handoff is still worthwhile with updated vPeak
                if (vPeakSolved - v0adj > 1e-9) {
                    doHandoff = true;
                }
            }
        }

        this.tBrake = tBrk;
        this.pBrake = pBrk;
        this.vPeak = vPeakSolved;
        this.T4 = Math.max(0.0, T4solved);

        // ---------------------------------------------------------------
        // Accel phase durations
        // ---------------------------------------------------------------
        double dvAccel = vPeakSolved - v0adj;
        double vAccelMin = this.aMaxAccel * this.aMaxAccel / this.jMax;
        double t1, t2, t3, aPkAccel;
        if (dvAccel <= 0) {
            t1 = 0;
            t2 = 0;
            t3 = 0;
            aPkAccel = 0;
        } else if (dvAccel >= vAccelMin) {
            aPkAccel = this.aMaxAccel;
            t1 = this.aMaxAccel / this.jMax;
            t2 = (dvAccel - vAccelMin) / this.aMaxAccel;
            t3 = t1;
        } else {
            aPkAccel = Math.sqrt(dvAccel * this.jMax);
            t1 = aPkAccel / this.jMax;
            t2 = 0;
            t3 = t1;
        }

        // Decel phase durations
        double vDecelMin = this.aMaxDecel * this.aMaxDecel / this.jMax;
        double t5, t6, t7, aPkDecel;
        if (vPeakSolved <= 0) {
            t5 = 0;
            t6 = 0;
            t7 = 0;
            aPkDecel = 0;
        } else if (vPeakSolved >= vDecelMin) {
            aPkDecel = this.aMaxDecel;
            t5 = this.aMaxDecel / this.jMax;
            t6 = (vPeakSolved - vDecelMin) / this.aMaxDecel;
            t7 = t5;
        } else {
            aPkDecel = Math.sqrt(vPeakSolved * this.jMax);
            t5 = aPkDecel / this.jMax;
            t6 = 0;
            t7 = t5;
        }

        // When Case B is active, T1 is absorbed into the handoff arc.
        this.T1 = (doHandoff) ? 0 : Math.max(0, t1);
        if (doHandoff && aPkAccel > 1e-9) {
            // T2 must be recomputed: T1=0 so velocity entering T2 is vHEnd (end of handoff arc),
            // not v0adj. Standard formula (dvAccel - vAccelMin)/aPkAccel assumes onset starts at
            // v=0; here the handoff arc already gained vHEnd = T_H*(brkA+aPkAccel)/2.
            double brkA = Math.abs(brakeSubAmpl[0]);
            double T_H = Math.abs(aPkAccel - brkA) / this.jMax;
            double vHEnd_calc = T_H * (brkA + aPkAccel) / 2.0;
            t2 =
                    Math.max(
                            0.0,
                            (vPeakSolved - vHEnd_calc - aPkAccel * aPkAccel / (2.0 * this.jMax))
                                    / aPkAccel);
        }
        // Midpoint combined: when T4=0 and both accel/decel peaks are nonzero,
        // replace T3-offset + T5-onset with a single general arc.
        boolean doMidpoint = (T4solved == 0) && (aPkAccel > 1e-9) && (aPkDecel > 1e-9);
        if (doMidpoint) {
            t3 = (aPkAccel + aPkDecel) / this.jMax; // combined arc stored in T3 slot
            t5 = 0; // T5 absorbed into combined arc
        }

        this.T2 = Math.max(0, t2);
        this.T3 = Math.max(0, t3);
        this.T5 = Math.max(0, t5);
        this.T6 = Math.max(0, t6);
        this.T7 = Math.max(0, t7);

        // ---------------------------------------------------------------
        // Case B handoff arc: compute endpoint state
        // ---------------------------------------------------------------
        double tHStart = 0, tHEnd = 0, aHStart = 0, aHEnd = 0;
        double pHStart = pBrk, pHEnd = pBrk, vHEnd = 0;

        if (doHandoff) {
            aHStart = brakeSubAmpl[0]; // brkAmpl (world frame, e.g. negative)
            aHEnd = aPkAccel * dir; // aPkAccel·dir (world frame)
            tHStart = tPre + tBrk;
            double T_h = Math.abs(aHEnd - aHStart) / this.jMax;
            tHEnd = tHStart + T_h;
            pHStart = pBrk;
            double[] handoffEnd = evalPhaseEndGen(pBrk, 0.0, aHStart, aHEnd, T_h);
            pHEnd = handoffEnd[0];
            vHEnd = handoffEnd[1];
        }

        this.handoffCombined = doHandoff;
        this.tHandoffStart = tHStart;
        this.tHandoffEnd = tHEnd;
        this.aHandoffStart = aHStart;
        this.aHandoffEnd = aHEnd;
        this.pHandoffStart = pHStart;
        this.pHandoffEnd = pHEnd;
        this.vHandoffEnd = vHEnd;

        this.midpointCombined = doMidpoint;
        if (doMidpoint) {
            this.aMidStart = aPkAccel * dir; // accelAmpl (world frame)
            this.aMidEnd = -aPkDecel * dir; // decelAmpl (world frame)
        } else {
            this.aMidStart = 0;
            this.aMidEnd = 0;
        }

        // ---------------------------------------------------------------
        // Build phase table
        //
        // Phase signs and amplitudes in world frame:
        //   0 (T1): onset,    ampl = +aPkAccel*dir  (a: 0 -> aPkAccel*dir)  [T1=0 when Case B]
        //   1 (T2): constant, ampl = +aPkAccel*dir  (a = aPkAccel*dir)
        //   2 (T3): offset,   ampl = +aPkAccel*dir  (a: aPkAccel*dir -> 0)  [midpoint arc when combined]
        //   3 (T4): constant, ampl = 0              (cruise)
        //   4 (T5): onset,    ampl = -aPkDecel*dir  (a: 0 -> -aPkDecel*dir)
        //   5 (T6): constant, ampl = -aPkDecel*dir  (a = -aPkDecel*dir)
        //   6 (T7): offset,   ampl = -aPkDecel*dir  (a: -aPkDecel*dir -> 0)
        // ---------------------------------------------------------------
        double accelAmpl = aPkAccel * dir;
        double decelAmpl = -aPkDecel * dir;

        phaseSign[0] = -1;
        phaseAmpl[0] = accelAmpl;
        phaseSign[1] = 0;
        phaseAmpl[1] = accelAmpl;
        phaseSign[2] = +1;
        phaseAmpl[2] = accelAmpl;
        phaseSign[3] = 0;
        phaseAmpl[3] = 0;
        phaseSign[4] = -1;
        phaseAmpl[4] = decelAmpl;
        phaseSign[5] = 0;
        phaseAmpl[5] = decelAmpl;
        phaseSign[6] = +1;
        phaseAmpl[6] = decelAmpl;

        double[] durations = {this.T1, this.T2, this.T3, this.T4, this.T5, this.T6, this.T7};
        phaseStartT[0] = tPre + tBrk + (doHandoff ? (tHEnd - tHStart) : 0);
        phaseStartP[0] = doHandoff ? pHEnd : pBrk;
        phaseStartV[0] = doHandoff ? vHEnd : (v0adj * dir);

        for (int i = 0; i < 7; i++) {
            double[] end;
            if (midpointCombined && i == 2) {
                end =
                        evalPhaseEndGen(
                                phaseStartP[i], phaseStartV[i], aMidStart, aMidEnd, durations[i]);
            } else {
                end =
                        evalPhaseEnd(
                                phaseStartP[i],
                                phaseStartV[i],
                                phaseAmpl[i],
                                phaseSign[i],
                                durations[i]);
            }
            phaseStartT[i + 1] = phaseStartT[i] + durations[i];
            phaseStartP[i + 1] = end[0];
            phaseStartV[i + 1] = end[1];
        }
    }

    // ---------------------------------------------------------------
    // PositionTrajectory interface
    // ---------------------------------------------------------------

    @Override
    public double getPosition(double t) {
        if (trivial) return p0;
        if (t <= 0) return p0;
        double tf = getTotalTime();
        if (t >= tf) return pTarget;

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                double delta = brakeSubAmpl[0] - a0;
                return evalPgen(p0, v0, a0, delta, t, tPrefix);
            }
            // p(t) = p0 + v0*t + a0*(4*tPrefix^2/pi^2)*(1 - cos(pi*t/(2*tPrefix)))
            return p0
                    + v0 * t
                    + a0
                            * (4.0 * tPrefix * tPrefix / (Math.PI * Math.PI))
                            * (1.0 - Math.cos(Math.PI * t / (2.0 * tPrefix)));
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            return evalP(
                    brakeSubP[bph],
                    brakeSubV[bph],
                    brakeSubAmpl[bph],
                    brakeSubSign[bph],
                    dt,
                    brakeSubDur[bph]);
        }

        if (handoffCombined && t < tHandoffEnd) {
            double dt = t - tHandoffStart;
            double T_h = tHandoffEnd - tHandoffStart;
            return evalPgen(
                    pHandoffStart, 0.0, aHandoffStart, aHandoffEnd - aHandoffStart, dt, T_h);
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
        if (midpointCombined && ph == 2) {
            return evalPgen(phaseStartP[2], phaseStartV[2], aMidStart, aMidEnd - aMidStart, dt, T);
        }
        return evalP(phaseStartP[ph], phaseStartV[ph], phaseAmpl[ph], phaseSign[ph], dt, T);
    }

    @Override
    public double getVelocity(double t) {
        if (trivial) return v0;
        if (t <= 0) return v0;
        double tf = getTotalTime();
        if (t >= tf) return 0;

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                double delta = brakeSubAmpl[0] - a0;
                return evalVgen(v0, a0, delta, t, tPrefix);
            }
            // v(t) = v0 + a0*(2*tPrefix/pi)*sin(pi*t/(2*tPrefix))
            return v0 + a0 * (2.0 * tPrefix / Math.PI) * Math.sin(Math.PI * t / (2.0 * tPrefix));
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            return evalV(
                    brakeSubV[bph], brakeSubAmpl[bph], brakeSubSign[bph], dt, brakeSubDur[bph]);
        }

        if (handoffCombined && t < tHandoffEnd) {
            double dt = t - tHandoffStart;
            double T_h = tHandoffEnd - tHandoffStart;
            return evalVgen(0.0, aHandoffStart, aHandoffEnd - aHandoffStart, dt, T_h);
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
        if (midpointCombined && ph == 2) {
            return evalVgen(phaseStartV[2], aMidStart, aMidEnd - aMidStart, dt, T);
        }
        return evalV(phaseStartV[ph], phaseAmpl[ph], phaseSign[ph], dt, T);
    }

    @Override
    public double getAcceleration(double t) {
        if (trivial) return a0;
        if (t <= 0) return a0;
        double tf = getTotalTime();
        if (t >= tf) return 0;

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                double aStart = a0;
                double delta = brakeSubAmpl[0] - a0;
                return evalAgen(aStart, delta, t, tPrefix);
            }
            // a(t) = a0 * cos(pi*t / (2*tPrefix))
            return a0 * Math.cos(Math.PI * t / (2.0 * tPrefix));
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            return evalA(brakeSubAmpl[bph], brakeSubSign[bph], dt, brakeSubDur[bph]);
        }

        if (handoffCombined && t < tHandoffEnd) {
            double dt = t - tHandoffStart;
            double T_h = tHandoffEnd - tHandoffStart;
            return evalAgen(aHandoffStart, aHandoffEnd - aHandoffStart, dt, T_h);
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
        if (midpointCombined && ph == 2) {
            return evalAgen(aMidStart, aMidEnd - aMidStart, dt, T);
        }
        return evalA(phaseAmpl[ph], phaseSign[ph], dt, T);
    }

    @Override
    public double getTotalTime() {
        return trivial ? 0 : phaseStartT[7];
    }

    @Override
    public boolean isZeroJerk(double t) {
        if (trivial) return true;
        if (t <= 0 || t >= getTotalTime()) return true;
        if (tPrefix > 0 && t < tPrefix) return false; // active sinusoidal prefix
        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            return brakeSubSign[findBrakePhase(t)] == 0;
        }
        if (handoffCombined && t < tHandoffEnd) return false; // active handoff arc
        if (midpointCombined && findPhase(t) == 2) return false;
        return phaseSign[findPhase(t)] == 0;
    }

    public double getJerk(double t) {
        if (trivial) return 0;
        if (t <= 0 || t >= getTotalTime()) return 0;

        if (tPrefix > 0 && t < tPrefix) {
            if (prefixMergedWithBrake) {
                return evalJgen(brakeSubAmpl[0] - a0, t, tPrefix);
            }
            return -a0 * Math.PI * Math.sin(Math.PI * t / (2.0 * tPrefix)) / (2.0 * tPrefix);
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            return evalJ(
                    brakeSubAmpl[bph], brakeSubSign[bph], t - brakeSubT[bph], brakeSubDur[bph]);
        }

        if (handoffCombined && t < tHandoffEnd) {
            double T_h = tHandoffEnd - tHandoffStart;
            return evalJgen(aHandoffEnd - aHandoffStart, t - tHandoffStart, T_h);
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
        if (midpointCombined && ph == 2) {
            return evalJgen(aMidEnd - aMidStart, dt, T);
        }
        return evalJ(phaseAmpl[ph], phaseSign[ph], dt, T);
    }

    public List<TrajectoryCurveSegment> positionSegments() {
        return buildCurveSegments(this::getPosition, this::getVelocity);
    }

    public List<TrajectoryCurveSegment> velocitySegments() {
        return buildCurveSegments(this::getVelocity, this::getAcceleration);
    }

    public List<TrajectoryCurveSegment> accelerationSegments() {
        return buildCurveSegments(this::getAcceleration, this::getJerk);
    }

    private List<TrajectoryCurveSegment> buildCurveSegments(
            DoubleUnaryOperator valueFunction, DoubleUnaryOperator slopeFunction) {
        List<TrajectoryCurveSegment> segments = new ArrayList<>();
        if (trivial) return segments;

        addCurveSegment(segments, 0.0, tPrefix, valueFunction, slopeFunction);
        for (int i = 0; i < 3; i++) {
            addCurveSegment(
                    segments, brakeSubT[i], brakeSubT[i + 1], valueFunction, slopeFunction);
        }
        if (handoffCombined) {
            addCurveSegment(segments, tHandoffStart, tHandoffEnd, valueFunction, slopeFunction);
        }
        for (int i = 0; i < 7; i++) {
            addCurveSegment(
                    segments, phaseStartT[i], phaseStartT[i + 1], valueFunction, slopeFunction);
        }
        return segments;
    }

    private static void addCurveSegment(
            List<TrajectoryCurveSegment> segments,
            double startTime,
            double endTime,
            DoubleUnaryOperator valueFunction,
            DoubleUnaryOperator slopeFunction) {
        if (endTime <= startTime) return;
        segments.add(new FunctionalCurveSegment(startTime, endTime, valueFunction, slopeFunction));
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

    private static double evalP(
            double pStar, double vStar, double ampl, int sign, double dt, double T) {
        if (sign == 0) {
            return pStar + vStar * dt + 0.5 * ampl * dt * dt;
        }
        double cosVal = (T > 1e-15) ? Math.cos(Math.PI * dt / T) : 1.0;
        return pStar
                + vStar * dt
                + (ampl / 2.0)
                        * (dt * dt / 2.0 + sign * T * T * (1.0 - cosVal) / (Math.PI * Math.PI));
    }

    private static double evalV(double vStar, double ampl, int sign, double dt, double T) {
        if (sign == 0) {
            return vStar + ampl * dt;
        }
        double sinVal = (T > 1e-15) ? Math.sin(Math.PI * dt / T) : 0.0;
        return vStar + (ampl / 2.0) * (dt + sign * T * sinVal / Math.PI);
    }

    private static double evalA(double ampl, int sign, double dt, double T) {
        if (sign == 0) {
            return ampl;
        }
        double cosVal = (T > 1e-15) ? Math.cos(Math.PI * dt / T) : 1.0;
        return (ampl / 2.0) * (1.0 + sign * cosVal);
    }

    private static double evalJ(double ampl, int sign, double dt, double T) {
        if (sign == 0 || T <= 1e-15) {
            return 0.0;
        }
        return -(ampl * sign * Math.PI / (2.0 * T)) * Math.sin(Math.PI * dt / T);
    }

    /** Returns [p, v] at the end of a phase. Used for forward-chaining the phase table. */
    private static double[] evalPhaseEnd(
            double pStar, double vStar, double ampl, int sign, double T) {
        return new double[] {evalP(pStar, vStar, ampl, sign, T, T), evalV(vStar, ampl, sign, T, T)};
    }

    /**
     * General sinusoidal arc: a goes from aStart to (aStart+delta) over duration T. Reduces to
     * evalA when aStart=0 (onset, sign=-1) or delta=-aStart (offset, sign=+1).
     */
    private static double evalAgen(double aStart, double delta, double dt, double T) {
        double cosVal = (T > 1e-15) ? Math.cos(Math.PI * dt / T) : 1.0;
        return aStart + (delta / 2.0) * (1.0 - cosVal);
    }

    private static double evalJgen(double delta, double dt, double T) {
        if (T <= 1e-15) {
            return 0.0;
        }
        return (delta * Math.PI / (2.0 * T)) * Math.sin(Math.PI * dt / T);
    }

    private static double evalVgen(double vStar, double aStart, double delta, double dt, double T) {
        double sinVal = (T > 1e-15) ? Math.sin(Math.PI * dt / T) : 0.0;
        return vStar + aStart * dt + (delta / 2.0) * (dt - T * sinVal / Math.PI);
    }

    private static double evalPgen(
            double pStar, double vStar, double aStart, double delta, double dt, double T) {
        double cosVal = (T > 1e-15) ? Math.cos(Math.PI * dt / T) : 1.0;
        // p(t') = pStar + vStar*t' + (aStart + delta/2)*t'^2/2 - delta/2*T^2*(1-cos(pi*t'/T))/pi^2
        // Derived by integrating v(t') = vStar + aStart*t' + delta/2*(t' - T*sin(pi*t'/T)/pi)
        return pStar
                + vStar * dt
                + (aStart + delta / 2.0) * dt * dt / 2.0
                - (delta / 2.0) * T * T * (1.0 - cosVal) / (Math.PI * Math.PI);
    }

    /** Returns [p, v] at the end of a general arc. */
    private static double[] evalPhaseEndGen(
            double pStar, double vStar, double aStart, double aEnd, double T) {
        double delta = aEnd - aStart;
        return new double[] {
            evalPgen(pStar, vStar, aStart, delta, T, T), evalVgen(vStar, aStart, delta, T, T)
        };
    }

    // ---------------------------------------------------------------
    // Sinusoidal distance helper
    // ---------------------------------------------------------------

    /**
     * Distance covered during a sinusoidal acceleration from v0 (a=0) to vPeak (a=0). Uses
     * raised-cosine ramp shape (trapezoidal or triangular depending on Δv vs aMax²/jMax). Returns 0
     * if vPeak &le; v0.
     */
    static double sinHalfDist(double v0, double vPeak, double aMax, double jMax) {
        double dv = vPeak - v0;
        if (dv <= 0) return 0;
        double vMin = aMax * aMax / jMax; // triangular threshold
        if (dv >= vMin) {
            // Trapezoidal: onset (0->aMax), constant (aMax), offset (aMax->0)
            double T1 = aMax / jMax;
            double T3 = T1;
            double T2 = (dv - vMin) / aMax;
            double v1 = v0 + aMax * T1 / 2.0;
            double v2 = v1 + aMax * T2;
            double d1 = v0 * T1 + aMax * T1 * T1 * C_ONSET;
            double d2 = v1 * T2 + 0.5 * aMax * T2 * T2;
            double d3 = v2 * T3 + aMax * T3 * T3 * C_OFFSET;
            return d1 + d2 + d3;
        } else {
            // Triangular: onset (0->aPeak), offset (aPeak->0)
            double aPk = Math.sqrt(dv * jMax);
            double T1 = aPk / jMax;
            double T3 = T1;
            double v1 = v0 + aPk * T1 / 2.0;
            double d1 = v0 * T1 + aPk * T1 * T1 * C_ONSET;
            double d3 = v1 * T3 + aPk * T3 * T3 * C_OFFSET;
            return d1 + d3;
        }
    }

    /**
     * Like {@link #sinHalfDist} but for the acceleration half when Case B (braking handoff) is
     * active. Replaces the normal onset (0 → aPkAccel) with a combined arc (brkA → aPkAccel),
     * starting from v = 0 (braking already brought velocity to zero).
     *
     * @param v0adj unused (always 0 when Case B applies; kept for call-site symmetry)
     * @param vPeak target peak velocity
     * @param aMaxAccel maximum acceleration magnitude
     * @param jMax jerk limit
     * @param brkA braking amplitude magnitude (|brkAmpl|)
     */
    static double sinHalfDistCombined(
            double v0adj, double vPeak, double aMaxAccel, double jMax, double brkA) {
        if (vPeak <= 0) return 0;
        double vMin = aMaxAccel * aMaxAccel / jMax;
        double vMinHandoff = brkA * brkA / (2.0 * jMax);

        // Triangular case: aPkAccel < aMaxAccel AND aMaxAccel >= brkA (ascending handoff)
        // Threshold: vPeak <= vMin - vMinHandoff  (derived from aPkAccel reaching aMaxAccel at that
        // vPeak)
        double aPkAccel, T_H, T2;
        if (aMaxAccel >= brkA && vPeak <= vMin - vMinHandoff) {
            // Triangular: aPkAccel = sqrt(jMax * vPeak + brkA²/2)
            aPkAccel = Math.sqrt(jMax * vPeak + brkA * brkA / 2.0);
            T_H = (aPkAccel - brkA) / jMax; // ascending arc, aPkAccel >= brkA
            T2 = 0;
        } else {
            // Trapezoidal: aPkAccel = aMaxAccel (ascending or descending handoff)
            aPkAccel = aMaxAccel;
            T_H = Math.abs(aPkAccel - brkA) / jMax;
            double v_after_H = T_H * (brkA + aPkAccel) / 2.0;
            T2 = Math.max(0.0, (vPeak - v_after_H - aPkAccel * aPkAccel / (2.0 * jMax)) / aPkAccel);
        }
        double T3 = aPkAccel / jMax;
        double v_after_H = T_H * (brkA + aPkAccel) / 2.0;

        // Distance during handoff arc (a: brkA → aPkAccel), starting from v=0, p=0
        double delta_H = aPkAccel - brkA;
        double d_H = evalPgen(0, 0, brkA, delta_H, T_H, T_H);

        // Distance during constant phase (a = aPkAccel)
        double d_T2 = v_after_H * T2 + 0.5 * aPkAccel * T2 * T2;

        // Distance during offset phase (a: aPkAccel → 0)
        double v_start_T3 = v_after_H + aPkAccel * T2;
        double d_T3 = v_start_T3 * T3 + aPkAccel * T3 * T3 * C_OFFSET;

        return d_H + d_T2 + d_T3;
    }

    /** Full no-cruise distance when T3+T5 are replaced by a single midpoint arc. */
    static double sinFullDistCombinedMid(
            double v0adj, double vPeak, double aMaxAccel, double aMaxDecel, double jMax) {
        if (vPeak <= v0adj) {
            return sinHalfDist(0, vPeak, aMaxDecel, jMax);
        }

        double dvAccel = vPeak - v0adj;
        double vAccelMin = aMaxAccel * aMaxAccel / jMax;
        double aPkAccel;
        double T1;
        double T2;
        if (dvAccel >= vAccelMin) {
            aPkAccel = aMaxAccel;
            T1 = aPkAccel / jMax;
            T2 = (dvAccel - vAccelMin) / aPkAccel;
        } else {
            aPkAccel = Math.sqrt(dvAccel * jMax);
            T1 = aPkAccel / jMax;
            T2 = 0;
        }

        double vDecelMin = aMaxDecel * aMaxDecel / jMax;
        double aPkDecel;
        double T6;
        double T7;
        if (vPeak >= vDecelMin) {
            aPkDecel = aMaxDecel;
            T6 = (vPeak - vDecelMin) / aPkDecel;
            T7 = aPkDecel / jMax;
        } else {
            aPkDecel = Math.sqrt(vPeak * jMax);
            T6 = 0;
            T7 = aPkDecel / jMax;
        }

        double p = 0.0;
        double v = v0adj;

        double[] end = evalPhaseEnd(p, v, aPkAccel, -1, T1);
        p = end[0];
        v = end[1];

        end = evalPhaseEnd(p, v, aPkAccel, 0, T2);
        p = end[0];
        v = end[1];

        double Tmid = (aPkAccel + aPkDecel) / jMax;
        end = evalPhaseEndGen(p, v, aPkAccel, -aPkDecel, Tmid);
        p = end[0];
        v = end[1];

        end = evalPhaseEnd(p, v, -aPkDecel, 0, T6);
        p = end[0];
        v = end[1];

        end = evalPhaseEnd(p, v, -aPkDecel, +1, T7);
        return end[0];
    }

    /** Full no-cruise distance when both the braking handoff and midpoint arc are combined. */
    static double sinFullDistHandoffAndMid(
            double v0adj, double vPeak, double aMaxAccel, double aMaxDecel, double jMax, double brkA) {
        if (vPeak <= 0) {
            return 0;
        }

        double vMin = aMaxAccel * aMaxAccel / jMax;
        double vMinHandoff = brkA * brkA / (2.0 * jMax);
        double aPkAccel;
        double T_H;
        double T2;
        if (aMaxAccel >= brkA && vPeak <= vMin - vMinHandoff) {
            aPkAccel = Math.sqrt(jMax * vPeak + brkA * brkA / 2.0);
            T_H = (aPkAccel - brkA) / jMax;
            T2 = 0;
        } else {
            aPkAccel = aMaxAccel;
            T_H = Math.abs(aPkAccel - brkA) / jMax;
            double vAfterH = T_H * (brkA + aPkAccel) / 2.0;
            T2 =
                    Math.max(
                            0.0,
                            (vPeak - vAfterH - aPkAccel * aPkAccel / (2.0 * jMax)) / aPkAccel);
        }

        double vDecelMin = aMaxDecel * aMaxDecel / jMax;
        double aPkDecel;
        double T6;
        double T7;
        if (vPeak >= vDecelMin) {
            aPkDecel = aMaxDecel;
            T6 = (vPeak - vDecelMin) / aPkDecel;
            T7 = aPkDecel / jMax;
        } else {
            aPkDecel = Math.sqrt(vPeak * jMax);
            T6 = 0;
            T7 = aPkDecel / jMax;
        }

        double p = 0.0;
        double v = 0.0;

        double[] end = evalPhaseEndGen(p, v, brkA, aPkAccel, T_H);
        p = end[0];
        v = end[1];

        end = evalPhaseEnd(p, v, aPkAccel, 0, T2);
        p = end[0];
        v = end[1];

        double Tmid = (aPkAccel + aPkDecel) / jMax;
        end = evalPhaseEndGen(p, v, aPkAccel, -aPkDecel, Tmid);
        p = end[0];
        v = end[1];

        end = evalPhaseEnd(p, v, -aPkDecel, 0, T6);
        p = end[0];
        v = end[1];

        end = evalPhaseEnd(p, v, -aPkDecel, +1, T7);
        return end[0];
    }

    // ---------------------------------------------------------------
    // Phase lookup
    // ---------------------------------------------------------------

    /** Returns brake sub-phase index in [0,2] for time t during the braking prefix. */
    private int findBrakePhase(double t) {
        if (t >= brakeSubT[2]) return 2;
        if (t >= brakeSubT[1]) return 1;
        return 0;
    }

    /** Returns phase index in [0,6] for time t, assuming t >= phaseStartT[0]. */
    private int findPhase(double t) {
        for (int i = 6; i >= 1; i--) {
            if (t >= phaseStartT[i]) return i;
        }
        return 0;
    }
}
