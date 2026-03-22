package org.marsroboticsassociation.controllib.motion;

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
        //   a(t) = a0 * cos(pi*t / (2*tPre))
        //   v(t) = v0 + a0 * (2*tPre/pi) * sin(pi*t / (2*tPre))
        //   p(t) = p0 + v0*t + a0 * (4*tPre^2/pi^2) * (1 - cos(pi*t / (2*tPre)))
        // ---------------------------------------------------------------
        double tPre, pPre, vPre;
        if (Math.abs(a0) < 1e-9) {
            tPre = 0;
            pPre = p0;
            vPre = v0;
        } else {
            tPre = Math.abs(a0) / this.jMax;
            // sin(pi/2) = 1, (1 - cos(pi/2)) = 1
            vPre = v0 + a0 * (2.0 * tPre / Math.PI);
            pPre = p0 + v0 * tPre + a0 * (4.0 * tPre * tPre / (Math.PI * Math.PI));
        }
        this.tPrefix = tPre;
        this.pAfterPrefix = pPre;
        this.vAfterPrefix = vPre;

        // ---------------------------------------------------------------
        // Braking prefix: symmetric sinusoidal 3-sub-phase decel if vPre is wrong-way.
        // Sub-phases: onset (0 -> aBrake), constant (aBrake), offset (aBrake -> 0).
        // Ends at v=0, a=0. Main phases start cleanly from rest.
        // ---------------------------------------------------------------
        double tBrk, pBrk;
        if (vPre * dir < -1e-9) {
            double speed = Math.abs(vPre);
            double aBrake = Math.max(this.aMaxAccel, this.aMaxDecel);
            double vBrakeMin = aBrake * aBrake / this.jMax;
            double tB1, tB2, tB3, aBrkPeak;
            if (speed >= vBrakeMin) {
                // Trapezoidal: ramp up to aBrake, hold, ramp back down to 0
                tB1 = aBrake / this.jMax;
                tB2 = (speed - vBrakeMin) / aBrake;
                tB3 = tB1;
                aBrkPeak = aBrake;
            } else {
                // Triangular: ramp up to aPeak (< aBrake), ramp back down
                aBrkPeak = Math.sqrt(speed * this.jMax);
                tB1 = aBrkPeak / this.jMax;
                tB2 = 0;
                tB3 = tB1;
            }
            tBrk = tB1 + tB2 + tB3;
            // Braking accelerates opposite to vPre to kill it
            double brkAmpl = -Math.signum(vPre) * aBrkPeak;
            brakeSubDur[0] = tB1;
            brakeSubAmpl[0] = brkAmpl;
            brakeSubSign[0] = -1; // onset
            brakeSubDur[1] = tB2;
            brakeSubAmpl[1] = brkAmpl;
            brakeSubSign[1] = 0; // constant
            brakeSubDur[2] = tB3;
            brakeSubAmpl[2] = brkAmpl;
            brakeSubSign[2] = +1; // offset

            // Forward-chain brake sub-phases from (pPre, vPre) at absolute time tPrefix
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
        this.tBrake = tBrk;
        this.pBrake = pBrk;

        // ---------------------------------------------------------------
        // Main 7-phase setup
        // ---------------------------------------------------------------
        double distRemaining = Math.max(0.0, dir * (pTarget - pBrk));
        double v0adj = (tBrk > 0) ? 0.0 : Math.min(Math.max(vPre * dir, 0.0), this.vMax);

        // Bisect for vPeak
        double dAtVMax =
                sinHalfDist(v0adj, this.vMax, this.aMaxAccel, this.jMax)
                        + sinHalfDist(0, this.vMax, this.aMaxDecel, this.jMax);
        double vPeakSolved, T4solved;
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
                            sinHalfDist(v0adj, mid, this.aMaxAccel, this.jMax)
                                    + sinHalfDist(0, mid, this.aMaxDecel, this.jMax);
                    if (d < distRemaining) lo = mid;
                    else hi = mid;
                }
                vPeakSolved = (lo + hi) * 0.5;
            }
        }
        this.vPeak = vPeakSolved;
        this.T4 = Math.max(0.0, T4solved);

        // Accel phase durations
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

        this.T1 = Math.max(0, t1);
        this.T2 = Math.max(0, t2);
        this.T3 = Math.max(0, t3);
        this.T5 = Math.max(0, t5);
        this.T6 = Math.max(0, t6);
        this.T7 = Math.max(0, t7);

        // ---------------------------------------------------------------
        // Build phase table by forward-chaining from (pBrk, v0adj*dir)
        //
        // Phase signs and amplitudes in world frame:
        //   0 (T1): onset,    ampl = +aPkAccel*dir  (a: 0 -> aPkAccel*dir)
        //   1 (T2): constant, ampl = +aPkAccel*dir  (a = aPkAccel*dir)
        //   2 (T3): offset,   ampl = +aPkAccel*dir  (a: aPkAccel*dir -> 0)
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

        double[] durations = {T1, T2, T3, T4, T5, T6, T7};
        phaseStartT[0] = tPre + tBrk;
        phaseStartP[0] = pBrk;
        phaseStartV[0] = v0adj * dir;

        for (int i = 0; i < 7; i++) {
            double[] end =
                    evalPhaseEnd(
                            phaseStartP[i],
                            phaseStartV[i],
                            phaseAmpl[i],
                            phaseSign[i],
                            durations[i]);
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

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
        return evalP(phaseStartP[ph], phaseStartV[ph], phaseAmpl[ph], phaseSign[ph], dt, T);
    }

    @Override
    public double getVelocity(double t) {
        if (trivial) return v0;
        if (t <= 0) return v0;
        double tf = getTotalTime();
        if (t >= tf) return 0;

        if (tPrefix > 0 && t < tPrefix) {
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

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
        return evalV(phaseStartV[ph], phaseAmpl[ph], phaseSign[ph], dt, T);
    }

    @Override
    public double getAcceleration(double t) {
        if (trivial) return a0;
        if (t <= 0) return a0;
        double tf = getTotalTime();
        if (t >= tf) return 0;

        if (tPrefix > 0 && t < tPrefix) {
            // a(t) = a0 * cos(pi*t / (2*tPrefix))
            return a0 * Math.cos(Math.PI * t / (2.0 * tPrefix));
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            return evalA(brakeSubAmpl[bph], brakeSubSign[bph], dt, brakeSubDur[bph]);
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double T = phaseStartT[ph + 1] - phaseStartT[ph];
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
        return phaseSign[findPhase(t)] == 0;
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

    /** Returns [p, v] at the end of a phase. Used for forward-chaining the phase table. */
    private static double[] evalPhaseEnd(
            double pStar, double vStar, double ampl, int sign, double T) {
        return new double[] {evalP(pStar, vStar, ampl, sign, T, T), evalV(vStar, ampl, sign, T, T)};
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
