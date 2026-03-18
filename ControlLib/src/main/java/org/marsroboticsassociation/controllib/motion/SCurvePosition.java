package org.marsroboticsassociation.controllib.motion;

/**
 * Jerk-limited (S-curve) position trajectory from p0 to pTarget, ending at rest.
 * Supports asymmetric acceleration limits: aMaxAccel for speeding up, aMaxDecel for braking.
 * Symmetric jerk: single jMax value.
 *
 * <p>The trajectory has an optional "prefix" phase that nulls out any non-zero initial
 * acceleration (by applying ±jMax until a=0), followed by up to 7 standard phases:
 * <ol>
 *   <li>Jerk +jMax:  a ramps from 0 → aMaxAccel (or triangular peak)</li>
 *   <li>Zero jerk:   constant acceleration aMaxAccel</li>
 *   <li>Jerk -jMax:  a ramps from aMaxAccel → 0</li>
 *   <li>Zero jerk:   constant velocity vPeak (cruise)</li>
 *   <li>Jerk -jMax:  a ramps from 0 → -aMaxDecel</li>
 *   <li>Zero jerk:   constant deceleration -aMaxDecel</li>
 *   <li>Jerk +jMax:  a ramps from -aMaxDecel → 0</li>
 * </ol>
 */
public class SCurvePosition implements PositionTrajectory {

    public final double p0, pTarget, v0, a0, vMax, aMaxAccel, aMaxDecel, jMax;

    // Phase durations (7-phase section only, after prefix)
    public final double T1, T2, T3, T4, T5, T6, T7;

    // Peak velocity reached during cruise
    public final double vPeak;

    private final boolean trivial;
    private final double dir; // +1 if pTarget >= p0, else -1

    // Prefix phase: nulls out a0 before the 7-phase section
    public final double tPrefix;    // duration (0 if a0 == 0)
    private final double jPrefix;   // world-frame jerk during prefix

    // State at end of a0 prefix (start of braking prefix)
    private final double pAfterA0Prefix;
    private final double vAfterA0Prefix;

    // Braking prefix: jerk-limited 3-sub-phase decel to bring wrong-way velocity to 0
    public final double tBrake;        // total duration (0 if not needed)
    public final double aBrakeHandoff; // acceleration magnitude at brake→main handoff (0 if no braking)
    private final double pBrake;       // position at end of braking
    private final double aMaxBrake;    // max(aMaxAccel, aMaxDecel) used during braking

    // Brake sub-phase table (up to 3 sub-phases)
    private final double[] brakeSubT = new double[4]; // absolute start times
    private final double[] brakeSubP = new double[4];
    private final double[] brakeSubV = new double[4];
    private final double[] brakeSubA = new double[4];
    private final double[] brakeSubJ = new double[3]; // world-frame jerk per sub-phase

    // Phase table: 8 breakpoints for the 7-phase section.
    // phaseStartT[0] = tPrefix (7-phase section begins after prefix).
    private final double[] phaseStartT = new double[8];
    private final double[] phaseStartP = new double[8];
    private final double[] phaseStartV = new double[8];
    private final double[] phaseStartA = new double[8];
    private final double[] phaseJerk   = new double[7]; // world-frame jerk for each phase

    public SCurvePosition(double p0, double pTarget, double v0, double a0,
                          double vMax, double aMaxAccel, double aMaxDecel, double jMax) {
        this.p0 = p0;
        this.pTarget = pTarget;
        this.v0 = v0;
        this.a0 = a0;
        this.vMax     = Math.abs(vMax);
        this.aMaxAccel = Math.abs(aMaxAccel);
        this.aMaxDecel = Math.abs(aMaxDecel);
        this.jMax     = Math.abs(jMax);

        double totalDistAbs = Math.abs(pTarget - p0);
        this.dir = (pTarget > p0) ? 1.0 : (pTarget < p0) ? -1.0 : 1.0;

        if (totalDistAbs < 1e-9) {
            trivial = true;
            T1 = T2 = T3 = T4 = T5 = T6 = T7 = 0;
            vPeak = 0;
            tPrefix = 0;
            jPrefix = 0;
            pAfterA0Prefix = p0;
            vAfterA0Prefix = v0;
            tBrake = 0; pBrake = p0; aMaxBrake = 0; aBrakeHandoff = 0;
            phaseStartP[0] = p0; phaseStartV[0] = v0; phaseStartA[0] = a0;
            for (int i = 1; i < 8; i++) {
                phaseStartT[i] = 0;
                phaseStartP[i] = p0;
                phaseStartV[i] = v0;
                phaseStartA[i] = 0;
            }
            return;
        }
        trivial = false;

        // ---------------------------------------------------------------
        // Prefix: apply ±jMax for |a0| / jMax seconds to bring a → 0.
        // For a0pos > 0 (accelerating toward target): jerk = -jMax in pos frame
        // For a0pos < 0 (decelerating / wrong direction): jerk = +jMax in pos frame
        // ---------------------------------------------------------------
        double a0pos = a0 * dir;
        double tPre, jPre;
        double pStart, vStart;

        if (Math.abs(a0pos) < 1e-9) {
            tPre   = 0;
            jPre   = 0;
            pStart = p0;
            vStart = v0;
        } else {
            tPre = Math.abs(a0pos) / this.jMax;
            // jerk in world frame that nulls a0:
            jPre = (a0pos > 0) ? (-dir * this.jMax) : (dir * this.jMax);
            pStart = p0 + v0 * tPre + 0.5 * a0 * tPre * tPre
                       + (1.0 / 6.0) * jPre * tPre * tPre * tPre;
            vStart = v0 + a0 * tPre + 0.5 * jPre * tPre * tPre;
            // a at end of prefix = 0 by construction
        }
        this.tPrefix = tPre;
        this.jPrefix = jPre;
        this.pAfterA0Prefix = pStart;
        this.vAfterA0Prefix = vStart;

        // Braking prefix: if vStart is in the wrong direction, brake to 0 with jerk-limited profile
        double tBrk, pBrk, aBrakeVal;
        double aHandoffActual = 0.0;
        if (vStart * dir < -1e-9) {
            double speed = Math.abs(vStart);
            aBrakeVal = Math.max(this.aMaxAccel, this.aMaxDecel);
            double aHandoff = this.aMaxAccel; // asymmetric: handoff at aMaxAccel, not aBrakeVal
            double triThresh = aHandoff * aHandoff / (2.0 * this.jMax); // one-sided triangle threshold
            double tBrk1, tBrk2, tBrk3;
            if (speed > triThresh) {
                // Trapezoidal: ramp 0→aHandoff, hold until v=0, no ramp-down
                tBrk1 = aHandoff / this.jMax;
                tBrk2 = (speed - triThresh) / aHandoff;
                tBrk3 = 0;
                aHandoffActual = aHandoff; // = aMaxAccel
            } else {
                // Triangular: single ramp-up until v=0
                tBrk1 = Math.sqrt(2.0 * speed / this.jMax);
                tBrk2 = 0;
                tBrk3 = 0;
                aHandoffActual = this.jMax * tBrk1; // = sqrt(2*speed*jMax) <= aMaxAccel
            }
            tBrk = tBrk1 + tBrk2 + tBrk3;
            // Jerk in world frame: braking opposes vStart direction
            double jBrk = -Math.signum(vStart) * this.jMax;
            brakeSubJ[0] = jBrk;
            brakeSubJ[1] = 0;
            brakeSubJ[2] = 0; // no ramp-down: tBrk3=0 makes this a no-op
            // Forward-chain brake sub-phases from (pStart, vStart, 0) at absolute time tPrefix
            brakeSubT[0] = tPrefix;
            brakeSubP[0] = pStart;
            brakeSubV[0] = vStart;
            brakeSubA[0] = 0.0;
            double[] brkDurs = { tBrk1, tBrk2, tBrk3 };
            for (int i = 0; i < 3; i++) {
                double dt = brkDurs[i];
                double j  = brakeSubJ[i];
                brakeSubP[i + 1] = brakeSubP[i] + brakeSubV[i] * dt
                        + 0.5 * brakeSubA[i] * dt * dt + (1.0 / 6.0) * j * dt * dt * dt;
                brakeSubV[i + 1] = brakeSubV[i] + brakeSubA[i] * dt + 0.5 * j * dt * dt;
                brakeSubA[i + 1] = brakeSubA[i] + j * dt;
                brakeSubT[i + 1] = brakeSubT[i] + dt;
            }
            pBrk = brakeSubP[3];
        } else {
            tBrk = 0; aBrakeVal = 0; pBrk = pStart;
        }
        this.tBrake = tBrk;
        this.aMaxBrake = aBrakeVal;
        this.pBrake = pBrk;
        this.aBrakeHandoff = aHandoffActual;

        double distRemaining = Math.max(0, dir * (pTarget - pBrk));
        double v0adj = (tBrk > 0) ? 0.0 : Math.min(Math.max(vStart * dir, 0.0), this.vMax);
        double vStartWorld = v0adj * dir;
        double aStart = (tBrk > 0) ? aHandoffActual : 0.0;

        // ---------------------------------------------------------------
        // Bisect for vPeak. D(v) is strictly increasing in v for v in [v0adj, vMax].
        // ---------------------------------------------------------------
        double dAtVMax = accelHalfDistFrom(v0adj, this.vMax, this.aMaxAccel, this.jMax, aStart)
                       + halfDist(this.vMax, this.aMaxDecel, this.jMax);

        double vPeakSolved;
        double T4solved;

        if (distRemaining >= dAtVMax) {
            vPeakSolved = this.vMax;
            T4solved = (this.vMax > 1e-9) ? (distRemaining - dAtVMax) / this.vMax : 0;
        } else {
            T4solved = 0;
            double dAtV0adj = halfDist(v0adj, this.aMaxDecel, this.jMax);
            if (distRemaining <= dAtV0adj) {
                // Not enough room to even accelerate; just decel from v0adj
                vPeakSolved = v0adj;
            } else {
                double lo = v0adj, hi = this.vMax;
                for (int i = 0; i < 64; i++) {
                    double mid = (lo + hi) * 0.5;
                    double d = accelHalfDistFrom(v0adj, mid, this.aMaxAccel, this.jMax, aStart)
                             + halfDist(mid, this.aMaxDecel, this.jMax);
                    if (d < distRemaining) lo = mid;
                    else hi = mid;
                }
                vPeakSolved = (lo + hi) * 0.5;
            }
        }
        this.vPeak = vPeakSolved;
        this.T4 = Math.max(0, T4solved);

        // ---------------------------------------------------------------
        // Phase durations from vPeak
        // ---------------------------------------------------------------
        double vAccelMin = this.aMaxAccel * this.aMaxAccel / this.jMax;
        double vDecelMin = this.aMaxDecel * this.aMaxDecel / this.jMax;
        double dvAccel = vPeakSolved - v0adj;

        double t1, t2, t3;
        double trapThresh = (2.0 * this.aMaxAccel * this.aMaxAccel - aStart * aStart) / (2.0 * this.jMax);
        if (dvAccel <= 0) {
            t1 = 0; t2 = 0; t3 = 0;
        } else if (dvAccel >= trapThresh) {
            // Trapezoidal: ramp aStart→aMaxAccel, hold, ramp aMaxAccel→0
            t1 = (this.aMaxAccel - aStart) / this.jMax;
            t2 = (dvAccel - trapThresh) / this.aMaxAccel;
            t3 = this.aMaxAccel / this.jMax;
        } else {
            // Triangular: ramp aStart→aPk→0
            double aPk = Math.sqrt(dvAccel * this.jMax + aStart * aStart / 2.0);
            t1 = (aPk - aStart) / this.jMax;
            t2 = 0;
            t3 = aPk / this.jMax;
        }

        double t5, t6, t7;
        if (vPeakSolved >= vDecelMin) {
            t5 = this.aMaxDecel / this.jMax;
            t6 = (vPeakSolved - vDecelMin) / this.aMaxDecel;
            t7 = t5;
        } else {
            t5 = (vPeakSolved > 0) ? Math.sqrt(vPeakSolved / this.jMax) : 0;
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
        phaseJerk[0] =  dir * this.jMax;
        phaseJerk[1] =  0;
        phaseJerk[2] = -dir * this.jMax;
        phaseJerk[3] =  0;
        phaseJerk[4] = -dir * this.jMax;
        phaseJerk[5] =  0;
        phaseJerk[6] =  dir * this.jMax;

        phaseStartT[0] = tPre + tBrk;
        phaseStartP[0] = pBrk;
        phaseStartV[0] = vStartWorld;
        phaseStartA[0] = (tBrk > 0) ? aHandoffActual * dir : 0.0;

        double[] durations = { T1, T2, T3, T4, T5, T6, T7 };
        for (int i = 0; i < 7; i++) {
            double dt = durations[i];
            double j  = phaseJerk[i];
            double ps = phaseStartP[i];
            double vs = phaseStartV[i];
            double as = phaseStartA[i];
            phaseStartP[i + 1] = ps + vs * dt + 0.5 * as * dt * dt
                                    + (1.0 / 6.0) * j * dt * dt * dt;
            phaseStartV[i + 1] = vs + as * dt + 0.5 * j * dt * dt;
            phaseStartA[i + 1] = as + j * dt;
            phaseStartT[i + 1] = phaseStartT[i] + dt;
        }
    }

    // ---------------------------------------------------------------
    // Distance helpers
    // ---------------------------------------------------------------

    /**
     * Distance covered accelerating (with jerk-limit) from vFrom (a=0) to vPeak (a=0).
     * Returns 0 if vPeak <= vFrom.
     */
    private static double accelHalfDistFrom(double vFrom, double vPeak,
                                             double aMax, double jMax) {
        return accelHalfDistFrom(vFrom, vPeak, aMax, jMax, 0.0);
    }

    /**
     * Distance covered accelerating (with jerk-limit) from vFrom (a=aStart) to vPeak (a=0).
     * Returns 0 if vPeak <= vFrom.
     */
    private static double accelHalfDistFrom(double vFrom, double vPeak,
                                             double aMax, double jMax, double aStart) {
        double dv = vPeak - vFrom;
        if (dv <= 0) return 0;
        double trapThresh = (2.0 * aMax * aMax - aStart * aStart) / (2.0 * jMax);
        if (dv >= trapThresh) {
            // Trapezoidal: ramp aStart→aMax, hold, ramp aMax→0
            double T1p   = (aMax - aStart) / jMax;
            double T2p   = (dv - trapThresh) / aMax;
            double T3    = aMax / jMax;
            double vEnd1 = vFrom + aStart * T1p + 0.5 * jMax * T1p * T1p;
            double d1    = vFrom * T1p + 0.5 * aStart * T1p * T1p + (1.0 / 6.0) * jMax * T1p * T1p * T1p;
            double d2    = vEnd1 * T2p + 0.5 * aMax * T2p * T2p;
            double vEnd2 = vEnd1 + aMax * T2p;
            double d3    = vEnd2 * T3 + 0.5 * aMax * T3 * T3 - (1.0 / 6.0) * jMax * T3 * T3 * T3;
            return d1 + d2 + d3;
        } else {
            // Triangular: ramp aStart→aPk→0
            double aPk   = Math.sqrt(dv * jMax + aStart * aStart / 2.0);
            double T1p   = (aPk - aStart) / jMax;
            double T3    = aPk / jMax;
            double vEnd1 = vFrom + aStart * T1p + 0.5 * jMax * T1p * T1p;
            double d1    = vFrom * T1p + 0.5 * aStart * T1p * T1p + (1.0 / 6.0) * jMax * T1p * T1p * T1p;
            double d3    = vEnd1 * T3 + 0.5 * aPk * T3 * T3 - (1.0 / 6.0) * jMax * T3 * T3 * T3;
            return d1 + d3;
        }
    }

    /** Distance covered decelerating (jerk-limited) from vPeak (a=0) to 0 (a=0). */
    private static double halfDist(double vPeak, double aMax, double jMax) {
        return accelHalfDistFrom(0, vPeak, aMax, jMax);
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
            return p0 + v0 * t + 0.5 * a0 * t * t
                   + (1.0 / 6.0) * jPrefix * t * t * t;
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            double j  = brakeSubJ[bph];
            return brakeSubP[bph] + brakeSubV[bph] * dt
                   + 0.5 * brakeSubA[bph] * dt * dt
                   + (1.0 / 6.0) * j * dt * dt * dt;
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double j  = phaseJerk[ph];
        return phaseStartP[ph] + phaseStartV[ph] * dt
               + 0.5 * phaseStartA[ph] * dt * dt
               + (1.0 / 6.0) * j * dt * dt * dt;
    }

    @Override
    public double getVelocity(double t) {
        if (trivial) return v0;
        if (t <= 0) return v0;
        double tf = getTotalTime();
        if (t >= tf) return 0;

        if (tPrefix > 0 && t < tPrefix) {
            return v0 + a0 * t + 0.5 * jPrefix * t * t;
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            double j  = brakeSubJ[bph];
            return brakeSubV[bph] + brakeSubA[bph] * dt + 0.5 * j * dt * dt;
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        double j  = phaseJerk[ph];
        return phaseStartV[ph] + phaseStartA[ph] * dt + 0.5 * j * dt * dt;
    }

    @Override
    public double getAcceleration(double t) {
        if (trivial) return a0;
        if (t <= 0) return a0;
        double tf = getTotalTime();
        if (t >= tf) return 0;

        if (tPrefix > 0 && t < tPrefix) {
            return a0 + jPrefix * t;
        }

        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) {
            int bph = findBrakePhase(t);
            double dt = t - brakeSubT[bph];
            return brakeSubA[bph] + brakeSubJ[bph] * dt;
        }

        int ph = findPhase(t);
        double dt = t - phaseStartT[ph];
        return phaseStartA[ph] + phaseJerk[ph] * dt;
    }

    @Override
    public double getTotalTime() {
        return trivial ? 0 : phaseStartT[7];
    }

    @Override
    public boolean isZeroJerk(double t) {
        if (trivial) return true;
        if (t <= 0 || t >= getTotalTime()) return true;
        if (tPrefix > 0 && t < tPrefix) return false; // prefix has active jerk
        double tBrakeEnd = tPrefix + tBrake;
        if (tBrake > 0 && t < tBrakeEnd) return brakeSubJ[findBrakePhase(t)] == 0;
        return phaseJerk[findPhase(t)] == 0;
    }

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
