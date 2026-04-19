package org.marsroboticsassociation.controllib.motion;

import java.util.ArrayList;
import java.util.List;

/**
 * Generates a jerk-limited (S-curve) velocity trajectory between two velocities,
 * with support for asymmetric jerk (different magnitudes when increasing vs decreasing acceleration).
 * <p>
 * Behavior notes:
 * - Backwards-compatible constructor: SCurveVelocity(v0, v1, a0, aMax, jMax) => symmetric jerk
 * - New constructor: SCurveVelocity(v0, v1, a0, aMax, jInc, jDec) => asymmetric jerk
 * <p>
 * Tuning hint: to mimic a flywheel motor that ramps up quickly but settles slowly, use
 * jInc >> jDec (large jerk to speed up, small jerk to slow the change at the end).
 */
public class SCurveVelocity implements VelocityTrajectory {
    public final double v0, v1, a0, aMax;
    public final double jInc, jDec; // jInc = jerk magnitude when increasing accel, jDec = when decreasing accel
    public final double t1, t2, t3, tf;
    public final double aPeak;
    private final double dir;        // +1 if accelerating (v1>v0), -1 if decelerating
    private final boolean trivial;   // true if v0 ≈ v1
    private final boolean singlePhase; // true if only one jerk phase used

    /**
     * Symmetric-jerk constructor (backwards compatible).
     */
    public SCurveVelocity(double v0, double v1, double a0, double aMax, double jMax) {
        this(v0, v1, a0, aMax, jMax, jMax);
    }

    /**
     * Asymmetric-jerk constructor.
     *
     * @param v0   Initial velocity
     * @param v1   Target velocity
     * @param a0   Initial acceleration (may be positive or negative)
     * @param aMax Maximum magnitude of acceleration allowed (>= 0)
     * @param jInc Maximum magnitude of jerk when increasing acceleration (>= 0)
     * @param jDec Maximum magnitude of jerk when decreasing acceleration (>= 0)
     */
    public SCurveVelocity(double v0, double v1, double a0, double aMax, double jInc, double jDec) {
        this.v0 = v0;
        this.v1 = v1;
        this.a0 = a0;
        this.aMax = Math.abs(aMax);
        this.jInc = Math.abs(jInc);
        this.jDec = Math.abs(jDec);

        this.dir = Math.signum(v1 - v0);
        double dv = Math.abs(v1 - v0);

        // --- Trivial case: no velocity change ---
        if (dv < 1e-9) {
            trivial = true;
            singlePhase = false;
            t1 = t2 = t3 = tf = aPeak = 0.0;
            return;
        }

        trivial = false;

        // Work in magnitude (positive-motion) frame for acceleration math
        double a0pos = Math.abs(a0);

        // --- Determine if it's too small to null a0 within jerk limits ---
        // To reduce the initial acceleration to zero we use the *decreasing* jerk magnitude (jDec)
        double dv_to_zero_accel = 0.5 * a0pos * a0pos / Math.max(jDec, 1e-12);

        if (dv < dv_to_zero_accel && a0pos > 1e-9) {
            // Single-phase motion: constant jerk to reduce acceleration magnitude toward zero
            singlePhase = true;

            // Solve 0.5 * jDec * t^2 - a0pos * t + dv = 0  (note: jDec used because we're reducing accel)
            double disc = a0pos * a0pos - 2.0 * jDec * dv;
            if (disc < 0) disc = 0;
            double tActual = (a0pos - Math.sqrt(disc)) / Math.max(jDec, 1e-12);

            this.t1 = tActual;
            this.t2 = 0;
            this.t3 = 0;
            this.tf = tActual;
            this.aPeak = Math.max(0.0, a0pos - jDec * tActual);
            return;
        }

        singlePhase = false;

        // --- Decide whether to jerk up (increase accel) or down (decrease accel) first ---
        boolean jerkUpFirst = (dv > dv_to_zero_accel);

        // If jerkUpFirst: we will *increase* acceleration toward aMax using jInc,
        // then decrease to zero using jDec.
        // If not jerkUpFirst: we will *decrease* acceleration toward zero first using jDec,
        // then possibly increase later (handled by same equations with aTarget = 0).

        // --- Try full aMax motion (trapezoidal) ---
        double aTarget = jerkUpFirst ? this.aMax : 0.0;

        // t1: ramp from a0pos -> aTarget using jInc if increasing, jDec if decreasing
        double t1tmp;
        if (aTarget >= a0pos) {
            // increasing accel phase uses jInc
            t1tmp = (aTarget - a0pos) / Math.max(jInc, 1e-12);
        } else {
            // decreasing accel phase uses jDec
            t1tmp = (a0pos - aTarget) / Math.max(jDec, 1e-12);
        }
        if (t1tmp < 0) t1tmp = 0;

        // t3: fall from aTarget -> 0 uses jDec (always decreasing to zero at end)
        double t3tmp = aTarget / Math.max(jDec, 1e-12);

        // Velocity change from full (trapezoidal) motion:
        // dvFull = area from ramp up + plateau + ramp down (we'll handle plateau separately)
        double dvFull = 0.5 * (a0pos + aTarget) * Math.abs(t1tmp) + 0.5 * aTarget * t3tmp;
        double t2tmp = (dv - dvFull) / Math.max(aTarget, 1e-9);

        double aPeakLocal, t1Local, t2Local, t3Local;

        if (t2tmp < 0) {
            // --- Triangular profile (never reach full aMax) ---
            t2Local = 0;

            // For asymmetric jerk, derive aPeak from:
            // dv = (aPeak^2 - a0pos^2) / (2*jInc) + aPeak^2 / (2*jDec)
            // Solve for aPeak^2:
            // aPeak^2 * (1/jInc + 1/jDec) = 2*dv + a0pos^2 / jInc
            // => aPeak^2 = (2*dv + a0pos^2/jInc) / (1/jInc + 1/jDec)
            double denom = (1.0 / Math.max(jInc, 1e-12)) + (1.0 / Math.max(jDec, 1e-12));
            double numer = 2.0 * dv + (a0pos * a0pos) / Math.max(jInc, 1e-12);
            double aPeakSq = Math.max(0.0, numer / denom);
            aPeakLocal = Math.sqrt(aPeakSq);

            // t1: from a0pos -> aPeakLocal. Use jInc when increasing, jDec when decreasing
            if (aPeakLocal >= a0pos) t1Local = (aPeakLocal - a0pos) / Math.max(jInc, 1e-12);
            else t1Local = (a0pos - aPeakLocal) / Math.max(jDec, 1e-12);

            // t3: ramp down from aPeakLocal -> 0 uses jDec
            t3Local = aPeakLocal / Math.max(jDec, 1e-12);
        } else {
            // --- Full trapezoidal profile ---
            aPeakLocal = aTarget;

            if (aTarget >= a0pos) t1Local = (aTarget - a0pos) / Math.max(jInc, 1e-12);
            else t1Local = (a0pos - aTarget) / Math.max(jDec, 1e-12);

            t2Local = t2tmp;
            t3Local = t3tmp;
        }

        this.aPeak = aPeakLocal;
        this.t1 = t1Local;
        this.t2 = t2Local;
        this.t3 = t3Local;
        this.tf = t1 + t2 + t3;
    }

    @Override
    public double getAcceleration(double t) {
        if (trivial) return 0;

        double aMag; // magnitude (positive-motion frame)
        if (singlePhase) {
            if (t < 0) aMag = Math.abs(a0);
            else if (t < tf)
                aMag = Math.abs(a0) - jDec * t; // single-phase reduces accel -> use jDec
            else aMag = 0;
            return dir * aMag;
        }

        if (t < 0) aMag = Math.abs(a0);
        else if (t < t1) {
            // ramp from a0 -> aPeak
            if (aPeak >= Math.abs(a0)) {
                // increasing accel: use jInc
                aMag = Math.abs(a0) + jInc * t;
            } else {
                // decreasing accel: use jDec
                aMag = Math.abs(a0) - jDec * t;
            }
        } else if (t < t1 + t2) aMag = aPeak;
        else if (t < tf) {
            double dt3 = t - t1 - t2;
            // final ramp down to zero uses jDec
            aMag = aPeak - jDec * dt3;
            if (aMag < 0) aMag = 0;
        } else aMag = 0;

        return dir * aMag;
    }

    @Override
    public double getVelocity(double t) {
        if (trivial) return v1;
        if (t <= 0) return v0;

        double v; // in magnitude frame
        if (singlePhase) {
            if (t < tf)
                v = v0 + Math.abs(a0) * t - 0.5 * jDec * t * t;
            else
                v = v0 + Math.abs(a0) * tf - 0.5 * jDec * tf * tf;
            return v0 + dir * Math.abs(v - v0);
        }

        // Multi-phase
        if (t < t1) {
            // first ramp
            if (aPeak >= Math.abs(a0)) {
                // increasing accel -> +0.5 * jInc * t^2
                v = v0 + Math.abs(a0) * t + 0.5 * jInc * t * t;
            } else {
                // decreasing accel -> -0.5 * jDec * t^2
                v = v0 + Math.abs(a0) * t - 0.5 * jDec * t * t;
            }
        } else if (t < t1 + t2) {
            // plateau region
            v = v0 + (Math.abs(a0) + aPeak) * t1 / 2.0 + aPeak * (t - t1);
        } else if (t < tf) {
            double dt3 = t - t1 - t2;
            double v2 = v0 + (Math.abs(a0) + aPeak) * t1 / 2.0 + aPeak * t2;
            // final ramp down uses jDec
            v = v2 + aPeak * dt3 - 0.5 * jDec * dt3 * dt3;
        } else {
            v = v1;
        }

        return v0 + dir * Math.abs(v - v0);
    }

    @Override
    public double getTotalTime() {
        return tf;
    }

    @Override
    public boolean isZeroJerk(double t) {
        return trivial || singlePhase ||
                t <= 0 || (t >= t1 && t < t1 + t2) || t > tf;
    }

    public List<PolynomialCurveSegment> velocitySegments() {
        List<PolynomialCurveSegment> segments = new ArrayList<>();
        if (trivial || tf <= 0) return segments;

        double a0Signed = dir * Math.abs(a0);

        if (singlePhase) {
            segments.add(
                    new PolynomialCurveSegment(
                            0.0,
                            tf,
                            v0,
                            a0Signed,
                            -0.5 * dir * jDec,
                            0.0));
            return segments;
        }

        double tStart = 0.0;
        double vStart = v0;
        double aStart = a0Signed;

        if (t1 > 0) {
            double jerk1 = firstPhaseJerkSigned();
            segments.add(
                    new PolynomialCurveSegment(
                            tStart,
                            tStart + t1,
                            vStart,
                            aStart,
                            0.5 * jerk1,
                            0.0));
            vStart = getVelocity(tStart + t1);
            aStart = getAcceleration(tStart + t1);
            tStart += t1;
        }

        if (t2 > 0) {
            segments.add(
                    new PolynomialCurveSegment(
                            tStart,
                            tStart + t2,
                            vStart,
                            aStart,
                            0.0,
                            0.0));
            vStart = getVelocity(tStart + t2);
            aStart = getAcceleration(tStart + t2);
            tStart += t2;
        }

        if (t3 > 0) {
            double jerk3 = -dir * jDec;
            segments.add(
                    new PolynomialCurveSegment(
                            tStart,
                            tStart + t3,
                            vStart,
                            aStart,
                            0.5 * jerk3,
                            0.0));
        }
        return segments;
    }

    public List<PolynomialCurveSegment> accelerationSegments() {
        List<PolynomialCurveSegment> segments = new ArrayList<>();
        if (trivial || tf <= 0) return segments;

        double a0Signed = dir * Math.abs(a0);

        if (singlePhase) {
            segments.add(
                    new PolynomialCurveSegment(
                            0.0,
                            tf,
                            a0Signed,
                            -dir * jDec,
                            0.0,
                            0.0));
            return segments;
        }

        double tStart = 0.0;
        double aStart = a0Signed;

        if (t1 > 0) {
            double jerk1 = firstPhaseJerkSigned();
            segments.add(new PolynomialCurveSegment(tStart, tStart + t1, aStart, jerk1, 0.0, 0.0));
            aStart = getAcceleration(tStart + t1);
            tStart += t1;
        }

        if (t2 > 0) {
            segments.add(new PolynomialCurveSegment(tStart, tStart + t2, aStart, 0.0, 0.0, 0.0));
            tStart += t2;
        }

        if (t3 > 0) {
            double jerk3 = -dir * jDec;
            segments.add(new PolynomialCurveSegment(tStart, tStart + t3, aStart, jerk3, 0.0, 0.0));
        }

        return segments;
    }

    private double firstPhaseJerkSigned() {
        return (aPeak >= Math.abs(a0) ? dir * jInc : -dir * jDec);
    }

    /**
     * Find the largest jDec (jerk when decreasing acceleration) that keeps a trajectory within
     * the motor's back-EMF voltage budget. Uses binary search over [1, 5000] and delegates to
     * the configurable overload with 30 iterations and 1000 samples.
     *
     * <p>Degenerate inputs are handled as follows rather than producing NaN or infinite values:
     * <ul>
     *   <li>{@code kA <= 0} — no acceleration constant, voltage limit is irrelevant →
     *       returns {@link Double#POSITIVE_INFINITY}</li>
     *   <li>{@code voltage <= kS} — motor cannot overcome static friction at any velocity →
     *       returns {@code 0.0}</li>
     *   <li>{@code aMax <= 0} or {@code aMax} is not finite — no useful trajectory can be
     *       constructed; pass the result of {@link #findMaxAMax} here →
     *       returns {@code 0.0}</li>
     *   <li>{@code jInc <= 0} — degenerate: acceleration rise time is near-infinite, sampling
     *       would miss violations → returns {@code 0.0}</li>
     *   <li>{@code v0 ≈ v1} (within 1e-9) — trivial trajectory, jDec has no effect →
     *       returns {@link Double#POSITIVE_INFINITY}</li>
     * </ul>
     *
     * @param v0        Initial velocity
     * @param v1        Target velocity
     * @param a0        Initial acceleration
     * @param aMax      Maximum acceleration magnitude; use result of {@link #findMaxAMax} if available
     * @param jInc      Jerk magnitude when increasing acceleration (must be &gt; 0)
     * @param voltage   Motor supply voltage (e.g. 12.0 V)
     * @param kS        Motor static feedforward (V); voltage needed to overcome friction
     * @param kV        Motor velocity feedforward (V per velocity unit)
     * @param kA        Motor acceleration feedforward (V per acceleration unit); must be &gt; 0
     * @return          Largest jDec in [1, 5000] that doesn't violate back-EMF limits, or a
     *                  sentinel as described above for degenerate inputs
     */
    public static double findMaxJDec(double v0, double v1, double a0, double aMax, double jInc,
            double voltage, double kS, double kV, double kA) {
        return findMaxJDec(v0, v1, a0, aMax, jInc, voltage, kS, kV, kA, 30, 1000);
    }

    /**
     * Find the largest jDec with configurable binary-search iterations and sample count.
     * See {@link #findMaxJDec(double, double, double, double, double, double, double, double, double)}
     * for full parameter and return-value documentation.
     *
     * @param iterations Number of binary-search bisection steps (default 30 gives ~1 part in 10^9 precision)
     * @param samples    Number of trajectory time points to check per candidate (default 1000)
     */
    public static double findMaxJDec(double v0, double v1, double a0, double aMax, double jInc,
            double voltage, double kS, double kV, double kA, int iterations, int samples) {
        if (kA <= 0) return Double.POSITIVE_INFINITY;
        if (voltage <= kS) return 0.0;                    // motor can't overcome static friction
        if (aMax <= 0 || !Double.isFinite(aMax)) return 0.0; // degenerate aMax produces NaN trajectories
        if (jInc <= 0) return 0.0;                        // degenerate jInc produces near-infinite rise times
        if (Math.abs(v1 - v0) < 1e-9) return Double.POSITIVE_INFINITY; // trivial trajectory, jDec irrelevant

        // Closed-form fast path: when the peak of P(τ) during phase 3 is interior (τ* > 0),
        // the voltage-budget constraint collapses to a linear equation in jDec:
        //   jDec = 2·kV·(voltage - kS - kV·|v1|) / kA²
        // Derivation: at the peak, a(τ*) = kA·jDec/kV and v(τ*) = |v1| - kA²·jDec/(2·kV²);
        // substituting into kS + kV·v + kA·a = voltage cancels all aPeak terms.
        // The interior-max condition is τ* > 0, equivalent to kV·aPeak > kA·jDec.
        double vHead = voltage - kS - kV * Math.abs(v1);
        if (vHead > 0) {
            double jDecClosed = 2.0 * kV * vHead / (kA * kA);
            double dv = Math.abs(v1 - v0);
            double aPeakSq = 2.0 * dv * jInc * jDecClosed / (jInc + jDecClosed);
            double aPeak = Math.min(Math.sqrt(aPeakSq), aMax);
            if (kV * aPeak > kA * jDecClosed && jDecClosed <= 5000) {
                return jDecClosed;
            }
        }

        // Boundary case (low target velocity, aPeak < kA·jDec/kV) or jDec capped:
        // peak of P(τ) is not interior, so constraint is different. Fall back to binary search.
        double low = 1;
        double high = 5000;
        double best = low;

        for (int iter = 0; iter < iterations; iter++) {
            double mid = (low + high) / 2;

            SCurveVelocity traj = new SCurveVelocity(v0, v1, a0, aMax, jInc, mid);
            double totalTime = traj.getTotalTime();
            if (!Double.isFinite(totalTime)) {
                high = mid;
                continue;
            }
            
            boolean violates = false;
            for (int i = 0; i <= samples; i++) {
                double t = totalTime * i / samples;
                double v = traj.getVelocity(t);
                double a = traj.getAcceleration(t);
                
                double availableVoltage = voltage - kS - kV * Math.abs(v);
                if (availableVoltage <= 0) {
                    violates = true;
                    break;
                }
                double motorAMax = availableVoltage / kA;
                if (a > motorAMax) {
                    violates = true;
                    break;
                }
            }
            
            if (violates) {
                high = mid;
            } else {
                best = mid;
                low = mid;
            }
        }
        
        return best;
    }

    /**
     * Find the largest aMax (peak acceleration) that keeps a trajectory within the motor's
     * back-EMF voltage budget. The search starts from the theoretical motor maximum at v=0
     * ({@code (voltage - kS) / kA}) and binary-searches upward to 5000 using 30 iterations
     * and 1000 samples per candidate. The search uses symmetric jerk ({@code jInc = jDec})
     * and zero initial acceleration, which is appropriate for finding a conservative peak
     * acceleration limit independent of jerk shape.
     *
     * <p>Degenerate inputs are handled as follows rather than producing NaN or infinite values:
     * <ul>
     *   <li>{@code kA <= 0} — no acceleration constant, voltage limit is irrelevant →
     *       returns {@link Double#POSITIVE_INFINITY}</li>
     *   <li>{@code voltage <= kS} — motor cannot overcome static friction, so no acceleration
     *       is safe → returns {@code 0.0}</li>
     *   <li>{@code jInc <= 0} — degenerate: the search would construct trajectories with
     *       near-infinite rise times, causing samples to miss violations → returns {@code 0.0}</li>
     * </ul>
     *
     * <p>Typical usage — call this first, then pass the result to {@link #findMaxJDec}:
     * <pre>
     *   double aMax = SCurveVelocity.findMaxAMax(0, targetV, jInc, voltage, kS, kV, kA);
     *   double jDec = SCurveVelocity.findMaxJDec(0, targetV, 0, aMax, jInc, voltage, kS, kV, kA);
     * </pre>
     *
     * @param v0        Initial velocity
     * @param v1        Target velocity
     * @param jInc      Jerk magnitude when increasing acceleration (must be &gt; 0)
     * @param voltage   Motor supply voltage (e.g. 12.0 V)
     * @param kS        Motor static feedforward (V); voltage needed to overcome friction
     * @param kV        Motor velocity feedforward (V per velocity unit)
     * @param kA        Motor acceleration feedforward (V per acceleration unit); must be &gt; 0
     * @return          Largest aMax in [{@code (voltage-kS)/kA}, 5000] that doesn't violate
     *                  back-EMF limits, or a sentinel as described above for degenerate inputs
     */
    public static double findMaxAMax(double v0, double v1, double jInc,
            double voltage, double kS, double kV, double kA) {
        if (kA <= 0) return Double.POSITIVE_INFINITY;
        if (voltage <= kS) return 0.0;  // motor can't overcome static friction
        if (jInc <= 0) return 0.0;      // degenerate jInc produces near-infinite rise times

        double motorAMaxAtZero = (voltage - kS) / kA;
        double low = motorAMaxAtZero;
        double high = 5000;
        double best = low;
        
        for (int iter = 0; iter < 30; iter++) {
            double mid = (low + high) / 2;
            
            SCurveVelocity traj = new SCurveVelocity(v0, v1, 0, mid, jInc, jInc);
            double totalTime = traj.getTotalTime();
            
            boolean violates = false;
            int samples = 1000;
            for (int i = 0; i <= samples; i++) {
                double t = totalTime * i / samples;
                double v = traj.getVelocity(t);
                double a = traj.getAcceleration(t);
                
                double availableVoltage = voltage - kS - kV * Math.abs(v);
                if (availableVoltage <= 0) {
                    violates = true;
                    break;
                }
                double motorAMax = availableVoltage / kA;
                if (a > motorAMax) {
                    violates = true;
                    break;
                }
            }
            
            if (violates) {
                high = mid;
            } else {
                best = mid;
                low = mid;
            }
        }
        
        return best;
    }
}
