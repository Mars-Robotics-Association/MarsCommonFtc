package org.marsroboticsassociation.controllib.mechanism;

import java.util.ArrayList;
import java.util.List;

/**
 * Arm system identification via the integrated equation of motion.
 *
 * <p>Recovers the five-parameter feedforward model
 *
 * <pre>
 *   V = kS·sign(ω) + kV·ω + kA·α + kCos·cos(θ) + kSin·sin(θ)
 * </pre>
 *
 * by regressing on integrated intervals rather than differentiating noisy encoder velocity:
 *
 * <pre>
 *   ∫V dt = kS·∫sign(ω) dt + kV·Δθ + kA·Δω + kCos·∫cosθ dt + kSin·∫sinθ dt
 * </pre>
 *
 * Every term is exact or a smoothing integral: {@code Δθ} comes from encoder position;
 * gravity terms are trapezoidal integrals of measured angle; the only boundary term is
 * {@code Δω}, obtained from a local quadratic fit of position (noise-tolerant, low-lag).
 * Hard steps / mid-run acceleration excite {@code kA}; angle sweeps excite gravity.
 *
 * <p>Intervals are kept only where velocity holds a single sign above a minimum speed and
 * stays clear of the hard stops, so {@code sign(ω)} is constant and contact forces do not
 * pollute the samples.
 *
 * <p>Typical workflow: call {@link #accumulateRun} once per constant-power run (robot or
 * sim), then {@link #solve}. ControlLab's simulated battery and the on-robot
 * {@code ArmSysIdTuning} OpMode both use this path.
 *
 * <p><b>Through a lashy/flexy drivetrain</b>, prefer the two-stage path: friction and gravity
 * from constant-velocity holds ({@link #accumulateHold}) plus inertia from the moving runs,
 * combined by {@link #solveTwoStage}. A hard-accelerating run through the lash and flex inflates
 * {@code kS} — the direction-flipping flex/lash deflection is collinear with the {@code sign(ω)}
 * regressor — and that over-estimate becomes an anti-braking feedforward term that reintroduces
 * arrival overshoot. Holding a steady speed keeps {@code α ≈ 0}, starving that bias. Same logging
 * (angle + voltage) as {@link #accumulateRun}; the caller just drives a held speed instead of a
 * constant voltage.
 *
 * @see ArmModel
 */
public final class ArmSysId {

    private ArmSysId() {}

    /** Identified feedforward coefficients plus a fit-quality score. */
    public static final class Result {
        public final double kS;
        public final double kV;
        public final double kA;
        public final double kCos;
        public final double kSin;
        public final double rSquared;
        public final int samples;

        public Result(
                double kS,
                double kV,
                double kA,
                double kCos,
                double kSin,
                double rSquared,
                int samples) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kCos = kCos;
            this.kSin = kSin;
            this.rSquared = rSquared;
            this.samples = samples;
        }

        /**
         * Gravity magnitude {@code √(kCos² + kSin²)}, the {@code kG} that a pure-cosine
         * {@code ArmFeedforward} would use when the encoder zero is corrected by {@link #phiRad()}.
         */
        public double kG() {
            return Math.hypot(kCos, kSin);
        }

        /**
         * Encoder-zero offset from horizontal implied by the gravity regressors:
         * {@code V_grav = kG·cos(θ − φ)} with {@code φ = atan2(kSin, kCos)}.
         * Zero when gravity is a pure cosine of the measured angle.
         */
        public double phiRad() {
            return Math.atan2(kSin, kCos);
        }
    }

    /**
     * Fit hyper-parameters. Defaults match ControlLab's simulated sysid battery.
     *
     * <p>All angles in radians, times in seconds, speeds in rad/s.
     */
    public static final class FitParams {
        /** Half-window (samples) for local quadratic velocity at interval endpoints. */
        public int velHalf = 3;
        /** |ω| must stay above this for an interval to be usable (sign well-defined). */
        public double minSpeedRad = 0.2;
        /** Discard samples within this margin of either hard stop. */
        public double stopMarginRad = Math.toRadians(4);
        /** Interval lengths in samples (multi-scale windows). */
        public int[] intervalLens = {8, 16, 30};
        /** Stride between interval starts (samples). */
        public int stride = 4;
        /**
         * A constant-velocity hold sample counts as steady (quasi-static) only when {@code |α|} is
         * below this, so the flex sits at its quasi-static deflection and there is no {@code kA·α}
         * term to bias {@code kS}. See {@link #accumulateHold}.
         */
        public double holdAccelGate = 1.0;
    }

    public static final FitParams DEFAULT_PARAMS = new FitParams();

    /**
     * Stack integrated-dynamics equations from one constant-voltage run into the OLS system.
     *
     * @param thetaRad equally-spaced arm angles (rad from horizontal, or encoder angle with offset)
     * @param voltage  constant applied voltage for the whole run (volts)
     * @param dt       sample period (seconds)
     * @param minAngleRad hard-stop lower bound (same units as {@code thetaRad})
     * @param maxAngleRad hard-stop upper bound
     * @param params   fit hyper-parameters
     * @param rows     feature rows {@code [sign·Δt, Δθ, Δω, ∫cos, ∫sin]} (appended)
     * @param rhs      right-hand side {@code V·Δt} (appended)
     */
    public static void accumulateRun(
            double[] thetaRad,
            double voltage,
            double dt,
            double minAngleRad,
            double maxAngleRad,
            FitParams params,
            List<double[]> rows,
            List<Double> rhs) {
        int n = thetaRad.length;
        if (n < 2 * params.velHalf + 2) {
            return;
        }
        double[] w = new double[n];
        for (int i = 0; i < n; i++) {
            w[i] = localVelocity(thetaRad, i, n, dt, params.velHalf);
        }

        int firstValid = params.velHalf;
        int lastValid = n - 1 - params.velHalf;
        for (int len : params.intervalLens) {
            for (int s = firstValid; s + len <= lastValid; s += params.stride) {
                int j = s + len;
                if (!intervalUsable(thetaRad, w, s, j, minAngleRad, maxAngleRad, params)) {
                    continue;
                }
                double dtInterval = len * dt;
                double sign = Math.signum(w[(s + j) / 2]);
                double dTheta = thetaRad[j] - thetaRad[s];
                double dW = w[j] - w[s];
                double iCos = trapz(thetaRad, s, j, dt, true);
                double iSin = trapz(thetaRad, s, j, dt, false);
                rows.add(new double[] {sign * dtInterval, dTheta, dW, iCos, iSin});
                rhs.add(voltage * dtInterval);
            }
        }
    }

    /**
     * Like {@link #accumulateRun(double[], double, double, double, double, FitParams, List, List)}
     * but with a per-sample applied voltage (handles battery sag). Interval RHS uses the mean
     * voltage over the interval times {@code Δt}.
     */
    public static void accumulateRun(
            double[] thetaRad,
            double[] voltage,
            double dt,
            double minAngleRad,
            double maxAngleRad,
            FitParams params,
            List<double[]> rows,
            List<Double> rhs) {
        int n = thetaRad.length;
        if (n < 2 || voltage.length != n) {
            return;
        }
        double[] timeSec = uniformTimes(n, dt);
        accumulateRun(thetaRad, voltage, timeSec, minAngleRad, maxAngleRad, params, rows, rhs);
    }

    /**
     * Variable-dt form for robot loops: sample every iteration with wall-clock timestamps.
     * Integrals use the actual per-step {@code dt}; no assumed fixed sample period.
     *
     * @param thetaRad  arm angles (rad)
     * @param voltage   applied voltage at each sample (volts)
     * @param timeSec   monotonically increasing sample times (seconds); any zero origin is fine
     */
    public static void accumulateRun(
            double[] thetaRad,
            double[] voltage,
            double[] timeSec,
            double minAngleRad,
            double maxAngleRad,
            FitParams params,
            List<double[]> rows,
            List<Double> rhs) {
        int n = thetaRad.length;
        if (n < 2 * params.velHalf + 2
                || voltage.length != n
                || timeSec.length != n) {
            return;
        }
        double[] w = new double[n];
        for (int i = 0; i < n; i++) {
            w[i] = localVelocity(thetaRad, timeSec, i, n, params.velHalf);
        }

        int firstValid = params.velHalf;
        int lastValid = n - 1 - params.velHalf;
        for (int len : params.intervalLens) {
            for (int s = firstValid; s + len <= lastValid; s += params.stride) {
                int j = s + len;
                if (!intervalUsable(thetaRad, w, s, j, minAngleRad, maxAngleRad, params)) {
                    continue;
                }
                double dtInterval = timeSec[j] - timeSec[s];
                if (dtInterval < 1e-6) {
                    continue;
                }
                double sign = Math.signum(w[(s + j) / 2]);
                double dTheta = thetaRad[j] - thetaRad[s];
                double dW = w[j] - w[s];
                double iCos = trapz(thetaRad, timeSec, s, j, true);
                double iSin = trapz(thetaRad, timeSec, s, j, false);
                double iV = trapz(voltage, timeSec, s, j);
                rows.add(new double[] {sign * dtInterval, dTheta, dW, iCos, iSin});
                rhs.add(iV);
            }
        }
    }

    /**
     * Stack quasi-static rows from one <b>constant-velocity hold</b> into the friction/gravity
     * system. Where {@link #accumulateRun} excites {@code kA} with acceleration, a hold suppresses
     * it: at a steady speed {@code α ≈ 0}, so each sample obeys {@code V = kS·sign(ω) + kV·ω +
     * kCos·cosθ + kSin·sinθ} and the flex spring sits at its quasi-static deflection. That is what
     * keeps a lashy/flexy drivetrain from inflating {@code kS} — the direction-flipping flex/lash
     * bias a hard-accelerating run pumps into the {@code sign(ω)} regressor is starved here.
     *
     * <p>Velocity and acceleration are recovered from the position log (local quadratic fit against
     * the actual sample times, then a central difference), so loop-rate jitter does not bias the fit
     * and the caller only logs angle, applied voltage, and wall-clock time — exactly like {@link
     * #accumulateRun}, just driven at a held speed instead of a constant voltage. Only steady
     * samples (|α| under {@link FitParams#holdAccelGate}, |ω| above {@link FitParams#minSpeedRad},
     * clear of the stops) are kept; each becomes a row {@code [sign(ω), ω, cosθ, sinθ, α]} with the
     * applied voltage as its right-hand side. Feed the accumulated rows to {@link #solveTwoStage}.
     *
     * @param thetaRad arm angles over the hold (rad)
     * @param voltage  per-sample applied voltage sustaining the hold (volts)
     * @param timeSec  monotonically increasing sample times (seconds); any zero origin is fine
     * @param rows     feature rows {@code [sign(ω), ω, cosθ, sinθ, α]} (appended)
     * @param rhs      right-hand side (applied voltage) (appended)
     */
    public static void accumulateHold(
            double[] thetaRad,
            double[] voltage,
            double[] timeSec,
            double minAngleRad,
            double maxAngleRad,
            FitParams params,
            List<double[]> rows,
            List<Double> rhs) {
        int n = thetaRad.length;
        if (n < 2 * params.velHalf + 3 || voltage.length != n || timeSec.length != n) {
            return;
        }
        double[] w = new double[n];
        for (int i = 0; i < n; i++) {
            w[i] = localVelocity(thetaRad, timeSec, i, n, params.velHalf);
        }
        for (int i = params.velHalf + 1; i < n - params.velHalf - 1; i++) {
            double dtc = timeSec[i + 1] - timeSec[i - 1];
            if (dtc < 1e-9) {
                continue;
            }
            double accel = (w[i + 1] - w[i - 1]) / dtc;
            if (Math.abs(w[i]) < params.minSpeedRad
                    || Math.abs(accel) > params.holdAccelGate
                    || thetaRad[i] < minAngleRad + params.stopMarginRad
                    || thetaRad[i] > maxAngleRad - params.stopMarginRad) {
                continue;
            }
            rows.add(new double[] {
                Math.signum(w[i]), w[i], Math.cos(thetaRad[i]), Math.sin(thetaRad[i]), accel
            });
            rhs.add(voltage[i]);
        }
    }

    /**
     * Two-stage fit that splits each parameter to the data that identifies it cleanly.
     *
     * <p><b>Stage 1 — {@code kS} and gravity from the holds.</b> An OLS over the {@link
     * #accumulateHold} rows yields {@code kS, kCos, kSin}. These are the terms a hard-accelerating
     * run through the lash + flex corrupts: the direction-flipping friction/lash deflection is
     * collinear with the {@code sign(ω)} regressor, so it lands in {@code kS} and biases gravity.
     * The quasi-static holds (α ≈ 0) starve that bias. (The rows also carry an {@code ω} column and
     * an {@code α} nuisance column so the intercept is clean; both fitted coefficients are discarded
     * here — {@code kV} is better taken from the moving runs, where speed varies over a wide range.)
     *
     * <p><b>Stage 2 — {@code kV} and {@code kA} from the moving runs.</b> With {@code kS} and gravity
     * subtracted, each {@link #accumulateRun} row reduces to {@code kV·Δθ + kA·Δω}; a 2-parameter OLS
     * over {@code (Δθ, Δω)} recovers both, where they are well-excited (a wide speed range and hard
     * onsets) and mutually separable — so {@code kA} does not inherit any hold-side {@code kV} error.
     * The reported {@code rSquared} is over the moving rows.
     *
     * <p>Falls back to a plain {@link #solve} on the moving rows if there are too few hold rows.
     *
     * @param holdRows   rows from {@link #accumulateHold}
     * @param holdRhs    their right-hand sides
     * @param movingRows rows from {@link #accumulateRun}
     * @param movingRhs  their right-hand sides
     */
    public static Result solveTwoStage(
            List<double[]> holdRows,
            List<Double> holdRhs,
            List<double[]> movingRows,
            List<Double> movingRhs) {
        if (holdRows.size() < 5 || movingRows.size() < 2) {
            return solve(movingRows, movingRhs);
        }
        double[] s1 = solveOls(holdRows.toArray(new double[0][]), toArray(holdRhs));
        double kS = s1[0], kCos = s1[2], kSin = s1[3]; // s1[1] (ω) and s1[4] (α) discarded

        // Stage 2: kV and kA together from the moving runs, with kS and gravity removed.
        double[][] x2 = new double[movingRows.size()][2]; // [Δθ, Δω]
        double[] y2 = new double[movingRows.size()];
        for (int i = 0; i < movingRows.size(); i++) {
            double[] xi = movingRows.get(i); // [sign·Δt, Δθ, Δω, ∫cos, ∫sin]
            x2[i][0] = xi[1];
            x2[i][1] = xi[2];
            y2[i] = movingRhs.get(i) - kS * xi[0] - kCos * xi[3] - kSin * xi[4];
        }
        double[] s2 = solveOls(x2, y2);
        double kV = s2[0], kA = s2[1];

        double r2 = rSquared(movingRows.toArray(new double[0][]), toArray(movingRhs),
                new double[] {kS, kV, kA, kCos, kSin});
        return new Result(kS, kV, kA, kCos, kSin, r2, holdRows.size() + movingRows.size());
    }

    /**
     * Solve the stacked OLS system for {@code kS, kV, kA, kCos, kSin}.
     *
     * @return fit result, or a zeroed result with {@code samples = 0} if there are too few rows
     */
    public static Result solve(List<double[]> rows, List<Double> rhs) {
        if (rows.size() < 5) {
            return new Result(0, 0, 0, 0, 0, 0, rows.size());
        }
        double[][] x = rows.toArray(new double[0][]);
        double[] y = toArray(rhs);
        double[] b = solveOls(x, y);
        double r2 = rSquared(x, y, b);
        return new Result(b[0], b[1], b[2], b[3], b[4], r2, y.length);
    }

    /**
     * Convenience: accumulate many constant-voltage runs then solve.
     *
     * @param runs each element is {@code {thetaRad[], voltage (boxed as length-1 or use overload)}}
     *             — prefer calling {@link #accumulateRun} + {@link #solve} for clarity
     */
    public static Result characterize(
            List<double[]> thetaRuns,
            List<Double> voltages,
            double dt,
            double minAngleRad,
            double maxAngleRad,
            FitParams params) {
        List<double[]> rows = new ArrayList<>();
        List<Double> rhs = new ArrayList<>();
        for (int i = 0; i < thetaRuns.size(); i++) {
            accumulateRun(
                    thetaRuns.get(i),
                    voltages.get(i),
                    dt,
                    minAngleRad,
                    maxAngleRad,
                    params,
                    rows,
                    rhs);
        }
        return solve(rows, rhs);
    }

    // ── internals ─────────────────────────────────────────────────────────────

    /** An interval is usable if velocity keeps one sign (above threshold) and clears hard stops. */
    static boolean intervalUsable(
            double[] theta,
            double[] w,
            int s,
            int j,
            double minAngleRad,
            double maxAngleRad,
            FitParams params) {
        double sign = Math.signum(w[(s + j) / 2]);
        for (int k = s; k <= j; k++) {
            if (Math.abs(w[k]) < params.minSpeedRad || Math.signum(w[k]) != sign) {
                return false;
            }
            if (theta[k] < minAngleRad + params.stopMarginRad) {
                return false;
            }
            if (theta[k] > maxAngleRad - params.stopMarginRad) {
                return false;
            }
        }
        return true;
    }

    /** Trapezoidal integral of cos(theta) (or sin) over [s, j] with fixed step {@code dt}. */
    static double trapz(double[] theta, int s, int j, double dt, boolean cos) {
        double sum = 0;
        for (int k = s; k < j; k++) {
            double a = cos ? Math.cos(theta[k]) : Math.sin(theta[k]);
            double b = cos ? Math.cos(theta[k + 1]) : Math.sin(theta[k + 1]);
            sum += 0.5 * (a + b) * dt;
        }
        return sum;
    }

    /** Trapezoidal integral of cos/sin(theta) over [s, j] using wall-clock sample times. */
    static double trapz(double[] theta, double[] timeSec, int s, int j, boolean cos) {
        double sum = 0;
        for (int k = s; k < j; k++) {
            double dtk = timeSec[k + 1] - timeSec[k];
            if (dtk < 1e-9) {
                continue;
            }
            double a = cos ? Math.cos(theta[k]) : Math.sin(theta[k]);
            double b = cos ? Math.cos(theta[k + 1]) : Math.sin(theta[k + 1]);
            sum += 0.5 * (a + b) * dtk;
        }
        return sum;
    }

    /** Trapezoidal integral of a scalar series over [s, j] using wall-clock sample times. */
    static double trapz(double[] values, double[] timeSec, int s, int j) {
        double sum = 0;
        for (int k = s; k < j; k++) {
            double dtk = timeSec[k + 1] - timeSec[k];
            if (dtk < 1e-9) {
                continue;
            }
            sum += 0.5 * (values[k] + values[k + 1]) * dtk;
        }
        return sum;
    }

    /** Velocity at index c from a local quadratic fit of position (noise-tolerant, low-lag). */
    static double localVelocity(double[] theta, int c, int n, double dt, int velHalf) {
        int half = Math.min(velHalf, Math.min(c, n - 1 - c));
        if (half < 1) {
            return 0.0;
        }
        int m = 2 * half + 1;
        double[][] x = new double[m][3];
        double[] y = new double[m];
        for (int k = -half, r = 0; k <= half; k++, r++) {
            double t = k * dt;
            x[r][0] = 1.0;
            x[r][1] = t;
            x[r][2] = t * t;
            y[r] = theta[c + k];
        }
        return solveOls(x, y)[1]; // coefficient of t is the velocity at the center
    }

    /**
     * Velocity at index c from a local quadratic fit using actual sample times (variable loop dt).
     */
    static double localVelocity(double[] theta, double[] timeSec, int c, int n, int velHalf) {
        int half = Math.min(velHalf, Math.min(c, n - 1 - c));
        if (half < 1) {
            return 0.0;
        }
        int m = 2 * half + 1;
        double t0 = timeSec[c];
        double[][] x = new double[m][3];
        double[] y = new double[m];
        for (int k = -half, r = 0; k <= half; k++, r++) {
            double t = timeSec[c + k] - t0;
            x[r][0] = 1.0;
            x[r][1] = t;
            x[r][2] = t * t;
            y[r] = theta[c + k];
        }
        return solveOls(x, y)[1];
    }

    private static double[] uniformTimes(int n, double dt) {
        double[] t = new double[n];
        for (int i = 0; i < n; i++) {
            t[i] = i * dt;
        }
        return t;
    }

    // ── ordinary least squares via normal equations (X^T X) beta = X^T y ──────

    static double[] solveOls(double[][] x, double[] y) {
        int n = x.length;
        int m = x[0].length;
        double[][] ata = new double[m][m];
        double[] atb = new double[m];
        for (int i = 0; i < n; i++) {
            double[] xi = x[i];
            for (int r = 0; r < m; r++) {
                atb[r] += xi[r] * y[i];
                for (int c = 0; c < m; c++) {
                    ata[r][c] += xi[r] * xi[c];
                }
            }
        }
        return gaussianSolve(ata, atb);
    }

    /** Solve a small dense linear system by Gaussian elimination with partial pivoting. */
    static double[] gaussianSolve(double[][] a, double[] b) {
        int m = b.length;
        for (int col = 0; col < m; col++) {
            int piv = col;
            for (int r = col + 1; r < m; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[piv][col])) {
                    piv = r;
                }
            }
            double[] tmp = a[col];
            a[col] = a[piv];
            a[piv] = tmp;
            double t = b[col];
            b[col] = b[piv];
            b[piv] = t;

            double diag = safeDiag(a[col][col]);
            for (int r = col + 1; r < m; r++) {
                double f = a[r][col] / diag;
                for (int c = col; c < m; c++) {
                    a[r][c] -= f * a[col][c];
                }
                b[r] -= f * b[col];
            }
        }
        double[] out = new double[m];
        for (int row = m - 1; row >= 0; row--) {
            double sum = b[row];
            for (int c = row + 1; c < m; c++) {
                sum -= a[row][c] * out[c];
            }
            out[row] = sum / safeDiag(a[row][row]);
        }
        return out;
    }

    private static double safeDiag(double d) {
        return Math.abs(d) < 1e-12 ? Math.copySign(1e-12, d == 0 ? 1 : d) : d;
    }

    private static double[] toArray(List<Double> list) {
        double[] a = new double[list.size()];
        for (int i = 0; i < a.length; i++) {
            a[i] = list.get(i);
        }
        return a;
    }

    static double rSquared(double[][] x, double[] y, double[] beta) {
        double mean = 0;
        for (double v : y) {
            mean += v;
        }
        mean /= y.length;
        double ssRes = 0;
        double ssTot = 0;
        for (int i = 0; i < y.length; i++) {
            double pred = 0;
            for (int j = 0; j < beta.length; j++) {
                pred += x[i][j] * beta[j];
            }
            ssRes += (y[i] - pred) * (y[i] - pred);
            ssTot += (y[i] - mean) * (y[i] - mean);
        }
        return ssTot < 1e-12 ? 0 : 1.0 - ssRes / ssTot;
    }
}
