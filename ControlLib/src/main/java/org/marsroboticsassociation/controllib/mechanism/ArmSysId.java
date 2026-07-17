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
 * <p><b>Through a lashy/flexy drivetrain</b>, prefer the two-stage path: friction, viscous drag,
 * gravity, and the backlash half-angle from constant-velocity holds ({@link #accumulateHold}) plus
 * inertia from the moving runs, combined by {@link #solveTwoStage}. A hard-accelerating run through
 * the lash and flex inflates {@code kS} — the direction-flipping flex/lash deflection is collinear
 * with the {@code sign(ω)} regressor — and that over-estimate becomes an anti-braking feedforward
 * term that reintroduces arrival overshoot. Holding a steady speed keeps {@code α ≈ 0}, starving
 * that bias; direction-split gravity columns absorb the ±half-lash encoder offset the holds still
 * see. Same logging (angle + voltage) as {@link #accumulateRun}; the caller just drives a held
 * speed instead of a constant voltage. When the structural flex period is known (config, or
 * {@link #estimateFlexPeriod} on a ring-down log), set {@link FitParams#flexPeriodSec} so the
 * moving-run intervals span whole flex periods and the ring cancels out of the {@code kA} fit.
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
        /**
         * Hold-stage {@code kV} (quasi-static, immune to flex/lash transients). This is the shipped
         * {@link #kV} when the holds cover enough of a speed range to pin it; {@code NaN} when they
         * do not.
         */
        public final double kVHold;
        /**
         * Run-stage {@code kV} from the two-parameter {@code (Δθ, Δω)} fit over the moving runs.
         * Kept as a cross-check even when the hold-stage value ships; {@code NaN} on the
         * no-holds fallback path only when the moving fit itself failed.
         */
        public final double kVRun;
        /**
         * Backlash half-angle (rad) implied by the split gravity regressors of the hold fit:
         * gravity acts at the arm angle, offset from the motor encoder by ±half-lash on whichever
         * tooth face carries the load, and that offset shows up as an {@code offsetSign·sinθ}
         * voltage term of size {@code kG·halfLash}. The estimate sums the direction-selected and
         * gravity-selected contributions, so it tracks the mechanical lash regardless of which
         * regime dominates the sweep. {@code NaN} when neither family was identifiable (holds all
         * one direction and never crossing a horizontal).
         */
        public final double halfLashRad;

        public Result(
                double kS,
                double kV,
                double kA,
                double kCos,
                double kSin,
                double rSquared,
                int samples) {
            this(kS, kV, kA, kCos, kSin, rSquared, samples, Double.NaN, kV, Double.NaN);
        }

        public Result(
                double kS,
                double kV,
                double kA,
                double kCos,
                double kSin,
                double rSquared,
                int samples,
                double kVHold,
                double kVRun,
                double halfLashRad) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kCos = kCos;
            this.kSin = kSin;
            this.rSquared = rSquared;
            this.samples = samples;
            this.kVHold = kVHold;
            this.kVRun = kVRun;
            this.halfLashRad = halfLashRad;
        }

        /**
         * Relative disagreement {@code |kVHold − kVRun| / kV} between the quasi-static and moving
         * estimates of {@code kV}. The two agree on a rigid drivetrain; a gap beyond ~10% flags
         * flex/lash contamination of the moving runs (the hold-side value is the trustworthy one).
         * {@code NaN} when either estimate is unavailable.
         */
        public double kVDisagreement() {
            return Math.abs(kVHold - kVRun) / Math.max(Math.abs(kV), 1e-9);
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
        /**
         * Structural-flex fundamental period (seconds); 0 disables flex-aware interval selection.
         * When set, {@link #accumulateRun} spans each integration interval over a whole number of
         * flex periods (1–3), so the near-undamped ring contributes equal phase at both interval
         * boundaries and its {@code Δθ}/{@code Δω} contamination largely cancels. Known from the
         * mechanism (ControlLab's plant config) or measured with {@link #estimateFlexPeriod}.
         */
        public double flexPeriodSec = 0.0;
        /**
         * With {@link #flexPeriodSec} set, additionally skip samples within this many seconds of a
         * run's start — the voltage step that rings the flex hardest. 0 keeps every sample; runs
         * are short, so large values starve the {@code kA} fit of its acceleration onset.
         */
        public double flexSettleSec = 0.0;
        /**
         * Minimum spread of hold speeds ({@code max|ω| − min|ω|}, rad/s) for the hold-stage
         * {@code kV} to be trusted as the shipped value. Below this the {@code sign(ω)} and
         * {@code ω} regressors are nearly collinear (e.g. every hold at one speed), so {@code kV}
         * falls back to the moving-run estimate.
         */
        public double minHoldKvSpeedSpread = 0.5;
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
        double[] volts = new double[n];
        java.util.Arrays.fill(volts, voltage);
        accumulateRun(thetaRad, volts, dt, minAngleRad, maxAngleRad, params, rows, rhs);
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
        if (params.flexPeriodSec > 0) {
            // Flex-aware intervals: span whole flex periods so the near-undamped structural ring
            // contributes equal phase at both boundaries and cancels out of Δθ/Δω, and optionally
            // skip the hard-ringing samples right after the voltage step.
            while (firstValid <= lastValid
                    && timeSec[firstValid] - timeSec[0] < params.flexSettleSec) {
                firstValid++;
            }
            for (int periods = 1; periods <= 3; periods++) {
                double span = periods * params.flexPeriodSec;
                int j = firstValid;
                for (int s = firstValid; s <= lastValid; s += params.stride) {
                    if (j < s) {
                        j = s;
                    }
                    while (j < lastValid && timeSec[j] - timeSec[s] < span) {
                        j++;
                    }
                    // Nearest sample to a whole-period span (j is the first at/after it).
                    if (j > s + 1
                            && (timeSec[j] - timeSec[s] - span)
                                    > (span - (timeSec[j - 1] - timeSec[s]))) {
                        j--;
                    }
                    if (timeSec[j] - timeSec[s] < 0.5 * span) {
                        break; // the log ends before this period count fits
                    }
                    emitRunInterval(thetaRad, voltage, timeSec, w, s, j,
                            minAngleRad, maxAngleRad, params, rows, rhs);
                }
            }
            return;
        }
        for (int len : params.intervalLens) {
            for (int s = firstValid; s + len <= lastValid; s += params.stride) {
                emitRunInterval(thetaRad, voltage, timeSec, w, s, s + len,
                        minAngleRad, maxAngleRad, params, rows, rhs);
            }
        }
    }

    /** Append one integrated-dynamics row for the interval [s, j] if it is usable. */
    private static void emitRunInterval(
            double[] thetaRad,
            double[] voltage,
            double[] timeSec,
            double[] w,
            int s,
            int j,
            double minAngleRad,
            double maxAngleRad,
            FitParams params,
            List<double[]> rows,
            List<Double> rhs) {
        if (j <= s + 1 || !intervalUsable(thetaRad, w, s, j, minAngleRad, maxAngleRad, params)) {
            return;
        }
        double dtInterval = timeSec[j] - timeSec[s];
        if (dtInterval < 1e-6) {
            return;
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
     * clear of the stops) are kept; each becomes a row {@code [sign(ω), ω, cosθ, sinθ, sign(ω)·cosθ,
     * sign(ω)·sinθ, sign(cosθ)·cosθ, sign(cosθ)·sinθ, α]} with the applied voltage as its
     * right-hand side. The two split-column families absorb the backlash geometry a motor-side
     * encoder cannot see: gravity acts at the arm angle, offset by ±half-lash on whichever tooth
     * face carries the load — a face that flips with <i>travel direction</i> when drivetrain
     * wind-up (friction/flex) dominates, and with the <i>sign of the gravity torque</i> (at the
     * horizontals) when gravity dominates. Either offset linearizes to a
     * {@code kG·halfLash·offsetSign·sinθ} voltage term that would otherwise be dumped into
     * {@code kS} or gravity. Feed the accumulated rows to {@link #solveTwoStage}, which also turns
     * those columns into a half-lash estimate ({@link Result#halfLashRad}).
     *
     * @param thetaRad arm angles over the hold (rad)
     * @param voltage  per-sample applied voltage sustaining the hold (volts)
     * @param timeSec  monotonically increasing sample times (seconds); any zero origin is fine
     * @param rows     feature rows {@code [sign(ω), ω, cosθ, sinθ, sign·cosθ, sign·sinθ, α]}
     *                 (appended)
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
            double sign = Math.signum(w[i]);
            double cos = Math.cos(thetaRad[i]);
            double sin = Math.sin(thetaRad[i]);
            double faceSign = Math.signum(cos); // contact face under a gravity-dominated load
            rows.add(new double[] {
                sign, w[i], cos, sin,
                sign * cos, sign * sin,
                faceSign * cos, faceSign * sin,
                accel
            });
            rhs.add(voltage[i]);
        }
    }

    /**
     * Two-stage fit that splits each parameter to the data that identifies it cleanly.
     *
     * <p><b>Stage 1 — {@code kS}, {@code kV}, gravity, and half-lash from the holds.</b> An OLS over
     * the {@link #accumulateHold} rows yields {@code kS, kV, kCos, kSin} plus the split-column
     * gravity offsets. These are the terms a hard-accelerating run through the lash + flex
     * corrupts: the direction-flipping friction/lash deflection is collinear with the {@code
     * sign(ω)} regressor, so it lands in {@code kS} and biases gravity. The quasi-static holds
     * (α ≈ 0) starve that bias, and the two split-column families absorb the ±half-lash offset
     * between the motor encoder and the arm — flipping with travel direction (wind-up-dominated
     * contact) or with the gravity torque at the horizontals (gravity-dominated contact) — so it
     * cannot leak into {@code kS} either; instead it comes back out as a backlash half-angle
     * estimate ({@link Result#halfLashRad}). The hold-side {@code kV} is quasi-static and therefore
     * flex-immune; it ships as the model's {@code kV} whenever the holds span enough speeds
     * ({@link FitParams#minHoldKvSpeedSpread}). Each family is fit only where it is identifiable:
     * the direction family needs holds in both travel directions, the gravity-face family needs
     * the sweep to cross a horizontal.
     *
     * <p><b>Stage 2 — {@code kA} from the moving runs.</b> With {@code kS}, {@code kV}, gravity, and
     * the lash offset subtracted, each {@link #accumulateRun} row reduces to {@code kA·Δω} — a
     * single-parameter fit with nothing left to trade error against. (When the holds could not pin
     * {@code kV}, stage 2 falls back to the two-parameter {@code (Δθ, Δω)} fit.) The two-parameter
     * run-side {@code kV} is always computed as a cross-check and reported as
     * {@link Result#kVRun}; a large {@link Result#kVDisagreement()} flags flex/lash contamination
     * of the runs. The reported {@code rSquared} is over the moving rows.
     *
     * <p>Falls back to a plain {@link #solve} on the moving rows if there are too few hold rows.
     *
     * @param params     fit hyper-parameters (thresholds for trusting the hold-side {@code kV})
     * @param holdRows   rows from {@link #accumulateHold}
     * @param holdRhs    their right-hand sides
     * @param movingRows rows from {@link #accumulateRun}
     * @param movingRhs  their right-hand sides
     */
    public static Result solveTwoStage(
            FitParams params,
            List<double[]> holdRows,
            List<Double> holdRhs,
            List<double[]> movingRows,
            List<Double> movingRhs) {
        if (holdRows.size() < 5 || movingRows.size() < 2) {
            return solve(movingRows, movingRhs);
        }

        // Coverage decides which stage-1 offset columns are identifiable: the wind-up family needs
        // both travel directions (sign(ω) constant otherwise), the gravity-face family needs the
        // sweep to cross a horizontal (sign(cosθ) constant otherwise).
        int posRows = 0;
        int negRows = 0;
        int posCos = 0;
        int negCos = 0;
        double minSpeed = Double.POSITIVE_INFINITY;
        double maxSpeed = 0.0;
        for (double[] row : holdRows) {
            if (row[0] > 0) {
                posRows++;
            } else if (row[0] < 0) {
                negRows++;
            }
            if (row[2] > 0.05) {
                posCos++;
            } else if (row[2] < -0.05) {
                negCos++;
            }
            double speed = Math.abs(row[1]);
            minSpeed = Math.min(minSpeed, speed);
            maxSpeed = Math.max(maxSpeed, speed);
        }
        boolean bothDirections = posRows >= 3 && negRows >= 3;
        boolean crossesHorizontal = posCos >= 3 && negCos >= 3;
        boolean speedSpreadOk = maxSpeed - minSpeed >= params.minHoldKvSpeedSpread;

        // Stage-1 design matrix: base columns always, each offset family only when identifiable.
        List<Integer> cols = new ArrayList<>(java.util.Arrays.asList(0, 1, 2, 3));
        if (bothDirections) {
            cols.add(4);
            cols.add(5);
        }
        if (crossesHorizontal) {
            cols.add(6);
            cols.add(7);
        }
        cols.add(8); // α nuisance column
        double[] y1 = toArray(holdRhs);
        double[][] x1 = new double[holdRows.size()][cols.size()];
        for (int i = 0; i < holdRows.size(); i++) {
            double[] xi = holdRows.get(i);
            for (int c = 0; c < cols.size(); c++) {
                x1[i][c] = xi[cols.get(c)];
            }
        }
        double[] b = solveOls(x1, y1);
        double kS = b[0];
        double stage1Kv = b[1];
        double kCos = b[2];
        double kSin = b[3];
        // Gravity at an offset arm angle linearizes to offsetSign·h·(kCos·sinθ − kSin·cosθ), so
        // each family's fitted [·cos, ·sin] pair projects back to a half-lash contribution; their
        // sum estimates the mechanical half-lash however the contact face happens to be selected
        // across the sweep.
        double halfLash = Double.NaN;
        double kG2 = kCos * kCos + kSin * kSin;
        if (kG2 > 1e-6 && cols.size() > 5) {
            halfLash = 0.0;
            int next = 4;
            if (bothDirections) {
                halfLash += (b[next + 1] * kCos - b[next] * kSin) / kG2;
                next += 2;
            }
            if (crossesHorizontal) {
                halfLash += (b[next + 1] * kCos - b[next] * kSin) / kG2;
            }
        }
        double kVHold = speedSpreadOk ? stage1Kv : Double.NaN;

        // Moving-run residuals with the hold-identified terms (and lash offset) removed.
        double[][] x2 = new double[movingRows.size()][2]; // [Δθ, Δω]
        double[] y2 = new double[movingRows.size()];
        for (int i = 0; i < movingRows.size(); i++) {
            double[] xi = movingRows.get(i); // [sign·Δt, Δθ, Δω, ∫cos, ∫sin]
            x2[i][0] = xi[1];
            x2[i][1] = xi[2];
            double y = movingRhs.get(i) - kS * xi[0] - kCos * xi[3] - kSin * xi[4];
            if (!Double.isNaN(halfLash)) {
                // ∫kG·cos(θ − sign·h) ≈ kG·∫cos + sign·h·(kCos·∫sin − kSin·∫cos)
                y -= Math.signum(xi[0]) * halfLash * (kCos * xi[4] - kSin * xi[3]);
            }
            y2[i] = y;
        }
        // Run-side two-parameter fit: the shipped path when holds cannot pin kV, and the
        // cross-check (kVRun) that flags flex contamination when they can.
        double[] s2 = solveOls(x2, y2);
        double kVRun = s2[0];

        double kV;
        double kA;
        if (!Double.isNaN(kVHold)) {
            kV = kVHold;
            double num = 0.0;
            double den = 0.0;
            for (int i = 0; i < y2.length; i++) {
                double residual = y2[i] - kV * x2[i][0];
                num += x2[i][1] * residual;
                den += x2[i][1] * x2[i][1];
            }
            kA = den > 1e-12 ? num / den : s2[1];
        } else {
            kV = kVRun;
            kA = s2[1];
        }

        double r2 = rSquared(movingRows.toArray(new double[0][]), toArray(movingRhs),
                new double[] {kS, kV, kA, kCos, kSin});
        return new Result(kS, kV, kA, kCos, kSin, r2,
                holdRows.size() + movingRows.size(), kVHold, kVRun, halfLash);
    }

    /**
     * {@link #solveTwoStage(FitParams, List, List, List, List)} with {@link #DEFAULT_PARAMS}
     * thresholds.
     */
    public static Result solveTwoStage(
            List<double[]> holdRows,
            List<Double> holdRhs,
            List<double[]> movingRows,
            List<Double> movingRhs) {
        return solveTwoStage(DEFAULT_PARAMS, holdRows, holdRhs, movingRows, movingRhs);
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

    /**
     * Estimate the structural-flex period from a ring-down log: park the arm, flick or step-release
     * it, and log the free oscillation. The angle series is detrended with a moving-average
     * baseline; the period is the mean spacing of same-direction zero crossings of the residual.
     *
     * <p>Feed the result to {@link FitParams#flexPeriodSec}. Works from either encoder side — on a
     * motor-side encoder the ring couples through the drivetrain attenuated but at the same
     * frequency.
     *
     * @param thetaRad logged angles (rad) covering at least a few oscillations
     * @param timeSec  monotonically increasing sample times (seconds)
     * @return the estimated period in seconds, or 0 when no consistent oscillation is present
     *     (too few crossings, sub-noise amplitude, or irregular spacing)
     */
    public static double estimateFlexPeriod(double[] thetaRad, double[] timeSec) {
        int n = thetaRad.length;
        if (n < 16 || timeSec.length != n) {
            return 0.0;
        }
        // Moving-average baseline: wide enough to pass the ring through into the residual.
        int half = Math.max(2, n / 16);
        double[] residual = new double[n];
        double maxAbs = 0.0;
        for (int i = 0; i < n; i++) {
            int lo = Math.max(0, i - half);
            int hi = Math.min(n - 1, i + half);
            double mean = 0.0;
            for (int k = lo; k <= hi; k++) {
                mean += thetaRad[k];
            }
            mean /= (hi - lo + 1);
            residual[i] = thetaRad[i] - mean;
            maxAbs = Math.max(maxAbs, Math.abs(residual[i]));
        }
        if (maxAbs < 1e-4) {
            return 0.0; // sub-noise: no ring to measure
        }
        // Rising zero crossings of the residual, linearly interpolated in time.
        List<Double> crossings = new ArrayList<>();
        for (int i = 1; i < n; i++) {
            if (residual[i - 1] < 0 && residual[i] >= 0) {
                double frac = residual[i - 1] / (residual[i - 1] - residual[i]);
                crossings.add(timeSec[i - 1] + frac * (timeSec[i] - timeSec[i - 1]));
            }
        }
        if (crossings.size() < 3) {
            return 0.0;
        }
        double meanGap = (crossings.get(crossings.size() - 1) - crossings.get(0))
                / (crossings.size() - 1);
        double var = 0.0;
        for (int i = 1; i < crossings.size(); i++) {
            double gap = crossings.get(i) - crossings.get(i - 1);
            var += (gap - meanGap) * (gap - meanGap);
        }
        double sd = Math.sqrt(var / (crossings.size() - 1));
        return sd < 0.35 * meanGap ? meanGap : 0.0;
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
