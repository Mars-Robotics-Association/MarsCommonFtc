package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.sim.ArmMotorSim;
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim;
import org.marsroboticsassociation.controllib.sim.EncoderSim;

import java.util.ArrayList;
import java.util.List;

/**
 * Simulated system-identification for the arm. It drives a fresh plant built from the config with a
 * battery of known-voltage runs across the travel, logs the motor-side encoder, and recovers the arm
 * feedforward model
 *
 * <pre>
 *   V = kS*sign(w) + kV*w + kA*(dw/dt) + kCos*cos(theta) + kSin*sin(theta)
 * </pre>
 *
 * <p><b>Strategy: regress on the integrated equation of motion.</b> Rather than differentiating the
 * (quantized, windowed) encoder to estimate acceleration — the fragile step that wrecks a naive fit
 * — each run is cut into intervals {@code [t0, t1]} and the dynamics are integrated over each one:
 *
 * <pre>
 *   V*Δt = kS*(sign*Δt) + kV*Δθ + kA*Δw + kCos*∫cosθ dt + kSin*∫sinθ dt
 * </pre>
 *
 * Every term is either exact or a smoothing integral: {@code V*Δt} is exact (known applied voltage);
 * {@code ∫w dt = Δθ} comes <em>exactly</em> from encoder position; the gravity terms are trapezoidal
 * integrals of {@code cos/sin} of the measured angle; and the only boundary term, {@code Δw}, is the
 * velocity change (whose coefficient is the inertia {@code kA}, well excited by hard steps). One such
 * equation per interval, stacked and solved by ordinary least squares over all five unknowns.
 *
 * <p>Intervals are formed only where the velocity keeps a single sign and stays clear of the hard
 * stops, so {@code sign(w)} is constant and no contact force pollutes the samples.
 *
 * <p><b>Plant choice.</b> {@link #characterize(ArmPlantConfig)} uses a rigid {@link ArmMotorSim}.
 * {@link #characterize(ArmPlantConfig, boolean) characterize(cfg, true)} uses the two-inertia
 * {@link BacklashArmMotorSim} and logs its <em>motor-side</em> encoder — the realistic case for a
 * robot with an unavoidable lashy gearbox. Steady one-directional runs keep the teeth engaged, so
 * the motor encoder still sees the coupled inertia and gravity and the fit holds. Identified {@code
 * kA} is what the model-based controller needs when the true plant is heavier or lashier than the
 * hand-tuned model.
 */
public final class ArmSysId {

    private ArmSysId() {}

    /** Identified feedforward coefficients plus a fit-quality score. */
    public static final class Result {
        public final double kS, kV, kA, kCos, kSin, rSquared;
        public final int samples;

        Result(double kS, double kV, double kA, double kCos, double kSin, double rSquared, int samples) {
            this.kS = kS; this.kV = kV; this.kA = kA;
            this.kCos = kCos; this.kSin = kSin;
            this.rSquared = rSquared; this.samples = samples;
        }
    }

    private static final double DT = 0.01;           // 100 Hz logging
    private static final int RUN_STEPS = 90;         // 0.9 s per run
    private static final int VEL_HALF = 3;           // local-fit half window for endpoint velocity
    private static final double MIN_SPEED_RAD = 0.2; // sign(w) well-defined above this
    private static final double STOP_MARGIN_RAD = Math.toRadians(4);
    private static final int[] INTERVAL_LENS = {8, 16, 30}; // 0.08 / 0.16 / 0.30 s windows
    private static final int STRIDE = 4;
    private static final double HUB = ArmEngine.HUB_VOLTAGE;

    /** Run the characterization against a rigid plant built from {@code cfg}. */
    public static Result characterize(ArmPlantConfig cfg) {
        return characterize(cfg, false);
    }

    /**
     * Run the characterization against the plant built from {@code cfg}, optionally through the
     * two-inertia backlash plant (i.e. logging the motor-side encoder, exactly what a real robot with
     * an unavoidable lashy gearbox exposes) rather than a rigid plant.
     *
     * @param throughBacklash true to identify through the backlash plant's motor encoder
     */
    public static Result characterize(ArmPlantConfig cfg, boolean throughBacklash) {
        double ticksPerRad = cfg.ticksPerRad();
        double lo = cfg.minAngleRad, hi = cfg.maxAngleRad, span = hi - lo;
        double bottom = lo + 0.05 * span, top = hi - 0.05 * span, mid = lo + 0.55 * span;
        // Gravity peaks at horizontals (cos θ = ±1). On an over-the-top workspace those are 0 and
        // +π, not the span midpoint (often upright). Seed extra runs from every horizontal that
        // sits clear of the hard stops so kS/kCos stay identifiable.
        double margin = 0.08 * span;
        double[] gravityStarts = gravityRichStarts(lo, hi, margin, mid);

        List<double[]> rows = new ArrayList<>(); // [sign*Δt, Δθ, Δw, ∫cos, ∫sin]
        List<Double> rhs = new ArrayList<>();    // V*Δt

        // Moderate runs (rich angle sweep at near-terminal velocity) in both directions, plus hard
        // steps from rest whose onset makes Δw large (exciting kA), plus gravity-rich starts.
        for (double p : new double[]{0.45, 0.60, 0.80, 0.95}) runRun(cfg, ticksPerRad, bottom, p, throughBacklash, rows, rhs);
        for (double p : new double[]{-0.45, -0.60, -0.80, -0.95}) runRun(cfg, ticksPerRad, top, p, throughBacklash, rows, rhs);
        for (double start : gravityStarts) {
            runRun(cfg, ticksPerRad, start, 0.70, throughBacklash, rows, rhs);
            runRun(cfg, ticksPerRad, start, -0.70, throughBacklash, rows, rhs);
        }

        double[][] x = rows.toArray(new double[0][]);
        double[] y = toArray(rhs);
        double[] b = solveOls(x, y);
        double r2 = rSquared(x, y, b);
        return new Result(b[0], b[1], b[2], b[3], b[4], r2, y.length);
    }

    /** One constant-power run: log position, then stack one integrated-dynamics equation per interval. */
    private static void runRun(ArmPlantConfig cfg, double ticksPerRad, double startRad, double power,
                               boolean throughBacklash, List<double[]> rows, List<Double> rhs) {
        int n = RUN_STEPS;
        double[] theta = new double[n];
        if (throughBacklash) {
            BacklashArmMotorSim sim = new BacklashArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                    cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                    cfg.minAngleRad, cfg.maxAngleRad, startRad, cfg.backlashRad);
            sim.setEncoder(new EncoderSim()); // clean encoder: isolate the backlash effect
            for (int i = 0; i < n; i++) {
                sim.step(DT, power, HUB);
                theta[i] = sim.getPositionTicks() / ticksPerRad + cfg.encoderZeroOffsetRad;
            }
        } else {
            ArmMotorSim sim = new ArmMotorSim(cfg.kS, cfg.kG, cfg.kV, cfg.kA,
                    cfg.ticksPerRev, cfg.gearRatio, cfg.encoderZeroOffsetRad,
                    cfg.minAngleRad, cfg.maxAngleRad, startRad);
            sim.setEncoder(new EncoderSim());
            for (int i = 0; i < n; i++) {
                sim.step(DT, power, HUB);
                theta[i] = sim.getPositionTicks() / ticksPerRad + cfg.encoderZeroOffsetRad;
            }
        }
        // Endpoint velocities from a local linear/quadratic fit of the (clean, live) position signal.
        double[] w = new double[n];
        for (int i = 0; i < n; i++) w[i] = localVelocity(theta, i, n);

        double v = power * HUB;
        int firstValid = VEL_HALF, lastValid = n - 1 - VEL_HALF;
        for (int len : INTERVAL_LENS) {
            for (int s = firstValid; s + len <= lastValid; s += STRIDE) {
                int j = s + len;
                if (!intervalUsable(theta, w, s, j, cfg)) continue;
                double dtInterval = len * DT;
                double sign = Math.signum(w[(s + j) / 2]);
                double dTheta = theta[j] - theta[s];
                double dW = w[j] - w[s];
                double iCos = trapz(theta, s, j, true);
                double iSin = trapz(theta, s, j, false);
                rows.add(new double[]{sign * dtInterval, dTheta, dW, iCos, iSin});
                rhs.add(v * dtInterval);
            }
        }
    }

    /**
     * Start angles that excite gravity for OLS: horizontals in range (cos θ = ±1), plus the span
     * midpoint as a fallback when neither horizontal is available.
     */
    private static double[] gravityRichStarts(double lo, double hi, double margin, double mid) {
        List<Double> starts = new ArrayList<>();
        for (double h : new double[]{0.0, Math.PI, -Math.PI}) {
            if (h > lo + margin && h < hi - margin) {
                starts.add(h);
            }
        }
        if (starts.isEmpty()) {
            starts.add(mid);
        }
        double[] out = new double[starts.size()];
        for (int i = 0; i < starts.size(); i++) out[i] = starts.get(i);
        return out;
    }

    /** An interval is usable if velocity keeps one sign (above threshold) and clears the hard stops. */
    private static boolean intervalUsable(double[] theta, double[] w, int s, int j, ArmPlantConfig cfg) {
        double sign = Math.signum(w[(s + j) / 2]);
        for (int k = s; k <= j; k++) {
            if (Math.abs(w[k]) < MIN_SPEED_RAD || Math.signum(w[k]) != sign) return false;
            if (theta[k] < cfg.minAngleRad + STOP_MARGIN_RAD) return false;
            if (theta[k] > cfg.maxAngleRad - STOP_MARGIN_RAD) return false;
        }
        return true;
    }

    /** Trapezoidal integral of cos(theta) (or sin) over [s, j] with step DT. */
    private static double trapz(double[] theta, int s, int j, boolean cos) {
        double sum = 0;
        for (int k = s; k < j; k++) {
            double a = cos ? Math.cos(theta[k]) : Math.sin(theta[k]);
            double b = cos ? Math.cos(theta[k + 1]) : Math.sin(theta[k + 1]);
            sum += 0.5 * (a + b) * DT;
        }
        return sum;
    }

    /** Velocity at index c from a local quadratic fit of position (noise-tolerant, low-lag). */
    private static double localVelocity(double[] theta, int c, int n) {
        int half = Math.min(VEL_HALF, Math.min(c, n - 1 - c));
        if (half < 1) return 0.0;
        int m = 2 * half + 1;
        double[][] x = new double[m][3];
        double[] y = new double[m];
        for (int k = -half, r = 0; k <= half; k++, r++) {
            double t = k * DT;
            x[r][0] = 1.0; x[r][1] = t; x[r][2] = t * t;
            y[r] = theta[c + k];
        }
        return solveOls(x, y)[1]; // coefficient of t is the velocity at the center
    }

    // ── ordinary least squares via normal equations (X^T X) beta = X^T y ──────────

    private static double[] solveOls(double[][] x, double[] y) {
        int n = x.length, m = x[0].length;
        double[][] ata = new double[m][m];
        double[] atb = new double[m];
        for (int i = 0; i < n; i++) {
            double[] xi = x[i];
            for (int r = 0; r < m; r++) {
                atb[r] += xi[r] * y[i];
                for (int c = 0; c < m; c++) ata[r][c] += xi[r] * xi[c];
            }
        }
        return gaussianSolve(ata, atb);
    }

    /** Solve a small dense linear system by Gaussian elimination with partial pivoting. */
    private static double[] gaussianSolve(double[][] a, double[] b) {
        int m = b.length;
        for (int col = 0; col < m; col++) {
            int piv = col;
            for (int r = col + 1; r < m; r++) if (Math.abs(a[r][col]) > Math.abs(a[piv][col])) piv = r;
            double[] tmp = a[col]; a[col] = a[piv]; a[piv] = tmp;
            double t = b[col]; b[col] = b[piv]; b[piv] = t;

            double diag = safeDiag(a[col][col]);
            for (int r = col + 1; r < m; r++) {
                double f = a[r][col] / diag;
                for (int c = col; c < m; c++) a[r][c] -= f * a[col][c];
                b[r] -= f * b[col];
            }
        }
        double[] out = new double[m];
        for (int row = m - 1; row >= 0; row--) {
            double sum = b[row];
            for (int c = row + 1; c < m; c++) sum -= a[row][c] * out[c];
            out[row] = sum / safeDiag(a[row][row]);
        }
        return out;
    }

    private static double safeDiag(double d) {
        return Math.abs(d) < 1e-12 ? Math.copySign(1e-12, d == 0 ? 1 : d) : d;
    }

    private static double[] toArray(List<Double> list) {
        double[] a = new double[list.size()];
        for (int i = 0; i < a.length; i++) a[i] = list.get(i);
        return a;
    }

    private static double rSquared(double[][] x, double[] y, double[] beta) {
        double mean = 0;
        for (double v : y) mean += v;
        mean /= y.length;
        double ssRes = 0, ssTot = 0;
        for (int i = 0; i < y.length; i++) {
            double pred = 0;
            for (int j = 0; j < beta.length; j++) pred += x[i][j] * beta[j];
            ssRes += (y[i] - pred) * (y[i] - pred);
            ssTot += (y[i] - mean) * (y[i] - mean);
        }
        return ssTot < 1e-12 ? 0 : 1.0 - ssRes / ssTot;
    }
}
