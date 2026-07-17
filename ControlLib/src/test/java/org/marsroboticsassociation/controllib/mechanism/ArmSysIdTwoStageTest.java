package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Unit test of the two-stage solver's linear algebra, independent of any simulation: build hold and
 * moving-run rows directly from a known model and confirm {@link ArmSysId#solveTwoStage} recovers
 * every coefficient. Complements {@code ControlLab}'s ArmSysIdTest, which exercises the same path
 * end-to-end through the plant sims.
 */
class ArmSysIdTwoStageTest {

    private static final double KS = 0.30, KV = 1.20, KA = 0.35, KCOS = 3.50, KSIN = 0.40;

    /** V a quasi-static hold sample sees: kS·sign(w) + kV·w + gravity(θ). */
    private static double holdVoltage(double w, double theta) {
        return KS * Math.signum(w) + KV * w + KCOS * Math.cos(theta) + KSIN * Math.sin(theta);
    }

    @Test
    void solveTwoStageRecoversAllFiveFromCleanRows() {
        List<double[]> holdRows = new ArrayList<>();
        List<Double> holdRhs = new ArrayList<>();
        // Constant-velocity holds: several speeds, both directions, swept across a range of angles.
        for (double w : new double[] {-2.5, -1.5, -0.75, 0.75, 1.5, 2.5}) {
            for (double theta = -0.4; theta <= 2.6; theta += 0.05) {
                holdRows.add(new double[] {Math.signum(w), w, Math.cos(theta), Math.sin(theta), 0.0});
                holdRhs.add(holdVoltage(w, theta));
            }
        }

        // Moving-run rows are the integrated form [sign·Δt, Δθ, Δω, ∫cos, ∫sin] with rhs the model's
        // integrated voltage. Arbitrary but varied interval geometry, with Δω well excited so kA is
        // identifiable.
        List<double[]> movingRows = new ArrayList<>();
        List<Double> movingRhs = new ArrayList<>();
        double[][] geom = {
            {0.08, 0.20, 0.35, 0.079, 0.012},
            {0.16, 0.55, -0.20, 0.150, 0.030},
            {0.30, 1.10, 0.05, 0.280, 0.070},
            {0.08, 0.18, -0.40, 0.070, 0.020},
            {0.16, 0.40, 0.60, 0.120, 0.045},
            {0.30, 0.95, -0.15, 0.240, 0.090},
        };
        for (double[] g : geom) {
            double signDt = g[0], dTheta = g[1], dW = g[2], iCos = g[3], iSin = g[4];
            movingRows.add(new double[] {signDt, dTheta, dW, iCos, iSin});
            movingRhs.add(KS * signDt + KV * dTheta + KA * dW + KCOS * iCos + KSIN * iSin);
        }

        ArmSysId.Result r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs);
        assertEquals(KS, r.kS, 1e-6, "kS from holds");
        assertEquals(KV, r.kV, 1e-6, "kV from moving runs");
        assertEquals(KA, r.kA, 1e-6, "kA from moving runs");
        assertEquals(KCOS, r.kCos, 1e-6, "kCos from holds");
        assertEquals(KSIN, r.kSin, 1e-6, "kSin from holds");
    }

    @Test
    void accumulateHoldRecoversFrictionAndGravityUnderLoopJitter() {
        // Synthetic constant-velocity holds at several speeds/directions, logged with jittered
        // wall-clock timestamps (7–13 ms) like a real robot loop. The pooled OLS over the hold rows
        // must recover kS, kV, kCos, kSin despite the non-uniform dt.
        Random rng = new Random(7);
        List<double[]> rows = new ArrayList<>();
        List<Double> rhs = new ArrayList<>();
        for (double w : new double[] {-2.5, -1.5, -0.75, 0.75, 1.5, 2.5}) {
            int n = 260;
            double[] theta = new double[n], volt = new double[n], time = new double[n];
            double t = 0.0;
            double th = w > 0 ? -0.4 : 3.0; // sweep across the [-1, 3.6] window
            for (int i = 0; i < n; i++) {
                double dt = 0.010 * (0.7 + 0.6 * rng.nextDouble());
                t += dt;
                th += w * dt;
                time[i] = t;
                theta[i] = th;
                volt[i] = KS * Math.signum(w) + KV * w + KCOS * Math.cos(th) + KSIN * Math.sin(th);
            }
            ArmSysId.accumulateHold(theta, volt, time, -1.0, 3.6, ArmSysId.DEFAULT_PARAMS, rows, rhs);
        }
        assertTrue(rows.size() > 100, "should keep many steady samples, got " + rows.size());

        // Hold rows are [sign(ω), ω, cosθ, sinθ, α]; OLS recovers [kS, kV, kCos, kSin, ~0].
        double[] b = ArmSysId.solveOls(rows.toArray(new double[0][]), toPrimitive(rhs));
        assertEquals(KS, b[0], 0.02, "kS");
        assertEquals(KV, b[1], 0.02, "kV");
        assertEquals(KCOS, b[2], 0.03, "kCos");
        assertEquals(KSIN, b[3], 0.03, "kSin");
    }

    private static double[] toPrimitive(List<Double> list) {
        double[] a = new double[list.size()];
        for (int i = 0; i < a.length; i++) a[i] = list.get(i);
        return a;
    }

    @Test
    void fallsBackToSingleStageWhenNoHolds() {
        // Too few hold rows: solveTwoStage must fall back to solve() on the moving rows alone.
        List<double[]> movingRows = new ArrayList<>();
        List<Double> movingRhs = new ArrayList<>();
        double[][] geom = {
            {0.08, 0.20, 0.35, 0.079, 0.012}, {0.16, 0.55, -0.20, 0.150, 0.030},
            {0.30, 1.10, 0.05, 0.280, 0.070}, {0.08, 0.18, -0.40, 0.070, 0.020},
            {0.16, 0.40, 0.60, 0.120, 0.045}, {0.30, 0.95, -0.15, 0.240, 0.090},
        };
        for (double[] g : geom) {
            movingRows.add(new double[] {g[0], g[1], g[2], g[3], g[4]});
            movingRhs.add(KS * g[0] + KV * g[1] + KA * g[2] + KCOS * g[3] + KSIN * g[4]);
        }
        ArmSysId.Result r = ArmSysId.solveTwoStage(
                new ArrayList<>(), new ArrayList<>(), movingRows, movingRhs);
        assertTrue(r.samples > 0, "fallback should still solve from the moving rows");
        assertEquals(KA, r.kA, 1e-6, "single-stage fallback recovers kA");
    }
}
