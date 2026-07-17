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

    /** A hold row in {@link ArmSysId#accumulateHold}'s layout for a steady sample. */
    private static double[] holdRow(double w, double theta) {
        double sign = Math.signum(w);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);
        double faceSign = Math.signum(cos);
        return new double[] {
            sign, w, cos, sin, sign * cos, sign * sin, faceSign * cos, faceSign * sin, 0.0
        };
    }

    @Test
    void solveTwoStageRecoversAllFiveFromCleanRows() {
        List<double[]> holdRows = new ArrayList<>();
        List<Double> holdRhs = new ArrayList<>();
        // Constant-velocity holds: several speeds, both directions, swept across a range of angles.
        for (double w : new double[] {-2.5, -1.5, -0.75, 0.75, 1.5, 2.5}) {
            for (double theta = -0.4; theta <= 2.6; theta += 0.05) {
                holdRows.add(holdRow(w, theta));
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
        assertEquals(KV, r.kV, 1e-6, "kV (hold-side ships)");
        assertEquals(KA, r.kA, 1e-6, "kA from moving runs");
        assertEquals(KCOS, r.kCos, 1e-6, "kCos from holds");
        assertEquals(KSIN, r.kSin, 1e-6, "kSin from holds");
        assertEquals(KV, r.kVHold, 1e-6, "hold-side kV pinned by the speed spread");
        assertEquals(KV, r.kVRun, 1e-6, "run-side cross-check agrees on clean data");
        // Lash-free holds: the split columns either get dropped by the residual gate (NaN) or fit
        // an offset of ~0 — both mean "no lash detected".
        assertTrue(Double.isNaN(r.halfLashRad) || Math.abs(r.halfLashRad) < 1e-6,
                "no lash in the synthetic holds, got " + r.halfLashRad);
        assertTrue(r.kVDisagreement() < 1e-6, "no flex flag on clean data");
    }

    @Test
    void solveTwoStageAbsorbsBacklashOffsetIntoHalfLashNotKs() {
        // Motor-side holds through backlash: gravity acts at the arm angle, offset ±h from the
        // encoder with travel direction. Without the direction-split gravity columns this offset
        // is collinear with sign(ω) and inflates kS; with them it must come back out as h.
        double h = Math.toRadians(2.5);
        List<double[]> holdRows = new ArrayList<>();
        List<Double> holdRhs = new ArrayList<>();
        for (double w : new double[] {-2.5, -1.5, -0.75, 0.75, 1.5, 2.5}) {
            for (double theta = -0.4; theta <= 2.6; theta += 0.05) {
                holdRows.add(holdRow(w, theta));
                double armTheta = theta - Math.signum(w) * h;
                holdRhs.add(KS * Math.signum(w) + KV * w
                        + KCOS * Math.cos(armTheta) + KSIN * Math.sin(armTheta));
            }
        }
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

        ArmSysId.Result r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs);
        assertEquals(KS, r.kS, 0.02, "kS must not absorb the lash offset");
        assertEquals(h, r.halfLashRad, Math.toRadians(0.3), "half-lash recovered from the holds");
        assertEquals(KCOS, r.kCos, 0.03, "kCos survives the lash offset");
        assertEquals(KV, r.kV, 0.02, "hold-side kV survives the lash offset");
    }

    @Test
    void solveTwoStageAbsorbsGravityFaceOffsetIntoHalfLashNotKs() {
        // A gravity-dominated pure-lash drivetrain: the loaded tooth face follows the sign of the
        // gravity torque, so the motor-vs-arm offset flips at the horizontals (sign(cosθ)), not
        // with travel direction. The gravity-face column family must absorb it and report h.
        double h = Math.toRadians(6.0);
        List<double[]> holdRows = new ArrayList<>();
        List<Double> holdRhs = new ArrayList<>();
        for (double w : new double[] {-2.5, -1.5, -0.75, 0.75, 1.5, 2.5}) {
            for (double theta = -0.4; theta <= 2.6; theta += 0.05) {
                holdRows.add(holdRow(w, theta));
                double armTheta = theta - Math.signum(Math.cos(theta)) * h;
                holdRhs.add(KS * Math.signum(w) + KV * w
                        + KCOS * Math.cos(armTheta) + KSIN * Math.sin(armTheta));
            }
        }
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

        ArmSysId.Result r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs);
        assertEquals(KS, r.kS, 0.03, "kS must not absorb the gravity-face offset");
        assertEquals(KV, r.kV, 0.02, "hold-side kV survives the gravity-face offset");
        assertEquals(h, r.halfLashRad, Math.toRadians(0.8),
                "half-lash recovered from the gravity-face columns");
    }

    @Test
    void narrowSpeedHoldsFallBackToRunSideKv() {
        // Holds bunched at nearly one speed magnitude: sign(ω) and ω are close to collinear, so
        // the hold-side kV is untrustworthy and the run-side estimate must ship instead.
        List<double[]> holdRows = new ArrayList<>();
        List<Double> holdRhs = new ArrayList<>();
        for (double w : new double[] {-1.6, -1.4, 1.4, 1.6}) {
            for (double theta = -0.4; theta <= 2.6; theta += 0.05) {
                holdRows.add(holdRow(w, theta));
                holdRhs.add(holdVoltage(w, theta));
            }
        }
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

        ArmSysId.Result r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs);
        assertTrue(Double.isNaN(r.kVHold), "hold-side kV unavailable without speed spread");
        assertEquals(KV, r.kVRun, 0.05, "run-side kV ships");
        assertEquals(KV, r.kV, 0.05, "shipped kV falls back to the run side");
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

        // Hold rows are [sign(ω), ω, cosθ, sinθ, then the split-column families and α]; OLS
        // recovers [kS, kV, kCos, kSin, ~0...].
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

    @Test
    void estimateFlexPeriodFindsARingDown() {
        // 3 Hz decaying ring around a rest angle, sampled at a jittery ~100 Hz robot loop.
        Random rng = new Random(3);
        int n = 250;
        double[] theta = new double[n];
        double[] time = new double[n];
        double t = 0.0;
        for (int i = 0; i < n; i++) {
            t += 0.010 * (0.8 + 0.4 * rng.nextDouble());
            time[i] = t;
            theta[i] = 1.2 + 0.05 * Math.exp(-0.6 * t) * Math.sin(2 * Math.PI * 3.0 * t);
        }
        double period = ArmSysId.estimateFlexPeriod(theta, time);
        assertEquals(1.0 / 3.0, period, 0.02, "3 Hz ring-down period");
    }

    @Test
    void estimateFlexPeriodRejectsAQuietLog() {
        int n = 100;
        double[] theta = new double[n];
        double[] time = new double[n];
        for (int i = 0; i < n; i++) {
            time[i] = i * 0.01;
            theta[i] = 1.2 + 1e-5 * Math.sin(i); // sub-noise wiggle
        }
        assertEquals(0.0, ArmSysId.estimateFlexPeriod(theta, time), "no ring to measure");
    }

    @Test
    void flexPeriodIntervalsCancelTheRingOutOfKa() {
        // A constant-voltage run whose position log carries an undamped-ish 3 Hz ring on top of
        // the rigid response. Whole-period intervals must beat the fixed-length ones on kA.
        double flexHz = 3.0;
        ArmSysId.FitParams plain = new ArmSysId.FitParams();
        ArmSysId.FitParams gated = new ArmSysId.FitParams();
        gated.flexPeriodSec = 1.0 / flexHz;

        double kaPlain = fitKaFromRingingRun(plain);
        double kaGated = fitKaFromRingingRun(gated);
        assertTrue(Math.abs(kaGated - KA) < Math.abs(kaPlain - KA),
                String.format("gated kA %.4f should beat plain kA %.4f (true %.2f)",
                        kaGated, kaPlain, KA));
        assertEquals(KA, kaGated, KA * 0.10, "gated kA within 10%");
    }

    /** Accumulate one synthetic ringing run with {@code params} and solve; return the fitted kA. */
    private static double fitKaFromRingingRun(ArmSysId.FitParams params) {
        int n = 200;
        double dt = 0.01;
        double volts = 6.0;
        double[] theta = new double[n];
        double[] voltage = new double[n];
        double[] time = new double[n];
        // Rigid trajectory obeying the full model, integrated forward, plus a 3 Hz flex ring the
        // motor encoder sees.
        double pos = 0.0;
        double vel = 0.0;
        for (int i = 0; i < n; i++) {
            double t = i * dt;
            double accel = (volts - KS - KV * vel
                    - KCOS * Math.cos(pos) - KSIN * Math.sin(pos)) / KA;
            vel += accel * dt;
            pos += vel * dt;
            time[i] = t;
            voltage[i] = volts;
            theta[i] = pos + 0.015 * Math.exp(-0.5 * t) * Math.sin(2 * Math.PI * 3.0 * t);
        }
        List<double[]> rows = new ArrayList<>();
        List<Double> rhs = new ArrayList<>();
        ArmSysId.accumulateRun(theta, voltage, time, -100.0, 100.0, params, rows, rhs);
        assertTrue(rows.size() >= 3, "ringing run should still yield rows, got " + rows.size());
        // Stage-2 style: everything but kA fixed at truth, so the fit isolates the Δω boundary
        // term — exactly what the whole-period intervals are supposed to clean of the ring.
        double num = 0.0;
        double den = 0.0;
        for (int i = 0; i < rows.size(); i++) {
            double[] xi = rows.get(i); // [sign·Δt, Δθ, Δω, ∫cos, ∫sin]
            double residual = rhs.get(i)
                    - KS * xi[0] - KV * xi[1] - KCOS * xi[3] - KSIN * xi[4];
            num += xi[2] * residual;
            den += xi[2] * xi[2];
        }
        return num / den;
    }
}
