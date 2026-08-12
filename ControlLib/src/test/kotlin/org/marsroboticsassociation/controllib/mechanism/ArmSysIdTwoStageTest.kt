package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.sign
import kotlin.math.sin
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Unit test of the two-stage solver's linear algebra, independent of any simulation: build hold and
 * moving-run rows directly from a known model and confirm [ArmSysId.solveTwoStage] recovers every
 * coefficient. Complements `ControlLab`'s ArmSysIdTest, which exercises the same path end-to-end
 * through the plant sims.
 */
class ArmSysIdTwoStageTest {

    /** V a quasi-static hold sample sees: kS·sign(w) + kV·w + gravity(θ). */
    private fun holdVoltage(w: Double, theta: Double): Double =
        KS * sign(w) + KV * w + KCOS * cos(theta) + KSIN * sin(theta)

    /** A hold row in [ArmSysId.accumulateHold]'s layout for a steady sample. */
    private fun holdRow(w: Double, theta: Double): DoubleArray {
        val signW = sign(w)
        val cosT = cos(theta)
        val sinT = sin(theta)
        val faceSign = sign(cosT)
        return doubleArrayOf(
            signW,
            w,
            cosT,
            sinT,
            signW * cosT,
            signW * sinT,
            faceSign * cosT,
            faceSign * sinT,
            0.0,
        )
    }

    @Test
    fun solveTwoStageRecoversAllFiveFromCleanRows() {
        val holdRows = ArrayList<DoubleArray>()
        val holdRhs = ArrayList<Double>()
        // Constant-velocity holds: several speeds, both directions, swept across a range of angles.
        for (w in doubleArrayOf(-2.5, -1.5, -0.75, 0.75, 1.5, 2.5)) {
            var theta = -0.4
            while (theta <= 2.6) {
                holdRows.add(holdRow(w, theta))
                holdRhs.add(holdVoltage(w, theta))
                theta += 0.05
            }
        }

        // Moving-run rows are the integrated form [sign·Δt, Δθ, Δω, ∫cos, ∫sin] with rhs the
        // model's
        // integrated voltage. Arbitrary but varied interval geometry, with Δω well excited so kA is
        // identifiable.
        val movingRows = ArrayList<DoubleArray>()
        val movingRhs = ArrayList<Double>()
        val geom =
            arrayOf(
                doubleArrayOf(0.08, 0.20, 0.35, 0.079, 0.012),
                doubleArrayOf(0.16, 0.55, -0.20, 0.150, 0.030),
                doubleArrayOf(0.30, 1.10, 0.05, 0.280, 0.070),
                doubleArrayOf(0.08, 0.18, -0.40, 0.070, 0.020),
                doubleArrayOf(0.16, 0.40, 0.60, 0.120, 0.045),
                doubleArrayOf(0.30, 0.95, -0.15, 0.240, 0.090),
            )
        for (g in geom) {
            val signDt = g[0]
            val dTheta = g[1]
            val dW = g[2]
            val iCos = g[3]
            val iSin = g[4]
            movingRows.add(doubleArrayOf(signDt, dTheta, dW, iCos, iSin))
            movingRhs.add(KS * signDt + KV * dTheta + KA * dW + KCOS * iCos + KSIN * iSin)
        }

        val r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs)
        assertEquals(KS, r.kS, 1e-6, "kS from holds")
        assertEquals(KV, r.kV, 1e-6, "kV (hold-side ships)")
        assertEquals(KA, r.kA, 1e-6, "kA from moving runs")
        assertEquals(KCOS, r.kCos, 1e-6, "kCos from holds")
        assertEquals(KSIN, r.kSin, 1e-6, "kSin from holds")
        assertEquals(KV, r.kVHold, 1e-6, "hold-side kV pinned by the speed spread")
        assertEquals(KV, r.kVRun, 1e-6, "run-side cross-check agrees on clean data")
        // Lash-free holds: the split columns either get dropped by the residual gate (NaN) or fit
        // an offset of ~0 — both mean "no lash detected".
        assertTrue(
            r.halfLashRad.isNaN() || abs(r.halfLashRad) < 1e-6,
            "no lash in the synthetic holds, got ${r.halfLashRad}",
        )
        assertTrue(r.kVDisagreement() < 1e-6, "no flex flag on clean data")
    }

    @Test
    fun solveTwoStageAbsorbsBacklashOffsetIntoHalfLashNotKs() {
        // Motor-side holds through backlash: gravity acts at the arm angle, offset ±h from the
        // encoder with travel direction. Without the direction-split gravity columns this offset
        // is collinear with sign(ω) and inflates kS; with them it must come back out as h.
        val h = Math.toRadians(2.5)
        val holdRows = ArrayList<DoubleArray>()
        val holdRhs = ArrayList<Double>()
        for (w in doubleArrayOf(-2.5, -1.5, -0.75, 0.75, 1.5, 2.5)) {
            var theta = -0.4
            while (theta <= 2.6) {
                holdRows.add(holdRow(w, theta))
                val armTheta = theta - sign(w) * h
                holdRhs.add(KS * sign(w) + KV * w + KCOS * cos(armTheta) + KSIN * sin(armTheta))
                theta += 0.05
            }
        }
        val movingRows = ArrayList<DoubleArray>()
        val movingRhs = ArrayList<Double>()
        val geom =
            arrayOf(
                doubleArrayOf(0.08, 0.20, 0.35, 0.079, 0.012),
                doubleArrayOf(0.16, 0.55, -0.20, 0.150, 0.030),
                doubleArrayOf(0.30, 1.10, 0.05, 0.280, 0.070),
                doubleArrayOf(0.08, 0.18, -0.40, 0.070, 0.020),
                doubleArrayOf(0.16, 0.40, 0.60, 0.120, 0.045),
                doubleArrayOf(0.30, 0.95, -0.15, 0.240, 0.090),
            )
        for (g in geom) {
            movingRows.add(doubleArrayOf(g[0], g[1], g[2], g[3], g[4]))
            movingRhs.add(KS * g[0] + KV * g[1] + KA * g[2] + KCOS * g[3] + KSIN * g[4])
        }

        val r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs)
        assertEquals(KS, r.kS, 0.02, "kS must not absorb the lash offset")
        assertEquals(h, r.halfLashRad, Math.toRadians(0.3), "half-lash recovered from the holds")
        assertEquals(KCOS, r.kCos, 0.03, "kCos survives the lash offset")
        assertEquals(KV, r.kV, 0.02, "hold-side kV survives the lash offset")
    }

    @Test
    fun solveTwoStageAbsorbsGravityFaceOffsetIntoHalfLashNotKs() {
        // A gravity-dominated pure-lash drivetrain: the loaded tooth face follows the sign of the
        // gravity torque, so the motor-vs-arm offset flips at the horizontals (sign(cosθ)), not
        // with travel direction. The gravity-face column family must absorb it and report h.
        val h = Math.toRadians(6.0)
        val holdRows = ArrayList<DoubleArray>()
        val holdRhs = ArrayList<Double>()
        for (w in doubleArrayOf(-2.5, -1.5, -0.75, 0.75, 1.5, 2.5)) {
            var theta = -0.4
            while (theta <= 2.6) {
                holdRows.add(holdRow(w, theta))
                val armTheta = theta - sign(cos(theta)) * h
                holdRhs.add(KS * sign(w) + KV * w + KCOS * cos(armTheta) + KSIN * sin(armTheta))
                theta += 0.05
            }
        }
        val movingRows = ArrayList<DoubleArray>()
        val movingRhs = ArrayList<Double>()
        val geom =
            arrayOf(
                doubleArrayOf(0.08, 0.20, 0.35, 0.079, 0.012),
                doubleArrayOf(0.16, 0.55, -0.20, 0.150, 0.030),
                doubleArrayOf(0.30, 1.10, 0.05, 0.280, 0.070),
                doubleArrayOf(0.08, 0.18, -0.40, 0.070, 0.020),
                doubleArrayOf(0.16, 0.40, 0.60, 0.120, 0.045),
                doubleArrayOf(0.30, 0.95, -0.15, 0.240, 0.090),
            )
        for (g in geom) {
            movingRows.add(doubleArrayOf(g[0], g[1], g[2], g[3], g[4]))
            movingRhs.add(KS * g[0] + KV * g[1] + KA * g[2] + KCOS * g[3] + KSIN * g[4])
        }

        val r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs)
        assertEquals(KS, r.kS, 0.03, "kS must not absorb the gravity-face offset")
        assertEquals(KV, r.kV, 0.02, "hold-side kV survives the gravity-face offset")
        assertEquals(
            h,
            r.halfLashRad,
            Math.toRadians(0.8),
            "half-lash recovered from the gravity-face columns",
        )
    }

    @Test
    fun narrowSpeedHoldsFallBackToRunSideKv() {
        // Holds bunched at nearly one speed magnitude: sign(ω) and ω are close to collinear, so
        // the hold-side kV is untrustworthy and the run-side estimate must ship instead.
        val holdRows = ArrayList<DoubleArray>()
        val holdRhs = ArrayList<Double>()
        for (w in doubleArrayOf(-1.6, -1.4, 1.4, 1.6)) {
            var theta = -0.4
            while (theta <= 2.6) {
                holdRows.add(holdRow(w, theta))
                holdRhs.add(holdVoltage(w, theta))
                theta += 0.05
            }
        }
        val movingRows = ArrayList<DoubleArray>()
        val movingRhs = ArrayList<Double>()
        val geom =
            arrayOf(
                doubleArrayOf(0.08, 0.20, 0.35, 0.079, 0.012),
                doubleArrayOf(0.16, 0.55, -0.20, 0.150, 0.030),
                doubleArrayOf(0.30, 1.10, 0.05, 0.280, 0.070),
                doubleArrayOf(0.08, 0.18, -0.40, 0.070, 0.020),
                doubleArrayOf(0.16, 0.40, 0.60, 0.120, 0.045),
                doubleArrayOf(0.30, 0.95, -0.15, 0.240, 0.090),
            )
        for (g in geom) {
            movingRows.add(doubleArrayOf(g[0], g[1], g[2], g[3], g[4]))
            movingRhs.add(KS * g[0] + KV * g[1] + KA * g[2] + KCOS * g[3] + KSIN * g[4])
        }

        val r = ArmSysId.solveTwoStage(holdRows, holdRhs, movingRows, movingRhs)
        assertTrue(r.kVHold.isNaN(), "hold-side kV unavailable without speed spread")
        assertEquals(KV, r.kVRun, 0.05, "run-side kV ships")
        assertEquals(KV, r.kV, 0.05, "shipped kV falls back to the run side")
    }

    @Test
    fun accumulateHoldRecoversFrictionAndGravityUnderLoopJitter() {
        // Synthetic constant-velocity holds at several speeds/directions, logged with jittered
        // wall-clock timestamps (7–13 ms) like a real robot loop. The pooled OLS over the hold rows
        // must recover kS, kV, kCos, kSin despite the non-uniform dt.
        val rng = Random(7)
        val rows = ArrayList<DoubleArray>()
        val rhs = ArrayList<Double>()
        for (w in doubleArrayOf(-2.5, -1.5, -0.75, 0.75, 1.5, 2.5)) {
            val n = 260
            val theta = DoubleArray(n)
            val volt = DoubleArray(n)
            val time = DoubleArray(n)
            var t = 0.0
            var th = if (w > 0) -0.4 else 3.0 // sweep across the [-1, 3.6] window
            for (i in 0 until n) {
                val dt = 0.010 * (0.7 + 0.6 * rng.nextDouble())
                t += dt
                th += w * dt
                time[i] = t
                theta[i] = th
                volt[i] = KS * sign(w) + KV * w + KCOS * cos(th) + KSIN * sin(th)
            }
            ArmSysId.accumulateHold(
                theta,
                volt,
                time,
                -1.0,
                3.6,
                ArmSysId.DEFAULT_PARAMS,
                rows,
                rhs,
            )
        }
        assertTrue(rows.size > 100, "should keep many steady samples, got ${rows.size}")

        // Hold rows are [sign(ω), ω, cosθ, sinθ, then the split-column families and α]; OLS
        // recovers [kS, kV, kCos, kSin, ~0...].
        val b = ArmSysId.solveOls(rows.toTypedArray(), toPrimitive(rhs))
        assertEquals(KS, b[0], 0.02, "kS")
        assertEquals(KV, b[1], 0.02, "kV")
        assertEquals(KCOS, b[2], 0.03, "kCos")
        assertEquals(KSIN, b[3], 0.03, "kSin")
    }

    private fun toPrimitive(list: List<Double>): DoubleArray {
        val a = DoubleArray(list.size)
        for (i in a.indices) a[i] = list[i]
        return a
    }

    @Test
    fun fallsBackToSingleStageWhenNoHolds() {
        // Too few hold rows: solveTwoStage must fall back to solve() on the moving rows alone.
        val movingRows = ArrayList<DoubleArray>()
        val movingRhs = ArrayList<Double>()
        val geom =
            arrayOf(
                doubleArrayOf(0.08, 0.20, 0.35, 0.079, 0.012),
                doubleArrayOf(0.16, 0.55, -0.20, 0.150, 0.030),
                doubleArrayOf(0.30, 1.10, 0.05, 0.280, 0.070),
                doubleArrayOf(0.08, 0.18, -0.40, 0.070, 0.020),
                doubleArrayOf(0.16, 0.40, 0.60, 0.120, 0.045),
                doubleArrayOf(0.30, 0.95, -0.15, 0.240, 0.090),
            )
        for (g in geom) {
            movingRows.add(doubleArrayOf(g[0], g[1], g[2], g[3], g[4]))
            movingRhs.add(KS * g[0] + KV * g[1] + KA * g[2] + KCOS * g[3] + KSIN * g[4])
        }
        val r = ArmSysId.solveTwoStage(ArrayList(), ArrayList(), movingRows, movingRhs)
        assertTrue(r.samples > 0, "fallback should still solve from the moving rows")
        assertEquals(KA, r.kA, 1e-6, "single-stage fallback recovers kA")
    }

    @Test
    fun estimateFlexPeriodFindsARingDown() {
        // 3 Hz decaying ring around a rest angle, sampled at a jittery ~100 Hz robot loop.
        val rng = Random(3)
        val n = 250
        val theta = DoubleArray(n)
        val time = DoubleArray(n)
        var t = 0.0
        for (i in 0 until n) {
            t += 0.010 * (0.8 + 0.4 * rng.nextDouble())
            time[i] = t
            theta[i] = 1.2 + 0.05 * exp(-0.6 * t) * sin(2 * Math.PI * 3.0 * t)
        }
        val period = ArmSysId.estimateFlexPeriod(theta, time)
        assertEquals(1.0 / 3.0, period, 0.02, "3 Hz ring-down period")
    }

    @Test
    fun estimateFlexPeriodRejectsAQuietLog() {
        val n = 100
        val theta = DoubleArray(n)
        val time = DoubleArray(n)
        for (i in 0 until n) {
            time[i] = i * 0.01
            theta[i] = 1.2 + 1e-5 * sin(i.toDouble()) // sub-noise wiggle
        }
        assertEquals(0.0, ArmSysId.estimateFlexPeriod(theta, time), "no ring to measure")
    }

    @Test
    fun flexPeriodIntervalsCancelTheRingOutOfKa() {
        // A constant-voltage run whose position log carries an undamped-ish 3 Hz ring on top of
        // the rigid response. Whole-period intervals must beat the fixed-length ones on kA.
        val flexHz = 3.0
        val plain = ArmSysId.FitParams()
        val gated = ArmSysId.FitParams()
        gated.flexPeriodSec = 1.0 / flexHz

        val kaPlain = fitKaFromRingingRun(plain)
        val kaGated = fitKaFromRingingRun(gated)
        assertTrue(
            abs(kaGated - KA) < abs(kaPlain - KA),
            String.format(
                "gated kA %.4f should beat plain kA %.4f (true %.2f)",
                kaGated,
                kaPlain,
                KA,
            ),
        )
        assertEquals(KA, kaGated, KA * 0.10, "gated kA within 10%")
    }

    /** Accumulate one synthetic ringing run with [params] and solve; return the fitted kA. */
    private fun fitKaFromRingingRun(params: ArmSysId.FitParams): Double {
        val n = 200
        val dt = 0.01
        val volts = 6.0
        val theta = DoubleArray(n)
        val voltage = DoubleArray(n)
        val time = DoubleArray(n)
        // Rigid trajectory obeying the full model, integrated forward, plus a 3 Hz flex ring the
        // motor encoder sees.
        var pos = 0.0
        var vel = 0.0
        for (i in 0 until n) {
            val t = i * dt
            val accel = (volts - KS - KV * vel - KCOS * cos(pos) - KSIN * sin(pos)) / KA
            vel += accel * dt
            pos += vel * dt
            time[i] = t
            voltage[i] = volts
            theta[i] = pos + 0.015 * exp(-0.5 * t) * sin(2 * Math.PI * 3.0 * t)
        }
        val rows = ArrayList<DoubleArray>()
        val rhs = ArrayList<Double>()
        ArmSysId.accumulateRun(theta, voltage, time, -100.0, 100.0, params, rows, rhs)
        assertTrue(rows.size >= 3, "ringing run should still yield rows, got ${rows.size}")
        // Stage-2 style: everything but kA fixed at truth, so the fit isolates the Δω boundary
        // term — exactly what the whole-period intervals are supposed to clean of the ring.
        var num = 0.0
        var den = 0.0
        for (i in rows.indices) {
            val xi = rows[i] // [sign·Δt, Δθ, Δω, ∫cos, ∫sin]
            val residual = rhs[i] - KS * xi[0] - KV * xi[1] - KCOS * xi[3] - KSIN * xi[4]
            num += xi[2] * residual
            den += xi[2] * xi[2]
        }
        return num / den
    }

    companion object {
        private const val KS = 0.30
        private const val KV = 1.20
        private const val KA = 0.35
        private const val KCOS = 3.50
        private const val KSIN = 0.40
    }
}
