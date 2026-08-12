package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotNull
import org.junit.jupiter.api.Assertions.assertNull
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Unit tests for the pure multi-hypothesis core [PoseHypothesisBank]: deterministic synthetic
 * true+mirror branch tracks, fed frame by frame, asserting the bank's emergent behaviour rather
 * than any hardware loop. The load-bearing claims of the lean bank-only core:
 * <ul>
 * <li>under rotation OR translation the mirror's weight collapses and the bank commits to the true
 *   datum;
 * <li>parked, the two branches stay tied — no false commit, with no special-case gate;
 * <li>a deliberately wrong latch self-corrects under motion with no reseed call;
 * <li>the bank stays bounded under a long run;
 * <li>the per-frame ambiguity confidence lets a confident stationary frame commit while an
 *   ambiguous one abstains;
 * <li>the sigma(span) shoulders discount far/small-tag frames.
 * </ul>
 */
class PoseHypothesisBankTest {

    companion object {
        // True datum the synthetic robot obeys: heading offset θ₀ and translation (t0x, t0y).
        private const val TH0 = 0.5
        private const val T0X = 30.0
        private const val T0Y = -20.0

        private val ORIGIN = p(0.0, 0.0, 0.0)

        private fun p(x: Double, y: Double, h: Double): Pose2d {
            return Pose2d(x, y, Rotation2d(h))
        }

        private fun wrap(a: Double): Double {
            return atan2(sin(a), cos(a))
        }

        /** Reflect a pose across the field line at angle [phi] through the tag at [(qx,qy)]. */
        private fun reflect(pose: Pose2d, phi: Double, qx: Double, qy: Double): Pose2d {
            val c2 = cos(2 * phi)
            val s2 = sin(2 * phi)
            val dx = pose.x - qx
            val dy = pose.y - qy
            return p(
                c2 * dx + s2 * dy + qx,
                s2 * dx - c2 * dy + qy,
                wrap(2 * phi - pose.rotation.radians),
            )
        }

        /** Odometry pose at frame [i] of [n]: spin in place, drive a diagonal, or sit still. */
        private fun odoAt(i: Int, n: Int, motion: String): Pose2d {
            val f = if (n > 1) i.toDouble() / (n - 1) else 0.0
            return when (motion) {
                "rotate" -> p(5.0, 3.0, Math.toRadians(99.0) * f)
                "translate" -> p(40.0 * f, 24.0 * f, 0.3)
                else -> p(5.0, 3.0, 0.3) // parked
            }
        }

        /** True branch field pose = datum applied to odometry. */
        private fun truePose(odo: Pose2d): Pose2d {
            val c = cos(TH0)
            val s = sin(TH0)
            return p(
                c * odo.x - s * odo.y + T0X,
                s * odo.x + c * odo.y + T0Y,
                wrap(odo.rotation.radians + TH0),
            )
        }

        /**
         * Both IPPE branches {true, mirror} for a frame, in the order vision would (mirror first on
         * even).
         */
        private fun branches(odo: Pose2d, swap: Boolean): Array<Pose2d> {
            val t = truePose(odo)
            val m = reflect(t, 0.9, 60.0, 40.0)
            return if (swap) arrayOf(m, t) else arrayOf(t, m)
        }

        private fun bank(): PoseHypothesisBank {
            return PoseHypothesisBank(PoseHypothesisBank.Params())
        }

        /** Run a motion profile through the bank and return it. */
        private fun run(motion: String, n: Int, swap: Boolean): PoseHypothesisBank {
            val b = bank()
            for (i in 0 until n) {
                val odo = odoAt(i, n, motion)
                b.observe(branches(odo, swap && (i % 2 == 0)), odo)
            }
            return b
        }

        private fun assertCommittedToTrue(b: PoseHypothesisBank, why: String) {
            val datum = b.mapDatum()
            assertNotNull(datum, "$why: expected a dominant hypothesis")
            assertEquals(TH0, datum!!.rotation.radians, 0.05, "$why: datum θ₀")
            assertEquals(T0X, datum.x, 1.5, "$why: datum x")
            assertEquals(T0Y, datum.y, 1.5, "$why: datum y")
            assertTrue(
                b.dominantWeight() > 0.8,
                "$why: dominant weight should commit (was ${b.dominantWeight()})",
            )
        }

        /** Feed a stationary stream of {true=best, mirror=alt} branches at a fixed ambiguity. */
        private fun stationaryConfident(ambiguity: Double, n: Int): PoseHypothesisBank {
            val b = bank()
            val odo = odoAt(0, 30, "parked") // fixed pose — no motion to separate the branches
            val conf = doubleArrayOf(1.0, ambiguity) // best branch full, alt scaled by ambiguity
            for (i in 0 until n) {
                b.observe(branches(odo, false), conf, odo, 150.0) // index 0 = true = best
            }
            return b
        }

        /** Weight of hyp A after one frame on A's datum, with a competitor B `gap` inches away. */
        private fun oneFramePull(spanPx: Double, gap: Double): Double {
            val b = bank()
            b.seed(p(0.0, 0.0, 0.0), 0.5) // A
            b.seed(p(gap, 0.0, 0.0), 0.5) // B, `gap` inches away
            val origin = p(0.0, 0.0, 0.0)
            b.observe(arrayOf(p(0.0, 0.0, 0.0)), origin, spanPx) // implied datum == A
            return b.dominantWeight()
        }
    }

    // --- commit under each kind of motion -------------------------------------------------------

    @Test
    fun commitsUnderRotation() {
        assertCommittedToTrue(run("rotate", 30, false), "rotation")
    }

    @Test
    fun commitsUnderPureTranslationWhereHeadingTestIsBlind() {
        // No rotation at all — the heading-only discriminators are blind here; the datum's
        // translation part still separates the branches.
        assertCommittedToTrue(run("translate", 30, false), "translation")
    }

    @Test
    fun commitIsImmuneToPerFrameBranchOrder() {
        // Shuffling which branch is listed first each frame must not change the outcome.
        assertCommittedToTrue(run("rotate", 30, true), "rotation, branches swapped")
    }

    // --- parked: explicit ambiguity, no false commit --------------------------------------------

    @Test
    fun abstainsWhileParked() {
        val b = run("parked", 30, false)
        // Both branches' datums are constant while parked, so neither weight runs away: the bank
        // stays undecided. No special-case "don't guess while parked" gate produces this — it falls
        // out of the datum representation.
        assertTrue(b.size() >= 2, "parked must keep ≥2 live hypotheses (true + mirror)")
        assertTrue(
            b.dominantWeight() < 0.8,
            "parked dominant weight must not commit (was ${b.dominantWeight()})",
        )
    }

    // --- reseed-free recovery from a wrong latch (the headline win) ------------------------------

    @Test
    fun recoversFromAWrongLatchWithoutAReseed() {
        val b = bank()

        // Latch the bank hard on the MIRROR datum, as a bad bootstrap would. Seed the true datum
        // only faintly, to prove recovery isn't just "it was there all along at equal weight".
        val odo0 = odoAt(0, 30, "rotate")
        val mirrorDatum =
            PoseHypothesisBank.impliedDatum(reflect(truePose(odo0), 0.9, 60.0, 40.0), odo0)
        val trueDatum = p(T0X, T0Y, TH0)
        b.seed(mirrorDatum, 0.95)
        b.seed(trueDatum, 0.05)
        assertTrue(b.dominantWeight() > 0.8, "precondition: starts latched on the mirror")
        // And the dominant pose really is the wrong (mirror) one to begin with.
        assertTrue(
            hypot(b.mapDatum()!!.x - T0X, b.mapDatum()!!.y - T0Y) > 10.0,
            "precondition: dominant datum is the mirror, far from true",
        )

        // Now just drive. No reseed, no setPose — only observations.
        for (i in 0 until 30) {
            val odo = odoAt(i, 30, "rotate")
            b.observe(branches(odo, false), odo)
        }
        assertCommittedToTrue(b, "post-recovery")
    }

    // --- boundedness ----------------------------------------------------------------------------

    @Test
    fun staysBoundedOverALongRun() {
        val b = bank()
        // Alternate motion and parking for many frames; the bank must never blow past its cap even
        // as births keep spawning mirror datums.
        for (i in 0 until 400) {
            val motion = if ((i / 20) % 2 == 0) "rotate" else "parked"
            val odo = odoAt(i % 30, 30, motion)
            b.observe(branches(odo, i % 2 == 0), odo)
            assertTrue(
                b.size() <= PoseHypothesisBank.Params().maxHypotheses,
                "bank size must stay capped (was ${b.size()})",
            )
        }
    }

    // --- mapPose wiring -------------------------------------------------------------------------

    @Test
    fun mapPoseAppliesTheDominantDatumToCurrentOdometry() {
        val b = run("rotate", 30, false)
        val odoNow = p(12.0, -7.0, 0.4)
        val expected = truePose(odoNow) // true datum ∘ odoNow
        val got = b.mapPose(odoNow)
        assertNotNull(got)
        assertEquals(expected.x, got!!.x, 2.0, "mapPose x")
        assertEquals(expected.y, got.y, 2.0, "mapPose y")
        assertEquals(
            0.0,
            wrap(expected.rotation.radians - got.rotation.radians),
            0.05,
            "mapPose heading",
        )
    }

    // --- ambiguity weighting: commit while stationary when the frame is confident ----------------

    @Test
    fun commitsWhileStationaryWhenTheFrameIsConfident() {
        // The close, low-ambiguity case: MT1 plainly correct, no motion. With the per-frame branch
        // confidence the bank commits to the best (true) branch even parked.
        val b = stationaryConfident(0.1, 20)
        assertTrue(
            b.dominantWeight() > 0.8,
            "a confident stationary stream commits (was ${b.dominantWeight()})",
        )
        val datum = b.mapDatum()!!
        assertEquals(
            TH0,
            datum.rotation.radians,
            0.05,
            "commits to the best (true) branch θ₀",
        )
        assertEquals(T0X, datum.x, 1.5, "datum x")
        assertEquals(T0Y, datum.y, 1.5, "datum y")
    }

    @Test
    fun stillAbstainsWhileStationaryWhenTheFrameIsAmbiguous() {
        // Near-1 ambiguity: the two branches fit comparably, so there's no single-frame evidence
        // and
        // no motion — the bank must stay undecided (the genuinely-ambiguous far/head-on case).
        val b = stationaryConfident(0.95, 20)
        assertTrue(
            b.dominantWeight() < 0.8,
            "an ambiguous stationary stream must not commit (was ${b.dominantWeight()})",
        )
        assertTrue(b.size() >= 2, "both branches stay alive")
    }

    @Test
    fun equalConfidencePreservesMotionOnlyBehavior() {
        // null/equal confidence == the motion-only bank: stationary stays tied even though the
        // helper labels a "best" branch (no confidence signal to exploit).
        val b = bank()
        val odo = odoAt(0, 30, "parked")
        for (i in 0 until 20) {
            b.observe(branches(odo, false), null, odo, 150.0)
        }
        assertTrue(b.dominantWeight() < 0.8, "equal-confidence stationary stays undecided")
    }

    @Test
    fun emptyBankReportsNoPose() {
        assertEquals(0, bank().size())
        assertNull(bank().mapPose(p(0.0, 0.0, 0.0)))
    }

    // --- sigma(span) shoulders + residual logging -----------------------------------------------

    @Test
    fun farSmallTagFramesDiscountAMarginalResidualMoreThanCloseFrames() {
        // The sigma(span) shoulder bites in the sub-gate (marginal) regime: a frame whose datum is
        // a *moderate* distance from a competing hypothesis should down-weight that competitor less
        // when the tag is small/far (inflated sigma) than when it's large/close.
        val prm = PoseHypothesisBank.Params()
        val moderate = 2.0 * prm.datumPosSigmaIn // ~2 sigma: inside residualGate (3 sigma)

        val aClose = oneFramePull(prm.fineKneePx + 30, moderate) // large tag -> no inflation
        val aFar = oneFramePull(prm.uselessKneePx - 10, moderate) // small tag -> inflated sigma
        assertTrue(
            aClose > 0.8,
            "close frame pulls the matching hypothesis well ahead (was $aClose)",
        )
        assertTrue(
            aFar < aClose - 0.2,
            "far frame discounts the same residual, so it pulls ahead much less (was $aFar vs close $aClose)",
        )
    }

    @Test
    fun birthsAreSuppressedBelowTheUselessKnee() {
        val prm = PoseHypothesisBank.Params()
        val b = PoseHypothesisBank(prm)
        // Seed one good hypothesis, then feed only far/garbage frames whose datums don't match it.
        // With births suppressed, those junk datums must NOT accumulate into the bank.
        val odo = odoAt(5, 30, "rotate")
        b.seed(p(T0X, T0Y, TH0), 1.0)
        for (i in 0 until 10) {
            val garbage = p(T0X + 40 + i, T0Y - 40, TH0 + 1.5) // far from the seed datum
            b.observe(
                arrayOf(PoseHypothesisBank.composeDatumOnOdo(garbage, odo)),
                odo,
                prm.uselessKneePx - 20,
            )
        }
        assertEquals(1, b.size(), "far garbage must not spawn surviving hypotheses")
    }

    @Test
    fun splitHeadInflationKeepsAFarFramesHeadingVote() {
        // Two hypotheses at the same position, headings 30° apart, fed far frames (below the
        // useless
        // knee) that match one heading exactly. Uniform inflation stretches the heading sigma to
        // 36°, so the 30° discrepancy barely votes; with the head inflation split back to 1, the
        // mismatched hypothesis is gated out in a couple of frames.
        val datumA = p(10.0, 5.0, 0.0)
        val datumB = p(10.0, 5.0, Math.toRadians(30.0))
        for (split in booleanArrayOf(false, true)) {
            val params = PoseHypothesisBank.Params()
            if (split) {
                params.farSigmaInflationHead = 1.0
            }
            val b = PoseHypothesisBank(params)
            b.seed(datumA, 0.5)
            b.seed(datumB, 0.5)
            val farSpan = params.uselessKneePx - 20
            for (i in 0 until 3) {
                b.observe(arrayOf(datumA), null, ORIGIN, farSpan)
            }
            if (split) {
                assertTrue(
                    b.dominantWeight() > 0.95,
                    "split: far heading evidence must separate (was ${b.dominantWeight()})",
                )
            } else {
                assertTrue(
                    b.dominantWeight() < 0.9,
                    "uniform: far heading evidence stays muted (was ${b.dominantWeight()})",
                )
            }
        }
    }

    @Test
    fun recordsTheDatumResidualForCalibration() {
        val b = run("rotate", 30, false) // committed to the true datum
        val odo = odoAt(10, 30, "rotate")
        // A frame on the true branch: residual vs the dominant datum should be ~0.
        b.observe(arrayOf(truePose(odo)), odo, 150.0)
        assertTrue(b.lastResidPosIn() < 2.0, "on-datum residual is small")
        assertEquals(150.0, b.lastSpanPx(), 1e-9, "span is recorded")
        assertEquals(1.0, b.lastSigmaInflation(), 1e-9, "no inflation in the fine zone")
    }
}
