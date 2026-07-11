package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for the pure multi-hypothesis core {@link PoseHypothesisBank}: deterministic synthetic
 * true+mirror branch tracks, fed frame by frame, asserting the bank's emergent behaviour rather than
 * any hardware loop. The load-bearing claims of the lean bank-only core:
 *
 * <ul>
 *   <li>under rotation OR translation the mirror's weight collapses and the bank commits to the
 *       true datum;
 *   <li>parked, the two branches stay tied — no false commit, with no special-case gate;
 *   <li>a deliberately wrong latch self-corrects under motion with no reseed call;
 *   <li>the bank stays bounded under a long run;
 *   <li>the per-frame ambiguity confidence lets a confident stationary frame commit while an
 *       ambiguous one abstains;
 *   <li>the sigma(span) shoulders discount far/small-tag frames.
 * </ul>
 */
class PoseHypothesisBankTest {

    // True datum the synthetic robot obeys: heading offset θ₀ and translation (t0x, t0y).
    private static final double TH0 = 0.5;
    private static final double T0X = 30.0;
    private static final double T0Y = -20.0;

    private static final Pose2d ORIGIN = p(0, 0, 0);

    private static Pose2d p(double x, double y, double h) {
        return new Pose2d(x, y, new Rotation2d(h));
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    /**
     * Reflect a pose across the field line at angle {@code phi} through the tag at {@code (qx,qy)}.
     */
    private static Pose2d reflect(Pose2d pose, double phi, double qx, double qy) {
        double c2 = Math.cos(2 * phi), s2 = Math.sin(2 * phi);
        double dx = pose.getX() - qx, dy = pose.getY() - qy;
        return p(
                c2 * dx + s2 * dy + qx,
                s2 * dx - c2 * dy + qy,
                wrap(2 * phi - pose.getRotation().getRadians()));
    }

    /**
     * Odometry pose at frame {@code i} of {@code n}: spin in place, drive a diagonal, or sit still.
     */
    private static Pose2d odoAt(int i, int n, String motion) {
        double f = (n > 1) ? (double) i / (n - 1) : 0.0;
        switch (motion) {
            case "rotate":
                return p(5.0, 3.0, Math.toRadians(99) * f);
            case "translate":
                return p(40.0 * f, 24.0 * f, 0.3);
            default: // parked
                return p(5.0, 3.0, 0.3);
        }
    }

    /** True branch field pose = datum applied to odometry. */
    private static Pose2d truePose(Pose2d odo) {
        double c = Math.cos(TH0), s = Math.sin(TH0);
        return p(
                c * odo.getX() - s * odo.getY() + T0X,
                s * odo.getX() + c * odo.getY() + T0Y,
                wrap(odo.getRotation().getRadians() + TH0));
    }

    /**
     * Both IPPE branches {true, mirror} for a frame, in the order vision would (mirror first on
     * even).
     */
    private static Pose2d[] branches(Pose2d odo, boolean swap) {
        Pose2d t = truePose(odo);
        Pose2d m = reflect(t, 0.9, 60, 40);
        return swap ? new Pose2d[] {m, t} : new Pose2d[] {t, m};
    }

    private static PoseHypothesisBank bank() {
        return new PoseHypothesisBank(new PoseHypothesisBank.Params());
    }

    /** Run a motion profile through the bank and return it. */
    private static PoseHypothesisBank run(String motion, int n, boolean swap) {
        PoseHypothesisBank b = bank();
        for (int i = 0; i < n; i++) {
            Pose2d odo = odoAt(i, n, motion);
            b.observe(branches(odo, swap && (i % 2 == 0)), odo);
        }
        return b;
    }

    private static void assertCommittedToTrue(PoseHypothesisBank b, String why) {
        Pose2d datum = b.mapDatum();
        assertNotNull(datum, why + ": expected a dominant hypothesis");
        assertEquals(TH0, datum.getRotation().getRadians(), 0.05, why + ": datum θ₀");
        assertEquals(T0X, datum.getX(), 1.5, why + ": datum x");
        assertEquals(T0Y, datum.getY(), 1.5, why + ": datum y");
        assertTrue(
                b.dominantWeight() > 0.8,
                why + ": dominant weight should commit (was " + b.dominantWeight() + ")");
    }

    // --- commit under each kind of motion -------------------------------------------------------

    @Test
    void commitsUnderRotation() {
        assertCommittedToTrue(run("rotate", 30, false), "rotation");
    }

    @Test
    void commitsUnderPureTranslationWhereHeadingTestIsBlind() {
        // No rotation at all — the heading-only discriminators are blind here; the datum's
        // translation part still separates the branches.
        assertCommittedToTrue(run("translate", 30, false), "translation");
    }

    @Test
    void commitIsImmuneToPerFrameBranchOrder() {
        // Shuffling which branch is listed first each frame must not change the outcome.
        assertCommittedToTrue(run("rotate", 30, true), "rotation, branches swapped");
    }

    // --- parked: explicit ambiguity, no false commit --------------------------------------------

    @Test
    void abstainsWhileParked() {
        PoseHypothesisBank b = run("parked", 30, false);
        // Both branches' datums are constant while parked, so neither weight runs away: the bank
        // stays undecided. No special-case "don't guess while parked" gate produces this — it falls
        // out of the datum representation.
        assertTrue(b.size() >= 2, "parked must keep ≥2 live hypotheses (true + mirror)");
        assertTrue(
                b.dominantWeight() < 0.8,
                "parked dominant weight must not commit (was " + b.dominantWeight() + ")");
    }

    // --- reseed-free recovery from a wrong latch (the headline win) ------------------------------

    @Test
    void recoversFromAWrongLatchWithoutAReseed() {
        PoseHypothesisBank b = bank();

        // Latch the bank hard on the MIRROR datum, as a bad bootstrap would. Seed the true datum
        // only faintly, to prove recovery isn't just "it was there all along at equal weight".
        Pose2d odo0 = odoAt(0, 30, "rotate");
        Pose2d mirrorDatum =
                PoseHypothesisBank.impliedDatum(reflect(truePose(odo0), 0.9, 60, 40), odo0);
        Pose2d trueDatum = p(T0X, T0Y, TH0);
        b.seed(mirrorDatum, 0.95);
        b.seed(trueDatum, 0.05);
        assertTrue(b.dominantWeight() > 0.8, "precondition: starts latched on the mirror");
        // And the dominant pose really is the wrong (mirror) one to begin with.
        assertTrue(
                Math.hypot(b.mapDatum().getX() - T0X, b.mapDatum().getY() - T0Y) > 10.0,
                "precondition: dominant datum is the mirror, far from true");

        // Now just drive. No reseed, no setPose — only observations.
        for (int i = 0; i < 30; i++) {
            Pose2d odo = odoAt(i, 30, "rotate");
            b.observe(branches(odo, false), odo);
        }
        assertCommittedToTrue(b, "post-recovery");
    }

    // --- boundedness ----------------------------------------------------------------------------

    @Test
    void staysBoundedOverALongRun() {
        PoseHypothesisBank b = bank();
        // Alternate motion and parking for many frames; the bank must never blow past its cap even
        // as births keep spawning mirror datums.
        for (int i = 0; i < 400; i++) {
            String motion = (i / 20) % 2 == 0 ? "rotate" : "parked";
            Pose2d odo = odoAt(i % 30, 30, motion);
            b.observe(branches(odo, i % 2 == 0), odo);
            assertTrue(
                    b.size() <= new PoseHypothesisBank.Params().maxHypotheses,
                    "bank size must stay capped (was " + b.size() + ")");
        }
    }

    // --- mapPose wiring -------------------------------------------------------------------------

    @Test
    void mapPoseAppliesTheDominantDatumToCurrentOdometry() {
        PoseHypothesisBank b = run("rotate", 30, false);
        Pose2d odoNow = p(12.0, -7.0, 0.4);
        Pose2d expected = truePose(odoNow); // true datum ∘ odoNow
        Pose2d got = b.mapPose(odoNow);
        assertNotNull(got);
        assertEquals(expected.getX(), got.getX(), 2.0, "mapPose x");
        assertEquals(expected.getY(), got.getY(), 2.0, "mapPose y");
        assertEquals(
                0.0,
                wrap(expected.getRotation().getRadians() - got.getRotation().getRadians()),
                0.05,
                "mapPose heading");
    }

    // --- ambiguity weighting: commit while stationary when the frame is confident ----------------

    /** Feed a stationary stream of {true=best, mirror=alt} branches at a fixed ambiguity. */
    private static PoseHypothesisBank stationaryConfident(double ambiguity, int n) {
        PoseHypothesisBank b = bank();
        Pose2d odo = odoAt(0, 30, "parked"); // fixed pose — no motion to separate the branches
        double[] conf = {1.0, ambiguity}; // best branch full, alt scaled by ambiguity
        for (int i = 0; i < n; i++) {
            b.observe(branches(odo, false), conf, odo, 150.0); // index 0 = true = best
        }
        return b;
    }

    @Test
    void commitsWhileStationaryWhenTheFrameIsConfident() {
        // The close, low-ambiguity case: MT1 plainly correct, no motion. With the per-frame branch
        // confidence the bank commits to the best (true) branch even parked.
        PoseHypothesisBank b = stationaryConfident(0.1, 20);
        assertTrue(
                b.dominantWeight() > 0.8,
                "a confident stationary stream commits (was " + b.dominantWeight() + ")");
        Pose2d datum = b.mapDatum();
        assertEquals(
                TH0, datum.getRotation().getRadians(), 0.05, "commits to the best (true) branch θ₀");
        assertEquals(T0X, datum.getX(), 1.5, "datum x");
        assertEquals(T0Y, datum.getY(), 1.5, "datum y");
    }

    @Test
    void stillAbstainsWhileStationaryWhenTheFrameIsAmbiguous() {
        // Near-1 ambiguity: the two branches fit comparably, so there's no single-frame evidence and
        // no motion — the bank must stay undecided (the genuinely-ambiguous far/head-on case).
        PoseHypothesisBank b = stationaryConfident(0.95, 20);
        assertTrue(
                b.dominantWeight() < 0.8,
                "an ambiguous stationary stream must not commit (was " + b.dominantWeight() + ")");
        assertTrue(b.size() >= 2, "both branches stay alive");
    }

    @Test
    void equalConfidencePreservesMotionOnlyBehavior() {
        // null/equal confidence == the motion-only bank: stationary stays tied even though the
        // helper labels a "best" branch (no confidence signal to exploit).
        PoseHypothesisBank b = bank();
        Pose2d odo = odoAt(0, 30, "parked");
        for (int i = 0; i < 20; i++) {
            b.observe(branches(odo, false), null, odo, 150.0);
        }
        assertTrue(b.dominantWeight() < 0.8, "equal-confidence stationary stays undecided");
    }

    @Test
    void emptyBankReportsNoPose() {
        assertEquals(0, bank().size());
        assertNull(bank().mapPose(p(0, 0, 0)));
    }

    // --- sigma(span) shoulders + residual logging -----------------------------------------------

    @Test
    void farSmallTagFramesDiscountAMarginalResidualMoreThanCloseFrames() {
        // The sigma(span) shoulder bites in the sub-gate (marginal) regime: a frame whose datum is
        // a *moderate* distance from a competing hypothesis should down-weight that competitor less
        // when the tag is small/far (inflated sigma) than when it's large/close.
        PoseHypothesisBank.Params prm = new PoseHypothesisBank.Params();
        double moderate = 2.0 * prm.datumPosSigmaIn; // ~2 sigma: inside residualGate (3 sigma)

        double aClose = oneFramePull(prm.fineKneePx + 30, moderate); // large tag -> no inflation
        double aFar = oneFramePull(prm.uselessKneePx - 10, moderate); // small tag -> inflated sigma
        assertTrue(
                aClose > 0.8,
                "close frame pulls the matching hypothesis well ahead (was " + aClose + ")");
        assertTrue(
                aFar < aClose - 0.2,
                "far frame discounts the same residual, so it pulls ahead much less (was "
                        + aFar
                        + " vs close "
                        + aClose
                        + ")");
    }

    /** Weight of hyp A after one frame on A's datum, with a competitor B `gap` inches away. */
    private static double oneFramePull(double spanPx, double gap) {
        PoseHypothesisBank b = bank();
        b.seed(p(0, 0, 0), 0.5); // A
        b.seed(p(gap, 0, 0), 0.5); // B, `gap` inches away
        Pose2d origin = p(0, 0, 0);
        b.observe(new Pose2d[] {p(0, 0, 0)}, origin, spanPx); // implied datum == A
        return b.dominantWeight();
    }

    @Test
    void birthsAreSuppressedBelowTheUselessKnee() {
        PoseHypothesisBank.Params prm = new PoseHypothesisBank.Params();
        PoseHypothesisBank b = new PoseHypothesisBank(prm);
        // Seed one good hypothesis, then feed only far/garbage frames whose datums don't match it.
        // With births suppressed, those junk datums must NOT accumulate into the bank.
        Pose2d odo = odoAt(5, 30, "rotate");
        b.seed(p(T0X, T0Y, TH0), 1.0);
        for (int i = 0; i < 10; i++) {
            Pose2d garbage = p(T0X + 40 + i, T0Y - 40, TH0 + 1.5); // far from the seed datum
            b.observe(
                    new Pose2d[] {PoseHypothesisBank.composeDatumOnOdo(garbage, odo)},
                    odo,
                    prm.uselessKneePx - 20);
        }
        assertEquals(1, b.size(), "far garbage must not spawn surviving hypotheses");
    }

    @Test
    void splitHeadInflationKeepsAFarFramesHeadingVote() {
        // Two hypotheses at the same position, headings 30° apart, fed far frames (below the useless
        // knee) that match one heading exactly. Uniform inflation stretches the heading sigma to
        // 36°, so the 30° discrepancy barely votes; with the head inflation split back to 1, the
        // mismatched hypothesis is gated out in a couple of frames.
        Pose2d datumA = p(10, 5, 0.0);
        Pose2d datumB = p(10, 5, Math.toRadians(30));
        for (boolean split : new boolean[] {false, true}) {
            PoseHypothesisBank.Params params = new PoseHypothesisBank.Params();
            if (split) {
                params.farSigmaInflationHead = 1.0;
            }
            PoseHypothesisBank b = new PoseHypothesisBank(params);
            b.seed(datumA, 0.5);
            b.seed(datumB, 0.5);
            double farSpan = params.uselessKneePx - 20;
            for (int i = 0; i < 3; i++) {
                b.observe(new Pose2d[] {datumA}, null, ORIGIN, farSpan);
            }
            if (split) {
                assertTrue(
                        b.dominantWeight() > 0.95,
                        "split: far heading evidence must separate (was " + b.dominantWeight() + ")");
            } else {
                assertTrue(
                        b.dominantWeight() < 0.9,
                        "uniform: far heading evidence stays muted (was " + b.dominantWeight() + ")");
            }
        }
    }

    @Test
    void recordsTheDatumResidualForCalibration() {
        PoseHypothesisBank b = run("rotate", 30, false); // committed to the true datum
        Pose2d odo = odoAt(10, 30, "rotate");
        // A frame on the true branch: residual vs the dominant datum should be ~0.
        b.observe(new Pose2d[] {truePose(odo)}, odo, 150.0);
        assertTrue(b.lastResidPosIn() < 2.0, "on-datum residual is small");
        assertEquals(150.0, b.lastSpanPx(), 1e-9, "span is recorded");
        assertEquals(1.0, b.lastSigmaInflation(), 1e-9, "no inflation in the fine zone");
    }
}
