package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Test;

import java.util.HashMap;

/**
 * Tests the policy core's non-vision behavior — seeding, the pre-commit odometry fallback, and the
 * per-frame gating no-ops — with a trivial solver (the full PnP → branch-pose path is covered by
 * {@link VisionPoseSolverTest} and {@link PoseHypothesisBankTest}).
 */
class HypothesisBankLocalizerTest {

    private static Pose2d p(double x, double y, double h) {
        return new Pose2d(x, y, new Rotation2d(h));
    }

    private static HypothesisBankLocalizer localizer() {
        // An empty tag table means any vision frame yields no branch poses — fine for these tests,
        // which never feed a valid frame.
        VisionPoseSolver solver =
                new VisionPoseSolver(Transform3D.identity(), new HashMap<>());
        return new HypothesisBankLocalizer(new HypothesisBankLocalizer.Params(), solver);
    }

    @Test
    void emptyBankReturnsRawOdometry() {
        HypothesisBankLocalizer loc = localizer();
        Pose2d odo = p(12, -7, 0.4);
        Pose2d got = loc.getPose(odo);
        assertEquals(odo.getX(), got.getX(), 1e-9);
        assertEquals(odo.getY(), got.getY(), 1e-9);
        assertEquals(odo.getRotation().getRadians(), got.getRotation().getRadians(), 1e-9);
        assertFalse(loc.isCommitted());
    }

    @Test
    void setPoseSeedsFallbackButDoesNotCommit() {
        HypothesisBankLocalizer loc = localizer();
        Pose2d odo = p(5, 3, 0.3);
        Pose2d field = p(30, -20, 0.8);
        loc.setPose(field, odo);

        // The seed is the pre-commit fallback, so getPose returns it — but commit stays vision-driven
        // (a wrong prior must never read as "committed").
        assertFalse(loc.isCommitted());
        Pose2d got = loc.getPose(odo);
        assertEquals(field.getX(), got.getX(), 1e-6);
        assertEquals(field.getY(), got.getY(), 1e-6);
        assertEquals(field.getRotation().getRadians(), got.getRotation().getRadians(), 1e-6);
    }

    @Test
    void seededPoseRidesOdometry() {
        // After seeding, the datum is constant, so a later odometry moves the field pose by the same
        // rigid motion (datum ∘ odoNow).
        HypothesisBankLocalizer loc = localizer();
        Pose2d odo0 = p(0, 0, 0);
        Pose2d field0 = p(10, 5, 0.0);
        loc.setPose(field0, odo0);

        // Drive forward 4 in in odometry (heading unchanged): field pose advances 4 in in +x.
        Pose2d odo1 = p(4, 0, 0);
        Pose2d got = loc.getPose(odo1);
        assertEquals(14, got.getX(), 1e-6);
        assertEquals(5, got.getY(), 1e-6);
        assertEquals(0.0, got.getRotation().getRadians(), 1e-6);
    }

    @Test
    void invalidFrameIsANoOp() {
        HypothesisBankLocalizer loc = localizer();
        Pose2d odo = p(5, 3, 0.3);
        VisionFrame invalid = new VisionFrame(); // valid=false by default
        loc.update(1_000_000L, odo, 0.0, invalid);

        assertFalse(loc.isCommitted());
        assertEquals(0.0, loc.dominantWeight(), 1e-9);
        // getPose still falls back to odometry.
        assertEquals(odo.getX(), loc.getPose(odo).getX(), 1e-9);
    }
}
