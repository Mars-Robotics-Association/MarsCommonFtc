package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Test

/**
 * Tests the policy core's non-vision behavior — seeding, the pre-commit odometry fallback, and the
 * per-frame gating no-ops — with a trivial solver (the full PnP → branch-pose path is covered by
 * [VisionPoseSolverTest] and [PoseHypothesisBankTest]).
 */
class HypothesisBankLocalizerTest {

    companion object {
        private fun p(x: Double, y: Double, h: Double): Pose2d {
            return Pose2d(x, y, Rotation2d(h))
        }

        private fun localizer(): HypothesisBankLocalizer {
            // An empty tag table means any vision frame yields no branch poses — fine for these
            // tests,
            // which never feed a valid frame.
            val solver = VisionPoseSolver(Transform3D.identity(), HashMap())
            return HypothesisBankLocalizer(HypothesisBankLocalizer.Params(), solver)
        }
    }

    @Test
    fun emptyBankReturnsRawOdometry() {
        val loc = localizer()
        val odo = p(12.0, -7.0, 0.4)
        val got = loc.getPose(odo)
        assertEquals(odo.x, got.x, 1e-9)
        assertEquals(odo.y, got.y, 1e-9)
        assertEquals(odo.rotation.radians, got.rotation.radians, 1e-9)
        assertFalse(loc.isCommitted())
    }

    @Test
    fun setPoseSeedsFallbackButDoesNotCommit() {
        val loc = localizer()
        val odo = p(5.0, 3.0, 0.3)
        val field = p(30.0, -20.0, 0.8)
        loc.setPose(field, odo)

        // The seed is the pre-commit fallback, so getPose returns it — but commit stays
        // vision-driven
        // (a wrong prior must never read as "committed").
        assertFalse(loc.isCommitted())
        val got = loc.getPose(odo)
        assertEquals(field.x, got.x, 1e-6)
        assertEquals(field.y, got.y, 1e-6)
        assertEquals(field.rotation.radians, got.rotation.radians, 1e-6)
    }

    @Test
    fun seededPoseRidesOdometry() {
        // After seeding, the datum is constant, so a later odometry moves the field pose by the
        // same
        // rigid motion (datum ∘ odoNow).
        val loc = localizer()
        val odo0 = p(0.0, 0.0, 0.0)
        val field0 = p(10.0, 5.0, 0.0)
        loc.setPose(field0, odo0)

        // Drive forward 4 in in odometry (heading unchanged): field pose advances 4 in in +x.
        val odo1 = p(4.0, 0.0, 0.0)
        val got = loc.getPose(odo1)
        assertEquals(14.0, got.x, 1e-6)
        assertEquals(5.0, got.y, 1e-6)
        assertEquals(0.0, got.rotation.radians, 1e-6)
    }

    @Test
    fun invalidFrameIsANoOp() {
        val loc = localizer()
        val odo = p(5.0, 3.0, 0.3)
        val invalid = VisionFrame() // valid=false by default
        loc.update(1_000_000L, odo, 0.0, invalid)

        assertFalse(loc.isCommitted())
        assertEquals(0.0, loc.dominantWeight(), 1e-9)
        // getPose still falls back to odometry.
        assertEquals(odo.x, loc.getPose(odo).x, 1e-9)
    }
}
