package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;

import org.junit.jupiter.api.Test;

import java.util.HashMap;
import java.util.Map;

/**
 * OpenCV-free unit tests for the "DIY MegaTag2" transform chain and heading disambiguation. The
 * real {@code solvePnPGeneric} candidate recovery is OpenCV/Android-only and validated on-robot;
 * this pins the pure geometry around it.
 *
 * <p>Strategy: a <b>forward model</b> synthesizes the exact camera-relative tag pose a perfect
 * camera would see from a known robot pose ({@code cameraFromTag = (fieldFromRobot ·
 * robotFromCamera)^-1 · fieldFromTag}), then we assert {@link VisionPoseSolver#solve} inverts it
 * back to the robot pose. Because the chain only round-trips if {@link Transform3D#multiply}/{@link
 * Transform3D#inverse} are both correct, this exercises the whole math — and a separate
 * hand-computed case guards against a forward/inverse bug that could cancel.
 */
class VisionPoseSolverTest {

    private static final double TOL = 1e-9;
    private static final int TAG = 7;

    /** √½ — the exact w and z of a unit quaternion for a 90° rotation about +z. */
    private static final double ROOT_HALF = Math.sqrt(0.5);

    // --- Transform3D primitives (the chain rests on these) -----------------------------------

    @Test
    void inverseRoundTripsToIdentity() {
        Transform3D t = Transform3D.fromTranslationYPR(12, -5, 3, 0.7, -0.2, 0.4);
        Transform3D shouldBeI = t.multiply(t.inverse());
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertEquals(i == j ? 1.0 : 0.0, shouldBeI.r[i][j], TOL);
            }
            assertEquals(0.0, shouldBeI.t[i], TOL);
        }
    }

    @Test
    void yawComesFromYprYaw() {
        assertEquals(0.9, Transform3D.fromTranslationYPR(0, 0, 0, 0.9, 0.3, -0.1).yaw(), TOL);
    }

    @Test
    void rodriguesZeroVectorIsIdentityRotationWithTranslation() {
        Transform3D t = Transform3D.fromRodrigues(new double[] {0, 0, 0}, new double[] {1, 2, 3});
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertEquals(i == j ? 1.0 : 0.0, t.r[i][j], TOL);
            }
        }
        assertEquals(1.0, t.x(), TOL);
        assertEquals(2.0, t.y(), TOL);
        assertEquals(3.0, t.z(), TOL);
    }

    @Test
    void rodriguesAboutZMatchesYaw() {
        // A Rodrigues vector along +z of magnitude θ is a yaw of θ.
        double yaw = 0.42;
        Transform3D t = Transform3D.fromRodrigues(new double[] {0, 0, yaw}, new double[] {0, 0, 0});
        assertEquals(yaw, t.yaw(), TOL);
        assertEquals(Math.cos(yaw), t.r[0][0], TOL);
        assertEquals(-Math.sin(yaw), t.r[0][1], TOL);
    }

    @Test
    void rodriguesProducesAProperRotation() {
        // Arbitrary axis/angle → orthonormal, det +1 (a rotation, not a reflection/scale).
        Transform3D t =
                Transform3D.fromRodrigues(new double[] {0.3, -0.7, 1.1}, new double[] {0, 0, 0});
        Transform3D shouldBeI = t.multiply(t.inverse());
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                assertEquals(i == j ? 1.0 : 0.0, shouldBeI.r[i][j], 1e-12);
            }
        }
        double[][] r = t.r;
        double det =
                r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        assertEquals(1.0, det, 1e-12);
    }

    @Test
    void yawOnlyQuaternionMatchesYprYaw() {
        double yaw = 1.1;
        Transform3D q =
                Transform3D.fromTranslationQuaternion(
                        0, 0, 0, Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2));
        assertEquals(yaw, q.yaw(), TOL);
    }

    // --- Hand-computed sanity case ------------------------------------------------------------

    @Test
    void recoversOriginWithIdentityExtrinsicAndTagAhead() {
        // Camera == robot center (identity extrinsic), robot at the field origin facing +x, so the
        // camera frame coincides with the field frame. A tag 60 in ahead, axis-aligned, therefore
        // sits at camera coords (60, 0, 0) with identity rotation.
        VisionPoseSolver solver =
                new VisionPoseSolver(
                        Transform3D.identity(),
                        tagTable(Transform3D.fromTranslationYPR(60, 0, 0, 0, 0, 0)));
        Transform3D cameraFromTag = Transform3D.fromTranslationYPR(60, 0, 0, 0, 0, 0);

        VisionPoseSolver.Result r = solver.solve(TAG, new Transform3D[] {cameraFromTag}, 0.0);

        assertPose(0, 0, 0, r.fieldPose);
        assertEquals(0.0, r.yawResidualRad, TOL);
        assertEquals(0, r.chosenIndex);
    }

    // --- Forward-model round trips (arbitrary poses) ------------------------------------------

    @Test
    void recoversArbitraryPoseThroughNonTrivialExtrinsicAndTag() {
        Transform3D fieldFromRobot = Transform3D.fromTranslationYPR(84, 36, 0, 1.2, 0, 0);
        // A camera mounted forward-left, raised, yawed 15 deg and pitched up 10 deg on the robot.
        Transform3D robotFromCamera =
                Transform3D.fromTranslationYPR(7, 3, 9, Math.toRadians(15), Math.toRadians(-10), 0);
        Transform3D fieldFromTag =
                Transform3D.fromTranslationQuaternion(120, 50, 20, ROOT_HALF, 0, 0, ROOT_HALF);

        VisionPoseSolver solver = new VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag));
        Transform3D trueCam = forwardCameraFromTag(fieldFromRobot, robotFromCamera, fieldFromTag);

        VisionPoseSolver.Result r =
                solver.solve(TAG, new Transform3D[] {trueCam}, fieldFromRobot.yaw());

        assertPose(fieldFromRobot.x(), fieldFromRobot.y(), fieldFromRobot.yaw(), r.fieldPose);
        assertEquals(0.0, r.yawResidualRad, 1e-7);
    }

    @Test
    void picksHeadingConsistentCandidateOverFlip() {
        Transform3D robotFromCamera = Transform3D.fromTranslationYPR(6, 0, 8, 0, 0, 0);
        Transform3D fieldFromTag =
                Transform3D.fromTranslationQuaternion(120, 50, 20, ROOT_HALF, 0, 0, ROOT_HALF);
        VisionPoseSolver solver = new VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag));

        // The true pose (heading ~0.30 rad) and a decoy "flip" pose well away in heading (~2.7 rad).
        Transform3D trueRobot = Transform3D.fromTranslationYPR(70, 40, 0, 0.30, 0, 0);
        Transform3D decoyRobot = Transform3D.fromTranslationYPR(95, 15, 0, 2.70, 0, 0);
        Transform3D trueCam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag);
        Transform3D decoyCam = forwardCameraFromTag(decoyRobot, robotFromCamera, fieldFromTag);

        // Decoy first, so a correct pick can't be an artifact of preferring index 0.
        VisionPoseSolver.Result r =
                solver.solve(TAG, new Transform3D[] {decoyCam, trueCam}, trueRobot.yaw());

        assertEquals(1, r.chosenIndex);
        assertPose(trueRobot.x(), trueRobot.y(), trueRobot.yaw(), r.fieldPose);
        assertTrue(r.yawResidualRad < 1e-6, "residual should be tiny for the matching candidate");
    }

    @Test
    void substitutesFedHeadingRatherThanSolvedYaw() {
        // Build the candidate from a robot at heading exactly 0, but feed a heading 2 deg off. The
        // result must carry the *fed* heading (the "zero the rotational DOF" step), and the residual
        // must report the 2 deg gap against the solve's own yaw.
        Transform3D robotFromCamera = Transform3D.fromTranslationYPR(6, 0, 8, 0, 0, 0);
        Transform3D fieldFromTag =
                Transform3D.fromTranslationQuaternion(120, 50, 20, ROOT_HALF, 0, 0, ROOT_HALF);
        VisionPoseSolver solver = new VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag));

        Transform3D trueRobot = Transform3D.fromTranslationYPR(70, 40, 0, 0.0, 0, 0);
        Transform3D cam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag);
        double fedHeading = Math.toRadians(2);

        VisionPoseSolver.Result r = solver.solve(TAG, new Transform3D[] {cam}, fedHeading);

        assertEquals(fedHeading, r.fieldPose.getRotation().getRadians(), TOL);
        assertEquals(0.0, r.solvedYawRad, 1e-7);
        assertEquals(Math.toRadians(2), r.yawResidualRad, 1e-7);
    }

    // --- Mirror branch (full 6-DOF, heading-free disambiguation) ------------------------------

    @Test
    void solveBranchesReturnsBothFull6DofPoses() {
        Transform3D robotFromCamera = Transform3D.fromTranslationYPR(6, 0, 8, 0, 0, 0);
        Transform3D fieldFromTag =
                Transform3D.fromTranslationQuaternion(120, 50, 20, ROOT_HALF, 0, 0, ROOT_HALF);
        VisionPoseSolver solver = new VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag));

        // True branch: a level robot on the floor. Mirror: floating and tilted (a rotational flip).
        Transform3D trueRobot = Transform3D.fromTranslationYPR(70, 40, 0, 0.30, 0, 0);
        Transform3D mirrorRobot = Transform3D.fromTranslationYPR(95, 15, 9, 2.70, 0.5, 0);
        Transform3D trueCam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag);
        Transform3D mirrorCam = forwardCameraFromTag(mirrorRobot, robotFromCamera, fieldFromTag);

        VisionPoseSolver.Branch[] branches =
                solver.solveBranches(TAG, new Transform3D[] {trueCam, mirrorCam});

        assertEquals(2, branches.length);
        // Branch 0 reconstructs the level/on-floor robot to full 6-DOF.
        assertEquals(0, branches[0].index);
        assertEquals(70, branches[0].fieldFromRobot.x(), 1e-7);
        assertEquals(40, branches[0].fieldFromRobot.y(), 1e-7);
        assertEquals(0.30, branches[0].yawRad, 1e-7);
        assertEquals(0.0, branches[0].zOffset, 1e-7);
        assertEquals(0.0, branches[0].tiltRad, 1e-7);
        // Branch 1 reconstructs the floating/tilted mirror — admissibility diagnostics flag it.
        assertEquals(1, branches[1].index);
        assertEquals(9.0, branches[1].zOffset, 1e-7);
        assertEquals(0.5, branches[1].tiltRad, 1e-7);
    }

    @Test
    void admissibleBranchPicksTheLevelOneWithoutAHeading() {
        Transform3D robotFromCamera = Transform3D.fromTranslationYPR(6, 0, 8, 0, 0, 0);
        Transform3D fieldFromTag =
                Transform3D.fromTranslationQuaternion(120, 50, 20, ROOT_HALF, 0, 0, ROOT_HALF);
        VisionPoseSolver solver = new VisionPoseSolver(robotFromCamera, tagTable(fieldFromTag));

        Transform3D trueRobot = Transform3D.fromTranslationYPR(70, 40, 0, 0.30, 0, 0);
        Transform3D tiltedMirror = Transform3D.fromTranslationYPR(95, 15, 9, 2.70, 0.5, 0);
        Transform3D trueCam = forwardCameraFromTag(trueRobot, robotFromCamera, fieldFromTag);
        Transform3D mirrorCam = forwardCameraFromTag(tiltedMirror, robotFromCamera, fieldFromTag);

        // Mirror first, so the pick can't be an artifact of preferring index 0 — and no heading fed.
        VisionPoseSolver.Branch picked =
                solver.admissibleBranch(TAG, new Transform3D[] {mirrorCam, trueCam});

        assertEquals(1, picked.index);
        assertEquals(70, picked.fieldFromRobot.x(), 1e-7);
        assertEquals(40, picked.fieldFromRobot.y(), 1e-7);
        assertEquals(0.0, picked.tiltRad, 1e-7);
    }

    @Test
    void solveBranchesGuardsUnknownTagAndEmpty() {
        VisionPoseSolver solver =
                new VisionPoseSolver(Transform3D.identity(), tagTable(Transform3D.identity()));
        assertNull(solver.solveBranches(999, new Transform3D[] {Transform3D.identity()}));
        assertNull(solver.solveBranches(TAG, new Transform3D[] {}));
        assertNull(solver.solveBranches(TAG, null));
        assertNull(solver.admissibleBranch(TAG, new Transform3D[] {null}));
    }

    // --- Guards -------------------------------------------------------------------------------

    @Test
    void returnsNullForUnknownTag() {
        VisionPoseSolver solver =
                new VisionPoseSolver(Transform3D.identity(), tagTable(Transform3D.identity()));
        assertNull(solver.solve(999, new Transform3D[] {Transform3D.identity()}, 0.0));
    }

    @Test
    void returnsNullForNoCandidates() {
        VisionPoseSolver solver =
                new VisionPoseSolver(Transform3D.identity(), tagTable(Transform3D.identity()));
        assertNull(solver.solve(TAG, new Transform3D[] {}, 0.0));
        assertNull(solver.solve(TAG, null, 0.0));
    }

    // --- helpers ------------------------------------------------------------------------------

    /** The camera-relative tag pose a perfect camera at {@code fieldFromRobot} would observe. */
    private static Transform3D forwardCameraFromTag(
            Transform3D fieldFromRobot, Transform3D robotFromCamera, Transform3D fieldFromTag) {
        Transform3D fieldFromCamera = fieldFromRobot.multiply(robotFromCamera);
        return fieldFromCamera.inverse().multiply(fieldFromTag);
    }

    private static Map<Integer, Transform3D> tagTable(Transform3D fieldFromTag) {
        Map<Integer, Transform3D> m = new HashMap<>();
        m.put(TAG, fieldFromTag);
        return m;
    }

    private static void assertPose(double x, double y, double headingRad, Pose2d actual) {
        assertEquals(x, actual.getX(), 1e-7);
        assertEquals(y, actual.getY(), 1e-7);
        assertEquals(headingRad, actual.getRotation().getRadians(), 1e-7);
    }
}
