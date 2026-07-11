package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Pure (OpenCV-free) checks that {@link VisionPoseSolverConfig} parses the {@code .fmap} tag table
 * and {@code .vpr} extrinsic into the right numbers — guarding against an index typo in the 4×4
 * extraction or a transposed {@code rs*} value. Does not assert the extrinsic <i>rotation</i>
 * convention. (These example values are the Curiosity DECODE goal tags; replace the config for your
 * field/robot.)
 */
class VisionPoseSolverConfigTest {

    private static final double TOL = 1e-9;

    @Test
    void tagTableHoldsBothDecodeGoalTags() {
        VisionPoseSolverConfig cfg = new VisionPoseSolverConfig();
        assertTrue(cfg.tagFieldPoses().containsKey(20));
        assertTrue(cfg.tagFieldPoses().containsKey(24));
        assertEquals(2, cfg.tagFieldPoses().size());
    }

    @Test
    void tag20PositionAndYawMatchFmap() {
        Transform3D t = new VisionPoseSolverConfig().tagFieldPoses().get(20);
        assertEquals(-1.4827, t.x(), TOL);
        assertEquals(-1.4133, t.y(), TOL);
        assertEquals(0.7493, t.z(), TOL); // tag height, metres
        assertEquals(Math.toRadians(54), t.yaw(), 1e-6);
    }

    @Test
    void tag24MirrorsTag20InYAndYaw() {
        Transform3D t = new VisionPoseSolverConfig().tagFieldPoses().get(24);
        assertEquals(-1.4827, t.x(), TOL);
        assertEquals(1.4133, t.y(), TOL);
        assertEquals(Math.toRadians(-54), t.yaw(), 1e-6);
    }

    @Test
    void extrinsicTranslationMatchesVpr() {
        // The translation is an exact fact (unlike the rotation hypothesis): forward/side/up metres.
        Transform3D e = VisionPoseSolverConfig.defaultRobotFromCamera();
        assertEquals(VisionPoseSolverConfig.RS_FORWARD_M, e.x(), TOL);
        assertEquals(VisionPoseSolverConfig.RS_SIDE_M, e.y(), TOL);
        assertEquals(VisionPoseSolverConfig.RS_UP_M, e.z(), TOL);
    }

    @Test
    void opticalToRobotBaseIsAProperRotation() {
        // det = +1 and orthonormal: a real rotation, not a reflection (a reflection would silently
        // mirror every recovered pose).
        Transform3D b = VisionPoseSolverConfig.OPTICAL_TO_ROBOT_BASE;
        double[][] r = b.r;
        double det =
                r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                        - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                        + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
        assertEquals(1.0, det, TOL);
    }
}
