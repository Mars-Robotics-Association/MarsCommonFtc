package org.marsroboticsassociation.controllib.localization.vision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

import org.junit.jupiter.api.Test;

/**
 * Pure (OpenCV-free) checks that {@link VisionPoseSolverConfig} composes a Limelight {@code rs*}
 * extrinsic and freezes a caller-supplied tag table — guarding against an index typo in the 4×4
 * extraction or a transposed axis. Does not assert the extrinsic <i>rotation</i> convention.
 * Fixture numbers below are one robot/field pair (Curiosity DECODE); real robots/seasons supply
 * their own from TeamCode.
 */
class VisionPoseSolverConfigTest {

    private static final double TOL = 1e-9;

    // Example fixture only — not a library default. Matches curiosity_Limelight (5).vpr / hand-eye.
    private static final double FIXTURE_FORWARD_M = 0.0816;
    private static final double FIXTURE_SIDE_M = -0.0400;
    private static final double FIXTURE_UP_M = 0.3835;
    private static final double FIXTURE_ROLL_DEG = 180.0;
    private static final double FIXTURE_PITCH_DEG = 0.23;
    private static final double FIXTURE_YAW_DEG = -0.78;

    // ftc2025DECODE.fmap goal tags (fixture for table-parse tests).
    private static final double[] TAG20_FIELD_FROM_TAG = {
        0.5877852522924731,
        -0.8090169943749473,
        0,
        -1.4827,
        0.8090169943749473,
        0.5877852522924731,
        0,
        -1.4133,
        0,
        0,
        1,
        0.7493,
        0,
        0,
        0,
        1
    };
    private static final double[] TAG24_FIELD_FROM_TAG = {
        0.5877852522924731,
        0.8090169943749473,
        0,
        -1.4827,
        -0.8090169943749473,
        0.5877852522924731,
        0,
        1.4133,
        0,
        0,
        1,
        0.7493,
        0,
        0,
        0,
        1
    };

    private static Transform3D fixtureExtrinsic() {
        return VisionPoseSolverConfig.robotFromCameraFromLimelightRs(
                FIXTURE_FORWARD_M,
                FIXTURE_SIDE_M,
                FIXTURE_UP_M,
                FIXTURE_ROLL_DEG,
                FIXTURE_PITCH_DEG,
                FIXTURE_YAW_DEG);
    }

    private static Map<Integer, Transform3D> fixtureTagMap() {
        Map<Integer, Transform3D> m = new HashMap<>();
        m.put(20, VisionPoseSolverConfig.fromRowMajor4x4(TAG20_FIELD_FROM_TAG));
        m.put(24, VisionPoseSolverConfig.fromRowMajor4x4(TAG24_FIELD_FROM_TAG));
        return Collections.unmodifiableMap(m);
    }

    private static VisionPoseSolverConfig fixtureConfig() {
        return new VisionPoseSolverConfig(fixtureExtrinsic(), fixtureTagMap());
    }

    @Test
    void tagTableHoldsBothDecodeGoalTags() {
        VisionPoseSolverConfig cfg = fixtureConfig();
        assertTrue(cfg.tagFieldPoses().containsKey(20));
        assertTrue(cfg.tagFieldPoses().containsKey(24));
        assertEquals(2, cfg.tagFieldPoses().size());
    }

    @Test
    void tag20PositionAndYawMatchFmap() {
        Transform3D t = fixtureConfig().tagFieldPoses().get(20);
        assertEquals(-1.4827, t.x(), TOL);
        assertEquals(-1.4133, t.y(), TOL);
        assertEquals(0.7493, t.z(), TOL); // tag height, metres
        assertEquals(Math.toRadians(54), t.yaw(), 1e-6);
    }

    @Test
    void tag24MirrorsTag20InYAndYaw() {
        Transform3D t = fixtureConfig().tagFieldPoses().get(24);
        assertEquals(-1.4827, t.x(), TOL);
        assertEquals(1.4133, t.y(), TOL);
        assertEquals(Math.toRadians(-54), t.yaw(), 1e-6);
    }

    @Test
    void limelightRsTranslationMatchesInput() {
        // The translation is an exact fact (unlike the rotation hypothesis): forward/side/up metres.
        Transform3D e = fixtureExtrinsic();
        assertEquals(FIXTURE_FORWARD_M, e.x(), TOL);
        assertEquals(FIXTURE_SIDE_M, e.y(), TOL);
        assertEquals(FIXTURE_UP_M, e.z(), TOL);
    }

    @Test
    void fromRowMajor4x4ExtractsTranslationAndRotationBlock() {
        Transform3D t = VisionPoseSolverConfig.fromRowMajor4x4(TAG20_FIELD_FROM_TAG);
        assertEquals(-1.4827, t.x(), TOL);
        assertEquals(-1.4133, t.y(), TOL);
        assertEquals(0.7493, t.z(), TOL);
        assertEquals(0.5877852522924731, t.r[0][0], TOL);
        assertEquals(-0.8090169943749473, t.r[0][1], TOL);
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
