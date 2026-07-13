package org.marsroboticsassociation.controllib.localization.vision;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Geometry wiring for the {@link VisionPoseSolver}: an explicit camera extrinsic plus an explicit
 * field tag-pose table. Both are <b>caller-supplied</b> — extrinsics are per-robot and the tag map
 * is per-field/season — so neither lives as a library default. TeamCode / quickstart owns the
 * numbers; this class owns only the frame conventions and helpers used to build them.
 *
 * <p>Use {@link #robotFromCameraFromLimelightRs} to convert Limelight-style forward / side / up
 * (metres) and roll / pitch / yaw (degrees) into the optical-frame extrinsic, {@link
 * #fromRowMajor4x4} to parse a Limelight {@code .fmap} 4×4 row, then pass both into {@link
 * #VisionPoseSolverConfig(Transform3D, Map)}.
 *
 * <p><b>Everything is in METRES and the Limelight's own robot/field frames.</b> A solver built from
 * this config produces a botpose in the same frame as Limelight's own MegaTag2, so its output is
 * directly comparable to the logged {@code vis_mt2*} column. Convert the result to inches at the
 * boundary exactly as the rest of the code does with {@code getBotpose()}.
 *
 * <h3>Convention — SOLVED (2026-06-25)</h3>
 *
 * The full chain that reproduces Limelight's MT1 6-DOF botpose is {@code robot = fieldFromTag ·
 * TAG_LOCAL_FIX · cameraFromTag⁻¹ · robotFromCamera⁻¹}, validated to ~1.7&nbsp;in&nbsp;/&nbsp;1.2°
 * against {@code t6rFs} on the 2026-06-25 close-range stationary captures (see {@link
 * #TAG_LOCAL_FIX} and {@code solveChainConvention}). The two pieces that took the longest to pin:
 *
 * <ul>
 *   <li><b>Extrinsic.</b> {@link #robotFromCameraFromLimelightRs} (the {@link
 *       #OPTICAL_TO_ROBOT_BASE} axis swap composed with the {@code rs*} pose) <i>was</i> right —
 *       the least-squares extrinsic recovered from the captures matches it. The earlier "naive
 *       chain leaves a 24&nbsp;in residual" finding was the missing tag-local relabel, not a bad
 *       extrinsic.
 *   <li><b>Tag-local frame.</b> Our PnP object points (+x right, +y up, +z out of the face) and the
 *       {@code .fmap} tag frame differ by the cardinal relabel {@link #TAG_LOCAL_FIX} {@code
 *       [-z,+x,-y]}. That was the whole gap. (Compare against MT1/{@code t6rFs}, never the
 *       heading-fed {@code vis_mt2}, per the ground-truth note in CLAUDE.md.)
 * </ul>
 */
public final class VisionPoseSolverConfig {

    /**
     * OpenCV optical frame (+x right, +y down, +z forward) → robot frame (+x forward, +y left, +z
     * up) axis swap, rotation only: optical&nbsp;z→robot&nbsp;x, optical&nbsp;x→robot&nbsp;−y,
     * optical&nbsp;y→robot&nbsp;−z.
     */
    public static final Transform3D OPTICAL_TO_ROBOT_BASE =
            new Transform3D(
                    new double[][] {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}, new double[] {0, 0, 0});

    /**
     * Tag-local frame relabel {@code [-z,+x,-y]} between our PnP object points ({@code +x} right,
     * {@code +y} up, {@code +z} out of the face) and the {@code .fmap} tag frame. Folded into
     * {@link #solver()} as {@code fieldFromTag · TAG_LOCAL_FIX · cameraFromTag⁻¹}; this is the
     * piece that makes the chain reproduce Limelight's MT1 6-DOF (validated to ~1.7 in / 1.2° on
     * the 2026-06-25 tag-20 + tag-24 stationary captures — see {@code solveChainConvention}).
     */
    public static final Transform3D TAG_LOCAL_FIX =
            new Transform3D(
                    new double[][] {{0, 0, -1}, {1, 0, 0}, {0, -1, 0}}, new double[] {0, 0, 0});

    private final Transform3D robotFromCamera;
    private final Map<Integer, Transform3D> tagFieldPoses;

    /**
     * @param robotFromCamera the camera optical frame expressed in the robot frame, in metres —
     *     typically {@link #robotFromCameraFromLimelightRs} applied to this robot's {@code .vpr}
     *     {@code rs*} values
     * @param tagFieldPoses {@code tagId → field←tag} table in metres (raw from a season {@code
     *     .fmap}); copied and frozen
     */
    public VisionPoseSolverConfig(
            Transform3D robotFromCamera, Map<Integer, Transform3D> tagFieldPoses) {
        this.robotFromCamera = robotFromCamera;
        this.tagFieldPoses = Collections.unmodifiableMap(new HashMap<>(tagFieldPoses));
    }

    /**
     * Builds the optical-frame extrinsic from Limelight's {@code rs*} "camera pose in robot space"
     * values: forward / side / up in metres, roll / pitch / yaw in degrees. The {@code rs*} pose
     * (robot-frame translation + Z-Y-X rotation) is composed after {@link #OPTICAL_TO_ROBOT_BASE}.
     *
     * <p>These numbers are robot-specific (mount height, pitch, roll 0 vs 180 for upside-down,
     * etc.). Source them from each robot's Limelight {@code .vpr} or a hand-eye solve — do not
     * hard-code them in shared library code.
     */
    public static Transform3D robotFromCameraFromLimelightRs(
            double forwardM,
            double sideM,
            double upM,
            double rollDeg,
            double pitchDeg,
            double yawDeg) {
        Transform3D mount =
                Transform3D.fromTranslationYPR(
                        forwardM,
                        sideM,
                        upM,
                        Math.toRadians(yawDeg),
                        Math.toRadians(pitchDeg),
                        Math.toRadians(rollDeg));
        return mount.multiply(OPTICAL_TO_ROBOT_BASE);
    }

    /**
     * Builds a {@link Transform3D} from a row-major 4×4 (rigid) transform as stored in a Limelight
     * {@code .fmap}; ignores the bottom row.
     */
    public static Transform3D fromRowMajor4x4(double[] m) {
        double[][] r = {
            {m[0], m[1], m[2]},
            {m[4], m[5], m[6]},
            {m[8], m[9], m[10]}
        };
        double[] t = {m[3], m[7], m[11]};
        return new Transform3D(r, t);
    }

    /** Camera optical frame expressed in the robot frame (metres). */
    public Transform3D robotFromCamera() {
        return robotFromCamera;
    }

    /** Unmodifiable {@code tagId → field←tag} table (metres). */
    public Map<Integer, Transform3D> tagFieldPoses() {
        return tagFieldPoses;
    }

    /**
     * A solver wired with this config's extrinsic, tag table, and the {@link #TAG_LOCAL_FIX
     * validated} tag-local relabel. Reproduces Limelight's MT1 6-DOF to ~1.7 in / 1.2° on clean
     * close-range captures when both the extrinsic and the tag map match the recording. The mirror
     * branch is built — {@link VisionPoseSolver#solveBranches} returns both IPPE solutions as full
     * 6-DOF poses and {@link VisionPoseSolver#admissibleBranch} disambiguates on the floor/level
     * physical priors (no heading).
     */
    public VisionPoseSolver solver() {
        return new VisionPoseSolver(robotFromCamera, tagFieldPoses, TAG_LOCAL_FIX);
    }
}
