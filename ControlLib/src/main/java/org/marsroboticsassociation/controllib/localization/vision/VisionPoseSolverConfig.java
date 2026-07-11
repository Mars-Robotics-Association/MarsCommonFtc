package org.marsroboticsassociation.controllib.localization.vision;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * Geometry config for the {@link VisionPoseSolver vision pose solver}, sourced verbatim from the
 * Limelight config files in {@code /limelight}: the camera extrinsic from {@code
 * curiosity_Limelight.vpr} (its {@code rs*} "camera pose in robot space") and the tag field-pose
 * table from {@code ftc2025DECODE.fmap}.
 *
 * <p><b>Everything here is in METRES and the Limelight's own robot/field frames.</b> That is
 * deliberate: a solver built from this config produces a botpose in the same frame as Limelight's
 * own MegaTag2, so its output is directly comparable to the logged {@code vis_mt2*} column. Convert
 * the result to inches at the boundary exactly as the rest of the code does with {@code
 * getBotpose()}.
 *
 * <h3>Provenance — exact values copied from the config files</h3>
 *
 * <ul>
 *   <li><b>Extrinsic</b> (hand-eye over the 2026-06-19/20 competition runs, seeded from {@code
 *       curiosity_Limelight (5).vpr}): forward {@value #RS_FORWARD_M} m, side {@value #RS_SIDE_M}
 *       m, up {@value #RS_UP_M} m; roll {@value #RS_ROLL_DEG}°, pitch {@value #RS_PITCH_DEG}°, yaw
 *       {@value #RS_YAW_DEG}° — a camera ~15 in up, mounted upside-down (roll 180), essentially
 *       level and forward-facing. FORWARD/SIDE/UP/YAW come from the hand-eye solve
 *       (well-constrained axes, UP pinned 75% (4)→(5)); ROLL/PITCH are kept at vpr(5) because
 *       hand-eye barely beats the seed there and its pitch is field-scattered. See {@link #RS_UP_M}
 *       and {@code handEyeSolve}.
 *   <li><b>Tag table</b> ({@code .fmap}): DECODE goal tags 20 and 24, each a {@code field←tag} 4×4
 *       row-major transform in metres.
 * </ul>
 *
 * <h3>Convention — SOLVED (2026-06-25)</h3>
 *
 * The full chain that reproduces Limelight's MT1 6-DOF botpose is {@code robot = fieldFromTag ·
 * TAG_LOCAL_FIX · cameraFromTag⁻¹ · robotFromCamera⁻¹}, validated to ~1.7&nbsp;in&nbsp;/&nbsp;1.2°
 * against {@code t6rFs} on the 2026-06-25 close-range stationary captures (see {@link
 * #TAG_LOCAL_FIX} and {@code solveChainConvention}). The two pieces that took the longest to pin:
 *
 * <ul>
 *   <li><b>Extrinsic.</b> {@link #defaultRobotFromCamera()} (the {@link #OPTICAL_TO_ROBOT_BASE}
 *       axis swap composed with the {@code rs*} pose) <i>was</i> right — the least-squares
 *       extrinsic recovered from the captures matches it. The earlier "naive chain leaves a
 *       24&nbsp;in residual" finding was the missing tag-local relabel, not a bad extrinsic.
 *   <li><b>Tag-local frame.</b> Our PnP object points (+x right, +y up, +z out of the face) and the
 *       {@code .fmap} tag frame differ by the cardinal relabel {@link #TAG_LOCAL_FIX} {@code
 *       [-z,+x,-y]}. That was the whole gap. (Compare against MT1/{@code t6rFs}, never the
 *       heading-fed {@code vis_mt2}, per the ground-truth note in CLAUDE.md.)
 * </ul>
 */
public final class VisionPoseSolverConfig {

    // --- extrinsic: hand-eye over the 2026-06-19/20 competition runs, seeded from vpr(5)
    // ----------
    // FORWARD/SIDE/UP/YAW are the hand-eye solve over the multi-field competition set (0619+0620
    // together; -Dmt2diy.glob=_20260619,_20260620), decomposed back to the mount frame. These are
    // the
    // well-constrained axes. PITCH/ROLL are KEPT at vpr(5) on purpose: hand-eye barely beats the
    // vpr
    // seed there (meanCost 8.53→8.45) and pitch disagreed ~9° between the two competition days
    // (−8.6° vs +0.2°), so its combined −4.7° is scatter, not signal — baking it would inject
    // noise.
    public static final double RS_FORWARD_M = 0.0816;
    public static final double RS_SIDE_M = -0.0400;
    // Camera height is field-independent but hand-eye CANNOT observe it (planar floor motion is
    // vertically degenerate), so the solve can't adjudicate vpr4=0.361 vs vpr5=0.391. Pinned at 75%
    // of the way (4)→(5) = 0.3835 m as a prior leaning toward the better-tuned vpr(5) (whose yaw
    // the
    // competition data confirms); replace with a ruler measurement when available. -Dmt2diy.upM
    // overrides the pin in handEyeSolve.
    public static final double RS_UP_M = 0.3835;
    public static final double RS_ROLL_DEG = 180.0; // kept: vpr(5) — see FORWARD note
    public static final double RS_PITCH_DEG = 0.23; // kept: vpr(5) — hand-eye pitch is noise
    public static final double RS_YAW_DEG = -0.78;

    // --- ftc2025DECODE.fmap : field←tag, 4×4 row-major, metres ----------------------------------
    // 2×2 rotation block is a pure yaw of ±54°; z is the tag height (0.7493 m).
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

    /**
     * OpenCV optical frame (+x right, +y down, +z forward) → robot frame (+x forward, +y left, +z
     * up) axis swap, rotation only: optical&nbsp;z→robot&nbsp;x, optical&nbsp;x→robot&nbsp;−y,
     * optical&nbsp;y→robot&nbsp;−z. <b>Hypothesis</b> — see the class-level convention note.
     */
    public static final Transform3D OPTICAL_TO_ROBOT_BASE =
            new Transform3D(
                    new double[][] {{0, 0, 1}, {-1, 0, 0}, {0, -1, 0}}, new double[] {0, 0, 0});

    // CONVENTION SOLVED (2026-06-25), NOT yet wired into solver(). On clean close-range stationary
    // captures (tag 20 + tag 24, hand-tuned .vpr) the full chain that reproduces Limelight's MT1
    // 6-DOF botpose is:
    //     robot = fieldFromTag · T_tag · cameraFromTag⁻¹ · robotFromCamera⁻¹
    // where cameraFromTag is our PnP (== LL t6tCs to 0.0003 m), robotFromCamera is the .vpr
    // extrinsic
    // (defaultRobotFromCamera()), and T_tag = [-z,+x,-y] is the tag-local frame relabel between our
    // PnP object points and the .fmap tag frame. This reproduces t6rFs to ~3 in / ~2.7° across both
    // tags (see VisionPoseSolverAnalysisTest#solveChainConvention). Earlier "Ry(-90)" / field-frame
    // [Rz(180),(tx,ty)] claims were RETRACTED branch-shopping artifacts; the real gap was T_tag,
    // not
    // the extrinsic or the map. NOTE: solver() folds the T_tag relabel in and now builds the mirror
    // branch (solveBranches/admissibleBranch); the floor/yaw snap remains before relying on a
    // single
    // wired-live pose.

    private final Transform3D robotFromCamera;
    private final Map<Integer, Transform3D> tagFieldPoses;

    /** Uses the {@link #defaultRobotFromCamera() hypothesised} extrinsic from the {@code .vpr}. */
    public VisionPoseSolverConfig() {
        this(defaultRobotFromCamera());
    }

    /**
     * @param robotFromCamera the camera optical frame expressed in the robot frame, in metres —
     *     pass a corrected transform here once {@code vis_mt2*} validation pins down the convention
     */
    public VisionPoseSolverConfig(Transform3D robotFromCamera) {
        this.robotFromCamera = robotFromCamera;
        // RAW .fmap table (tag z = up), used everywhere — the floor-admissibility guard
        // (candidateOutOfPlane) is tuned around its systematic offset, and the frame convention
        // that
        // reproduces MT1 (the T_tag relabel, see the note above) is not yet folded into solver(),
        // so
        // this table is kept raw to avoid a half-applied correction.
        Map<Integer, Transform3D> m = new HashMap<>();
        m.put(20, fromRowMajor4x4(TAG20_FIELD_FROM_TAG));
        m.put(24, fromRowMajor4x4(TAG24_FIELD_FROM_TAG));
        this.tagFieldPoses = Collections.unmodifiableMap(m);
    }

    /**
     * The best-effort extrinsic from the {@code .vpr}: the {@code rs*} pose (as a robot-frame
     * translation + Z-Y-X rotation) applied after the {@link #OPTICAL_TO_ROBOT_BASE} axis swap.
     * Metres. See the class-level convention note — this is the piece to confirm/correct against
     * {@code vis_mt2*}.
     */
    public static Transform3D defaultRobotFromCamera() {
        Transform3D mount =
                Transform3D.fromTranslationYPR(
                        RS_FORWARD_M,
                        RS_SIDE_M,
                        RS_UP_M,
                        Math.toRadians(RS_YAW_DEG),
                        Math.toRadians(RS_PITCH_DEG),
                        Math.toRadians(RS_ROLL_DEG));
        return mount.multiply(OPTICAL_TO_ROBOT_BASE);
    }

    /** Camera optical frame expressed in the robot frame (metres). */
    public Transform3D robotFromCamera() {
        return robotFromCamera;
    }

    /** Unmodifiable {@code tagId → field←tag} table (metres), raw from the {@code .fmap}. */
    public Map<Integer, Transform3D> tagFieldPoses() {
        return tagFieldPoses;
    }

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

    /**
     * A solver wired with this config's extrinsic, the raw {@code .fmap} table, and the {@link
     * #TAG_LOCAL_FIX validated} tag-local relabel. Reproduces Limelight's MT1 6-DOF to ~1.7 in /
     * 1.2° on clean close-range captures. The mirror branch is built — {@link
     * VisionPoseSolver#solveBranches} returns both IPPE solutions as full 6-DOF poses and {@link
     * VisionPoseSolver#admissibleBranch} disambiguates on the floor/level physical priors (no
     * heading). The floor/yaw snap is the remaining piece before wiring it live.
     */
    public VisionPoseSolver solver() {
        return new VisionPoseSolver(robotFromCamera, tagFieldPoses, TAG_LOCAL_FIX);
    }

    /**
     * Builds a {@link Transform3D} from a row-major 4×4 (rigid) transform; ignores the bottom row.
     */
    static Transform3D fromRowMajor4x4(double[] m) {
        double[][] r = {
            {m[0], m[1], m[2]},
            {m[4], m[5], m[6]},
            {m[8], m[9], m[10]}
        };
        double[] t = {m[3], m[7], m[11]};
        return new Transform3D(r, t);
    }
}
