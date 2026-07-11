package org.marsroboticsassociation.controllib.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Map;

/**
 * vision pose solver: recovers a <b>flip-free</b> robot field pose from a single AprilTag by using
 * a known robot heading to disambiguate the planar-tag PnP flip. The tag flip is fundamentally a
 * <i>rotational</i> ambiguity — the two {@code SOLVEPNP_IPPE_SQUARE} solutions differ mainly in
 * tilt and place the camera at two different spots — so once the robot's yaw is known, the solution
 * inconsistent with it is rejected. That is exactly why Limelight's MegaTag2 never flips; this does
 * it ourselves so we can (a) feed the heading at the frame's <i>capture time</i> instead of
 * whatever stale value the camera last received, and (b) work entirely in our own field frame.
 *
 * <p>This class is the <b>pure</b> half (no OpenCV / Android): it consumes the 1–2 candidate
 * camera-relative tag poses from {@link TagAmbiguitySolver#solve} (convert each {@code rvec}/{@code
 * tvec} via {@link Transform3D#fromRodrigues}), plus the camera extrinsics and a tag field-pose
 * table, and runs the transform chain + disambiguation. Solve it for each candidate:
 *
 * <pre>{@code
 * fieldFromRobot = fieldFromTag · (cameraFromTag)^-1 · (robotFromCamera)^-1
 * }</pre>
 *
 * then keep the candidate whose recovered yaw is closest to the supplied heading, returning its (x,
 * y) with the heading itself substituted for the orientation (we trust the fed heading over the
 * single-tag solve — that is the "zero the rotational DOF" step).
 *
 * <h3>Caller responsibilities / gotchas</h3>
 *
 * <ul>
 *   <li><b>Corner order</b> matters for a pose (unlike the order-invariant ambiguity ratio), but it
 *       is already <b>confirmed correct</b>: the flip investigation verified our PnP reproduces
 *       Limelight's own per-tag pose ({@code t6t_cs}) to 2&nbsp;mm&nbsp;/&nbsp;1°, pose included,
 *       with Limelight's real distortion and a ZYX euler convention. No further corner-order
 *       calibration is needed.
 *   <li><b>Units.</b> The candidate translations and the tag table and the extrinsics must all be
 *       in the same length unit. {@link TagAmbiguitySolver#solve}'s {@code tvec}s are in metres,
 *       so build the tag table and extrinsic in metres too (see {@link VisionPoseSolverConfig}) and
 *       convert the output {@link Pose2d} to inches at the boundary.
 *   <li><b>Extrinsics own the convention change.</b> {@code robotFromCamera} expresses the camera's
 *       <i>optical</i> frame (OpenCV: +x right, +y down, +z along the optical axis) in the robot
 *       frame, so it encodes both the mount offset and the optical→robot axis swap. A bad extrinsic
 *       biases every solve. Derive it from Limelight's own per-tag pose streams ({@code T_cr =
 *       t6tRs · inv(t6tCs)}) — see {@link VisionPoseSolverConfig}.
 *   <li><b>Tag orientation.</b> {@code fieldFromTag} must put the tag's local frame ({@code +x}
 *       right across the face, {@code +y} up, {@code +z} <i>out of</i> the printed face toward the
 *       viewer) into the field frame. A vertical tag facing −x in the field has its +z pointing −x.
 * </ul>
 */
public final class VisionPoseSolver {

    /** Camera optical frame expressed in the robot frame (the extrinsic; see class javadoc). */
    private final Transform3D robotFromCamera;

    /**
     * tagId → that tag's frame expressed in the field frame (same length unit as everything else).
     */
    private final Map<Integer, Transform3D> tagFieldPoses;

    /**
     * Tag-local frame relabel between our PnP object points and the {@code .fmap} tag frame,
     * inserted as {@code fieldFromTag · tagLocalFix · cameraFromTag⁻¹}. Identity for raw use; the
     * {@link VisionPoseSolverConfig#TAG_LOCAL_FIX validated} {@code [-z,+x,-y]} relabel is what
     * makes the chain reproduce Limelight's MT1 (see {@code solveChainConvention}).
     */
    private final Transform3D tagLocalFix;

    /** Raw chain (no tag-local relabel) — for synthetic/unit use where frames already agree. */
    public VisionPoseSolver(Transform3D robotFromCamera, Map<Integer, Transform3D> tagFieldPoses) {
        this(robotFromCamera, tagFieldPoses, Transform3D.identity());
    }

    public VisionPoseSolver(
            Transform3D robotFromCamera,
            Map<Integer, Transform3D> tagFieldPoses,
            Transform3D tagLocalFix) {
        this.robotFromCamera = robotFromCamera;
        this.tagFieldPoses = tagFieldPoses;
        this.tagLocalFix = tagLocalFix;
    }

    /**
     * One IPPE solution resolved all the way to a <b>full 6-DOF</b> robot field pose, carrying the
     * <i>physical-admissibility</i> diagnostics that separate the true branch from its mirror
     * <b>without consulting any heading</b>. This is the building block for the mirror branch: the
     * planar-tag PnP flip is fundamentally a <i>rotational</i> ambiguity, so its two solutions
     * place the robot at two poses that differ mainly in tilt and height. The only admissible
     * single-frame discriminators are the a-priori physical truths — the robot <b>rolls on a flat
     * floor</b>, so it sits at {@code z≈0} ({@link #zOffset}) and {@code ≈level} ({@link #tiltRad});
     * a candidate that floats or tilts is inadmissible regardless of where it lands. No
     * held/estimated heading is treated as known-correct here.
     */
    public static final class Branch {
        /** Which IPPE solution this is (index into the candidate array). */
        public final int index;

        /**
         * The full 6-DOF robot pose in the field frame ({@code field←robot}), in the table unit.
         */
        public final Transform3D fieldFromRobot;

        /**
         * Height of the recovered robot origin above the floor, in the table length unit — {@code
         * |fieldFromRobot.z|}. The robot rolls on a flat floor, so the admissible branch has this
         * ≈0; the mirror typically floats well off it. A floor-admissibility check, not a heading.
         */
        public final double zOffset;

        /**
         * Angle between the recovered robot's local {@code +z} axis and the field {@code +z} (up),
         * in radians — {@code acos(R[2][2])}. The robot is level on the floor, so the admissible
         * branch has this ≈0; the mirror, being the reflected rotational solution, tilts. This is
         * the cleanest single-frame mirror separator because the flip is rotational.
         */
        public final double tiltRad;

        /** The branch's <i>own</i> solved field heading, {@code fieldFromRobot.yaw()} (radians). */
        public final double yawRad;

        Branch(int index, Transform3D fieldFromRobot) {
            this.index = index;
            this.fieldFromRobot = fieldFromRobot;
            this.zOffset = Math.abs(fieldFromRobot.z());
            this.tiltRad = tiltFromLevel(fieldFromRobot);
            this.yawRad = fieldFromRobot.yaw();
        }
    }

    /** The disambiguated field pose plus the diagnostics needed to tell whether to trust it. */
    public static final class Result {
        /** Robot field pose: (x, y) from the chosen candidate, heading = the supplied heading. */
        public final Pose2d fieldPose;

        /** The chosen candidate's <i>own</i> solved yaw — a cross-check against the fed heading. */
        public final double solvedYawRad;

        /**
         * Wrapped magnitude of (solvedYaw − fedHeading) for the chosen candidate, in radians. Small
         * = the solve and the heading agree (healthy). Large even after disambiguation = a bad
         * extrinsic / tag pose / heading; gate on it before fusing.
         */
        public final double yawResidualRad;

        /** Index of the chosen candidate (0 or 1) — which IPPE solution won. */
        public final int chosenIndex;

        Result(Pose2d fieldPose, double solvedYawRad, double yawResidualRad, int chosenIndex) {
            this.fieldPose = fieldPose;
            this.solvedYawRad = solvedYawRad;
            this.yawResidualRad = yawResidualRad;
            this.chosenIndex = chosenIndex;
        }
    }

    /**
     * Recovers the flip-free robot field pose from one tag's PnP candidates.
     *
     * @param tagId the anchoring tag's fiducial id; must be present in the field-pose table
     * @param cameraFromTagCandidates the 1–2 IPPE solutions, each the tag's pose <i>in the camera
     *     optical frame</i> (maps tag-frame points to camera-frame points), translation already in
     *     the same length unit as the tag table
     * @param fieldHeadingRad the trusted robot heading — the fused estimate sampled at the frame's
     *     capture time
     * @return the disambiguated pose + diagnostics, or null if the tag is unknown or no candidate
     *     was usable
     */
    public Result solve(int tagId, Transform3D[] cameraFromTagCandidates, double fieldHeadingRad) {
        Branch[] branches = solveBranches(tagId, cameraFromTagCandidates);
        if (branches == null) {
            return null;
        }
        Result best = null;
        double bestErr = Double.POSITIVE_INFINITY;
        for (Branch b : branches) {
            double err = Math.abs(wrap(b.yawRad - fieldHeadingRad));
            if (err < bestErr) {
                bestErr = err;
                best =
                        new Result(
                                new Pose2d(
                                        b.fieldFromRobot.x(),
                                        b.fieldFromRobot.y(),
                                        new Rotation2d(fieldHeadingRad)),
                                b.yawRad,
                                err,
                                b.index);
            }
        }
        return best;
    }

    /**
     * Builds <b>both</b> IPPE branches as full 6-DOF robot field poses — the MT1 pose and its
     * mirror — <b>consuming no heading</b>. Each candidate is run through the full chain
     *
     * <pre>{@code
     * fieldFromRobot = fieldFromTag · tagLocalFix · cameraFromTag⁻¹ · robotFromCamera⁻¹
     * }</pre>
     *
     * and returned as a {@link Branch} carrying the physical-admissibility diagnostics ({@link
     * Branch#zOffset floor} and {@link Branch#tiltRad level}). The caller disambiguates from those
     * priors (see {@link #admissibleBranch}) — or, if it must, from a heading it does not trust as
     * truth (see {@link #solve}). Returned in candidate order; nulls in the input are skipped.
     *
     * @return the 1–2 branches, or null if the tag is unknown or no candidate was usable
     */
    public Branch[] solveBranches(int tagId, Transform3D[] cameraFromTagCandidates) {
        Transform3D fieldFromTag = tagFieldPoses.get(tagId);
        if (fieldFromTag == null
                || cameraFromTagCandidates == null
                || cameraFromTagCandidates.length == 0) {
            return null;
        }
        Transform3D cameraFromRobot = robotFromCamera.inverse();

        Branch[] out = new Branch[cameraFromTagCandidates.length];
        int n = 0;
        for (int i = 0; i < cameraFromTagCandidates.length; i++) {
            Transform3D cameraFromTag = cameraFromTagCandidates[i];
            if (cameraFromTag == null) {
                continue;
            }
            // field ← tag ← (tag-local relabel) ← camera ← robot
            Transform3D fieldFromRobot =
                    fieldFromTag
                            .multiply(tagLocalFix)
                            .multiply(cameraFromTag.inverse())
                            .multiply(cameraFromRobot);
            out[n++] = new Branch(i, fieldFromRobot);
        }
        if (n == 0) {
            return null;
        }
        return n == out.length ? out : java.util.Arrays.copyOf(out, n);
    }

    /**
     * Picks the physically <b>admissible</b> branch of the mirror pair — the one most consistent
     * with a robot rolling on a flat floor — using <b>only</b> the a-priori physical priors, never
     * a heading. The robot is level on the floor, and the planar-tag flip is a <i>rotational</i>
     * ambiguity, so the mirror tilts the robot out of level; {@link Branch#tiltRad} is therefore
     * the cleanest separator and is the primary key here. (Height off the floor, {@link
     * Branch#zOffset}, is exposed for the same purpose but kept a diagnostic — it shares the
     * table's systematic vertical offset, so it is less reliable as the deciding key.)
     *
     * @return the admissible branch, or null if the tag is unknown or no candidate was usable
     */
    public Branch admissibleBranch(int tagId, Transform3D[] cameraFromTagCandidates) {
        Branch[] branches = solveBranches(tagId, cameraFromTagCandidates);
        if (branches == null) {
            return null;
        }
        Branch best = null;
        for (Branch b : branches) {
            if (best == null || b.tiltRad < best.tiltRad) {
                best = b;
            }
        }
        return best;
    }

    /**
     * Angle (radians) of the transform's local {@code +z} axis away from the field {@code +z} (up).
     * The local up axis is the third column of {@code R}, so its field-{@code z} component is
     * {@code R[2][2]} and the off-level angle is {@code acos(R[2][2])}. Zero = perfectly level.
     */
    private static double tiltFromLevel(Transform3D t) {
        return Math.acos(Math.max(-1.0, Math.min(1.0, t.r[2][2])));
    }

    /** Shortest signed angle equivalent of {@code a}, in (−π, π]. */
    private static double wrap(double a) {
        return Math.IEEEremainder(a, 2 * Math.PI);
    }
}
