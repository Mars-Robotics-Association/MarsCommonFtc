package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import java.util.ArrayList
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.acos
import kotlin.math.max
import kotlin.math.min

/**
 * vision pose solver: recovers a **flip-free** robot field pose from a single AprilTag by using a
 * known robot heading to disambiguate the planar-tag PnP flip. The tag flip is fundamentally a
 * *rotational* ambiguity — the two `SOLVEPNP_IPPE_SQUARE` solutions differ mainly in tilt and place
 * the camera at two different spots — so once the robot's yaw is known, the solution inconsistent
 * with it is rejected. That is exactly why Limelight's MegaTag2 never flips; this does it ourselves
 * so we can (a) feed the heading at the frame's *capture time* instead of whatever stale value the
 * camera last received, and (b) work entirely in our own field frame.
 *
 * <p>This class is the **pure** half (no OpenCV / Android): it consumes the 1–2 candidate
 * camera-relative tag poses from [TagAmbiguitySolver.solve] (convert each `rvec`/`tvec` via
 * [Transform3D.fromRodrigues]), plus the camera extrinsics and a tag field-pose table, and runs the
 * transform chain + disambiguation. Solve it for each candidate:
 * <pre>`fieldFromRobot = fieldFromTag · (cameraFromTag)^-1 · (robotFromCamera)^-1`</pre>
 *
 * then keep the candidate whose recovered yaw is closest to the supplied heading, returning its (x,
 * y) with the heading itself substituted for the orientation (we trust the fed heading over the
 * single-tag solve — that is the "zero the rotational DOF" step).
 *
 * <h3>Caller responsibilities / gotchas</h3>
 * <ul>
 * <li><b>Corner order</b> matters for a pose (unlike the order-invariant ambiguity ratio), but it
 *   is already <b>confirmed correct</b>: the flip investigation verified our PnP reproduces
 *   Limelight's own per-tag pose (`t6t_cs`) to 2 mm / 1°, pose included, with Limelight's real
 *   distortion and a ZYX euler convention. No further corner-order calibration is needed.
 * <li><b>Units.</b> The candidate translations and the tag table and the extrinsics must all be in
 *   the same length unit. [TagAmbiguitySolver.solve]'s `tvec`s are in metres, so build the tag
 *   table and extrinsic in metres too (see [VisionPoseSolverConfig]) and convert the output
 *   [Pose2d] to inches at the boundary.
 * <li><b>Extrinsics own the convention change.</b> `robotFromCamera` expresses the camera's
 *   *optical* frame (OpenCV: +x right, +y down, +z along the optical axis) in the robot frame, so
 *   it encodes both the mount offset and the optical→robot axis swap. A bad extrinsic biases every
 *   solve. Derive it from Limelight's own per-tag pose streams (`T_cr = t6tRs · inv(t6tCs)`) — see
 *   [VisionPoseSolverConfig].
 * <li><b>Tag orientation.</b> `fieldFromTag` must put the tag's local frame (`+x` right across the
 *   face, `+y` up, `+z` *out of* the printed face toward the viewer) into the field frame. A
 *   vertical tag facing −x in the field has its +z pointing −x.
 * </ul>
 */
class VisionPoseSolver
@JvmOverloads
constructor(
    /** Camera optical frame expressed in the robot frame (the extrinsic; see class javadoc). */
    private val robotFromCamera: Transform3D,
    /**
     * tagId → that tag's frame expressed in the field frame (same length unit as everything else).
     */
    private val tagFieldPoses: Map<Int, Transform3D>,
    /**
     * Tag-local frame relabel between our PnP object points and the `.fmap` tag frame, inserted as
     * `fieldFromTag · tagLocalFix · cameraFromTag⁻¹`. Identity for raw use; the
     * [VisionPoseSolverConfig.TAG_LOCAL_FIX] validated `[-z,+x,-y]` relabel is what makes the chain
     * reproduce Limelight's MT1 (see `solveChainConvention`).
     */
    private val tagLocalFix: Transform3D = Transform3D.identity(),
) {

    /**
     * One IPPE solution resolved all the way to a **full 6-DOF** robot field pose, carrying the
     * *physical-admissibility* diagnostics that separate the true branch from its mirror **without
     * consulting any heading**. This is the building block for the mirror branch: the planar-tag
     * PnP flip is fundamentally a *rotational* ambiguity, so its two solutions place the robot at
     * two poses that differ mainly in tilt and height. The only admissible single-frame
     * discriminators are the a-priori physical truths — the robot **rolls on a flat floor**, so it
     * sits at `z≈0` ([zOffset]) and `≈level` ([tiltRad]); a candidate that floats or tilts is
     * inadmissible regardless of where it lands. No held/estimated heading is treated as
     * known-correct here.
     */
    class Branch
    internal constructor(
        /** Which IPPE solution this is (index into the candidate array). */
        @JvmField val index: Int,
        /** The full 6-DOF robot pose in the field frame (`field←robot`), in the table unit. */
        @JvmField val fieldFromRobot: Transform3D,
    ) {
        /**
         * Height of the recovered robot origin above the floor, in the table length unit —
         * `|fieldFromRobot.z|`. The robot rolls on a flat floor, so the admissible branch has this
         * ≈0; the mirror typically floats well off it. A floor-admissibility check, not a heading.
         */
        @JvmField val zOffset: Double = abs(fieldFromRobot.z())

        /**
         * Angle between the recovered robot's local `+z` axis and the field `+z` (up), in radians —
         * `acos(R[2][2])`. The robot is level on the floor, so the admissible branch has this ≈0;
         * the mirror, being the reflected rotational solution, tilts. This is the cleanest
         * single-frame mirror separator because the flip is rotational.
         */
        @JvmField val tiltRad: Double = tiltFromLevel(fieldFromRobot)

        /** The branch's *own* solved field heading, `fieldFromRobot.yaw()` (radians). */
        @JvmField val yawRad: Double = fieldFromRobot.yaw()
    }

    /** The disambiguated field pose plus the diagnostics needed to tell whether to trust it. */
    class Result
    internal constructor(
        /** Robot field pose: (x, y) from the chosen candidate, heading = the supplied heading. */
        @JvmField val fieldPose: Pose2d,
        /** The chosen candidate's *own* solved yaw — a cross-check against the fed heading. */
        @JvmField val solvedYawRad: Double,
        /**
         * Wrapped magnitude of (solvedYaw − fedHeading) for the chosen candidate, in radians. Small
         * = the solve and the heading agree (healthy). Large even after disambiguation = a bad
         * extrinsic / tag pose / heading; gate on it before fusing.
         */
        @JvmField val yawResidualRad: Double,
        /** Index of the chosen candidate (0 or 1) — which IPPE solution won. */
        @JvmField val chosenIndex: Int,
    )

    /**
     * Recovers the flip-free robot field pose from one tag's PnP candidates.
     *
     * @param tagId the anchoring tag's fiducial id; must be present in the field-pose table
     * @param cameraFromTagCandidates the 1–2 IPPE solutions, each the tag's pose *in the camera
     *   optical frame* (maps tag-frame points to camera-frame points), translation already in the
     *   same length unit as the tag table
     * @param fieldHeadingRad the trusted robot heading — the fused estimate sampled at the frame's
     *   capture time
     * @return the disambiguated pose + diagnostics, or null if the tag is unknown or no candidate
     *   was usable
     */
    fun solve(
        tagId: Int,
        cameraFromTagCandidates: Array<out Transform3D?>?,
        fieldHeadingRad: Double,
    ): Result? {
        val branches = solveBranches(tagId, cameraFromTagCandidates) ?: return null
        var best: Result? = null
        var bestErr = Double.POSITIVE_INFINITY
        for (b in branches) {
            val err = abs(wrap(b.yawRad - fieldHeadingRad))
            if (err < bestErr) {
                bestErr = err
                best =
                    Result(
                        Pose2d(
                            b.fieldFromRobot.x(),
                            b.fieldFromRobot.y(),
                            Rotation2d(fieldHeadingRad),
                        ),
                        b.yawRad,
                        err,
                        b.index,
                    )
            }
        }
        return best
    }

    /**
     * Builds **both** IPPE branches as full 6-DOF robot field poses — the MT1 pose and its mirror —
     * **consuming no heading**. Each candidate is run through the full chain
     * <pre>`fieldFromRobot = fieldFromTag · tagLocalFix · cameraFromTag⁻¹ · robotFromCamera⁻¹`</pre>
     *
     * and returned as a [Branch] carrying the physical-admissibility diagnostics ([Branch.zOffset]
     * floor and [Branch.tiltRad] level). The caller disambiguates from those priors (see
     * [admissibleBranch]) — or, if it must, from a heading it does not trust as truth (see
     * [solve]). Returned in candidate order; nulls in the input are skipped.
     *
     * @return the 1–2 branches, or null if the tag is unknown or no candidate was usable
     */
    fun solveBranches(
        tagId: Int,
        cameraFromTagCandidates: Array<out Transform3D?>?,
    ): Array<Branch>? {
        val fieldFromTag = tagFieldPoses[tagId]
        if (
            fieldFromTag == null ||
                cameraFromTagCandidates == null ||
                cameraFromTagCandidates.isEmpty()
        ) {
            return null
        }
        val cameraFromRobot = robotFromCamera.inverse()

        val built = ArrayList<Branch>(cameraFromTagCandidates.size)
        for (i in cameraFromTagCandidates.indices) {
            val cameraFromTag = cameraFromTagCandidates[i] ?: continue
            // field ← tag ← (tag-local relabel) ← camera ← robot
            val fieldFromRobot =
                fieldFromTag
                    .multiply(tagLocalFix)
                    .multiply(cameraFromTag.inverse())
                    .multiply(cameraFromRobot)
            built.add(Branch(i, fieldFromRobot))
        }
        if (built.isEmpty()) {
            return null
        }
        return built.toTypedArray()
    }

    /**
     * Picks the physically **admissible** branch of the mirror pair — the one most consistent with
     * a robot rolling on a flat floor — using **only** the a-priori physical priors, never a
     * heading. The robot is level on the floor, and the planar-tag flip is a *rotational*
     * ambiguity, so the mirror tilts the robot out of level; [Branch.tiltRad] is therefore the
     * cleanest separator and is the primary key here. (Height off the floor, [Branch.zOffset], is
     * exposed for the same purpose but kept a diagnostic — it shares the table's systematic
     * vertical offset, so it is less reliable as the deciding key.)
     *
     * @return the admissible branch, or null if the tag is unknown or no candidate was usable
     */
    fun admissibleBranch(tagId: Int, cameraFromTagCandidates: Array<out Transform3D?>?): Branch? {
        val branches = solveBranches(tagId, cameraFromTagCandidates) ?: return null
        var best: Branch? = null
        for (b in branches) {
            if (best == null || b.tiltRad < best.tiltRad) {
                best = b
            }
        }
        return best
    }

    companion object {
        /**
         * Angle (radians) of the transform's local `+z` axis away from the field `+z` (up). The
         * local up axis is the third column of `R`, so its field-`z` component is `R[2][2]` and the
         * off-level angle is `acos(R[2][2])`. Zero = perfectly level.
         */
        private fun tiltFromLevel(t: Transform3D): Double {
            return acos(max(-1.0, min(1.0, t.r[2][2])))
        }

        /** Shortest signed angle equivalent of `a`, in (−π, π]. */
        private fun wrap(a: Double): Double {
            return Math.IEEEremainder(a, 2 * PI)
        }
    }
}
