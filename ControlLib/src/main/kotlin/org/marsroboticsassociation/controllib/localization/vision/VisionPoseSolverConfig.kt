package org.marsroboticsassociation.controllib.localization.vision

/**
 * Geometry wiring for the [VisionPoseSolver]: an explicit camera extrinsic plus an explicit field
 * tag-pose table. Both are **caller-supplied** — extrinsics are per-robot and the tag map is
 * per-field/season — so neither lives as a library default. TeamCode / quickstart owns the numbers;
 * this class owns only the frame conventions and helpers used to build them.
 *
 * <p>Use [robotFromCameraFromLimelightRs] to convert Limelight-style forward / side / up (metres)
 * and roll / pitch / yaw (degrees) into the optical-frame extrinsic, [fromRowMajor4x4] to parse a
 * Limelight `.fmap` 4×4 row, then pass both into the constructor.
 *
 * <p>**Everything is in METRES and the Limelight's own robot/field frames.** A solver built from
 * this config produces a botpose in the same frame as Limelight's own MegaTag2, so its output is
 * directly comparable to the logged `vis_mt2*` column. Convert the result to inches at the boundary
 * exactly as the rest of the code does with `getBotpose()`.
 *
 * <h3>Convention — SOLVED (2026-06-25)</h3>
 *
 * The full chain that reproduces Limelight's MT1 6-DOF botpose is `robot = fieldFromTag ·
 * TAG_LOCAL_FIX · cameraFromTag⁻¹ · robotFromCamera⁻¹`, validated to ~1.7 in / 1.2° against `t6rFs`
 * on the 2026-06-25 close-range stationary captures (see [TAG_LOCAL_FIX] and
 * `solveChainConvention`). The two pieces that took the longest to pin:
 * <ul>
 * <li><b>Extrinsic.</b> [robotFromCameraFromLimelightRs] (the [OPTICAL_TO_ROBOT_BASE] axis swap
 *   composed with the `rs*` pose) *was* right — the least-squares extrinsic recovered from the
 *   captures matches it. The earlier "naive chain leaves a 24 in residual" finding was the missing
 *   tag-local relabel, not a bad extrinsic.
 * <li><b>Tag-local frame.</b> Our PnP object points (+x right, +y up, +z out of the face) and the
 *   `.fmap` tag frame differ by the cardinal relabel [TAG_LOCAL_FIX] `[-z,+x,-y]`. That was the
 *   whole gap. (Compare against MT1/`t6rFs`, never the heading-fed `vis_mt2`, per the ground-truth
 *   note in CLAUDE.md.)
 * </ul>
 */
class VisionPoseSolverConfig(
    robotFromCamera: Transform3D,
    tagFieldPoses: Map<Int, Transform3D>,
) {
    private val robotFromCameraExtrinsic: Transform3D = robotFromCamera
    private val tagFieldPosesTable: Map<Int, Transform3D> = tagFieldPoses.toMap()

    /** Camera optical frame expressed in the robot frame (metres). */
    fun robotFromCamera(): Transform3D = robotFromCameraExtrinsic

    /** Unmodifiable `tagId → field←tag` table (metres). */
    fun tagFieldPoses(): Map<Int, Transform3D> = tagFieldPosesTable

    /**
     * A solver wired with this config's extrinsic, tag table, and the [TAG_LOCAL_FIX] validated
     * tag-local relabel. Reproduces Limelight's MT1 6-DOF to ~1.7 in / 1.2° on clean close-range
     * captures when both the extrinsic and the tag map match the recording. The mirror branch is
     * built — [VisionPoseSolver.solveBranches] returns both IPPE solutions as full 6-DOF poses and
     * [VisionPoseSolver.admissibleBranch] disambiguates on the floor/level physical priors (no
     * heading).
     */
    fun solver(): VisionPoseSolver {
        return VisionPoseSolver(robotFromCameraExtrinsic, tagFieldPosesTable, TAG_LOCAL_FIX)
    }

    companion object {
        /**
         * OpenCV optical frame (+x right, +y down, +z forward) → robot frame (+x forward, +y left,
         * +z up) axis swap, rotation only: optical z→robot x, optical x→robot −y, optical y→robot
         * −z.
         */
        @JvmField
        val OPTICAL_TO_ROBOT_BASE: Transform3D =
            Transform3D(
                arrayOf(
                    doubleArrayOf(0.0, 0.0, 1.0),
                    doubleArrayOf(-1.0, 0.0, 0.0),
                    doubleArrayOf(0.0, -1.0, 0.0),
                ),
                doubleArrayOf(0.0, 0.0, 0.0),
            )

        /**
         * Tag-local frame relabel `[-z,+x,-y]` between our PnP object points (`+x` right, `+y` up,
         * `+z` out of the face) and the `.fmap` tag frame. Folded into [solver] as `fieldFromTag ·
         * TAG_LOCAL_FIX · cameraFromTag⁻¹`; this is the piece that makes the chain reproduce
         * Limelight's MT1 6-DOF (validated to ~1.7 in / 1.2° on the 2026-06-25 tag-20 + tag-24
         * stationary captures — see `solveChainConvention`).
         */
        @JvmField
        val TAG_LOCAL_FIX: Transform3D =
            Transform3D(
                arrayOf(
                    doubleArrayOf(0.0, 0.0, -1.0),
                    doubleArrayOf(1.0, 0.0, 0.0),
                    doubleArrayOf(0.0, -1.0, 0.0),
                ),
                doubleArrayOf(0.0, 0.0, 0.0),
            )

        /**
         * Builds the optical-frame extrinsic from Limelight's `rs*` "camera pose in robot space"
         * values: forward / side / up in metres, roll / pitch / yaw in degrees. The `rs*` pose
         * (robot-frame translation + Z-Y-X rotation) is composed after [OPTICAL_TO_ROBOT_BASE].
         *
         * <p>These numbers are robot-specific (mount height, pitch, roll 0 vs 180 for upside-down,
         * etc.). Source them from each robot's Limelight `.vpr` or a hand-eye solve — do not
         * hard-code them in shared library code.
         */
        @JvmStatic
        fun robotFromCameraFromLimelightRs(
            forwardM: Double,
            sideM: Double,
            upM: Double,
            rollDeg: Double,
            pitchDeg: Double,
            yawDeg: Double,
        ): Transform3D {
            val mount =
                Transform3D.fromTranslationYPR(
                    forwardM,
                    sideM,
                    upM,
                    Math.toRadians(yawDeg),
                    Math.toRadians(pitchDeg),
                    Math.toRadians(rollDeg),
                )
            return mount.multiply(OPTICAL_TO_ROBOT_BASE)
        }

        /**
         * Builds a [Transform3D] from a row-major 4×4 (rigid) transform as stored in a Limelight
         * `.fmap`; ignores the bottom row.
         */
        @JvmStatic
        fun fromRowMajor4x4(m: DoubleArray): Transform3D {
            val r =
                arrayOf(
                    doubleArrayOf(m[0], m[1], m[2]),
                    doubleArrayOf(m[4], m[5], m[6]),
                    doubleArrayOf(m[8], m[9], m[10]),
                )
            val t = doubleArrayOf(m[3], m[7], m[11])
            return Transform3D(r, t)
        }
    }
}
