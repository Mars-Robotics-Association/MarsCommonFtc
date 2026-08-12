package org.marsroboticsassociation.controllib.localization.vision.replay

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import java.io.IOException
import java.nio.file.Path
import kotlin.math.max
import org.marsroboticsassociation.controllib.localization.vision.HypothesisBankLocalizer
import org.marsroboticsassociation.controllib.localization.vision.PlanarPnpSolver
import org.marsroboticsassociation.controllib.localization.vision.TagAmbiguitySolver
import org.marsroboticsassociation.controllib.localization.vision.VisionFrame
import org.marsroboticsassociation.controllib.localization.vision.VisionPoseSolver

/**
 * Replays one recorded telemetry CSV through the pure [HypothesisBankLocalizer] off-robot: each row
 * reconstructs the [VisionFrame] (via [CsvVisionSource]) and feeds the recorded absolute odometry
 * straight into `update()` — no odometry-integration replay is needed because the bank consumes
 * field-frame odometry directly. Returns the replayed vs. recorded fused trajectory so a harness
 * can (a) assert byte-fidelity under the recording's own params, or (b) measure how a *different*
 * configuration (bank params, solver, or PnP method) would have behaved on the same sensor stream.
 *
 * <h3>Testing a different PnP method</h3>
 *
 * Pass a [PlanarPnpSolver] + tag size to the re-solving constructor and the driver re-runs the
 * winning tag's PnP from the logged corners + intrinsics (replacing the logged `sol_*` rvec/tvec)
 * before feeding the bank — so you can compare, e.g., a different IPPE backend against the recorded
 * one on identical corner observations.
 */
class BankReplay(
    private val params: HypothesisBankLocalizer.Params,
    private val solver: VisionPoseSolver,
    private val pnp: PlanarPnpSolver? = null, // null = replay logged rvec/tvec; non-null = re-solve
    private val tagSizeMeters: Double = 0.0,
) {

    /** Replay from the logged PnP solutions (no re-solve). */
    constructor(
        params: HypothesisBankLocalizer.Params,
        solver: VisionPoseSolver,
    ) : this(params, solver, null, 0.0)

    /**
     * @param pnp if non-null, re-solve each frame's winning tag from its logged corners +
     *   intrinsics with this solver before feeding the bank
     * @param tagSizeMeters AprilTag side length for the re-solve
     */
    // primary constructor covers the 4-arg form

    /** Replayed vs. recorded fused trajectory, plus bank-health summaries. */
    class Result(
        @JvmField val rows: Int,
        @JvmField val tMs: DoubleArray,
        @JvmField val fusedX: DoubleArray,
        @JvmField val fusedY: DoubleArray,
        @JvmField val fusedHeadingDeg: DoubleArray, // replayed
        @JvmField val recX: DoubleArray,
        @JvmField val recY: DoubleArray,
        @JvmField val recHeadingDeg: DoubleArray, // recorded
        @JvmField val maxBankSize: Int,
        @JvmField val committedRows: Long,
    )

    @Throws(IOException::class)
    fun replay(input: Path): Result {
        val csv = TelemetryCsv.read(input)
        val n = csv.size()

        val core = HypothesisBankLocalizer(params, solver)
        val vision = CsvVisionSource()

        // Seed the pre-commit fallback from the recording's first fused pose, so the replayed
        // trajectory matches the recording before the bank commits (it rides odometry from there).
        if (n > 0) {
            val fused0 = pose(csv, 0, "fused_x", "fused_y", "fused_headingDeg")
            val odo0 = pose(csv, 0, "odo_x", "odo_y", "odo_headingDeg")
            if (fused0 != null && odo0 != null) {
                core.setPose(fused0, odo0)
            }
        }

        val tMs = DoubleArray(n)
        val fx = DoubleArray(n)
        val fy = DoubleArray(n)
        val fh = DoubleArray(n)
        val rx = DoubleArray(n)
        val ry = DoubleArray(n)
        val rh = DoubleArray(n)
        var maxBankSize = 0
        var committedRows = 0L

        var resolver: TagAmbiguitySolver? = null // built lazily once intrinsics are available

        for (i in 0 until n) {
            val nowNanos = (csv.get(i, "t_ms") * 1e6).toLong()
            var odo = pose(csv, i, "odo_x", "odo_y", "odo_headingDeg")
            if (odo == null) {
                odo = Pose2d.kZero
            }
            val yawRate = Math.toRadians(nz(csv.get(i, "ang_vel_deg_s")))

            vision.prepare(csv, i, nowNanos)
            val f = vision.latest()

            if (pnp != null && f.valid) {
                if (resolver == null && vision.getCalFx().isFinite()) {
                    resolver =
                        TagAmbiguitySolver(
                            pnp,
                            vision.cameraMatrix(),
                            vision.getCalDistCoeffs(),
                            tagSizeMeters,
                        )
                }
                if (resolver != null) {
                    resolveWinningTag(f, resolver!!)
                }
            }

            core.update(nowNanos, odo, yawRate, f)
            val fused = core.getPose(odo)

            tMs[i] = csv.get(i, "t_ms")
            fx[i] = fused.x
            fy[i] = fused.y
            fh[i] = Math.toDegrees(fused.rotation.radians)
            rx[i] = csv.get(i, "fused_x")
            ry[i] = csv.get(i, "fused_y")
            rh[i] = csv.get(i, "fused_headingDeg")

            maxBankSize = max(maxBankSize, core.bank().size())
            if (core.isCommitted()) {
                committedRows++
            }
        }

        return Result(n, tMs, fx, fy, fh, rx, ry, rh, maxBankSize, committedRows)
    }

    companion object {
        /**
         * Re-solve the winning tag's PnP from its logged corners and overwrite the frame's sol_*.
         */
        private fun resolveWinningTag(f: VisionFrame, resolver: TagAmbiguitySolver) {
            if (f.solTagId < 0 || f.allTagIds == null || f.allTagCorners == null) {
                return
            }
            var idx = -1
            for (k in f.allTagIds!!.indices) {
                if (f.allTagIds!![k] == f.solTagId) {
                    idx = k
                    break
                }
            }
            if (idx < 0 || idx >= f.allTagCorners!!.size) {
                return
            }
            val s = resolver.solve(toCornerList(f.allTagCorners!![idx])) ?: return
            f.solBestRvec = s.rvecBest
            f.solBestTvec = s.tvecBest
            f.solAltRvec = s.rvecAlt
            f.solAltTvec = s.tvecAlt
            f.ambiguity = s.ratio
            f.reprojErrPx = s.reprojErrBest
        }

        /** Flat `[x0,y0,...,x3,y3]` corners to the `List<List<Double>>` the solver takes. */
        private fun toCornerList(flat: DoubleArray): List<List<Double>> {
            val out = ArrayList<List<Double>>(4)
            for (j in 0 until 4) {
                out.add(listOf(flat[2 * j], flat[2 * j + 1]))
            }
            return out
        }

        /**
         * Reads three columns as a pose (x, y inches; heading degrees), or null if x/y is not
         * finite.
         */
        private fun pose(
            csv: TelemetryCsv,
            row: Int,
            xc: String,
            yc: String,
            hc: String,
        ): Pose2d? {
            val x = csv.get(row, xc)
            val y = csv.get(row, yc)
            val h = csv.get(row, hc)
            if (!x.isFinite() || !y.isFinite()) {
                return null
            }
            return Pose2d(x, y, Rotation2d(Math.toRadians(nz(h))))
        }

        private fun nz(v: Double): Double = if (v.isFinite()) v else 0.0
    }
}
