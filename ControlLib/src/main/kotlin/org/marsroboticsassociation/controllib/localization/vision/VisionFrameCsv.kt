package org.marsroboticsassociation.controllib.localization.vision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import java.util.ArrayList
import java.util.LinkedHashMap
import java.util.function.Function
import kotlin.math.round

/**
 * The CSV serialization contract for a [VisionFrame] plus its camera calibration — the raw
 * per-frame inputs the localizer chain consumes. It is the single source of truth for the *replay*
 * schema: the on-robot logger writes these columns, and the off-robot replay harness reads them
 * back to reconstruct the exact [VisionFrame] and re-run the PnP → solver → bank chain (optionally
 * re-solving the PnP from the logged corners with a different method).
 *
 * <p>Both sides are driven from one ordered map ([toMap]), so the written column order and the
 * parsed names can never drift apart. Every column is numeric (booleans/ids are written as
 * `0/1`/`-1`); an absent value is [Double.NaN]. A round-trip (`toMap` → CSV → [parse]) reproduces
 * the frame's replay-relevant fields exactly.
 *
 * <p>What is **not** here: `timestamp`/`receiptNanos` (the replay clock synthesizes them) and the
 * purely-diagnostic fields the chain never reads (MT2 botpose, MT1 stddevs, out-of-plane
 * z/roll/pitch). The odometry, loop timing, and fused/bank *outputs* are the logger's own columns
 * wrapped around this frame block, not part of this codec.
 */
object VisionFrameCsv {

    private val T6_SUFFIX = arrayOf("_x", "_y", "_z", "_yaw", "_pitch", "_roll")

    /**
     * The frame's replay columns and their values, in write order, from a [VisionFrame] plus the
     * camera intrinsics (`cam = [fx,fy,cx,cy]`) and distortion (OpenCV order `[k1,k2,p1,p2,k3]`)
     * the on-robot PnP used. Insertion order defines [COLUMNS].
     */
    @JvmStatic
    fun toMap(
        f: VisionFrame,
        cam: DoubleArray?,
        dist: DoubleArray?,
    ): LinkedHashMap<String, Double> {
        val m = LinkedHashMap<String, Double>()

        // Frame meta.
        m["vis_newFrame"] = if (f.valid) 1.0 else 0.0
        m["vis_tagCount"] = f.tagCount.toDouble()
        m["vis_avgDistM"] = f.avgDistM
        m["vis_latencyMs"] = f.latencySec * 1e3

        // MegaTag1 botpose (field inches, heading deg) — reference; the chain only null-checks it.
        m["vision_x"] = f.mt1Pose?.x ?: Double.NaN
        m["vision_y"] = f.mt1Pose?.y ?: Double.NaN
        m["vision_headingDeg"] =
            f.mt1Pose?.let { Math.toDegrees(it.rotation.radians) } ?: Double.NaN

        // Detection quality.
        m["vis_ambiguity"] = f.ambiguity
        m["vis_reprojErrPx"] = f.reprojErrPx
        m["focus_metric"] = f.focusMetric
        m["tag_skew"] = f.skew

        // The winning tag's PnP solutions (the replay-critical raw inputs): tag id + both IPPE
        // (rvec, tvec) candidates, best-first.
        m["sol_tagId"] = f.solTagId.toDouble()
        putVec3(m, "sol_best_r", f.solBestRvec)
        putVec3(m, "sol_best_t", f.solBestTvec)
        putVec3(m, "sol_alt_r", f.solAltRvec)
        putVec3(m, "sol_alt_t", f.solAltTvec)

        // Limelight's own per-tag / botpose 6-DOF (reference for comparison).
        putVec6(m, "t6tCs", f.t6tCs)
        putVec6(m, "t6tRs", f.t6tRs)
        putVec6(m, "t6rFs", f.t6rFs)

        // Winning tag bounding box + centering angles (the sigma(span) regressor).
        m["vis_tagTxDeg"] = f.tagTxDeg
        m["vis_tagTyDeg"] = f.tagTyDeg
        m["vis_tagMinXPx"] = f.tagMinXPx
        m["vis_tagMaxXPx"] = f.tagMaxXPx
        m["vis_tagMinYPx"] = f.tagMinYPx
        m["vis_tagMaxYPx"] = f.tagMaxYPx

        // ALL detected tags (up to 4 slots): id + 4 image corners each — the raw per-landmark
        // observations an offline PnP re-solve consumes. Empty slots are id -1 / NaN corners.
        for (k in 0 until 4) {
            val id = if (f.allTagIds != null && k < f.allTagIds!!.size) f.allTagIds!![k] else -1
            val c =
                if (f.allTagCorners != null && k < f.allTagCorners!!.size) {
                    f.allTagCorners!![k]
                } else {
                    null
                }
            m["tag${k}_id"] = id.toDouble()
            for (j in 0 until 4) {
                m["tag${k}_c${j}x"] = if (c != null && 2 * j < c.size) c[2 * j] else Double.NaN
                m["tag${k}_c${j}y"] =
                    if (c != null && 2 * j + 1 < c.size) c[2 * j + 1] else Double.NaN
            }
        }

        // Camera intrinsics + distortion actually used on-robot (so an offline re-solve matches).
        m["vis_calFx"] = at(cam, 0)
        m["vis_calFy"] = at(cam, 1)
        m["vis_calCx"] = at(cam, 2)
        m["vis_calCy"] = at(cam, 3)
        val distNames =
            arrayOf("vis_dist_k1", "vis_dist_k2", "vis_dist_p1", "vis_dist_p2", "vis_dist_k3")
        for (i in distNames.indices) {
            m[distNames[i]] = at(dist, i)
        }
        return m
    }

    /** The frame column names, in write order (derived from an empty [toMap]). */
    @JvmField
    val COLUMNS: List<String> = ArrayList(toMap(VisionFrame(), DoubleArray(4), DoubleArray(5)).keys)

    /** The frame columns joined as a CSV header fragment. */
    @JvmField val HEADER: String = COLUMNS.joinToString(",")

    /** The reconstructed frame plus the camera calibration read from a row. */
    class Parsed
    internal constructor(
        @JvmField val frame: VisionFrame,
        @JvmField
        val cameraMatrix: DoubleArray, // row-major 3x3 [fx,0,cx, 0,fy,cy, 0,0,1], NaN if absent
        @JvmField val distCoeffs: DoubleArray, // [k1,k2,p1,p2,k3]
    )

    /**
     * Reconstruct a [VisionFrame] (and its calibration) from a row, read by column name via `get`
     * ([Double.NaN] for absent columns). `valid`/`timestamp` are NOT set here — the replay source
     * assigns them from `vis_newFrame` and the frame clock.
     */
    @JvmStatic
    fun parse(get: Function<String, Double>): Parsed {
        val f = VisionFrame()
        f.tagCount = round(nz(get.apply("vis_tagCount"))).toInt()
        f.avgDistM = get.apply("vis_avgDistM")
        f.latencySec = get.apply("vis_latencyMs") / 1e3

        val vx = get.apply("vision_x")
        val vy = get.apply("vision_y")
        val vh = get.apply("vision_headingDeg")
        f.mt1Pose =
            if (vx.isFinite()) {
                Pose2d(vx, vy, Rotation2d(Math.toRadians(vh)))
            } else {
                null
            }

        f.ambiguity = get.apply("vis_ambiguity")
        f.reprojErrPx = get.apply("vis_reprojErrPx")
        f.focusMetric = get.apply("focus_metric")
        f.skew = get.apply("tag_skew")

        f.solTagId = round(nz(get.apply("sol_tagId"), -1.0)).toInt()
        f.solBestRvec = vec3(get, "sol_best_r")
        f.solBestTvec = vec3(get, "sol_best_t")
        f.solAltRvec = vec3(get, "sol_alt_r")
        f.solAltTvec = vec3(get, "sol_alt_t")

        f.t6tCs = vec6(get, "t6tCs")
        f.t6tRs = vec6(get, "t6tRs")
        f.t6rFs = vec6(get, "t6rFs")

        f.tagTxDeg = get.apply("vis_tagTxDeg")
        f.tagTyDeg = get.apply("vis_tagTyDeg")
        f.tagMinXPx = get.apply("vis_tagMinXPx")
        f.tagMaxXPx = get.apply("vis_tagMaxXPx")
        f.tagMinYPx = get.apply("vis_tagMinYPx")
        f.tagMaxYPx = get.apply("vis_tagMaxYPx")

        val ids = ArrayList<Int>()
        val corners = ArrayList<DoubleArray>()
        for (k in 0 until 4) {
            val id = get.apply("tag${k}_id")
            if (!id.isFinite() || id < 0) {
                continue
            }
            ids.add(round(id).toInt())
            val c = DoubleArray(8)
            for (j in 0 until 4) {
                c[2 * j] = get.apply("tag${k}_c${j}x")
                c[2 * j + 1] = get.apply("tag${k}_c${j}y")
            }
            corners.add(c)
        }
        if (ids.isNotEmpty()) {
            f.allTagIds = ids.stream().mapToInt { it }.toArray()
            f.allTagCorners = corners.toTypedArray()
        }

        val fx = get.apply("vis_calFx")
        val fy = get.apply("vis_calFy")
        val cx = get.apply("vis_calCx")
        val cy = get.apply("vis_calCy")
        val cameraMatrix = doubleArrayOf(fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0)
        val dist =
            doubleArrayOf(
                get.apply("vis_dist_k1"),
                get.apply("vis_dist_k2"),
                get.apply("vis_dist_p1"),
                get.apply("vis_dist_p2"),
                get.apply("vis_dist_k3"),
            )
        return Parsed(f, cameraMatrix, dist)
    }

    // --- helpers --------------------------------------------------------------------------------

    private fun putVec3(m: MutableMap<String, Double>, prefix: String, v: DoubleArray?) {
        for (i in 0 until 3) {
            m[prefix + i] = if (v != null && i < v.size) v[i] else Double.NaN
        }
    }

    private fun putVec6(m: MutableMap<String, Double>, prefix: String, v: DoubleArray?) {
        for (i in 0 until 6) {
            m[prefix + T6_SUFFIX[i]] = if (v != null && i < v.size) v[i] else Double.NaN
        }
    }

    /** Reads `<prefix>0..2`, or null if all three are NaN. */
    private fun vec3(get: Function<String, Double>, prefix: String): DoubleArray? {
        val x = get.apply(prefix + "0")
        val y = get.apply(prefix + "1")
        val z = get.apply(prefix + "2")
        return if (x.isFinite() || y.isFinite() || z.isFinite()) {
            doubleArrayOf(x, y, z)
        } else {
            null
        }
    }

    /** Reads the six `<prefix>{_x,_y,_z,_yaw,_pitch,_roll}` columns, or null if all NaN. */
    private fun vec6(get: Function<String, Double>, prefix: String): DoubleArray? {
        val v = DoubleArray(6)
        var any = false
        for (i in 0 until 6) {
            v[i] = get.apply(prefix + T6_SUFFIX[i])
            any = any || v[i].isFinite()
        }
        return if (any) v else null
    }

    private fun at(a: DoubleArray?, i: Int): Double {
        return if (a != null && i < a.size) a[i] else Double.NaN
    }

    private fun nz(v: Double): Double = if (v.isFinite()) v else 0.0

    private fun nz(v: Double, fallback: Double): Double = if (v.isFinite()) v else fallback
}
