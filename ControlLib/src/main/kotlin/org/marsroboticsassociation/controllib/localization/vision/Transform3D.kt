package org.marsroboticsassociation.controllib.localization.vision

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Minimal rigid-body transform in SE(3): a 3x3 rotation [r] and a 3-vector translation [t], acting
 * as `p_parent = R * p_child + t`. Read it by the frames it relates — a transform built as "child
 * expressed in parent" maps child-frame points into the parent frame and composes left-to-right:
 * `parentFromChild = parentFromMid.multiply(midFromChild)`.
 *
 * <p>Pure arithmetic, no OpenCV / Android, so the whole MegaTag2 transform chain is unit-testable
 * on the desktop JVM — the same split as [TagAmbiguityMath] (pure) vs [TagAmbiguitySolver]
 * (delegates the PnP solve to an injected [PlanarPnpSolver]). Unit-agnostic: keep every translation
 * in one length unit (this project uses inches at the field-frame boundary, so scale the PnP
 * translation meters→inches in the adapter that builds the camera-relative candidates).
 */
class Transform3D(
    /** Row-major 3x3 rotation. */
    @JvmField val r: Array<DoubleArray>,
    /** 3-vector translation. */
    @JvmField val t: DoubleArray,
) {

    /** Composition `this ∘ other`: applies `other` first, then `this`. */
    fun multiply(o: Transform3D): Transform3D {
        val R = Array(3) { DoubleArray(3) }
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                var s = 0.0
                for (k in 0 until 3) {
                    s += r[i][k] * o.r[k][j]
                }
                R[i][j] = s
            }
        }
        val T = DoubleArray(3)
        for (i in 0 until 3) {
            var s = t[i]
            for (k in 0 until 3) {
                s += r[i][k] * o.t[k]
            }
            T[i] = s
        }
        return Transform3D(R, T)
    }

    /** Inverse of a rigid transform: `R^T, -R^T t`. */
    fun inverse(): Transform3D {
        val Rt = Array(3) { DoubleArray(3) }
        for (i in 0 until 3) {
            for (j in 0 until 3) {
                Rt[i][j] = r[j][i]
            }
        }
        val T = DoubleArray(3)
        for (i in 0 until 3) {
            var s = 0.0
            for (k in 0 until 3) {
                s += Rt[i][k] * t[k]
            }
            T[i] = -s
        }
        return Transform3D(Rt, T)
    }

    fun x(): Double = t[0]

    fun y(): Double = t[1]

    fun z(): Double = t[2]

    /**
     * Field-frame heading — rotation about +z, `atan2(R[1][0], R[0][0])`. Correct when the field
     * frame has +z up and the robot's forward axis is its local +x (the standard FTC/Road Runner
     * field convention).
     */
    fun yaw(): Double = atan2(r[1][0], r[0][0])

    companion object {
        @JvmStatic
        fun identity(): Transform3D {
            return Transform3D(
                arrayOf(
                    doubleArrayOf(1.0, 0.0, 0.0),
                    doubleArrayOf(0.0, 1.0, 0.0),
                    doubleArrayOf(0.0, 0.0, 1.0),
                ),
                doubleArrayOf(0.0, 0.0, 0.0),
            )
        }

        /**
         * Builds a transform from a translation and an intrinsic Z-Y-X (yaw-pitch-roll) rotation,
         * all in radians: `R = Rz(yaw) · Ry(pitch) · Rx(roll)` (yaw about +z, pitch about +y, roll
         * about +x). Convenient for field-frame tag poses and a level camera mount.
         */
        @JvmStatic
        fun fromTranslationYPR(
            x: Double,
            y: Double,
            z: Double,
            yaw: Double,
            pitch: Double,
            roll: Double,
        ): Transform3D {
            val cy = cos(yaw)
            val sy = sin(yaw)
            val cp = cos(pitch)
            val sp = sin(pitch)
            val cr = cos(roll)
            val sr = sin(roll)
            val R =
                arrayOf(
                    doubleArrayOf(cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
                    doubleArrayOf(sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
                    doubleArrayOf(-sp, cp * sr, cp * cr),
                )
            return Transform3D(R, doubleArrayOf(x, y, z))
        }

        /**
         * Builds a transform from a translation and a unit quaternion `(w, x, y, z)` — the form the
         * FTC AprilTag field library reports tag orientations in.
         */
        @JvmStatic
        fun fromTranslationQuaternion(
            x: Double,
            y: Double,
            z: Double,
            qw: Double,
            qx: Double,
            qy: Double,
            qz: Double,
        ): Transform3D {
            val R =
                arrayOf(
                    doubleArrayOf(
                        1 - 2 * (qy * qy + qz * qz),
                        2 * (qx * qy - qz * qw),
                        2 * (qx * qz + qy * qw),
                    ),
                    doubleArrayOf(
                        2 * (qx * qy + qz * qw),
                        1 - 2 * (qx * qx + qz * qz),
                        2 * (qy * qz - qx * qw),
                    ),
                    doubleArrayOf(
                        2 * (qx * qz - qy * qw),
                        2 * (qy * qz + qx * qw),
                        1 - 2 * (qx * qx + qy * qy),
                    ),
                )
            return Transform3D(R, doubleArrayOf(x, y, z))
        }

        /**
         * Builds a transform from an OpenCV `solvePnP` result: a length-3 Rodrigues rotation vector
         * and a length-3 translation. This is the closed-form Rodrigues formula in plain
         * arithmetic, so converting a [PlanarPnpSolver.PnpSolution] (`rvec`/`tvec`) into
         * [VisionPoseSolver] candidates needs no OpenCV — keeping the whole `cameraFromTag` → field
         * chain on the desktop-testable side. Translation units pass straight through (metres, as
         * the PnP object points are in metres).
         */
        @JvmStatic
        fun fromRodrigues(rvec: DoubleArray, tvec: DoubleArray): Transform3D {
            val vx = rvec[0]
            val vy = rvec[1]
            val vz = rvec[2]
            val theta = sqrt(vx * vx + vy * vy + vz * vz)
            val R: Array<DoubleArray>
            if (theta < 1e-12) {
                R =
                    arrayOf(
                        doubleArrayOf(1.0, 0.0, 0.0),
                        doubleArrayOf(0.0, 1.0, 0.0),
                        doubleArrayOf(0.0, 0.0, 1.0),
                    )
            } else {
                val kx = vx / theta
                val ky = vy / theta
                val kz = vz / theta
                val s = sin(theta)
                val c = cos(theta)
                val v1 = 1 - c
                R =
                    arrayOf(
                        doubleArrayOf(
                            c + kx * kx * v1,
                            kx * ky * v1 - kz * s,
                            kx * kz * v1 + ky * s,
                        ),
                        doubleArrayOf(
                            ky * kx * v1 + kz * s,
                            c + ky * ky * v1,
                            ky * kz * v1 - kx * s,
                        ),
                        doubleArrayOf(
                            kz * kx * v1 - ky * s,
                            kz * ky * v1 + kx * s,
                            c + kz * kz * v1,
                        ),
                    )
            }
            return Transform3D(R, doubleArrayOf(tvec[0], tvec[1], tvec[2]))
        }
    }
}
