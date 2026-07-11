package org.marsroboticsassociation.controllib.localization.vision;

/**
 * Minimal rigid-body transform in SE(3): a 3x3 rotation {@link #r} and a 3-vector translation
 * {@link #t}, acting as {@code p_parent = R * p_child + t}. Read it by the frames it relates — a
 * transform built as "child expressed in parent" maps child-frame points into the parent frame and
 * composes left-to-right: {@code parentFromChild = parentFromMid.multiply(midFromChild)}.
 *
 * <p>Pure arithmetic, no OpenCV / Android, so the whole MegaTag2 transform chain is unit-testable
 * on the desktop JVM — the same split as {@link TagAmbiguityMath} (pure) vs {@link
 * TagAmbiguitySolver} (delegates the PnP solve to an injected {@link PlanarPnpSolver}). Unit-agnostic:
 * keep every translation in one length unit (this project uses inches at the field-frame boundary, so
 * scale the PnP translation meters&rarr;inches in the adapter that builds the camera-relative
 * candidates).
 */
public final class Transform3D {

    /** Row-major 3x3 rotation. */
    public final double[][] r;

    /** 3-vector translation. */
    public final double[] t;

    public Transform3D(double[][] r, double[] t) {
        this.r = r;
        this.t = t;
    }

    public static Transform3D identity() {
        return new Transform3D(
                new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, new double[] {0, 0, 0});
    }

    /**
     * Builds a transform from a translation and an intrinsic Z-Y-X (yaw-pitch-roll) rotation, all
     * in radians: {@code R = Rz(yaw) · Ry(pitch) · Rx(roll)} (yaw about +z, pitch about +y, roll
     * about +x). Convenient for field-frame tag poses and a level camera mount.
     */
    public static Transform3D fromTranslationYPR(
            double x, double y, double z, double yaw, double pitch, double roll) {
        double cy = Math.cos(yaw), sy = Math.sin(yaw);
        double cp = Math.cos(pitch), sp = Math.sin(pitch);
        double cr = Math.cos(roll), sr = Math.sin(roll);
        double[][] R = {
            {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
            {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
            {-sp, cp * sr, cp * cr}
        };
        return new Transform3D(R, new double[] {x, y, z});
    }

    /**
     * Builds a transform from a translation and a unit quaternion {@code (w, x, y, z)} — the form
     * the FTC AprilTag field library reports tag orientations in.
     */
    public static Transform3D fromTranslationQuaternion(
            double x, double y, double z, double qw, double qx, double qy, double qz) {
        double[][] R = {
            {1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)},
            {2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)},
            {2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)}
        };
        return new Transform3D(R, new double[] {x, y, z});
    }

    /**
     * Builds a transform from an OpenCV {@code solvePnP} result: a length-3 Rodrigues rotation
     * vector and a length-3 translation. This is the closed-form Rodrigues formula in plain Java,
     * so converting a {@link PlanarPnpSolver.PnpSolution} ({@code rvec}/{@code tvec}) into {@link
     * VisionPoseSolver} candidates needs no OpenCV — keeping the whole {@code cameraFromTag} &rarr;
     * field chain on the desktop-testable side. Translation units pass straight through (metres, as
     * the PnP object points are in metres).
     */
    public static Transform3D fromRodrigues(double[] rvec, double[] tvec) {
        double vx = rvec[0], vy = rvec[1], vz = rvec[2];
        double theta = Math.sqrt(vx * vx + vy * vy + vz * vz);
        double[][] R;
        if (theta < 1e-12) {
            R = new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        } else {
            double kx = vx / theta, ky = vy / theta, kz = vz / theta;
            double s = Math.sin(theta), c = Math.cos(theta), v1 = 1 - c;
            R =
                    new double[][] {
                        {c + kx * kx * v1, kx * ky * v1 - kz * s, kx * kz * v1 + ky * s},
                        {ky * kx * v1 + kz * s, c + ky * ky * v1, ky * kz * v1 - kx * s},
                        {kz * kx * v1 - ky * s, kz * ky * v1 + kx * s, c + kz * kz * v1}
                    };
        }
        return new Transform3D(R, new double[] {tvec[0], tvec[1], tvec[2]});
    }

    /** Composition {@code this ∘ other}: applies {@code other} first, then {@code this}. */
    public Transform3D multiply(Transform3D o) {
        double[][] R = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                double s = 0;
                for (int k = 0; k < 3; k++) {
                    s += r[i][k] * o.r[k][j];
                }
                R[i][j] = s;
            }
        }
        double[] T = new double[3];
        for (int i = 0; i < 3; i++) {
            double s = t[i];
            for (int k = 0; k < 3; k++) {
                s += r[i][k] * o.t[k];
            }
            T[i] = s;
        }
        return new Transform3D(R, T);
    }

    /** Inverse of a rigid transform: {@code R^T, -R^T t}. */
    public Transform3D inverse() {
        double[][] Rt = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                Rt[i][j] = r[j][i];
            }
        }
        double[] T = new double[3];
        for (int i = 0; i < 3; i++) {
            double s = 0;
            for (int k = 0; k < 3; k++) {
                s += Rt[i][k] * t[k];
            }
            T[i] = -s;
        }
        return new Transform3D(Rt, T);
    }

    public double x() {
        return t[0];
    }

    public double y() {
        return t[1];
    }

    public double z() {
        return t[2];
    }

    /**
     * Field-frame heading — rotation about +z, {@code atan2(R[1][0], R[0][0])}. Correct when the
     * field frame has +z up and the robot's forward axis is its local +x (the standard FTC/Road
     * Runner field convention).
     */
    public double yaw() {
        return Math.atan2(r[1][0], r[0][0]);
    }
}
