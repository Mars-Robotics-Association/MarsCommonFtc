package org.marsroboticsassociation.controllib.localization.pinpoint

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.interpolation.Interpolatable

/**
 * A mutable container for field-centric pose data from an odometry computer (e.g., GoBilda
 * Pinpoint).
 *
 * <p>WPILib's [edu.wpi.first.math.kinematics.Odometry] expects a mutable "wheel positions" type
 * that it can update in place. Since [Pose2d] is immutable, this class provides a mutable
 * alternative that stores x, y (in meters) and rotation (in radians).
 *
 * <p>Unlike traditional wheel encoder positions, this represents a complete field-centric pose as
 * reported directly by the odometry computer.
 */
class PinpointPose : Interpolatable<PinpointPose> {

    /** X position in meters (field-centric). */
    @JvmField var x: Double

    /** Y position in meters (field-centric). */
    @JvmField var y: Double

    /** Heading in radians (field-centric). */
    @JvmField var rot: Double

    /**
     * Constructs a PinpointPose with the specified values.
     *
     * @param x X position in meters.
     * @param y Y position in meters.
     * @param rot Heading in radians.
     */
    constructor(x: Double, y: Double, rot: Double) {
        this.x = x
        this.y = y
        this.rot = rot
    }

    /** Constructs a PinpointPose at the origin with zero heading. */
    constructor() : this(0.0, 0.0, 0.0)

    constructor(wpiPose: Pose2d) : this(wpiPose.x, wpiPose.y, wpiPose.rotation.radians)

    override fun interpolate(endValue: PinpointPose, t: Double): PinpointPose {
        return PinpointPose(
            MathUtil.interpolate(x, endValue.x, t),
            MathUtil.interpolate(y, endValue.y, t),
            MathUtil.interpolate(rot, endValue.rot, t),
        )
    }
}
