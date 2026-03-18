package org.marsroboticsassociation.controllib.localization.pinpoint;

import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.MathUtil;

/**
 * A mutable container for field-centric pose data from an odometry computer (e.g., GoBilda Pinpoint).
 *
 * <p>WPILib's {@link edu.wpi.first.math.kinematics.Odometry} expects a mutable "wheel positions" type
 * that it can update in place. Since {@link edu.wpi.first.math.geometry.Pose2d} is immutable, this
 * class provides a mutable alternative that stores x, y (in meters) and rotation (in radians).
 *
 * <p>Unlike traditional wheel encoder positions, this represents a complete field-centric pose
 * as reported directly by the odometry computer.
 */
public class PinpointPose implements Interpolatable<PinpointPose> {

    /** X position in meters (field-centric). */
    public double x;
    /** Y position in meters (field-centric). */
    public double y;
    /** Heading in radians (field-centric). */
    public double rot;

    /**
     * Constructs a PinpointPose with the specified values.
     *
     * @param x   X position in meters.
     * @param y   Y position in meters.
     * @param rot Heading in radians.
     */
    public PinpointPose(double x, double y, double rot) {
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    /**
     * Constructs a PinpointPose at the origin with zero heading.
     */
    public PinpointPose() {
        this(0, 0, 0);
    }

    public PinpointPose(edu.wpi.first.math.geometry.Pose2d wpiPose) {
        this(wpiPose.getX(), wpiPose.getY(), wpiPose.getRotation().getRadians());
    }

    @Override
    public PinpointPose interpolate(PinpointPose endValue, double t) {
        return new PinpointPose(
                MathUtil.interpolate(x, endValue.x, t),
                MathUtil.interpolate(y, endValue.y, t),
                MathUtil.interpolate(rot, endValue.rot, t)
        );
    }
}
