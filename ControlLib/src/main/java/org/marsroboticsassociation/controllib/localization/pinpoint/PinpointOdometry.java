package org.marsroboticsassociation.controllib.localization.pinpoint;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Odometry adapter for use with an odometry computer that provides field-centric poses directly.
 *
 * <p>This class extends WPILib's {@link Odometry} using {@link PinpointPose} as the "wheel positions"
 * type. Each call to {@link #update} receives the current field-centric pose from the odometry
 * computer, and {@link PinpointKinematics#toTwist2d} handles the conversion to robot-frame motion
 * that WPILib's pose integration expects.
 *
 * @see PinpointKinematics
 * @see PinpointPoseEstimator
 */
public class PinpointOdometry extends Odometry<PinpointPose> {

    /**
     * Constructs a PinpointOdometry instance.
     *
     * @param kinematics        The {@link PinpointKinematics} instance for pose delta conversion.
     * @param gyroAngle         The initial heading from the odometry computer.
     * @param initialPose       The initial field-centric pose from the odometry computer.
     * @param initialPoseMeters The initial pose estimate (typically matches initialPose).
     */
    public PinpointOdometry(PinpointKinematics kinematics, Rotation2d gyroAngle, PinpointPose initialPose, Pose2d initialPoseMeters) {
        super(kinematics, gyroAngle, initialPose, initialPoseMeters);
    }
}
