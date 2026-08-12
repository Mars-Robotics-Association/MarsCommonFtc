package org.marsroboticsassociation.controllib.localization.pinpoint

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.Odometry

/**
 * Odometry adapter for use with an odometry computer that provides field-centric poses directly.
 *
 * <p>This class extends WPILib's [Odometry] using [PinpointPose] as the "wheel positions" type.
 * Each call to [update] receives the current field-centric pose from the odometry computer, and
 * [PinpointKinematics.toTwist2d] handles the conversion to robot-frame motion that WPILib's pose
 * integration expects.
 *
 * @see PinpointKinematics
 * @see PinpointPoseEstimator
 */
class PinpointOdometry(
    kinematics: PinpointKinematics,
    gyroAngle: Rotation2d,
    initialPose: PinpointPose,
    initialPoseMeters: Pose2d,
) : Odometry<PinpointPose>(kinematics, gyroAngle, initialPose, initialPoseMeters)
