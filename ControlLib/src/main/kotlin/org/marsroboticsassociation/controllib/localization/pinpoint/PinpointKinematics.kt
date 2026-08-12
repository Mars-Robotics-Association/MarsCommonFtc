package org.marsroboticsassociation.controllib.localization.pinpoint

import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.kinematics.Kinematics
import kotlin.math.cos
import kotlin.math.sin

/**
 * Adapts WPILib's [Kinematics] interface for use with an odometry computer that provides
 * field-centric poses directly (e.g., GoBilda Pinpoint).
 *
 * <p>WPILib's odometry system was designed for drivetrains with wheel encoders, where kinematics
 * converts between wheel speeds/positions and chassis motion. Since an odometry computer already
 * outputs field-centric poses, this class adapts the interface by:
 * <ul>
 * <li>Treating "wheel speeds" as direct field-centric velocities ([PinpointSpeeds])</li>
 * <li>Treating "wheel positions" as field-centric poses ([PinpointPose])</li>
 * <li>Converting field-frame pose deltas to robot-frame twists in [toTwist2d]</li>
 * </ul>
 *
 * <p>The key adaptation is in [toTwist2d]: WPILib's [edu.wpi.first.math.kinematics.Odometry]
 * expects a robot-frame [Twist2d], but we have field-frame pose deltas. This method rotates the
 * field-frame delta into the robot frame so that `Pose2d.exp(twist)` produces the correct result.
 */
class PinpointKinematics : Kinematics<PinpointSpeeds, PinpointPose> {

    override fun toChassisSpeeds(wheelSpeeds: PinpointSpeeds): ChassisSpeeds {
        return ChassisSpeeds(wheelSpeeds.xSpeed, wheelSpeeds.ySpeed, wheelSpeeds.rotSpeed)
    }

    override fun toWheelSpeeds(chassisSpeeds: ChassisSpeeds): PinpointSpeeds {
        return PinpointSpeeds(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond,
            chassisSpeeds.omegaRadiansPerSecond,
        )
    }

    /**
     * Computes the robot-frame twist between two field-centric poses.
     *
     * <p>WPILib's Odometry.update() calls `Pose2d.exp(twist)` which interprets the twist as
     * robot-relative motion. Since we have field-centric poses from the odometry computer, we must
     * rotate the field-frame delta into the robot frame.
     *
     * @param start The starting field-centric pose.
     * @param end The ending field-centric pose.
     * @return A robot-frame twist representing the motion between the two poses.
     */
    override fun toTwist2d(start: PinpointPose, end: PinpointPose): Twist2d {
        // Field-frame deltas
        val fieldDx = end.x - start.x
        val fieldDy = end.y - start.y
        val dtheta = end.rot - start.rot

        // Convert field-frame delta to robot-frame delta
        // Rotate by negative of start heading to transform into robot frame
        val cos = cos(-start.rot)
        val sin = sin(-start.rot)
        val robotDx = fieldDx * cos - fieldDy * sin
        val robotDy = fieldDx * sin + fieldDy * cos

        return Twist2d(robotDx, robotDy, dtheta)
    }

    override fun copy(positions: PinpointPose): PinpointPose {
        return PinpointPose(positions.x, positions.y, positions.rot)
    }

    override fun copyInto(positions: PinpointPose, output: PinpointPose) {
        output.x = positions.x
        output.y = positions.y
        output.rot = positions.rot
    }

    override fun interpolate(
        startValue: PinpointPose,
        endValue: PinpointPose,
        t: Double,
    ): PinpointPose {
        return startValue.interpolate(endValue, t)
    }
}
