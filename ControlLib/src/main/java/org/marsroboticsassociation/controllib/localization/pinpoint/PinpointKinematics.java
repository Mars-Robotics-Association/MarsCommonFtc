package org.marsroboticsassociation.controllib.localization.pinpoint;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Adapts WPILib's {@link Kinematics} interface for use with an odometry computer that provides
 * field-centric poses directly (e.g., GoBilda Pinpoint).
 *
 * <p>WPILib's odometry system was designed for drivetrains with wheel encoders, where kinematics
 * converts between wheel speeds/positions and chassis motion. Since an odometry computer already
 * outputs field-centric poses, this class adapts the interface by:
 * <ul>
 *   <li>Treating "wheel speeds" as direct field-centric velocities ({@link PinpointSpeeds})</li>
 *   <li>Treating "wheel positions" as field-centric poses ({@link PinpointPose})</li>
 *   <li>Converting field-frame pose deltas to robot-frame twists in {@link #toTwist2d}</li>
 * </ul>
 *
 * <p>The key adaptation is in {@link #toTwist2d}: WPILib's {@link edu.wpi.first.math.kinematics.Odometry}
 * expects a robot-frame {@link Twist2d}, but we have field-frame pose deltas. This method rotates
 * the field-frame delta into the robot frame so that {@code Pose2d.exp(twist)} produces the correct result.
 */
public class PinpointKinematics implements Kinematics<PinpointSpeeds, PinpointPose> {

    @Override
    public ChassisSpeeds toChassisSpeeds(PinpointSpeeds wheelSpeeds) {
        return new ChassisSpeeds(wheelSpeeds.xSpeed, wheelSpeeds.ySpeed, wheelSpeeds.rotSpeed);
    }

    @Override
    public PinpointSpeeds toWheelSpeeds(ChassisSpeeds chassisSpeeds) {
        return new PinpointSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Computes the robot-frame twist between two field-centric poses.
     *
     * <p>WPILib's Odometry.update() calls {@code Pose2d.exp(twist)} which interprets the twist as
     * robot-relative motion. Since we have field-centric poses from the odometry computer, we must
     * rotate the field-frame delta into the robot frame.
     *
     * @param start The starting field-centric pose.
     * @param end   The ending field-centric pose.
     * @return A robot-frame twist representing the motion between the two poses.
     */
    @Override
    public Twist2d toTwist2d(PinpointPose start, PinpointPose end) {
        // Field-frame deltas
        double fieldDx = end.x - start.x;
        double fieldDy = end.y - start.y;
        double dtheta = end.rot - start.rot;

        // Convert field-frame delta to robot-frame delta
        // Rotate by negative of start heading to transform into robot frame
        double cos = Math.cos(-start.rot);
        double sin = Math.sin(-start.rot);
        double robotDx = fieldDx * cos - fieldDy * sin;
        double robotDy = fieldDx * sin + fieldDy * cos;

        return new Twist2d(robotDx, robotDy, dtheta);
    }

    @Override
    public PinpointPose copy(PinpointPose positions) {
        return new PinpointPose(positions.x, positions.y, positions.rot);
    }

    @Override
    public void copyInto(PinpointPose positions, PinpointPose output) {
        output.x = positions.x;
        output.y = positions.y;
        output.rot = positions.rot;
    }

    @Override
    public PinpointPose interpolate(PinpointPose startValue, PinpointPose endValue, double t) {
        return startValue.interpolate(endValue, t);
    }
}
