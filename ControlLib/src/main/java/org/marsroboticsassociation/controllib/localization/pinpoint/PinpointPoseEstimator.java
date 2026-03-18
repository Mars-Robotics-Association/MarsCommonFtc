package org.marsroboticsassociation.controllib.localization.pinpoint;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/**
 * Fuses odometry computer poses with vision measurements using WPILib's {@link PoseEstimator}.
 *
 * <p>This class adapts WPILib's PoseEstimator (designed for wheel encoder odometry) to work with
 * an odometry computer like the GoBilda Pinpoint that outputs field-centric poses directly.
 * It internally creates the required {@link PinpointKinematics} and {@link PinpointOdometry}.
 *
 * <p>Usage:
 * <ol>
 *   <li>Call {@link #update} every loop with the current heading and pose from the odometry computer</li>
 *   <li>Call {@link #addVisionMeasurement} when vision data is available</li>
 *   <li>Call {@link #getEstimatedPosition} to get the fused pose estimate</li>
 * </ol>
 *
 * <p>The estimator maintains a history buffer for latency compensation, allowing vision measurements
 * to be incorporated at their actual capture time rather than when they're processed.
 *
 * @see PinpointKinematics
 * @see PinpointOdometry
 */
public class PinpointPoseEstimator extends PoseEstimator<PinpointPose> {

    /**
     * Constructs a PinpointPoseEstimator with default initial pose at the origin.
     *
     * <p>This is the recommended constructor for most use cases. It internally creates the
     * required {@link PinpointKinematics} and {@link PinpointOdometry} instances.
     *
     * @param stateStdDevs             Standard deviations for the odometry estimate (x in meters,
     *                                 y in meters, heading in radians). Increase to trust odometry less.
     * @param visionMeasurementStdDevs Standard deviations for vision measurements (x in meters,
     *                                 y in meters, heading in radians). Increase to trust vision less.
     */
    public PinpointPoseEstimator(Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        this(new PinpointKinematics(), stateStdDevs, visionMeasurementStdDevs);
    }

    /**
     * Constructs a PinpointPoseEstimator with the specified kinematics.
     *
     * <p>This constructor is useful if you need to share a kinematics instance or customize it.
     *
     * @param kinematics               The {@link PinpointKinematics} instance.
     * @param stateStdDevs             Standard deviations for the odometry estimate (x in meters,
     *                                 y in meters, heading in radians). Increase to trust odometry less.
     * @param visionMeasurementStdDevs Standard deviations for vision measurements (x in meters,
     *                                 y in meters, heading in radians). Increase to trust vision less.
     */
    public PinpointPoseEstimator(PinpointKinematics kinematics, Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs) {
        super(
                kinematics,
                new PinpointOdometry(kinematics, Rotation2d.kZero, new PinpointPose(), Pose2d.kZero),
                stateStdDevs,
                visionMeasurementStdDevs
        );
    }

    /**
     * Updates the pose estimate with the current odometry computer reading.
     *
     * <p>This convenience method accepts position in inches (typical for FTC) and converts
     * to meters internally.
     *
     * @param headingRadians The current heading in radians.
     * @param xInches        The current x position in inches (field-centric).
     * @param yInches        The current y position in inches (field-centric).
     * @return The updated pose estimate in meters.
     */
    public Pose2d update(double headingRadians, double xInches, double yInches) {
        return update(
                Rotation2d.fromRadians(headingRadians),
                new PinpointPose(Units.inchesToMeters(xInches), Units.inchesToMeters(yInches), headingRadians)
        );
    }
}
