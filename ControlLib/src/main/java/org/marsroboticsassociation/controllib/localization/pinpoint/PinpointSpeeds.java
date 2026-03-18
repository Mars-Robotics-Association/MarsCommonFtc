package org.marsroboticsassociation.controllib.localization.pinpoint;

/**
 * Represents field-centric velocities, used as the "wheel speeds" type for {@link PinpointKinematics}.
 *
 * <p>This is a simple pass-through since the odometry computer provides velocities directly
 * rather than requiring kinematic conversion from individual wheel speeds.
 */
public class PinpointSpeeds {

    /** X velocity in meters per second (field-centric). */
    public double xSpeed;
    /** Y velocity in meters per second (field-centric). */
    public double ySpeed;
    /** Angular velocity in radians per second. */
    public double rotSpeed;

    /**
     * Constructs a PinpointSpeeds with the specified velocities.
     *
     * @param xSpeed   X velocity in meters per second.
     * @param ySpeed   Y velocity in meters per second.
     * @param rotSpeed Angular velocity in radians per second.
     */
    public PinpointSpeeds(double xSpeed, double ySpeed, double rotSpeed) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
    }
}
