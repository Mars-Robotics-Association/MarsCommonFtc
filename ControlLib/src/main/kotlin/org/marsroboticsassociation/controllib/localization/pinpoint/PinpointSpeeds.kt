package org.marsroboticsassociation.controllib.localization.pinpoint

/**
 * Represents field-centric velocities, used as the "wheel speeds" type for [PinpointKinematics].
 *
 * <p>This is a simple pass-through since the odometry computer provides velocities directly rather
 * than requiring kinematic conversion from individual wheel speeds.
 */
class PinpointSpeeds(
    /** X velocity in meters per second (field-centric). */
    @JvmField var xSpeed: Double,
    /** Y velocity in meters per second (field-centric). */
    @JvmField var ySpeed: Double,
    /** Angular velocity in radians per second. */
    @JvmField var rotSpeed: Double,
)
