package org.marsroboticsassociation.controllib.motion.ruckig;

/** Kinematic state output from one Ruckig update cycle. */
public class RuckigOutput {
    public double[] newPosition;
    public double[] newVelocity;
    public double[] newAcceleration;
    /** Total planned trajectory duration in seconds (from trajectory.get_duration()). */
    public double duration;

    public RuckigOutput(int dofs) {
        newPosition     = new double[dofs];
        newVelocity     = new double[dofs];
        newAcceleration = new double[dofs];
    }
}
