package org.marsroboticsassociation.controllib.motion.ruckig;

/** Kinematic limits and target state for one Ruckig update cycle. */
public class RuckigInput {
    public double[] currentPosition;
    public double[] currentVelocity;
    public double[] currentAcceleration;
    public double[] targetPosition;
    public double[] targetVelocity;
    public double[] targetAcceleration;
    public double[] maxVelocity;
    public double[] maxAcceleration;
    public double[] maxJerk;

    public RuckigInput(int dofs) {
        currentPosition     = new double[dofs];
        currentVelocity     = new double[dofs];
        currentAcceleration = new double[dofs];
        targetPosition      = new double[dofs];
        targetVelocity      = new double[dofs];
        targetAcceleration  = new double[dofs];
        maxVelocity         = new double[dofs];
        maxAcceleration     = new double[dofs];
        maxJerk             = new double[dofs];
    }
}
