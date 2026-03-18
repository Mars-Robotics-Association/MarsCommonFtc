package org.marsroboticsassociation.controllib.motion;

public interface VelocityTrajectory {
    double getAcceleration(double t);
    double getVelocity(double t);
    double getTotalTime();
    boolean isZeroJerk(double t);
}
