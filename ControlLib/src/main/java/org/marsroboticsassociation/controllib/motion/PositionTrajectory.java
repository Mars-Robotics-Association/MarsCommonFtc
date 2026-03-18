package org.marsroboticsassociation.controllib.motion;

public interface PositionTrajectory {
    double getPosition(double t);
    double getVelocity(double t);
    double getAcceleration(double t);
    double getTotalTime();
    boolean isZeroJerk(double t);
}
