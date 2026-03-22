package org.marsroboticsassociation.controllab.trajectory;

public enum TrajectoryType {
    SCURVE_POSITION("SCurvePosition"),
    SIN_CURVE_POSITION("Sinusoidal Position"),
    SCURVE_VELOCITY("SCurveVelocity"),
    RUCKIG("Ruckig (1-DOF)");

    private final String displayName;

    TrajectoryType(String displayName) {
        this.displayName = displayName;
    }

    @Override
    public String toString() {
        return displayName;
    }
}
