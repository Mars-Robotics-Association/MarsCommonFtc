package org.marsroboticsassociation.controllab.trajectory

enum class TrajectoryType(private val displayName: String) {
    SCURVE_POSITION("SCurvePosition"),
    SIN_CURVE_POSITION("Sinusoidal Position"),
    SCURVE_VELOCITY("SCurveVelocity");

    override fun toString(): String = displayName
}
