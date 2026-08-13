package org.marsroboticsassociation.controllib.motion

interface VelocityTrajectory {
    fun getAcceleration(t: Double): Double

    fun getVelocity(t: Double): Double

    val totalTime: Double

    fun isZeroJerk(t: Double): Boolean
}
