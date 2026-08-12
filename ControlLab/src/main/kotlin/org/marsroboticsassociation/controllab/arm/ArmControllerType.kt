package org.marsroboticsassociation.controllab.arm

/**
 * The controllers the Arm tab can drive the plant with. The first two are Lineage A (`void
 * update(dt)` over an [org.marsroboticsassociation.controllib.hardware.IMotor] port); the last is
 * Lineage B (pure `calculate(...)->voltage` plus its own EKF).
 */
enum class ArmControllerType {
    /** [org.marsroboticsassociation.controllib.control.ArmController]: SCurve + Kalman + PD. */
    ARM_PD,
    /**
     * [org.marsroboticsassociation.controllib.control.VerticalArmController]: feedback-lin + LQR.
     */
    ARM_LQR,
    /** [org.marsroboticsassociation.controllib.mechanism.MotorMechanismController] + EKF. */
    MECHANISM_PIDF,
}
