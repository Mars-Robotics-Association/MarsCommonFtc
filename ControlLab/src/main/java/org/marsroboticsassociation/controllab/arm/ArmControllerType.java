package org.marsroboticsassociation.controllab.arm;

/**
 * The controllers the Arm tab can drive the plant with. The first two are Lineage A
 * ({@code void update(dt)} over an {@link org.marsroboticsassociation.controllib.hardware.IMotor}
 * port); the last is Lineage B (pure {@code calculate(...)->voltage} plus its own EKF).
 */
public enum ArmControllerType {
    /** {@link org.marsroboticsassociation.controllib.control.ArmController}: SCurve + Kalman + PD. */
    ARM_PD,
    /** {@link org.marsroboticsassociation.controllib.control.VerticalArmController}: feedback-lin + LQR. */
    ARM_LQR,
    /** {@link org.marsroboticsassociation.controllib.mechanism.MotorMechanismController} + EKF. */
    MECHANISM_PIDF
}
