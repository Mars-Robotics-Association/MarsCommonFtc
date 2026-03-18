// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math.controller;

//import edu.wpi.first.math.controller.proto.ArmFeedforwardProto;
//import edu.wpi.first.math.controller.struct.ArmFeedforwardStruct;
//import edu.wpi.first.util.protobuf.ProtobufSerializable;
//import edu.wpi.first.util.struct.StructSerializable;

/**
 * A helper class that computes feedforward outputs for a simple arm (modeled as a motor acting
 * against the force of gravity on a beam suspended at an angle).
 */
public class ArmFeedforward
//        implements ProtobufSerializable, StructSerializable
{
  /** The static gain, in volts. */
  private double ks;

  /** The gravity gain, in volts. */
  private double kg;

  /** The velocity gain, in V/(rad/s). */
  private double kv;

  /** The acceleration gain, in V/(rad/s²). */
  private double ka;

  /** The period, in seconds. */
  private final double m_dt;

  /**
   * Creates a new ArmFeedforward with the specified gains and period.
   *
   * @param ks The static gain in volts.
   * @param kg The gravity gain in volts.
   * @param kv The velocity gain in V/(rad/s).
   * @param ka The acceleration gain in V/(rad/s²).
   * @param dtSeconds The period in seconds.
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   * @throws IllegalArgumentException for period &le; zero.
   */
  public ArmFeedforward(double ks, double kg, double kv, double ka, double dtSeconds) {
    this.ks = ks;
    this.kg = kg;
    this.kv = kv;
    this.ka = ka;
    if (kv < 0.0) {
      throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
    }
    if (ka < 0.0) {
      throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
    }
    if (dtSeconds <= 0.0) {
      throw new IllegalArgumentException(
          "period must be a positive number, got " + dtSeconds + "!");
    }
    m_dt = dtSeconds;
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. The period is defaulted to 20 ms.
   *
   * @param ks The static gain in volts.
   * @param kg The gravity gain in volts.
   * @param kv The velocity gain in V/(rad/s).
   * @param ka The acceleration gain in V/(rad/s²).
   * @throws IllegalArgumentException for kv &lt; zero.
   * @throws IllegalArgumentException for ka &lt; zero.
   */
  public ArmFeedforward(double ks, double kg, double kv, double ka) {
    this(ks, kg, kv, ka, 0.020);
  }

  /**
   * Creates a new ArmFeedforward with the specified gains. The period is defaulted to 20 ms.
   *
   * @param ks The static gain in volts.
   * @param kg The gravity gain in volts.
   * @param kv The velocity gain in V/(rad/s).
   * @throws IllegalArgumentException for kv &lt; zero.
   */
  public ArmFeedforward(double ks, double kg, double kv) {
    this(ks, kg, kv, 0);
  }

  /**
   * Sets the static gain.
   *
   * @param ks The static gain in volts.
   */
  public void setKs(double ks) {
    this.ks = ks;
  }

  /**
   * Sets the gravity gain.
   *
   * @param kg The gravity gain in volts.
   */
  public void setKg(double kg) {
    this.kg = kg;
  }

  /**
   * Sets the velocity gain.
   *
   * @param kv The velocity gain in V/(rad/s).
   */
  public void setKv(double kv) {
    this.kv = kv;
  }

  /**
   * Sets the acceleration gain.
   *
   * @param ka The acceleration gain in V/(rad/s²).
   */
  public void setKa(double ka) {
    this.ka = ka;
  }

  /**
   * Returns the static gain in volts.
   *
   * @return The static gain in volts.
   */
  public double getKs() {
    return ks;
  }

  /**
   * Returns the gravity gain in volts.
   *
   * @return The gravity gain in volts.
   */
  public double getKg() {
    return kg;
  }

  /**
   * Returns the velocity gain in V/(rad/s).
   *
   * @return The velocity gain.
   */
  public double getKv() {
    return kv;
  }

  /**
   * Returns the acceleration gain in V/(rad/s²).
   *
   * @return The acceleration gain.
   */
  public double getKa() {
    return ka;
  }

  /**
   * Returns the period in seconds.
   *
   * @return The period in seconds.
   */
  public double getDt() {
    return m_dt;
  }

  /**
   * Calculates the feedforward from the gains and setpoints.
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocityRadPerSec The velocity setpoint.
   * @param accelRadPerSecSquared The acceleration setpoint.
   * @return The computed feedforward.
   * @deprecated Use {@link #calculateWithVelocities(double, double, double)} instead
   */
  @Deprecated(forRemoval = true, since = "2025")
  public double calculate(
      double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
    return ks * Math.signum(velocityRadPerSec)
        + kg * Math.cos(positionRadians)
        + kv * velocityRadPerSec
        + ka * accelRadPerSecSquared;
  }

  /**
   * Calculates the feedforward from the gains and velocity setpoint assuming continuous control
   * (acceleration is assumed to be zero).
   *
   * @param positionRadians The position (angle) setpoint. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocity The velocity setpoint.
   * @return The computed feedforward.
   */
  public double calculate(double positionRadians, double velocity) {
    return calculate(positionRadians, velocity, 0);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming continuous control.
   *
   * @param currentAngle The current angle in radians. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel to the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param currentVelocity The current velocity setpoint in radians per second.
   * @param nextVelocity The next velocity setpoint in radians per second.
   * @param dt Time between velocity setpoints in seconds.
   * @return The computed feedforward in volts.
   * @deprecated Use {@link #calculateWithVelocities(double, double, double)} instead.
   */
  @Deprecated(forRemoval = true, since = "2025")
  public double calculate(
      double currentAngle, double currentVelocity, double nextVelocity, double dt) {
    return new ArmFeedforward(ks, kg, kv, ka, dt)
        .calculateWithVelocities(currentAngle, currentVelocity, nextVelocity);
  }

  /**
   * Calculates the feedforward from the gains and setpoints assuming discrete control.
   *
   * @param currentAngle The current angle in radians. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel to the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param currentVelocity The current velocity setpoint in radians per second.
   * @param nextVelocity The next velocity setpoint in radians per second.
   * @return The computed feedforward in volts.
   */
  public double calculateWithVelocities(
      double currentAngle, double currentVelocity, double nextVelocity) {
    // Simple closed-form when kA is small (avoids numerical issues in Newton loop).
    if (ka < 0.1) {
      return ks * Math.signum(currentVelocity)
          + kg * Math.cos(currentAngle)
          + kv * currentVelocity
          + ka * (nextVelocity - currentVelocity) / m_dt;
    }

    // RK4 + Newton's method: find u such that integrating arm dynamics for one
    // timestep with constant voltage u yields omega(t + dt) == nextVelocity.
    // Initial guess from the closed-form (exact for linear dynamics).
    double u =
        ks * Math.signum(currentVelocity)
            + kg * Math.cos(currentAngle)
            + kv * currentVelocity
            + ka * (nextVelocity - currentVelocity) / m_dt;

    double prevAbsG = Double.MAX_VALUE;
    for (int i = 0; i < 50; i++) {
      double[] result = rk4ArmAugmented(currentAngle, currentVelocity, u);
      double omegaNext = result[0];
      double dOmegaDu = result[1];

      double error = omegaNext - nextVelocity;
      double cost = error * error;
      double g = 2.0 * error * dOmegaDu;
      double H = 2.0 * dOmegaDu * dOmegaDu;
      double step = -g / Math.max(H, 1e-4);

      // Stop if the gradient is no longer decreasing.
      if (Math.abs(g) >= (1.0 - 1e-10) * prevAbsG) {
        break;
      }
      prevAbsG = Math.abs(g);

      // Backtracking line search.
      double alpha = 1.0;
      for (int j = 0; j < 64; j++) {
        double[] newResult = rk4ArmAugmented(currentAngle, currentVelocity, u + alpha * step);
        double newError = newResult[0] - nextVelocity;
        if (newError * newError <= cost) {
          break;
        }
        alpha *= 0.5;
      }
      u += alpha * step;
    }
    return u;
  }

  // Rearranging the main equation from the calculate() method yields the
  // formulas for the methods below:

  /**
   * Calculates the maximum achievable velocity given a maximum voltage supply, a position, and an
   * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the acceleration constraint, and this will give
   * you a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm.
   * @param angle The angle of the arm, in radians. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param acceleration The acceleration of the arm, in (rad/s²).
   * @return The maximum possible velocity in (rad/s) at the given acceleration and angle.
   */
  public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
    // Assume max velocity is positive
    return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
  }

  /**
   * Calculates the minimum achievable velocity given a maximum voltage supply, a position, and an
   * acceleration. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the acceleration constraint, and this will give
   * you a simultaneously-achievable velocity constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm, in volts.
   * @param angle The angle of the arm, in radians. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param acceleration The acceleration of the arm, in (rad/s²).
   * @return The minimum possible velocity in (rad/s) at the given acceleration and angle.
   */
  public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
    // Assume min velocity is negative, ks flips sign
    return (-maxVoltage + ks - Math.cos(angle) * kg - acceleration * ka) / kv;
  }

  /**
   * Calculates the maximum achievable acceleration given a maximum voltage supply, a position, and
   * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm, in volts.
   * @param angle The angle of the arm, in radians. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocity The velocity of the elevator, in (rad/s)
   * @return The maximum possible acceleration in (rad/s²) at the given velocity.
   */
  public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
    return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg - velocity * kv) / ka;
  }

  /**
   * Calculates the minimum achievable acceleration given a maximum voltage supply, a position, and
   * a velocity. Useful for ensuring that velocity and acceleration constraints for a trapezoidal
   * profile are simultaneously achievable - enter the velocity constraint, and this will give you a
   * simultaneously-achievable acceleration constraint.
   *
   * @param maxVoltage The maximum voltage that can be supplied to the arm, in volts.
   * @param angle The angle of the arm, in radians. This angle should be measured from the
   *     horizontal (i.e. if the provided angle is 0, the arm should be parallel with the floor). If
   *     your encoder does not follow this convention, an offset should be added.
   * @param velocity The velocity of the elevator, in (rad/s)
   * @return The maximum possible acceleration in (rad/s²) at the given velocity.
   */
  public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
    return maxAchievableAcceleration(-maxVoltage, angle, velocity);
  }

  /**
   * Arm dynamics augmented with sensitivity equations.
   *
   * <p>State layout: [angle, omega, s_angle, s_omega] where s = d[angle, omega]/du with s(0) =
   * [0, 0].
   */
  private double[] armDynamicsAugmented(double[] x, double u) {
    double omega = x[1];
    double sOmega = x[3];
    return new double[] {
      omega,
      -kv / ka * omega + u / ka - Math.signum(omega) * ks / ka - Math.cos(x[0]) * kg / ka,
      sOmega,
      -kv / ka * sOmega + 1.0 / ka
    };
  }

  /**
   * Returns [omegaNext, dOmegaNext/du] via one RK4 step of the augmented system.
   *
   * <p>This gives the exact derivative dOmegaNext/du at no extra RK4 cost (sensitivity equations).
   */
  private double[] rk4ArmAugmented(double angle, double omega, double u) {
    double h = m_dt;
    double[] x0 = {angle, omega, 0.0, 0.0};
    double[] k1 = armDynamicsAugmented(x0, u);
    double[] x1 = new double[4];
    for (int j = 0; j < 4; j++) x1[j] = x0[j] + 0.5 * h * k1[j];
    double[] k2 = armDynamicsAugmented(x1, u);
    double[] x2 = new double[4];
    for (int j = 0; j < 4; j++) x2[j] = x0[j] + 0.5 * h * k2[j];
    double[] k3 = armDynamicsAugmented(x2, u);
    double[] x3 = new double[4];
    for (int j = 0; j < 4; j++) x3[j] = x0[j] + h * k3[j];
    double[] k4 = armDynamicsAugmented(x3, u);
    double omegaNext = x0[1] + h / 6.0 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
    double dOmegaDu = x0[3] + h / 6.0 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
    return new double[] {omegaNext, dOmegaDu};
  }

  /** Arm feedforward struct for serialization. */
//  public static final ArmFeedforwardStruct struct = new ArmFeedforwardStruct();

  /** Arm feedforward protobuf for serialization. */
//  public static final ArmFeedforwardProto proto = new ArmFeedforwardProto();
}
