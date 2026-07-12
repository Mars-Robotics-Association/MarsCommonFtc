package org.marsroboticsassociation.controllab.arm;

import org.marsroboticsassociation.controllib.mechanism.ArmModel;
import org.marsroboticsassociation.controllib.mechanism.MotorMechanismController;
import org.marsroboticsassociation.controllib.mechanism.MotorMechanismEkf;

/**
 * Lineage B adapter: wraps {@link MotorMechanismController} (pure {@code calculate(...)->voltage})
 * plus its own {@link MotorMechanismEkf} state estimator and a shared {@link ArmModel}.
 *
 * <p>This is the <b>only</b> unit-conversion seam in the project: the plant reports the encoder in
 * ticks / TPS, while the mechanism controller and EKF work in radians / rad·s⁻¹. The conversion is
 * isolated in {@link #ticksToRad}/{@link #tpsToRadPerSec} and unit-tested.
 */
class MechanismArmAdapter implements ArmControlAdapter {

    /** Editable gains, gathered so a rebuild is a single call. */
    static class Gains {
        double kP = 40.0;
        double kI = 8.0;
        double kD = 1.5;
        double kCos = 3.5;   // gravity at horizontal (== plant kG by default, heavy end-effector)
        double kSin = 0.0;   // center-of-mass angular offset term
        double kS = 0.3;     // static friction
        double kV = 1.2;     // back-EMF / viscous
        double kA = 0.35;    // inertia (matches the heavy plant default)
        double maxVel = 8.0;
        double maxAccel = 12.0;
        double maxJerk = 480.0;
        double feedbackVoltageMargin = 1.5;
        double velocityLagSec = 0.025;
        double modelAccelStdDev = 5.0;
        double positionStdDev = 0.003;
        double velocityStdDev = 0.1;
        double positionTimingJitterStdDev = 0.0;
    }

    private final Gains gains;
    private final int ticksPerRev;
    private final double gearRatio;
    private final double encoderZeroOffsetRad;

    private ArmPlant plant;
    private ArmModel model;
    private MotorMechanismController controller;
    private MotorMechanismEkf ekf;

    private double targetRad;
    private double lastPower = 0.0;

    MechanismArmAdapter(Gains gains, ArmPlant plant,
                        int ticksPerRev, double gearRatio, double encoderZeroOffsetRad) {
        this.gains = gains;
        this.plant = plant;
        this.ticksPerRev = ticksPerRev;
        this.gearRatio = gearRatio;
        this.encoderZeroOffsetRad = encoderZeroOffsetRad;
        double posRad = currentMeasuredPosRad();
        this.targetRad = posRad;
        build(posRad);
    }

    /** (Re)build the model, controller, and EKF seeded from the given pose. */
    private void build(double initialPosRad) {
        model = new ArmModel(gains.kS, gains.kV, gains.kA, gains.kCos, gains.kSin);
        controller = new MotorMechanismController(
                model, gains.kP, gains.kI, gains.kD,
                gains.maxVel, gains.maxAccel, gains.maxJerk,
                gains.feedbackVoltageMargin, initialPosRad);
        ekf = new MotorMechanismEkf(
                model, gains.velocityLagSec, gains.modelAccelStdDev,
                gains.positionStdDev, gains.velocityStdDev,
                gains.positionTimingJitterStdDev, initialPosRad);
        lastPower = 0.0;
    }

    /** Rebuild after a gain edit, reseeding controller + EKF from the current pose. */
    void rebuild() {
        build(currentMeasuredPosRad());
    }

    @Override public void setPlant(ArmPlant plant) {
        this.plant = plant;
    }

    @Override public void setTargetRad(double rad) {
        this.targetRad = rad;
    }

    @Override public void step(double dt, double hubVoltage) {
        double posRad = currentMeasuredPosRad();
        double velRad = tpsToRadPerSec(plant.getVelocityTps());

        ekf.predict(dt, lastPower, hubVoltage);
        ekf.correct(posRad, velRad);

        double volts = controller.calculate(
                targetRad, ekf.getPosition(), ekf.getVelocity(), hubVoltage, dt);
        lastPower = volts / hubVoltage;
    }

    @Override public double commandedPower()  { return lastPower; }
    @Override public double estimatedPosRad() { return ekf.getPosition(); }
    @Override public double estimatedVelRad() { return ekf.getVelocity(); }
    @Override public double trajPosRad()      { return controller.getSetpointPosition(); }
    @Override public double trajVelRad()      { return controller.getSetpointVelocity(); }
    @Override public String modeLabel()       { return "MECHANISM_PIDF"; }

    private double currentMeasuredPosRad() {
        return ticksToRad(plant.getPositionTicks());
    }

    private double ticksToRad(int ticks) {
        return ticksToRad(ticks, ticksPerRev, gearRatio, encoderZeroOffsetRad);
    }

    private double tpsToRadPerSec(double tps) {
        return tpsToRadPerSec(tps, ticksPerRev, gearRatio);
    }

    // ── the isolated, unit-tested conversion seam ────────────────────────────────

    /** Convert motor encoder ticks to output-shaft radians from horizontal. */
    static double ticksToRad(int ticks, int ticksPerRev, double gearRatio,
                             double encoderZeroOffsetRad) {
        return ticks * 2.0 * Math.PI / (ticksPerRev * gearRatio) + encoderZeroOffsetRad;
    }

    /** Convert motor encoder ticks-per-second to output-shaft rad/s. */
    static double tpsToRadPerSec(double tps, int ticksPerRev, double gearRatio) {
        return tps * 2.0 * Math.PI / (ticksPerRev * gearRatio);
    }
}
