package org.marsroboticsassociation.controllib.control;

import java.util.function.LongSupplier;

import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt;
import org.marsroboticsassociation.controllib.filter.LowPassFilter;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.motion.VelocityTrajectoryManager;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Abstract base class for velocity-controlled motors with encoder feedback.
 * <p>
 * Extends {@link MotorBase} to add velocity measurement and filtering.
 * Subclasses implement specific control strategies (e.g., PF, SDK PIDF).
 */
public abstract class VelocityMotorBase extends MotorBase {

    protected final VelocityTrajectoryManager trajectory;

    private double tpsActual;
    private final LowPassFilter tpsLpf;

    public VelocityMotorBase(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                             double motorPowerChangeTolerance, IMotor motor,
                             double aMax, double jInc, double jDec, double vChangeTolerance) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor);
        tpsLpf = new BiquadLowPassVarDt(12, 1.0 / Math.sqrt(2.0));
        trajectory = new VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry);
        trajectory.updateConfig(aMax, jInc, jDec);
    }

    public VelocityMotorBase(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                             double motorPowerChangeTolerance, IMotor motor, String name,
                             double aMax, double jInc, double jDec, double vChangeTolerance) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor, name);
        tpsLpf = new BiquadLowPassVarDt(12, 1.0 / Math.sqrt(2.0));
        trajectory = new VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry);
        trajectory.updateConfig(aMax, jInc, jDec);
    }

    /** Package-private constructor for tests — injects a custom clock. */
    VelocityMotorBase(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                      double motorPowerChangeTolerance, IMotor motor,
                      double aMax, double jInc, double jDec, double vChangeTolerance,
                      LongSupplier clock) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor);
        tpsLpf = new BiquadLowPassVarDt(12, 1.0 / Math.sqrt(2.0));
        trajectory = new VelocityTrajectoryManager(aMax, jInc, vChangeTolerance, telemetry, clock);
        trajectory.updateConfig(aMax, jInc, jDec);
    }

    public double tpsToRpm(double tps) {
        return tps * 60 / motorPPR / gearRatio;
    }

    public double rpmToTps(double rpm) {
        return rpm * gearRatio * motorPPR / 60.0;
    }

    protected abstract void setTPS(double tps);

    public void setRPM(double rpm) {
        setTPS(rpmToTps(rpm));
    }

    public double getTpsSetpoint() { return trajectory.getTarget(); }

    public double getRpmSetpoint() {
        return tpsToRpm(getTpsSetpoint());
    }

    protected void setFilterCutoff(double cutoffHz) {
        tpsLpf.setCutoffHz(cutoffHz);
    }

    public abstract boolean isAtTargetSpeed();

    @Override
    protected void updateInternal(double dt) {
        tpsActual = motor.getVelocity();
        tpsLpf.update(tpsActual, dt);
    }

    @Override
    public void writeTelemetry() {
        telemetry.addData(name + " TPS setpoint", "%.0f", getTpsSetpoint());
        telemetry.addData(name + " RPM setpoint", "%.0f", getRpmSetpoint());
        telemetry.addData(name + " TPS measured", "%.0f", tpsActual);
        telemetry.addData(name + " RPM measured", "%.0f", tpsToRpm(tpsActual));
    }

    public double getTpsMeasurement() {
        return tpsActual;
    }

    public double getTpsFiltered() {
        return tpsLpf.getValue();
    }
}
