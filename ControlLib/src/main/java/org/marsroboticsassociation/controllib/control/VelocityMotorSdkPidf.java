package org.marsroboticsassociation.controllib.control;

import java.util.function.LongSupplier;

import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Velocity-controlled motor using the SDK's built-in PIDF controller.
 * <p>
 * The motor must be set to {@code RUN_USING_ENCODER} mode before being passed to this
 * constructor. Motor direction and zero-power behavior are configured externally on the
 * {@link IMotor}.
 * <p>
 * {@link #setPower(double)} is not supported; use {@link #setRPM}/{@link #setTPS} instead.
 */
public class VelocityMotorSdkPidf extends VelocityMotorBase {

    public static class MotorPIDFConfig {
        public double Kv;
        public double nominalVoltage;
        public double Kp;
        public double Ki;
        public double Kd;
        public double maxSettableVelocity;
        public double maxAccel;
        public double jerkIncreasing;
        public double jerkDecreasing;
        public double lpfCutoff;

        public MotorPIDFConfig(double maxTPS) {
            Kv = 32767.0 / maxTPS;
            nominalVoltage = 12.0;
            Kp = 0.1 * Kv;
            Ki = 0.1 * Kp;
            Kd = 0.01 * Kp;
            maxSettableVelocity = 0.8 * maxTPS;
            maxAccel = 1196.7;
            jerkIncreasing = 2669.2;
            jerkDecreasing = 800;
            lpfCutoff = 4;
        }
    }

    public final MotorPIDFConfig config;

    private final SetOnChange<Double> motorVelocitySetpoint;
    private final SetOnChange<Double> voltageFactor;
    private final SetOnChange<Double> Kp;
    private final SetOnChange<Double> Ki;
    private final SetOnChange<Double> Kd;
    private final SetOnChange<Double> Kv;

    public VelocityMotorSdkPidf(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                                 MotorPIDFConfig config, IMotor motor) {
        super(telemetry, gearRatio, motorPPR, 0.05, motor,
                config.maxAccel, config.jerkIncreasing, config.jerkDecreasing, 1.0);
        this.config = config;
        motorVelocitySetpoint = SetOnChange.ofDouble(0.0, 1.0, motor::setVelocity);
        Runnable updatePIDF = () -> motor.setVelocityPIDFCoefficients(
                config.Kp, config.Ki, config.Kd,
                config.Kv * config.nominalVoltage / getVoltage());
        voltageFactor = SetOnChange.ofDouble(config.nominalVoltage / getVoltage(), 0.025, (vf) -> updatePIDF.run());
        Kp = SetOnChange.ofDouble(config.Kp, (kp) -> updatePIDF.run());
        Ki = SetOnChange.ofDouble(config.Ki, (ki) -> updatePIDF.run());
        Kd = SetOnChange.ofDouble(config.Kd, (kd) -> updatePIDF.run());
        Kv = SetOnChange.ofDouble(config.Kv, (kv) -> updatePIDF.run());
    }

    /** Package-private constructor for tests — injects a custom clock. */
    VelocityMotorSdkPidf(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                          MotorPIDFConfig config, IMotor motor, LongSupplier clock) {
        super(telemetry, gearRatio, motorPPR, 0.05, motor,
                config.maxAccel, config.jerkIncreasing, config.jerkDecreasing, 1.0, clock);
        this.config = config;
        motorVelocitySetpoint = SetOnChange.ofDouble(0.0, 1.0, motor::setVelocity);
        Runnable updatePIDF = () -> motor.setVelocityPIDFCoefficients(
                config.Kp, config.Ki, config.Kd,
                config.Kv * config.nominalVoltage / getVoltage());
        voltageFactor = SetOnChange.ofDouble(config.nominalVoltage / getVoltage(), 0.025, (vf) -> updatePIDF.run());
        Kp = SetOnChange.ofDouble(config.Kp, (kp) -> updatePIDF.run());
        Ki = SetOnChange.ofDouble(config.Ki, (ki) -> updatePIDF.run());
        Kd = SetOnChange.ofDouble(config.Kd, (kd) -> updatePIDF.run());
        Kv = SetOnChange.ofDouble(config.Kv, (kv) -> updatePIDF.run());
    }

    @Override
    public void setTPS(double tps) {
        tps = MathUtil.clamp(tps, -config.maxSettableVelocity, config.maxSettableVelocity);
        trajectory.setTarget(tps);
    }

    @Override
    public boolean isAtTargetSpeed() {
        return trajectory.getAcceleration() == 0 && Math.abs(getTpsFiltered() - trajectory.getTarget()) < 30;
    }

    @Override
    public void setPower(double power) {
        throw new UnsupportedOperationException("VelocityMotorSdkPidf is velocity-only; use setRPM/setTPS");
    }

    @Override
    protected void updateInternal(double dt) {
        super.setFilterCutoff(config.lpfCutoff);
        super.updateInternal(dt);
        trajectory.updateConfig(config.maxAccel, config.jerkIncreasing, config.jerkDecreasing);
        Kp.set(config.Kp);
        Ki.set(config.Ki);
        Kd.set(config.Kd);
        Kv.set(config.Kv);
        voltageFactor.set(config.nominalVoltage / getVoltage());
        trajectory.update();
        if (Math.abs(trajectory.getTarget()) < 1e-6) {
            stop();
        } else {
            motorVelocitySetpoint.set(trajectory.getVelocity());
        }
    }

    @Override
    public void writeTelemetry() {
        super.writeTelemetry();
        telemetry.addData(name + " Voltage Factor", "%.3f", voltageFactor.get());
        telemetry.addData(name + " TPS LPF", "%.1f", getTpsFiltered());
        telemetry.addData(name + " RPM LPF", "%.1f", tpsToRpm(getTpsFiltered()));
    }

    @Override
    public void stop() {
        trajectory.setTarget(0.0);
        motor.setVelocity(0);
    }
}
