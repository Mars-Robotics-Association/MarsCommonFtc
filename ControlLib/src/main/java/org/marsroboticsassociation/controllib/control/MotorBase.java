package org.marsroboticsassociation.controllib.control;

import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Abstract base class for controlling a motor (or motor group via {@code LinkedMotorGroup}).
 * <p>
 * Accepts a single {@link IMotor} — multi-motor grouping is handled externally.
 * Provides quantized power updates to reduce unnecessary motor commands.
 */
public abstract class MotorBase {
    protected final IMotor motor;
    protected final TelemetryAddData telemetry;
    protected final String name;
    protected final double gearRatio;
    protected final double motorPPR;

    private double lastPower = Double.NaN;
    private final double quantStep;

    public MotorBase(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                     double motorPowerChangeTolerance, IMotor motor) {
        this.motor = motor;
        this.name = motor.getName();
        this.gearRatio = gearRatio;
        this.motorPPR = motorPPR;
        this.telemetry = telemetry;
        this.quantStep = motorPowerChangeTolerance;
    }

    /**
     * Sets power to the motor, only if the quantized value changes from the last command.
     * Power is clamped to [-1, 1] and quantized to reduce command noise.
     */
    public void setPower(double power) {
        if (Double.isNaN(power)) {
            telemetry.addData("power", "%s", "NaN");
            return;
        } else if (Double.isInfinite(power)) {
            telemetry.addData("power", "%s", "Infinite");
            return;
        } else {
            telemetry.addData("power", "%.2f", power);
        }

        power = MathUtil.clamp(power, -1.0, 1.0);
        double quantPower = Math.round(power / quantStep) * quantStep;
        if (Double.isNaN(lastPower) || quantPower != lastPower) {
            motor.setPower(quantPower);
            lastPower = quantPower;
        }
    }

    public double getVoltage() {
        return motor.getHubVoltage();
    }

    public final void update(double dt) {
        if (dt < 1e-6) return;
        updateInternal(dt);
    }

    protected abstract void updateInternal(double dt);

    public abstract void writeTelemetry();

    public abstract void stop();
}
