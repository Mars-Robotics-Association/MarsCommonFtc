package org.marsroboticsassociation.controllib.control;

import java.util.function.LongSupplier;

import edu.wpi.first.math.MathUtil;

import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt;
import org.marsroboticsassociation.controllib.filter.LowPassFilter;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Velocity-controlled motor using proportional + feedforward (PF) control.
 * <p>
 * Features jerk-limited motion profiling, voltage-scaled feedforward (kS, kV, kA),
 * proportional feedback, and low-pass filtered velocity/acceleration.
 * <p>
 * Motor direction and run mode are configured externally on the {@link IMotor} before
 * passing it to this constructor (e.g., via {@code EncapsulatedDcMotorEx} or
 * {@code LinkedMotorGroup}).
 */
public class VelocityMotorPF extends VelocityMotorBase {

    /**
     * Configuration parameters for the Proportional-Feedforward (PF) velocity controller.
     * <p>
     * This class encapsulates all tuning constants and physical properties required to
     * operate the {@link VelocityMotorPF} controller. It includes coefficients for
     * feedback (kP) and feedforward (kS, kV, kA) control, as well as constraints for
     * motion profiling (acceleration, jerk) and hardware specifications.
     */
    public static class VelocityMotorPFConfig {
        /**
         * Proportional Gain.
         * <p>
         * Multiplies the velocity error (target velocity - measured velocity).
         * Used to correct small errors and disturbances during steady-state motion.
         * Unit: Motor Power per (Ticks/Second error).
         */
        public double kP = 0.0045;

        /**
         * Static Friction Feedforward.
         * <p>
         * The y-intercept or offset for the velocity feedforward.
         * Unit: Volts.
         */
        public double kS = 0.8931;

        /**
         * Velocity Feedforward.
         * <p>
         * Models the voltage required to maintain a specific velocity, ignoring acceleration.
         * Ideally calculated as {@code Battery_Voltage / Max_Ticks_Per_Second}.
         * Unit: Volts per (Ticks/Second).
         */
        public double kV = 12.5 / 2632.1;

        /**
         * Acceleration Feedforward.
         * <p>
         * Models the voltage required to produce acceleration (F=ma).
         * Helps the controller anticipate power needs during the ramp-up and ramp-down phases
         * of the motion profile.
         * Unit: Volts per (Ticks/Second^2).
         */
        public double kA = 12.5 / 2087.9;

        /**
         * Fraction of the theoretical max velocity to reserve as headroom.
         * {@link #getMaxVelocity()} returns {@code (1 - headroomAllowance)} times
         * the physical max derived from kS, kV, and battery voltage.
         */
        public double headroomAllowance = 0.2;

        /**
         * Maximum Acceleration Constraint.
         * <p>
         * The maximum rate at which velocity can change. No sense having it
         * exceed what the motor is physically capable of.
         * Unit: Ticks per Second^2.
         */
        public double accelMax = 2500;

        /**
         * Jerk Constraint (Increasing Acceleration).
         * <p>
         * The derivative of acceleration. Controls the smoothness of the motion when
         * beginning a move (ramping up acceleration). Lower values reduce mechanical stress.
         * Unit: Ticks per Second^3.
         */
        public double jerkIncreasing = 2000;

        /**
         * Jerk Constraint (Decreasing Acceleration).
         * <p>
         * Controls the smoothness of the motion when approaching constant speed or
         * when beginning to decelerate.
         * Unit: Ticks per Second^3.
         */
        public double jerkDecreasing = 1000;

        /**
         * Measurement Low-Pass Filter Cutoff.
         * <p>
         * The frequency cutoff for filtering the raw encoder velocity. Lower values smooth out noise
         * but introduce phase lag (latency).
         * Unit: Hertz (Hz).
         */
        public double measurementLpfCutoffHz = 6.5;

        /**
         * Acceleration Low-Pass Filter Cutoff.
         * <p>
         * The frequency cutoff for filtering the calculated acceleration. This is used primarily
         * for the {@link #isAtTargetSpeed()} stability check.
         * Unit: Hertz (Hz).
         */
        public double accelLpfCutoffHz = 2;

        /**
         * Target Speed Tolerance.
         * <p>
         * The acceptable error range between target velocity and measured velocity
         * to consider the motor "at speed".
         * Unit: Ticks per Second.
         */
        public double targetSpeedTolerance = 15;

        /**
         * Acceleration Tolerance.
         * <p>
         * The acceptable range of acceleration fluctuation to consider the motor "stable".
         * Ensures the motor isn't oscillating even if the average speed is correct.
         * Unit: Ticks per Second^2.
         */
        public double accelerationTolerance = 75;

        /**
         * Feedforward Source Logic.
         * <p>
         * If {@code true}, feedforward is calculated based on the instantaneous velocity/acceleration
         * of the generated motion profile (smoother).
         * <p>
         * If {@code false}, feedforward is calculated based on the final target setpoint immediately.
         */
        public boolean ffFromTrajectory = true;
    }

    private final VelocityMotorPFConfig _config;
    private final LowPassFilter accelLpf;
    private double lastTpsSet = 0;
    private double voltage = 12.0;
    private double lastFilteredTps = 0;

    public VelocityMotorPF(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                           double motorPowerChangeTolerance, VelocityMotorPFConfig config,
                           IMotor motor) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor,
                config.accelMax, config.jerkIncreasing, config.jerkDecreasing, 10.0);
        _config = config;
        accelLpf = new BiquadLowPassVarDt(_config.accelLpfCutoffHz, 0.5);
    }

    /**
     * Public constructor for simulations — injects a custom clock.
     */
    public VelocityMotorPF(TelemetryAddData telemetry, double gearRatio, double motorPPR,
                    double motorPowerChangeTolerance, VelocityMotorPFConfig config,
                    IMotor motor, LongSupplier clock) {
        super(telemetry, gearRatio, motorPPR, motorPowerChangeTolerance, motor,
                config.accelMax, config.jerkIncreasing, config.jerkDecreasing, 10.0, clock);
        _config = config;
        accelLpf = new BiquadLowPassVarDt(_config.accelLpfCutoffHz, 0.5);
    }

    @Override
    public void setTPS(double tps) {
        if (tps != lastTpsSet) {
            voltage = getVoltage();
            double vmax = getMaxVelocity();
            tps = MathUtil.clamp(tps, -vmax, vmax);
            if (lastTpsSet == 0) {
                trajectory.resetFromMeasurement(getTpsFiltered());
            }
            trajectory.setTarget(tps);
            lastTpsSet = tps;
        }
    }

    /**
     * Returns the conservative maximum velocity at the last known battery voltage.
     * Derived from feedforward model: {@code (1 - headroomAllowance) * (voltage - kS) / kV}.
     */
    public double getMaxVelocity() {
        return (1.0 - _config.headroomAllowance) * (voltage - _config.kS) / _config.kV;
    }

    @Override
    protected void updateInternal(double dt) {
        super.setFilterCutoff(_config.measurementLpfCutoffHz);
        super.updateInternal(dt);

        accelLpf.setCutoffHz(_config.accelLpfCutoffHz);

        trajectory.updateConfig(_config.accelMax, _config.jerkIncreasing, _config.jerkDecreasing);
        trajectory.update();

        if (trajectory.getAcceleration() == 0) {
            voltage = getVoltage();
        }

        double v = trajectory.getVelocity();
        double t = _config.ffFromTrajectory ? v : trajectory.getTarget();
        double a = trajectory.getAcceleration();
        double ff = _config.kS * Math.signum(t) + _config.kV * t + _config.kA * a;
        ff /= voltage;
        double tpsFiltered = getTpsFiltered();
        if (dt > 1e-6) {
            double accel = (tpsFiltered - lastFilteredTps) / dt;
            if (!Double.isInfinite(accel) && !Double.isNaN(accel)) {
                accelLpf.update(accel, dt);
            }
        }
        lastFilteredTps = tpsFiltered;
        double ve = v - tpsFiltered;
        double kPEffective = _config.kP * Math.max(0, 1.0 - Math.abs(a) / _config.accelMax);
        double power = ff + kPEffective * ve;

        if (Math.abs(trajectory.getTarget()) < 1e-6) {
            setPower(0);
        } else {
            setPower(power);
        }
    }

    @Override
    public boolean isAtTargetSpeed() {
        return Math.abs(trajectory.getTarget()) > 200 &&
                trajectory.getAcceleration() == 0 &&
                Math.abs(trajectory.getTarget() - getTpsFiltered()) < _config.targetSpeedTolerance &&
                Math.abs(accelLpf.getValue()) < _config.accelerationTolerance;
    }

    @Override
    public void writeTelemetry() {
        super.writeTelemetry();
        telemetry.addData(name + " TPS LPF", "%.1f", getTpsFiltered());
        telemetry.addData(name + " RPM LPF", "%.1f", tpsToRpm(getTpsFiltered()));
        telemetry.addData(name + " accel LPF", "%.1f", accelLpf.getValue());
        telemetry.addData("voltage lpf", "%.2f", getVoltage());
        telemetry.addData("voltage used", "%.2f", voltage);
    }

    @Override
    public void stop() {
        trajectory.setTarget(0.0);
        setPower(0);
        lastTpsSet = 0;
    }
}
