package org.marsroboticsassociation.controllib.control;

import java.util.function.LongSupplier;

import edu.wpi.first.math.MathUtil;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

/**
 * Simplified flywheel control with voltage-based feedforward + proportional feedback,
 * battery voltage compensation, and a clamped-exponential motion profile.
 * <p>
 * Coasting / spin-up behavior:
 * - When tps is 0, the motor is set to power = 0 (coast).
 * - On resume from zero, the velocity filter is re-seeded from the current encoder
 *   reading and the profile resumes from the current (coasting) speed.
 */
public class FlywheelSimple {

    /**
     * Parameters for tuning
     */
    public static class Params {
        /** Low-pass filter cutoff frequency for measured velocity, in Hz. */
        public double velLpfCutoffHz = 4.0;

        /** Feedforward voltage constant (voltage per unit velocity). From FlywheelsFeedforwardTuning. */
        public double kV = 12.5 / 2632.1;
        /** Feedforward voltage constant (voltage per unit acceleration). From FlywheelsFeedforwardTuning. */
        public double kA = 12.5 / 2087.9;
        /** Feedforward voltage constant (Coulomb friction). From FlywheelsFeedforwardTuning. */
        public double kS = 0.8931;

        /** Proportional feedback gain (volts per tps error). */
        public double kP = 0.010;
        /** Maximum feedback as a fraction of hub voltage. */
        public double fbMax = 0.2;

        /** Maximum duration of a single profile step, in seconds. Caps dt to prevent gap overshoot. */
        public double maxProfileDt = 0.060;

        /**
         * Maximum acceleration cap for the motion profile, in TPS^2.
         *
         * <p>The actual acceleration limit at any point is:
         * {@code min(maxAccel, (V - kS - kV * profiledVelocity) / kA)},
         * accounting for back-EMF stealing voltage at higher speeds.
         */
        public double maxAccel = (12.0 - kS) / kA;

        /** Velocity threshold (TPS) within which isReady() returns true. */
        public double readyThreshold = 40.0;
        /** Velocity error (TPS) below which the profile snaps to the setpoint. */
        public double snapThresholdTPS = 10.0;
    }

    /**
     * Global tuning parameters. Shared between instances.
     */
    public static Params PARAMS = new Params();

    private final IMotor motor;
    private final String primaryName;   // used as telemetry prefix
    private final double hubVoltage;
    private final TelemetryAddData telemetry;
    private final LongSupplier clock;
    private double smoothVelocity = 0;

    private double profiledVelocity = 0;
    private boolean stopped = true;
    private long lastTimeNanos = 0;

    private double rawSetpoint = 0;     // commanded TPS
    private double rawVelocity = 0;     // last measured velocity
    private double lastPower = 0;       // last power sent to motors
    private boolean powerTooLowForTarget = false;

    /**
     * @param telemetry typically telemetry::addData
     * @param motor     motor providing both encoder feedback and power output
     */
    public FlywheelSimple(TelemetryAddData telemetry, IMotor motor) {
        this.motor       = motor;
        this.hubVoltage  = motor.getHubVoltage();
        this.primaryName = motor.getName();
        this.telemetry   = telemetry;
        this.clock       = System::nanoTime;
    }

    /**
     * Returns the current profiled (commanded) velocity in ticks per second.
     * This is the velocity the controller is actively targeting via feedforward,
     * which may differ from the measured velocity during spin-up or disturbance recovery.
     */
    public double getProfiledVelocity() { return profiledVelocity; }

    /** Returns the low-pass filtered measured velocity in ticks per second. */
    public double getFilteredVelocity() { return smoothVelocity; }

    /** For unit tests and simulations — allows injecting a custom clock. */
    public FlywheelSimple(TelemetryAddData telemetry, LongSupplier clock, IMotor motor) {
        this.motor       = motor;
        this.hubVoltage  = motor.getHubVoltage();
        this.primaryName = motor.getName();
        this.telemetry   = telemetry;
        this.clock       = clock;
    }

    /**
     * Command a new target velocity (ticks per second).
     */
    public void setTps(double tps) {
        rawSetpoint = Math.max(0, tps);
    }

    /**
     * Update the flywheel controller. Should be called once on each control loop.
     */
    public void update() {
        // ── Timing ──────────────────────────────────────────────────────
        long now = clock.getAsLong();
        if (lastTimeNanos == 0) {
            lastTimeNanos = now;
            return;
        }
        double dt = (now - lastTimeNanos) / 1e9;
        lastTimeNanos = now;
        if (dt < 1e-6) return;

        // ── Velocity filter (always runs so readings stay fresh) ─────────
        double filterTau = 1.0 / (2.0 * Math.PI * PARAMS.velLpfCutoffHz);
        double alpha = 1.0 - Math.exp(-dt / filterTau);
        rawVelocity = motor.getVelocity();
        smoothVelocity = alpha * rawVelocity + (1.0 - alpha) * smoothVelocity;

        // ── Coast when target is zero ────────────────────────────────────
        if (rawSetpoint == 0) {
            motor.setPower(0);
            lastPower = 0;
            stopped = true;
            return;
        }

        // ── Seed profile on resume from stopped ──────────────────────────
        // Reset the filter so the profile and feedback start from a clean
        // encoder reading, not a stale smoothed value from before the gap.
        if (stopped) {
            smoothVelocity = rawVelocity;
            profiledVelocity = rawVelocity;
            stopped = false;
        }

        // ── Velocity ramp profile ────────────────────────────────────────
        // Ramp toward target at accelLimit, decelerate when approaching.
        // accelLimit accounts for back-EMF: (V - kS - kV * profiledVelocity) / kA.
        double profileDt = Math.min(dt, PARAMS.maxProfileDt);
        double accelLimit = (hubVoltage - PARAMS.kS - PARAMS.kV * profiledVelocity) / PARAMS.kA;
        accelLimit = Math.min(PARAMS.maxAccel, Math.max(accelLimit, 0));
        double error = rawSetpoint - profiledVelocity;
        double accel = Math.signum(error) * accelLimit;
        profiledVelocity += accel * profileDt;

        // Snap to target once within one quantization step (avoids asymptotic creep)
        if (Math.abs(rawSetpoint - profiledVelocity) < PARAMS.snapThresholdTPS) {
            profiledVelocity = rawSetpoint;
            accel = 0;
        }
        // ── Voltage-based feedforward + proportional feedback ────────────
        double ff_voltage = PARAMS.kS * Math.signum(profiledVelocity)
                + PARAMS.kV * profiledVelocity
                + PARAMS.kA * accel;
        double fb_voltage = PARAMS.kP * (profiledVelocity - smoothVelocity);
        fb_voltage = MathUtil.clamp(fb_voltage, -PARAMS.fbMax * hubVoltage, PARAMS.fbMax * hubVoltage);

        lastPower = MathUtil.clamp((ff_voltage + fb_voltage) / hubVoltage, -1.0, 1.0);
        motor.setPower(lastPower);

        // ── Update powerTooLowForTarget ───────────────────────────────────
        double ff_fraction = (PARAMS.kS + PARAMS.kV * rawSetpoint) / hubVoltage;
        powerTooLowForTarget = ff_fraction > 0.85;
    }

    /**
     * Returns true when both the profile has settled and measured velocity is
     * within {@link Params#readyThreshold} of the target.
     */
    public boolean isReady() {
        return rawSetpoint != 0
                && profiledVelocity == rawSetpoint
                && Math.abs(smoothVelocity - rawSetpoint) < PARAMS.readyThreshold;
    }

    /**
     * Returns true if the steady-state feedforward at current battery voltage
     * leaves insufficient headroom for feedback to reliably reach the target velocity.
     */
    public boolean isPowerTooLowForTargetVelocity() {
        return powerTooLowForTarget;
    }

    /**
     * Add flywheel telemetry to driver station.
     */
    public void writeTelemetry() {
        telemetry.addData(primaryName + " setpoint", "%.0f", rawSetpoint);
        telemetry.addData(primaryName + " profiled", "%.0f", profiledVelocity);
        telemetry.addData(primaryName + " velocity (raw)", "%.0f", rawVelocity);
        telemetry.addData(primaryName + " velocity (smooth)", "%.1f", smoothVelocity);
        telemetry.addData(primaryName + " power", "%.2f", lastPower);
        telemetry.addData(primaryName + " isReady", "%b", isReady());
    }

}
