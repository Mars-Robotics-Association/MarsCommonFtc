package org.marsroboticsassociation.controllib.control;

import org.marsroboticsassociation.controllib.filter.BiquadLowPassVarDt;
import org.marsroboticsassociation.controllib.filter.LowPassFilter;
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * Flywheel velocity controller using a state-space LQR + Kalman filter observer.
 *
 * <p>Uses {@code LinearSystemId.identifyVelocitySystem(kV, kA)} to build the plant from
 * characterization constants, avoiding the need for FTC motor specs that are absent from
 * {@code DCMotor.java}. {@link Params#kV} and {@link Params#kA} are stored in WPILib SI units
 * (V·s/rad and V·s²/rad); see {@link Params} for the conversion from characterization data.
 *
 * <p>The Kalman filter runs at a nominal 20 ms dt for gain computation but accepts the real
 * measured dt in {@link #update(double)} for accurate state propagation. FTC's non-deterministic
 * loop timing (typically 10–30 ms) is handled automatically.
 *
 * <p>Voltage normalization: the loop outputs a voltage command which is divided by the live hub
 * voltage to produce a motor power in [−1, 1], compensating for battery sag. Battery voltage is
 * sampled each control loop; pass a pre-sampled value via {@link #update(double, double)} if
 * voltage was already read elsewhere in the OpMode.
 *
 * <p>Usage:
 * <pre>
 *   FlywheelStateSpace flywheel = new FlywheelStateSpace(motor, telemetry::addData, "shooter");
 *   flywheel.setTps(2000);
 *   // each loop iteration:
 *   flywheel.update(dt);
 * </pre>
 */
public class FlywheelStateSpace {

    /**
     * Tuning parameters. Shared between instances.
     *
     * <p>Plant constants ({@link #kV}, {@link #kA}) are in WPILib SI units. To convert from
     * characterization data (voltage, max TPS, ticksPerRev):
     * <pre>
     *   kV = measuredVoltage * ticksPerRev / (maxTps * 2π)   // V·s/rad
     *   kA = measuredVoltage * ticksPerRev / (maxAccelTps * 2π) // V·s²/rad
     * </pre>
     *
     * <p>Kalman std-dev values trade off responsiveness vs. noise rejection:
     * higher modelStdDevRadPerSec trusts measurements more; lower
     * measurementStdDevRadPerSec trusts the encoder more.
     *
     * <p>LQR values: smaller qVelocityRadPerSec → more aggressive velocity tracking;
     * larger rVoltage → penalizes voltage use more heavily.
     */
    public static class Params {
        // Plant — WPILib SI units (V·s/rad, V·s²/rad)
        // Defaults derived from: 12.5 V characterization run, 28 ticks/rev
        public double kV = 12.5 * 28 / (2632.1 * 2 * Math.PI);  // V·s/rad
        public double kA = 12.5 * 28 / (2087.9 * 2 * Math.PI);  // V·s²/rad
        public int ticksPerRev = 28;            // encoder ticks per revolution at output shaft

        // Kalman observer noise model
        public double modelStdDevRadPerSec = 3.0;        // process noise (higher → trust sensor more)
        public double measurementStdDevRadPerSec = 0.01; // measurement noise (lower → trust encoder more)

        // LQR cost weights
        public double qVelocityRadPerSec = 8.0; // velocity tolerance in rad/s (tighter → more aggressive)
        public double rVoltage = 12.0;          // voltage penalty

        public double dtSeconds = 0.020;        // nominal loop period for Kalman gain computation

        /** Estimated velocity must be within this many TPS of the setpoint for {@link #isReady()}. */
        public double readyThresholdTps = 30.0;
        /** Estimated acceleration must be below this (TPS/s) for {@link #isReady()}. */
        public double readyAccelToleranceTps2 = 50.0;
        /** Biquad low-pass cutoff frequency for smoothing the acceleration estimate (Hz). */
        public double accelLpfCutoffHz = 2.0;
    }

    public static Params PARAMS = new Params();

    private final IMotor motor;
    private final String primaryName;   // used as telemetry prefix
    private final TelemetryAddData telemetry;

    private final LinearSystemLoop<N1, N1, N1> loop;

    private double targetTps = 0;
    private double lastVoltageCmded = 0;
    private double lastPower = 0;
    private double prevEstimatedTps = 0;
    private final LowPassFilter accelLpf;

    /**
     * @param motor       IMotor providing velocity, power, and hub voltage
     * @param telemetry   typically {@code telemetry::addData}
     * @param primaryName name used as telemetry prefix
     */
    public FlywheelStateSpace(IMotor motor, TelemetryAddData telemetry, String primaryName) {
        this.motor       = motor;
        this.primaryName = primaryName;
        this.telemetry   = telemetry;

        double startupVoltage = motor.getHubVoltage();
        loop = buildLoop(PARAMS, startupVoltage);
        accelLpf = new BiquadLowPassVarDt(PARAMS.accelLpfCutoffHz, 0.5);
        // Seed observer to current velocity so the first correction isn't a large jump
        double initialRadPerSec = motor.getVelocity() * 2 * Math.PI / PARAMS.ticksPerRev;
        loop.reset(VecBuilder.fill(initialRadPerSec));
    }

    // ---------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------

    /**
     * Set the target flywheel velocity in ticks per second.
     * Pass 0 to coast the motor.
     */
    public void setTps(double tps) {
        targetTps = tps;
    }

    /**
     * Run one control cycle, reading battery voltage from the motor adapter.
     *
     * <p>Must be called once per loop iteration. Pass the elapsed time since the last call
     * so the Kalman predictor uses the real dt rather than the nominal value.
     *
     * <p>If battery voltage was already sampled elsewhere in the OpMode this loop,
     * use {@link #update(double, double)} to avoid a redundant hub read.
     *
     * @param dt elapsed time in seconds since the last {@code update()} call
     */
    public void update(double dt) {
        update(dt, motor.getHubVoltage());
    }

    /**
     * Run one control cycle with a pre-sampled battery voltage.
     *
     * <p>Use this overload when battery voltage has already been read elsewhere in the OpMode
     * to avoid a redundant I²C read.
     *
     * @param dt         elapsed time in seconds since the last {@code update()} call
     * @param hubVoltage battery voltage in volts, sampled this loop iteration
     */
    public void update(double dt, double hubVoltage) {
        if (dt < 1e-6) return;  // likely a duplicate call in the same frame

        double twoPI = 2.0 * Math.PI;
        double measuredRadPerSec = motor.getVelocity() * twoPI / PARAMS.ticksPerRev;
        double targetRadPerSec   = targetTps           * twoPI / PARAMS.ticksPerRev;

        loop.setNextR(VecBuilder.fill(targetRadPerSec));
        loop.correct(VecBuilder.fill(measuredRadPerSec));
        loop.predict(dt);   // uses real dt for accurate propagation

        lastVoltageCmded = loop.getU(0); // already clamped to ±startupVoltage by the loop
        double voltage = MathUtil.clamp(lastVoltageCmded, -hubVoltage, hubVoltage);
        lastPower = MathUtil.clamp(voltage / hubVoltage, -1.0, 1.0);

        motor.setPower(targetTps == 0 ? 0 : lastPower);

        double currentEstimatedTps = getEstimatedTps();
        double rawAccel = (currentEstimatedTps - prevEstimatedTps) / dt;
        accelLpf.update(rawAccel, dt);
        prevEstimatedTps = currentEstimatedTps;
    }

    /**
     * Returns the Kalman-estimated flywheel velocity in ticks per second.
     * This is smoother than the raw encoder reading and typically more accurate
     * than a fixed-cutoff low-pass filter.
     */
    public double getEstimatedTps() {
        return loop.getXHat(0) * PARAMS.ticksPerRev / (2.0 * Math.PI);
    }

    /**
     * Returns true when the flywheel is spinning, the Kalman-estimated velocity
     * is within {@link Params#readyThresholdTps} of the setpoint, and the estimated
     * acceleration is below {@link Params#readyAccelToleranceTps2} (indicating the
     * velocity has settled rather than just passing through the threshold).
     * Returns false when the setpoint is zero.
     */
    public boolean isReady() {
        return targetTps != 0
                && Math.abs(getEstimatedTps() - targetTps) < PARAMS.readyThresholdTps
                && Math.abs(accelLpf.getValue()) < PARAMS.readyAccelToleranceTps2;
    }

    /** Returns the low pass filtered acceleration in TPS/s, derived from consecutive Kalman estimates. */
    public double getEstimatedAccelTps2() {
        return accelLpf.getValue();
    }

    /**
     * Reset the observer to the current measured velocity.
     * Call this when restarting after a long idle period to avoid a transient
     * where the observer state is far from reality.
     */
    public void reset() {
        double currentRadPerSec = motor.getVelocity() * 2.0 * Math.PI / PARAMS.ticksPerRev;
        loop.reset(VecBuilder.fill(currentRadPerSec));
    }

    /** Add state-space flywheel telemetry to the driver station. */
    public void writeTelemetry() {
        telemetry.addData(primaryName + " ss target TPS",    "%.0f",  targetTps);
        telemetry.addData(primaryName + " ss estimated TPS", "%.1f",  getEstimatedTps());
        telemetry.addData(primaryName + " ss measured TPS",  "%.1f",  motor.getVelocity());
        telemetry.addData(primaryName + " ss voltage cmd",   "%.2f V", lastVoltageCmded);
        telemetry.addData(primaryName + " ss power",         "%.3f",  lastPower);
    }

    // ---------------------------------------------------------------------------
    // Loop construction
    // ---------------------------------------------------------------------------

    private static LinearSystemLoop<N1, N1, N1> buildLoop(Params p, double maxVoltage) {
        LinearSystem<N1, N1, N1> plant =
                LinearSystemId.identifyVelocitySystem(p.kV, p.kA);

        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                plant,
                VecBuilder.fill(p.modelStdDevRadPerSec),
                VecBuilder.fill(p.measurementStdDevRadPerSec),
                p.dtSeconds);

        LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(p.qVelocityRadPerSec),
                VecBuilder.fill(p.rVoltage),
                p.dtSeconds);

        return new LinearSystemLoop<>(plant, controller, observer, maxVoltage, p.dtSeconds);
    }
}
