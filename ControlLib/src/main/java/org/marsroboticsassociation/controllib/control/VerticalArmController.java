package org.marsroboticsassociation.controllib.control;

import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.motion.PositionTrajectoryManager;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import java.util.function.LongSupplier;

/**
 * Vertical arm position controller using feedback linearization + LQR + Kalman filter.
 *
 * <p>The design has three cleanly separated layers:
 * <ol>
 *   <li><b>Feedback linearization</b> &mdash; cancels gravity ({@code kg*cos(theta)}) and
 *       static friction ({@code ks*sign(omega)}) at the predicted arm angle, reducing the
 *       nonlinear arm to a linear motor.</li>
 *   <li><b>LQR + linear feedforward</b> &mdash; operates on the linearized plant.
 *       Feedforward provides {@code kv*vel + ka*accel}; LQR closes the loop on tracking error.
 *       LQR gains are derived from the plant model and Q/R cost weights, so they auto-adjust
 *       when motor parameters change.</li>
 *   <li><b>Kalman filter</b> &mdash; linear state observer on the gravity-free plant, with
 *       forward prediction for latency compensation.</li>
 * </ol>
 *
 * <p>The Kalman filter input is always the <em>linearized</em> voltage (total voltage minus the
 * gravity/friction cancellation), keeping the linear model consistent with the actual dynamics.
 *
 * <p>Voltage normalization: output voltages are divided by the live hub voltage to produce motor
 * power in [-1, 1], compensating for battery sag. Near hard stops, the controller coasts (cuts
 * power) when both the target and position are in the coast zone.
 *
 * <p>Usage:
 * <pre>
 *   VerticalArmController arm = new VerticalArmController(motor, telemetry::addData);
 *   arm.setTarget(Math.toRadians(45));
 *   // each loop iteration:
 *   arm.update(dt);
 * </pre>
 */
public class VerticalArmController {

    /**
     * Tuning parameters. Shared between instances.
     *
     * <p>All angles are in radians measured from horizontal (positive = above horizontal).
     * All velocities/accelerations are in rad/s and rad/s^2 at the output shaft.
     */
    public static class Params {
        // --- Motor / Encoder ---
        public int ticksPerRev = 28;
        public double gearRatio = 100.0;

        // --- Geometry ---
        /** Angle from horizontal when the encoder reads 0 (front hard stop). */
        public double encoderZeroOffsetRad = -Math.PI / 4;
        /** Back hard stop angle (radians from horizontal). */
        public double minAngleRad = -Math.PI * 5 / 4;
        /** Front hard stop angle (radians from horizontal). */
        public double maxAngleRad = -Math.PI / 4;

        // --- Feedforward (output-shaft units) ---
        public double ks = 0.0;    // static friction voltage
        public double kg = 0.0;    // gravity voltage at horizontal
        public double kv = 0.0;    // V/(rad/s)
        public double ka = 0.0;    // V/(rad/s^2)

        // --- Kalman filter noise model ---
        public double modelStdDevPos = 0.01;
        public double modelStdDevVel = 3.0;
        public double measurementStdDevPos = 0.05;
        public double measurementStdDevVel = 0.5;

        // --- LQR cost weights (replace PD gains) ---
        /** Position tolerance for LQR (rad). Smaller = tighter tracking. */
        public double qPosition = 0.5;
        /** Velocity tolerance for LQR (rad/s). Smaller = more aggressive damping. */
        public double qVelocity = 5.0;
        /** Control effort penalty (V). Larger = gentler output. */
        public double rVoltage = 12.0;

        // --- Trajectory (SCurve) ---
        public double maxVelRad = 2.0;
        public double maxAccelRad = 4.0;
        public double maxDecelRad = 6.0;
        public double maxJerkRad = 20.0;

        // --- Tracking / replanning ---
        public double replanThresholdRad = Math.toRadians(15);

        // --- Hard stop behavior ---
        public double coastZoneRad = Math.toRadians(10);

        // --- Latency compensation ---
        public double latencyCompensationSec = 0.030;

        // --- Nominal dt ---
        public double dtSeconds = 0.020;

        // --- Convergence ---
        public double atTargetPositionTolerance = Math.toRadians(2);
        public double atTargetVelocityTolerance = Math.toRadians(5);
    }

    public static Params PARAMS = new Params();

    public enum Mode { TRACKING, COASTING }

    private final IMotor motor;
    private final String name;
    private final TelemetryAddData telemetry;

    // Components
    private final LinearSystem<N2, N1, N2> plant;
    private final KalmanFilter<N2, N1, N2> kalman;
    private final LinearQuadraticRegulator<N2, N1, N2> lqr;
    private final ArmFeedforward armFf;  // used only for computeMoveLimits
    private final PositionTrajectoryManager trajectory;

    // State
    private Mode mode = Mode.COASTING;
    private double targetAngleRad;
    private double lastVoltageCmded = 0;
    private double lastLinearVoltage = 0;
    private double lastPower = 0;

    // Cached predicted state (after latency compensation)
    private double predictedPosRad = 0;
    private double predictedVelRad = 0;

    /**
     * @param motor     IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically {@code telemetry::addData}
     */
    public VerticalArmController(IMotor motor, TelemetryAddData telemetry) {
        this(motor, telemetry, System::nanoTime);
    }

    /**
     * Test constructor with injectable clock.
     *
     * @param motor     IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically {@code telemetry::addData}
     * @param clock     clock supplier (nanoseconds) for trajectory timing
     */
    public VerticalArmController(IMotor motor, TelemetryAddData telemetry, LongSupplier clock) {
        this.motor = motor;
        this.name = motor.getName();
        this.telemetry = telemetry;

        // Linear plant (motor dynamics only, no gravity)
        plant = LinearSystemId.identifyPositionSystem(PARAMS.kv, PARAMS.ka);

        // Kalman filter on the linear plant
        kalman = new KalmanFilter<>(
                Nat.N2(), Nat.N2(),
                plant,
                VecBuilder.fill(PARAMS.modelStdDevPos, PARAMS.modelStdDevVel),
                VecBuilder.fill(PARAMS.measurementStdDevPos, PARAMS.measurementStdDevVel),
                PARAMS.dtSeconds);

        // LQR on the linear plant (replaces hand-tuned PD)
        lqr = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(PARAMS.qPosition, PARAMS.qVelocity),
                VecBuilder.fill(PARAMS.rVoltage),
                PARAMS.dtSeconds);

        // ArmFeedforward used only for maxAchievableAcceleration/Velocity in computeMoveLimits
        armFf = new ArmFeedforward(PARAMS.ks, PARAMS.kg, PARAMS.kv, PARAMS.ka, PARAMS.dtSeconds);

        // Trajectory manager
        trajectory = new PositionTrajectoryManager(
                PARAMS.maxVelRad, PARAMS.maxAccelRad, PARAMS.maxDecelRad, PARAMS.maxJerkRad,
                Math.toRadians(0.5), telemetry, clock);

        // Seed observer and trajectory to current position
        double posRad = ticksToRad(motor.getPosition());
        double velRad = tpsToRadPerSec(motor.getVelocity());
        kalman.setXhat(VecBuilder.fill(posRad, velRad));
        targetAngleRad = posRad;
        trajectory.resetFromMeasurement(posRad, velRad);
        predictedPosRad = posRad;
        predictedVelRad = velRad;
    }

    // ---------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------

    /**
     * Set the target arm angle in radians (measured from horizontal).
     *
     * <p>Computes per-move trajectory limits based on worst-case gravity torque
     * across the sweep, then plans the trajectory with those limits.
     */
    public void setTarget(double angleRad) {
        setTarget(angleRad, motor.getHubVoltage());
    }

    /**
     * Set the target arm angle with a pre-sampled hub voltage.
     *
     * @param angleRad   target angle in radians from horizontal
     * @param hubVoltage current battery voltage in volts
     */
    public void setTarget(double angleRad, double hubVoltage) {
        targetAngleRad = MathUtil.clamp(angleRad, PARAMS.minAngleRad, PARAMS.maxAngleRad);

        double fromRad;
        if (mode == Mode.COASTING) {
            double hardStopAngle = nearestHardStop(predictedPosRad);
            kalman.setXhat(VecBuilder.fill(hardStopAngle, 0));
            trajectory.resetFromMeasurement(hardStopAngle, 0);
            predictedPosRad = hardStopAngle;
            predictedVelRad = 0;
            mode = Mode.TRACKING;
            fromRad = hardStopAngle;
        } else {
            fromRad = predictedPosRad;
        }

        double[] limits = computeMoveLimits(fromRad, targetAngleRad, hubVoltage);
        trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
        trajectory.setTarget(targetAngleRad);
    }

    /**
     * Run one control cycle, reading battery voltage from the motor.
     *
     * @param dt elapsed time in seconds since the last update
     */
    public void update(double dt) {
        update(dt, motor.getHubVoltage());
    }

    /**
     * Run one control cycle with a pre-sampled battery voltage.
     *
     * @param dt         elapsed time in seconds since the last update
     * @param hubVoltage battery voltage in volts
     */
    public void update(double dt, double hubVoltage) {
        if (dt < 1e-6) return;

        // 1. Read sensors
        double measuredPosRad = ticksToRad(motor.getPosition());
        double measuredVelRad = tpsToRadPerSec(motor.getVelocity());

        // 2. Kalman correct + predict with linearized voltage
        Matrix<N1, N1> uLinear = VecBuilder.fill(lastLinearVoltage);
        kalman.correct(uLinear, VecBuilder.fill(measuredPosRad, measuredVelRad));
        kalman.predict(uLinear, dt);

        // 3. Forward predict for latency compensation
        Matrix<N2, N1> predicted = plant.calculateX(
                kalman.getXhat(), uLinear, PARAMS.latencyCompensationSec);
        predictedPosRad = predicted.get(0, 0);
        predictedVelRad = predicted.get(1, 0);

        // 4. Coast check
        if (shouldCoast(predictedPosRad)) {
            mode = Mode.COASTING;
            motor.setPower(0);
            lastVoltageCmded = 0;
            lastLinearVoltage = 0;
            lastPower = 0;
            return;
        }

        mode = Mode.TRACKING;

        // 5. Trajectory update + replan check
        trajectory.update();
        double trajPos = trajectory.getPosition();
        double trajVel = trajectory.getVelocity();
        double trajAccel = trajectory.getAcceleration();

        if (Math.abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
            double[] limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage);
            trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
            trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad);
            trajectory.update();
            trajPos = trajectory.getPosition();
            trajVel = trajectory.getVelocity();
            trajAccel = trajectory.getAcceleration();
        }

        // 6. Layer 1: Feedback linearization — cancel gravity + friction at predicted angle
        double uCancel = PARAMS.kg * Math.cos(predictedPosRad)
                        + PARAMS.ks * Math.signum(predictedVelRad);

        // 7. Layer 2a: Linear feedforward (kv*vel + ka*accel on the linearized plant)
        double uFfLinear = PARAMS.kv * trajVel + PARAMS.ka * trajAccel;

        // 8. Layer 2b: LQR feedback on linearized tracking error
        Matrix<N2, N1> reference = VecBuilder.fill(trajPos, trajVel);
        Matrix<N2, N1> state = VecBuilder.fill(predictedPosRad, predictedVelRad);
        double uLqr = lqr.calculate(state, reference).get(0, 0);

        // 9. Total voltage
        double totalVoltage = uCancel + uFfLinear + uLqr;
        lastVoltageCmded = totalVoltage;
        lastPower = MathUtil.clamp(totalVoltage / hubVoltage, -1.0, 1.0);
        motor.setPower(lastPower);

        // 10. Compute linearized voltage for next Kalman cycle
        //     Use the clamped actual voltage minus the cancellation, so the linear model
        //     sees the true linear-equivalent input even under motor saturation.
        double actualVoltage = lastPower * hubVoltage;
        lastLinearVoltage = actualVoltage - uCancel;
    }

    public double getEstimatedPositionRad() {
        return predictedPosRad;
    }

    public double getEstimatedVelocityRadPerSec() {
        return predictedVelRad;
    }

    public double getTrajectoryPositionRad() {
        return trajectory.getPosition();
    }

    public double getTrajectoryVelocityRadPerSec() {
        return trajectory.getVelocity();
    }

    public Mode getMode() {
        return mode;
    }

    public boolean isAtTarget() {
        return Math.abs(predictedPosRad - targetAngleRad) < PARAMS.atTargetPositionTolerance
                && Math.abs(predictedVelRad) < PARAMS.atTargetVelocityTolerance;
    }

    public void reset() {
        double posRad = ticksToRad(motor.getPosition());
        double velRad = tpsToRadPerSec(motor.getVelocity());
        kalman.setXhat(VecBuilder.fill(posRad, velRad));
        predictedPosRad = posRad;
        predictedVelRad = velRad;
        lastVoltageCmded = 0;
        lastLinearVoltage = 0;
    }

    public void writeTelemetry() {
        telemetry.addData(name + " arm mode",          "%s", mode.name());
        telemetry.addData(name + " arm target deg",    "%.1f", Math.toDegrees(targetAngleRad));
        telemetry.addData(name + " arm predicted deg", "%.1f", Math.toDegrees(predictedPosRad));
        telemetry.addData(name + " arm predicted v",   "%.1f", Math.toDegrees(predictedVelRad));
        telemetry.addData(name + " arm traj pos deg",  "%.1f", Math.toDegrees(trajectory.getPosition()));
        telemetry.addData(name + " arm traj vel",      "%.1f", Math.toDegrees(trajectory.getVelocity()));
        telemetry.addData(name + " arm voltage cmd",   "%.2f V", lastVoltageCmded);
        telemetry.addData(name + " arm power",         "%.3f", lastPower);
    }

    // ---------------------------------------------------------------------------
    // Unit conversions
    // ---------------------------------------------------------------------------

    private double ticksToRad(int ticks) {
        return ticks * 2.0 * Math.PI / (PARAMS.ticksPerRev * PARAMS.gearRatio)
                + PARAMS.encoderZeroOffsetRad;
    }

    private double tpsToRadPerSec(double tps) {
        return tps * 2.0 * Math.PI / (PARAMS.ticksPerRev * PARAMS.gearRatio);
    }

    // ---------------------------------------------------------------------------
    // Hard stop helpers
    // ---------------------------------------------------------------------------

    private boolean shouldCoast(double posRad) {
        boolean posNearMin = posRad <= PARAMS.minAngleRad + PARAMS.coastZoneRad;
        boolean posNearMax = posRad >= PARAMS.maxAngleRad - PARAMS.coastZoneRad;
        boolean targetNearMin = targetAngleRad <= PARAMS.minAngleRad + PARAMS.coastZoneRad;
        boolean targetNearMax = targetAngleRad >= PARAMS.maxAngleRad - PARAMS.coastZoneRad;
        return (posNearMin && targetNearMin) || (posNearMax && targetNearMax);
    }

    private double nearestHardStop(double posRad) {
        double distToMin = Math.abs(posRad - PARAMS.minAngleRad);
        double distToMax = Math.abs(posRad - PARAMS.maxAngleRad);
        return distToMin < distToMax ? PARAMS.minAngleRad : PARAMS.maxAngleRad;
    }

    // ---------------------------------------------------------------------------
    // Per-move trajectory limits
    // ---------------------------------------------------------------------------

    /**
     * Compute per-move trajectory limits based on the worst-case gravity torque in the sweep.
     * Returns [maxVel, maxAccel, maxDecel].
     */
    double[] computeMoveLimits(double fromRad, double toRad, double hubVoltage) {
        double midRad = (fromRad + toRad) / 2.0;

        double worstAccelAngle = worstCaseAngle(fromRad, midRad);
        double worstDecelAngle = worstCaseAngle(midRad, toRad);
        double worstVelAngle = worstCaseAngle(fromRad, toRad);

        double accel = armFf.maxAchievableAcceleration(
                hubVoltage, worstAccelAngle, PARAMS.maxVelRad);
        double decel = armFf.maxAchievableAcceleration(
                hubVoltage, worstDecelAngle, PARAMS.maxVelRad);
        double vel = armFf.maxAchievableVelocity(
                hubVoltage, worstVelAngle, 0);

        vel = Math.min(Math.max(vel, 0), PARAMS.maxVelRad);
        accel = Math.min(Math.max(accel, 0), PARAMS.maxAccelRad);
        decel = Math.min(Math.max(decel, 0), PARAMS.maxDecelRad);

        return new double[] { vel, accel, decel };
    }

    /**
     * Find the angle in the range between a and b where |cos(theta)| is maximized
     * (worst-case gravity torque).
     */
    static double worstCaseAngle(double a, double b) {
        double lo = Math.min(a, b);
        double hi = Math.max(a, b);

        double bestAngle = a;
        double bestAbsCos = Math.abs(Math.cos(a));

        double absCosB = Math.abs(Math.cos(b));
        if (absCosB > bestAbsCos) {
            bestAngle = b;
            bestAbsCos = absCosB;
        }

        if (lo <= 0 && hi >= 0) {
            bestAngle = 0;
            bestAbsCos = 1.0;
        }
        if (lo <= -Math.PI && hi >= -Math.PI) {
            if (1.0 > bestAbsCos) {
                bestAngle = -Math.PI;
            }
        }

        return bestAngle;
    }
}
