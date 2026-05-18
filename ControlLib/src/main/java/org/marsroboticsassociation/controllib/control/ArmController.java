package org.marsroboticsassociation.controllib.control;

import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.motion.PositionTrajectoryManager;
import org.marsroboticsassociation.controllib.util.TelemetryAddData;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

import java.util.function.LongSupplier;

/**
 * Arm position controller using S-curve trajectory + ArmFeedforward + Kalman filter + PD feedback.
 *
 * <p>The Kalman filter runs on a linear motor model (no gravity) and predicts the arm state
 * forward in time to compensate for sensor-to-actuator latency. Gravity compensation is handled
 * entirely by {@link ArmFeedforward}, which is applied additively as a voltage offset.
 *
 * <p>Voltage normalization: feedforward and PD output voltages which are divided by the live hub
 * voltage to produce motor power in [-1, 1], compensating for battery sag.
 *
 * <p>Near hard stops, the controller coasts (cuts power) to let the arm rest against the stop.
 *
 * <p>Usage:
 * <pre>
 *   ArmController arm = new ArmController(motor, telemetry::addData);
 *   arm.setTarget(Math.toRadians(45));
 *   // each loop iteration:
 *   arm.update(dt);
 * </pre>
 */
public class ArmController {

    /**
     * Tuning parameters. Shared between instances.
     *
     * <p>All angles are in radians measured from horizontal (positive = above horizontal).
     * All velocities/accelerations are in rad/s and rad/s^2 at the output shaft.
     */
    public static class Params {
        // --- Motor / Encoder ---
        public int ticksPerRev = 28;       // motor encoder ticks per motor revolution
        public double gearRatio = 100.0;   // output:input (e.g. 100:1)

        // --- Geometry ---
        /** Angle from horizontal when the encoder reads 0 (front hard stop). Found by sysid. */
        public double encoderZeroOffsetRad = -Math.PI / 4;  // default: 45 deg below horizontal

        /** Back hard stop angle (radians from horizontal). */
        public double minAngleRad = -Math.PI * 5 / 4;  // -225 deg = 45 deg below horizontal behind
        /** Front hard stop angle (radians from horizontal). Encoder reads 0 here. */
        public double maxAngleRad = -Math.PI / 4;       // -45 deg = 45 deg below horizontal in front

        // --- Feedforward (ArmFeedforward gains, output-shaft units) ---
        public double ks = 0.0;    // static friction voltage
        public double kg = 0.0;    // gravity voltage at horizontal
        public double kv = 0.0;    // V/(rad/s)
        public double ka = 0.0;    // V/(rad/s^2)

        // --- Kalman filter noise model ---
        public double modelStdDevPos = 0.01;       // position process noise (rad)
        public double modelStdDevVel = 3.0;        // velocity process noise (rad/s)
        public double measurementStdDevPos = 0.05;  // position measurement noise (rad)
        public double measurementStdDevVel = 0.5;   // velocity measurement noise (rad/s)

        // --- PD feedback ---
        public double kP = 0.0;    // volts per radian of position error
        public double kD = 0.0;    // volts per (rad/s) of velocity error

        // --- Trajectory (SCurve) ---
        public double maxVelRad = 2.0;         // rad/s
        public double maxAccelRad = 4.0;       // rad/s^2 (acceleration limit)
        public double maxDecelRad = 6.0;       // rad/s^2 (deceleration limit, asymmetric)
        public double maxJerkRad = 20.0;       // rad/s^3

        // --- Tracking / replanning ---
        /** Replan trajectory if position tracking error exceeds this (rad). */
        public double replanThresholdRad = Math.toRadians(15);

        // --- Hard stop behavior ---
        /** Coast (cut power) when within this angle of a hard stop (rad). */
        public double coastZoneRad = Math.toRadians(10);

        // --- Latency compensation ---
        /** Forward-predict state by this many seconds to compensate sensor-to-actuator latency. */
        public double latencyCompensationSec = 0.030;

        // --- Nominal dt ---
        public double dtSeconds = 0.020;  // nominal loop period for Kalman gain computation

        /** Position tolerance for {@link #isAtTarget()} (rad). */
        public double atTargetPositionTolerance = Math.toRadians(2);
        /** Velocity tolerance for {@link #isAtTarget()} (rad/s). */
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
    private final ArmFeedforward feedforward;
    private final PositionTrajectoryManager trajectory;

    // State
    private Mode mode = Mode.COASTING;
    private double targetAngleRad;
    private double lastVoltageCmded = 0;
    /** Voltage with gravity+friction subtracted, consistent with the linear plant model. */
    private double lastLinearVoltage = 0;
    private double lastPower = 0;

    // Cached predicted state (after latency compensation)
    private double predictedPosRad = 0;
    private double predictedVelRad = 0;

    /**
     * @param motor     IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically {@code telemetry::addData}
     */
    public ArmController(IMotor motor, TelemetryAddData telemetry) {
        this(motor, telemetry, System::nanoTime);
    }

    /**
     * Test constructor with injectable clock for the trajectory manager.
     *
     * @param motor     IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically {@code telemetry::addData}
     * @param clock     clock supplier (nanoseconds) for trajectory timing
     */
    public ArmController(IMotor motor, TelemetryAddData telemetry, LongSupplier clock) {
        this.motor = motor;
        this.name = motor.getName();
        this.telemetry = telemetry;

        // Build the linear plant (motor dynamics only, no gravity)
        // State: [position (rad), velocity (rad/s)], Input: [voltage], Output: [position, velocity]
        plant = LinearSystemId.identifyPositionSystem(PARAMS.kv, PARAMS.ka);

        // Build Kalman filter
        kalman = new KalmanFilter<>(
                Nat.N2(), Nat.N2(),
                plant,
                VecBuilder.fill(PARAMS.modelStdDevPos, PARAMS.modelStdDevVel),
                VecBuilder.fill(PARAMS.measurementStdDevPos, PARAMS.measurementStdDevVel),
                PARAMS.dtSeconds);

        // Build feedforward (nominal dt; variable dt handled via deprecated calculate overload)
        feedforward = new ArmFeedforward(PARAMS.ks, PARAMS.kg, PARAMS.kv, PARAMS.ka, PARAMS.dtSeconds);

        // Build trajectory manager with injectable clock
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
     * <p>Computes per-move trajectory limits based on the worst-case gravity torque
     * across the sweep, then plans the trajectory with those limits.
     *
     * <p>If the arm is currently coasting at a hard stop, this wakes it up:
     * the state is reset to the hard stop angle with zero velocity, and a
     * new trajectory is planned from there.
     */
    public void setTarget(double angleRad) {
        setTarget(angleRad, motor.getHubVoltage());
    }

    /**
     * Set the target arm angle with a pre-sampled hub voltage (for testing).
     *
     * @param angleRad   target angle in radians from horizontal
     * @param hubVoltage current battery voltage in volts
     */
    public void setTarget(double angleRad, double hubVoltage) {
        targetAngleRad = MathUtil.clamp(angleRad, PARAMS.minAngleRad, PARAMS.maxAngleRad);

        double fromRad;
        if (mode == Mode.COASTING) {
            // Wake from hard stop: assume position is at the nearest hard stop
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

        // Compute gravity-aware trajectory limits for this move
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
    @SuppressWarnings("deprecation")
    public void update(double dt, double hubVoltage) {
        if (dt < 1e-6) return;

        // 1. Read sensors
        double measuredPosRad = ticksToRad(motor.getPosition());
        double measuredVelRad = tpsToRadPerSec(motor.getVelocity());

        // 2. Kalman correct — use the linear-model-consistent voltage (gravity/friction removed)
        //    so the Kalman filter's plant model prediction matches the actual dynamics.
        Matrix<N1, N1> uLinear = VecBuilder.fill(lastLinearVoltage);
        kalman.correct(uLinear, VecBuilder.fill(measuredPosRad, measuredVelRad));

        // 3. Kalman predict
        kalman.predict(uLinear, dt);

        // 4. Forward predict (latency compensation)
        Matrix<N2, N1> predicted = plant.calculateX(
                kalman.getXhat(), uLinear, PARAMS.latencyCompensationSec);
        predictedPosRad = predicted.get(0, 0);
        predictedVelRad = predicted.get(1, 0);

        // 5. Coast check
        if (shouldCoast(predictedPosRad)) {
            mode = Mode.COASTING;
            motor.setPower(0);
            lastVoltageCmded = 0;
            lastLinearVoltage = 0;
            lastPower = 0;
            return;
        }

        mode = Mode.TRACKING;

        // 6. Trajectory update
        trajectory.update();
        double trajPos = trajectory.getPosition();
        double trajVel = trajectory.getVelocity();
        double trajAccel = trajectory.getAcceleration();

        // 7. Replan check
        if (Math.abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
            // Recompute limits for the remaining sweep
            double[] limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage);
            trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);

            trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad);
            // Re-read trajectory state after replan
            trajectory.update();
            trajPos = trajectory.getPosition();
            trajVel = trajectory.getVelocity();
            trajAccel = trajectory.getAcceleration();
        }

        // 8. Feedforward (use deprecated overload for variable dt with RK4 accuracy)
        //    Use predicted position (not trajectory position) for gravity compensation —
        //    gravity acts at the arm's actual angle, not where the trajectory expects it.
        double nextVel = trajVel + trajAccel * dt;
        double ffVoltage = feedforward.calculate(predictedPosRad, trajVel, nextVel, dt);

        // 9. PD feedback
        double posError = trajPos - predictedPosRad;
        double velError = trajVel - predictedVelRad;
        double pdVoltage = PARAMS.kP * posError + PARAMS.kD * velError;

        // 10. Output
        double totalVoltage = ffVoltage + pdVoltage;
        lastVoltageCmded = totalVoltage;
        lastPower = MathUtil.clamp(totalVoltage / hubVoltage, -1.0, 1.0);
        motor.setPower(lastPower);
        // For the Kalman filter: use the clamped (actually applied) voltage, then subtract
        // gravity and friction so the linear model sees only the motor dynamics portion.
        // This prevents biased predictions when gravity compensation is a large fraction
        // of the total voltage, and avoids feeding unrealistically large voltages to the
        // observer when the motor saturates.
        double actualVoltage = lastPower * hubVoltage;
        double gravityVoltage = PARAMS.kg * Math.cos(predictedPosRad);
        double frictionVoltage = PARAMS.ks * Math.signum(predictedVelRad);
        lastLinearVoltage = actualVoltage - gravityVoltage - frictionVoltage;
    }

    /** Get the predicted position in radians (after latency compensation). */
    public double getEstimatedPositionRad() {
        return predictedPosRad;
    }

    /** Get the predicted velocity in rad/s (after latency compensation). */
    public double getEstimatedVelocityRadPerSec() {
        return predictedVelRad;
    }

    /** Get the current trajectory setpoint position in radians. */
    public double getTrajectoryPositionRad() {
        return trajectory.getPosition();
    }

    /** Get the current trajectory setpoint velocity in rad/s. */
    public double getTrajectoryVelocityRadPerSec() {
        return trajectory.getVelocity();
    }

    /** Get the current controller mode. */
    public Mode getMode() {
        return mode;
    }

    /**
     * Returns true when the predicted position is near the target and velocity is near zero.
     */
    public boolean isAtTarget() {
        return Math.abs(predictedPosRad - targetAngleRad) < PARAMS.atTargetPositionTolerance
                && Math.abs(predictedVelRad) < PARAMS.atTargetVelocityTolerance;
    }

    /**
     * Reset the observer to the current measured state. Call after long idle or mode transitions.
     */
    public void reset() {
        double posRad = ticksToRad(motor.getPosition());
        double velRad = tpsToRadPerSec(motor.getVelocity());
        kalman.setXhat(VecBuilder.fill(posRad, velRad));
        predictedPosRad = posRad;
        predictedVelRad = velRad;
        lastVoltageCmded = 0;
        lastLinearVoltage = 0;
    }

    /** Add arm controller telemetry to the driver station. */
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

    /**
     * Coast when both the target and the current position are near the same hard stop.
     * This avoids coasting when the arm is merely passing through the stop zone on its way
     * to a target further away.
     */
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

    /**
     * Compute per-move trajectory limits based on the worst-case gravity torque in the sweep.
     * Returns [maxVel, maxAccel, maxDecel].
     *
     * <p>The sweep is split at the midpoint: the first half constrains acceleration,
     * the second half constrains deceleration. Max velocity is constrained across the full
     * sweep. All results are capped at the PARAMS global maximums.
     *
     * @param fromRad    start angle (radians from horizontal)
     * @param toRad      target angle (radians from horizontal)
     * @param hubVoltage current battery voltage
     * @return double[3]: {maxVel, maxAccel, maxDecel} in rad/s and rad/s^2
     */
    double[] computeMoveLimits(double fromRad, double toRad, double hubVoltage) {
        double midRad = (fromRad + toRad) / 2.0;

        // Worst-case angle in each phase
        double worstAccelAngle = worstCaseAngle(fromRad, midRad);
        double worstDecelAngle = worstCaseAngle(midRad, toRad);
        double worstVelAngle = worstCaseAngle(fromRad, toRad);

        // Use PARAMS.maxVelRad as the velocity for accel/decel queries (conservative:
        // back-EMF at max velocity consumes the most voltage, leaving least for torque)
        double accel = feedforward.maxAchievableAcceleration(
                hubVoltage, worstAccelAngle, PARAMS.maxVelRad);
        double decel = feedforward.maxAchievableAcceleration(
                hubVoltage, worstDecelAngle, PARAMS.maxVelRad);
        double vel = feedforward.maxAchievableVelocity(
                hubVoltage, worstVelAngle, 0);

        // Cap at global maximums and ensure non-negative
        vel = Math.min(Math.max(vel, 0), PARAMS.maxVelRad);
        accel = Math.min(Math.max(accel, 0), PARAMS.maxAccelRad);
        decel = Math.min(Math.max(decel, 0), PARAMS.maxDecelRad);

        return new double[] { vel, accel, decel };
    }

    /**
     * Find the angle in the range between {@code a} and {@code b} where {@code |cos(theta)|}
     * is maximized (worst-case gravity torque). Candidates: endpoints plus any horizontal
     * crossing (0 or -pi) within the range.
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

        // Check horizontal crossings: 0 and -pi
        if (lo <= 0 && hi >= 0) {
            // cos(0) = 1.0
            bestAngle = 0;
            bestAbsCos = 1.0;
        }
        if (lo <= -Math.PI && hi >= -Math.PI) {
            // |cos(-pi)| = 1.0
            if (1.0 > bestAbsCos) {
                bestAngle = -Math.PI;
            }
        }

        return bestAngle;
    }
}
