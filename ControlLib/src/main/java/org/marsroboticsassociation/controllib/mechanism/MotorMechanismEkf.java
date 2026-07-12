package org.marsroboticsassociation.controllib.mechanism;

/**
 * Model-based extended Kalman filter (EKF) for a DC-motor driven mechanism. Where a
 * constant-velocity filter assumes velocity just coasts between updates, this predicts the motion
 * from a {@link MechanismModel} (the same model the controller uses), driven by the voltage the
 * controller actually commanded.
 *
 * <p><b>Prediction.</b> The predict step integrates the model's dynamics with the commanded
 * voltage, so it expects velocity to rise under drive, sag as back-EMF builds, and bend as gravity
 * changes with position. Those dynamics are nonlinear (an arm's gravity follows a cosine), so the
 * covariance is pushed through the model's Jacobian, which is what makes this an EKF rather than a
 * plain linear Kalman filter. For a lift, whose gravity is constant, the Jacobian is constant and
 * it reduces to a linear filter.
 *
 * <p><b>Process noise as model error.</b> {@code modelAccelStdDev} is how far the real mechanism
 * can accelerate beyond what the model predicts. With a good model it is small, so the filter leans
 * on prediction and gets low lag and low noise; widen it if the model is shaky and it degrades
 * gracefully toward kinematic behavior.
 *
 * <p><b>Velocity-lag compensation.</b> {@code getVelocity()} (the sensor) averages over a 50 ms
 * window, so it lags the present by about 25 ms (measured ~25-28 ms on a REV hub). The model gives
 * an instantaneous acceleration, so the velocity sensor is modeled as measuring {@code velocity -
 * velocityLagSec * acceleration}, which de-lags it.
 *
 * <p><b>Timing-jitter noise.</b> The position and velocity come from one bulk read, captured at the
 * hub's read instant and delivered after a small, variable transport delay (worse over an Expansion
 * Hub's RS485 hop). {@code getCurrentPosition()} is a live counter, so it needs no de-lag bias, but
 * that same delay smears both signals at speed: the position by {@code velocity *
 * positionTimingJitterStdDev}, and the windowed velocity by {@code acceleration *
 * positionTimingJitterStdDev} (the delay can catch a slightly older 50 ms window). Both variances
 * are added to the measurement noise, so a fast, hard-accelerating read is trusted a little less.
 * The sub-millisecond mean of the delay is left as a small bias absorbed by this noise rather than
 * de-lagged. Set {@code positionTimingJitterStdDev} small on the Control Hub and larger on an
 * Expansion Hub to match the wiring.
 *
 * <p><b>Saturation awareness.</b> {@link #predict(double, double, double)} takes the commanded
 * motor power (the {@code [-1, 1]} value you pass {@code setPower}) and the live bus voltage, and
 * integrates {@code clamp(power, -1, 1) * busVoltage} &mdash; the voltage the motor actually
 * applied. Because power is clamped to the rail, an over-command cannot be expressed, so the filter
 * never expects an acceleration the mechanism did not get when the controller saturates (a hard
 * move, a sagging battery, a stall).
 *
 * <p>Typical use in a control loop:
 *
 * <pre>
 *   filter.predict(dt, commandedPower, busVoltage);
 *   filter.correct(measuredPosition, measuredVelocity);
 *   double position = filter.getPosition();
 *   double velocity = filter.getVelocity();
 * </pre>
 */
public class MotorMechanismEkf {

    private final MechanismModel model;
    private final double velocityLagSec;
    private final double modelAccelStdDev;
    private final double positionMeasVar;
    private final double velocityMeasVar;
    private final double positionTimingJitterStdDev;

    private Matrix x; // state [position, velocity]
    private Matrix p; // 2x2 covariance
    private double lastVoltage = 0.0;

    private static final double INITIAL_VELOCITY_STD_DEV = 20.0;

    /**
     * @param model the shared mechanism model (arm or lift)
     * @param velocityLagSec half the getVelocity averaging window, typically 0.025 s
     * @param modelAccelStdDev how far the mechanism can accelerate beyond the model
     * @param positionStdDev encoder position noise
     * @param velocityStdDev measured-velocity noise (inflate; it shares the encoder)
     * @param positionTimingJitterStdDev std of the position read-timing delay (small on a Control
     *     Hub, larger on an Expansion Hub); inflates position noise by {@code velocity * this}
     * @param initialPosition the mechanism's position right now
     */
    public MotorMechanismEkf(
            MechanismModel model,
            double velocityLagSec,
            double modelAccelStdDev,
            double positionStdDev,
            double velocityStdDev,
            double positionTimingJitterStdDev,
            double initialPosition) {
        this.model = model;
        this.velocityLagSec = velocityLagSec;
        this.modelAccelStdDev = modelAccelStdDev;
        this.positionMeasVar = positionStdDev * positionStdDev;
        this.velocityMeasVar = velocityStdDev * velocityStdDev;
        this.positionTimingJitterStdDev = positionTimingJitterStdDev;

        this.x = Matrix.column(initialPosition, 0.0);
        this.p =
                new Matrix(
                        new double[][] {
                            {positionMeasVar, 0.0},
                            {0.0, INITIAL_VELOCITY_STD_DEV * INITIAL_VELOCITY_STD_DEV}
                        });
    }

    /**
     * Predict step: integrate the dynamics forward by dt under the commanded motor power, which is
     * the same {@code [-1, 1]} value you hand {@code DcMotor.setPower}.
     *
     * <p>Saturation awareness is built in: the power is clamped to {@code [-1, 1]} and scaled by
     * the live bus voltage, so the observer integrates the voltage that was <em>actually
     * applied</em> ({@code clamp(power, -1, 1) * busVoltage}), not a larger one the controller may
     * have asked for. Because the motor can only ever apply what the battery delivers, an
     * over-command is impossible to express here, so the filter can never expect an acceleration
     * the mechanism did not get. That applied voltage is also what {@link #correct} uses to de-lag
     * the velocity, so both steps stay consistent.
     *
     * @param dt time step in seconds
     * @param commandedPower the commanded motor power, in {@code [-1, 1]} (clamped if outside)
     * @param busVoltage the live bus voltage in volts; scales power to the applied voltage
     */
    public void predict(double dt, double commandedPower, double busVoltage) {
        double appliedVoltage = clamp(commandedPower, -1.0, 1.0) * busVoltage;
        this.lastVoltage = appliedVoltage;
        double position = x.get(0, 0);
        double velocity = x.get(1, 0);

        double[] next = rk4(position, velocity, appliedVoltage, dt);
        x = Matrix.column(next[0], next[1]);

        // F = I + A*dt, with A linearized at the prior mean via the model's slopes.
        double dAccelDPosition = model.accelerationSlopeByPosition(position);
        double dAccelDVelocity = model.accelerationSlopeByVelocity();
        Matrix f =
                new Matrix(
                        new double[][] {
                            {1.0, dt},
                            {dAccelDPosition * dt, 1.0 + dAccelDVelocity * dt}
                        });
        p = f.times(p).times(f.transpose()).plus(processNoise(dt));
    }

    /**
     * Correct step: fuse a live position read and a measured (lagged) velocity.
     *
     * <p>Position is live, so it is used as-is (no de-lag bias). Velocity is averaged over the
     * sensor's 50 ms window, so it is de-lagged by {@code velocityLagSec} using the model's
     * acceleration. The position read-timing jitter smears the reading by {@code velocity *
     * positionTimingJitterStdDev}, which is folded into the position measurement noise so a fast
     * read is trusted a little less.
     *
     * @param measuredPosition live {@code getCurrentPosition()} read
     * @param measuredVelocity {@code getVelocity()} from the same read
     */
    public void correct(double measuredPosition, double measuredVelocity) {
        double position = x.get(0, 0);
        double velocity = x.get(1, 0);
        double accel = model.acceleration(position, velocity, lastVoltage);

        // Position is live (used as-is); velocity is de-lagged by the sensor's 50 ms averaging lag.
        Matrix predicted = Matrix.column(position, velocity - velocityLagSec * accel);

        double dAccelDPosition = model.accelerationSlopeByPosition(position);
        double dAccelDVelocity = model.accelerationSlopeByVelocity();
        Matrix h =
                new Matrix(
                        new double[][] {
                            {1.0, 0.0},
                            {
                                -velocityLagSec * dAccelDPosition,
                                1.0 - velocityLagSec * dAccelDVelocity
                            }
                        });

        // One bulk read, one transport delay: the same read-timing jitter (std
        // positionTimingJitterStdDev) stales both signals. On the live position it smears by
        // velocity * std; on the windowed velocity it smears by acceleration * std (catching a
        // slightly older 50 ms window). Add each variance to R so a fast, hard-accelerating read is
        // trusted a little less. The sub-millisecond mean of the delay is left as a bias absorbed
        // by
        // this noise rather than de-lagged.
        double jitterPositionStd = velocity * positionTimingJitterStdDev;
        double jitterVelocityStd = accel * positionTimingJitterStdDev;
        Matrix r =
                new Matrix(
                        new double[][] {
                            {positionMeasVar + jitterPositionStd * jitterPositionStd, 0.0},
                            {0.0, velocityMeasVar + jitterVelocityStd * jitterVelocityStd}
                        });

        Matrix z = Matrix.column(measuredPosition, measuredVelocity);
        Matrix innovation = z.minus(predicted);
        Matrix s = h.times(p).times(h.transpose()).plus(r);
        Matrix k = p.times(h.transpose()).times(s.inverse());
        x = x.plus(k.times(innovation));
        p = Matrix.identity(2).minus(k.times(h)).times(p);
    }

    /** The filtered position estimate. */
    public double getPosition() {
        return x.get(0, 0);
    }

    /** The filtered velocity estimate, de-lagged and ready for a PIDF D term. */
    public double getVelocity() {
        return x.get(1, 0);
    }

    /**
     * Project the current estimate forward by {@code horizonSec} under a held command,
     * <em>without</em> mutating the filter. This is open-loop actuation-latency compensation: hand
     * a controller where the mechanism will be when its command actually lands, rather than where
     * it is now. The command is the same {@code [-1, 1]} power and bus voltage {@link #predict}
     * takes, so it is clamped and scaled identically.
     *
     * @param horizonSec how far ahead to project, in seconds (0 returns the present estimate)
     * @param commandedPower the power that will be held over the horizon, in {@code [-1, 1]}
     * @param busVoltage the live bus voltage in volts
     * @return {@code {position, velocity}} projected to {@code now + horizonSec}
     */
    public double[] projectedState(double horizonSec, double commandedPower, double busVoltage) {
        double appliedVoltage = clamp(commandedPower, -1.0, 1.0) * busVoltage;
        return rk4(x.get(0, 0), x.get(1, 0), appliedVoltage, horizonSec);
    }

    private static double clamp(double value, double lo, double hi) {
        return Math.max(lo, Math.min(hi, value));
    }

    private Matrix processNoise(double dt) {
        double q = modelAccelStdDev * modelAccelStdDev;
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        return new Matrix(
                new double[][] {
                    {q * dt4 / 4.0, q * dt3 / 2.0},
                    {q * dt3 / 2.0, q * dt2}
                });
    }

    private double[] rk4(double position, double velocity, double voltage, double dt) {
        double k1P = velocity;
        double k1V = model.acceleration(position, velocity, voltage);

        double k2P = velocity + 0.5 * dt * k1V;
        double k2V =
                model.acceleration(position + 0.5 * dt * k1P, velocity + 0.5 * dt * k1V, voltage);

        double k3P = velocity + 0.5 * dt * k2V;
        double k3V =
                model.acceleration(position + 0.5 * dt * k2P, velocity + 0.5 * dt * k2V, voltage);

        double k4P = velocity + dt * k3V;
        double k4V = model.acceleration(position + dt * k3P, velocity + dt * k3V, voltage);

        double positionNext = position + (dt / 6.0) * (k1P + 2 * k2P + 2 * k3P + k4P);
        double velocityNext = velocity + (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V);
        return new double[] {positionNext, velocityNext};
    }
}
