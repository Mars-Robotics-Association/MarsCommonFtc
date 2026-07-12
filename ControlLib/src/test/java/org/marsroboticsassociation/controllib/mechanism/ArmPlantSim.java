package org.marsroboticsassociation.controllib.mechanism;

/**
 * Test-only physics model of a single-joint arm with gravity, mirroring the dynamics used by the
 * team's {@code ArmMotorSim}. It exists so a test can run the {@link MotorMechanismEkf} inside a
 * closed control loop against a realistic plant.
 *
 * <p>Second-order nonlinear dynamics, in output-shaft radians measured from horizontal (positive
 * above horizontal):
 *
 * <pre>
 *   dθ/dt = ω
 *   dω/dt = -(kV/kA)·ω + (1/kA)·(u - kS·sign(ω) - kG·cos(θ))
 * </pre>
 *
 * where u is applied voltage. Integrated with 4th-order Runge-Kutta. Hard stops clamp the angle and
 * zero the velocity at the limits.
 *
 * <p>Position is read <em>live</em> from the plant (with a small read-timing jitter, like the real
 * hub), while the {@link EncoderSim} ring buffer models only the 50 ms velocity window. The physics
 * ({@link #integrate}) is kept separate from the encoder latch ({@link #latchEncoder}) so a test
 * driver can run the plant on a fine timestep, latch the velocity window every 10 ms like the REV
 * Hub firmware, and fire its control loop on a slower, unsynced cadence.
 */
public class ArmPlantSim {

    private final double kS;
    private final double kG;
    private final double aCoeff; // -kV/kA  [1/s]
    private final double bCoeff; //  1/kA   [(rad/s^2) per volt]

    private final double ticksPerRad;
    private final double minAngleRad;
    private final double maxAngleRad;

    // Ring buffer models the 50 ms velocity window only; position is read live below.
    private final EncoderSim encoder = new EncoderSim();
    private final ReadTimingJitter positionJitter;

    private double thetaRad;
    private double omegaRadPerSec;

    // One bulk-read snapshot per loop step: a shared transport delay (drawn lazily, invalidated on
    // integrate) and the time since the last 10 ms latch, used to stale velocity consistently.
    private double timeSinceLatchSec = 0.0;
    private double snapshotDelta = Double.NaN;

    // Actuation delay: a real round-trip lag in the plant. A commanded power does not drive the
    // motor until actuationDelaySec later. Off by default (the ring is null); opt in via
    // setActuationDelaySec. Modeled as a FIFO of recent commanded powers, one slot per integrate.
    private double actuationDelaySec = 0.0;
    private double[] powerRing = null;
    private int powerRingHead = -1;

    /** Construct with read-timing jitter disabled (exact live position). */
    public ArmPlantSim(
            double kS,
            double kG,
            double kV,
            double kA,
            double ticksPerRad,
            double minAngleRad,
            double maxAngleRad,
            double initialAngleRad) {
        this(
                kS,
                kG,
                kV,
                kA,
                ticksPerRad,
                minAngleRad,
                maxAngleRad,
                initialAngleRad,
                ReadTimingJitter.disabled());
    }

    /** Construct with a configurable position read-timing jitter model. */
    public ArmPlantSim(
            double kS,
            double kG,
            double kV,
            double kA,
            double ticksPerRad,
            double minAngleRad,
            double maxAngleRad,
            double initialAngleRad,
            ReadTimingJitter positionJitter) {
        this.kS = kS;
        this.kG = kG;
        this.aCoeff = -kV / kA;
        this.bCoeff = 1.0 / kA;
        this.ticksPerRad = ticksPerRad;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.positionJitter = positionJitter;
        this.thetaRad = initialAngleRad;
        this.omegaRadPerSec = 0.0;
        // Seed the encoder ring buffer with the arm at rest so velocity reads as 0.
        for (int i = 0; i < 6; i++) {
            encoder.sample(angleTicks());
        }
    }

    /**
     * Enable a real actuation delay: a commanded power only starts driving the motor this many
     * seconds later (a FIFO on the applied power). Off when 0. Call before integrating; assumes a
     * fixed integrate timestep.
     */
    public void setActuationDelaySec(double sec) {
        this.actuationDelaySec = Math.max(0.0, sec);
        this.powerRing = sec > 0.0 ? new double[1024] : null;
        this.powerRingHead = -1;
    }

    /** Advance the physics by dt seconds with the given normalized power and bus voltage. */
    public void integrate(double dt, double normalizedPower, double voltage) {
        double effectivePower = applyActuationDelay(normalizedPower, dt);
        double u = clamp(effectivePower, -1.0, 1.0) * voltage;
        rk4Step(u, dt);
        enforceHardStops();
        timeSinceLatchSec += dt;
        snapshotDelta = Double.NaN; // new step -> new bulk-read snapshot
    }

    /** Buffer the just-commanded power and return the one that has finished its transport delay. */
    private double applyActuationDelay(double power, double dt) {
        if (actuationDelaySec <= 0.0 || powerRing == null) {
            return power;
        }
        int len = powerRing.length;
        powerRingHead = (powerRingHead + 1) % len;
        powerRing[powerRingHead] = power;
        int delaySteps = (int) Math.round(actuationDelaySec / dt);
        if (delaySteps <= 0) {
            return power;
        }
        // Before delaySteps writes, this index lands on a still-zero slot: nothing has arrived yet.
        int idx = ((powerRingHead - delaySteps) % len + len) % len;
        return powerRing[idx];
    }

    /** Latch the current true angle into the encoder ring buffer (call every 10 ms). */
    public void latchEncoder() {
        encoder.sample(angleTicks());
        timeSinceLatchSec = 0.0;
    }

    /**
     * The shared read-timing delay for the current snapshot, drawn once and reused until integrate.
     */
    private double snapshotDelta() {
        if (Double.isNaN(snapshotDelta)) {
            snapshotDelta = positionJitter.nextDelta();
        }
        return snapshotDelta;
    }

    /**
     * Live encoder position in ticks, staled by the shared read-timing delay (like the real hub).
     */
    public int getEncoderPosition() {
        return ReadTimingJitter.staleTicks(angleTicks(), angularVelocityTps(), snapshotDelta());
    }

    /** Windowed encoder velocity, seen through the same bulk-read snapshot as the position read. */
    public double getEncoderVelocityTps() {
        return encoder.velocityTpsStaledBy(snapshotDelta(), timeSinceLatchSec);
    }

    /**
     * True arm angle in radians from horizontal (for assertions, not available on a real robot).
     */
    public double getTrueAngleRad() {
        return thetaRad;
    }

    /** True angular velocity in rad/s (for assertions). */
    public double getTrueAngularVelocityRadPerSec() {
        return omegaRadPerSec;
    }

    public double ticksPerRad() {
        return ticksPerRad;
    }

    private double angleTicks() {
        return thetaRad * ticksPerRad;
    }

    private double angularVelocityTps() {
        return omegaRadPerSec * ticksPerRad;
    }

    private double dOmega(double theta, double omega, double u) {
        double friction = kS * Math.signum(omega);
        double gravity = kG * Math.cos(theta);
        return aCoeff * omega + bCoeff * (u - friction - gravity);
    }

    private void rk4Step(double u, double dt) {
        double th = thetaRad;
        double om = omegaRadPerSec;

        double k1Th = om;
        double k1Om = dOmega(th, om, u);

        double k2Th = om + 0.5 * dt * k1Om;
        double k2Om = dOmega(th + 0.5 * dt * k1Th, om + 0.5 * dt * k1Om, u);

        double k3Th = om + 0.5 * dt * k2Om;
        double k3Om = dOmega(th + 0.5 * dt * k2Th, om + 0.5 * dt * k2Om, u);

        double k4Th = om + dt * k3Om;
        double k4Om = dOmega(th + dt * k3Th, om + dt * k3Om, u);

        thetaRad += (dt / 6.0) * (k1Th + 2 * k2Th + 2 * k3Th + k4Th);
        omegaRadPerSec += (dt / 6.0) * (k1Om + 2 * k2Om + 2 * k3Om + k4Om);
    }

    private void enforceHardStops() {
        if (thetaRad <= minAngleRad) {
            thetaRad = minAngleRad;
            if (omegaRadPerSec < 0) omegaRadPerSec = 0;
        }
        if (thetaRad >= maxAngleRad) {
            thetaRad = maxAngleRad;
            if (omegaRadPerSec > 0) omegaRadPerSec = 0;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
