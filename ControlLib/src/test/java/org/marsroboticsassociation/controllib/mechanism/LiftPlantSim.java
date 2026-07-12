package org.marsroboticsassociation.controllib.mechanism;

/**
 * Test-only physics model of a linear lift: the lift counterpart to {@link ArmPlantSim}. Same
 * DC-motor dynamics, but gravity is a constant pull regardless of height, so the dynamics are
 * linear:
 *
 * <pre>
 *   dx/dt = v
 *   dv/dt = -(kV/kA)*v + (1/kA)*(u - kS*sign(v) - kG)
 * </pre>
 *
 * Integrated with 4th-order Runge-Kutta. Position is read <em>live</em> from the plant (with a
 * small read-timing jitter, like the real hub), while the {@link EncoderSim} ring buffer models
 * only the 50 ms velocity window. Physics ({@link #integrate}) is kept separate from the encoder
 * latch ({@link #latchEncoder}) so a test driver can run them at different rates.
 *
 * <p>Works in encoder ticks and ticks/sec; the feedforward constants are in volts.
 */
public class LiftPlantSim {

    private final double kS;
    private final double kG;
    private final double aCoeff; // -kV/kA
    private final double bCoeff; //  1/kA

    private final double minTicks;
    private final double maxTicks;

    // Ring buffer models the 50 ms velocity window only; position is read live below.
    private final EncoderSim encoder = new EncoderSim();
    private final ReadTimingJitter positionJitter;

    private double positionTicks;
    private double velocityTps;

    // One bulk-read snapshot per loop step: a shared transport delay (drawn lazily, invalidated on
    // integrate) and the time since the last 10 ms latch, used to stale velocity consistently.
    private double timeSinceLatchSec = 0.0;
    private double snapshotDelta = Double.NaN;

    /** Construct with read-timing jitter disabled (exact live position). */
    public LiftPlantSim(
            double kS,
            double kG,
            double kV,
            double kA,
            double minTicks,
            double maxTicks,
            double initialTicks) {
        this(kS, kG, kV, kA, minTicks, maxTicks, initialTicks, ReadTimingJitter.disabled());
    }

    /** Construct with a configurable position read-timing jitter model. */
    public LiftPlantSim(
            double kS,
            double kG,
            double kV,
            double kA,
            double minTicks,
            double maxTicks,
            double initialTicks,
            ReadTimingJitter positionJitter) {
        this.kS = kS;
        this.kG = kG;
        this.aCoeff = -kV / kA;
        this.bCoeff = 1.0 / kA;
        this.minTicks = minTicks;
        this.maxTicks = maxTicks;
        this.positionJitter = positionJitter;
        this.positionTicks = initialTicks;
        this.velocityTps = 0.0;
        for (int i = 0; i < 6; i++) {
            encoder.sample(positionTicks);
        }
    }

    /** Advance the physics by dt seconds with the given normalized power and bus voltage. */
    public void integrate(double dt, double normalizedPower, double voltage) {
        double u = clamp(normalizedPower, -1.0, 1.0) * voltage;
        rk4Step(u, dt);
        enforceHardStops();
        timeSinceLatchSec += dt;
        snapshotDelta = Double.NaN; // new step -> new bulk-read snapshot
    }

    /** Latch the current true position into the encoder ring buffer (call every 10 ms). */
    public void latchEncoder() {
        encoder.sample(positionTicks);
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
        return ReadTimingJitter.staleTicks(positionTicks, velocityTps, snapshotDelta());
    }

    /** Windowed encoder velocity, seen through the same bulk-read snapshot as the position read. */
    public double getEncoderVelocityTps() {
        return encoder.velocityTpsStaledBy(snapshotDelta(), timeSinceLatchSec);
    }

    public double getTruePositionTicks() {
        return positionTicks;
    }

    public double getTrueVelocityTps() {
        return velocityTps;
    }

    private double dVelocity(double velocity, double u) {
        double friction = kS * Math.signum(velocity);
        return aCoeff * velocity + bCoeff * (u - friction - kG);
    }

    private void rk4Step(double u, double dt) {
        double pos = positionTicks;
        double vel = velocityTps;

        double k1P = vel;
        double k1V = dVelocity(vel, u);

        double k2P = vel + 0.5 * dt * k1V;
        double k2V = dVelocity(vel + 0.5 * dt * k1V, u);

        double k3P = vel + 0.5 * dt * k2V;
        double k3V = dVelocity(vel + 0.5 * dt * k2V, u);

        double k4P = vel + dt * k3V;
        double k4V = dVelocity(vel + dt * k3V, u);

        positionTicks += (dt / 6.0) * (k1P + 2 * k2P + 2 * k3P + k4P);
        velocityTps += (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V);
    }

    private void enforceHardStops() {
        if (positionTicks <= minTicks) {
            positionTicks = minTicks;
            if (velocityTps < 0) velocityTps = 0;
        }
        if (positionTicks >= maxTicks) {
            positionTicks = maxTicks;
            if (velocityTps > 0) velocityTps = 0;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
