package org.marsroboticsassociation.controllib.sim;

/**
 * Physics mock for a single-joint arm motor with gravity.
 *
 * <p>Simulates second-order nonlinear arm dynamics:
 * <pre>
 *   dθ/dt = ω
 *   dω/dt = −(kV/kA)·ω + (1/kA)·(u − kS·sign(ω) − kG·cos(θ))
 * </pre>
 * where θ is the arm angle measured from horizontal (positive = above horizontal),
 * ω is angular velocity in rad/s, and u is applied voltage.
 *
 * <p>Integrated with 4th-order Runge-Kutta. Hard stops clamp position and zero velocity.
 *
 * <p>Encoder model: an {@link EncoderSim} ring buffer converts true velocity into
 * integer tick positions sampled every 10 ms, faithfully reproducing the REV Hub
 * encoder behavior (6-entry buffer, 20 TPS quantization).
 *
 * <p>Typical use:
 * <pre>
 *   ArmMotorSim sim = new ArmMotorSim(kS, kG, kV, kA, ticksPerRev, gearRatio,
 *                                      encoderZeroOffsetRad, minAngleRad, maxAngleRad);
 *   for (int i = 0; i &lt; 300; i++) {
 *       int    posTicks = sim.getPositionTicks();
 *       double velTps   = sim.getVelocityTps();
 *       double power    = controller.update(dt);
 *       sim.step(dt, power, 12.0);
 *   }
 * </pre>
 */
public class ArmMotorSim {

    // Feedforward constants (output-shaft units, radians)
    private final double kS;   // static friction voltage
    private final double kG;   // gravity voltage at horizontal
    private final double kV;   // V/(rad/s)
    private final double kA;   // V/(rad/s^2)

    // Derived motor dynamics coefficients
    private final double aCoeff;   // −kV/kA  [1/s]
    private final double bCoeff;   //  1/kA   [rad/s per V·s]

    // Encoder conversion
    private final double ticksPerRad;  // motor encoder ticks per output radian
    private final double encoderZeroOffsetRad;  // angle (from horizontal) when encoder reads 0

    // Hard stops (radians from horizontal)
    private final double minAngleRad;
    private final double maxAngleRad;

    private final EncoderSim encoder;

    // True state (output shaft, radians from horizontal)
    private double truePositionRad;
    private double trueVelocityRadPerSec;
    private double disturbanceVoltage = 0.0;

    /**
     * Construct an arm plant simulation.
     *
     * @param kS                   static friction voltage
     * @param kG                   gravity voltage at horizontal
     * @param kV                   velocity constant V/(rad/s) at output shaft
     * @param kA                   acceleration constant V/(rad/s^2) at output shaft
     * @param ticksPerRev          motor encoder ticks per motor revolution (e.g. 28)
     * @param gearRatio            output:input gear ratio (e.g. 100.0)
     * @param encoderZeroOffsetRad angle from horizontal when encoder reads 0
     * @param minAngleRad          minimum angle hard stop (radians from horizontal)
     * @param maxAngleRad          maximum angle hard stop (radians from horizontal)
     * @param initialAngleRad      initial arm angle (radians from horizontal)
     */
    public ArmMotorSim(double kS, double kG, double kV, double kA,
                        int ticksPerRev, double gearRatio,
                        double encoderZeroOffsetRad,
                        double minAngleRad, double maxAngleRad,
                        double initialAngleRad) {
        this.kS = kS;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
        this.aCoeff = -kV / kA;
        this.bCoeff = 1.0 / kA;
        this.ticksPerRad = (ticksPerRev * gearRatio) / (2.0 * Math.PI);
        this.encoderZeroOffsetRad = encoderZeroOffsetRad;
        this.minAngleRad = minAngleRad;
        this.maxAngleRad = maxAngleRad;
        this.truePositionRad = initialAngleRad;
        this.trueVelocityRadPerSec = 0.0;

        // Seed encoder: set fractionalTicks to initial position, then write enough
        // samples at zero velocity to fill the ring buffer so getPosition/getVelocity work.
        double initialTicksDouble = (initialAngleRad - encoderZeroOffsetRad) * ticksPerRad;
        this.encoder = new EncoderSim();
        this.encoder.setState(0, initialTicksDouble);
        // Write 6 samples (fills the buffer) at zero velocity, spaced 10ms apart
        for (int i = 0; i < 6; i++) {
            this.encoder.advance(0.010, 0.0);
        }
    }

    /**
     * Advance the simulation by one time step.
     *
     * @param dt              time step in seconds
     * @param normalizedPower motor power in [-1, 1]
     * @param hubVoltage      bus voltage in volts (typically 12.0)
     */
    public void step(double dt, double normalizedPower, double hubVoltage) {
        double u = normalizedPower * hubVoltage + disturbanceVoltage;
        rk4Step(u, dt);
        enforceHardStops();
        // Feed encoder in motor ticks per second
        double velocityTps = trueVelocityRadPerSec * ticksPerRad;
        encoder.advance(dt, velocityTps);
    }

    /**
     * Inject a voltage-equivalent disturbance into the plant.
     *
     * @param v disturbance voltage in volts
     */
    public void setDisturbanceVoltage(double v) {
        disturbanceVoltage = v;
    }

    /** Returns the most recent integer tick position from the encoder ring buffer. */
    public int getPositionTicks() {
        return encoder.getPosition();
    }

    /** Returns the velocity in TPS from the encoder ring buffer. */
    public double getVelocityTps() {
        return encoder.getVelocityTps();
    }

    /** Returns the true (noiseless) position in radians from horizontal. */
    public double getTruePositionRad() {
        return truePositionRad;
    }

    /** Returns the true (noiseless) angular velocity in rad/s. */
    public double getTrueVelocityRadPerSec() {
        return trueVelocityRadPerSec;
    }

    // -------------------------------------------------------------------------
    // 4th-order Runge-Kutta integration of the arm dynamics
    //
    //   dθ/dt = ω
    //   dω/dt = aCoeff·ω + bCoeff·(u − kS·sign(ω) − kG·cos(θ))
    // -------------------------------------------------------------------------

    private double dTheta(double omega) {
        return omega;
    }

    private double dOmega(double theta, double omega, double u) {
        double friction = kS * Math.signum(omega);
        double gravity = kG * Math.cos(theta);
        return aCoeff * omega + bCoeff * (u - friction - gravity);
    }

    private void rk4Step(double u, double dt) {
        double th = truePositionRad;
        double om = trueVelocityRadPerSec;

        double k1_th = dTheta(om);
        double k1_om = dOmega(th, om, u);

        double k2_th = dTheta(om + 0.5 * dt * k1_om);
        double k2_om = dOmega(th + 0.5 * dt * k1_th, om + 0.5 * dt * k1_om, u);

        double k3_th = dTheta(om + 0.5 * dt * k2_om);
        double k3_om = dOmega(th + 0.5 * dt * k2_th, om + 0.5 * dt * k2_om, u);

        double k4_th = dTheta(om + dt * k3_om);
        double k4_om = dOmega(th + dt * k3_th, om + dt * k3_om, u);

        truePositionRad += (dt / 6.0) * (k1_th + 2 * k2_th + 2 * k3_th + k4_th);
        trueVelocityRadPerSec += (dt / 6.0) * (k1_om + 2 * k2_om + 2 * k3_om + k4_om);
    }

    private void enforceHardStops() {
        if (truePositionRad <= minAngleRad) {
            truePositionRad = minAngleRad;
            if (trueVelocityRadPerSec < 0) trueVelocityRadPerSec = 0;
        }
        if (truePositionRad >= maxAngleRad) {
            truePositionRad = maxAngleRad;
            if (trueVelocityRadPerSec > 0) trueVelocityRadPerSec = 0;
        }
    }

}
