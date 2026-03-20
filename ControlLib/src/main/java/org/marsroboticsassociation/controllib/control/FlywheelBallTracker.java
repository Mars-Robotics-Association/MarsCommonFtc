package org.marsroboticsassociation.controllib.control;

/**
 * Augmented Kalman filter that estimates flywheel velocity and a ball-drag disturbance state.
 * <p>
 * Physics model (continuous time):
 * <pre>
 *   d/dt [ω]  =  [[-kV/kA,  -1    ]] * [ω]  +  [[1/kA]] * u
 *        [d]     [[ 0,     -1/τ_d ]]   [d]     [[ 0  ]]
 *   y = ω
 * </pre>
 * The disturbance state {@code d} represents ball drag in TPS-equivalent units.
 * A positive {@code d} means something is slowing the flywheel. When {@code d} rises
 * above {@code detectionThreshold}, a ball is considered engaged; when it falls below
 * {@code exitThreshold}, the ball has cleared.
 * <p>
 * The 2×2 symmetric covariance is stored as three scalars (p00, p01, p11) and all
 * math is done as scalar arithmetic — no matrix library needed.
 */
public class FlywheelBallTracker {

    public static class Config {
        /** Disturbance decay time constant (s). ~ball transit duration. */
        public double tauD = 0.12;
        /** Process noise variance for ω (TPS²). Keep low so the filter trusts the motor
         *  model for omega and attributes measurement residuals to the disturbance d. */
        public double qOmega = 10;
        /** Process noise variance for d (TPS²). Must be large enough that k1 reacts
         *  quickly to a sustained measurement drop; ~400 gives |k1|≈0.013 at steady state. */
        public double qD = 400;
        /** Measurement noise variance (TPS²). ~30 TPS RMS encoder noise. */
        public double r = 900;
        /** d above this → ball detected (TPS). */
        public double detectionThreshold = 50;
        /** d below this → ball cleared (TPS). ~40% of detectionThreshold for hysteresis. */
        public double exitThreshold = 20;
    }

    public static Config config = new Config();

    // Estimated states
    private double omega;
    private double d;

    // Symmetric 2×2 covariance: [[p00, p01], [p01, p11]]
    private double p00;
    private double p01;
    private double p11;

    private boolean ballDetected;

    private final VelocityMotorPF.VelocityMotorPFConfig motorConfig;

    /**
     * @param motorConfig The flywheel motor config providing kV, kA, and kS.
     *                    A reference is stored so live Dashboard changes take effect.
     */
    public FlywheelBallTracker(VelocityMotorPF.VelocityMotorPFConfig motorConfig) {
        this.motorConfig = motorConfig;
        reset();
    }

    /**
     * Runs one predict+update step of the Kalman filter.
     *
     * @param rawTpsMeasurement Raw (unfiltered) encoder velocity reading (TPS).
     * @param trajectoryTps     The motor's current trajectory setpoint (TPS).
     *                          Used to reconstruct the feedforward voltage applied to the motor.
     * @param dt                Time since last call (seconds). Typically ~0.010 s.
     */
    public void update(double rawTpsMeasurement, double trajectoryTps, double dt) {
        double kV = motorConfig.kV;
        double kA = motorConfig.kA;
        double kS = motorConfig.kS;

        // Reconstruct estimated applied voltage from feedforward at the setpoint.
        // The kP feedback term is not included; its residual effect lands in qOmega.
        double u = kS * Math.signum(trajectoryTps) + kV * trajectoryTps;

        // --- PREDICT ---
        double a = kV / kA;                      // F[0][0] = 1 - a*dt
        double f00 = 1.0 - a * dt;
        double f01 = -dt;                        // F[0][1]
        // F[1][0] = 0, F[1][1] = 1 - dt/tauD
        double f11 = 1.0 - dt / config.tauD;

        double omega_pred = f00 * omega + f01 * d + (dt / kA) * u;
        double d_pred     = f11 * d;

        // Covariance predict: P_pred = F * P * F' + Q
        // P_pred[0][0] = f00^2*p00 + 2*f00*f01*p01 + f01^2*p11 + qOmega
        double p00_pred = f00 * f00 * p00 + 2.0 * f00 * f01 * p01 + f01 * f01 * p11 + config.qOmega;
        // P_pred[0][1] = f00*f11*p01 + f01*f11*p11  (using F[1][0]=0)
        double p01_pred = f00 * f11 * p01 + f01 * f11 * p11;
        // P_pred[1][1] = f11^2*p11 + qD
        double p11_pred = f11 * f11 * p11 + config.qD;

        // --- UPDATE --- (H = [1, 0], so S = p00_pred + r)
        double innovation = rawTpsMeasurement - omega_pred;
        double S = p00_pred + config.r;
        double k0 = p00_pred / S;   // Kalman gain for omega
        double k1 = p01_pred / S;   // Kalman gain for d

        omega = omega_pred + k0 * innovation;
        d     = d_pred     + k1 * innovation;

        // Joseph-form-adjacent update (simplified for H=[1,0]):
        p00 = (1.0 - k0) * p00_pred;
        p01 = (1.0 - k0) * p01_pred;
        p11 = p11_pred - k1 * p01_pred;

        // --- DETECTION with hysteresis ---
        if (d > config.detectionThreshold) ballDetected = true;
        if (d < config.exitThreshold)      ballDetected = false;
    }

    /** Estimated ball-drag disturbance (TPS). Positive = ball is slowing the flywheel. */
    public double getDisturbance() { return d; }

    /** Kalman-filtered flywheel velocity estimate (TPS). */
    public double getOmegaEst() { return omega; }

    /** True while d is above detectionThreshold; false once it drops below exitThreshold. */
    public boolean isBallDetected() { return ballDetected; }

    /** Resets filter state. Call when flywheel goes idle or on first spin-up. */
    public void reset() {
        omega = 0.0;
        d = 0.0;
        p01 = 0.0;
        p00 = 10000.0;
        p11 = 10000.0;
        ballDetected = false;
    }
}
