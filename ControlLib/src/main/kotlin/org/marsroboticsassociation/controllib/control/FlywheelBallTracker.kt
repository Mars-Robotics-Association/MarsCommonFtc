package org.marsroboticsassociation.controllib.control

import kotlin.math.sign

/**
 * Augmented Kalman filter that estimates flywheel velocity and a ball-drag disturbance state.
 *
 * Physics model (continuous time):
 * ```
 *   d/dt [ω]  =  [[-kV/kA,  -1    ]] * [ω]  +  [[1/kA]] * u
 *        [d]     [[ 0,     -1/τ_d ]]   [d]     [[ 0  ]]
 *   y = ω
 * ```
 *
 * The disturbance state `d` represents ball drag in TPS-equivalent units. A positive `d` means
 * something is slowing the flywheel. When `d` rises above `detectionThreshold`, a ball is
 * considered engaged; when it falls below `exitThreshold`, the ball has cleared.
 *
 * The 2×2 symmetric covariance is stored as three scalars (p00, p01, p11) and all math is done as
 * scalar arithmetic — no matrix library needed.
 */
class FlywheelBallTracker(private val motorConfig: VelocityMotorPF.VelocityMotorPFConfig) {

    class Config {
        /** Disturbance decay time constant (s). ~ball transit duration. */
        @JvmField var tauD: Double = 0.12
        /**
         * Process noise variance for ω (TPS²). Keep low so the filter trusts the motor model for
         * omega and attributes measurement residuals to the disturbance d.
         */
        @JvmField var qOmega: Double = 10.0
        /**
         * Process noise variance for d (TPS²). Must be large enough that k1 reacts quickly to a
         * sustained measurement drop; ~400 gives |k1|≈0.013 at steady state.
         */
        @JvmField var qD: Double = 400.0
        /** Measurement noise variance (TPS²). ~30 TPS RMS encoder noise. */
        @JvmField var r: Double = 900.0
        /** d above this → ball detected (TPS). */
        @JvmField var detectionThreshold: Double = 50.0
        /** d below this → ball cleared (TPS). ~40% of detectionThreshold for hysteresis. */
        @JvmField var exitThreshold: Double = 20.0
    }

    // Estimated states
    private var omega: Double = 0.0
    private var d: Double = 0.0

    // Symmetric 2×2 covariance: [[p00, p01], [p01, p11]]
    private var p00: Double = 0.0
    private var p01: Double = 0.0
    private var p11: Double = 0.0

    private var ballDetected: Boolean = false

    init {
        reset()
    }

    /**
     * Runs one predict+update step of the Kalman filter.
     *
     * @param rawTpsMeasurement Raw (unfiltered) encoder velocity reading (TPS).
     * @param trajectoryTps The motor's current trajectory setpoint (TPS). Used to reconstruct the
     *   feedforward voltage applied to the motor.
     * @param dt Time since last call (seconds). Typically ~0.010 s.
     */
    fun update(rawTpsMeasurement: Double, trajectoryTps: Double, dt: Double) {
        val kV = motorConfig.kV
        val kA = motorConfig.kA
        val kS = motorConfig.kS

        // Reconstruct estimated applied voltage from feedforward at the setpoint.
        // The kP feedback term is not included; its residual effect lands in qOmega.
        val u = kS * sign(trajectoryTps) + kV * trajectoryTps

        // --- PREDICT ---
        val a = kV / kA // F[0][0] = 1 - a*dt
        val f00 = 1.0 - a * dt
        val f01 = -dt // F[0][1]
        // F[1][0] = 0, F[1][1] = 1 - dt/tauD
        val f11 = 1.0 - dt / config.tauD

        val omegaPred = f00 * omega + f01 * d + (dt / kA) * u
        val dPred = f11 * d

        // Covariance predict: P_pred = F * P * F' + Q
        // P_pred[0][0] = f00^2*p00 + 2*f00*f01*p01 + f01^2*p11 + qOmega
        val p00Pred = f00 * f00 * p00 + 2.0 * f00 * f01 * p01 + f01 * f01 * p11 + config.qOmega
        // P_pred[0][1] = f00*f11*p01 + f01*f11*p11  (using F[1][0]=0)
        val p01Pred = f00 * f11 * p01 + f01 * f11 * p11
        // P_pred[1][1] = f11^2*p11 + qD
        val p11Pred = f11 * f11 * p11 + config.qD

        // --- UPDATE --- (H = [1, 0], so S = p00_pred + r)
        val innovation = rawTpsMeasurement - omegaPred
        val S = p00Pred + config.r
        val k0 = p00Pred / S // Kalman gain for omega
        val k1 = p01Pred / S // Kalman gain for d

        omega = omegaPred + k0 * innovation
        d = dPred + k1 * innovation

        // Joseph-form-adjacent update (simplified for H=[1,0]):
        p00 = (1.0 - k0) * p00Pred
        p01 = (1.0 - k0) * p01Pred
        p11 = p11Pred - k1 * p01Pred

        // --- DETECTION with hysteresis ---
        if (d > config.detectionThreshold) ballDetected = true
        if (d < config.exitThreshold) ballDetected = false
    }

    /** Estimated ball-drag disturbance (TPS). Positive = ball is slowing the flywheel. */
    fun getDisturbance(): Double = d

    /** Kalman-filtered flywheel velocity estimate (TPS). */
    fun getOmegaEst(): Double = omega

    /** True while d is above detectionThreshold; false once it drops below exitThreshold. */
    fun isBallDetected(): Boolean = ballDetected

    /** Resets filter state. Call when flywheel goes idle or on first spin-up. */
    fun reset() {
        omega = 0.0
        d = 0.0
        p01 = 0.0
        p00 = 10000.0
        p11 = 10000.0
        ballDetected = false
    }

    companion object {
        @JvmField var config: Config = Config()
    }
}
