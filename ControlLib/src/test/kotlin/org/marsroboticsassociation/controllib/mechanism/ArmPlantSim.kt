package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt
import kotlin.math.sign

/**
 * Test-only physics model of a single-joint arm with gravity, mirroring the dynamics used by the
 * team's `ArmMotorSim`. It exists so a test can run the [MotorMechanismEkf] inside a closed control
 * loop against a realistic plant.
 *
 * <p>Second-order nonlinear dynamics, in output-shaft radians measured from horizontal (positive
 * above horizontal):
 * <pre>
 *   dθ/dt = ω
 *   dω/dt = -(kV/kA)·ω + (1/kA)·(u - kS·sign(ω) - kG·cos(θ))
 * </pre>
 * where u is applied voltage. Integrated with 4th-order Runge-Kutta. Hard stops clamp the angle and
 * zero the velocity at the limits.
 *
 * <p>Position is read *live* from the plant (with a small read-timing jitter, like the real hub),
 * while the [EncoderSim] ring buffer models only the 50 ms velocity window. The physics
 * ([integrate]) is kept separate from the encoder latch ([latchEncoder]) so a test driver can run
 * the plant on a fine timestep, latch the velocity window every 10 ms like the REV Hub firmware,
 * and fire its control loop on a slower, unsynced cadence.
 */
class ArmPlantSim
@JvmOverloads
constructor(
    private val kS: Double,
    private val kG: Double,
    kV: Double,
    kA: Double,
    private val ticksPerRad: Double,
    private val minAngleRad: Double,
    private val maxAngleRad: Double,
    initialAngleRad: Double,
    private val positionJitter: ReadTimingJitter = ReadTimingJitter.disabled(),
) {
    private val aCoeff = -kV / kA // -kV/kA  [1/s]
    private val bCoeff = 1.0 / kA //  1/kA   [(rad/s^2) per volt]

    // Ring buffer models the 50 ms velocity window only; position is read live below.
    private val encoder = EncoderSim()

    private var thetaRad = initialAngleRad
    private var omegaRadPerSec = 0.0

    // One bulk-read snapshot per loop step: a shared transport delay (drawn lazily, invalidated on
    // integrate) and the time since the last 10 ms latch, used to stale velocity consistently.
    private var timeSinceLatchSec = 0.0
    private var snapshotDelta = Double.NaN

    // Actuation delay: a real round-trip lag in the plant. A commanded power does not drive the
    // motor until actuationDelaySec later. Off by default (the ring is null); opt in via
    // setActuationDelaySec. Modeled as a FIFO of recent commanded powers, one slot per integrate.
    private var actuationDelaySec = 0.0
    private var powerRing: DoubleArray? = null
    private var powerRingHead = -1

    init {
        // Seed the encoder ring buffer with the arm at rest so velocity reads as 0.
        for (i in 0 until 6) {
            encoder.sample(angleTicks())
        }
    }

    /**
     * Enable a real actuation delay: a commanded power only starts driving the motor this many
     * seconds later (a FIFO on the applied power). Off when 0. Call before integrating; assumes a
     * fixed integrate timestep.
     */
    fun setActuationDelaySec(sec: Double) {
        actuationDelaySec = max(0.0, sec)
        powerRing = if (sec > 0.0) DoubleArray(1024) else null
        powerRingHead = -1
    }

    /** Advance the physics by dt seconds with the given normalized power and bus voltage. */
    fun integrate(dt: Double, normalizedPower: Double, voltage: Double) {
        val effectivePower = applyActuationDelay(normalizedPower, dt)
        val u = clamp(effectivePower, -1.0, 1.0) * voltage
        rk4Step(u, dt)
        enforceHardStops()
        timeSinceLatchSec += dt
        snapshotDelta = Double.NaN // new step -> new bulk-read snapshot
    }

    /** Buffer the just-commanded power and return the one that has finished its transport delay. */
    private fun applyActuationDelay(power: Double, dt: Double): Double {
        val ring = powerRing
        if (actuationDelaySec <= 0.0 || ring == null) {
            return power
        }
        val len = ring.size
        powerRingHead = (powerRingHead + 1) % len
        ring[powerRingHead] = power
        val delaySteps = (actuationDelaySec / dt).roundToInt()
        if (delaySteps <= 0) {
            return power
        }
        // Before delaySteps writes, this index lands on a still-zero slot: nothing has arrived yet.
        val idx = ((powerRingHead - delaySteps) % len + len) % len
        return ring[idx]
    }

    /** Latch the current true angle into the encoder ring buffer (call every 10 ms). */
    fun latchEncoder() {
        encoder.sample(angleTicks())
        timeSinceLatchSec = 0.0
    }

    /**
     * The shared read-timing delay for the current snapshot, drawn once and reused until integrate.
     */
    private fun snapshotDelta(): Double {
        if (snapshotDelta.isNaN()) {
            snapshotDelta = positionJitter.nextDelta()
        }
        return snapshotDelta
    }

    /**
     * Live encoder position in ticks, staled by the shared read-timing delay (like the real hub).
     */
    fun getEncoderPosition(): Int =
        ReadTimingJitter.staleTicks(angleTicks(), angularVelocityTps(), snapshotDelta())

    /** Windowed encoder velocity, seen through the same bulk-read snapshot as the position read. */
    fun getEncoderVelocityTps(): Double =
        encoder.velocityTpsStaledBy(snapshotDelta(), timeSinceLatchSec)

    /**
     * True arm angle in radians from horizontal (for assertions, not available on a real robot).
     */
    fun getTrueAngleRad(): Double = thetaRad

    /** True angular velocity in rad/s (for assertions). */
    fun getTrueAngularVelocityRadPerSec(): Double = omegaRadPerSec

    fun ticksPerRad(): Double = ticksPerRad

    private fun angleTicks(): Double = thetaRad * ticksPerRad

    private fun angularVelocityTps(): Double = omegaRadPerSec * ticksPerRad

    private fun dOmega(theta: Double, omega: Double, u: Double): Double {
        val friction = kS * sign(omega)
        val gravity = kG * cos(theta)
        return aCoeff * omega + bCoeff * (u - friction - gravity)
    }

    private fun rk4Step(u: Double, dt: Double) {
        val th = thetaRad
        val om = omegaRadPerSec

        val k1Th = om
        val k1Om = dOmega(th, om, u)

        val k2Th = om + 0.5 * dt * k1Om
        val k2Om = dOmega(th + 0.5 * dt * k1Th, om + 0.5 * dt * k1Om, u)

        val k3Th = om + 0.5 * dt * k2Om
        val k3Om = dOmega(th + 0.5 * dt * k2Th, om + 0.5 * dt * k2Om, u)

        val k4Th = om + dt * k3Om
        val k4Om = dOmega(th + dt * k3Th, om + dt * k3Om, u)

        thetaRad += (dt / 6.0) * (k1Th + 2 * k2Th + 2 * k3Th + k4Th)
        omegaRadPerSec += (dt / 6.0) * (k1Om + 2 * k2Om + 2 * k3Om + k4Om)
    }

    private fun enforceHardStops() {
        if (thetaRad <= minAngleRad) {
            thetaRad = minAngleRad
            if (omegaRadPerSec < 0) omegaRadPerSec = 0.0
        }
        if (thetaRad >= maxAngleRad) {
            thetaRad = maxAngleRad
            if (omegaRadPerSec > 0) omegaRadPerSec = 0.0
        }
    }

    private fun clamp(v: Double, lo: Double, hi: Double): Double = max(lo, min(hi, v))
}
