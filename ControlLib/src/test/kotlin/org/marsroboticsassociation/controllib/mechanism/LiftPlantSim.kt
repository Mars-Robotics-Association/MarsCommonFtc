package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

/**
 * Test-only physics model of a linear lift: the lift counterpart to [ArmPlantSim]. Same DC-motor
 * dynamics, but gravity is a constant pull regardless of height, so the dynamics are linear:
 * <pre>
 *   dx/dt = v
 *   dv/dt = -(kV/kA)*v + (1/kA)*(u - kS*sign(v) - kG)
 * </pre>
 * Integrated with 4th-order Runge-Kutta. Position is read *live* from the plant (with a small
 * read-timing jitter, like the real hub), while the [EncoderSim] ring buffer models only the 50 ms
 * velocity window. Physics ([integrate]) is kept separate from the encoder latch ([latchEncoder])
 * so a test driver can run them at different rates.
 *
 * <p>Works in encoder ticks and ticks/sec; the feedforward constants are in volts.
 */
class LiftPlantSim
@JvmOverloads
constructor(
    private val kS: Double,
    private val kG: Double,
    kV: Double,
    kA: Double,
    private val minTicks: Double,
    private val maxTicks: Double,
    initialTicks: Double,
    private val positionJitter: ReadTimingJitter = ReadTimingJitter.disabled(),
) {
    private val aCoeff = -kV / kA // -kV/kA
    private val bCoeff = 1.0 / kA //  1/kA

    // Ring buffer models the 50 ms velocity window only; position is read live below.
    private val encoder = EncoderSim()

    private var positionTicks = initialTicks
    private var velocityTps = 0.0

    // One bulk-read snapshot per loop step: a shared transport delay (drawn lazily, invalidated on
    // integrate) and the time since the last 10 ms latch, used to stale velocity consistently.
    private var timeSinceLatchSec = 0.0
    private var snapshotDelta = Double.NaN

    init {
        for (i in 0 until 6) {
            encoder.sample(positionTicks)
        }
    }

    /** Advance the physics by dt seconds with the given normalized power and bus voltage. */
    fun integrate(dt: Double, normalizedPower: Double, voltage: Double) {
        val u = clamp(normalizedPower, -1.0, 1.0) * voltage
        rk4Step(u, dt)
        enforceHardStops()
        timeSinceLatchSec += dt
        snapshotDelta = Double.NaN // new step -> new bulk-read snapshot
    }

    /** Latch the current true position into the encoder ring buffer (call every 10 ms). */
    fun latchEncoder() {
        encoder.sample(positionTicks)
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
        ReadTimingJitter.staleTicks(positionTicks, velocityTps, snapshotDelta())

    /** Windowed encoder velocity, seen through the same bulk-read snapshot as the position read. */
    fun getEncoderVelocityTps(): Double =
        encoder.velocityTpsStaledBy(snapshotDelta(), timeSinceLatchSec)

    fun getTruePositionTicks(): Double = positionTicks

    fun getTrueVelocityTps(): Double = velocityTps

    private fun dVelocity(velocity: Double, u: Double): Double {
        val friction = kS * sign(velocity)
        return aCoeff * velocity + bCoeff * (u - friction - kG)
    }

    private fun rk4Step(u: Double, dt: Double) {
        val pos = positionTicks
        val vel = velocityTps

        val k1P = vel
        val k1V = dVelocity(vel, u)

        val k2P = vel + 0.5 * dt * k1V
        val k2V = dVelocity(vel + 0.5 * dt * k1V, u)

        val k3P = vel + 0.5 * dt * k2V
        val k3V = dVelocity(vel + 0.5 * dt * k2V, u)

        val k4P = vel + dt * k3V
        val k4V = dVelocity(vel + dt * k3V, u)

        positionTicks += (dt / 6.0) * (k1P + 2 * k2P + 2 * k3P + k4P)
        velocityTps += (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V)
    }

    private fun enforceHardStops() {
        if (positionTicks <= minTicks) {
            positionTicks = minTicks
            if (velocityTps < 0) velocityTps = 0.0
        }
        if (positionTicks >= maxTicks) {
            positionTicks = maxTicks
            if (velocityTps > 0) velocityTps = 0.0
        }
    }

    private fun clamp(v: Double, lo: Double, hi: Double): Double = max(lo, min(hi, v))
}
