package org.marsroboticsassociation.controllib.sim

import edu.wpi.first.math.system.plant.LinearSystemId

/**
 * Physics mock for a single flywheel motor.
 *
 * <p>Simulates first-order linear velocity dynamics using the same
 * [LinearSystemId.identifyVelocitySystem] model used by {@code FlywheelStateSpace}, so closed-loop
 * unit tests exercise the controller against its assumed plant.
 *
 * <p>State equation (TPS units):
 * <pre>
 *   dv/dt = A·v + B·u     A = −kV/kA,  B = 1/kA
 *   v = velocity (TPS),   u = applied voltage (V)
 * </pre>
 * Integrated with 4th-order Runge-Kutta.
 *
 * <p>Encoder model: an [EncoderSim] provides a live integer tick position, while its ring buffer
 * models the 50 ms velocity window (6-entry buffer, 20 TPS quantization). Flywheel control reads
 * only velocity, so the position model does not affect it.
 *
 * <p>Typical use:
 * <pre>
 *   FlywheelMotorSim sim = new FlywheelMotorSim(kV, kA);
 *   for (int i = 0; i &lt; 300; i++) {
 *       double vTps  = sim.getVelocityTps();           // ring-buffer velocity → feed to controller
 *       double power = controller.update(vTps, targetTps, dt);
 *       sim.step(dt, power, 12.0);
 *   }
 * </pre>
 */
class FlywheelMotorSim
@JvmOverloads
constructor(
    kV: Double,
    kA: Double,
    positionTicks: Int = 0,
    velocityTps: Double = 0.0,
) {

    private val a: Double // A matrix element: −kV/kA  [1/s]
    private val b: Double // B matrix element:  1/kA   [TPS/(V·s)]
    private val encoder: EncoderSim

    private var trueVelocityTps: Double
    private var disturbanceVoltage = 0.0

    init {
        val plant = LinearSystemId.identifyVelocitySystem(kV, kA)
        a = plant.getA(0, 0) // −kV/kA
        b = plant.getB(0, 0) //  1/kA
        encoder = EncoderSim()
        encoder.setState(
            positionTicks,
            positionTicks.toDouble(),
        ) // Use pos as initial fractionalTicks
        trueVelocityTps = velocityTps
    }

    /**
     * Advance the simulation by one time step.
     *
     * @param dt time step in seconds
     * @param normalizedPower motor power in [−1, 1]
     * @param nominalVoltage bus voltage in volts (typically 12.0)
     */
    fun step(dt: Double, normalizedPower: Double, nominalVoltage: Double) {
        val u = normalizedPower * nominalVoltage + disturbanceVoltage
        trueVelocityTps = rk4(trueVelocityTps, u, dt)
        if (trueVelocityTps < 0.0) trueVelocityTps = 0.0
        encoder.advance(dt, trueVelocityTps)
    }

    /**
     * Inject a voltage-equivalent disturbance into the plant. Positive = boost, negative = drag.
     * Ball engagement is approximately {@code −kA * expectedDecelerationTps2}.
     *
     * @param v disturbance voltage in volts
     */
    fun setDisturbanceVoltage(v: Double) {
        disturbanceVoltage = v
    }

    /**
     * Reset the plant to the given velocity and clear encoder state.
     *
     * @param velocityTps initial velocity in ticks per second
     */
    fun reset(velocityTps: Double) {
        trueVelocityTps = velocityTps
        encoder.reset()
    }

    /**
     * Returns the velocity in TPS from the encoder ring buffer. Feed this to the controller under
     * test.
     */
    fun getVelocityTps(): Double {
        return encoder.velocityTps
    }

    /** Returns the most recent integer tick position from the encoder. */
    fun getPositionTicks(): Int {
        return encoder.position
    }

    /** Returns the true (noiseless) velocity in TPS. Use this for assertions in tests. */
    fun getTrueVelocityTps(): Double {
        return trueVelocityTps
    }

    // -------------------------------------------------------------------------
    // 4th-order Runge-Kutta integration of dv/dt = a·v + b·u
    // -------------------------------------------------------------------------

    private fun dynamics(v: Double, u: Double): Double {
        return a * v + b * u
    }

    private fun rk4(v: Double, u: Double, dt: Double): Double {
        val k1 = dynamics(v, u)
        val k2 = dynamics(v + 0.5 * dt * k1, u)
        val k3 = dynamics(v + 0.5 * dt * k2, u)
        val k4 = dynamics(v + dt * k3, u)
        return v + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    }
}
