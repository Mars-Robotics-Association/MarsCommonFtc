package org.marsroboticsassociation.controllib.sim;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;

/**
 * Physics mock for a single flywheel motor.
 *
 * <p>Simulates first-order linear velocity dynamics using the same
 * {@link LinearSystemId#identifyVelocitySystem} model used by {@code FlywheelStateSpace},
 * so closed-loop unit tests exercise the controller against its assumed plant.
 *
 * <p>State equation (TPS units):
 * <pre>
 *   dv/dt = A·v + B·u     A = −kV/kA,  B = 1/kA
 *   v = velocity (TPS),   u = applied voltage (V)
 * </pre>
 * Integrated with 4th-order Runge-Kutta.
 *
 * <p>Encoder model: an {@link EncoderSim} ring buffer converts true velocity into
 * integer tick positions sampled every 10 ms, faithfully reproducing the REV Hub
 * encoder behavior (5-entry buffer, 20 TPS quantization).
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
public class FlywheelMotorSim {

    private final double a;           // A matrix element: −kV/kA  [1/s]
    private final double b;           // B matrix element:  1/kA   [TPS/(V·s)]
    private final EncoderSim encoder;

    private double trueVelocityTps;
    private double disturbanceVoltage = 0.0;

    /**
     * Construct a flywheel plant simulation.
     *
     * @param kV feedforward velocity constant (same units as {@code FlywheelSimple.PARAMS.kV})
     * @param kA feedforward acceleration constant (same units as {@code FlywheelSimple.PARAMS.kA})
     */
    public FlywheelMotorSim(double kV, double kA) {
        this(kV, kA, 0, 0.0);
    }

    /**
     * Construct a flywheel plant simulation with existing state.
     *
     * @param kV           feedforward velocity constant
     * @param kA           feedforward acceleration constant
     * @param positionTicks current encoder position
     * @param velocityTps  current true velocity
     */
    public FlywheelMotorSim(double kV, double kA, int positionTicks, double velocityTps) {
        LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV, kA);
        this.a = plant.getA(0, 0);   // −kV/kA
        this.b = plant.getB(0, 0);   //  1/kA
        this.encoder = new EncoderSim();
        this.encoder.setState(positionTicks, positionTicks); // Use pos as initial fractionalTicks
        this.trueVelocityTps = velocityTps;
    }

    /**
     * Advance the simulation by one time step.
     *
     * @param dt              time step in seconds
     * @param normalizedPower motor power in [−1, 1]
     * @param nominalVoltage  bus voltage in volts (typically 12.0)
     */
    public void step(double dt, double normalizedPower, double nominalVoltage) {
        double u = normalizedPower * nominalVoltage + disturbanceVoltage;
        trueVelocityTps = rk4(trueVelocityTps, u, dt);
        if (trueVelocityTps < 0.0) trueVelocityTps = 0.0;
        encoder.advance(dt, trueVelocityTps);
    }

    /**
     * Inject a voltage-equivalent disturbance into the plant.
     * Positive = boost, negative = drag.
     * Ball engagement is approximately {@code −kA * expectedDecelerationTps2}.
     *
     * @param v disturbance voltage in volts
     */
    public void setDisturbanceVoltage(double v) {
        disturbanceVoltage = v;
    }

    /**
     * Reset the plant to the given velocity and clear encoder state.
     *
     * @param velocityTps initial velocity in ticks per second
     */
    public void reset(double velocityTps) {
        trueVelocityTps = velocityTps;
        encoder.reset();
    }

    /**
     * Returns the velocity in TPS from the encoder ring buffer.
     * Feed this to the controller under test.
     */
    public double getVelocityTps() {
        return encoder.getVelocityTps();
    }

    /**
     * Returns the most recent integer tick position from the encoder.
     */
    public int getPositionTicks() {
        return encoder.getPosition();
    }

    /**
     * Returns the true (noiseless) velocity in TPS. Use this for assertions in tests.
     */
    public double getTrueVelocityTps() {
        return trueVelocityTps;
    }

    // -------------------------------------------------------------------------
    // 4th-order Runge-Kutta integration of dv/dt = a·v + b·u
    // -------------------------------------------------------------------------

    private double dynamics(double v, double u) {
        return a * v + b * u;
    }

    private double rk4(double v, double u, double dt) {
        double k1 = dynamics(v,                  u);
        double k2 = dynamics(v + 0.5 * dt * k1, u);
        double k3 = dynamics(v + 0.5 * dt * k2, u);
        double k4 = dynamics(v +       dt * k3, u);
        return v + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    }
}
