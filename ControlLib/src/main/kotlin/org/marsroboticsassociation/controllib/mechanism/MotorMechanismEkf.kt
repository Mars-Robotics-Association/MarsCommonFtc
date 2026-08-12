package org.marsroboticsassociation.controllib.mechanism

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import kotlin.math.max
import kotlin.math.min

/**
 * Model-based extended Kalman filter (EKF) for a DC-motor driven mechanism. Where a
 * constant-velocity filter assumes velocity just coasts between updates, this predicts the motion
 * from a [MechanismModel] (the same model the controller uses), driven by the voltage the
 * controller actually commanded.
 *
 * <p><b>Prediction.</b> The predict step integrates the model's dynamics with the commanded
 * voltage, so it expects velocity to rise under drive, sag as back-EMF builds, and bend as gravity
 * changes with position. Those dynamics are nonlinear (an arm's gravity follows a cosine), so the
 * covariance is pushed through the model's Jacobian, which is what makes this an EKF rather than a
 * plain linear Kalman filter. For a lift, whose gravity is constant, the Jacobian is constant and
 * it reduces to a linear filter.
 *
 * <p><b>Process noise as model error.</b> `modelAccelStdDev` is how far the real mechanism can
 * accelerate beyond what the model predicts. With a good model it is small, so the filter leans on
 * prediction and gets low lag and low noise; widen it if the model is shaky and it degrades
 * gracefully toward kinematic behavior.
 *
 * <p><b>Velocity-lag compensation.</b> `getVelocity()` (the sensor) averages over a 50 ms window,
 * so it lags the present by about 25 ms (measured ~25-28 ms on a REV hub). The model gives an
 * instantaneous acceleration, so the velocity sensor is modeled as measuring `velocity -
 * velocityLagSec * acceleration`, which de-lags it.
 *
 * <p><b>Timing-jitter noise.</b> The position and velocity come from one bulk read, captured at the
 * hub's read instant and delivered after a small, variable transport delay (worse over an Expansion
 * Hub's RS485 hop). `getCurrentPosition()` is a live counter, so it needs no de-lag bias, but that
 * same delay smears both signals at speed: the position by `velocity * positionTimingJitterStdDev`,
 * and the windowed velocity by `acceleration * positionTimingJitterStdDev` (the delay can catch a
 * slightly older 50 ms window). Both variances are added to the measurement noise, so a fast,
 * hard-accelerating read is trusted a little less. The sub-millisecond mean of the delay is left as
 * a small bias absorbed by this noise rather than de-lagged. Set `positionTimingJitterStdDev` small
 * on the Control Hub and larger on an Expansion Hub to match the wiring.
 *
 * <p><b>Saturation awareness.</b> [predict] takes the commanded motor power (the `[-1, 1]` value
 * you pass `setPower`) and the live bus voltage, and integrates `clamp(power, -1, 1) * busVoltage`
 * &mdash; the voltage the motor actually applied. Because power is clamped to the rail, an
 * over-command cannot be expressed, so the filter never expects an acceleration the mechanism did
 * not get when the controller saturates (a hard move, a sagging battery, a stall).
 *
 * <p>The matrix work runs on preallocated [Matrix] scratch (via the destination-writing `*Into`
 * operations), so a steady-state predict/correct allocates nothing.
 *
 * <p>Typical use in a control loop:
 * <pre>
 *   filter.predict(dt, commandedPower, busVoltage);
 *   filter.correct(measuredPosition, measuredVelocity);
 *   double position = filter.getPosition();
 *   double velocity = filter.getVelocity();
 * </pre>
 */
class MotorMechanismEkf(
    private val model: MechanismModel,
    private val velocityLagSec: Double,
    private val modelAccelStdDev: Double,
    positionStdDev: Double,
    velocityStdDev: Double,
    private val positionTimingJitterStdDev: Double,
    initialPosition: Double,
) {

    private val positionMeasVar = positionStdDev * positionStdDev
    private val velocityMeasVar = velocityStdDev * velocityStdDev

    private val x: Matrix<N2, N1> = Matrix(Nat.N2(), Nat.N1()) // state [position, velocity]
    private val p: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2()) // 2x2 covariance
    private var lastVoltage = 0.0

    // Preallocated scratch so predict/correct allocate no matrices. All 2x2 unless noted.
    private val f: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val fT: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val q: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val h: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val hT: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val r: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val s: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val sInv: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val k: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val identity: Matrix<N2, N2> = Matrix.eye(Nat.N2())
    private val tmpA: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val tmpB: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val tmpC: Matrix<N2, N2> = Matrix(Nat.N2(), Nat.N2())
    private val predicted: Matrix<N2, N1> = Matrix(Nat.N2(), Nat.N1())
    private val z: Matrix<N2, N1> = Matrix(Nat.N2(), Nat.N1())
    private val innov: Matrix<N2, N1> = Matrix(Nat.N2(), Nat.N1())
    private val kInnov: Matrix<N2, N1> = Matrix(Nat.N2(), Nat.N1())
    private val rk4Out = DoubleArray(2)

    init {
        x.set(0, 0, initialPosition)
        x.set(1, 0, 0.0)
        p.set(0, 0, positionMeasVar)
        p.set(1, 1, INITIAL_VELOCITY_STD_DEV * INITIAL_VELOCITY_STD_DEV)
    }

    /**
     * Predict step: integrate the dynamics forward by dt under the commanded motor power, which is
     * the same `[-1, 1]` value you hand `DcMotor.setPower`.
     *
     * <p>Saturation awareness is built in: the power is clamped to `[-1, 1]` and scaled by the live
     * bus voltage, so the observer integrates the voltage that was *actually applied*
     * (`clamp(power, -1, 1) * busVoltage`), not a larger one the controller may have asked for.
     * Because the motor can only ever apply what the battery delivers, an over-command is
     * impossible to express here, so the filter can never expect an acceleration the mechanism did
     * not get. That applied voltage is also what [correct] uses to de-lag the velocity, so both
     * steps stay consistent.
     *
     * @param dt time step in seconds
     * @param commandedPower the commanded motor power, in `[-1, 1]` (clamped if outside)
     * @param busVoltage the live bus voltage in volts; scales power to the applied voltage
     */
    fun predict(dt: Double, commandedPower: Double, busVoltage: Double) {
        val appliedVoltage = clamp(commandedPower, -1.0, 1.0) * busVoltage
        this.lastVoltage = appliedVoltage
        val position = x.get(0, 0)
        val velocity = x.get(1, 0)

        rk4(position, velocity, appliedVoltage, dt, rk4Out)
        x.set(0, 0, rk4Out[0])
        x.set(1, 0, rk4Out[1])

        // F = I + A*dt, with A linearized at the prior mean via the model's slopes.
        val dAccelDPosition = model.accelerationSlopeByPosition(position)
        val dAccelDVelocity = model.accelerationSlopeByVelocity()
        f.set(0, 0, 1.0)
        f.set(0, 1, dt)
        f.set(1, 0, dAccelDPosition * dt)
        f.set(1, 1, 1.0 + dAccelDVelocity * dt)

        // p = F p F^T + Q
        f.transposeInto(fT)
        f.multInto(p, tmpA)
        tmpA.multInto(fT, tmpB)
        fillProcessNoise(dt)
        tmpB.plusInto(q, p)
    }

    /**
     * Correct step: fuse a live position read and a measured (lagged) velocity.
     *
     * <p>Position is live, so it is used as-is (no de-lag bias). Velocity is averaged over the
     * sensor's 50 ms window, so it is de-lagged by `velocityLagSec` using the model's acceleration.
     * The position read-timing jitter smears the reading by `velocity *
     * positionTimingJitterStdDev`, which is folded into the position measurement noise so a fast
     * read is trusted a little less.
     *
     * @param measuredPosition live `getCurrentPosition()` read
     * @param measuredVelocity `getVelocity()` from the same read
     */
    fun correct(measuredPosition: Double, measuredVelocity: Double) {
        val position = x.get(0, 0)
        val velocity = x.get(1, 0)
        val accel = model.acceleration(position, velocity, lastVoltage)

        // Position is live (used as-is); velocity is de-lagged by the sensor's 50 ms averaging lag.
        predicted.set(0, 0, position)
        predicted.set(1, 0, velocity - velocityLagSec * accel)

        val dAccelDPosition = model.accelerationSlopeByPosition(position)
        val dAccelDVelocity = model.accelerationSlopeByVelocity()
        h.set(0, 0, 1.0)
        h.set(0, 1, 0.0)
        h.set(1, 0, -velocityLagSec * dAccelDPosition)
        h.set(1, 1, 1.0 - velocityLagSec * dAccelDVelocity)

        // One bulk read, one transport delay: the same read-timing jitter (std
        // positionTimingJitterStdDev) stales both signals. On the live position it smears by
        // velocity * std; on the windowed velocity it smears by acceleration * std (catching a
        // slightly older 50 ms window). Add each variance to R so a fast, hard-accelerating read is
        // trusted a little less. The sub-millisecond mean of the delay is left as a bias absorbed
        // by
        // this noise rather than de-lagged.
        val jitterPositionStd = velocity * positionTimingJitterStdDev
        val jitterVelocityStd = accel * positionTimingJitterStdDev
        r.set(0, 0, positionMeasVar + jitterPositionStd * jitterPositionStd)
        r.set(0, 1, 0.0)
        r.set(1, 0, 0.0)
        r.set(1, 1, velocityMeasVar + jitterVelocityStd * jitterVelocityStd)

        z.set(0, 0, measuredPosition)
        z.set(1, 0, measuredVelocity)
        z.minusInto(predicted, innov) // innovation = z - predicted

        // S = H p H^T + R
        h.transposeInto(hT)
        h.multInto(p, tmpA)
        tmpA.multInto(hT, tmpB)
        tmpB.plusInto(r, s)
        s.invertInto(sInv)

        // K = p H^T S^-1
        p.multInto(hT, tmpA)
        tmpA.multInto(sInv, k)

        // x = x + K (z - predicted)
        k.multInto(innov, kInnov)
        x.plusInto(kInnov, x)

        // p = (I - K H) p
        k.multInto(h, tmpA)
        identity.minusInto(tmpA, tmpB)
        tmpB.multInto(p, tmpC)
        p.setTo(tmpC)
    }

    /** The filtered position estimate. */
    fun getPosition(): Double = x.get(0, 0)

    /** The filtered velocity estimate, de-lagged and ready for a PIDF D term. */
    fun getVelocity(): Double = x.get(1, 0)

    /**
     * Project the current estimate forward by `horizonSec` under a held command, *without* mutating
     * the filter. This is open-loop actuation-latency compensation: hand a controller where the
     * mechanism will be when its command actually lands, rather than where it is now. The command
     * is the same `[-1, 1]` power and bus voltage [predict] takes, so it is clamped and scaled
     * identically.
     *
     * @param horizonSec how far ahead to project, in seconds (0 returns the present estimate)
     * @param commandedPower the power that will be held over the horizon, in `[-1, 1]`
     * @param busVoltage the live bus voltage in volts
     * @return `{position, velocity}` projected to `now + horizonSec`
     */
    fun projectedState(
        horizonSec: Double,
        commandedPower: Double,
        busVoltage: Double,
    ): DoubleArray {
        val appliedVoltage = clamp(commandedPower, -1.0, 1.0) * busVoltage
        val out = DoubleArray(2)
        rk4(x.get(0, 0), x.get(1, 0), appliedVoltage, horizonSec, out)
        return out
    }

    private fun fillProcessNoise(dt: Double) {
        val qScale = modelAccelStdDev * modelAccelStdDev
        val dt2 = dt * dt
        val dt3 = dt2 * dt
        val dt4 = dt3 * dt
        q.set(0, 0, qScale * dt4 / 4.0)
        q.set(0, 1, qScale * dt3 / 2.0)
        q.set(1, 0, qScale * dt3 / 2.0)
        q.set(1, 1, qScale * dt2)
    }

    private fun rk4(
        position: Double,
        velocity: Double,
        voltage: Double,
        dt: Double,
        out: DoubleArray,
    ): DoubleArray {
        val k1P = velocity
        val k1V = model.acceleration(position, velocity, voltage)

        val k2P = velocity + 0.5 * dt * k1V
        val k2V = model.acceleration(position + 0.5 * dt * k1P, velocity + 0.5 * dt * k1V, voltage)

        val k3P = velocity + 0.5 * dt * k2V
        val k3V = model.acceleration(position + 0.5 * dt * k2P, velocity + 0.5 * dt * k2V, voltage)

        val k4P = velocity + dt * k3V
        val k4V = model.acceleration(position + dt * k3P, velocity + dt * k3V, voltage)

        out[0] = position + (dt / 6.0) * (k1P + 2 * k2P + 2 * k3P + k4P)
        out[1] = velocity + (dt / 6.0) * (k1V + 2 * k2V + 2 * k3V + k4V)
        return out
    }

    companion object {
        private const val INITIAL_VELOCITY_STD_DEV = 20.0

        private fun clamp(value: Double, lo: Double, hi: Double): Double {
            return max(lo, min(hi, value))
        }
    }
}
