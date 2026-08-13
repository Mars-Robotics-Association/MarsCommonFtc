package org.marsroboticsassociation.controllib.control

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ArmFeedforward
import edu.wpi.first.math.controller.LinearQuadraticRegulator
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import java.util.function.LongSupplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.motion.PositionTrajectoryManager
import org.marsroboticsassociation.controllib.util.TelemetryAddData

/**
 * Vertical arm position controller using feedback linearization + LQR + Kalman filter.
 *
 * The design has three cleanly separated layers:
 * 1. **Feedback linearization** — cancels gravity (`kg*cos(theta)`) and static friction
 *    (`ks*sign(omega)`) at the predicted arm angle, reducing the nonlinear arm to a linear motor.
 * 2. **LQR + linear feedforward** — operates on the linearized plant. Feedforward provides
 *    `kv*vel + ka*accel`; LQR closes the loop on tracking error. LQR gains are derived from the
 *    plant model and Q/R cost weights, so they auto-adjust when motor parameters change.
 * 3. **Kalman filter** — linear state observer on the gravity-free plant, with forward prediction
 *    for latency compensation.
 *
 * The Kalman filter input is always the *linearized* voltage (total voltage minus the
 * gravity/friction cancellation), keeping the linear model consistent with the actual dynamics.
 *
 * Voltage normalization: output voltages are divided by the live hub voltage to produce motor power
 * in [-1, 1], compensating for battery sag. Near hard stops, the controller coasts (cuts power)
 * when both the target and position are in the coast zone.
 *
 * Usage:
 * ```
 *   val arm = VerticalArmController(motor, telemetry::addData)
 *   arm.setTarget(Math.toRadians(45.0))
 *   // each loop iteration:
 *   arm.update(dt)
 * ```
 */
class VerticalArmController {

    /**
     * Tuning parameters. Shared between instances.
     *
     * All angles are in radians measured from horizontal (positive = above horizontal). All
     * velocities/accelerations are in rad/s and rad/s^2 at the output shaft.
     */
    class Params {
        // --- Motor / Encoder ---
        @JvmField var ticksPerRev: Int = 28
        @JvmField var gearRatio: Double = 100.0

        // --- Geometry ---
        /** Angle from horizontal when the encoder reads 0 (front hard stop). */
        @JvmField var encoderZeroOffsetRad: Double = -PI / 4
        /** Back hard stop angle (radians from horizontal). */
        @JvmField var minAngleRad: Double = -PI * 5 / 4
        /** Front hard stop angle (radians from horizontal). */
        @JvmField var maxAngleRad: Double = -PI / 4

        // --- Feedforward (output-shaft units) ---
        @JvmField var ks: Double = 0.0 // static friction voltage
        @JvmField var kg: Double = 0.0 // gravity voltage at horizontal
        @JvmField var kv: Double = 0.0 // V/(rad/s)
        @JvmField var ka: Double = 0.0 // V/(rad/s^2)

        // --- Kalman filter noise model ---
        @JvmField var modelStdDevPos: Double = 0.01
        @JvmField var modelStdDevVel: Double = 3.0
        @JvmField var measurementStdDevPos: Double = 0.05
        @JvmField var measurementStdDevVel: Double = 0.5

        // --- LQR cost weights (replace PD gains) ---
        /** Position tolerance for LQR (rad). Smaller = tighter tracking. */
        @JvmField var qPosition: Double = 0.5
        /** Velocity tolerance for LQR (rad/s). Smaller = more aggressive damping. */
        @JvmField var qVelocity: Double = 5.0
        /** Control effort penalty (V). Larger = gentler output. */
        @JvmField var rVoltage: Double = 12.0

        // --- Trajectory (SCurve) ---
        @JvmField var maxVelRad: Double = 2.0
        @JvmField var maxAccelRad: Double = 4.0
        @JvmField var maxDecelRad: Double = 6.0
        @JvmField var maxJerkRad: Double = 20.0

        // --- Tracking / replanning ---
        @JvmField var replanThresholdRad: Double = Math.toRadians(15.0)

        // --- Hard stop behavior ---
        @JvmField var coastZoneRad: Double = Math.toRadians(10.0)

        // --- Backlash compensation (rest-only) ---
        /**
         * Total gearbox backlash at the output shaft (rad). When nonzero, targets are biased a
         * half-backlash against gravity so the load — which settles a half-backlash off the motor,
         * hanging on the gravity-loaded tooth face — comes to rest at the stated angle. Only the
         * endpoint moves; the motion is unchanged. 0 disables.
         */
        @JvmField var backlashRad: Double = 0.0
        /**
         * Gravity hold-voltage below which the rest bias tapers to zero (V). Near vertical, gravity
         * is too weak to pin the resting face, so a full bias would be a coin flip.
         */
        @JvmField var backlashTaperVolts: Double = 0.3
        /**
         * Static rest compliance: radians of motor-to-load droop per volt of gravity hold-voltage,
         * from gear-tooth and structural (arm-tube) elasticity. Adds a gravity-proportional term to
         * the rest bias; needs no taper since it vanishes with gravity on its own. On a real
         * mechanism, measure the rest droop at two angles: the constant part is the half-lash, the
         * slope is this. 0 disables.
         */
        @JvmField var restComplianceRadPerVolt: Double = 0.0

        // --- Latency compensation ---
        @JvmField var latencyCompensationSec: Double = 0.030

        // --- Nominal dt ---
        @JvmField var dtSeconds: Double = 0.020

        // --- Convergence ---
        @JvmField var atTargetPositionTolerance: Double = Math.toRadians(2.0)
        @JvmField var atTargetVelocityTolerance: Double = Math.toRadians(5.0)
    }

    enum class Mode {
        TRACKING,
        COASTING,
    }

    private val motor: IMotor
    private val name: String
    private val telemetry: TelemetryAddData

    // Components
    private val plant: LinearSystem<N2, N1, N2>
    private val kalman: KalmanFilter<N2, N1, N2>
    private val lqr: LinearQuadraticRegulator<N2, N1, N2>
    private val armFf: ArmFeedforward // used only for computeMoveLimits
    private val trajectory: PositionTrajectoryManager

    // State
    var mode: Mode = Mode.COASTING
        private set

    /**
     * The target angle the trajectory actually drives to: the stated target plus any rest-only
     * backlash bias, clamped to the hard stops.
     */
    var targetAngleRad: Double = 0.0
        private set

    private var lastVoltageCmded: Double = 0.0
    private var lastLinearVoltage: Double = 0.0
    private var lastPower: Double = 0.0

    // Cached predicted state (after latency compensation)
    private var predictedPosRad: Double = 0.0
    private var predictedVelRad: Double = 0.0

    /**
     * @param motor IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically `telemetry::addData`
     */
    constructor(
        motor: IMotor,
        telemetry: TelemetryAddData,
    ) : this(motor, telemetry, LongSupplier { System.nanoTime() })

    /**
     * Test constructor with injectable clock.
     *
     * @param motor IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically `telemetry::addData`
     * @param clock clock supplier (nanoseconds) for trajectory timing
     */
    constructor(motor: IMotor, telemetry: TelemetryAddData, clock: LongSupplier) {
        this.motor = motor
        this.name = motor.name
        this.telemetry = telemetry

        // Linear plant (motor dynamics only, no gravity)
        plant = LinearSystemId.identifyPositionSystem(PARAMS.kv, PARAMS.ka)

        // Kalman filter on the linear plant
        kalman =
            KalmanFilter(
                Nat.N2(),
                Nat.N2(),
                plant,
                VecBuilder.fill(PARAMS.modelStdDevPos, PARAMS.modelStdDevVel),
                VecBuilder.fill(PARAMS.measurementStdDevPos, PARAMS.measurementStdDevVel),
                PARAMS.dtSeconds,
            )

        // LQR on the linear plant (replaces hand-tuned PD)
        lqr =
            LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(PARAMS.qPosition, PARAMS.qVelocity),
                VecBuilder.fill(PARAMS.rVoltage),
                PARAMS.dtSeconds,
            )

        // ArmFeedforward used only for maxAchievableAcceleration/Velocity in computeMoveLimits
        armFf = ArmFeedforward(PARAMS.ks, PARAMS.kg, PARAMS.kv, PARAMS.ka, PARAMS.dtSeconds)

        // Trajectory manager
        trajectory =
            PositionTrajectoryManager(
                PARAMS.maxVelRad,
                PARAMS.maxAccelRad,
                PARAMS.maxDecelRad,
                PARAMS.maxJerkRad,
                Math.toRadians(0.5),
                telemetry,
                clock,
            )

        // Seed observer and trajectory to current position
        val posRad = ticksToRad(motor.position)
        val velRad = tpsToRadPerSec(motor.encoderVelocity)
        kalman.setXhat(VecBuilder.fill(posRad, velRad))
        targetAngleRad = posRad
        trajectory.resetFromMeasurement(posRad, velRad)
        predictedPosRad = posRad
        predictedVelRad = velRad
    }

    // ---------------------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------------------

    /**
     * Set the target arm angle in radians (measured from horizontal).
     *
     * Computes per-move trajectory limits based on worst-case gravity torque across the sweep, then
     * plans the trajectory with those limits.
     */
    fun setTarget(angleRad: Double) {
        setTarget(angleRad, motor.hubVoltage)
    }

    /**
     * Set the target arm angle with a pre-sampled hub voltage.
     *
     * @param angleRad target angle in radians from horizontal
     * @param hubVoltage current battery voltage in volts
     */
    fun setTarget(angleRad: Double, hubVoltage: Double) {
        targetAngleRad =
            MathUtil.clamp(
                angleRad + backlashBias(angleRad),
                PARAMS.minAngleRad,
                PARAMS.maxAngleRad,
            )

        val fromRad: Double
        if (mode == Mode.COASTING) {
            val hardStopAngle = nearestHardStop(predictedPosRad)
            kalman.setXhat(VecBuilder.fill(hardStopAngle, 0.0))
            trajectory.resetFromMeasurement(hardStopAngle, 0.0)
            predictedPosRad = hardStopAngle
            predictedVelRad = 0.0
            mode = Mode.TRACKING
            fromRad = hardStopAngle
        } else {
            fromRad = predictedPosRad
        }

        val limits = computeMoveLimits(fromRad, targetAngleRad, hubVoltage)
        trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad)
        trajectory.setTarget(targetAngleRad)
    }

    /**
     * Run one control cycle, reading battery voltage from the motor.
     *
     * @param dt elapsed time in seconds since the last update
     */
    fun update(dt: Double) {
        update(dt, motor.hubVoltage)
    }

    /**
     * Run one control cycle with a pre-sampled battery voltage.
     *
     * @param dt elapsed time in seconds since the last update
     * @param hubVoltage battery voltage in volts
     */
    fun update(dt: Double, hubVoltage: Double) {
        if (dt < 1e-6) return

        // 1. Read sensors
        val measuredPosRad = ticksToRad(motor.position)
        val measuredVelRad = tpsToRadPerSec(motor.encoderVelocity)

        // 2. Kalman correct + predict with linearized voltage
        val uLinear: Matrix<N1, N1> = VecBuilder.fill(lastLinearVoltage)
        kalman.correct(uLinear, VecBuilder.fill(measuredPosRad, measuredVelRad))
        kalman.predict(uLinear, dt)

        // 3. Forward predict for latency compensation
        val predicted: Matrix<N2, N1> =
            plant.calculateX(kalman.xhat, uLinear, PARAMS.latencyCompensationSec)
        predictedPosRad = predicted.get(0, 0)
        predictedVelRad = predicted.get(1, 0)

        // 4. Coast check
        if (shouldCoast(predictedPosRad)) {
            mode = Mode.COASTING
            motor.setPower(0.0)
            lastVoltageCmded = 0.0
            lastLinearVoltage = 0.0
            lastPower = 0.0
            return
        }

        mode = Mode.TRACKING

        // 5. Trajectory update + replan check
        trajectory.update()
        var trajPos = trajectory.position
        var trajVel = trajectory.velocity
        var trajAccel = trajectory.acceleration

        if (abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
            val limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage)
            trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad)
            trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad)
            trajectory.update()
            trajPos = trajectory.position
            trajVel = trajectory.velocity
            trajAccel = trajectory.acceleration
        }

        // 6. Layer 1: Feedback linearization — cancel gravity + friction at predicted angle
        val uCancel = PARAMS.kg * cos(predictedPosRad) + PARAMS.ks * sign(predictedVelRad)

        // 7. Layer 2a: Linear feedforward (kv*vel + ka*accel on the linearized plant)
        val uFfLinear = PARAMS.kv * trajVel + PARAMS.ka * trajAccel

        // 8. Layer 2b: LQR feedback on linearized tracking error
        val reference: Matrix<N2, N1> = VecBuilder.fill(trajPos, trajVel)
        val state: Matrix<N2, N1> = VecBuilder.fill(predictedPosRad, predictedVelRad)
        val uLqr = lqr.calculate(state, reference).get(0, 0)

        // 9. Total voltage
        val totalVoltage = uCancel + uFfLinear + uLqr
        lastVoltageCmded = totalVoltage
        lastPower = MathUtil.clamp(totalVoltage / hubVoltage, -1.0, 1.0)
        motor.setPower(lastPower)

        // 10. Compute linearized voltage for next Kalman cycle
        //     Use the clamped actual voltage minus the cancellation, so the linear model
        //     sees the true linear-equivalent input even under motor saturation.
        val actualVoltage = lastPower * hubVoltage
        lastLinearVoltage = actualVoltage - uCancel
    }

    val estimatedPositionRad: Double
        get() = predictedPosRad

    val estimatedVelocityRadPerSec: Double
        get() = predictedVelRad

    val trajectoryPositionRad: Double
        get() = trajectory.position

    val trajectoryVelocityRadPerSec: Double
        get() = trajectory.velocity

    val isAtTarget: Boolean
        get() =
            abs(predictedPosRad - targetAngleRad) < PARAMS.atTargetPositionTolerance &&
                abs(predictedVelRad) < PARAMS.atTargetVelocityTolerance

    fun reset() {
        val posRad = ticksToRad(motor.position)
        val velRad = tpsToRadPerSec(motor.encoderVelocity)
        kalman.setXhat(VecBuilder.fill(posRad, velRad))
        predictedPosRad = posRad
        predictedVelRad = velRad
        lastVoltageCmded = 0.0
        lastLinearVoltage = 0.0
    }

    fun writeTelemetry() {
        telemetry.addData(name + " arm mode", "%s", mode.name)
        telemetry.addData(name + " arm target deg", "%.1f", Math.toDegrees(targetAngleRad))
        telemetry.addData(name + " arm predicted deg", "%.1f", Math.toDegrees(predictedPosRad))
        telemetry.addData(name + " arm predicted v", "%.1f", Math.toDegrees(predictedVelRad))
        telemetry.addData(
            name + " arm traj pos deg",
            "%.1f",
            Math.toDegrees(trajectory.position),
        )
        telemetry.addData(name + " arm traj vel", "%.1f", Math.toDegrees(trajectory.velocity))
        telemetry.addData(name + " arm voltage cmd", "%.2f V", lastVoltageCmded)
        telemetry.addData(name + " arm power", "%.3f", lastPower)
    }

    // ---------------------------------------------------------------------------
    // Unit conversions
    // ---------------------------------------------------------------------------

    private fun ticksToRad(ticks: Int): Double {
        return ticks * 2.0 * PI / (PARAMS.ticksPerRev * PARAMS.gearRatio) +
            PARAMS.encoderZeroOffsetRad
    }

    private fun tpsToRadPerSec(tps: Double): Double {
        return tps * 2.0 * PI / (PARAMS.ticksPerRev * PARAMS.gearRatio)
    }

    // ---------------------------------------------------------------------------
    // Hard stop helpers
    // ---------------------------------------------------------------------------

    private fun shouldCoast(posRad: Double): Boolean {
        val posNearMin = posRad <= PARAMS.minAngleRad + PARAMS.coastZoneRad
        val posNearMax = posRad >= PARAMS.maxAngleRad - PARAMS.coastZoneRad
        val targetNearMin = targetAngleRad <= PARAMS.minAngleRad + PARAMS.coastZoneRad
        val targetNearMax = targetAngleRad >= PARAMS.maxAngleRad - PARAMS.coastZoneRad
        return (posNearMin && targetNearMin) || (posNearMax && targetNearMax)
    }

    private fun nearestHardStop(posRad: Double): Double {
        val distToMin = abs(posRad - PARAMS.minAngleRad)
        val distToMax = abs(posRad - PARAMS.maxAngleRad)
        return if (distToMin < distToMax) PARAMS.minAngleRad else PARAMS.maxAngleRad
    }

    // ---------------------------------------------------------------------------
    // Per-move trajectory limits
    // ---------------------------------------------------------------------------

    /**
     * Compute per-move trajectory limits based on the worst-case gravity torque in the sweep.
     * Returns [maxVel, maxAccel, maxDecel].
     */
    fun computeMoveLimits(fromRad: Double, toRad: Double, hubVoltage: Double): DoubleArray {
        val midRad = (fromRad + toRad) / 2.0

        val worstAccelAngle = worstCaseAngle(fromRad, midRad)
        val worstDecelAngle = worstCaseAngle(midRad, toRad)
        val worstVelAngle = worstCaseAngle(fromRad, toRad)

        var accel = armFf.maxAchievableAcceleration(hubVoltage, worstAccelAngle, PARAMS.maxVelRad)
        var decel = armFf.maxAchievableAcceleration(hubVoltage, worstDecelAngle, PARAMS.maxVelRad)
        var vel = armFf.maxAchievableVelocity(hubVoltage, worstVelAngle, 0.0)

        vel = min(max(vel, 0.0), PARAMS.maxVelRad)
        accel = min(max(accel, 0.0), PARAMS.maxAccelRad)
        decel = min(max(decel, 0.0), PARAMS.maxDecelRad)

        return doubleArrayOf(vel, accel, decel)
    }

    companion object {
        @JvmField var PARAMS: Params = Params()

        /**
         * The rest bias for a stated target: a half-backlash signed by which tooth face gravity
         * loads there (tapered where the gravity hold-voltage falls below
         * [Params.backlashTaperVolts]), plus the gravity-proportional elastic droop
         * ([Params.restComplianceRadPerVolt]).
         */
        @JvmStatic
        fun backlashBias(targetRad: Double): Double {
            if (PARAMS.backlashRad == 0.0 && PARAMS.restComplianceRadPerVolt == 0.0) {
                return 0.0
            }
            val gravityVolts = PARAMS.kg * cos(targetRad)
            var bias = PARAMS.restComplianceRadPerVolt * gravityVolts
            if (PARAMS.backlashRad > 0.0) {
                val pin = MathUtil.clamp(gravityVolts / PARAMS.backlashTaperVolts, -1.0, 1.0)
                bias += PARAMS.backlashRad / 2.0 * pin
            }
            return bias
        }

        /**
         * Find the angle in the range between a and b where |cos(theta)| is maximized (worst-case
         * gravity torque).
         */
        @JvmStatic
        fun worstCaseAngle(a: Double, b: Double): Double {
            val lo = min(a, b)
            val hi = max(a, b)

            var bestAngle = a
            var bestAbsCos = abs(cos(a))

            val absCosB = abs(cos(b))
            if (absCosB > bestAbsCos) {
                bestAngle = b
                bestAbsCos = absCosB
            }

            if (lo <= 0 && hi >= 0) {
                bestAngle = 0.0
                bestAbsCos = 1.0
            }
            if (lo <= -PI && hi >= -PI) {
                if (1.0 > bestAbsCos) {
                    bestAngle = -PI
                }
            }

            return bestAngle
        }
    }
}
