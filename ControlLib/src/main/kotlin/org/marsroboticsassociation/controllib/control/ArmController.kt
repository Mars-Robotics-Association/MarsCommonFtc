package org.marsroboticsassociation.controllib.control

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.ArmFeedforward
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
 * Arm position controller using S-curve trajectory + ArmFeedforward + Kalman filter + PD feedback.
 *
 * The Kalman filter runs on a linear motor model (no gravity) and predicts the arm state forward in
 * time to compensate for sensor-to-actuator latency. Gravity compensation is handled entirely by
 * [ArmFeedforward], which is applied additively as a voltage offset.
 *
 * Voltage normalization: feedforward and PD output voltages which are divided by the live hub
 * voltage to produce motor power in [-1, 1], compensating for battery sag.
 *
 * Near hard stops, the controller coasts (cuts power) to let the arm rest against the stop.
 *
 * Usage:
 * ```
 *   val arm = ArmController(motor, telemetry::addData)
 *   arm.setTarget(Math.toRadians(45.0))
 *   // each loop iteration:
 *   arm.update(dt)
 * ```
 */
class ArmController {

    /**
     * Tuning parameters. Shared between instances.
     *
     * All angles are in radians measured from horizontal (positive = above horizontal). All
     * velocities/accelerations are in rad/s and rad/s^2 at the output shaft.
     */
    class Params {
        // --- Motor / Encoder ---
        @JvmField var ticksPerRev: Int = 28 // motor encoder ticks per motor revolution
        @JvmField var gearRatio: Double = 100.0 // output:input (e.g. 100:1)

        // --- Geometry ---
        /** Angle from horizontal when the encoder reads 0 (front hard stop). Found by sysid. */
        @JvmField var encoderZeroOffsetRad: Double = -PI / 4 // default: 45 deg below horizontal

        /** Back hard stop angle (radians from horizontal). */
        @JvmField var minAngleRad: Double = -PI * 5 / 4 // -225 deg = 45 deg below horizontal behind
        /** Front hard stop angle (radians from horizontal). Encoder reads 0 here. */
        @JvmField var maxAngleRad: Double = -PI / 4 // -45 deg = 45 deg below horizontal in front

        // --- Feedforward (ArmFeedforward gains, output-shaft units) ---
        @JvmField var ks: Double = 0.0 // static friction voltage
        @JvmField var kg: Double = 0.0 // gravity voltage at horizontal
        @JvmField var kv: Double = 0.0 // V/(rad/s)
        @JvmField var ka: Double = 0.0 // V/(rad/s^2)

        // --- Kalman filter noise model ---
        @JvmField var modelStdDevPos: Double = 0.01 // position process noise (rad)
        @JvmField var modelStdDevVel: Double = 3.0 // velocity process noise (rad/s)
        @JvmField var measurementStdDevPos: Double = 0.05 // position measurement noise (rad)
        @JvmField var measurementStdDevVel: Double = 0.5 // velocity measurement noise (rad/s)

        // --- PD feedback ---
        @JvmField var kP: Double = 0.0 // volts per radian of position error
        @JvmField var kD: Double = 0.0 // volts per (rad/s) of velocity error

        // --- Trajectory (SCurve) ---
        @JvmField var maxVelRad: Double = 2.0 // rad/s
        @JvmField var maxAccelRad: Double = 4.0 // rad/s^2 (acceleration limit)
        @JvmField var maxDecelRad: Double = 6.0 // rad/s^2 (deceleration limit, asymmetric)
        @JvmField var maxJerkRad: Double = 20.0 // rad/s^3

        // --- Tracking / replanning ---
        /** Replan trajectory if position tracking error exceeds this (rad). */
        @JvmField var replanThresholdRad: Double = Math.toRadians(15.0)

        // --- Hard stop behavior ---
        /** Coast (cut power) when within this angle of a hard stop (rad). */
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
        /** Forward-predict state by this many seconds to compensate sensor-to-actuator latency. */
        @JvmField var latencyCompensationSec: Double = 0.030

        // --- Nominal dt ---
        @JvmField var dtSeconds: Double = 0.020 // nominal loop period for Kalman gain computation

        /** Position tolerance for [isAtTarget] (rad). */
        @JvmField var atTargetPositionTolerance: Double = Math.toRadians(2.0)
        /** Velocity tolerance for [isAtTarget] (rad/s). */
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
    private val feedforward: ArmFeedforward
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
    /** Voltage with gravity+friction subtracted, consistent with the linear plant model. */
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
     * Test constructor with injectable clock for the trajectory manager.
     *
     * @param motor IMotor providing position, velocity, power, and hub voltage
     * @param telemetry typically `telemetry::addData`
     * @param clock clock supplier (nanoseconds) for trajectory timing
     */
    constructor(motor: IMotor, telemetry: TelemetryAddData, clock: LongSupplier) {
        this.motor = motor
        this.name = motor.name
        this.telemetry = telemetry

        // Build the linear plant (motor dynamics only, no gravity)
        // State: [position (rad), velocity (rad/s)], Input: [voltage], Output: [position, velocity]
        plant = LinearSystemId.identifyPositionSystem(PARAMS.kv, PARAMS.ka)

        // Build Kalman filter
        kalman =
            KalmanFilter(
                Nat.N2(),
                Nat.N2(),
                plant,
                VecBuilder.fill(PARAMS.modelStdDevPos, PARAMS.modelStdDevVel),
                VecBuilder.fill(PARAMS.measurementStdDevPos, PARAMS.measurementStdDevVel),
                PARAMS.dtSeconds,
            )

        // Build feedforward (nominal dt; variable dt handled via deprecated calculate overload)
        feedforward = ArmFeedforward(PARAMS.ks, PARAMS.kg, PARAMS.kv, PARAMS.ka, PARAMS.dtSeconds)

        // Build trajectory manager with injectable clock
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
     * Computes per-move trajectory limits based on the worst-case gravity torque across the sweep,
     * then plans the trajectory with those limits.
     *
     * If the arm is currently coasting at a hard stop, this wakes it up: the state is reset to the
     * hard stop angle with zero velocity, and a new trajectory is planned from there.
     */
    fun setTarget(angleRad: Double) {
        setTarget(angleRad, motor.hubVoltage)
    }

    /**
     * Set the target arm angle with a pre-sampled hub voltage (for testing).
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
            // Wake from hard stop: assume position is at the nearest hard stop
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

        // Compute gravity-aware trajectory limits for this move
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
    @Suppress("DEPRECATION", "removal")
    fun update(dt: Double, hubVoltage: Double) {
        if (dt < 1e-6) return

        // 1. Read sensors
        val measuredPosRad = ticksToRad(motor.position)
        val measuredVelRad = tpsToRadPerSec(motor.encoderVelocity)

        // 2. Kalman correct — use the linear-model-consistent voltage (gravity/friction removed)
        //    so the Kalman filter's plant model prediction matches the actual dynamics.
        val uLinear: Matrix<N1, N1> = VecBuilder.fill(lastLinearVoltage)
        kalman.correct(uLinear, VecBuilder.fill(measuredPosRad, measuredVelRad))

        // 3. Kalman predict
        kalman.predict(uLinear, dt)

        // 4. Forward predict (latency compensation)
        val predicted: Matrix<N2, N1> =
            plant.calculateX(kalman.xhat, uLinear, PARAMS.latencyCompensationSec)
        predictedPosRad = predicted.get(0, 0)
        predictedVelRad = predicted.get(1, 0)

        // 5. Coast check
        if (shouldCoast(predictedPosRad)) {
            mode = Mode.COASTING
            motor.setPower(0.0)
            lastVoltageCmded = 0.0
            lastLinearVoltage = 0.0
            lastPower = 0.0
            return
        }

        mode = Mode.TRACKING

        // 6. Trajectory update
        trajectory.update()
        var trajPos = trajectory.position
        var trajVel = trajectory.velocity
        var trajAccel = trajectory.acceleration

        // 7. Replan check
        if (abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
            // Recompute limits for the remaining sweep
            val limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage)
            trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad)

            trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad)
            // Re-read trajectory state after replan
            trajectory.update()
            trajPos = trajectory.position
            trajVel = trajectory.velocity
            trajAccel = trajectory.acceleration
        }

        // 8. Feedforward (use deprecated overload for variable dt with RK4 accuracy)
        //    Use predicted position (not trajectory position) for gravity compensation —
        //    gravity acts at the arm's actual angle, not where the trajectory expects it.
        val nextVel = trajVel + trajAccel * dt
        val ffVoltage = feedforward.calculate(predictedPosRad, trajVel, nextVel, dt)

        // 9. PD feedback
        val posError = trajPos - predictedPosRad
        val velError = trajVel - predictedVelRad
        val pdVoltage = PARAMS.kP * posError + PARAMS.kD * velError

        // 10. Output
        val totalVoltage = ffVoltage + pdVoltage
        lastVoltageCmded = totalVoltage
        lastPower = MathUtil.clamp(totalVoltage / hubVoltage, -1.0, 1.0)
        motor.setPower(lastPower)
        // For the Kalman filter: use the clamped (actually applied) voltage, then subtract
        // gravity and friction so the linear model sees only the motor dynamics portion.
        // This prevents biased predictions when gravity compensation is a large fraction
        // of the total voltage, and avoids feeding unrealistically large voltages to the
        // observer when the motor saturates.
        val actualVoltage = lastPower * hubVoltage
        val gravityVoltage = PARAMS.kg * cos(predictedPosRad)
        val frictionVoltage = PARAMS.ks * sign(predictedVelRad)
        lastLinearVoltage = actualVoltage - gravityVoltage - frictionVoltage
    }

    /** Predicted position in radians (after latency compensation). */
    val estimatedPositionRad: Double
        get() = predictedPosRad

    /** Predicted velocity in rad/s (after latency compensation). */
    val estimatedVelocityRadPerSec: Double
        get() = predictedVelRad

    /** Current trajectory setpoint position in radians. */
    val trajectoryPositionRad: Double
        get() = trajectory.position

    /** Current trajectory setpoint velocity in rad/s. */
    val trajectoryVelocityRadPerSec: Double
        get() = trajectory.velocity

    /** True when the predicted position is near the target and velocity is near zero. */
    val isAtTarget: Boolean
        get() =
            abs(predictedPosRad - targetAngleRad) < PARAMS.atTargetPositionTolerance &&
                abs(predictedVelRad) < PARAMS.atTargetVelocityTolerance

    /**
     * Reset the observer to the current measured state. Call after long idle or mode transitions.
     */
    fun reset() {
        val posRad = ticksToRad(motor.position)
        val velRad = tpsToRadPerSec(motor.encoderVelocity)
        kalman.setXhat(VecBuilder.fill(posRad, velRad))
        predictedPosRad = posRad
        predictedVelRad = velRad
        lastVoltageCmded = 0.0
        lastLinearVoltage = 0.0
    }

    /** Add arm controller telemetry to the driver station. */
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

    /**
     * Coast when both the target and the current position are near the same hard stop. This avoids
     * coasting when the arm is merely passing through the stop zone on its way to a target further
     * away.
     */
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

    /**
     * Compute per-move trajectory limits based on the worst-case gravity torque in the sweep.
     * Returns [maxVel, maxAccel, maxDecel].
     *
     * The sweep is split at the midpoint: the first half constrains acceleration, the second half
     * constrains deceleration. Max velocity is constrained across the full sweep. All results are
     * capped at the PARAMS global maximums.
     *
     * @param fromRad start angle (radians from horizontal)
     * @param toRad target angle (radians from horizontal)
     * @param hubVoltage current battery voltage
     * @return double[3]: {maxVel, maxAccel, maxDecel} in rad/s and rad/s^2
     */
    fun computeMoveLimits(fromRad: Double, toRad: Double, hubVoltage: Double): DoubleArray {
        val midRad = (fromRad + toRad) / 2.0

        // Worst-case angle in each phase
        val worstAccelAngle = worstCaseAngle(fromRad, midRad)
        val worstDecelAngle = worstCaseAngle(midRad, toRad)
        val worstVelAngle = worstCaseAngle(fromRad, toRad)

        // Use PARAMS.maxVelRad as the velocity for accel/decel queries (conservative:
        // back-EMF at max velocity consumes the most voltage, leaving least for torque)
        var accel =
            feedforward.maxAchievableAcceleration(hubVoltage, worstAccelAngle, PARAMS.maxVelRad)
        var decel =
            feedforward.maxAchievableAcceleration(hubVoltage, worstDecelAngle, PARAMS.maxVelRad)
        var vel = feedforward.maxAchievableVelocity(hubVoltage, worstVelAngle, 0.0)

        // Cap at global maximums and ensure non-negative
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
         * Find the angle in the range between [a] and [b] where `|cos(theta)|` is maximized
         * (worst-case gravity torque). Candidates: endpoints plus any horizontal crossing (0 or
         * -pi) within the range.
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

            // Check horizontal crossings: 0 and -pi
            if (lo <= 0 && hi >= 0) {
                // cos(0) = 1.0
                bestAngle = 0.0
                bestAbsCos = 1.0
            }
            if (lo <= -PI && hi >= -PI) {
                // |cos(-pi)| = 1.0
                if (1.0 > bestAbsCos) {
                    bestAngle = -PI
                }
            }

            return bestAngle
        }
    }
}
