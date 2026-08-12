package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim

/**
 * Closed-loop test of the mechanism [MotorMechanismController] (built on an [ArmModel], with a
 * [MotorMechanismEkf] estimating state) driving the team's two-mass [BacklashArmMotorSim] from
 * MarsCommonFtc instead of the rigid [ArmPlantSim].
 *
 * <p>The controller and filter model a single rigid arm; the plant does not. `BacklashArmMotorSim`
 * splits the motor and the load across a gearbox dead band, so the encoder (motor side) is blind to
 * where the arm (load side) actually is by up to half the backlash. This test scores against the
 * *load* ground truth, [BacklashArmMotorSim.getTruePositionRad] — the angle the arm is really at —
 * to see how the rigid-model controller copes with lash.
 *
 * <p>Two scenarios:
 * <ul>
 * <li>A monotonic sweep up through horizontal (where gravity torque peaks), then a hold. The
 *   steady-state load error is expected to sit within roughly the half-backlash dead band, since
 *   the controller can only servo the motor side it can see.
 * <li>A direction reversal, the case backlash punishes hardest: on reversal the motor must cross
 *   the full dead band before the load responds (lost motion), and while the teeth are separated
 *   the arm free-falls under gravity. The controller must still recover the load to the new target.
 * </ul>
 */
class ArmControllerBacklashTest {

    @Test
    fun reachesAndHoldsTargetThroughBacklash() {
        val start = -PI / 4 // -45 deg
        val target = PI / 4 //  +45 deg

        val plant = makePlant(start)
        val model = ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(
                model,
                /* kP= */ 40.0,
                /* kI= */ 8.0,
                /* kD= */ 1.5,
                /* maxVelocity= */ 8.0,
                /* maxAcceleration= */ 12.0,
                /* maxJerk= */ 480.0,
                /* feedbackVoltageMargin= */ 1.5,
                start,
            )

        val rng = Random(1L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var maxOvershootDeg = 0.0
        var sumSteadyAbsErrDeg = 0.0
        var steadySamples = 0

        for (ms in 0..3000) {
            plant.step(0.001, power, VOLTAGE)
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD)

                val voltage =
                    controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt)
                power = voltage / VOLTAGE

                val trueDeg = Math.toDegrees(plant.getTruePositionRad())
                maxOvershootDeg = max(maxOvershootDeg, trueDeg - 45.0)
                if (ms / 1000.0 > 2.0) {
                    sumSteadyAbsErrDeg += abs(trueDeg - 45.0)
                    steadySamples++
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val finalDeg = Math.toDegrees(plant.getTruePositionRad())
        val steadyAbsErrDeg = sumSteadyAbsErrDeg / steadySamples
        System.out.printf(
            "backlash sweep: final=%.2f deg, steady|err|=%.2f deg, overshoot=%.2f deg " +
                "(half-backlash=%.2f deg)%n",
            finalDeg,
            steadyAbsErrDeg,
            maxOvershootDeg,
            HALF_BACKLASH_DEG,
        )

        // The controller servos the motor side it can see; the load can rest up to a half-backlash
        // off, plus a little gravity sag. Convergence within the full backlash band is the bar.
        assertTrue(
            abs(finalDeg - 45.0) < Math.toDegrees(BACKLASH_RAD) + 2.0,
            "load did not reach target through backlash, final $finalDeg deg",
        )
        assertTrue(
            steadyAbsErrDeg < HALF_BACKLASH_DEG + 2.0,
            "steady-state load error too high for backlash band: $steadyAbsErrDeg deg",
        )
        assertTrue(maxOvershootDeg < 10.0, "overshoot too high: $maxOvershootDeg deg")
    }

    @Test
    fun recoversTheLoadAfterADirectionReversal() {
        val start = -PI / 4 // -45 deg
        val up = PI / 6 //  +30 deg
        val down = -PI / 3 //  -60 deg, a reversal back through the lash

        val plant = makePlant(start)
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, start)

        val rng = Random(7L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        // Phase 1: go up to +30 and settle. Phase 2 (after 1.8 s): reverse down to -60.
        var reversalSettleErrDeg = 0.0
        var reversalSamples = 0

        for (ms in 0..4000) {
            val target = if (ms < 1800) up else down
            plant.step(0.001, power, VOLTAGE)
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD)

                val voltage =
                    controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt)
                power = voltage / VOLTAGE

                if (ms / 1000.0 > 3.0) {
                    reversalSettleErrDeg +=
                        abs(Math.toDegrees(plant.getTruePositionRad()) - (-60.0))
                    reversalSamples++
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val finalDeg = Math.toDegrees(plant.getTruePositionRad())
        val settleErrDeg = reversalSettleErrDeg / reversalSamples
        System.out.printf(
            "backlash reversal: final=%.2f deg (target -60), settle|err|=%.2f deg%n",
            finalDeg,
            settleErrDeg,
        )

        assertTrue(
            abs(finalDeg - (-60.0)) < Math.toDegrees(BACKLASH_RAD) + 2.0,
            "load did not recover to the reversed target, final $finalDeg deg",
        )
        assertTrue(
            settleErrDeg < HALF_BACKLASH_DEG + 2.0,
            "load did not settle after reversal: $settleErrDeg deg",
        )
    }

    /**
     * Runs the identical controller, filter, gains, and move against the rigid [ArmPlantSim] and
     * against the backlash plant, and contrasts the steady-state load error. The rigid plant should
     * converge tight; the backlash plant should stay bounded by the dead band but visibly worse —
     * the cost of a rigid-model controller meeting a lashy gearbox, made explicit.
     */
    @Test
    fun rigidConvergesTightWhileBacklashStaysWithinTheBand() {
        val start = -PI / 4 // -45 deg
        val target = PI / 4 //  +45 deg

        val rigidErrDeg = runRigid(start, target)
        val backlashErrDeg = runBacklash(start, target)
        System.out.printf(
            "rigid vs backlash: rigid steady|err|=%.2f deg, backlash steady|err|=%.2f deg " +
                "(half-backlash=%.2f deg)%n",
            rigidErrDeg,
            backlashErrDeg,
            HALF_BACKLASH_DEG,
        )

        // Rigid plant: the controller's model is exact, so it converges tight.
        assertTrue(rigidErrDeg < 1.5, "rigid plant steady error too high: $rigidErrDeg deg")
        // Backlash plant: bounded by the dead band the controller cannot see across...
        assertTrue(
            backlashErrDeg < HALF_BACKLASH_DEG + 2.0,
            "backlash plant steady error outside the band: $backlashErrDeg deg",
        )
        // ...and measurably worse than rigid — that is the point of running both.
        assertTrue(
            backlashErrDeg > rigidErrDeg,
            "backlash should degrade tracking vs rigid (rigid=$rigidErrDeg, backlash=$backlashErrDeg)",
        )
    }

    /**
     * Rest-only backlash compensation: bias the target a half-backlash *against* gravity, so the
     * load — which settles a half-backlash off the motor on the gravity-loaded tooth face — comes
     * to rest on the stated target instead of the motor. Only the endpoint moves, so this closes
     * the steady-state half-lash sag the other tests tolerate, without touching the motion.
     *
     * <p>Run on both sides of vertical, since the resting face (and so the bias sign) flips with
     * the sign of the gravity torque: below vertical the load hangs under the motor; past vertical
     * it rests above it.
     */
    @Test
    fun restCompensationClosesTheGapToRigid() {
        // Below vertical (gravity voltage positive: bias up).
        val start = -PI / 4 // -45 deg
        val target = PI / 4 // +45 deg
        val rigidErrDeg = runRigid(start, target)
        val rawErrDeg = runBacklash(start, target, false)
        val compErrDeg = runBacklash(start, target, true)

        // Past vertical (gravity voltage negative: bias down).
        val startPast = Math.toRadians(150.0)
        val targetPast = Math.toRadians(120.0)
        val rawPastErrDeg = runBacklash(startPast, targetPast, false)
        val compPastErrDeg = runBacklash(startPast, targetPast, true)

        System.out.printf(
            "rest compensation: rigid=%.2f, raw=%.2f -> comp=%.2f deg; " +
                "past-vertical raw=%.2f -> comp=%.2f deg (half-backlash=%.2f deg)%n",
            rigidErrDeg,
            rawErrDeg,
            compErrDeg,
            rawPastErrDeg,
            compPastErrDeg,
            HALF_BACKLASH_DEG,
        )

        // Compensation removes the half-lash sag: steady error drops from ~half-backlash to near
        // the rigid plant's level (residual: contact-spring sag + encoder quantization + tracking).
        assertTrue(
            compErrDeg < 1.0,
            "compensated steady error should be near rigid, got $compErrDeg deg (rigid=$rigidErrDeg)",
        )
        assertTrue(
            compErrDeg < rawErrDeg - 1.5,
            "compensation should remove most of the half-lash sag (raw=$rawErrDeg, comp=$compErrDeg)",
        )
        assertTrue(
            compPastErrDeg < 1.0 && compPastErrDeg < rawPastErrDeg - 1.5,
            "past-vertical bias sign must flip with the resting face (raw=$rawPastErrDeg, comp=$compPastErrDeg)",
        )
    }

    /**
     * On the three-inertia flex plant, the tip rests off the motor by more than the half-lash:
     * gear-tooth penetration and flex-spring sag add an elastic droop *proportional to gravity*.
     * Half-lash compensation alone leaves that behind (observed live in ControlLab: stated target
     * parked halfway between motor and arm). The full rest model is `halfLash·sign(g) +
     * compliance·g`; with the plant's compliance supplied, the tip lands on the stated target. Note
     * the fix is *not* full-lash compensation — the droop scales with gravity while the lash term
     * does not, so a constant full-lash bias is wrong at most angles.
     */
    @Test
    fun restCompensationCoversElasticSagOnTheFlexPlant() {
        val start = -PI / 4 // -45 deg
        val target = PI / 4 // +45 deg

        val rawErrDeg = runFlex(start, target, false)
        val halfLashOnlyErrDeg = runFlex(start, target, true, 0.0)
        val fullErrDeg = runFlex(start, target, true, Double.NaN) // NaN = use plant's compliance

        // Expected droop at +45 deg: half-lash 2.5 + (1/500 + 0.85/kFlex)·kG·cos(45) ≈ 1.7 more.
        System.out.printf(
            "flex rest compensation: raw=%.2f, half-lash-only=%.2f, +compliance=%.2f deg " +
                "(half-backlash=%.2f deg)%n",
            rawErrDeg,
            halfLashOnlyErrDeg,
            fullErrDeg,
            HALF_BACKLASH_DEG,
        )

        assertTrue(
            rawErrDeg > HALF_BACKLASH_DEG + 1.0,
            "raw flex droop should exceed the half-lash alone, got $rawErrDeg deg",
        )
        assertTrue(
            halfLashOnlyErrDeg > 1.0 && halfLashOnlyErrDeg < rawErrDeg,
            "half-lash-only should leave the elastic sag behind (raw=$rawErrDeg, half-lash-only=$halfLashOnlyErrDeg)",
        )
        assertTrue(
            fullErrDeg < 1.0,
            "lash + compliance should land the tip on the stated target, got $fullErrDeg deg",
        )
    }

    private fun runFlex(start: Double, target: Double, compensate: Boolean): Double =
        runFlex(start, target, compensate, Double.NaN)

    /**
     * Run the move against the flex plant; return mean steady-state tip error in degrees.
     *
     * @param complianceRadPerVolt rest compliance handed to the controller when compensating; NaN
     *   means use the plant's own [FlexArmMotorSim.getRestComplianceRadPerVolt]
     */
    private fun runFlex(
        start: Double,
        target: Double,
        compensate: Boolean,
        complianceRadPerVolt: Double,
    ): Double {
        val plant =
            FlexArmMotorSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_REV,
                GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                start,
                BACKLASH_RAD,
                /* flexHz= */ 3.0,
                /* flexZeta= */ 0.03,
            )
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, start)
        if (compensate) {
            val compliance =
                if (complianceRadPerVolt.isNaN()) plant.getRestComplianceRadPerVolt()
                else complianceRadPerVolt
            controller.setBacklashCompensation(BACKLASH_RAD, TAPER_VOLTS, compliance)
        }

        val rng = Random(1L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20
        var sumErrDeg = 0.0
        var samples = 0

        for (ms in 0..4000) {
            plant.step(0.001, power, VOLTAGE)
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(
                    plant.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD,
                    plant.getVelocityTps() / TICKS_PER_RAD,
                )
                val voltage =
                    controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt)
                power = voltage / VOLTAGE
                if (ms / 1000.0 > 3.0) {
                    sumErrDeg += abs(Math.toDegrees(plant.getTruePositionRad() - target))
                    samples++
                }
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }
        return sumErrDeg / samples
    }

    /** Run the move against the rigid plant; return mean steady-state load error in degrees. */
    private fun runRigid(start: Double, target: Double): Double {
        val plant =
            ArmPlantSim(K_S, K_G, K_V, K_A, TICKS_PER_RAD, MIN_ANGLE_RAD, MAX_ANGLE_RAD, start)
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, start)

        val rng = Random(1L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20
        var sumErrDeg = 0.0
        var samples = 0

        for (ms in 0..3000) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(
                    plant.getEncoderPosition() / TICKS_PER_RAD,
                    plant.getEncoderVelocityTps() / TICKS_PER_RAD,
                )
                val voltage =
                    controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt)
                power = voltage / VOLTAGE
                if (ms / 1000.0 > 2.0) {
                    sumErrDeg += abs(Math.toDegrees(plant.getTrueAngleRad() - target))
                    samples++
                }
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }
        return sumErrDeg / samples
    }

    /** Run the same move against the backlash plant; return mean steady-state load error. */
    private fun runBacklash(start: Double, target: Double): Double =
        runBacklash(start, target, false)

    /**
     * Run the move against the backlash plant, optionally with rest-only backlash compensation
     * enabled on the controller; return mean steady-state load error in degrees.
     */
    private fun runBacklash(start: Double, target: Double, compensate: Boolean): Double {
        val plant = makePlant(start)
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, start)
        if (compensate) {
            controller.setBacklashCompensation(BACKLASH_RAD, TAPER_VOLTS)
        }

        val rng = Random(1L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20
        var sumErrDeg = 0.0
        var samples = 0

        for (ms in 0..3000) {
            plant.step(0.001, power, VOLTAGE)
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD)
                val voltage =
                    controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt)
                power = voltage / VOLTAGE
                if (ms / 1000.0 > 2.0) {
                    sumErrDeg += abs(Math.toDegrees(plant.getTruePositionRad() - target))
                    samples++
                }
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }
        return sumErrDeg / samples
    }

    /**
     * On a downward (gravity-aided) reversal the load presses on the braking face the whole way
     * down (see ArmBacklashKeepEngagedTest), so the arrival must be clean at any descent speed: the
     * stop is planned under the jerk limit from the profile's own state, so braking starts early
     * enough regardless of how fast the descent runs. Fast (8 rad/s) and velocity-capped (4 rad/s)
     * descents arrive alike — within a couple of degrees past the load's half-backlash resting
     * offset — and both settle without chattering down. A stopping law that ignores the
     * jerk-limited swing from full accelerate to full brake fails this test at the fast speed only,
     * which would make "slow down" masquerade as a backlash mitigation.
     */
    @Test
    fun descentSpeedDoesNotCostBottomOvershoot() {
        // default accel=12, jerk=480; only the velocity cap changes.
        val fast = runReversalMetrics(/* maxVel= */ 8.0, 12.0, 480.0)
        val capped = runReversalMetrics(/* maxVel= */ 4.0, 12.0, 480.0)
        System.out.printf(
            "reversal bounce: fast(8) overshoot=%.1f deg p2p=%.2f; " +
                "capped(4) overshoot=%.1f deg p2p=%.2f%n",
            fast[0],
            fast[2],
            capped[0],
            capped[2],
        )

        // Both speeds arrive within the half-backlash resting offset plus a small dynamic margin
        // (closed-loop lag past the profile; half-lash ≈ 2.5 deg, + ~3 deg dynamic).
        assertTrue(
            fast[0] < HALF_BACKLASH_DEG + 3.0,
            "fast reversal overshoot too large: ${fast[0]} deg",
        )
        assertTrue(
            capped[0] < HALF_BACKLASH_DEG + 3.0,
            "capped reversal overshoot too large: ${capped[0]} deg",
        )
        // And going fast costs no meaningful extra overshoot.
        assertTrue(
            fast[0] < capped[0] + 1.0,
            "descent speed should not cost overshoot (fast=${fast[0]}, capped=${capped[0]})",
        )
        // Both settle rather than bouncing all the way down.
        assertTrue(fast[2] < 2.0, "fast reversal did not settle: p2p=${fast[2]} deg")
        assertTrue(capped[2] < 2.0, "capped reversal did not settle: p2p=${capped[2]} deg")
    }

    /**
     * Run the up-then-down reversal and return
     * [bottomOvershootDeg, steadyVelZeroCrossings, steadyP2PDeg, finalDeg]. Steady window is the
     * last 1 s (3000-4000 ms).
     */
    private fun runReversalMetrics(maxVel: Double, maxAccel: Double, maxJerk: Double): DoubleArray =
        runReversalMetrics(maxVel, maxAccel, maxJerk, 1.5)

    private fun runReversalMetrics(
        maxVel: Double,
        maxAccel: Double,
        maxJerk: Double,
        kD: Double,
    ): DoubleArray {
        val start = -PI / 4
        val up = PI / 6
        val down = -PI / 3
        val plant = makePlant(start)
        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val ekf = MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start)
        val controller =
            MotorMechanismController(model, 40.0, 8.0, kD, maxVel, maxAccel, maxJerk, 1.5, start)

        val rng = Random(7L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var bottomOvershoot = 0.0 // deg past -60 reached during arrival
        var prevLoadVel = 0.0
        var steadyZeroCrossings = 0
        var steadyMin = Double.POSITIVE_INFINITY
        var steadyMax = Double.NEGATIVE_INFINITY

        for (ms in 0..4000) {
            val target = if (ms < 1800) up else down
            plant.step(0.001, power, VOLTAGE)
            val loadDeg = Math.toDegrees(plant.getTruePositionRad())
            val loadVel = plant.getTrueVelocityRadPerSec()

            if (ms >= 1800) bottomOvershoot = max(bottomOvershoot, -60.0 - loadDeg)
            if (ms >= 3000) {
                steadyMin = min(steadyMin, loadDeg)
                steadyMax = max(steadyMax, loadDeg)
                if (
                    sign(loadVel) != sign(prevLoadVel) &&
                        (abs(loadVel) > 0.05 || abs(prevLoadVel) > 0.05)
                ) {
                    steadyZeroCrossings++
                }
            }
            prevLoadVel = loadVel

            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                ekf.predict(dt, power, VOLTAGE)
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD)
                val voltage =
                    controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt)
                power = voltage / VOLTAGE
                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }
        return doubleArrayOf(
            bottomOvershoot,
            steadyZeroCrossings.toDouble(),
            steadyMax - steadyMin,
            Math.toDegrees(plant.getTruePositionRad()),
        )
    }

    private fun makePlant(initialAngleRad: Double): BacklashArmMotorSim =
        BacklashArmMotorSim(
            K_S,
            K_G,
            K_V,
            K_A,
            TICKS_PER_REV,
            GEAR_RATIO,
            ENCODER_ZERO_OFFSET_RAD,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            initialAngleRad,
            BACKLASH_RAD,
        )

    companion object {
        // Same gain family as the rigid-plant ArmControllerTest, so this is an apples-to-apples
        // comparison with backlash as the only added effect.
        private const val TICKS_PER_REV = 28
        private const val GEAR_RATIO = 100.0
        private val TICKS_PER_RAD = TICKS_PER_REV * GEAR_RATIO / (2 * PI)
        private const val K_S = 0.1
        private const val K_G = 1.5
        private const val K_V = 2.0
        private const val K_A = 0.2
        private const val VOLTAGE = 12.0

        // Encoder reads 0 at horizontal here (zero offset 0), so angle = ticks / TICKS_PER_RAD.
        private const val ENCODER_ZERO_OFFSET_RAD = 0.0
        private val MIN_ANGLE_RAD = -PI * 0.9
        private val MAX_ANGLE_RAD = PI * 0.9
        private val BACKLASH_RAD = Math.toRadians(5.0) // half-backlash = 2.5 deg
        private val HALF_BACKLASH_DEG = Math.toDegrees(BACKLASH_RAD) / 2.0
        // Gravity hold-voltage below which the rest compensation tapers off (~ kG at 11 deg from
        // vertical): too little gravity to pin the load onto one tooth face.
        private const val TAPER_VOLTS = 0.3

        /** Motor-side encoder angle in radians — what the controller and filter actually see. */
        private fun measuredAngleRad(plant: BacklashArmMotorSim): Double =
            plant.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD
    }
}
