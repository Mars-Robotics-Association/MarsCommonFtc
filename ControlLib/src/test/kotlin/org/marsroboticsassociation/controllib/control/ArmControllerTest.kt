package org.marsroboticsassociation.controllib.control

import java.util.Random
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertNotEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.sim.ArmMotorSim
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim

class ArmControllerTest {

    // ── constants ───────────────────────────────────────────────────────────────

    companion object {
        const val HUB_VOLTAGE = 13.75
        const val SEED = 42L

        // Feedforward gains (output-shaft rad units)
        const val KS = 0.3
        const val KG = 1.5
        const val KV = 1.2
        const val KA = 0.15

        const val TICKS_PER_REV = 28
        const val GEAR_RATIO = 100.0

        // Arm geometry: front hard stop at -45 deg, back hard stop at -225 deg
        // Encoder reads 0 at the front hard stop.
        val ENCODER_ZERO_OFFSET_RAD = -Math.PI / 4
        val MIN_ANGLE_RAD = -Math.PI * 5 / 4 // -225 deg (back stop)
        val MAX_ANGLE_RAD = -Math.PI / 4 // -45 deg  (front stop)
    }

    // ── sim clock ───────────────────────────────────────────────────────────────

    private var simTimeNanos = 0L

    private fun clockSupplier(): Long = simTimeNanos

    // ── fixture ─────────────────────────────────────────────────────────────────

    class SimMotorAdapter(val sim: ArmMotorSim) : IMotor {
        var lastPower = 0.0

        override val name: String
            get() = "arm"

        override val position: Int
            get() = sim.getPositionTicks()

        override val velocity: Double
            get() = sim.getVelocityTps()

        override fun setPower(power: Double) {
            lastPower = power
        }

        override val hubVoltage: Double
            get() = HUB_VOLTAGE

        override fun setVelocity(tps: Double) {}

        override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {}
    }

    private fun makeSim(initialAngleRad: Double): ArmMotorSim =
        ArmMotorSim(
            KS,
            KG,
            KV,
            KA,
            TICKS_PER_REV,
            GEAR_RATIO,
            ENCODER_ZERO_OFFSET_RAD,
            MIN_ANGLE_RAD,
            MAX_ANGLE_RAD,
            initialAngleRad,
        )

    @BeforeEach
    fun setupParams() {
        ArmController.PARAMS = ArmController.Params()
        ArmController.PARAMS.ks = KS
        ArmController.PARAMS.kg = KG
        ArmController.PARAMS.kv = KV
        ArmController.PARAMS.ka = KA
        ArmController.PARAMS.ticksPerRev = TICKS_PER_REV
        ArmController.PARAMS.gearRatio = GEAR_RATIO
        ArmController.PARAMS.encoderZeroOffsetRad = ENCODER_ZERO_OFFSET_RAD
        ArmController.PARAMS.minAngleRad = MIN_ANGLE_RAD
        ArmController.PARAMS.maxAngleRad = MAX_ANGLE_RAD

        // PD gains
        ArmController.PARAMS.kP = 15.0
        ArmController.PARAMS.kD = 1.0

        // Trajectory limits
        ArmController.PARAMS.maxVelRad = 3.0
        ArmController.PARAMS.maxAccelRad = 6.0
        ArmController.PARAMS.maxDecelRad = 8.0
        ArmController.PARAMS.maxJerkRad = 30.0

        // Kalman tuning
        ArmController.PARAMS.modelStdDevPos = 0.01
        ArmController.PARAMS.modelStdDevVel = 3.0
        ArmController.PARAMS.measurementStdDevPos = 0.05
        ArmController.PARAMS.measurementStdDevVel = 0.5

        ArmController.PARAMS.latencyCompensationSec = 0.030
        ArmController.PARAMS.replanThresholdRad = Math.toRadians(15.0)
        ArmController.PARAMS.coastZoneRad = Math.toRadians(10.0)

        simTimeNanos = 0
    }

    /** Advance by a normally-distributed dt (mean 16 ms, sigma 5 ms). Returns actual dt. */
    private fun step(
        controller: ArmController,
        adapter: SimMotorAdapter,
        sim: ArmMotorSim,
        rng: Random,
    ): Double {
        val dt = maxOf(0.005, 0.016 + rng.nextGaussian() * 0.005)
        simTimeNanos += (dt * 1e9).toLong()
        controller.update(dt, HUB_VOLTAGE)
        sim.step(dt, adapter.lastPower, HUB_VOLTAGE)
        return dt
    }

    // ── tests ───────────────────────────────────────────────────────────────────

    @Test
    fun testMoveToTarget() {
        // Start at front hard stop (-45 deg), move to -120 deg (past straight down)
        val startAngle = MAX_ANGLE_RAD // -45 deg
        val targetAngle = Math.toRadians(-120.0) // -120 deg

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(targetAngle)

        val rng = Random(SEED)
        var elapsed = 0.0
        for (i in 0 until 500) {
            elapsed += step(controller, adapter, sim, rng)
        }

        val finalPos = sim.getTruePositionRad()
        System.out.printf(
            "testMoveToTarget: target=%.1f deg, actual=%.1f deg, elapsed=%.2f s%n",
            Math.toDegrees(targetAngle),
            Math.toDegrees(finalPos),
            elapsed,
        )

        assertEquals(
            targetAngle,
            finalPos,
            Math.toRadians(3.0),
            "arm should converge to target within 3 degrees",
        )
    }

    @Test
    fun testHoldPositionAgainstGravity() {
        // Start at -60 deg (significant gravity torque), hold position
        val holdAngle = Math.toRadians(-60.0)

        val sim = makeSim(holdAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(holdAngle)

        val rng = Random(SEED)
        // Let it settle
        for (i in 0 until 300) {
            step(controller, adapter, sim, rng)
        }

        // Check steady-state error over next 200 iterations
        var maxError = 0.0
        for (i in 0 until 200) {
            step(controller, adapter, sim, rng)
            maxError = maxOf(maxError, abs(sim.getTruePositionRad() - holdAngle))
        }

        System.out.printf("testHoldPosition: max error=%.2f deg%n", Math.toDegrees(maxError))
        assertTrue(
            maxError < Math.toRadians(5.0),
            "steady-state position error should be under 5 degrees",
        )
    }

    @Test
    fun testCoastNearHardStop() {
        // Start near front hard stop, target is the hard stop
        val startAngle = MAX_ANGLE_RAD

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(startAngle)

        // Run a few iterations — arm should coast immediately since both target and
        // position are within the coast zone of the front hard stop.
        val rng = Random(SEED)
        for (i in 0 until 5) {
            step(controller, adapter, sim, rng)
        }

        assertEquals(
            ArmController.Mode.COASTING,
            controller.getMode(),
            "should be coasting near hard stop",
        )
        assertEquals(0.0, adapter.lastPower, 0.001, "power should be 0 when coasting")
    }

    @Test
    fun testWakeFromHardStop() {
        // Start at front hard stop, coasting, then wake and move to -120 deg
        val startAngle = MAX_ANGLE_RAD // -45 deg
        val targetAngle = Math.toRadians(-120.0) // -120 deg

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        // Let it enter coast
        controller.setTarget(startAngle)
        val rng = Random(SEED)
        for (i in 0 until 20) {
            step(controller, adapter, sim, rng)
        }
        assertEquals(ArmController.Mode.COASTING, controller.getMode())

        // Now wake it up
        controller.setTarget(targetAngle)
        assertEquals(
            ArmController.Mode.TRACKING,
            controller.getMode(),
            "should switch to TRACKING after setTarget",
        )

        // Let it move
        for (i in 0 until 500) {
            step(controller, adapter, sim, rng)
        }

        val finalPos = sim.getTruePositionRad()
        System.out.printf(
            "testWakeFromHardStop: target=%.1f deg, actual=%.1f deg%n",
            Math.toDegrees(targetAngle),
            Math.toDegrees(finalPos),
        )

        assertEquals(
            targetAngle,
            finalPos,
            Math.toRadians(5.0),
            "arm should reach target after waking from hard stop",
        )
    }

    @Test
    fun testReplanOnDisturbance() {
        // Move to -120 deg, then apply a large disturbance
        val targetAngle = Math.toRadians(-120.0)

        val sim = makeSim(MAX_ANGLE_RAD)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(targetAngle)

        val rng = Random(SEED)
        // Let it converge
        for (i in 0 until 500) {
            step(controller, adapter, sim, rng)
        }
        assertEquals(
            targetAngle,
            sim.getTruePositionRad(),
            Math.toRadians(5.0),
            "should be near target before disturbance",
        )

        // Apply large disturbance (equivalent to someone pushing the arm)
        sim.setDisturbanceVoltage(-3.0)
        for (i in 0 until 30) {
            step(controller, adapter, sim, rng)
        }
        sim.setDisturbanceVoltage(0.0)

        // Let it recover
        for (i in 0 until 500) {
            step(controller, adapter, sim, rng)
        }

        val finalPos = sim.getTruePositionRad()
        System.out.printf(
            "testReplanOnDisturbance: target=%.1f deg, actual=%.1f deg%n",
            Math.toDegrees(targetAngle),
            Math.toDegrees(finalPos),
        )

        assertEquals(
            targetAngle,
            finalPos,
            Math.toRadians(5.0),
            "arm should recover to target after disturbance",
        )
    }

    @Test
    fun testAsymmetricProfile() {
        // Move a large distance to exercise both accel and decel phases
        val startAngle = MAX_ANGLE_RAD // -45 deg
        val targetAngle = Math.toRadians(-150.0) // -150 deg

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(targetAngle)

        val rng = Random(SEED)
        var maxAccelSeen = 0.0
        var maxDecelSeen = 0.0
        var prevVel = 0.0

        for (i in 0 until 500) {
            val dt = step(controller, adapter, sim, rng)
            val vel = controller.getTrajectoryVelocityRadPerSec()
            if (i > 0) {
                val accel = (vel - prevVel) / dt

                if (abs(accel) > 0.5) {
                    if (sign(accel) == sign(vel) || abs(vel) < 0.01) {
                        maxAccelSeen = maxOf(maxAccelSeen, abs(accel))
                    } else {
                        maxDecelSeen = maxOf(maxDecelSeen, abs(accel))
                    }
                }
            }
            prevVel = vel
        }

        System.out.printf(
            "testAsymmetricProfile: maxAccel=%.1f, maxDecel=%.1f (limits: %.1f, %.1f)%n",
            maxAccelSeen,
            maxDecelSeen,
            ArmController.PARAMS.maxAccelRad,
            ArmController.PARAMS.maxDecelRad,
        )

        // The accel and decel should be bounded by their respective limits (with some jerk margin)
        assertTrue(
            maxAccelSeen <= ArmController.PARAMS.maxAccelRad + 1.0,
            "acceleration should not exceed maxAccelRad (plus jerk margin)",
        )
        assertNotEquals(
            ArmController.PARAMS.maxAccelRad,
            ArmController.PARAMS.maxDecelRad,
            "test setup should have asymmetric limits",
        )
    }

    @Test
    fun testWorstCaseAngle_rangeNotCrossingHorizontal() {
        val result = ArmController.worstCaseAngle(Math.toRadians(-60.0), Math.toRadians(-120.0))
        val absCosResult = abs(cos(result))
        assertEquals(0.5, absCosResult, 0.01, "should pick an endpoint with |cos|=0.5")
    }

    @Test
    fun testWorstCaseAngle_rangeCrossingHorizontal() {
        val result = ArmController.worstCaseAngle(Math.toRadians(-30.0), Math.toRadians(30.0))
        assertEquals(0.0, result, 0.01, "should pick horizontal crossing at 0")
    }

    @Test
    fun testWorstCaseAngle_rangeCrossingNegativePi() {
        val result = ArmController.worstCaseAngle(Math.toRadians(-150.0), Math.toRadians(-210.0))
        assertEquals(-Math.PI, result, 0.01, "should pick horizontal crossing at -pi")
    }

    @Test
    fun testWorstCaseAngle_sameAngle() {
        val angle = Math.toRadians(-90.0)
        val result = ArmController.worstCaseAngle(angle, angle)
        assertEquals(angle, result, 0.01, "should return the single angle")
    }

    @Test
    fun testComputeMoveLimits_limitsVaryWithSweep() {
        // Create a controller with known gains
        val sim = makeSim(MAX_ANGLE_RAD)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        // Move near vertical (-90 deg): gravity torque is 0, max torque available
        val limitsVertical =
            controller.computeMoveLimits(
                Math.toRadians(-85.0),
                Math.toRadians(-95.0),
                HUB_VOLTAGE,
            )

        // Move crossing horizontal (0 deg): gravity torque is maximal, less torque available
        val limitsHorizontal =
            controller.computeMoveLimits(
                Math.toRadians(-10.0),
                Math.toRadians(10.0),
                HUB_VOLTAGE,
            )

        // Acceleration limit near vertical should be >= near horizontal
        assertTrue(
            limitsVertical[1] >= limitsHorizontal[1],
            "accel limit near vertical should be >= near horizontal",
        )

        // Velocity limit near vertical should be >= near horizontal
        assertTrue(
            limitsVertical[0] >= limitsHorizontal[0],
            "velocity limit near vertical should be >= near horizontal",
        )
    }

    @Test
    fun testComputeMoveLimits_cappedAtParamsMaximums() {
        val sim = makeSim(MAX_ANGLE_RAD)
        val adapter = SimMotorAdapter(sim)
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        // Move near vertical where motor has lots of headroom
        val limits =
            controller.computeMoveLimits(
                Math.toRadians(-85.0),
                Math.toRadians(-95.0),
                HUB_VOLTAGE,
            )

        assertTrue(
            limits[0] <= ArmController.PARAMS.maxVelRad,
            "velocity should be capped at PARAMS.maxVelRad",
        )
        assertTrue(
            limits[1] <= ArmController.PARAMS.maxAccelRad,
            "accel should be capped at PARAMS.maxAccelRad",
        )
        assertTrue(
            limits[2] <= ArmController.PARAMS.maxDecelRad,
            "decel should be capped at PARAMS.maxDecelRad",
        )
    }

    @Test
    fun testSetTarget_updatesTrajectoryLimits() {
        // Verify that setTarget calls computeMoveLimits by checking that a move
        // crossing horizontal uses different (lower) limits than a move near vertical.
        // We test this indirectly: a move near horizontal should take longer than
        // a move near vertical of the same angular distance, because the limits are lower.

        // Move near vertical: -80 to -100 deg (low gravity torque, high limits)
        val sim1 = makeSim(Math.toRadians(-80.0))
        val adapter1 = SimMotorAdapter(sim1)
        simTimeNanos = 0
        val ctrl1 = ArmController(adapter1, { _, _, _ -> }, this::clockSupplier)
        ctrl1.setTarget(Math.toRadians(-100.0))

        val rng1 = Random(SEED)
        var steps1 = 0
        for (i in 0 until 500) {
            step(ctrl1, adapter1, sim1, rng1)
            steps1++
            if (ctrl1.isAtTarget()) break
        }

        // Move crossing horizontal: +10 to -10 deg (high gravity torque, lower limits)
        ArmController.PARAMS.maxAngleRad = Math.toRadians(15.0) // widen range for this test
        val sim2 = makeSim(Math.toRadians(10.0))
        val adapter2 = SimMotorAdapter(sim2)
        simTimeNanos = 0
        val ctrl2 = ArmController(adapter2, { _, _, _ -> }, this::clockSupplier)
        ctrl2.setTarget(Math.toRadians(-10.0))

        val rng2 = Random(SEED)
        var steps2 = 0
        for (i in 0 until 500) {
            step(ctrl2, adapter2, sim2, rng2)
            steps2++
            if (ctrl2.isAtTarget()) break
        }

        System.out.printf(
            "testSetTarget_updatesTrajectoryLimits: vertical steps=%d, horizontal steps=%d%n",
            steps1,
            steps2,
        )

        // The horizontal move should take at least as many steps (lower limits = slower)
        assertTrue(
            steps2 >= steps1,
            "move crossing horizontal should take at least as long as move near vertical",
        )
    }

    @Test
    fun testVoltageCompensation() {
        // Run the same move at two different voltages, verify similar tracking
        val startAngle = MAX_ANGLE_RAD
        val targetAngle = Math.toRadians(-120.0)

        // --- Run at 13.75V ---
        val sim1 = makeSim(startAngle)
        val adapter1 = SimMotorAdapter(sim1)
        simTimeNanos = 0
        val controller1 = ArmController(adapter1, { _, _, _ -> }, this::clockSupplier)
        controller1.setTarget(targetAngle)

        val rng1 = Random(SEED)
        for (i in 0 until 500) {
            val dt = maxOf(0.005, 0.016 + rng1.nextGaussian() * 0.005)
            simTimeNanos += (dt * 1e9).toLong()
            controller1.update(dt, 13.75)
            sim1.step(dt, adapter1.lastPower, 13.75)
        }

        // --- Run at 10.5V ---
        val sim2 = makeSim(startAngle)
        val adapter2 = SimMotorAdapter(sim2)
        simTimeNanos = 0
        val controller2 = ArmController(adapter2, { _, _, _ -> }, this::clockSupplier)
        controller2.setTarget(targetAngle)

        val rng2 = Random(SEED)
        for (i in 0 until 500) {
            val dt = maxOf(0.005, 0.016 + rng2.nextGaussian() * 0.005)
            simTimeNanos += (dt * 1e9).toLong()
            controller2.update(dt, 10.5)
            sim2.step(dt, adapter2.lastPower, 10.5)
        }

        val pos1 = sim1.getTruePositionRad()
        val pos2 = sim2.getTruePositionRad()
        System.out.printf(
            "testVoltageCompensation: 13.75V final=%.1f deg, 10.5V final=%.1f deg%n",
            Math.toDegrees(pos1),
            Math.toDegrees(pos2),
        )

        assertEquals(
            targetAngle,
            pos1,
            Math.toRadians(5.0),
            "13.75V run should converge to target",
        )
        assertEquals(
            targetAngle,
            pos2,
            Math.toRadians(5.0),
            "10.5V run should converge to target",
        )
    }

    // ── backlash rest compensation ──────────────────────────────────────────────

    class BacklashSimMotorAdapter(val sim: BacklashArmMotorSim) : IMotor {
        var lastPower = 0.0

        override val name: String
            get() = "arm"

        override val position: Int
            get() = sim.getPositionTicks()

        override val velocity: Double
            get() = sim.getVelocityTps()

        override fun setPower(power: Double) {
            lastPower = power
        }

        override val hubVoltage: Double
            get() = HUB_VOLTAGE

        override fun setVelocity(tps: Double) {}

        override fun setVelocityPIDFCoefficients(p: Double, i: Double, d: Double, f: Double) {}
    }

    /**
     * Drive the -45 -> -120 deg move onto the two-mass backlash plant with the given
     * PARAMS.backlashRad compensation; return the mean steady-state load error in degrees.
     */
    private fun runBacklashMove(compensationBacklashRad: Double): Double {
        val plantBacklashRad = Math.toRadians(5.0)
        val targetAngle = Math.toRadians(-120.0)
        ArmController.PARAMS.backlashRad = compensationBacklashRad

        val sim =
            BacklashArmMotorSim(
                KS,
                KG,
                KV,
                KA,
                TICKS_PER_REV,
                GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD,
                MAX_ANGLE_RAD,
                MAX_ANGLE_RAD,
                plantBacklashRad,
            )
        val adapter = BacklashSimMotorAdapter(sim)
        simTimeNanos = 0
        val controller = ArmController(adapter, { _, _, _ -> }, this::clockSupplier)
        controller.setTarget(targetAngle)

        val rng = Random(SEED)
        var sumErrDeg = 0.0
        var samples = 0
        for (i in 0 until 500) {
            val dt = maxOf(0.005, 0.016 + rng.nextGaussian() * 0.005)
            simTimeNanos += (dt * 1e9).toLong()
            controller.update(dt, HUB_VOLTAGE)
            sim.step(dt, adapter.lastPower, HUB_VOLTAGE)
            if (i >= 350) {
                sumErrDeg += abs(Math.toDegrees(sim.getTruePositionRad() - targetAngle))
                samples++
            }
        }
        return sumErrDeg / samples
    }

    /**
     * Rest-only backlash compensation: with PARAMS.backlashRad set, the target is biased a
     * half-backlash against gravity so the load — which settles a half-backlash off the motor on
     * the gravity-loaded tooth face — comes to rest at the stated angle. At -120 deg the load rests
     * <em>above</em> the motor (gravity pulls it back toward vertical), so the bias is negative;
     * uncompensated, the load parks ~2.5 deg shy of the target.
     */
    @Test
    fun testBacklashRestCompensation() {
        val rawErrDeg = runBacklashMove(0.0)
        val compErrDeg = runBacklashMove(Math.toRadians(5.0))
        System.out.printf(
            "testBacklashRestCompensation: raw=%.2f deg, compensated=%.2f deg " +
                "(half-backlash=2.50 deg)%n",
            rawErrDeg,
            compErrDeg,
        )

        assertTrue(
            rawErrDeg > 1.5,
            "uncompensated run should show the half-lash sag, got $rawErrDeg deg",
        )
        assertTrue(
            compErrDeg < 1.2,
            "compensated load should rest near the stated target, got $compErrDeg deg",
        )
        assertTrue(
            compErrDeg < rawErrDeg - 1.0,
            "compensation should remove most of the sag (raw=$rawErrDeg" + ", comp=$compErrDeg)",
        )
    }

    /**
     * The rest bias is half-lash (signed by gravity, tapered at the crest) plus a
     * gravity-proportional compliance term for elastic droop (gear-tooth penetration, arm flex).
     */
    @Test
    fun testBacklashBiasComplianceTerm() {
        ArmController.PARAMS.backlashRad = Math.toRadians(5.0)
        ArmController.PARAMS.restComplianceRadPerVolt = 0.02
        val target = Math.toRadians(-60.0)
        val gravityVolts = KG * cos(target) // 0.75 V, above the 0.3 V taper

        assertEquals(
            Math.toRadians(2.5) + 0.02 * gravityVolts,
            ArmController.backlashBias(target),
            1e-9,
            "bias = half-lash + compliance * gravity",
        )

        // Compliance alone still biases (a stiff-lash-free but flexy arm), and it carries
        // gravity's sign past vertical with no taper needed.
        ArmController.PARAMS.backlashRad = 0.0
        assertEquals(0.02 * gravityVolts, ArmController.backlashBias(target), 1e-9)
        val pastVertical = Math.toRadians(-120.0)
        assertEquals(
            0.02 * KG * cos(pastVertical),
            ArmController.backlashBias(pastVertical),
            1e-9,
            "compliance term flips sign with gravity",
        )
    }
}
