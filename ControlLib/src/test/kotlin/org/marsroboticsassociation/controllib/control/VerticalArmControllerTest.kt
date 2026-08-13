package org.marsroboticsassociation.controllib.control

import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.hardware.IMotor
import org.marsroboticsassociation.controllib.sim.ArmMotorSim
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim

class VerticalArmControllerTest {

    // ── constants ───────────────────────────────────────────────────────────────

    companion object {
        const val HUB_VOLTAGE = 13.75
        const val SEED = 42L

        const val KS = 0.3
        const val KG = 1.5
        const val KV = 1.2
        const val KA = 0.15

        const val TICKS_PER_REV = 28
        const val GEAR_RATIO = 100.0

        val ENCODER_ZERO_OFFSET_RAD = -PI / 4
        val MIN_ANGLE_RAD = -PI * 5 / 4
        val MAX_ANGLE_RAD = -PI / 4
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
        VerticalArmController.PARAMS = VerticalArmController.Params()
        VerticalArmController.PARAMS.ks = KS
        VerticalArmController.PARAMS.kg = KG
        VerticalArmController.PARAMS.kv = KV
        VerticalArmController.PARAMS.ka = KA
        VerticalArmController.PARAMS.ticksPerRev = TICKS_PER_REV
        VerticalArmController.PARAMS.gearRatio = GEAR_RATIO
        VerticalArmController.PARAMS.encoderZeroOffsetRad = ENCODER_ZERO_OFFSET_RAD
        VerticalArmController.PARAMS.minAngleRad = MIN_ANGLE_RAD
        VerticalArmController.PARAMS.maxAngleRad = MAX_ANGLE_RAD

        // LQR cost weights — targeting K ≈ [15, 1] to match existing PD behavior
        VerticalArmController.PARAMS.qPosition = 0.5
        VerticalArmController.PARAMS.qVelocity = 5.0
        VerticalArmController.PARAMS.rVoltage = 12.0

        // Trajectory limits
        VerticalArmController.PARAMS.maxVelRad = 3.0
        VerticalArmController.PARAMS.maxAccelRad = 6.0
        VerticalArmController.PARAMS.maxDecelRad = 8.0
        VerticalArmController.PARAMS.maxJerkRad = 30.0

        // Kalman tuning
        VerticalArmController.PARAMS.modelStdDevPos = 0.01
        VerticalArmController.PARAMS.modelStdDevVel = 3.0
        VerticalArmController.PARAMS.measurementStdDevPos = 0.05
        VerticalArmController.PARAMS.measurementStdDevVel = 0.5

        VerticalArmController.PARAMS.latencyCompensationSec = 0.030
        VerticalArmController.PARAMS.replanThresholdRad = Math.toRadians(15.0)
        VerticalArmController.PARAMS.coastZoneRad = Math.toRadians(10.0)

        simTimeNanos = 0
    }

    private fun step(
        controller: VerticalArmController,
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
        val startAngle = MAX_ANGLE_RAD
        val targetAngle = Math.toRadians(-120.0)

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

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
        val holdAngle = Math.toRadians(-60.0)

        val sim = makeSim(holdAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(holdAngle)

        val rng = Random(SEED)
        for (i in 0 until 300) {
            step(controller, adapter, sim, rng)
        }

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
        val startAngle = MAX_ANGLE_RAD

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(startAngle)

        val rng = Random(SEED)
        for (i in 0 until 5) {
            step(controller, adapter, sim, rng)
        }

        assertEquals(
            VerticalArmController.Mode.COASTING,
            controller.mode,
            "should be coasting near hard stop",
        )
        assertEquals(0.0, adapter.lastPower, 0.001, "power should be 0 when coasting")
    }

    @Test
    fun testWakeFromHardStop() {
        val startAngle = MAX_ANGLE_RAD
        val targetAngle = Math.toRadians(-120.0)

        val sim = makeSim(startAngle)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(startAngle)
        val rng = Random(SEED)
        for (i in 0 until 20) {
            step(controller, adapter, sim, rng)
        }
        assertEquals(VerticalArmController.Mode.COASTING, controller.mode)

        controller.setTarget(targetAngle)
        assertEquals(
            VerticalArmController.Mode.TRACKING,
            controller.mode,
            "should switch to TRACKING after setTarget",
        )

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
        val targetAngle = Math.toRadians(-120.0)

        val sim = makeSim(MAX_ANGLE_RAD)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        controller.setTarget(targetAngle)

        val rng = Random(SEED)
        for (i in 0 until 500) {
            step(controller, adapter, sim, rng)
        }
        assertEquals(
            targetAngle,
            sim.getTruePositionRad(),
            Math.toRadians(5.0),
            "should be near target before disturbance",
        )

        sim.setDisturbanceVoltage(-3.0)
        for (i in 0 until 30) {
            step(controller, adapter, sim, rng)
        }
        sim.setDisturbanceVoltage(0.0)

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
    fun testVoltageCompensation() {
        val startAngle = MAX_ANGLE_RAD
        val targetAngle = Math.toRadians(-120.0)

        // --- Run at 13.75V ---
        val sim1 = makeSim(startAngle)
        val adapter1 = SimMotorAdapter(sim1)
        simTimeNanos = 0
        val ctrl1 = VerticalArmController(adapter1, { _, _, _ -> }, this::clockSupplier)
        ctrl1.setTarget(targetAngle)

        val rng1 = Random(SEED)
        for (i in 0 until 500) {
            val dt = maxOf(0.005, 0.016 + rng1.nextGaussian() * 0.005)
            simTimeNanos += (dt * 1e9).toLong()
            ctrl1.update(dt, 13.75)
            sim1.step(dt, adapter1.lastPower, 13.75)
        }

        // --- Run at 10.5V ---
        val sim2 = makeSim(startAngle)
        val adapter2 = SimMotorAdapter(sim2)
        simTimeNanos = 0
        val ctrl2 = VerticalArmController(adapter2, { _, _, _ -> }, this::clockSupplier)
        ctrl2.setTarget(targetAngle)

        val rng2 = Random(SEED)
        for (i in 0 until 500) {
            val dt = maxOf(0.005, 0.016 + rng2.nextGaussian() * 0.005)
            simTimeNanos += (dt * 1e9).toLong()
            ctrl2.update(dt, 10.5)
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

    // ── worstCaseAngle tests ────────────────────────────────────────────────────

    @Test
    fun testWorstCaseAngle_rangeNotCrossingHorizontal() {
        val result =
            VerticalArmController.worstCaseAngle(
                Math.toRadians(-60.0),
                Math.toRadians(-120.0),
            )
        val absCosResult = abs(cos(result))
        assertEquals(0.5, absCosResult, 0.01, "should pick an endpoint with |cos|=0.5")
    }

    @Test
    fun testWorstCaseAngle_rangeCrossingHorizontal() {
        val result =
            VerticalArmController.worstCaseAngle(Math.toRadians(-30.0), Math.toRadians(30.0))
        assertEquals(0.0, result, 0.01, "should pick horizontal crossing at 0")
    }

    @Test
    fun testWorstCaseAngle_rangeCrossingNegativePi() {
        val result =
            VerticalArmController.worstCaseAngle(
                Math.toRadians(-150.0),
                Math.toRadians(-210.0),
            )
        assertEquals(-PI, result, 0.01, "should pick horizontal crossing at -pi")
    }

    @Test
    fun testWorstCaseAngle_sameAngle() {
        val angle = Math.toRadians(-90.0)
        val result = VerticalArmController.worstCaseAngle(angle, angle)
        assertEquals(angle, result, 0.01, "should return the single angle")
    }

    @Test
    fun testComputeMoveLimits_limitsVaryWithSweep() {
        val sim = makeSim(MAX_ANGLE_RAD)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        val limitsVertical =
            controller.computeMoveLimits(
                Math.toRadians(-85.0),
                Math.toRadians(-95.0),
                HUB_VOLTAGE,
            )

        val limitsHorizontal =
            controller.computeMoveLimits(
                Math.toRadians(-10.0),
                Math.toRadians(10.0),
                HUB_VOLTAGE,
            )

        assertTrue(
            limitsVertical[1] >= limitsHorizontal[1],
            "accel limit near vertical should be >= near horizontal",
        )
        assertTrue(
            limitsVertical[0] >= limitsHorizontal[0],
            "velocity limit near vertical should be >= near horizontal",
        )
    }

    @Test
    fun testComputeMoveLimits_cappedAtParamsMaximums() {
        val sim = makeSim(MAX_ANGLE_RAD)
        val adapter = SimMotorAdapter(sim)
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)

        val limits =
            controller.computeMoveLimits(
                Math.toRadians(-85.0),
                Math.toRadians(-95.0),
                HUB_VOLTAGE,
            )

        assertTrue(
            limits[0] <= VerticalArmController.PARAMS.maxVelRad,
            "velocity should be capped at PARAMS.maxVelRad",
        )
        assertTrue(
            limits[1] <= VerticalArmController.PARAMS.maxAccelRad,
            "accel should be capped at PARAMS.maxAccelRad",
        )
        assertTrue(
            limits[2] <= VerticalArmController.PARAMS.maxDecelRad,
            "decel should be capped at PARAMS.maxDecelRad",
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
        VerticalArmController.PARAMS.backlashRad = compensationBacklashRad

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
        val controller = VerticalArmController(adapter, { _, _, _ -> }, this::clockSupplier)
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
        VerticalArmController.PARAMS.backlashRad = Math.toRadians(5.0)
        VerticalArmController.PARAMS.restComplianceRadPerVolt = 0.02
        val target = Math.toRadians(-60.0)
        val gravityVolts = KG * cos(target) // 0.75 V, above the 0.3 V taper

        assertEquals(
            Math.toRadians(2.5) + 0.02 * gravityVolts,
            VerticalArmController.backlashBias(target),
            1e-9,
            "bias = half-lash + compliance * gravity",
        )

        // Compliance alone still biases (a stiff-lash-free but flexy arm), and it carries
        // gravity's sign past vertical with no taper needed.
        VerticalArmController.PARAMS.backlashRad = 0.0
        assertEquals(0.02 * gravityVolts, VerticalArmController.backlashBias(target), 1e-9)
        val pastVertical = Math.toRadians(-120.0)
        assertEquals(
            0.02 * KG * cos(pastVertical),
            VerticalArmController.backlashBias(pastVertical),
            1e-9,
            "compliance term flips sign with gravity",
        )
    }
}
