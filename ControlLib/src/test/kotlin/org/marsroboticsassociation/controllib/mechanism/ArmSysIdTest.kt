package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.PI
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.parallel.Execution
import org.junit.jupiter.api.parallel.ExecutionMode
import org.marsroboticsassociation.controllib.sim.ArmMotorSim
import org.marsroboticsassociation.controllib.sim.EncoderSim

/**
 * ControlLib arm sysid must recover rigid-plant feedforward coefficients from encoder logs alone,
 * using the same integrated-EOM battery as ControlLab.
 */
@Execution(ExecutionMode.SAME_THREAD)
class ArmSysIdTest {

    @Test
    fun recoversDefaultHeavyArmParams() {
        val r = runBattery(KS, KG, KV, KA)
        System.out.printf(
            "sysid: kS=%.3f kV=%.3f kA=%.3f kCos=%.3f kSin=%.3f  R2=%.4f  n=%d%n",
            r.kS,
            r.kV,
            r.kA,
            r.kCos,
            r.kSin,
            r.rSquared,
            r.samples,
        )

        assertTrue(r.rSquared > 0.99, "fit should be excellent, R2=${r.rSquared}")
        assertEquals(KA, r.kA, KA * 0.15, "kA within 15%")
        assertEquals(KG, r.kCos, KG * 0.10, "kCos (gravity) within 10%")
        assertEquals(KV, r.kV, KV * 0.15, "kV within 15%")
        assertEquals(KS, r.kS, 0.20, "kS within 0.2 V")
        assertEquals(0.0, r.kSin, 0.25, "kSin ~ 0 (plant gravity is pure cosine)")
        assertEquals(KG, r.kG(), KG * 0.10, "kG() matches hypot(kCos,kSin)")
        assertEquals(0.0, r.phiRad(), 0.1, "phi ~ 0 when pure cosine")
    }

    @Test
    fun recoversAModifiedInertia() {
        val kA = 0.6
        val kG = 2.5
        val r = runBattery(KS, kG, KV, kA)
        System.out.printf(
            "sysid(kA=0.6,kG=2.5): kA=%.3f kCos=%.3f R2=%.4f%n",
            r.kA,
            r.kCos,
            r.rSquared,
        )
        assertEquals(kA, r.kA, kA * 0.15, "kA within 15%")
        assertEquals(kG, r.kCos, kG * 0.10, "kCos within 10%")
    }

    @Test
    fun perSampleVoltageMatchesConstantVoltage() {
        // Battery-sag path (per-sample V) with constant V should match the constant-V overload.
        val rowsA = ArrayList<DoubleArray>()
        val rhsA = ArrayList<Double>()
        val rowsB = ArrayList<DoubleArray>()
        val rhsB = ArrayList<Double>()

        val theta = logRun(MIN + 0.05 * (MAX - MIN), 0.80, KS, KG, KV, KA)
        val v = 0.80 * HUB
        val voltages = DoubleArray(theta.size) { v }

        ArmSysId.accumulateRun(theta, v, DT, MIN, MAX, ArmSysId.DEFAULT_PARAMS, rowsA, rhsA)
        ArmSysId.accumulateRun(theta, voltages, DT, MIN, MAX, ArmSysId.DEFAULT_PARAMS, rowsB, rhsB)

        assertEquals(rowsA.size, rowsB.size)
        for (i in rhsA.indices) {
            assertEquals(rhsA[i], rhsB[i], 1e-9)
        }
    }

    companion object {
        private const val DT = 0.01
        private const val RUN_STEPS = 90
        private const val HUB = 12.0

        private const val KS = 0.3
        private const val KG = 3.5
        private const val KV = 1.2
        private const val KA = 0.35
        private const val TICKS_PER_REV = 28
        private const val GEAR = 100.0
        private const val ZERO = 0.0
        private val MIN = Math.toRadians(-45.0)
        private val MAX = Math.toRadians(225.0)

        /** Same constant-power battery as ControlLab (rigid plant, clean encoder). */
        private fun runBattery(kS: Double, kG: Double, kV: Double, kA: Double): ArmSysId.Result {
            val span = MAX - MIN
            val bottom = MIN + 0.05 * span
            val top = MAX - 0.05 * span
            val mid = MIN + 0.55 * span
            val margin = 0.08 * span

            val rows = ArrayList<DoubleArray>()
            val rhs = ArrayList<Double>()

            for (p in doubleArrayOf(0.45, 0.60, 0.80, 0.95)) {
                accumulate(bottom, p, kS, kG, kV, kA, rows, rhs)
            }
            for (p in doubleArrayOf(-0.45, -0.60, -0.80, -0.95)) {
                accumulate(top, p, kS, kG, kV, kA, rows, rhs)
            }
            // Gravity-rich: horizontals in range
            val gravityStarts = ArrayList<Double>()
            for (h in doubleArrayOf(0.0, PI, -PI)) {
                if (h > MIN + margin && h < MAX - margin) {
                    gravityStarts.add(h)
                }
            }
            if (gravityStarts.isEmpty()) {
                gravityStarts.add(mid)
            }
            for (start in gravityStarts) {
                accumulate(start, 0.70, kS, kG, kV, kA, rows, rhs)
                accumulate(start, -0.70, kS, kG, kV, kA, rows, rhs)
            }

            return ArmSysId.solve(rows, rhs)
        }

        private fun accumulate(
            startRad: Double,
            power: Double,
            kS: Double,
            kG: Double,
            kV: Double,
            kA: Double,
            rows: MutableList<DoubleArray>,
            rhs: MutableList<Double>,
        ) {
            val theta = logRun(startRad, power, kS, kG, kV, kA)
            ArmSysId.accumulateRun(
                theta,
                power * HUB,
                DT,
                MIN,
                MAX,
                ArmSysId.DEFAULT_PARAMS,
                rows,
                rhs,
            )
        }

        private fun logRun(
            startRad: Double,
            power: Double,
            kS: Double,
            kG: Double,
            kV: Double,
            kA: Double,
        ): DoubleArray {
            val ticksPerRad = TICKS_PER_REV * GEAR / (2.0 * PI)
            val sim = ArmMotorSim(kS, kG, kV, kA, TICKS_PER_REV, GEAR, ZERO, MIN, MAX, startRad)
            sim.setEncoder(EncoderSim())
            val theta = DoubleArray(RUN_STEPS)
            for (i in 0 until RUN_STEPS) {
                sim.step(DT, power, HUB)
                theta[i] = sim.getPositionTicks() / ticksPerRad + ZERO
            }
            return theta
        }
    }
}
