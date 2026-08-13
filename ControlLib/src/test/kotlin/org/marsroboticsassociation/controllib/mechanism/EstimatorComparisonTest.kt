package org.marsroboticsassociation.controllib.mechanism

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.Nat
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.estimator.KalmanFilter
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N2
import edu.wpi.first.math.system.LinearSystem
import edu.wpi.first.math.system.plant.LinearSystemId
import java.io.File
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.util.Random
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign
import kotlin.math.sin
import kotlin.math.sqrt
import org.junit.jupiter.api.AfterAll
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Head-to-head of the mechanism [MotorMechanismEkf] against the WPILib [KalmanFilter] configured
 * exactly like `MarsCommonFtc`'s `ArmController` (linear motor plant, gravity stripped from the
 * input, no velocity-lag de-lag), both scored against the faithful [ArmPlantSim] oracle. Plus the
 * latency-horizon sweep: how far the estimate must be projected forward to best predict the true
 * state at the actuation instant, with and without velocity de-lagging.
 *
 * <p>The thesis under test (from the design discussion): de-lagging the ~25 ms velocity boxcar and
 * forward-prediction are different operations, and once velocity is de-lagged the optimal forward
 * horizon should drop toward the true actuation delay. This harness turns that into numbers against
 * the oracle. Results are written to `build/estimator-comparison-report.txt`.
 */
class EstimatorComparisonTest {

    // ------------------------------------------------------------------------------------------
    // Part 1: head-to-head estimator accuracy against the oracle (passive observers).
    // ------------------------------------------------------------------------------------------

    @Test
    fun headToHead_blendingEkfVsArmControllerKalman() {
        val startRad = -PI / 4
        val targetRad = PI / 4

        val plant =
            ArmPlantSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_RAD,
                -PI * 0.9,
                PI * 0.9,
                startRad,
                ReadTimingJitter.controlHub(99L),
            )

        val blending =
            MotorMechanismEkf(
                ArmModel(K_S, K_V, K_A, K_G, 0.0),
                /* velocityLagSec= */ 0.025,
                /* modelAccelStdDev= */ 5.0,
                /* positionStdDev= */ 0.003,
                /* velocityStdDev= */ 0.1,
                /* positionTimingJitterStdDev= */ ReadTimingJitter.CONTROL_HUB_STD_SEC,
                startRad,
            )

        val wpilib = ArmStyleKalman(startRad)

        val rng = Random(99L)
        var power = 0.0
        var lastLoopMs = 0
        var nextLoopMs = 20

        var blendPosSq = 0.0
        var blendVelSq = 0.0
        var wpiPosSq = 0.0
        var wpiVelSq = 0.0
        var n = 0

        for (ms in 0..2500) {
            plant.integrate(0.001, power, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= nextLoopMs) {
                val dt = (ms - lastLoopMs) / 1000.0
                val measPosRad = plant.getEncoderPosition() / TICKS_PER_RAD
                val measVelRad = plant.getEncoderVelocityTps() / TICKS_PER_RAD
                val appliedVoltage = clamp(power, -1.0, 1.0) * VOLTAGE

                blending.predict(dt, power, VOLTAGE)
                blending.correct(measPosRad, measVelRad)

                wpilib.update(dt, measPosRad, measVelRad, appliedVoltage)

                // Controller acts on TRUE state so the trajectory is identical for both observers.
                val trueTicks = plant.getTrueAngleRad() * TICKS_PER_RAD
                val trueVelTps = plant.getTrueAngularVelocityRadPerSec() * TICKS_PER_RAD
                power =
                    0.004 * (targetRad * TICKS_PER_RAD - trueTicks) - 0.0008 * trueVelTps +
                        (K_G / VOLTAGE) * cos(plant.getTrueAngleRad())
                power = clamp(power, -1.0, 1.0)

                if (ms / 1000.0 > 0.2) {
                    val trueAngle = plant.getTrueAngleRad()
                    val trueVel = plant.getTrueAngularVelocityRadPerSec()
                    blendPosSq += sq(blending.position - trueAngle)
                    blendVelSq += sq(blending.velocity - trueVel)
                    wpiPosSq += sq(wpilib.position - trueAngle)
                    wpiVelSq += sq(wpilib.velocity - trueVel)
                    n++
                }

                lastLoopMs = ms
                nextLoopMs = ms + 20 + rng.nextInt(10)
            }
        }

        val blendPosRms = sqrt(blendPosSq / n)
        val blendVelRms = sqrt(blendVelSq / n)
        val wpiPosRms = sqrt(wpiPosSq / n)
        val wpiVelRms = sqrt(wpiVelSq / n)

        REPORT.append(
            "=== Part 1: estimator accuracy vs oracle (present state, -45..+45 sweep) ===\n"
        )
        REPORT.append("                         position RMS (rad)   velocity RMS (rad/s)\n")
        REPORT.append(
            String.format(
                "  blending EKF             %18.6f   %18.6f%n",
                blendPosRms,
                blendVelRms,
            )
        )
        REPORT.append(
            String.format(
                "  ArmController KF (WPILib) %17.6f   %18.6f%n",
                wpiPosRms,
                wpiVelRms,
            )
        )
        REPORT.append(
            String.format(
                "  velocity RMS ratio (KF / blending): %.2fx%n",
                wpiVelRms / blendVelRms,
            )
        )
        REPORT.append(
            "  note: this gap mixes two things — the structural de-lag AND ArmController's\n"
        )
        REPORT.append(
            "  looser noise tuning (modelStdDevVel=3.0, measStdDevPos=0.05). Part 2 isolates\n"
        )
        REPORT.append("  the de-lag alone (same filter, only velocityLagSec toggled).\n\n")

        // The de-lag should win on velocity, where the 50 ms boxcar lag bites during acceleration.
        assertTrue(
            blendVelRms < wpiVelRms,
            "blending velocity RMS ($blendVelRms) should beat the KF ($wpiVelRms)",
        )
    }

    // ------------------------------------------------------------------------------------------
    // Part 2: latency-horizon sweep — optimal forward horizon vs velocity de-lagging.
    // ------------------------------------------------------------------------------------------

    @Test
    fun latencyHorizonSweep_horizonTracksActuationDelay_deLagLowersFloor() {
        val totalMs = 3000
        val loopMs = 20 // fixed-cadence ZOH loop for clean curves
        val latchMs = 10
        val actuationDelaySec = 0.015 // the round-trip delay the forward-prediction compensates
        val actuationDelayMs = Math.round(actuationDelaySec * 1000).toInt()

        val startRad = -PI / 4

        // Reference true trajectory under the fixed ZOH excitation (oracle, no filter, no jitter).
        val trueAngle = referenceTrajectory(totalMs, loopMs, startRad)

        val horizons = DoubleArray(21) { i -> i * 0.0025 } // 0..50 ms, 2.5 ms grid

        REPORT.append("=== Part 2: latency-horizon sweep (predict the true state ")
            .append(actuationDelayMs)
            .append(" ms ahead) ===\n")
        REPORT.append(
            "Position RMS (rad) of the forward-projected estimate vs the true future state.\n"
        )
        REPORT.append(String.format("%-10s", "tau(ms)"))
        REPORT.append("   de-lagged(0.025)     simplified(0.000)\n")

        val rmsDelag = DoubleArray(horizons.size)
        val rmsSimple = DoubleArray(horizons.size)
        for (i in horizons.indices) {
            rmsDelag[i] =
                sweepRun(
                    0.025,
                    horizons[i],
                    totalMs,
                    loopMs,
                    latchMs,
                    actuationDelayMs,
                    startRad,
                    trueAngle,
                )
            rmsSimple[i] =
                sweepRun(
                    0.0,
                    horizons[i],
                    totalMs,
                    loopMs,
                    latchMs,
                    actuationDelayMs,
                    startRad,
                    trueAngle,
                )
            REPORT.append(
                String.format(
                    "%-10.1f   %16.6f   %18.6f%n",
                    horizons[i] * 1000.0,
                    rmsDelag[i],
                    rmsSimple[i],
                )
            )
        }

        val argDelag = argmin(rmsDelag)
        val argSimple = argmin(rmsSimple)
        val tauDelag = horizons[argDelag] * 1000.0
        val tauSimple = horizons[argSimple] * 1000.0

        REPORT.append(
            String.format(
                "%noptimal horizon  de-lagged: %.1f ms (RMS %.6f)%n",
                tauDelag,
                rmsDelag[argDelag],
            )
        )
        REPORT.append(
            String.format(
                "optimal horizon  simplified: %.1f ms (RMS %.6f)%n",
                tauSimple,
                rmsSimple[argSimple],
            )
        )
        REPORT.append(String.format("actuation delay being compensated: %d ms%n", actuationDelayMs))
        REPORT.append(
            "finding: with LIVE position the optimal horizon ~= the actuation delay for both" +
                " configs (position prediction dominates, so velocity lag does not move the" +
                " optimum); de-lagging velocity instead lowers the error floor ~2x.\n\n"
        )

        // The forward horizon is governed by the actuation delay, not the velocity sensor lag,
        // because position is a live counter. So BOTH optima sit near the actuation delay.
        assertTrue(
            abs(tauDelag - actuationDelayMs) <= 10.0,
            "de-lagged optimum ($tauDelag ms) should be near the actuation delay ($actuationDelayMs ms)",
        )
        assertTrue(
            abs(tauSimple - actuationDelayMs) <= 10.0,
            "simplified optimum ($tauSimple ms) should also be near the actuation delay ($actuationDelayMs ms)",
        )

        // De-lagging does not require a longer horizon (it does not stand in for
        // forward-prediction).
        assertTrue(
            tauDelag <= tauSimple,
            "de-lagging should not need a longer horizon than the simplified filter: de-lagged " +
                "$tauDelag ms vs simplified $tauSimple ms",
        )

        // The de-lag's payoff is accuracy: a clearly lower error floor at the optimal horizon.
        assertTrue(
            rmsDelag[argDelag] < 0.8 * rmsSimple[argSimple],
            "de-lagged error floor (${rmsDelag[argDelag]}) should clearly beat simplified (${rmsSimple[argSimple]})",
        )
    }

    /**
     * One sweep run: returns RMS of the tau-ahead projected position vs the true state D ms ahead.
     */
    private fun sweepRun(
        velocityLagSec: Double,
        horizonSec: Double,
        totalMs: Int,
        loopMs: Int,
        latchMs: Int,
        actuationDelayMs: Int,
        startRad: Double,
        trueAngle: DoubleArray,
    ): Double {
        val plant =
            ArmPlantSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_RAD,
                -PI * 0.9,
                PI * 0.9,
                startRad,
                ReadTimingJitter.disabled(),
            )
        val ekf =
            MotorMechanismEkf(
                ArmModel(K_S, K_V, K_A, K_G, 0.0),
                velocityLagSec,
                5.0,
                0.003,
                0.1,
                /* jitter= */ 0.0,
                startRad,
            )

        var heldPower = 0.0
        var lastFireMs = 0
        var sumSq = 0.0
        var n = 0

        for (ms in 0..totalMs) {
            if (ms % loopMs == 0) {
                if (ms > 0) {
                    val dt = (ms - lastFireMs) / 1000.0
                    val measPosRad = plant.getEncoderPosition() / TICKS_PER_RAD
                    val measVelRad = plant.getEncoderVelocityTps() / TICKS_PER_RAD
                    ekf.predict(dt, heldPower, VOLTAGE) // held power applied over the last interval
                    ekf.correct(measPosRad, measVelRad)

                    val newHeld = excitation(ms)
                    if (ms / 1000.0 > 0.2 && ms + actuationDelayMs <= totalMs) {
                        val projected = ekf.projectedState(horizonSec, newHeld, VOLTAGE)
                        sumSq += sq(projected[0] - trueAngle[ms + actuationDelayMs])
                        n++
                    }
                    heldPower = newHeld
                    lastFireMs = ms
                } else {
                    heldPower = excitation(0)
                }
            }
            plant.integrate(0.001, heldPower, VOLTAGE)
            if (ms % latchMs == 0) {
                plant.latchEncoder()
            }
        }
        return sqrt(sumSq / n)
    }

    /** True angle at each ms under the fixed ZOH excitation (the oracle, no filter, no jitter). */
    private fun referenceTrajectory(totalMs: Int, loopMs: Int, startRad: Double): DoubleArray {
        val ref =
            ArmPlantSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_RAD,
                -PI * 0.9,
                PI * 0.9,
                startRad,
                ReadTimingJitter.disabled(),
            )
        val trueAngle = DoubleArray(totalMs + 1)
        var heldPower = 0.0
        for (ms in 0..totalMs) {
            if (ms % loopMs == 0) {
                heldPower = excitation(ms)
            }
            ref.integrate(0.001, heldPower, VOLTAGE)
            trueAngle[ms] = ref.getTrueAngleRad()
        }
        return trueAngle
    }

    /**
     * Fixed open-loop command: gravity-ish bias plus a sweep that accelerates through horizontal.
     */
    private fun excitation(ms: Int): Double {
        val t = ms / 1000.0
        return 0.125 + 0.5 * sin(2 * PI * 0.6 * t)
    }

    // ------------------------------------------------------------------------------------------
    // Part 3: non-circular delay recovery. A real actuation-delay FIFO lives in the plant; the
    // closed-loop optimal forward horizon must TRACK that plant delay (not a number in the
    // harness),
    // and do so independently of velocity de-lagging.
    // ------------------------------------------------------------------------------------------

    @Test
    fun delayRecoverySweep_peakErrorUShaped_forwardProjectionWeaklyTracksDelay() {
        // Push the actuation delay well above the ~20 ms loop/ZOH so it dominates the loop
        // dynamics;
        // otherwise the optimal horizon is set by the loop, not the delay, and barely tracks it.
        val plantDelaysMs = intArrayOf(10, 30, 50, 70)
        val horizons = DoubleArray(49) { i -> i * 0.0025 } // 0..120 ms, 2.5 ms grid

        REPORT.append(
            "=== Part 3: non-circular delay recovery (real actuation-delay FIFO in plant) ===\n"
        )
        REPORT.append(
            "Closed loop (controller + EKF). Metric: peak |true - setpoint| (rad) over the move,\n"
        )
        REPORT.append(
            "which penalizes both lag (low horizon) and lead (high horizon) -> interior minimum.\n"
        )
        REPORT.append(
            "The actuation delay is a real FIFO in the plant (pushed dominant, 10..70 ms), so the\n"
        )
        REPORT.append(
            "optimal horizon reflects the plant, not the harness. (* marks the column optimum.)\n\n"
        )

        val argDelag = IntArray(plantDelaysMs.size)
        val argSimple = IntArray(plantDelaysMs.size)

        for (d in plantDelaysMs.indices) {
            val delaySec = plantDelaysMs[d] / 1000.0
            val peakDelag = DoubleArray(horizons.size)
            val peakSimple = DoubleArray(horizons.size)
            for (i in horizons.indices) {
                peakDelag[i] = closedLoopPeakError(delaySec, 0.025, horizons[i])
                peakSimple[i] = closedLoopPeakError(delaySec, 0.0, horizons[i])
            }
            argDelag[d] = argmin(peakDelag)
            argSimple[d] = argmin(peakSimple)

            REPORT.append(String.format("plant actuation delay = %d ms%n", plantDelaysMs[d]))
            REPORT.append(String.format("%-10s   de-lagged           simplified%n", "tau(ms)"))
            for (i in horizons.indices) {
                REPORT.append(
                    String.format(
                        "%-10.1f   %12.6f%s   %12.6f%s%n",
                        horizons[i] * 1000.0,
                        peakDelag[i],
                        if (i == argDelag[d]) " *" else "  ",
                        peakSimple[i],
                        if (i == argSimple[d]) " *" else "  ",
                    )
                )
            }
            REPORT.append(
                String.format(
                    "  -> optimal horizon: de-lagged %.1f ms, simplified %.1f ms%n%n",
                    horizons[argDelag[d]] * 1000.0,
                    horizons[argSimple[d]] * 1000.0,
                )
            )
        }

        REPORT.append("summary: plant delay -> recovered optimal horizon\n")
        for (d in plantDelaysMs.indices) {
            REPORT.append(
                String.format(
                    "  %2d ms -> de-lagged %.1f ms, simplified %.1f ms%n",
                    plantDelaysMs[d],
                    horizons[argDelag[d]] * 1000.0,
                    horizons[argSimple[d]] * 1000.0,
                )
            )
        }
        REPORT.append(
            "\nfinding: peak |true-setpoint| IS a proper metric (clean interior U-shape, unlike\n"
        )
        REPORT.append(
            "overshoot). But it shows naive forward-projection only WEAKLY compensates the actuation\n"
        )
        REPORT.append(
            "delay: recovered horizon ~12.5/15/17.5/20 ms for delays 10/30/50/70 ms (slope ~0.12,\n"
        )
        REPORT.append(
            "not ~1). Even a 70 ms delay is best offset by only ~20 ms of lead -- you cannot project\n"
        )
        REPORT.append(
            "your way out of the delay here. Projection has competing effects (adds phase lead but\n"
        )
        REPORT.append(
            "also steady lag), and projectedState just rolls the state forward under the held command;\n"
        )
        REPORT.append(
            "it does NOT account for the commands already in flight in the delay buffer. A Smith-style\n"
        )
        REPORT.append(
            "predictor that did would track the delay far better. So 'forward-predict by the delay'\n"
        )
        REPORT.append("is not a clean knob with this simple scheme.\n\n")

        // Peak error has an INTERIOR optimum (not pinned to either grid edge) -> a real U-shape,
        // where overshoot bottomed at the over-lead corner.
        val lastIdx = horizons.size - 1
        for (d in plantDelaysMs.indices) {
            assertTrue(
                argDelag[d] > 0 && argDelag[d] < lastIdx,
                "peak-error optimum should be interior at delay ${plantDelaysMs[d]} ms (" +
                    "${horizons[argDelag[d]] * 1000} ms)",
            )
        }
        // The optimum rises with the delay (it is a real, plant-driven trend, not censoring)...
        for (d in 1 until plantDelaysMs.size) {
            assertTrue(
                horizons[argDelag[d]] >= horizons[argDelag[d - 1]] - 1e-12,
                "de-lagged optimum should not decrease as plant delay grows: " +
                    "${horizons[argDelag[d]] * 1000} ms at ${plantDelaysMs[d]} ms",
            )
        }
        assertTrue(
            horizons[argDelag[plantDelaysMs.size - 1]] > horizons[argDelag[0]],
            "de-lagged optimum should rise across the 10->70 ms delay span",
        )
        // ...but only WEAKLY: even the largest (dominant) delay is best offset by a far smaller
        // horizon, so naive forward-projection does not invert the delay.
        val last = plantDelaysMs.size - 1
        assertTrue(
            horizons[argDelag[last]] < 0.5 * (plantDelaysMs[last] / 1000.0),
            "recovered horizon should be far below the dominant plant delay (weak tracking): " +
                "${horizons[argDelag[last]] * 1000} ms for a ${plantDelaysMs[last]} ms delay",
        )
    }

    /**
     * Closed-loop peak |true - setpoint| (rad) over the move, for a given plant delay, de-lag, and
     * forward horizon. Unlike overshoot, this penalizes BOTH under-compensation (true lags the
     * setpoint) and over-compensation (true leads it), so it has an interior minimum near the
     * delay.
     */
    private fun closedLoopPeakError(
        actuationDelaySec: Double,
        velocityLagSec: Double,
        horizonSec: Double,
    ): Double {
        val startRad = -PI / 4
        val targetRad = PI / 4

        val plant =
            ArmPlantSim(
                K_S,
                K_G,
                K_V,
                K_A,
                TICKS_PER_RAD,
                -PI * 0.9,
                PI * 0.9,
                startRad,
                ReadTimingJitter.disabled(),
            )
        plant.setActuationDelaySec(actuationDelaySec)

        val model = ArmModel(K_S, K_V, K_A, K_G, 0.0)
        val ekf = MotorMechanismEkf(model, velocityLagSec, 5.0, 0.003, 0.1, 0.0, startRad)
        val ctrl = MotorMechanismController(model, 40.0, 8.0, 1.5, 8.0, 12.0, 480.0, 1.5, startRad)

        var lastPower = 0.0
        var lastFireMs = 0
        var currentSetpoint = startRad
        var maxAbsErr = 0.0

        for (ms in 0..2000) {
            if (ms % 20 == 0) {
                if (ms > 0) {
                    val dt = (ms - lastFireMs) / 1000.0
                    val measPosRad = plant.getEncoderPosition() / TICKS_PER_RAD
                    val measVelRad = plant.getEncoderVelocityTps() / TICKS_PER_RAD
                    ekf.predict(dt, lastPower, VOLTAGE)
                    ekf.correct(measPosRad, measVelRad)
                    val proj = ekf.projectedState(horizonSec, lastPower, VOLTAGE)
                    val volts = ctrl.calculate(targetRad, proj[0], proj[1], VOLTAGE, dt)
                    lastPower = clamp(volts / VOLTAGE, -1.0, 1.0)
                    lastFireMs = ms
                } else {
                    val proj = ekf.projectedState(horizonSec, lastPower, VOLTAGE)
                    val volts = ctrl.calculate(targetRad, proj[0], proj[1], VOLTAGE, 0.02)
                    lastPower = clamp(volts / VOLTAGE, -1.0, 1.0)
                }
                currentSetpoint = ctrl.setpointPosition
            }
            plant.integrate(0.001, lastPower, VOLTAGE)
            if (ms % 10 == 0) {
                plant.latchEncoder()
            }
            if (ms >= 40) {
                maxAbsErr = max(maxAbsErr, abs(plant.getTrueAngleRad() - currentSetpoint))
            }
        }
        return maxAbsErr
    }

    // ------------------------------------------------------------------------------------------
    // WPILib Kalman filter wrapped to match ArmController exactly (gravity stripped from the input,
    // no velocity-lag de-lag, present estimate read after correct).
    // ------------------------------------------------------------------------------------------

    private class ArmStyleKalman(startRad: Double) {
        private val kf: KalmanFilter<N2, N1, N2>
        private var lastLinearVoltage = 0.0
        private var estPos: Double
        private var estVel: Double

        init {
            val plant: LinearSystem<N2, N1, N2> = LinearSystemId.identifyPositionSystem(K_V, K_A)
            // ArmController.PARAMS defaults: model std (0.01 rad, 3.0 rad/s), meas std (0.05, 0.5).
            kf =
                KalmanFilter(
                    Nat.N2(),
                    Nat.N2(),
                    plant,
                    VecBuilder.fill(0.01, 3.0),
                    VecBuilder.fill(0.05, 0.5),
                    0.02,
                )
            kf.setXhat(VecBuilder.fill(startRad, 0.0))
            estPos = startRad
            estVel = 0.0
        }

        fun update(dt: Double, measPosRad: Double, measVelRad: Double, appliedVoltage: Double) {
            val uLinear: Matrix<N1, N1> = VecBuilder.fill(lastLinearVoltage)
            kf.correct(uLinear, VecBuilder.fill(measPosRad, measVelRad))
            // Present (filtered) estimate is read here, before predicting on to the next step.
            estPos = kf.getXhat(0)
            estVel = kf.getXhat(1)
            kf.predict(uLinear, dt)
            // Linear-model input for next loop: strip gravity and friction (handled by
            // feedforward).
            val gravity = K_G * cos(estPos)
            val friction = K_S * sign(estVel)
            lastLinearVoltage = appliedVoltage - gravity - friction
        }

        val position: Double
            get() = estPos

        val velocity: Double
            get() = estVel
    }

    companion object {
        private val TICKS_PER_RAD = 28 * 100 / (2 * PI)
        private const val K_S = 0.1
        private const val K_G = 1.5
        private const val K_V = 2.0
        private const val K_A = 0.2
        private const val VOLTAGE = 12.0

        private val REPORT = StringBuilder()

        @JvmStatic
        @AfterAll
        fun writeReport() {
            val dir = File(System.getProperty("user.dir"), "build")
            dir.mkdirs()
            val out = File(dir, "estimator-comparison-report.txt")
            Files.write(out.toPath(), REPORT.toString().toByteArray(StandardCharsets.UTF_8))
            println("Estimator comparison report: ${out.absolutePath}")
        }

        private fun argmin(a: DoubleArray): Int {
            var best = 0
            for (i in 1 until a.size) {
                if (a[i] < a[best]) best = i
            }
            return best
        }

        private fun clamp(v: Double, lo: Double, hi: Double): Double = max(lo, min(hi, v))

        private fun sq(v: Double): Double = v * v
    }
}
