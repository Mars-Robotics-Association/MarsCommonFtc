package org.marsroboticsassociation.controllab.arm

import java.util.Locale
import kotlin.math.abs
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.parallel.Execution
import org.junit.jupiter.api.parallel.ExecutionMode

/**
 * Headless quality gate for the Lineage B setpoint profiler (`ModelAwareRuckigProfiler`, per-loop
 * OTG replanning) on the full ControlLab arm stack (EKF + PIDF + back-EMF ceilings against the
 * backlash and flex plants).
 *
 * <p>The headline metric is <b>peak setpoint jerk</b>, measured as `|Δa_setpoint| / dt` per tick:
 * the OTG stop is planned, so it must stay near the configured limit through arrivals and
 * mid-flight retargets. Plant-side quality (settle, final error) is asserted absolutely.
 */
@Execution(ExecutionMode.SAME_THREAD)
class MechanismProfilerTest {

    companion object {
        private const val SEED = 42L

        // The profile limits and kD this scenario suite was designed around, pinned explicitly so
        // it
        // is insulated from the mechanism defaults: the flex-shaped defaults (a8/j24, kD 4)
        // deliberately trade retarget agility for arrival ringdown — FlexRingTuningTest's subject,
        // not this test's.
        private const val PIN_KD = 1.5
        private const val PIN_VMAX = 8.0
        private const val PIN_AMAX = 12.0
        private const val PIN_JMAX = 480.0

        /**
         * Settle band for the harness's own settle metric. ArmMetrics uses 2°, but on the backlash
         * plant the load parks ~2.5–3.3° from target (it rests against the gravity side of the
         * lash), so the 2° latch never fires and tells us nothing. 4° is outside the lash sag and
         * cleanly separates "arrived" from "still moving".
         */
        private const val SETTLE_BAND_DEG = 4.0

        // Workspace: min −45°, max +225°; engine parks at 225°.
        private fun backlashScript(): List<Segment> {
            val s = ArrayList<Segment>()
            s.add(Segment("big move 225→90", 90.0, 320, true))
            s.add(Segment("gravity descent 90→0", 0.0, 320, true))
            s.add(Segment("small step 0→10", 10.0, 200, true))
            s.add(Segment("retarget staging 10→100", 100.0, 30, false)) // interrupted mid-flight
            s.add(Segment("retarget mid-flight →45", 45.0, 320, true))
            return s
        }

        private fun flexScript(): List<Segment> {
            val s = ArrayList<Segment>()
            s.add(Segment("flex: big move 225→90", 90.0, 320, true))
            s.add(Segment("flex: gravity descent 90→0", 0.0, 320, true))
            return s
        }

        private fun run(plant: ArmEngine.PlantKind, script: List<Segment>): List<MoveResult> {
            val engine = ArmEngine(ArmControllerType.MECHANISM_PIDF, SEED)
            val g = engine.getMechGains()
            engine.setMechanismGains(
                g.kP,
                g.kI,
                PIN_KD,
                g.kS,
                g.kV,
                g.kA,
                g.kCos,
                g.kSin,
                PIN_VMAX,
                PIN_AMAX,
                PIN_AMAX,
                PIN_JMAX,
            )
            if (plant != engine.getPlantKind()) {
                engine.setPlantKind(plant)
            }

            val results = ArrayList<MoveResult>()
            for (seg in script) {
                engine.setTargetRad(Math.toRadians(seg.targetDeg))
                val segStart = engine.getElapsedSec()
                var prevA = engine.getTrajAccelRad()
                var prevT = engine.getElapsedSec()
                var peakJerk = 0.0
                var lastOutsideBand = segStart // time the load was last outside the settle band
                for (i in 0 until seg.ticks) {
                    engine.tick()
                    val dt = engine.getElapsedSec() - prevT
                    val a = engine.getTrajAccelRad()
                    peakJerk = max(peakJerk, abs(a - prevA) / dt)
                    val errDeg =
                        abs(Math.toDegrees(engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg)))
                    if (errDeg > SETTLE_BAND_DEG) {
                        lastOutsideBand = engine.getElapsedSec()
                    }
                    prevA = a
                    prevT = engine.getElapsedSec()
                }
                if (!seg.measured) {
                    continue
                }
                val r = MoveResult()
                r.name = seg.name
                val insideAtEnd =
                    abs(Math.toDegrees(engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg))) <=
                        SETTLE_BAND_DEG
                r.settleSec = if (insideAtEnd) lastOutsideBand - segStart else Double.NaN
                r.finalErrDeg =
                    abs(Math.toDegrees(engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg)))
                r.peakTrajJerk = peakJerk
                results.add(r)
            }
            return results
        }

        private fun printTable(header: String, results: List<MoveResult>) {
            println()
            println("=== $header — ModelAwareRuckigProfiler ===")
            System.out.printf(
                Locale.US,
                "%-28s %13s %13s %13s%n",
                "move",
                "settle (s)",
                "final err(°)",
                "peak jerk",
            )
            for (r in results) {
                System.out.printf(
                    Locale.US,
                    "%-28s %13.2f %13.2f %13.0f%n",
                    r.name,
                    r.settleSec,
                    r.finalErrDeg,
                    r.peakTrajJerk,
                )
            }
            println("(peak jerk in rad/s^3 against the configured limit $PIN_JMAX)")
        }
    }

    /** One commanded move plus how long to run it, in ticks (~16 ms each). */
    private class Segment(
        val name: String,
        val targetDeg: Double,
        val ticks: Int,
        val measured: Boolean, // false = staging move, excluded from metrics/assertions
    )

    private class MoveResult {
        var name: String = ""
        var settleSec: Double = 0.0 // time until the load last entered the 4° band (NaN = never)
        var finalErrDeg: Double = 0.0
        var peakTrajJerk: Double = 0.0 // rad/s^3, from setpoint accel differences
    }

    @Test
    fun setpointStaysJerkBoundedWhileTrackingWell() {
        val backlash = run(ArmEngine.PlantKind.BACKLASH, backlashScript())
        val flex = run(ArmEngine.PlantKind.FLEX, flexScript())

        printTable("BACKLASH plant", backlash)
        printTable("FLEX plant", flex)

        for (r in backlash) {
            assertTrue(!r.settleSec.isNaN(), "settles: " + r.name)
            assertTrue(r.finalErrDeg < 4.0, "final error " + r.name + ": " + r.finalErrDeg)
            // The OTG setpoint respects the configured jerk limit through its stops (small slack
            // for dt jitter across replan boundaries).
            assertTrue(
                r.peakTrajJerk <= PIN_JMAX * 1.15,
                "setpoint jerk bounded on " +
                    r.name +
                    ": " +
                    r.peakTrajJerk +
                    " vs limit " +
                    PIN_JMAX,
            )
        }

        // Flex plant: no settle-time assertion (structural ringing), but every move must arrive
        // and stay jerk-bounded. The flex descent parks ~6° off target (lash + flex sag), so
        // "arrives" is an 8° bound there.
        for (r in flex) {
            assertTrue(r.finalErrDeg < 8.0, "flex arrives: " + r.name)
            assertTrue(r.peakTrajJerk <= PIN_JMAX * 1.15, "flex jerk bounded on " + r.name)
        }
    }

    @Test
    fun survivesCollapsedBackEmfCeilings() {
        // With a high model kV (a plausible SysID result) the back-EMF velocity ceiling sits far
        // below the configured vMax, so the profile cruises pinned to a position-dependent
        // ceiling that is rewritten every loop while the accel ceiling reads zero. Replans fail
        // from that state, and the profiler must clamp into the fresh bounds and replan rather
        // than hold: a held out-of-band state reproduces the same failing inputs every loop,
        // freezing the profile mid-flight.
        val e = ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L)
        val g = e.getMechGains()
        e.setMechanismGains(
            g.kP,
            g.kI,
            g.kD,
            g.kS,
            4.0 /* kV */,
            g.kA,
            g.kCos,
            g.kSin,
            g.maxVel,
            g.maxAccel,
            g.maxDecel,
            g.maxJerk,
        )

        for (targetDeg in doubleArrayOf(90.0, 0.0, 90.0, 180.0)) {
            val target = Math.toRadians(targetDeg)
            e.setTargetRad(target)
            for (i in 0 until 900) {
                e.tick()
            }
            // Score the profile against the endpoint it actually chases — with the backlash
            // plant live, rest-only compensation biases that up to a half-lash off the stated
            // target.
            val profErrDeg = abs(Math.toDegrees(e.getTrajPosRad() - e.getProfileTargetRad()))
            assertTrue(
                profErrDeg < 1.0,
                "profile arrives at " +
                    targetDeg +
                    "° under collapsed ceilings; err=" +
                    profErrDeg +
                    "°",
            )
        }
    }
}
