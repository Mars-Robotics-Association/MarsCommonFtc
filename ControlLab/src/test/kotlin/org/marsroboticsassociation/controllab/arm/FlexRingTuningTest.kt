package org.marsroboticsassociation.controllab.arm

import java.util.Locale
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Experiment: can the arrival overshoot + ringing of the flexible arm be tuned out with the profile
 * limits, without load-side sensing?
 *
 * <p>Observed live (flight log armlab-20260716-090940, MECHANISM_PIDF + Ruckig on the flex plant,
 * vMax 8 / aMax 12 / jMax 60): on every big-move arrival the tip swings ~2.5 deg past the target
 * and rings at ~2 Hz for a couple of cycles. The log shows the mesh stays engaged and the motor
 * tracks the profile tightly while the tip lags it by up to ~9 deg during braking — the flex spring
 * winds up under deceleration (`delta = kA_tip * a / k_flex`, ~2 deg at a = 12), and the stored
 * energy releases as the ring when the profile lands.
 *
 * <p>The tuning lever: a jerk-limited profile is an input shaper. A constant-jerk ramp of duration
 * `t_j = aMax/jMax` leaves zero residual energy in an undamped mode when `t_j` equals the mode's
 * period (the ramp's excitation spectrum has a null there). The relevant period is not the arm's
 * free 3 Hz: while the mesh is engaged the tip rides the flex spring <em>in series with</em> the
 * contact spring and the position servo's stiffness, which lowers the mode to ~2 Hz (0.5 s) — the
 * frequency the log actually shows. Hence the candidates: jMax = aMax/0.5 s.
 *
 * <p>This harness replays the logged scenario (flex plant, same gains, over-the-top moves -5 -> 177
 * -> -1 deg) across profile-limit variants and prints arrival metrics.
 *
 * <p><b>Findings</b> (asserted in [sweepProfileLimitsForRingdown]):
 * <ul>
 * <li>The logged config swings ~2.3 deg past the target on every arrival — reproducing the live
 *   observation. jMax 60 barely improves on unshaped j480 (2.9-3.2 deg) because its 0.2 s ramp sits
 *   near the worst of the mode's excitation spectrum.
 * <li><b>kD is the free lever.</b> Raising kD 1.5 -> 4 at the same limits halves the swing (~1.0
 *   deg) at zero cost in move time: while the mesh is engaged, motor-side velocity damping does
 *   reach the coupled mode through the gears.
 * <li><b>The jerk ramp is the real input shaper.</b> Stretching t_j into the 0.33-0.5 s band (the
 *   coupled mode's period range) plus modest accel cuts: a8/j16 or a6/j18 land at 0.2-0.35 deg peak
 *   — a 7-10x reduction — for ~0.6 s more move time. a12/j24 + kD 4 is the compromise at ~0.6 deg
 *   for +0.36 s.
 * </ul>
 *
 * <p>These findings set the mechanism defaults: [MechanismArmAdapter.Gains] now ships kD 4 with
 * a8/j24 (t_j = 0.33 s).
 */
class FlexRingTuningTest {

    /** One profile/gain variant to score. */
    private class Config(
        val label: String,
        val maxVel: Double,
        val maxAccel: Double,
        val maxJerk: Double,
        val kD: Double,
    )

    /** Metrics for one arrival, measured from the moment the profile lands on its endpoint. */
    private class Arrival {
        var moveSec = 0.0 // setTarget -> profile landing
        var peakDevDeg = 0.0 // max |tip - stated target| after landing (overshoot swing)
        var tipSettleSec = 0.0 // time after landing until the tip stays within the settle band
        var residP2PDeg = 0.0 // tip peak-to-peak over the last second of the watch window

        override fun toString(): String =
            String.format(
                Locale.US,
                "move=%4.2fs peak=%5.2fdeg settle=%4.2fs residP2P=%4.2fdeg",
                moveSec,
                peakDevDeg,
                tipSettleSec,
                residP2PDeg,
            )
    }

    companion object {
        private const val NOMINAL_DT = 0.016
        private const val SETTLE_BAND_DEG = 0.7
        private const val WATCH_SEC = 4.0

        /** The worse of the two arrivals' peak deviations for the config whose label starts so. */
        private fun worstPeak(
            configs: Array<Config>,
            results: List<Array<Arrival>>,
            labelPrefix: String,
        ): Double {
            for (i in configs.indices) {
                if (configs[i].label.startsWith(labelPrefix)) {
                    return max(results[i][0].peakDevDeg, results[i][1].peakDevDeg)
                }
            }
            throw AssertionError("no config labelled $labelPrefix")
        }
    }

    private fun makeEngine(c: Config): ArmEngine {
        val e = ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L)
        e.setPlantKind(ArmEngine.PlantKind.FLEX)
        val g = e.getMechGains()
        // Isolate the profile/kD shaping this test studies: disable the static-friction taper (an
        // orthogonal arrival lever, studied in StaticFrictionTaperTest) so its numbers reflect the
        // jerk ramp and kD alone. With the true kS=0.3 baked in here, the taper only slightly
        // softens braking; the arm ships it on as insurance against a sysid-inflated kS.
        g.staticFrictionTaperVelocity = 0.0
        e.setMechanismGains(
            g.kP,
            g.kI,
            c.kD,
            g.kS,
            g.kV,
            g.kA,
            g.kCos,
            g.kSin,
            c.maxVel,
            c.maxAccel,
            c.maxAccel,
            c.maxJerk,
        )
        return e
    }

    /** Drive one move and score the arrival. */
    private fun runMove(e: ArmEngine, targetDeg: Double): Arrival {
        e.setTargetDeg(targetDeg)
        val statedDeg = Math.toDegrees(e.targetRad) // after hard-stop clamp

        val a = Arrival()

        // Phase 1: run until the profile lands on its (compensated) endpoint.
        var tick = 0
        val maxTicks = (20.0 / NOMINAL_DT).toInt()
        while (tick < maxTicks) {
            e.tick()
            tick++
            val landed =
                abs(e.trajVelRad) < 1e-4 &&
                    abs(e.trajPosRad - e.profileTargetRad) < Math.toRadians(0.05)
            if (landed) break
        }
        a.moveSec = tick * NOMINAL_DT

        // Phase 2: watch the tip ring down.
        val watchTicks = (WATCH_SEC / NOMINAL_DT).toInt()
        val tip = ArrayList<Double>(watchTicks)
        for (i in 0 until watchTicks) {
            e.tick()
            tip.add(Math.toDegrees(e.trueLoadRad))
        }

        var peak = 0.0
        var lastOutside = -1
        for (i in tip.indices) {
            val dev = abs(tip[i] - statedDeg)
            peak = max(peak, dev)
            if (dev > SETTLE_BAND_DEG) lastOutside = i
        }
        a.peakDevDeg = peak
        a.tipSettleSec = (lastOutside + 1) * NOMINAL_DT

        val lastSecond = (1.0 / NOMINAL_DT).toInt()
        var mn = Double.POSITIVE_INFINITY
        var mx = Double.NEGATIVE_INFINITY
        for (i in tip.size - lastSecond until tip.size) {
            mn = min(mn, tip[i])
            mx = max(mx, tip[i])
        }
        a.residP2PDeg = mx - mn
        return a
    }

    @Test
    fun sweepProfileLimitsForRingdown() {
        // The mode to cancel: tip on the flex spring in series with contact + servo stiffness,
        // ~2 Hz observed (free tip-on-flex would be 3 Hz). t_j = aMax/jMax is the jerk-ramp
        // duration the sweep tunes toward those periods.
        val configs =
            arrayOf(
                Config("unshaped a12 j480        ", 8.0, 12.0, 480.0, 1.5),
                Config("logged   a12 j60  tj=0.20", 8.0, 12.0, 60.0, 1.5),
                Config("3Hz-notch a12 j36 tj=0.33", 8.0, 12.0, 36.0, 1.5),
                Config("2Hz-notch a12 j24 tj=0.50", 8.0, 12.0, 24.0, 1.5),
                Config("slower    a6  j60 tj=0.10", 8.0, 6.0, 60.0, 1.5),
                Config("3Hz-notch a6  j18 tj=0.33", 8.0, 6.0, 18.0, 1.5),
                Config("2Hz-notch a6  j12 tj=0.50", 8.0, 6.0, 12.0, 1.5),
                Config("3Hz-notch a8  j24 tj=0.33", 8.0, 8.0, 24.0, 1.5),
                Config("2Hz-notch a8  j16 tj=0.50", 8.0, 8.0, 16.0, 1.5),
                Config("logged + kD=4            ", 8.0, 12.0, 60.0, 4.0),
                Config("2Hz-notch a12 j24 + kD=4 ", 8.0, 12.0, 24.0, 4.0),
                Config("3Hz-notch a8  j24 + kD=4 ", 8.0, 8.0, 24.0, 4.0),
            )

        println("=== flex-arm arrival ringdown: -5 -> 177 -> -1 deg (flex plant, Ruckig) ===")
        System.out.printf(
            Locale.US,
            "%-26s | %-45s | %-45s%n",
            "config",
            "ascent arrival (177 deg)",
            "descent arrival (-1 deg)",
        )
        val results = ArrayList<Array<Arrival>>()
        for (c in configs) {
            val e = makeEngine(c)
            e.setTargetDeg(-5.44)
            for (i in 0 until 500) e.tick() // park like the logged session
            val up = runMove(e, 177.17)
            val down = runMove(e, -0.87)
            results.add(arrayOf(up, down))
            System.out.printf(Locale.US, "%-26s | %s | %s%n", c.label, up, down)
        }

        val loggedPeak = worstPeak(configs, results, "logged   a12 j60")
        val loggedKd4Peak = worstPeak(configs, results, "logged + kD=4")
        val shapedPeak = worstPeak(configs, results, "3Hz-notch a6  j18")

        // Finding 1: the live observation reproduces — the logged limits ring well past the
        // target on arrival.
        assertTrue(
            loggedPeak > 1.8,
            "logged config should reproduce the observed arrival swing, got $loggedPeak",
        )
        // Finding 2: kD alone (same limits, same move time) roughly halves the swing.
        assertTrue(
            loggedKd4Peak < 0.6 * loggedPeak,
            "kD=4 should halve the arrival swing (logged=" +
                loggedPeak +
                ", kD4=" +
                loggedKd4Peak +
                ")",
        )
        // Finding 3: shaping the jerk ramp onto the mode period tunes the ring out.
        assertTrue(
            shapedPeak < 0.5,
            "a6/j18 (t_j on the mode period) should land clean, got $shapedPeak",
        )
    }
}
