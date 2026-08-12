package org.marsroboticsassociation.controllab.arm

import java.util.Locale
import kotlin.math.abs
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Regression for the overshoot that returned when a motor-side sysid through the lash + flex was
 * applied (flight log armlab-20260716-200720): the identified `kS` came out ~1.32 V (true plant kS
 * 0.3), and `kS·sign(v)` is an anti-braking term that fought every arrival — big moves swung ~5 deg
 * past target, versus sub-degree before the sysid.
 *
 * <p>The fix is the controller's static-friction taper ({@link
 * org.marsroboticsassociation.controllib.mechanism.MotorMechanismController#setStaticFrictionTaperVelocity}):
 * ramping the `kS` feedforward to zero as the profile decelerates into an arrival caps what an
 * over-estimated `kS` can do to braking. This replays the logged over-the-top moves on the flex
 * plant with the logged (post-sysid) gains and asserts the taper brings the arrival swing back
 * down, at no cost to move time.
 */
class StaticFrictionTaperTest {

    companion object {
        private const val NOMINAL_DT = 0.016
        private const val SETTLE_BAND_DEG = 0.7
        private const val WATCH_SEC = 4.0

        // The gains the flight log shows sysid applied (kS badly inflated by the lash + flex).
        private const val KP = 40.0
        private const val KI = 8.0
        private const val KD = 4.0
        private const val KS = 1.318
        private const val KV = 1.178
        private const val KA = 0.3108
        private const val KCOS = 2.720
        private const val KSIN = 0.0
        private const val VMAX = 8.0
        private const val AMAX = 8.0
        private const val DMAX = 8.0
        private const val JMAX = 24.0
    }

    private fun makeEngine(taperVel: Double): ArmEngine {
        val e = ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L)
        e.setPlantKind(ArmEngine.PlantKind.FLEX)
        val g = e.getMechGains()
        g.staticFrictionTaperVelocity = taperVel
        e.setMechanismGains(KP, KI, KD, KS, KV, KA, KCOS, KSIN, VMAX, AMAX, DMAX, JMAX)
        return e
    }

    /** Peak past-target tip swing after the profile lands, and the move time to landing. */
    private fun runMove(e: ArmEngine, targetDeg: Double): DoubleArray {
        e.setTargetDeg(targetDeg)
        val statedDeg = Math.toDegrees(e.getTargetRad())

        var tick = 0
        val maxTicks = (20.0 / NOMINAL_DT).toInt()
        while (tick < maxTicks) {
            e.tick()
            tick++
            val landed =
                abs(e.getTrajVelRad()) < 1e-4 &&
                    abs(e.getTrajPosRad() - e.getProfileTargetRad()) < Math.toRadians(0.05)
            if (landed) break
        }
        val moveSec = tick * NOMINAL_DT

        var peak = 0.0
        val watchTicks = (WATCH_SEC / NOMINAL_DT).toInt()
        for (i in 0 until watchTicks) {
            e.tick()
            peak = max(peak, abs(Math.toDegrees(e.getTrueLoadRad()) - statedDeg))
        }
        return doubleArrayOf(peak, moveSec)
    }

    private fun worstArrival(taperVel: Double): DoubleArray {
        val e = makeEngine(taperVel)
        e.setTargetDeg(-5.44)
        for (i in 0 until 500) e.tick() // park like the logged session
        val up = runMove(e, 177.17)
        val down = runMove(e, -0.87)
        return doubleArrayOf(max(up[0], down[0]), max(up[1], down[1]))
    }

    @Test
    fun taperCutsPostSysidArrivalSwing() {
        val off = worstArrival(0.0) // taper disabled: reproduce the log
        val on = worstArrival(1.0) // taper enabled (the arm default)
        System.out.printf(
            Locale.US,
            "post-sysid gains: taper off peak=%.2fdeg (move %.2fs), on peak=%.2fdeg (move %.2fs)%n",
            off[0],
            off[1],
            on[0],
            on[1],
        )

        // The inflated kS reproduces the logged arrival swing with the taper off.
        assertTrue(
            off[0] > 2.5,
            "inflated-kS gains should reproduce the logged arrival swing, got " + off[0],
        )
        // The taper neutralizes the kS anti-braking: swing drops by at least 30%. It plateaus here
        // (~2.5 deg) because the rest of the residual is the sysid's under-estimated kCos, not kS —
        // that half is the identification fix (a quasi-static kCos), not the taper's job.
        assertTrue(
            on[0] < 0.7 * off[0],
            "taper should cut the arrival swing by >=30% (off=" + off[0] + ", on=" + on[0] + ")",
        )
        // At no cost to move time.
        assertTrue(
            on[1] <= off[1] + 0.05,
            "taper should not slow the move (off=" + off[1] + ", on=" + on[1] + ")",
        )
    }

    /**
     * End to end: the whole point. Run the engine's own sysid through the flex plant, apply the
     * identified gains, then drive the logged over-the-top moves. With the quasi-static sweep (a
     * faithful kS/kCos instead of the lash-inflated ones) and the taper both live, the arrival
     * swing stays small — the regression the flight log showed is gone without any hand-set gains.
     */
    @Test
    fun sysIdThenMoveIsClean() {
        val e = ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L)
        e.setPlantKind(ArmEngine.PlantKind.FLEX)
        val r = e.runSysId()
        e.applyIdentifiedGains(r)
        System.out.printf(
            Locale.US,
            "flex sysid applied: kS=%.3f kV=%.3f kA=%.3f kCos=%.3f%n",
            r.kS,
            r.kV,
            r.kA,
            r.kCos,
        )

        e.setTargetDeg(-5.44)
        for (i in 0 until 500) e.tick()
        val up = runMove(e, 177.17)[0]
        val down = runMove(e, -0.87)[0]
        val worst = max(up, down)
        System.out.printf(Locale.US, "arrival swing after flex sysid: %.2fdeg%n", worst)

        assertTrue(
            worst < 1.5,
            "sysid through flex + taper should keep the arrival swing small, got $worst",
        )
    }
}
