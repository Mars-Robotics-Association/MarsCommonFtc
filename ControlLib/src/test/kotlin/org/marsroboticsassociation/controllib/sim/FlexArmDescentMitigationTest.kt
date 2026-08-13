package org.marsroboticsassociation.controllib.sim

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sign
import kotlin.math.sqrt
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

/**
 * Experiment: which controller/estimator strategies smooth the flexible-arm descent bounce on
 * [FlexArmMotorSim]? Companion to `ArmBacklashKeepEngagedTest` (which studied the rigid backlash
 * plant); here the load carries a lightly damped structural flex mode behind the lash.
 *
 * <p>Findings this harness demonstrates (see the printed table; asserted in [compareStrategies]):
 * <ul>
 * <li><b>The bounce is seeded by one event: the lash handoff where gravity preload is near
 *   zero.</b> Starting a descent from upright (cos θ ≈ 0), the teeth carry nothing; the arm
 *   free-falls through the gap and that single impact rings the flex mode for the next second-plus.
 *   Once gravity loads the mesh, a smooth profile keeps it loaded all the way down and the mode
 *   stays quiet.
 * <li><b>The soft-handoff profile wins.</b> Creeping across the zero-gravity zone (0.3 rad/s for
 *   the first ~10°) before opening up to full speed roughly halves the bounce metric (0.37 → 0.17
 *   rad/s) and removes the visible ring, for ~0.4 s of extra descent time. This is the same
 *   keep-engaged recipe `HandoffAwareArmModel` injects through the mechanism model's ceiling hooks.
 * <li><b>Input shaping — the textbook flexible-mode answer — does nothing here</b> (0.38 vs 0.37):
 *   the excitation is not the command step but the lash impact, which a shaper cannot schedule
 *   away.
 * <li><b>No estimator rescues this.</b> Even a perfect-model three-inertia observer buys almost
 *   nothing: PD on the estimated tip state is no better than PD on the motor (0.36 vs 0.37), and
 *   twist-rate (resonance) damping helps ~10% at its best gain, then goes violently unstable at 3×
 *   that gain (osc 2.3 rad/s, 11 lash separations). This is the classic non-collocated-control
 *   ceiling: the actuator sits on the wrong side of both the lash and the flex spring, and the mesh
 *   unloads exactly when the controller needs to push. The problem is not knowing where the tip is
 *   — it is not being able to grab it.
 * </ul>
 *
 * <p>Past the profile fix, the remaining levers are hardware: less lash, a stiffer arm (higher flex
 * frequency), or tip-side sensing <em>with tip-side actuation</em>.
 */
class FlexArmDescentMitigationTest {

    companion object {
        const val KS = 0.3
        const val KG = 3.5
        const val KV = 1.2
        const val KA = 0.35
        val MIN = Math.toRadians(-45.0)
        val MAX = Math.toRadians(225.0)
        val LASH = Math.toRadians(5.0)
        const val FLEX_HZ = 3.0
        const val FLEX_ZETA = 0.03
        const val VBUS = 12.0
        const val DT = 0.016
        val FROM = Math.toRadians(90.0)
        val TO = Math.toRadians(-40.0)
        const val SPEED = 1.2
    }

    // ── observer: replica of the plant model (perfect params — the estimator's best case),
    //    corrected on the motor states each loop ─────────────────────────────────────────────
    private class Observer(start: Double) {
        val kAm = 0.50 * KA
        val kAh = 0.05 * KA
        val kAt = 0.45 * KA
        val gTip = 0.85
        val kc = 500.0
        val cc = 2.0
        val kf = (2 * PI * FLEX_HZ).pow(2) * kAt
        val cf = 2 * FLEX_ZETA * sqrt(kf * kAt)
        var tm = 0.0
        var wm = 0.0
        var th = 0.0
        var wh = 0.0
        var tt = 0.0
        var wt = 0.0

        init {
            tt = start
            th = start + gTip * KG * cos(start) / kf
            val hold = gTip * KG * cos(start) + (1 - gTip) * KG * cos(th)
            tm = th + hold / kc + sign(hold) * LASH / 2
        }

        fun contact(): Double {
            val d = tm - th
            val h = LASH / 2
            if (d > h) return kc * (d - h) + cc * (wm - wh)
            if (d < -h) return kc * (d + h) + cc * (wm - wh)
            return 0.0
        }

        fun predict(u: Double, dt: Double) {
            var r = 0.0
            while (r < dt - 1e-12) {
                val tauC = contact()
                val tauF = kf * (th - tt) + cf * (wh - wt)
                wm += 0.0005 * (u - KV * wm - KS * sign(wm) - tauC) / kAm
                wh += 0.0005 * (tauC - tauF - (1 - gTip) * KG * cos(th)) / kAh
                wt += 0.0005 * (tauF - gTip * KG * cos(tt)) / kAt
                tm += 0.0005 * wm
                th += 0.0005 * wh
                tt += 0.0005 * wt
                r += 0.0005
            }
        }

        fun correct(posMeas: Double, velMeas: Double) {
            tm += 0.6 * (posMeas - tm)
            wm += 0.6 * (velMeas - wm)
        }
    }

    // ── setpoint generators (called once per DT, in order) ──────────────────────
    private fun interface Ramp {
        fun at(t: Double): DoubleArray // returns {sp, spv}
    }

    private fun instant(): Ramp {
        val end = (TO - FROM) / -SPEED
        return Ramp { t ->
            if (t >= end) doubleArrayOf(TO, 0.0) else doubleArrayOf(FROM - SPEED * t, -SPEED)
        }
    }

    /** ZV shaper: velocity steps of half amplitude, half a flex period apart. */
    private fun zvShaped(): Ramp {
        val t2 = 1.0 / (2 * FLEX_HZ)
        val end = (TO - FROM + SPEED * t2 / 2) / -SPEED
        return Ramp { t ->
            val v =
                when {
                    t < t2 -> -SPEED / 2
                    t < end -> -SPEED
                    t < end + t2 -> -SPEED / 2
                    else -> 0.0
                }
            var p = FROM
            p += -SPEED / 2 * minOf(t, t2)
            if (t > t2) p += -SPEED * (minOf(t, end) - t2)
            if (t > end) p += -SPEED / 2 * (minOf(t, end + t2) - end)
            doubleArrayOf(maxOf(TO, p), v)
        }
    }

    /** Accel-limited onset/stop at aLim. */
    private fun soft(aLim: Double): Ramp {
        val tAcc = SPEED / aLim
        val dAcc = 0.5 * aLim * tAcc * tAcc
        val cruise = (FROM - TO - 2 * dAcc) / SPEED
        return Ramp { t ->
            val v: Double
            val p: Double
            when {
                t < tAcc -> {
                    v = -aLim * t
                    p = FROM - 0.5 * aLim * t * t
                }
                t < tAcc + cruise -> {
                    v = -SPEED
                    p = FROM - dAcc - SPEED * (t - tAcc)
                }
                t < 2 * tAcc + cruise -> {
                    val s = t - tAcc - cruise
                    v = -SPEED + aLim * s
                    p = FROM - dAcc - SPEED * cruise - SPEED * s + 0.5 * aLim * s * s
                }
                else -> {
                    v = 0.0
                    p = TO
                }
            }
            doubleArrayOf(maxOf(TO, p), v)
        }
    }

    /**
     * Soft-handoff ramp (the `ArmBacklashKeepEngagedTest` recipe, adapted): creep at crossVel until
     * the setpoint is holdDeg past the zero-gravity crest so the lash crossing lands gently and
     * gravity loads the mesh, then accel-limited ramp up to full speed.
     */
    private fun handoff(crossVel: Double, holdDeg: Double, aLim: Double): Ramp {
        return object : Ramp {
            var sp = FROM
            var v = 0.0

            override fun at(t: Double): DoubleArray {
                var vmax = if (sp > Math.toRadians(90 - holdDeg)) crossVel else SPEED
                vmax = minOf(vmax, sqrt(2 * aLim * maxOf(0.0, sp - TO))) // stop at TO
                v = minOf(vmax, v + aLim * DT)
                sp = maxOf(TO, sp - v * DT)
                return doubleArrayOf(sp, if (sp <= TO) 0.0 else -v)
            }
        }
    }

    // ── one descent: returns {osc std rad/s, max rebound rad/s, separations} ────
    private fun run(label: String, ramp: Ramp, pdOnTip: Boolean, kTwist: Double): DoubleArray {
        val sim =
            FlexArmMotorSim(
                KS,
                KG,
                KV,
                KA,
                28,
                100.0,
                0.0,
                MIN,
                MAX,
                FROM,
                LASH,
                FLEX_HZ,
                FLEX_ZETA,
            )
        val obs = Observer(FROM)
        val kP = 12.0
        val kD = 0.8

        var sumSq = 0.0
        var n = 0
        var maxRebound = 0.0
        var seps = 0
        var wasEng = true
        var lastU = KG * cos(FROM)
        val moveEnd = (FROM - TO) / SPEED + 0.8

        var t = 0.0
        while (t < moveEnd + 1.0) {
            obs.predict(lastU, DT)
            obs.correct(sim.getMotorPositionRad(), sim.getMotorVelocityRadPerSec())

            val s = ramp.at(t)
            val sp = s[0]
            val spv = s[1]
            var u: Double
            if (pdOnTip) {
                u = kP * (sp - obs.tt) + kD * (spv - obs.wt) + KG * cos(obs.tt) + KV * spv
            } else {
                u =
                    kP * (sp - sim.getMotorPositionRad()) +
                        kD * (0 - sim.getMotorVelocityRadPerSec()) +
                        KG * cos(sim.getMotorPositionRad()) +
                        KV * spv
            }
            u += kTwist * (obs.wh - obs.wt) // twist-rate (resonance) damping
            u = maxOf(-VBUS, minOf(VBUS, u))
            lastU = u

            sim.step(DT, u / VBUS, VBUS)

            val eng = sim.isEngaged
            val moving = abs(spv) > 0.1
            if (moving && wasEng && !eng) seps++
            wasEng = eng
            if (moving && t > 0.3) {
                val dev = sim.getTrueVelocityRadPerSec() - spv
                sumSq += dev * dev
                n++
                maxRebound = maxOf(maxRebound, sim.getTrueVelocityRadPerSec())
            }
            t += DT
        }
        val osc = if (n == 0) 0.0 else sqrt(sumSq / n)
        System.out.printf(
            "%-44s osc=%5.2f rad/s  maxRebound=%+5.2f  seps=%2d%n",
            label,
            osc,
            maxRebound,
            seps,
        )
        return doubleArrayOf(osc, maxRebound, seps.toDouble())
    }

    @Test
    fun compareStrategies() {
        println("Descent 90 -> -40 at 1.2 rad/s on FlexArmMotorSim (3 Hz, zeta .03, 5 deg lash)")
        val baseline = run("A  baseline instant ramp, PD-on-motor", instant(), false, 0.0)
        val zv = run("B  ZV-shaped ramp, PD-on-motor", zvShaped(), false, 0.0)
        run("C  soft ramp (3 rad/s^2), PD-on-motor", soft(3.0), false, 0.0)
        val tipPd = run("D  soft ramp, PD-on-estimated-TIP", soft(3.0), true, 0.0)
        val twistOk = run("E  instant ramp + twist damping kT=1.0", instant(), false, 1.0)
        val twistHot = run("F  instant ramp + twist damping kT=3.0", instant(), false, 3.0)
        val handoff =
            run("G  soft-handoff ramp (creep 0.3 to 80 deg)", handoff(0.3, 10.0, 3.0), false, 0.0)

        // Finding 1: the soft-handoff profile roughly halves the bounce.
        assertTrue(
            handoff[0] < 0.6 * baseline[0],
            "soft handoff should cut the bounce to under 60% of baseline (baseline=" +
                "${baseline[0]}, handoff=${handoff[0]})",
        )
        assertTrue(
            handoff[2] <= 1,
            "soft handoff should keep the mesh loaded after the crossing",
        )

        // Finding 2: input shaping does not help — the excitation is the lash impact, not the
        // command step.
        assertTrue(
            zv[0] > 0.8 * baseline[0],
            "ZV shaping should not materially beat baseline (baseline=${baseline[0]}" +
                ", zv=${zv[0]})",
        )

        // Finding 3: even a perfect-model estimator barely helps (non-collocated ceiling)...
        assertTrue(
            tipPd[0] > 0.8 * baseline[0],
            "PD on the estimated tip should not materially beat baseline (baseline=" +
                "${baseline[0]}, tipPd=${tipPd[0]})",
        )
        assertTrue(
            twistOk[0] > 0.75 * baseline[0],
            "twist damping at its best gain should help under 25% (baseline=${baseline[0]}" +
                ", twist=${twistOk[0]})",
        )

        // ...and pushing the damping gain destabilizes the loop through the lash.
        assertTrue(
            twistHot[0] > 1.5 * baseline[0],
            "aggressive twist damping should go unstable through the lash (baseline=" +
                "${baseline[0]}, hot=${twistHot[0]})",
        )
    }
}
