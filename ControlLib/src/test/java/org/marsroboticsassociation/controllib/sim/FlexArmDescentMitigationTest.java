package org.marsroboticsassociation.controllib.sim;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Experiment: which controller/estimator strategies smooth the flexible-arm descent bounce on
 * {@link FlexArmMotorSim}? Companion to {@code ArmBacklashKeepEngagedTest} (which studied the rigid
 * backlash plant); here the load carries a lightly damped structural flex mode behind the lash.
 *
 * <p>Findings this harness demonstrates (see the printed table; asserted in
 * {@link #compareStrategies}):
 *
 * <ul>
 *   <li><b>The bounce is seeded by one event: the lash handoff where gravity preload is near
 *       zero.</b> Starting a descent from upright (cos θ ≈ 0), the teeth carry nothing; the arm
 *       free-falls through the gap and that single impact rings the flex mode for the next
 *       second-plus. Once gravity loads the mesh, a smooth profile keeps it loaded all the way down
 *       and the mode stays quiet.
 *   <li><b>The soft-handoff profile wins.</b> Creeping across the zero-gravity zone (0.3 rad/s for
 *       the first ~10°) before opening up to full speed roughly halves the bounce metric (0.37 →
 *       0.17 rad/s) and removes the visible ring, for ~0.4 s of extra descent time. This is the
 *       same keep-engaged recipe {@code HandoffAwareArmModel} injects through the mechanism model's
 *       ceiling hooks.
 *   <li><b>Input shaping — the textbook flexible-mode answer — does nothing here</b> (0.38 vs
 *       0.37): the excitation is not the command step but the lash impact, which a shaper cannot
 *       schedule away.
 *   <li><b>No estimator rescues this.</b> Even a perfect-model three-inertia observer buys almost
 *       nothing: PD on the estimated tip state is no better than PD on the motor (0.36 vs 0.37),
 *       and twist-rate (resonance) damping helps ~10% at its best gain, then goes violently
 *       unstable at 3× that gain (osc 2.3 rad/s, 11 lash separations). This is the classic
 *       non-collocated-control ceiling: the actuator sits on the wrong side of both the lash and
 *       the flex spring, and the mesh unloads exactly when the controller needs to push. The
 *       problem is not knowing where the tip is — it is not being able to grab it.
 * </ul>
 *
 * <p>Past the profile fix, the remaining levers are hardware: less lash, a stiffer arm (higher
 * flex frequency), or tip-side sensing <em>with tip-side actuation</em>.
 */
class FlexArmDescentMitigationTest {

    static final double KS = 0.3, KG = 3.5, KV = 1.2, KA = 0.35;
    static final double MIN = Math.toRadians(-45), MAX = Math.toRadians(225);
    static final double LASH = Math.toRadians(5), FLEX_HZ = 3.0, FLEX_ZETA = 0.03;
    static final double VBUS = 12.0, DT = 0.016;
    static final double FROM = Math.toRadians(90), TO = Math.toRadians(-40), SPEED = 1.2;

    // ── observer: replica of the plant model (perfect params — the estimator's best case),
    //    corrected on the motor states each loop ─────────────────────────────────────────────
    private static class Observer {
        final double kAm = 0.50 * KA, kAh = 0.05 * KA, kAt = 0.45 * KA;
        final double gTip = 0.85, kc = 500, cc = 2.0;
        final double kf = Math.pow(2 * Math.PI * FLEX_HZ, 2) * kAt;
        final double cf = 2 * FLEX_ZETA * Math.sqrt(kf * kAt);
        double tm, wm, th, wh, tt, wt;

        Observer(double start) {
            tt = start;
            th = start + gTip * KG * Math.cos(start) / kf;
            double hold = gTip * KG * Math.cos(start) + (1 - gTip) * KG * Math.cos(th);
            tm = th + hold / kc + Math.signum(hold) * LASH / 2;
        }

        double contact() {
            double d = tm - th, h = LASH / 2;
            if (d > h)  return kc * (d - h) + cc * (wm - wh);
            if (d < -h) return kc * (d + h) + cc * (wm - wh);
            return 0;
        }

        void predict(double u, double dt) {
            for (double r = 0; r < dt - 1e-12; r += 0.0005) {
                double tauC = contact();
                double tauF = kf * (th - tt) + cf * (wh - wt);
                wm += 0.0005 * (u - KV * wm - KS * Math.signum(wm) - tauC) / kAm;
                wh += 0.0005 * (tauC - tauF - (1 - gTip) * KG * Math.cos(th)) / kAh;
                wt += 0.0005 * (tauF - gTip * KG * Math.cos(tt)) / kAt;
                tm += 0.0005 * wm; th += 0.0005 * wh; tt += 0.0005 * wt;
            }
        }

        void correct(double posMeas, double velMeas) {
            tm += 0.6 * (posMeas - tm);
            wm += 0.6 * (velMeas - wm);
        }
    }

    // ── setpoint generators (called once per DT, in order) ──────────────────────
    private interface Ramp { double[] at(double t); } // returns {sp, spv}

    private static Ramp instant() {
        double end = (TO - FROM) / -SPEED;
        return t -> {
            if (t >= end) return new double[]{TO, 0};
            return new double[]{FROM - SPEED * t, -SPEED};
        };
    }

    /** ZV shaper: velocity steps of half amplitude, half a flex period apart. */
    private static Ramp zvShaped() {
        double t2 = 1.0 / (2 * FLEX_HZ);
        double end = (TO - FROM + SPEED * t2 / 2) / -SPEED;
        return t -> {
            double v;
            if (t < t2) v = -SPEED / 2;
            else if (t < end) v = -SPEED;
            else if (t < end + t2) v = -SPEED / 2;
            else v = 0;
            double p = FROM;
            p += -SPEED / 2 * Math.min(t, t2);
            if (t > t2) p += -SPEED * (Math.min(t, end) - t2);
            if (t > end) p += -SPEED / 2 * (Math.min(t, end + t2) - end);
            return new double[]{Math.max(TO, p), v};
        };
    }

    /** Accel-limited onset/stop at aLim. */
    private static Ramp soft(double aLim) {
        double tAcc = SPEED / aLim;
        double dAcc = 0.5 * aLim * tAcc * tAcc;
        double cruise = (FROM - TO - 2 * dAcc) / SPEED;
        return t -> {
            double v, p;
            if (t < tAcc) { v = -aLim * t; p = FROM - 0.5 * aLim * t * t; }
            else if (t < tAcc + cruise) { v = -SPEED; p = FROM - dAcc - SPEED * (t - tAcc); }
            else if (t < 2 * tAcc + cruise) {
                double s = t - tAcc - cruise;
                v = -SPEED + aLim * s;
                p = FROM - dAcc - SPEED * cruise - SPEED * s + 0.5 * aLim * s * s;
            } else { v = 0; p = TO; }
            return new double[]{Math.max(TO, p), v};
        };
    }

    /**
     * Soft-handoff ramp (the {@code ArmBacklashKeepEngagedTest} recipe, adapted): creep at crossVel
     * until the setpoint is holdDeg past the zero-gravity crest so the lash crossing lands gently
     * and gravity loads the mesh, then accel-limited ramp up to full speed.
     */
    private static Ramp handoff(double crossVel, double holdDeg, double aLim) {
        return new Ramp() {
            double sp = FROM, v = 0;
            @Override public double[] at(double t) {
                double vmax = sp > Math.toRadians(90 - holdDeg) ? crossVel : SPEED;
                vmax = Math.min(vmax, Math.sqrt(2 * aLim * Math.max(0, sp - TO))); // stop at TO
                v = Math.min(vmax, v + aLim * DT);
                sp = Math.max(TO, sp - v * DT);
                return new double[]{sp, sp <= TO ? 0 : -v};
            }
        };
    }

    // ── one descent: returns {osc std rad/s, max rebound rad/s, separations} ────
    private double[] run(String label, Ramp ramp, boolean pdOnTip, double kTwist) {
        FlexArmMotorSim sim = new FlexArmMotorSim(KS, KG, KV, KA, 28, 100.0, 0.0,
                MIN, MAX, FROM, LASH, FLEX_HZ, FLEX_ZETA);
        Observer obs = new Observer(FROM);
        double kP = 12.0, kD = 0.8;

        double sumSq = 0; int n = 0;
        double maxRebound = 0;
        int seps = 0; boolean wasEng = true;
        double lastU = KG * Math.cos(FROM);
        double moveEnd = (FROM - TO) / SPEED + 0.8;

        for (double t = 0; t < moveEnd + 1.0; t += DT) {
            obs.predict(lastU, DT);
            obs.correct(sim.getMotorPositionRad(), sim.getMotorVelocityRadPerSec());

            double[] s = ramp.at(t);
            double sp = s[0], spv = s[1];
            double u;
            if (pdOnTip) {
                u = kP * (sp - obs.tt) + kD * (spv - obs.wt) + KG * Math.cos(obs.tt) + KV * spv;
            } else {
                u = kP * (sp - sim.getMotorPositionRad()) + kD * (0 - sim.getMotorVelocityRadPerSec())
                        + KG * Math.cos(sim.getMotorPositionRad()) + KV * spv;
            }
            u += kTwist * (obs.wh - obs.wt);   // twist-rate (resonance) damping
            u = Math.max(-VBUS, Math.min(VBUS, u));
            lastU = u;

            sim.step(DT, u / VBUS, VBUS);

            boolean eng = sim.isEngaged();
            boolean moving = Math.abs(spv) > 0.1;
            if (moving && wasEng && !eng) seps++;
            wasEng = eng;
            if (moving && t > 0.3) {
                double dev = sim.getTrueVelocityRadPerSec() - spv;
                sumSq += dev * dev; n++;
                maxRebound = Math.max(maxRebound, sim.getTrueVelocityRadPerSec());
            }
        }
        double osc = n == 0 ? 0 : Math.sqrt(sumSq / n);
        System.out.printf("%-44s osc=%5.2f rad/s  maxRebound=%+5.2f  seps=%2d%n",
                label, osc, maxRebound, seps);
        return new double[]{osc, maxRebound, seps};
    }

    @Test
    void compareStrategies() {
        System.out.println(
                "Descent 90 -> -40 at 1.2 rad/s on FlexArmMotorSim (3 Hz, zeta .03, 5 deg lash)");
        double[] baseline = run("A  baseline instant ramp, PD-on-motor", instant(), false, 0);
        double[] zv       = run("B  ZV-shaped ramp, PD-on-motor", zvShaped(), false, 0);
        run("C  soft ramp (3 rad/s^2), PD-on-motor", soft(3.0), false, 0);
        double[] tipPd    = run("D  soft ramp, PD-on-estimated-TIP", soft(3.0), true, 0);
        double[] twistOk  = run("E  instant ramp + twist damping kT=1.0", instant(), false, 1.0);
        double[] twistHot = run("F  instant ramp + twist damping kT=3.0", instant(), false, 3.0);
        double[] handoff  = run("G  soft-handoff ramp (creep 0.3 to 80 deg)",
                handoff(0.3, 10, 3.0), false, 0);

        // Finding 1: the soft-handoff profile roughly halves the bounce.
        assertTrue(handoff[0] < 0.6 * baseline[0],
                "soft handoff should cut the bounce to under 60% of baseline (baseline="
                        + baseline[0] + ", handoff=" + handoff[0] + ")");
        assertTrue(handoff[2] <= 1, "soft handoff should keep the mesh loaded after the crossing");

        // Finding 2: input shaping does not help — the excitation is the lash impact, not the
        // command step.
        assertTrue(zv[0] > 0.8 * baseline[0],
                "ZV shaping should not materially beat baseline (baseline=" + baseline[0]
                        + ", zv=" + zv[0] + ")");

        // Finding 3: even a perfect-model estimator barely helps (non-collocated ceiling)...
        assertTrue(tipPd[0] > 0.8 * baseline[0],
                "PD on the estimated tip should not materially beat baseline (baseline="
                        + baseline[0] + ", tipPd=" + tipPd[0] + ")");
        assertTrue(twistOk[0] > 0.75 * baseline[0],
                "twist damping at its best gain should help under 25% (baseline=" + baseline[0]
                        + ", twist=" + twistOk[0] + ")");

        // ...and pushing the damping gain destabilizes the loop through the lash.
        assertTrue(twistHot[0] > 1.5 * baseline[0],
                "aggressive twist damping should go unstable through the lash (baseline="
                        + baseline[0] + ", hot=" + twistHot[0] + ")");
    }
}
