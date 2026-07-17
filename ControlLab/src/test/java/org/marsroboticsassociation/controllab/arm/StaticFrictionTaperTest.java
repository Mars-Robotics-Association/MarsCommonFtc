package org.marsroboticsassociation.controllab.arm;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.Locale;

/**
 * Regression for the overshoot that returned when a motor-side sysid through the lash + flex was
 * applied (flight log armlab-20260716-200720): the identified {@code kS} came out ~1.32 V (true
 * plant kS 0.3), and {@code kS·sign(v)} is an anti-braking term that fought every arrival — big
 * moves swung ~5 deg past target, versus sub-degree before the sysid.
 *
 * <p>The fix is the controller's static-friction taper ({@link
 * org.marsroboticsassociation.controllib.mechanism.MotorMechanismController#setStaticFrictionTaperVelocity}):
 * ramping the {@code kS} feedforward to zero as the profile decelerates into an arrival caps what
 * an over-estimated {@code kS} can do to braking. This replays the logged over-the-top moves on the
 * flex plant with the logged (post-sysid) gains and asserts the taper brings the arrival swing back
 * down, at no cost to move time.
 */
class StaticFrictionTaperTest {

    private static final double NOMINAL_DT = 0.016;
    private static final double SETTLE_BAND_DEG = 0.7;
    private static final double WATCH_SEC = 4.0;

    // The gains the flight log shows sysid applied (kS badly inflated by the lash + flex).
    private static final double KP = 40.0, KI = 8.0, KD = 4.0;
    private static final double KS = 1.318, KV = 1.178, KA = 0.3108, KCOS = 2.720, KSIN = 0.0;
    private static final double VMAX = 8.0, AMAX = 8.0, DMAX = 8.0, JMAX = 24.0;

    private ArmEngine makeEngine(double taperVel) {
        ArmEngine e = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L);
        e.setPlantKind(ArmEngine.PlantKind.FLEX);
        MechanismArmAdapter.Gains g = e.getMechGains();
        g.staticFrictionTaperVelocity = taperVel;
        e.setMechanismGains(KP, KI, KD, KS, KV, KA, KCOS, KSIN, VMAX, AMAX, DMAX, JMAX);
        return e;
    }

    /** Peak past-target tip swing after the profile lands, and the move time to landing. */
    private double[] runMove(ArmEngine e, double targetDeg) {
        e.setTargetDeg(targetDeg);
        double statedDeg = Math.toDegrees(e.getTargetRad());

        int tick = 0, maxTicks = (int) (20.0 / NOMINAL_DT);
        while (tick < maxTicks) {
            e.tick();
            tick++;
            boolean landed = Math.abs(e.getTrajVelRad()) < 1e-4
                    && Math.abs(e.getTrajPosRad() - e.getProfileTargetRad()) < Math.toRadians(0.05);
            if (landed) break;
        }
        double moveSec = tick * NOMINAL_DT;

        double peak = 0.0;
        int watchTicks = (int) (WATCH_SEC / NOMINAL_DT);
        for (int i = 0; i < watchTicks; i++) {
            e.tick();
            peak = Math.max(peak, Math.abs(Math.toDegrees(e.getTrueLoadRad()) - statedDeg));
        }
        return new double[] {peak, moveSec};
    }

    private double[] worstArrival(double taperVel) {
        ArmEngine e = makeEngine(taperVel);
        e.setTargetDeg(-5.44);
        for (int i = 0; i < 500; i++) e.tick(); // park like the logged session
        double[] up = runMove(e, 177.17);
        double[] down = runMove(e, -0.87);
        return new double[] {Math.max(up[0], down[0]), Math.max(up[1], down[1])};
    }

    @Test
    void taperCutsPostSysidArrivalSwing() {
        double[] off = worstArrival(0.0);  // taper disabled: reproduce the log
        double[] on = worstArrival(1.0);   // taper enabled (the arm default)
        System.out.printf(Locale.US,
                "post-sysid gains: taper off peak=%.2fdeg (move %.2fs), on peak=%.2fdeg (move %.2fs)%n",
                off[0], off[1], on[0], on[1]);

        // The inflated kS reproduces the logged arrival swing with the taper off.
        assertTrue(off[0] > 2.5,
                "inflated-kS gains should reproduce the logged arrival swing, got " + off[0]);
        // The taper neutralizes the kS anti-braking: swing drops by at least 30%. It plateaus here
        // (~2.5 deg) because the rest of the residual is the sysid's under-estimated kCos, not kS —
        // that half is the identification fix (a quasi-static kCos), not the taper's job.
        assertTrue(on[0] < 0.7 * off[0],
                "taper should cut the arrival swing by >=30% (off=" + off[0] + ", on=" + on[0] + ")");
        // At no cost to move time.
        assertTrue(on[1] <= off[1] + 0.05,
                "taper should not slow the move (off=" + off[1] + ", on=" + on[1] + ")");
    }

    /**
     * End to end: the whole point. Run the engine's own sysid through the flex plant, apply the
     * identified gains, then drive the logged over-the-top moves. With the quasi-static sweep (a
     * faithful kS/kCos instead of the lash-inflated ones) and the taper both live, the arrival swing
     * stays small — the regression the flight log showed is gone without any hand-set gains.
     */
    @Test
    void sysIdThenMoveIsClean() {
        ArmEngine e = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L);
        e.setPlantKind(ArmEngine.PlantKind.FLEX);
        ArmSysId.Result r = e.runSysId();
        e.applyIdentifiedGains(r);
        System.out.printf(Locale.US, "flex sysid applied: kS=%.3f kV=%.3f kA=%.3f kCos=%.3f%n",
                r.kS, r.kV, r.kA, r.kCos);

        e.setTargetDeg(-5.44);
        for (int i = 0; i < 500; i++) e.tick();
        double up = runMove(e, 177.17)[0];
        double down = runMove(e, -0.87)[0];
        double worst = Math.max(up, down);
        System.out.printf(Locale.US, "arrival swing after flex sysid: %.2fdeg%n", worst);

        assertTrue(worst < 1.5,
                "sysid through flex + taper should keep the arrival swing small, got " + worst);
    }
}
