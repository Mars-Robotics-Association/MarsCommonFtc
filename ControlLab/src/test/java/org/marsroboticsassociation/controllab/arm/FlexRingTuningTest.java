package org.marsroboticsassociation.controllab.arm;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Experiment: can the arrival overshoot + ringing of the flexible arm be tuned out with the
 * profile limits, without load-side sensing?
 *
 * <p>Observed live (flight log armlab-20260716-090940, MECHANISM_PIDF + Ruckig on the flex plant,
 * vMax 8 / aMax 12 / jMax 60): on every big-move arrival the tip swings ~2.5 deg past the target
 * and rings at ~2 Hz for a couple of cycles. The log shows the mesh stays engaged and the motor
 * tracks the profile tightly while the tip lags it by up to ~9 deg during braking — the flex
 * spring winds up under deceleration ({@code delta = kA_tip * a / k_flex}, ~2 deg at a = 12), and
 * the stored energy releases as the ring when the profile lands.
 *
 * <p>The tuning lever: a jerk-limited profile is an input shaper. A constant-jerk ramp of duration
 * {@code t_j = aMax/jMax} leaves zero residual energy in an undamped mode when {@code t_j} equals
 * the mode's period (the ramp's excitation spectrum has a null there). The relevant period is not
 * the arm's free 3 Hz: while the mesh is engaged the tip rides the flex spring <em>in series
 * with</em> the contact spring and the position servo's stiffness, which lowers the mode to ~2 Hz
 * (0.5 s) — the frequency the log actually shows. Hence the candidates: jMax = aMax/0.5 s.
 *
 * <p>This harness replays the logged scenario (flex plant, same gains, over-the-top moves
 * -5 -> 177 -> -1 deg) across profile-limit variants and prints arrival metrics.
 *
 * <p><b>Findings</b> (asserted in {@link #sweepProfileLimitsForRingdown}):
 * <ul>
 *   <li>The logged config swings ~2.3 deg past the target on every arrival — reproducing the
 *       live observation. jMax 60 barely improves on unshaped j480 (2.9-3.2 deg) because its
 *       0.2 s ramp sits near the worst of the mode's excitation spectrum.
 *   <li><b>kD is the free lever.</b> Raising kD 1.5 -> 4 at the same limits halves the swing
 *       (~1.0 deg) at zero cost in move time: while the mesh is engaged, motor-side velocity
 *       damping does reach the coupled mode through the gears.
 *   <li><b>The jerk ramp is the real input shaper.</b> Stretching t_j into the 0.33-0.5 s band
 *       (the coupled mode's period range) plus modest accel cuts: a8/j16 or a6/j18 land at
 *       0.2-0.35 deg peak — a 7-10x reduction — for ~0.6 s more move time. a12/j24 + kD 4 is the
 *       compromise at ~0.6 deg for +0.36 s.
 * </ul>
 *
 * <p>These findings set the mechanism defaults: {@link MechanismArmAdapter.Gains} now ships
 * kD 4 with a8/j24 (t_j = 0.33 s).
 */
class FlexRingTuningTest {

    /** One profile/gain variant to score. */
    private static class Config {
        final String label;
        final double maxVel, maxAccel, maxJerk, kD;

        Config(String label, double maxVel, double maxAccel, double maxJerk, double kD) {
            this.label = label;
            this.maxVel = maxVel;
            this.maxAccel = maxAccel;
            this.maxJerk = maxJerk;
            this.kD = kD;
        }
    }

    /** Metrics for one arrival, measured from the moment the profile lands on its endpoint. */
    private static class Arrival {
        double moveSec;       // setTarget -> profile landing
        double peakDevDeg;    // max |tip - stated target| after landing (overshoot swing)
        double tipSettleSec;  // time after landing until the tip stays within the settle band
        double residP2PDeg;   // tip peak-to-peak over the last second of the watch window

        @Override public String toString() {
            return String.format(Locale.US,
                    "move=%4.2fs peak=%5.2fdeg settle=%4.2fs residP2P=%4.2fdeg",
                    moveSec, peakDevDeg, tipSettleSec, residP2PDeg);
        }
    }

    private static final double NOMINAL_DT = 0.016;
    private static final double SETTLE_BAND_DEG = 0.7;
    private static final double WATCH_SEC = 4.0;

    private ArmEngine makeEngine(Config c) {
        ArmEngine e = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L);
        e.setPlantKind(ArmEngine.PlantKind.FLEX);
        MechanismArmAdapter.Gains g = e.getMechGains();
        e.setMechanismGains(g.kP, g.kI, c.kD, g.kS, g.kV, g.kA, g.kCos, g.kSin,
                c.maxVel, c.maxAccel, c.maxAccel, c.maxJerk);
        return e;
    }

    /** Drive one move and score the arrival. */
    private Arrival runMove(ArmEngine e, double targetDeg) {
        e.setTargetDeg(targetDeg);
        double statedDeg = Math.toDegrees(e.getTargetRad()); // after hard-stop clamp

        Arrival a = new Arrival();

        // Phase 1: run until the profile lands on its (compensated) endpoint.
        int tick = 0;
        int maxTicks = (int) (20.0 / NOMINAL_DT);
        while (tick < maxTicks) {
            e.tick();
            tick++;
            boolean landed =
                    Math.abs(e.getTrajVelRad()) < 1e-4
                            && Math.abs(e.getTrajPosRad() - e.getProfileTargetRad())
                                    < Math.toRadians(0.05);
            if (landed) break;
        }
        a.moveSec = tick * NOMINAL_DT;

        // Phase 2: watch the tip ring down.
        int watchTicks = (int) (WATCH_SEC / NOMINAL_DT);
        List<Double> tip = new ArrayList<>(watchTicks);
        for (int i = 0; i < watchTicks; i++) {
            e.tick();
            tip.add(Math.toDegrees(e.getTrueLoadRad()));
        }

        double peak = 0.0;
        int lastOutside = -1;
        for (int i = 0; i < tip.size(); i++) {
            double dev = Math.abs(tip.get(i) - statedDeg);
            peak = Math.max(peak, dev);
            if (dev > SETTLE_BAND_DEG) lastOutside = i;
        }
        a.peakDevDeg = peak;
        a.tipSettleSec = (lastOutside + 1) * NOMINAL_DT;

        int lastSecond = (int) (1.0 / NOMINAL_DT);
        double mn = Double.POSITIVE_INFINITY, mx = Double.NEGATIVE_INFINITY;
        for (int i = tip.size() - lastSecond; i < tip.size(); i++) {
            mn = Math.min(mn, tip.get(i));
            mx = Math.max(mx, tip.get(i));
        }
        a.residP2PDeg = mx - mn;
        return a;
    }

    @Test
    void sweepProfileLimitsForRingdown() {
        // The mode to cancel: tip on the flex spring in series with contact + servo stiffness,
        // ~2 Hz observed (free tip-on-flex would be 3 Hz). t_j = aMax/jMax is the jerk-ramp
        // duration the sweep tunes toward those periods.
        Config[] configs = {
            new Config("unshaped a12 j480        ", 8.0, 12.0, 480.0, 1.5),
            new Config("logged   a12 j60  tj=0.20", 8.0, 12.0, 60.0, 1.5),
            new Config("3Hz-notch a12 j36 tj=0.33", 8.0, 12.0, 36.0, 1.5),
            new Config("2Hz-notch a12 j24 tj=0.50", 8.0, 12.0, 24.0, 1.5),
            new Config("slower    a6  j60 tj=0.10", 8.0, 6.0, 60.0, 1.5),
            new Config("3Hz-notch a6  j18 tj=0.33", 8.0, 6.0, 18.0, 1.5),
            new Config("2Hz-notch a6  j12 tj=0.50", 8.0, 6.0, 12.0, 1.5),
            new Config("3Hz-notch a8  j24 tj=0.33", 8.0, 8.0, 24.0, 1.5),
            new Config("2Hz-notch a8  j16 tj=0.50", 8.0, 8.0, 16.0, 1.5),
            new Config("logged + kD=4            ", 8.0, 12.0, 60.0, 4.0),
            new Config("2Hz-notch a12 j24 + kD=4 ", 8.0, 12.0, 24.0, 4.0),
            new Config("3Hz-notch a8  j24 + kD=4 ", 8.0, 8.0, 24.0, 4.0),
        };

        System.out.println("=== flex-arm arrival ringdown: -5 -> 177 -> -1 deg (flex plant, Ruckig) ===");
        System.out.printf(Locale.US, "%-26s | %-45s | %-45s%n",
                "config", "ascent arrival (177 deg)", "descent arrival (-1 deg)");
        List<Arrival[]> results = new ArrayList<>();
        for (Config c : configs) {
            ArmEngine e = makeEngine(c);
            e.setTargetDeg(-5.44);
            for (int i = 0; i < 500; i++) e.tick(); // park like the logged session
            Arrival up = runMove(e, 177.17);
            Arrival down = runMove(e, -0.87);
            results.add(new Arrival[] {up, down});
            System.out.printf(Locale.US, "%-26s | %s | %s%n", c.label, up, down);
        }

        double loggedPeak = worstPeak(configs, results, "logged   a12 j60");
        double loggedKd4Peak = worstPeak(configs, results, "logged + kD=4");
        double shapedPeak = worstPeak(configs, results, "3Hz-notch a6  j18");

        // Finding 1: the live observation reproduces — the logged limits ring well past the
        // target on arrival.
        assertTrue(loggedPeak > 1.8,
                "logged config should reproduce the observed arrival swing, got " + loggedPeak);
        // Finding 2: kD alone (same limits, same move time) roughly halves the swing.
        assertTrue(loggedKd4Peak < 0.6 * loggedPeak,
                "kD=4 should halve the arrival swing (logged=" + loggedPeak
                        + ", kD4=" + loggedKd4Peak + ")");
        // Finding 3: shaping the jerk ramp onto the mode period tunes the ring out.
        assertTrue(shapedPeak < 0.5,
                "a6/j18 (t_j on the mode period) should land clean, got " + shapedPeak);
    }

    /** The worse of the two arrivals' peak deviations for the config whose label starts so. */
    private static double worstPeak(Config[] configs, List<Arrival[]> results, String labelPrefix) {
        for (int i = 0; i < configs.length; i++) {
            if (configs[i].label.startsWith(labelPrefix)) {
                return Math.max(results.get(i)[0].peakDevDeg, results.get(i)[1].peakDevDeg);
            }
        }
        throw new AssertionError("no config labelled " + labelPrefix);
    }
}
