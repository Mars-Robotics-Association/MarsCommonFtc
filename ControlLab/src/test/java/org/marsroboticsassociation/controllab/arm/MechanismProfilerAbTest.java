package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Headless A/B of the two Lineage B setpoint profilers on the full ControlLab arm stack
 * (EKF + PIDF + back-EMF ceilings against the backlash and flex plants):
 * {@code CascadedRateLimiter} (greedy one-step filter) vs {@code RuckigProfiler} (per-loop OTG
 * replanning with conservative braking).
 *
 * <p>Both variants run the identical scenario script with the same RNG seed, so they see the
 * same dt-jitter stream. The headline metric is <b>peak setpoint jerk</b>, measured as
 * {@code |Δa_setpoint| / dt} per tick: the cascade's documented late-braking clamps show up as
 * jerk spikes far above the configured limit; the OTG stop is planned and must stay near it.
 * Plant-side quality (settle, overshoot, final error) must not regress.
 */
@Execution(ExecutionMode.SAME_THREAD)
class MechanismProfilerAbTest {

    private static final long SEED = 42L;

    /** One commanded move plus how long to run it, in ticks (~16 ms each). */
    private static final class Segment {
        final String name;
        final double targetDeg;
        final int ticks;
        final boolean measured; // false = staging move, excluded from metrics/assertions

        Segment(String name, double targetDeg, int ticks, boolean measured) {
            this.name = name;
            this.targetDeg = targetDeg;
            this.ticks = ticks;
            this.measured = measured;
        }
    }

    /**
     * Settle band for the harness's own settle metric. ArmMetrics uses 2°, but on the backlash
     * plant both profilers park the load ~2.5–3.3° from target (it rests against the gravity side
     * of the lash), so the 2° latch never fires and tells us nothing. 4° is outside the lash sag
     * and cleanly separates "arrived" from "still moving".
     */
    private static final double SETTLE_BAND_DEG = 4.0;

    private static final class MoveResult {
        String name;
        double settleSec;      // time until the load last entered the 4° band (NaN = never)
        double overshootDeg;
        double finalErrDeg;
        double peakTrajJerk;   // rad/s^3, from setpoint accel differences
        double peakPowerSlew;  // 1/s, |Δpower|/dt
    }

    // Workspace: min −45°, max +225°; engine parks at 225°.
    private static List<Segment> backlashScript() {
        List<Segment> s = new ArrayList<>();
        s.add(new Segment("big move 225→90", 90, 320, true));
        s.add(new Segment("gravity descent 90→0", 0, 320, true));
        s.add(new Segment("small step 0→10", 10, 200, true));
        s.add(new Segment("retarget staging 10→100", 100, 30, false)); // interrupted mid-flight
        s.add(new Segment("retarget mid-flight →45", 45, 320, true));
        return s;
    }

    private static List<Segment> flexScript() {
        List<Segment> s = new ArrayList<>();
        s.add(new Segment("flex: big move 225→90", 90, 320, true));
        s.add(new Segment("flex: gravity descent 90→0", 0, 320, true));
        return s;
    }

    private static List<MoveResult> run(boolean ruckig, ArmEngine.PlantKind plant,
                                        List<Segment> script) {
        ArmEngine engine = new ArmEngine(ArmControllerType.MECHANISM_PIDF, SEED);
        engine.setMechanismProfiler(ruckig);
        if (plant != engine.getPlantKind()) {
            engine.setPlantKind(plant);
        }

        List<MoveResult> results = new ArrayList<>();
        for (Segment seg : script) {
            engine.setTargetRad(Math.toRadians(seg.targetDeg));
            double segStart = engine.getElapsedSec();
            double prevA = engine.getTrajAccelRad();
            double prevPower = engine.getCommandedPower();
            double prevT = engine.getElapsedSec();
            double peakJerk = 0.0;
            double peakSlew = 0.0;
            double lastOutsideBand = segStart; // time the load was last outside the settle band
            for (int i = 0; i < seg.ticks; i++) {
                engine.tick();
                double dt = engine.getElapsedSec() - prevT;
                double a = engine.getTrajAccelRad();
                double power = engine.getCommandedPower();
                peakJerk = Math.max(peakJerk, Math.abs(a - prevA) / dt);
                peakSlew = Math.max(peakSlew, Math.abs(power - prevPower) / dt);
                double errDeg = Math.abs(Math.toDegrees(
                        engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg)));
                if (errDeg > SETTLE_BAND_DEG) {
                    lastOutsideBand = engine.getElapsedSec();
                }
                prevA = a;
                prevPower = power;
                prevT = engine.getElapsedSec();
            }
            if (!seg.measured) {
                continue;
            }
            MoveResult r = new MoveResult();
            r.name = seg.name;
            boolean insideAtEnd = Math.abs(Math.toDegrees(
                    engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg))) <= SETTLE_BAND_DEG;
            r.settleSec = insideAtEnd ? lastOutsideBand - segStart : Double.NaN;
            r.overshootDeg = engine.getMetrics().overshootDeg();
            r.finalErrDeg = Math.abs(Math.toDegrees(
                    engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg)));
            r.peakTrajJerk = peakJerk;
            r.peakPowerSlew = peakSlew;
            results.add(r);
        }
        return results;
    }

    @Test
    void ruckigProfilerBeatsCascadeOnSetpointSmoothnessWithoutRegressingTracking() {
        double configuredJerk = new MechanismArmAdapter.Gains().maxJerk; // 480 rad/s^3

        List<MoveResult> cascadeBk = run(false, ArmEngine.PlantKind.BACKLASH, backlashScript());
        List<MoveResult> ruckigBk = run(true, ArmEngine.PlantKind.BACKLASH, backlashScript());
        List<MoveResult> cascadeFx = run(false, ArmEngine.PlantKind.FLEX, flexScript());
        List<MoveResult> ruckigFx = run(true, ArmEngine.PlantKind.FLEX, flexScript());

        printTable("BACKLASH plant", cascadeBk, ruckigBk);
        printTable("FLEX plant", cascadeFx, ruckigFx);

        double cascadeWorstJerk = 0;
        double ruckigWorstJerk = 0;
        for (int i = 0; i < cascadeBk.size(); i++) {
            MoveResult c = cascadeBk.get(i);
            MoveResult r = ruckigBk.get(i);
            cascadeWorstJerk = Math.max(cascadeWorstJerk, c.peakTrajJerk);
            ruckigWorstJerk = Math.max(ruckigWorstJerk, r.peakTrajJerk);

            // Neither variant may fail to arrive.
            assertTrue(!Double.isNaN(c.settleSec), "cascade settles: " + c.name);
            assertTrue(!Double.isNaN(r.settleSec), "ruckig settles: " + r.name);
            assertTrue(r.finalErrDeg < 4.0, "ruckig final error " + r.name + ": " + r.finalErrDeg);
            // Tracking must not regress meaningfully: allow 25% + 0.15 s slack on settle...
            assertTrue(r.settleSec <= c.settleSec * 1.25 + 0.15,
                    "ruckig settle not much worse on " + c.name
                            + ": cascade=" + c.settleSec + " ruckig=" + r.settleSec);
            // ...and the planned stop should not overshoot more than the clamped one.
            assertTrue(r.overshootDeg <= c.overshootDeg + 0.5,
                    "ruckig overshoot not worse on " + c.name
                            + ": cascade=" + c.overshootDeg + " ruckig=" + r.overshootDeg);
        }

        // The headline: the OTG setpoint respects the configured jerk limit through its stops;
        // the cascade's clamped stops do not.
        assertTrue(ruckigWorstJerk <= configuredJerk * 1.15,
                "ruckig setpoint jerk bounded: " + ruckigWorstJerk + " vs limit " + configuredJerk);
        assertTrue(ruckigWorstJerk < cascadeWorstJerk,
                "ruckig smoother than cascade: " + ruckigWorstJerk + " vs " + cascadeWorstJerk);

        // Flex plant: no settle-time assertion (structural ringing), but both must arrive and the
        // Ruckig setpoint must stay jerk-bounded there too.
        // The flex descent parks ~6° off target on both variants (lash + flex sag), so "arrives"
        // is an 8° bound there.
        for (int i = 0; i < cascadeFx.size(); i++) {
            assertTrue(ruckigFx.get(i).finalErrDeg < 8.0, "ruckig flex arrives: " + ruckigFx.get(i).name);
            assertTrue(cascadeFx.get(i).finalErrDeg < 8.0, "cascade flex arrives: " + cascadeFx.get(i).name);
            assertTrue(ruckigFx.get(i).peakTrajJerk <= configuredJerk * 1.15,
                    "ruckig flex jerk bounded on " + ruckigFx.get(i).name);
        }
    }

    private static void printTable(String header, List<MoveResult> cascade, List<MoveResult> ruckig) {
        System.out.println();
        System.out.println("=== " + header + " — CascadedRateLimiter vs RuckigProfiler ===");
        System.out.printf(Locale.US, "%-28s %13s %13s %13s %13s %15s%n",
                "move", "settle (s)", "overshoot(°)", "final err(°)", "peak jerk", "power slew(/s)");
        for (int i = 0; i < cascade.size(); i++) {
            MoveResult c = cascade.get(i);
            MoveResult r = ruckig.get(i);
            System.out.printf(Locale.US, "%-28s %6.2f→%6.2f %6.2f→%6.2f %6.2f→%6.2f %6.0f→%6.0f %7.1f→%7.1f%n",
                    c.name,
                    c.settleSec, r.settleSec,
                    c.overshootDeg, r.overshootDeg,
                    c.finalErrDeg, r.finalErrDeg,
                    c.peakTrajJerk, r.peakTrajJerk,
                    c.peakPowerSlew, r.peakPowerSlew);
        }
        System.out.println("(each cell: cascade→ruckig; peak jerk in rad/s^3 against a 480 limit)");
    }
}
