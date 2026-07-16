package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.parallel.Execution;
import org.junit.jupiter.api.parallel.ExecutionMode;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Headless quality gate for the Lineage B setpoint profiler ({@code ModelAwareRuckigProfiler},
 * per-loop OTG replanning) on the full ControlLab arm stack (EKF + PIDF + back-EMF ceilings
 * against the backlash and flex plants).
 *
 * <p>The headline metric is <b>peak setpoint jerk</b>, measured as {@code |Δa_setpoint| / dt} per
 * tick: the OTG stop is planned, so it must stay near the configured limit through arrivals and
 * mid-flight retargets. Plant-side quality (settle, final error) is asserted absolutely.
 */
@Execution(ExecutionMode.SAME_THREAD)
class MechanismProfilerTest {

    private static final long SEED = 42L;

    // The profile limits and kD this scenario suite was designed around, pinned explicitly so it
    // is insulated from the mechanism defaults: the flex-shaped defaults (a8/j24, kD 4)
    // deliberately trade retarget agility for arrival ringdown — FlexRingTuningTest's subject,
    // not this test's.
    private static final double PIN_KD = 1.5;
    private static final double PIN_VMAX = 8.0;
    private static final double PIN_AMAX = 12.0;
    private static final double PIN_JMAX = 480.0;

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
     * plant the load parks ~2.5–3.3° from target (it rests against the gravity side of the lash),
     * so the 2° latch never fires and tells us nothing. 4° is outside the lash sag and cleanly
     * separates "arrived" from "still moving".
     */
    private static final double SETTLE_BAND_DEG = 4.0;

    private static final class MoveResult {
        String name;
        double settleSec;      // time until the load last entered the 4° band (NaN = never)
        double finalErrDeg;
        double peakTrajJerk;   // rad/s^3, from setpoint accel differences
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

    private static List<MoveResult> run(ArmEngine.PlantKind plant, List<Segment> script) {
        ArmEngine engine = new ArmEngine(ArmControllerType.MECHANISM_PIDF, SEED);
        MechanismArmAdapter.Gains g = engine.getMechGains();
        engine.setMechanismGains(g.kP, g.kI, PIN_KD, g.kS, g.kV, g.kA, g.kCos, g.kSin,
                PIN_VMAX, PIN_AMAX, PIN_AMAX, PIN_JMAX);
        if (plant != engine.getPlantKind()) {
            engine.setPlantKind(plant);
        }

        List<MoveResult> results = new ArrayList<>();
        for (Segment seg : script) {
            engine.setTargetRad(Math.toRadians(seg.targetDeg));
            double segStart = engine.getElapsedSec();
            double prevA = engine.getTrajAccelRad();
            double prevT = engine.getElapsedSec();
            double peakJerk = 0.0;
            double lastOutsideBand = segStart; // time the load was last outside the settle band
            for (int i = 0; i < seg.ticks; i++) {
                engine.tick();
                double dt = engine.getElapsedSec() - prevT;
                double a = engine.getTrajAccelRad();
                peakJerk = Math.max(peakJerk, Math.abs(a - prevA) / dt);
                double errDeg = Math.abs(Math.toDegrees(
                        engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg)));
                if (errDeg > SETTLE_BAND_DEG) {
                    lastOutsideBand = engine.getElapsedSec();
                }
                prevA = a;
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
            r.finalErrDeg = Math.abs(Math.toDegrees(
                    engine.getTrueLoadRad() - Math.toRadians(seg.targetDeg)));
            r.peakTrajJerk = peakJerk;
            results.add(r);
        }
        return results;
    }

    @Test
    void setpointStaysJerkBoundedWhileTrackingWell() {
        List<MoveResult> backlash = run(ArmEngine.PlantKind.BACKLASH, backlashScript());
        List<MoveResult> flex = run(ArmEngine.PlantKind.FLEX, flexScript());

        printTable("BACKLASH plant", backlash);
        printTable("FLEX plant", flex);

        for (MoveResult r : backlash) {
            assertTrue(!Double.isNaN(r.settleSec), "settles: " + r.name);
            assertTrue(r.finalErrDeg < 4.0, "final error " + r.name + ": " + r.finalErrDeg);
            // The OTG setpoint respects the configured jerk limit through its stops (small slack
            // for dt jitter across replan boundaries).
            assertTrue(r.peakTrajJerk <= PIN_JMAX * 1.15,
                    "setpoint jerk bounded on " + r.name + ": " + r.peakTrajJerk
                            + " vs limit " + PIN_JMAX);
        }

        // Flex plant: no settle-time assertion (structural ringing), but every move must arrive
        // and stay jerk-bounded. The flex descent parks ~6° off target (lash + flex sag), so
        // "arrives" is an 8° bound there.
        for (MoveResult r : flex) {
            assertTrue(r.finalErrDeg < 8.0, "flex arrives: " + r.name);
            assertTrue(r.peakTrajJerk <= PIN_JMAX * 1.15, "flex jerk bounded on " + r.name);
        }
    }

    @Test
    void survivesCollapsedBackEmfCeilings() {
        // With a high model kV (a plausible SysID result) the back-EMF velocity ceiling sits far
        // below the configured vMax, so the profile cruises pinned to a position-dependent
        // ceiling that is rewritten every loop while the accel ceiling reads zero. Replans fail
        // from that state, and the profiler must clamp into the fresh bounds and replan rather
        // than hold: a held out-of-band state reproduces the same failing inputs every loop,
        // freezing the profile mid-flight.
        ArmEngine e = new ArmEngine(ArmControllerType.MECHANISM_PIDF, 42L);
        MechanismArmAdapter.Gains g = e.getMechGains();
        e.setMechanismGains(g.kP, g.kI, g.kD, g.kS, 4.0 /* kV */, g.kA, g.kCos, g.kSin,
                g.maxVel, g.maxAccel, g.maxDecel, g.maxJerk);

        for (double targetDeg : new double[] {90, 0, 90, 180}) {
            double target = Math.toRadians(targetDeg);
            e.setTargetRad(target);
            for (int i = 0; i < 900; i++) {
                e.tick();
            }
            // Score the profile against the endpoint it actually chases — with the backlash
            // plant live, rest-only compensation biases that up to a half-lash off the stated
            // target.
            double profErrDeg = Math.abs(Math.toDegrees(e.getTrajPosRad() - e.getProfileTargetRad()));
            assertTrue(profErrDeg < 1.0,
                    "profile arrives at " + targetDeg + "° under collapsed ceilings; err="
                            + profErrDeg + "°");
        }
    }

    private static void printTable(String header, List<MoveResult> results) {
        System.out.println();
        System.out.println("=== " + header + " — ModelAwareRuckigProfiler ===");
        System.out.printf(Locale.US, "%-28s %13s %13s %13s%n",
                "move", "settle (s)", "final err(°)", "peak jerk");
        for (MoveResult r : results) {
            System.out.printf(Locale.US, "%-28s %13.2f %13.2f %13.0f%n",
                    r.name, r.settleSec, r.finalErrDeg, r.peakTrajJerk);
        }
        System.out.println("(peak jerk in rad/s^3 against the configured limit " + PIN_JMAX + ")");
    }
}
