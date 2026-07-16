package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim;

import java.util.Random;

/**
 * Experiment: can the over-the-top / descent bounce seen with the rigid-model {@link
 * MotorMechanismController} on the two-mass {@link BacklashArmMotorSim} be removed by keeping the
 * gear teeth <em>engaged</em> on the way down?
 *
 * <p>Findings this harness demonstrates (see the printed tables; asserted in
 * {@link #compareStrategies}):
 *
 * <ul>
 *   <li><b>A downward move from a gravity-loaded hold never opens the lash at all.</b> The holding
 *       face is the braking face, and the load (free-fall {@code kG·cosθ/kA_L} ≈ 37.5·cosθ rad/s²)
 *       presses onto the motor (profile ≤ 12 rad/s²) the whole way down. The bottom overshoot such
 *       moves show is reproduced bit-for-bit with the backlash set to zero: it is profile/tracking
 *       overshoot, a tuning problem, not a lash problem.
 *   <li><b>An over-the-top move has exactly one unavoidable handoff</b> at the crest, where the
 *       gravity torque changes sign. The load crosses the lash and lands on the braking face at
 *       {@code v ≈ sqrt(2·Δa·gap)}, where Δa is how far the motor's acceleration deviates from the
 *       load's gravity-only acceleration while the gap is open. Back-EMF caps the motor's downhill
 *       acceleration well below free-fall once it is moving a few rad/s, so a fast crossing always
 *       lands hard; the handoff must complete <em>slowly, near the crest</em>. The soft-handoff
 *       recipe, injected entirely through the {@link MechanismModel} ceiling hooks the controller
 *       already consults:
 *       <ul>
 *         <li>a <b>two-sided engagement ceiling</b>: acceleration ≤ {@code margin·|kG·cosθ|/kA_L}
 *             while gravity aids travel (outrunning free-fall pulls the motor off the load), and
 *             the same bound on <em>deceleration</em> while gravity opposes travel (braking harder
 *             than gravity brakes the load tosses it off the driving face);
 *         <li>evaluated against the <b>weakest gravity within a half-lash either side</b>, since
 *             an airborne load's position is only known to ±half-lash (zero if the crest lies in
 *             that interval);
 *         <li>an <b>approach ramp</b> that brakes toward a small crossing speed as the gravity
 *             torque decays before the crest, and an <b>exit hold</b> that keeps that speed for
 *             ~10° past it while the load falls the lash and lands.
 *       </ul>
 *       Landing speed drops from 1.98 to 0.68 rad/s (0.55 at the slowest setting) — roughly a
 *       ninth of the impact energy — for about double the descent time. The residual is close to
 *       the observability floor: the load sits anywhere within ±half-lash, which near the crest is
 *       ~±1.6 rad/s² of free-fall uncertainty, so ~0.5 rad/s is the gentlest landing that can be
 *       planned blind. Doing better needs load-side sensing (an output-shaft encoder) or less
 *       lash.
 *   <li><b>Hold quality dominates perceived bounce: any setpoint motion at the hold broadcasts
 *       through the feedforward as tooth-separating voltage chatter.</b> A profile that
 *       limit-cycles around the target (a few rad/s² of acceleration dither per loop reaches the
 *       feedforward as ±1.5 V) separates the teeth ~10 times a second at ±0.3 deg regardless of
 *       descent tuning; zero backlash removes the symptom but not the dither. {@link
 *       #holdChatterTrace} guards the requirement: the setpoint must land at rest and stay
 *       motionless (p2p ≈ 0), leaving at most one tooth unload, during the arrival-overshoot
 *       recovery.
 *   <li><b>Arrival overshoot is the profile's responsibility, not backlash's</b> (it reproduces
 *       with zero lash). {@link #arrivalTrace} watches the setpoint itself: a stop must charge
 *       the jerk-limited swing from full accelerate to full brake against the stopping distance —
 *       a braking plan that assumes it can begin instantly sails past the target by an error that
 *       grows with speed. Reversal overshoot is ~1.3 deg at an 8 rad/s velocity cap, the same as
 *       at 4 rad/s, so descent speed does not trade against arrival accuracy.
 * </ul>
 */
public class ArmBacklashKeepEngagedTest {

    private static final int TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 100.0;
    private static final double TICKS_PER_RAD = TICKS_PER_REV * GEAR_RATIO / (2 * Math.PI);
    private static final double K_S = 0.1, K_G = 1.5, K_V = 2.0, K_A = 0.2;
    private static final double VOLTAGE = 12.0;
    private static final double MIN_ANGLE_RAD = -Math.PI * 0.9;
    private static final double MAX_ANGLE_RAD = Math.PI * 0.9;
    private static final double BACKLASH_RAD = Math.toRadians(5.0);
    private static final double HALF_BACKLASH_RAD = BACKLASH_RAD / 2.0;
    private static final double HALF_BACKLASH_DEG = Math.toDegrees(HALF_BACKLASH_RAD);

    // The sim splits kA 80/20 between motor and load (BacklashArmMotorSim default).
    private static final double LOAD_INERTIA = 0.2 * K_A;

    /**
     * {@link ArmModel} whose acceleration ceiling additionally enforces tooth engagement: while
     * gravity aids the travel, cap acceleration to a fraction of the load's gravity-only (free-fall)
     * acceleration. Below that ceiling the motor can never outrun the falling load, so the load
     * either stays pressed on the braking face or closes an open gap gently.
     */
    private static class EngagementAwareArmModel extends ArmModel {
        private final double loadInertia; // kA of the load alone, V/(rad/s^2)
        private final double margin;      // fraction of free-fall accel the profile may use
        private final double floor;       // rad/s^2, keeps the profile from stalling at the crest

        EngagementAwareArmModel(double loadInertia, double margin, double floor) {
            super(K_S, K_V, K_A, K_G, 0.0);
            this.loadInertia = loadInertia;
            this.margin = margin;
            this.floor = floor;
        }

        /**
         * The gravity voltage to plan engagement against. Subclasses may return the weakest
         * gravity the load could see (it sits anywhere within the lash of the motor).
         */
        double engagementGravity(double position) {
            return gravityVoltage(position);
        }

        /** The engagement budget: how hard gravity can accelerate/brake the load by itself. */
        private double engagementCeiling(double position) {
            return Math.max(floor, margin * Math.abs(engagementGravity(position)) / loadInertia);
        }

        @Override
        public double maxSustainableAcceleration(
                double availableVoltage, double position, double velocity, double travelDirection) {
            double base =
                    super.maxSustainableAcceleration(
                            availableVoltage, position, velocity, travelDirection);
            double gravityAlongTravel = Math.signum(travelDirection) * gravityVoltage(position);
            if (gravityAlongTravel >= 0) {
                return base; // gravity opposes: load rests on the driving face, no separation risk
            }
            // Gravity aids: accelerating harder than the load can free-fall pulls the motor away
            // from it across the lash.
            return Math.min(base, engagementCeiling(position));
        }

        @Override
        public double maxSustainableDeceleration(
                double availableVoltage, double position, double velocity) {
            double base = super.maxSustainableDeceleration(availableVoltage, position, velocity);
            double gravityAlongTravel = Math.signum(velocity) * gravityVoltage(position);
            if (gravityAlongTravel <= 0) {
                return base; // gravity aids travel: braking presses the load onto the face harder
            }
            // Gravity opposes travel (the driving face carries the load): braking harder than
            // gravity brakes the load alone tosses the load off the face and across the lash.
            return Math.min(base, engagementCeiling(position));
        }
    }

    /**
     * Adds a <b>handoff zone</b> on top of the engagement ceiling: while gravity aids the travel
     * and the gravity torque is still small (near the crest, where the contact face must hand off),
     * cap the velocity low so the lash crossing completes before the mechanism picks up speed. At
     * speed the motor's back-EMF makes it physically unable to chase the free-falling load, so a
     * fast crossing always ends in a hard landing; a slow crossing lands gently, and once the load
     * is on the braking face the speed can open right back up.
     */
    private static class HandoffAwareArmModel extends EngagementAwareArmModel {
        private final double crossingVelocity; // velocity cap right at the crest, rad/s
        private final double rampSlope;        // extra rad/s allowed per volt of gravity torque
        private final double exitHoldVolts;    // keep the crest cap until |gravity| exceeds this
        private final double crestShiftRad;    // evaluate the accel ceiling this far toward the crest

        HandoffAwareArmModel(
                double loadInertia,
                double margin,
                double floor,
                double crossingVelocity,
                double rampSlope) {
            this(loadInertia, margin, floor, crossingVelocity, rampSlope, 0.0, 0.0);
        }

        HandoffAwareArmModel(
                double loadInertia,
                double margin,
                double floor,
                double crossingVelocity,
                double rampSlope,
                double exitHoldVolts,
                double crestShiftRad) {
            super(loadInertia, margin, floor);
            this.crossingVelocity = crossingVelocity;
            this.rampSlope = rampSlope;
            this.exitHoldVolts = exitHoldVolts;
            this.crestShiftRad = crestShiftRad;
        }

        /**
         * The load sits anywhere within the lash of the motor, so plan engagement against the
         * weakest gravity in that interval (zero if the crest lies inside it).
         */
        @Override
        double engagementGravity(double position) {
            double lo = gravityVoltage(position - crestShiftRad);
            double hi = gravityVoltage(position + crestShiftRad);
            if (Math.signum(lo) != Math.signum(hi)) {
                return 0.0;
            }
            return Math.abs(lo) < Math.abs(hi) ? lo : hi;
        }

        @Override
        public double maxSustainableVelocity(
                double availableVoltage, double position, double travelDirection) {
            double base = super.maxSustainableVelocity(availableVoltage, position, travelDirection);
            double gravityHere = gravityVoltage(position);
            double gravityAlongTravel = Math.signum(travelDirection) * gravityHere;
            // Approaching a crest (gravity still opposes, but flips sign before the target —
            // position + travelDirection IS the target): ramp the cap down with the remaining
            // gravity torque so the profile brakes gradually and arrives at the crest at
            // crossingVelocity. The ramp slope must be shallow enough that v*dv/dx stays within
            // the deceleration limit.
            double gravityAtTarget = gravityVoltage(position + travelDirection);
            if (gravityAlongTravel >= 0
                    && Math.signum(gravityHere) != Math.signum(gravityAtTarget)) {
                return Math.min(base, crossingVelocity + rampSlope * Math.abs(gravityHere));
            }
            // Just past a crest (gravity aids travel but is still weak): hold the crossing speed
            // until the landing window has passed — the airborne load needs to fall the full lash
            // before it is back on a face, and the landing is only gentle while gravity is small.
            if (gravityAlongTravel < 0 && Math.abs(gravityHere) < exitHoldVolts) {
                return Math.min(base, crossingVelocity);
            }
            return base;
        }
    }

    private static class Metrics {
        // Descent phase: from the final target becoming active until the load first enters the
        // target band. Hold phase: everything after that.
        int descentSeps, holdSeps;
        double descentFreeMs, holdFreeMs;
        double maxDescentImpact, maxHoldImpact; // |w_m - w_L| at re-engagement, rad/s
        double bottomOvershootDeg;              // deg past the load's own resting point (target - h)
        double steadyP2PDeg;                    // load peak-to-peak in the last second
        double finalDeg;
        double descentMs;                       // time to first reach the target band

        @Override
        public String toString() {
            return String.format(
                    "descent[%4.0fms sep=%d free=%3.0fms impact=%.2f] "
                            + "hold[sep=%2d free=%3.0fms impact=%.2f] "
                            + "overshoot=%.2fdeg p2p=%.2fdeg final=%.2fdeg",
                    descentMs, descentSeps, descentFreeMs, maxDescentImpact,
                    holdSeps, holdFreeMs, maxHoldImpact,
                    bottomOvershootDeg, steadyP2PDeg, finalDeg);
        }
    }

    /**
     * Over-the-top move: start at +120 deg, drive to -60 deg. The arm crests the vertical, where
     * the gravity torque (and with it the engagement budget) passes through zero and the contact
     * face must hand off.
     */
    private Metrics runOverTheTop(ArmModel model, double maxVel, double maxAccel, boolean trace) {
        return run(model, maxVel, maxAccel, Math.toRadians(120), null, Math.toRadians(-60), 0,
                5000, trace, BACKLASH_RAD);
    }

    /** The existing reversal scenario: settle at +30, then reverse down to -60 at t=1.8s. */
    private Metrics runReversal(ArmModel model, double maxVel, double maxAccel, boolean trace) {
        return run(model, maxVel, maxAccel, -Math.PI / 4, Math.PI / 6, -Math.PI / 3, 1800, 4000,
                trace, BACKLASH_RAD);
    }

    /** The reversal with zero backlash: isolates profile/tracking overshoot from lash effects. */
    private Metrics runReversalNoLash(ArmModel model, double maxVel, double maxAccel) {
        return run(model, maxVel, maxAccel, -Math.PI / 4, Math.PI / 6, -Math.PI / 3, 1800, 4000,
                false, 0.0);
    }

    private Metrics run(
            ArmModel model,
            double maxVel,
            double maxAccel,
            double startRad,
            Double phase1TargetRad,
            double finalTargetRad,
            int reversalMs,
            int totalMs,
            boolean trace,
            double backlashRad) {
        return run(model, maxVel, maxAccel, startRad, phase1TargetRad, finalTargetRad, reversalMs,
                totalMs, trace, backlashRad, 40, 8, 1.5);
    }

    private Metrics run(
            ArmModel model,
            double maxVel,
            double maxAccel,
            double startRad,
            Double phase1TargetRad,
            double finalTargetRad,
            int reversalMs,
            int totalMs,
            boolean trace,
            double backlashRad,
            double kP,
            double kI,
            double kD) {
        BacklashArmMotorSim plant =
                new BacklashArmMotorSim(
                        K_S, K_G, K_V, K_A, TICKS_PER_REV, GEAR_RATIO, 0.0,
                        MIN_ANGLE_RAD, MAX_ANGLE_RAD, startRad, backlashRad);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, startRad);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model, kP, kI, kD, maxVel, maxAccel, 480.0, 1.5, startRad);

        Random rng = new Random(7L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        Metrics m = new Metrics();
        m.descentMs = Double.NaN;
        double targetDeg = Math.toDegrees(finalTargetRad);
        // The load's own resting point sits a half-backlash (plus spring sag) below the motor-side
        // target; measure overshoot past that, not past the encoder target.
        double halfBacklashDeg = Math.toDegrees(backlashRad / 2.0);
        double loadRestDeg = targetDeg - halfBacklashDeg;
        int prevFace = faceOf(plant, backlashRad / 2.0);
        double prevLoadVel = 0.0;
        double steadyMin = Double.POSITIVE_INFINITY, steadyMax = Double.NEGATIVE_INFINITY;
        boolean arrived = false;

        for (int ms = 0; ms <= totalMs; ms++) {
            double target =
                    (phase1TargetRad != null && ms < reversalMs) ? phase1TargetRad : finalTargetRad;
            plant.step(0.001, power, VOLTAGE);

            // Contact bookkeeping once the final target is active.
            if (ms >= reversalMs) {
                double loadDeg = Math.toDegrees(plant.getTruePositionRad());
                if (!arrived && Math.abs(loadDeg - loadRestDeg) < 1.0) {
                    arrived = true;
                    m.descentMs = ms - reversalMs;
                }
                int face = faceOf(plant, backlashRad / 2.0);
                double relVel =
                        plant.getMotorVelocityRadPerSec() - plant.getTrueVelocityRadPerSec();
                if (face == 0) {
                    if (arrived) m.holdFreeMs += 1; else m.descentFreeMs += 1;
                    if (prevFace != 0) {
                        if (arrived) m.holdSeps++; else m.descentSeps++;
                        if (trace) {
                            System.out.printf(
                                    "  t=%4dms SEP  from face %+d at load=%6.1f deg relVel=%+.2f%n",
                                    ms, prevFace, loadDeg, relVel);
                        }
                    }
                } else if (prevFace == 0 && (m.descentSeps + m.holdSeps) > 0) {
                    if (arrived) {
                        m.maxHoldImpact = Math.max(m.maxHoldImpact, Math.abs(relVel));
                    } else {
                        m.maxDescentImpact = Math.max(m.maxDescentImpact, Math.abs(relVel));
                    }
                    if (trace) {
                        System.out.printf(
                                "  t=%4dms HIT  face %+d at load=%6.1f deg relVel=%+.2f%n",
                                ms, face, loadDeg, relVel);
                    }
                }
                prevFace = face;
                m.bottomOvershootDeg = Math.max(m.bottomOvershootDeg, loadRestDeg - loadDeg);
            }

            if (ms >= totalMs - 1000) {
                double loadDeg = Math.toDegrees(plant.getTruePositionRad());
                steadyMin = Math.min(steadyMin, loadDeg);
                steadyMax = Math.max(steadyMax, loadDeg);
                double loadVel = plant.getTrueVelocityRadPerSec();
                prevLoadVel = loadVel;
            }

            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(
                        plant.getPositionTicks() / TICKS_PER_RAD,
                        plant.getVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        m.steadyP2PDeg = steadyMax - steadyMin;
        m.finalDeg = Math.toDegrees(plant.getTruePositionRad());
        return m;
    }

    /** +1 = forward face (motor above load: braking/holding), -1 = reverse face, 0 = teeth apart. */
    private static int faceOf(BacklashArmMotorSim plant, double halfBacklashRad) {
        double delta = plant.getMotorPositionRad() - plant.getTruePositionRad();
        if (delta > halfBacklashRad) return 1;
        if (delta < -halfBacklashRad) return -1;
        return 0;
    }

    @Test
    public void compareStrategies() {
        ArmModel rigid = new ArmModel(K_S, K_V, K_A, K_G, 0.0);

        System.out.println("== over the top (+120 -> -60), v8 a12 ==");
        Metrics baseline = runOverTheTop(rigid, 8.0, 12.0, false);
        System.out.println("baseline       : " + baseline);
        for (double margin : new double[] {0.5, 0.9}) {
            ArmModel engaged = new EngagementAwareArmModel(LOAD_INERTIA, margin, 0.5);
            System.out.printf(
                    "margin %.2f    : %s%n", margin, runOverTheTop(engaged, 8.0, 12.0, false));
        }
        // The full soft-handoff treatment: two-sided engagement ceiling, weakest-gravity
        // evaluation across the lash, approach ramp, and a slow-crossing hold window past the
        // crest. See softHandoffSweep for the knob trade-offs.
        Metrics best =
                runOverTheTop(
                        new HandoffAwareArmModel(
                                LOAD_INERTIA, 0.67, 0.3, 0.5, 2.0,
                                K_G * Math.sin(Math.toRadians(10)), HALF_BACKLASH_RAD),
                        8.0, 12.0, false);
        System.out.println("soft handoff   : " + best);
        System.out.println("soft, slowest  : " + runOverTheTop(
                new HandoffAwareArmModel(
                        LOAD_INERTIA, 0.67, 0.3, 0.2, 2.0,
                        K_G * Math.sin(Math.toRadians(10)), HALF_BACKLASH_RAD),
                8.0, 12.0, false));

        System.out.println("== reversal (+30 -> -60), a12 ==");
        Metrics reversal = runReversal(rigid, 8.0, 12.0, false);
        Metrics reversalNoLash = runReversalNoLash(rigid, 8.0, 12.0);
        System.out.println("baseline v8 : " + reversal);
        System.out.println("baseline v4 : " + runReversal(rigid, 4.0, 12.0, false));
        System.out.println("no-lash v8  : " + reversalNoLash);
        System.out.println("no-lash v4  : " + runReversalNoLash(rigid, 4.0, 12.0));

        // Finding 1: a from-hold descent stays engaged the whole way down, and its bottom
        // overshoot is not a lash effect (the zero-backlash plant overshoots the same).
        assertEquals("from-hold descent should never open the lash", 0, reversal.descentSeps);
        assertTrue(
                "reversal overshoot should be a tuning artifact, not lash (lash="
                        + reversal.bottomOvershootDeg + ", no-lash="
                        + reversalNoLash.bottomOvershootDeg + ")",
                Math.abs(reversal.bottomOvershootDeg - reversalNoLash.bottomOvershootDeg) < 1.0);

        // Finding 2: the soft handoff cuts the landing speed to under half of baseline
        // (impact energy scales with v², so under a quarter of the impact energy).
        assertTrue(
                "soft handoff should cut the landing to under half of baseline (baseline="
                        + baseline.maxDescentImpact + ", handoff=" + best.maxDescentImpact + ")",
                best.maxDescentImpact < 0.5 * baseline.maxDescentImpact);

        // Finding 3: the hold is quiet. A setpoint that limit-cycles around the target broadcasts
        // its acceleration dither through the feedforward (+/-1.5 V at +/-5 rad/s^2) and separates
        // the teeth ~10 times a second; a profile that lands at rest leaves at most one tooth
        // unload, during the arrival-overshoot recovery.
        assertTrue(
                "hold should be quiet (got " + baseline.holdSeps + " separations)",
                baseline.holdSeps <= 2);
        assertTrue(
                "hold should not wiggle: p2p=" + baseline.steadyP2PDeg + " deg",
                baseline.steadyP2PDeg < 0.2);
    }

    /**
     * Sweep the soft-handoff knobs (exit hold window, crossing speed, engagement margin, half-gap
     * crest shift) for the gentlest over-the-top landing.
     */
    @Test
    public void softHandoffSweep() {
        System.out.println("baseline ref   : "
                + runOverTheTop(new ArmModel(K_S, K_V, K_A, K_G, 0.0), 8.0, 12.0, false));
        double[] holds = {Math.toRadians(8), Math.toRadians(10)};
        for (double holdRad : holds) {
            double exitHold = K_G * Math.sin(holdRad);
            for (double crossVel : new double[] {0.2, 0.35, 0.5}) {
                for (double margin : new double[] {0.67, 0.9}) {
                    ArmModel m = new HandoffAwareArmModel(
                            LOAD_INERTIA, margin, 0.3, crossVel, 2.0, exitHold, HALF_BACKLASH_RAD);
                    System.out.printf(
                            "hold%2.0f cv%.1f m%.2f: %s%n",
                            Math.toDegrees(holdRad), crossVel, margin,
                            runOverTheTop(m, 8.0, 12.0, false));
                }
            }
        }
        // Ablations at the best-guess point: no crest shift, and no exit hold.
        System.out.println("no-shift       : " + runOverTheTop(
                new HandoffAwareArmModel(
                        LOAD_INERTIA, 0.67, 0.3, 0.5, 2.0, K_G * Math.sin(Math.toRadians(15)), 0.0),
                8.0, 12.0, false));
        System.out.println("no-hold        : " + runOverTheTop(
                new HandoffAwareArmModel(LOAD_INERTIA, 0.67, 0.3, 0.5, 2.0, 0.0, HALF_BACKLASH_RAD),
                8.0, 12.0, false));
    }

    /**
     * Which loop element drives the hold chatter? Sweep the feedback gains one at a time on the
     * reversal scenario and report only the hold-phase metrics.
     */
    @Test
    public void holdChatterGainSweep() {
        ArmModel rigid = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        double[][] gains = {
            {40, 8, 1.5},  // baseline
            {40, 8, 0.0},  // no D
            {40, 8, 4.0},  // heavy D
            {40, 0, 1.5},  // no I
            {10, 8, 1.5},  // soft P
            {10, 2, 0.5},  // everything soft
        };
        for (double[] g : gains) {
            Metrics m =
                    run(rigid, 8.0, 12.0, -Math.PI / 4, Math.PI / 6, -Math.PI / 3, 1800, 4000,
                            false, BACKLASH_RAD, g[0], g[1], g[2]);
            System.out.printf(
                    "kP=%4.0f kI=%3.0f kD=%3.1f : hold[sep=%2d free=%3.0fms impact=%.2f] "
                            + "p2p=%.2fdeg final=%.2fdeg%n",
                    g[0], g[1], g[2], m.holdSeps, m.holdFreeMs, m.maxHoldImpact,
                    m.steadyP2PDeg, m.finalDeg);
        }
    }

    /**
     * High-resolution look at the hold: sample the commanded voltage, the filter's estimate, and
     * both true angles every 10 ms through a chatter window.
     */
    @Test
    public void holdChatterTrace() {
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        double start = -Math.PI / 4, up = Math.PI / 6, down = -Math.PI / 3;
        BacklashArmMotorSim plant =
                new BacklashArmMotorSim(
                        K_S, K_G, K_V, K_A, TICKS_PER_REV, GEAR_RATIO, 0.0,
                        MIN_ANGLE_RAD, MAX_ANGLE_RAD, start, BACKLASH_RAD);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, start);

        Random rng = new Random(7L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;
        System.out.println(
                "t_ms, u_V, estPos_deg, estVel, motor_deg, load_deg, face, spPos_deg, spVel, spAcc");
        for (int ms = 0; ms <= 3400; ms++) {
            double target = ms < 1800 ? up : down;
            plant.step(0.001, power, VOLTAGE);
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(
                        plant.getPositionTicks() / TICKS_PER_RAD,
                        plant.getVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                if (ms >= 2600) {
                    System.out.printf(
                            "%5d, %+6.3f, %8.3f, %+7.3f, %8.3f, %8.3f, %+d, %8.3f, %+7.3f, %+7.2f%n",
                            ms, voltage, Math.toDegrees(ekf.getPosition()), ekf.getVelocity(),
                            Math.toDegrees(plant.getMotorPositionRad()),
                            Math.toDegrees(plant.getTruePositionRad()),
                            faceOf(plant, HALF_BACKLASH_RAD),
                            Math.toDegrees(controller.getSetpointPosition()),
                            controller.getSetpointVelocity(),
                            controller.getSetpointAcceleration());
                }
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
    }

    /**
     * Who overshoots at arrival — the profile, the estimate, or the plant? Sample the braking
     * window of the v8 reversal every loop.
     */
    @Test
    public void arrivalTrace() {
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        double start = -Math.PI / 4, up = Math.PI / 6, down = -Math.PI / 3;
        BacklashArmMotorSim plant =
                new BacklashArmMotorSim(
                        K_S, K_G, K_V, K_A, TICKS_PER_REV, GEAR_RATIO, 0.0,
                        MIN_ANGLE_RAD, MAX_ANGLE_RAD, start, BACKLASH_RAD);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, start);

        Random rng = new Random(7L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;
        System.out.println(
                "t_ms, spPos_deg, spVel, spAcc, estPos_deg, estVel, motor_deg, motorVel, load_deg, u_V");
        for (int ms = 0; ms <= 2900; ms++) {
            double target = ms < 1800 ? up : down;
            plant.step(0.001, power, VOLTAGE);
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(
                        plant.getPositionTicks() / TICKS_PER_RAD,
                        plant.getVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                if (ms >= 1900) {
                    System.out.printf(
                            "%5d, %8.2f, %+6.2f, %+7.2f, %8.2f, %+6.2f, %8.2f, %+6.2f, %8.2f, %+6.2f%n",
                            ms,
                            Math.toDegrees(controller.getSetpointPosition()),
                            controller.getSetpointVelocity(),
                            controller.getSetpointAcceleration(),
                            Math.toDegrees(ekf.getPosition()), ekf.getVelocity(),
                            Math.toDegrees(plant.getMotorPositionRad()),
                            plant.getMotorVelocityRadPerSec(),
                            Math.toDegrees(plant.getTruePositionRad()),
                            voltage);
                }
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
    }

    @Test
    public void traceEvents() {
        System.out.println("== trace: over the top, soft handoff hold10 cv0.5 m0.67 ==");
        runOverTheTop(
                new HandoffAwareArmModel(
                        LOAD_INERTIA, 0.67, 0.3, 0.5, 2.0,
                        K_G * Math.sin(Math.toRadians(10)), HALF_BACKLASH_RAD),
                8.0, 12.0, true);
    }
}
