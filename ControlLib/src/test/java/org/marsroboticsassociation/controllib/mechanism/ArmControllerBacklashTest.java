package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.marsroboticsassociation.controllib.sim.BacklashArmMotorSim;
import org.marsroboticsassociation.controllib.sim.FlexArmMotorSim;

import java.util.Random;

/**
 * Closed-loop test of the mechanism {@link MotorMechanismController} (built on an {@link ArmModel},
 * with a {@link MotorMechanismEkf} estimating state) driving the team's two-mass {@link
 * BacklashArmMotorSim} from MarsCommonFtc instead of the rigid {@link ArmPlantSim}.
 *
 * <p>The controller and filter model a single rigid arm; the plant does not. {@code
 * BacklashArmMotorSim} splits the motor and the load across a gearbox dead band, so the encoder
 * (motor side) is blind to where the arm (load side) actually is by up to half the backlash. This
 * test scores against the <em>load</em> ground truth, {@link BacklashArmMotorSim#getTruePositionRad}
 * — the angle the arm is really at — to see how the rigid-model controller copes with lash.
 *
 * <p>Two scenarios:
 * <ul>
 *   <li>A monotonic sweep up through horizontal (where gravity torque peaks), then a hold. The
 *       steady-state load error is expected to sit within roughly the half-backlash dead band, since
 *       the controller can only servo the motor side it can see.
 *   <li>A direction reversal, the case backlash punishes hardest: on reversal the motor must cross
 *       the full dead band before the load responds (lost motion), and while the teeth are separated
 *       the arm free-falls under gravity. The controller must still recover the load to the new
 *       target.
 * </ul>
 */
public class ArmControllerBacklashTest {

    // Same gain family as the rigid-plant ArmControllerTest, so this is an apples-to-apples
    // comparison with backlash as the only added effect.
    private static final int TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 100.0;
    private static final double TICKS_PER_RAD = TICKS_PER_REV * GEAR_RATIO / (2 * Math.PI);
    private static final double K_S = 0.1, K_G = 1.5, K_V = 2.0, K_A = 0.2;
    private static final double VOLTAGE = 12.0;

    // Encoder reads 0 at horizontal here (zero offset 0), so angle = ticks / TICKS_PER_RAD.
    private static final double ENCODER_ZERO_OFFSET_RAD = 0.0;
    private static final double MIN_ANGLE_RAD = -Math.PI * 0.9;
    private static final double MAX_ANGLE_RAD = Math.PI * 0.9;
    private static final double BACKLASH_RAD = Math.toRadians(5.0); // half-backlash = 2.5 deg
    private static final double HALF_BACKLASH_DEG = Math.toDegrees(BACKLASH_RAD) / 2.0;
    // Gravity hold-voltage below which the rest compensation tapers off (~ kG at 11 deg from
    // vertical): too little gravity to pin the load onto one tooth face.
    private static final double TAPER_VOLTS = 0.3;

    @Test
    public void reachesAndHoldsTargetThroughBacklash() {
        double start = -Math.PI / 4; // -45 deg
        double target = Math.PI / 4; //  +45 deg

        BacklashArmMotorSim plant = makePlant(start);
        ArmModel model = new ArmModel(K_S, K_V, K_A, /* kCos= */ K_G, /* kSin= */ 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model,
                        /* kP= */ 40,
                        /* kI= */ 8,
                        /* kD= */ 1.5,
                        /* maxVelocity= */ 8.0,
                        /* maxAcceleration= */ 12.0,
                        /* maxJerk= */ 480.0,
                        /* feedbackVoltageMargin= */ 1.5,
                        start);

        Random rng = new Random(1L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double maxOvershootDeg = 0.0;
        double sumSteadyAbsErrDeg = 0.0;
        int steadySamples = 0;

        for (int ms = 0; ms <= 3000; ms++) {
            plant.step(0.001, power, VOLTAGE);
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD);

                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;

                double trueDeg = Math.toDegrees(plant.getTruePositionRad());
                maxOvershootDeg = Math.max(maxOvershootDeg, trueDeg - 45.0);
                if (ms / 1000.0 > 2.0) {
                    sumSteadyAbsErrDeg += Math.abs(trueDeg - 45.0);
                    steadySamples++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double finalDeg = Math.toDegrees(plant.getTruePositionRad());
        double steadyAbsErrDeg = sumSteadyAbsErrDeg / steadySamples;
        System.out.printf(
                "backlash sweep: final=%.2f deg, steady|err|=%.2f deg, overshoot=%.2f deg "
                        + "(half-backlash=%.2f deg)%n",
                finalDeg, steadyAbsErrDeg, maxOvershootDeg, HALF_BACKLASH_DEG);

        // The controller servos the motor side it can see; the load can rest up to a half-backlash
        // off, plus a little gravity sag. Convergence within the full backlash band is the bar.
        assertTrue(
                "load did not reach target through backlash, final " + finalDeg + " deg",
                Math.abs(finalDeg - 45.0) < Math.toDegrees(BACKLASH_RAD) + 2.0);
        assertTrue(
                "steady-state load error too high for backlash band: " + steadyAbsErrDeg + " deg",
                steadyAbsErrDeg < HALF_BACKLASH_DEG + 2.0);
        assertTrue("overshoot too high: " + maxOvershootDeg + " deg", maxOvershootDeg < 10.0);
    }

    @Test
    public void recoversTheLoadAfterADirectionReversal() {
        double start = -Math.PI / 4; // -45 deg
        double up = Math.PI / 6; //  +30 deg
        double down = -Math.PI / 3; //  -60 deg, a reversal back through the lash

        BacklashArmMotorSim plant = makePlant(start);
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, start);

        Random rng = new Random(7L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        // Phase 1: go up to +30 and settle. Phase 2 (after 1.8 s): reverse down to -60.
        double reversalSettleErrDeg = 0.0;
        int reversalSamples = 0;

        for (int ms = 0; ms <= 4000; ms++) {
            double target = ms < 1800 ? up : down;
            plant.step(0.001, power, VOLTAGE);
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD);

                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;

                if (ms / 1000.0 > 3.0) {
                    reversalSettleErrDeg +=
                            Math.abs(Math.toDegrees(plant.getTruePositionRad()) - (-60.0));
                    reversalSamples++;
                }

                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }

        double finalDeg = Math.toDegrees(plant.getTruePositionRad());
        double settleErrDeg = reversalSettleErrDeg / reversalSamples;
        System.out.printf(
                "backlash reversal: final=%.2f deg (target -60), settle|err|=%.2f deg%n",
                finalDeg, settleErrDeg);

        assertTrue(
                "load did not recover to the reversed target, final " + finalDeg + " deg",
                Math.abs(finalDeg - (-60.0)) < Math.toDegrees(BACKLASH_RAD) + 2.0);
        assertTrue(
                "load did not settle after reversal: " + settleErrDeg + " deg",
                settleErrDeg < HALF_BACKLASH_DEG + 2.0);
    }

    /**
     * Runs the identical controller, filter, gains, and move against the rigid {@link ArmPlantSim}
     * and against the backlash plant, and contrasts the steady-state load error. The rigid plant
     * should converge tight; the backlash plant should stay bounded by the dead band but visibly
     * worse — the cost of a rigid-model controller meeting a lashy gearbox, made explicit.
     */
    @Test
    public void rigidConvergesTightWhileBacklashStaysWithinTheBand() {
        double start = -Math.PI / 4; // -45 deg
        double target = Math.PI / 4; //  +45 deg

        double rigidErrDeg = runRigid(start, target);
        double backlashErrDeg = runBacklash(start, target);
        System.out.printf(
                "rigid vs backlash: rigid steady|err|=%.2f deg, backlash steady|err|=%.2f deg "
                        + "(half-backlash=%.2f deg)%n",
                rigidErrDeg, backlashErrDeg, HALF_BACKLASH_DEG);

        // Rigid plant: the controller's model is exact, so it converges tight.
        assertTrue("rigid plant steady error too high: " + rigidErrDeg + " deg", rigidErrDeg < 1.5);
        // Backlash plant: bounded by the dead band the controller cannot see across...
        assertTrue(
                "backlash plant steady error outside the band: " + backlashErrDeg + " deg",
                backlashErrDeg < HALF_BACKLASH_DEG + 2.0);
        // ...and measurably worse than rigid — that is the point of running both.
        assertTrue(
                "backlash should degrade tracking vs rigid (rigid=" + rigidErrDeg + ", backlash="
                        + backlashErrDeg + ")",
                backlashErrDeg > rigidErrDeg);
    }

    /**
     * Rest-only backlash compensation: bias the target a half-backlash <em>against</em> gravity,
     * so the load — which settles a half-backlash off the motor on the gravity-loaded tooth face —
     * comes to rest on the stated target instead of the motor. Only the endpoint moves, so this
     * closes the steady-state half-lash sag the other tests tolerate, without touching the motion.
     *
     * <p>Run on both sides of vertical, since the resting face (and so the bias sign) flips with
     * the sign of the gravity torque: below vertical the load hangs under the motor; past vertical
     * it rests above it.
     */
    @Test
    public void restCompensationClosesTheGapToRigid() {
        // Below vertical (gravity voltage positive: bias up).
        double start = -Math.PI / 4; // -45 deg
        double target = Math.PI / 4; // +45 deg
        double rigidErrDeg = runRigid(start, target);
        double rawErrDeg = runBacklash(start, target, false);
        double compErrDeg = runBacklash(start, target, true);

        // Past vertical (gravity voltage negative: bias down).
        double startPast = Math.toRadians(150);
        double targetPast = Math.toRadians(120);
        double rawPastErrDeg = runBacklash(startPast, targetPast, false);
        double compPastErrDeg = runBacklash(startPast, targetPast, true);

        System.out.printf(
                "rest compensation: rigid=%.2f, raw=%.2f -> comp=%.2f deg; "
                        + "past-vertical raw=%.2f -> comp=%.2f deg (half-backlash=%.2f deg)%n",
                rigidErrDeg, rawErrDeg, compErrDeg, rawPastErrDeg, compPastErrDeg,
                HALF_BACKLASH_DEG);

        // Compensation removes the half-lash sag: steady error drops from ~half-backlash to near
        // the rigid plant's level (residual: contact-spring sag + encoder quantization + tracking).
        assertTrue(
                "compensated steady error should be near rigid, got " + compErrDeg
                        + " deg (rigid=" + rigidErrDeg + ")",
                compErrDeg < 1.0);
        assertTrue(
                "compensation should remove most of the half-lash sag (raw=" + rawErrDeg
                        + ", comp=" + compErrDeg + ")",
                compErrDeg < rawErrDeg - 1.5);
        assertTrue(
                "past-vertical bias sign must flip with the resting face (raw=" + rawPastErrDeg
                        + ", comp=" + compPastErrDeg + ")",
                compPastErrDeg < 1.0 && compPastErrDeg < rawPastErrDeg - 1.5);
    }

    /**
     * On the three-inertia flex plant, the tip rests off the motor by more than the half-lash:
     * gear-tooth penetration and flex-spring sag add an elastic droop <em>proportional to
     * gravity</em>. Half-lash compensation alone leaves that behind (observed live in ControlLab:
     * stated target parked halfway between motor and arm). The full rest model is {@code
     * halfLash·sign(g) + compliance·g}; with the plant's compliance supplied, the tip lands on the
     * stated target. Note the fix is <em>not</em> full-lash compensation — the droop scales with
     * gravity while the lash term does not, so a constant full-lash bias is wrong at most angles.
     */
    @Test
    public void restCompensationCoversElasticSagOnTheFlexPlant() {
        double start = -Math.PI / 4; // -45 deg
        double target = Math.PI / 4; // +45 deg

        double rawErrDeg = runFlex(start, target, false);
        double halfLashOnlyErrDeg = runFlex(start, target, true, 0.0);
        double fullErrDeg = runFlex(start, target, true, Double.NaN); // NaN = use plant's compliance

        // Expected droop at +45 deg: half-lash 2.5 + (1/500 + 0.85/kFlex)·kG·cos(45) ≈ 1.7 more.
        System.out.printf(
                "flex rest compensation: raw=%.2f, half-lash-only=%.2f, +compliance=%.2f deg "
                        + "(half-backlash=%.2f deg)%n",
                rawErrDeg, halfLashOnlyErrDeg, fullErrDeg, HALF_BACKLASH_DEG);

        assertTrue(
                "raw flex droop should exceed the half-lash alone, got " + rawErrDeg + " deg",
                rawErrDeg > HALF_BACKLASH_DEG + 1.0);
        assertTrue(
                "half-lash-only should leave the elastic sag behind (raw=" + rawErrDeg
                        + ", half-lash-only=" + halfLashOnlyErrDeg + ")",
                halfLashOnlyErrDeg > 1.0 && halfLashOnlyErrDeg < rawErrDeg);
        assertTrue(
                "lash + compliance should land the tip on the stated target, got "
                        + fullErrDeg + " deg",
                fullErrDeg < 1.0);
    }

    private double runFlex(double start, double target, boolean compensate) {
        return runFlex(start, target, compensate, Double.NaN);
    }

    /**
     * Run the move against the flex plant; return mean steady-state tip error in degrees.
     *
     * @param complianceRadPerVolt rest compliance handed to the controller when compensating;
     *     NaN means use the plant's own {@link FlexArmMotorSim#getRestComplianceRadPerVolt}
     */
    private double runFlex(
            double start, double target, boolean compensate, double complianceRadPerVolt) {
        FlexArmMotorSim plant = new FlexArmMotorSim(
                K_S, K_G, K_V, K_A,
                TICKS_PER_REV, GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD, MAX_ANGLE_RAD,
                start, BACKLASH_RAD,
                /* flexHz= */ 3.0, /* flexZeta= */ 0.03);
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, start);
        if (compensate) {
            double compliance = Double.isNaN(complianceRadPerVolt)
                    ? plant.getRestComplianceRadPerVolt()
                    : complianceRadPerVolt;
            controller.setBacklashCompensation(BACKLASH_RAD, TAPER_VOLTS, compliance);
        }

        Random rng = new Random(1L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;
        double sumErrDeg = 0.0;
        int samples = 0;

        for (int ms = 0; ms <= 4000; ms++) {
            plant.step(0.001, power, VOLTAGE);
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(
                        plant.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD,
                        plant.getVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                if (ms / 1000.0 > 3.0) {
                    sumErrDeg +=
                            Math.abs(Math.toDegrees(plant.getTruePositionRad() - target));
                    samples++;
                }
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
        return sumErrDeg / samples;
    }

    /** Run the move against the rigid plant; return mean steady-state load error in degrees. */
    private double runRigid(double start, double target) {
        ArmPlantSim plant =
                new ArmPlantSim(
                        K_S, K_G, K_V, K_A, TICKS_PER_RAD, MIN_ANGLE_RAD, MAX_ANGLE_RAD, start);
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, start);

        Random rng = new Random(1L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;
        double sumErrDeg = 0.0;
        int samples = 0;

        for (int ms = 0; ms <= 3000; ms++) {
            plant.integrate(0.001, power, VOLTAGE);
            if (ms % 10 == 0) {
                plant.latchEncoder();
            }
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(
                        plant.getEncoderPosition() / TICKS_PER_RAD,
                        plant.getEncoderVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                if (ms / 1000.0 > 2.0) {
                    sumErrDeg +=
                            Math.abs(Math.toDegrees(plant.getTrueAngleRad() - target));
                    samples++;
                }
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
        return sumErrDeg / samples;
    }

    /** Run the same move against the backlash plant; return mean steady-state load error. */
    private double runBacklash(double start, double target) {
        return runBacklash(start, target, false);
    }

    /**
     * Run the move against the backlash plant, optionally with rest-only backlash compensation
     * enabled on the controller; return mean steady-state load error in degrees.
     */
    private double runBacklash(double start, double target, boolean compensate) {
        BacklashArmMotorSim plant = makePlant(start);
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(model, 40, 8, 1.5, 8.0, 12.0, 480.0, 1.5, start);
        if (compensate) {
            controller.setBacklashCompensation(BACKLASH_RAD, TAPER_VOLTS);
        }

        Random rng = new Random(1L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;
        double sumErrDeg = 0.0;
        int samples = 0;

        for (int ms = 0; ms <= 3000; ms++) {
            plant.step(0.001, power, VOLTAGE);
            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                if (ms / 1000.0 > 2.0) {
                    sumErrDeg +=
                            Math.abs(Math.toDegrees(plant.getTruePositionRad() - target));
                    samples++;
                }
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
        return sumErrDeg / samples;
    }

    /**
     * On a downward (gravity-aided) reversal the load presses on the braking face the whole way
     * down (see ArmBacklashKeepEngagedTest), so the arrival must be clean at any descent speed:
     * the stop is planned under the jerk limit from the profile's own state, so braking starts
     * early enough regardless of how fast the descent runs. Fast (8 rad/s) and velocity-capped
     * (4 rad/s) descents arrive alike — within a couple of degrees past the load's half-backlash
     * resting offset — and both settle without chattering down. A stopping law that ignores the
     * jerk-limited swing from full accelerate to full brake fails this test at the fast speed
     * only, which would make "slow down" masquerade as a backlash mitigation.
     */
    @Test
    public void descentSpeedDoesNotCostBottomOvershoot() {
        // default accel=12, jerk=480; only the velocity cap changes.
        double[] fast = runReversalMetrics(/* maxVel= */ 8.0, 12.0, 480.0);
        double[] capped = runReversalMetrics(/* maxVel= */ 4.0, 12.0, 480.0);
        System.out.printf(
                "reversal bounce: fast(8) overshoot=%.1f deg p2p=%.2f; "
                        + "capped(4) overshoot=%.1f deg p2p=%.2f%n",
                fast[0], fast[2], capped[0], capped[2]);

        // Both speeds arrive within the half-backlash resting offset plus a small dynamic margin
        // (closed-loop lag past the profile; half-lash ≈ 2.5 deg, + ~3 deg dynamic).
        assertTrue(
                "fast reversal overshoot too large: " + fast[0] + " deg",
                fast[0] < HALF_BACKLASH_DEG + 3.0);
        assertTrue(
                "capped reversal overshoot too large: " + capped[0] + " deg",
                capped[0] < HALF_BACKLASH_DEG + 3.0);
        // And going fast costs no meaningful extra overshoot.
        assertTrue(
                "descent speed should not cost overshoot (fast=" + fast[0]
                        + ", capped=" + capped[0] + ")",
                fast[0] < capped[0] + 1.0);
        // Both settle rather than bouncing all the way down.
        assertTrue("fast reversal did not settle: p2p=" + fast[2] + " deg", fast[2] < 2.0);
        assertTrue("capped reversal did not settle: p2p=" + capped[2] + " deg", capped[2] < 2.0);
    }

    /**
     * Run the up-then-down reversal and return [bottomOvershootDeg, steadyVelZeroCrossings,
     * steadyP2PDeg, finalDeg]. Steady window is the last 1 s (3000-4000 ms).
     */
    private double[] runReversalMetrics(double maxVel, double maxAccel, double maxJerk) {
        return runReversalMetrics(maxVel, maxAccel, maxJerk, 1.5);
    }

    private double[] runReversalMetrics(double maxVel, double maxAccel, double maxJerk, double kD) {
        double start = -Math.PI / 4, up = Math.PI / 6, down = -Math.PI / 3;
        BacklashArmMotorSim plant = makePlant(start);
        ArmModel model = new ArmModel(K_S, K_V, K_A, K_G, 0.0);
        MotorMechanismEkf ekf = new MotorMechanismEkf(model, 0.025, 5.0, 0.003, 0.1, 0.0, start);
        MotorMechanismController controller =
                new MotorMechanismController(
                        model, 40, 8, kD, maxVel, maxAccel, maxJerk, 1.5, start);

        Random rng = new Random(7L);
        double power = 0.0;
        int lastLoopMs = 0, nextLoopMs = 20;

        double bottomOvershoot = 0.0;       // deg past -60 reached during arrival
        double prevLoadVel = 0.0;
        int steadyZeroCrossings = 0;
        double steadyMin = Double.POSITIVE_INFINITY, steadyMax = Double.NEGATIVE_INFINITY;

        for (int ms = 0; ms <= 4000; ms++) {
            double target = ms < 1800 ? up : down;
            plant.step(0.001, power, VOLTAGE);
            double loadDeg = Math.toDegrees(plant.getTruePositionRad());
            double loadVel = plant.getTrueVelocityRadPerSec();

            if (ms >= 1800) bottomOvershoot = Math.max(bottomOvershoot, -60.0 - loadDeg);
            if (ms >= 3000) {
                steadyMin = Math.min(steadyMin, loadDeg);
                steadyMax = Math.max(steadyMax, loadDeg);
                if (Math.signum(loadVel) != Math.signum(prevLoadVel)
                        && (Math.abs(loadVel) > 0.05 || Math.abs(prevLoadVel) > 0.05)) {
                    steadyZeroCrossings++;
                }
            }
            prevLoadVel = loadVel;

            if (ms >= nextLoopMs) {
                double dt = (ms - lastLoopMs) / 1000.0;
                ekf.predict(dt, power, VOLTAGE);
                ekf.correct(measuredAngleRad(plant), plant.getVelocityTps() / TICKS_PER_RAD);
                double voltage =
                        controller.calculate(
                                target, ekf.getPosition(), ekf.getVelocity(), VOLTAGE, dt);
                power = voltage / VOLTAGE;
                lastLoopMs = ms;
                nextLoopMs = ms + 20 + rng.nextInt(10);
            }
        }
        return new double[] {
            bottomOvershoot, steadyZeroCrossings, steadyMax - steadyMin,
            Math.toDegrees(plant.getTruePositionRad())
        };
    }

    private BacklashArmMotorSim makePlant(double initialAngleRad) {
        return new BacklashArmMotorSim(
                K_S, K_G, K_V, K_A,
                TICKS_PER_REV, GEAR_RATIO,
                ENCODER_ZERO_OFFSET_RAD,
                MIN_ANGLE_RAD, MAX_ANGLE_RAD,
                initialAngleRad, BACKLASH_RAD);
    }

    /** Motor-side encoder angle in radians — what the controller and filter actually see. */
    private static double measuredAngleRad(BacklashArmMotorSim plant) {
        return plant.getPositionTicks() / TICKS_PER_RAD + ENCODER_ZERO_OFFSET_RAD;
    }
}
