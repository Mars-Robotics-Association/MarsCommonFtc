package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertThrows;
import static org.junit.Assert.assertTrue;

import java.util.Random;
import org.junit.Test;

public class CascadedRateLimiterTest {

    private static final double V_MAX = 5.2;
    private static final double A_MAX = 12.0;
    private static final double J_MAX = 480.0;
    private static final double DT = 0.02; // realistic 50 Hz loop

    /**
     * Over a long move that reaches cruise, velocity, acceleration, and jerk must all stay within
     * their limits. Acceleration and jerk are bounded by construction; velocity is held to the cap
     * by the headroom-aware accel envelope (no residual a²/2j overshoot into cruise).
     */
    @Test
    public void respectsVelocityAccelerationAndJerkLimits() {
        CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
        double target = 30.0; // far enough to spend time at cruise

        double previousAcceleration = 0.0;
        double maxVelocity = 0.0;
        double maxAcceleration = 0.0;
        double maxJerk = 0.0;

        for (int i = 0; i < 200; i++) {
            limiter.update(target, DT);
            double jerk = Math.abs((limiter.getAcceleration() - previousAcceleration) / DT);
            previousAcceleration = limiter.getAcceleration();
            maxVelocity = Math.max(maxVelocity, Math.abs(limiter.getVelocity()));
            maxAcceleration = Math.max(maxAcceleration, Math.abs(limiter.getAcceleration()));
            maxJerk = Math.max(maxJerk, jerk);
        }

        assertTrue(
                "acceleration exceeded its limit: " + maxAcceleration,
                maxAcceleration <= A_MAX * 1.001);
        assertTrue("jerk exceeded its limit: " + maxJerk, maxJerk <= J_MAX * 1.001);
        assertTrue(
                "velocity exceeded its limit: " + maxVelocity, maxVelocity <= V_MAX * 1.000001);
        // It really did reach cruise, so the velocity bound was actually exercised.
        assertTrue("never reached cruise velocity: " + maxVelocity, maxVelocity > 0.9 * V_MAX);
    }

    /**
     * With a low jerk limit the residual-accel overshoot would be large (~aMax²/2jMax) without the
     * headroom envelope. Peak |velocity| must still stay at or under the cap.
     */
    @Test
    public void lowJerkDoesNotOvershootVelocityCap() {
        double vMax = 5.2;
        double aMax = 12.0;
        double jMax = 60.0; // would allow ~1.2 of continuous residual overshoot without the fix
        CascadedRateLimiter limiter = new CascadedRateLimiter(vMax, aMax, aMax, jMax, 0.0);
        double peak = 0.0;
        for (int i = 0; i < 500; i++) {
            limiter.update(30.0, DT);
            peak = Math.max(peak, Math.abs(limiter.getVelocity()));
        }
        assertTrue(
                "low-jerk ramp overshot velocity cap: peak=" + peak + " (cap=" + vMax + ")",
                peak <= vMax * 1.000001);
        assertTrue("never reached cruise under low jerk: " + peak, peak > 0.9 * vMax);
    }

    /**
     * Low-jerk limits used to sail the profile past the target because residual accel lagged the
     * falling stopping command. Position overshoot must stay tiny.
     */
    @Test
    public void lowJerkDoesNotSailPastTarget() {
        CascadedRateLimiter limiter =
                new CascadedRateLimiter(3.0, 6.0, 8.0, 30.0, Math.toRadians(-100));
        double target = Math.toRadians(30);
        double maxOvershoot = 0.0;
        double peakVel = 0.0;
        for (int i = 0; i < 1000; i++) {
            limiter.update(target, 0.016);
            maxOvershoot = Math.max(maxOvershoot, limiter.getPosition() - target);
            peakVel = Math.max(peakVel, Math.abs(limiter.getVelocity()));
        }
        assertTrue(
                "velocity cap violated under low jerk: " + peakVel, peakVel <= 3.0 * 1.000001);
        assertTrue(
                "profile sailed past target: overshootDeg=" + Math.toDegrees(maxOvershoot),
                maxOvershoot < Math.toRadians(0.5));
        assertTrue(
                "did not settle: " + limiter.getPosition(),
                Math.abs(limiter.getPosition() - target) < 1e-4);
    }

    /**
     * Mechanism path: {@code MotorMechanismController} lowers maxAcceleration every loop as
     * back-EMF eats headroom. A pure cascade keeps the old high residual accel and overshoots the
     * velocity command/cap while jerk bleeds a down. Peak velocity must stay at or under the cap
     * even when the accel ceiling collapses mid-ramp.
     */
    @Test
    public void collapsingAccelCapDoesNotOvershootVelocity() {
        double vMax = 5.0;
        CascadedRateLimiter limiter = new CascadedRateLimiter(vMax, 20.0, 20.0, 100.0, 0.0);
        double peak = 0.0;
        for (int i = 0; i < 200; i++) {
            // Simulate back-EMF: full accel at rest, nearly none near cruise.
            double speed = Math.abs(limiter.getVelocity());
            double aCap = Math.max(0.5, 20.0 * (1.0 - speed / vMax));
            limiter.setMaxAcceleration(aCap);
            limiter.setMaxDeceleration(20.0);
            limiter.update(50.0, 0.016);
            peak = Math.max(peak, Math.abs(limiter.getVelocity()));
            assertTrue(
                    "velocity overshot cap under collapsing accel (v="
                            + limiter.getVelocity()
                            + ", aCap="
                            + aCap
                            + ", a="
                            + limiter.getAcceleration()
                            + ")",
                    Math.abs(limiter.getVelocity()) <= vMax * 1.000001);
        }
        assertTrue("never approached cruise: " + peak, peak > 0.85 * vMax);
    }

    /**
     * Full mechanism controller (dynamic back-EMF ceilings) must keep the profile velocity at or
     * under the configured cap for a long uphill move.
     */
    @Test
    public void mechanismControllerProfileRespectsVelocityCap() {
        ArmModel model = new ArmModel(0.3, 1.2, 0.35, 3.5, 0.0);
        double vMax = 8.0;
        MotorMechanismController controller =
                new MotorMechanismController(
                        model, 40.0, 8.0, 1.5, vMax, 12.0, 480.0, 1.5, Math.toRadians(-100));
        double target = Math.toRadians(30);
        double peak = 0.0;
        for (int i = 0; i < 800; i++) {
            // Perfect tracking isolates the profile.
            controller.calculate(
                    target,
                    controller.getSetpointPosition(),
                    controller.getSetpointVelocity(),
                    12.0,
                    0.016);
            peak = Math.max(peak, Math.abs(controller.getSetpointVelocity()));
        }
        assertTrue(
                "mechanism profile velocity overshot configured cap: " + peak,
                peak <= vMax * 1.000001);
    }

    /** A point-to-point move settles on the target with only a small transient overshoot. */
    @Test
    public void convergesToTargetWithSmallOvershoot() {
        CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
        double target = 1.5708;

        double maxOvershoot = 0.0;
        for (int i = 0; i < 400; i++) { // 8 seconds
            limiter.update(target, DT);
            maxOvershoot = Math.max(maxOvershoot, limiter.getPosition() - target);
        }

        assertTrue(
                "did not settle on target: " + limiter.getPosition(),
                Math.abs(limiter.getPosition() - target) < 0.01);
        assertTrue("overshoot too large: " + maxOvershoot, maxOvershoot < 0.15 * target);
    }

    /** Braking uses the deceleration limit, which can be larger than the acceleration limit. */
    @Test
    public void brakingUsesADifferentLimitThanAcceleration() {
        double accel = 5.0;
        double decel = 20.0;
        double jerk = 10000.0;
        CascadedRateLimiter limiter =
                new CascadedRateLimiter(/* maxVelocity= */ 30.0, accel, decel, jerk, 0.0);
        double target = 40.0;

        double maxSpeedingUpAccel = 0.0;
        double maxBrakingAccel = 0.0;
        for (int i = 0; i < 400; i++) {
            limiter.update(target, DT);
            double v = limiter.getVelocity();
            double a = limiter.getAcceleration();
            if (Math.abs(v) > 0.05) {
                if (a * v >= 0) {
                    maxSpeedingUpAccel = Math.max(maxSpeedingUpAccel, Math.abs(a));
                } else {
                    maxBrakingAccel = Math.max(maxBrakingAccel, Math.abs(a));
                }
            }
            // Stop at first arrival, before any tiny settling oscillation muddies the
            // speeding-up-versus-braking classification near the velocity zero-crossing.
            if (Math.abs(target - limiter.getPosition()) < 0.5 && Math.abs(v) < 1.0) {
                break;
            }
        }

        assertTrue(
                "speeding-up exceeded the accel limit: " + maxSpeedingUpAccel,
                maxSpeedingUpAccel <= accel * 1.05);
        assertTrue(
                "braking did not use the larger decel limit: " + maxBrakingAccel,
                maxBrakingAccel > accel * 1.2);
        assertTrue(
                "braking exceeded the decel limit: " + maxBrakingAccel,
                maxBrakingAccel <= decel * 1.05);
    }

    /**
     * A faster velocity cap must not cost extra overshoot past the target. Regression test for the
     * stopping law that assumed braking could begin instantly: when the setpoint is still
     * accelerating toward the target, its acceleration must first swing to the braking side at the
     * jerk limit, and that swing distance was not charged against the stopping distance, so braking
     * began one accel-reversal too late. The setpoint then sailed past the target by an amount that
     * grew with speed. Here a long move under a high cap stays on the accelerating ramp when
     * braking must begin, while the same move under a lower cap reaches cruise (acceleration back
     * to zero) first; before the fix the high-cap run overshot ~0.063 rad and the capped run not at
     * all.
     */
    @Test
    public void fasterCapDoesNotCostExtraOvershoot() {
        double start = 0.5;
        double target = -1.0; // a 1.5 rad move, long enough for the low cap to reach cruise

        double fastOvershoot = reversalOvershoot(/* maxVelocity= */ 8.0, start, target);
        double cappedOvershoot = reversalOvershoot(/* maxVelocity= */ 4.0, start, target);

        // Going fast no longer costs meaningful extra overshoot (the bug's signature).
        assertTrue(
                "faster cap overshot more than the capped move (fast="
                        + fastOvershoot
                        + ", capped="
                        + cappedOvershoot
                        + ")",
                fastOvershoot < cappedOvershoot + 0.01);
        // And the fast move settles close on the target rather than sailing past it.
        assertTrue(
                "faster cap overshot the target: " + fastOvershoot + " rad", fastOvershoot < 0.02);
    }

    /**
     * Run a from-rest move from {@code start} to {@code target}, returning the max overshoot past
     * the target (in the direction of travel).
     */
    private double reversalOvershoot(double maxVelocity, double start, double target) {
        CascadedRateLimiter limiter =
                new CascadedRateLimiter(maxVelocity, A_MAX, A_MAX, J_MAX, start);
        double direction = Math.signum(target - start);
        double maxOvershoot = 0.0;
        for (int i = 0; i < 400; i++) { // 8 seconds, long enough to settle
            limiter.update(target, DT);
            maxOvershoot = Math.max(maxOvershoot, direction * (limiter.getPosition() - target));
        }
        return maxOvershoot;
    }

    /** A lowered velocity cap (as the controller does for back-EMF) is honored. */
    @Test
    public void honorsADynamicallyLoweredVelocityCap() {
        CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
        limiter.setMaxVelocity(1.0);
        double maxVelocity = 0.0;
        for (int i = 0; i < 200; i++) {
            limiter.update(30.0, DT);
            maxVelocity = Math.max(maxVelocity, Math.abs(limiter.getVelocity()));
        }
        assertTrue("ignored the lowered velocity cap: " + maxVelocity, maxVelocity <= 1.0 * 1.000001);
    }

    /**
     * A non-positive jerk limit is rejected rather than silently freezing the profile: a zero jerk
     * limit would clamp every acceleration change to nothing, so the setpoint could never move.
     * {@code NaN} is rejected too, since it would poison the profile. Callers who genuinely want no
     * jerk limit must ask for it explicitly with {@link CascadedRateLimiter#UNLIMITED_JERK}.
     */
    @Test
    public void rejectsNonPositiveJerkLimit() {
        for (double badJerk : new double[] {0.0, -1.0, Double.NaN}) {
            assertThrows(
                    "constructor accepted invalid maxJerk " + badJerk,
                    IllegalArgumentException.class,
                    () -> new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, badJerk, 0.0));
            CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
            assertThrows(
                    "setMaxJerk accepted invalid maxJerk " + badJerk,
                    IllegalArgumentException.class,
                    () -> limiter.setMaxJerk(badJerk));
        }
    }

    /**
     * Velocity, acceleration, and deceleration limits reject negatives and {@code NaN} rather than
     * silently clamping them to zero, so a bad limit surfaces at its source. Zero itself is
     * allowed: the controller lowers these ceilings to zero when the back-EMF headroom runs out.
     */
    @Test
    public void rejectsNegativeOrNaNMotionLimits() {
        for (double bad : new double[] {-1.0, Double.NaN}) {
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new CascadedRateLimiter(bad, A_MAX, A_MAX, J_MAX, 0.0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new CascadedRateLimiter(V_MAX, bad, A_MAX, J_MAX, 0.0));
            assertThrows(
                    IllegalArgumentException.class,
                    () -> new CascadedRateLimiter(V_MAX, A_MAX, bad, J_MAX, 0.0));
            CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
            assertThrows(IllegalArgumentException.class, () -> limiter.setMaxVelocity(bad));
            assertThrows(IllegalArgumentException.class, () -> limiter.setMaxAcceleration(bad));
            assertThrows(IllegalArgumentException.class, () -> limiter.setMaxDeceleration(bad));
        }
        // Zero ceilings are accepted (constructor and setters do not throw).
        CascadedRateLimiter zeroed = new CascadedRateLimiter(0.0, 0.0, 0.0, J_MAX, 0.0);
        zeroed.setMaxVelocity(0.0);
        zeroed.setMaxAcceleration(0.0);
        zeroed.setMaxDeceleration(0.0);
    }

    /**
     * A zero deceleration ceiling (which the controller can set when the braking headroom runs out)
     * must not poison the profile with NaN. With no braking authority the setpoint simply holds its
     * approach speed instead of dividing by zero in the stopping law.
     */
    @Test
    public void zeroDecelerationCeilingDoesNotProduceNaN() {
        CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
        limiter.setMaxDeceleration(0.0);
        for (int i = 0; i < 100; i++) {
            limiter.update(30.0, DT);
            assertFalse(
                    "zero deceleration ceiling produced NaN",
                    Double.isNaN(limiter.getVelocity()) || Double.isNaN(limiter.getPosition()));
        }
    }

    /**
     * {@link CascadedRateLimiter#UNLIMITED_JERK} disables the jerk limit: the acceleration may step
     * straight to its limit, but velocity and acceleration limits still hold and the profile
     * settles on the target without producing NaN.
     */
    @Test
    public void unlimitedJerkStillRespectsVelocityAndAcceleration() {
        CascadedRateLimiter limiter =
                new CascadedRateLimiter(
                        V_MAX, A_MAX, A_MAX, CascadedRateLimiter.UNLIMITED_JERK, 0.0);
        double target = 30.0;
        double maxVelocity = 0.0;
        double maxAcceleration = 0.0;
        for (int i = 0; i < 400; i++) {
            limiter.update(target, DT);
            maxVelocity = Math.max(maxVelocity, Math.abs(limiter.getVelocity()));
            maxAcceleration = Math.max(maxAcceleration, Math.abs(limiter.getAcceleration()));
            assertFalse(
                    "profile went NaN under unlimited jerk", Double.isNaN(limiter.getPosition()));
        }
        assertTrue(
                "acceleration exceeded its limit under unlimited jerk: " + maxAcceleration,
                maxAcceleration <= A_MAX * 1.001);
        assertTrue(
                "velocity exceeded its limit under unlimited jerk: " + maxVelocity,
                maxVelocity <= V_MAX * 1.000001);
        assertTrue(
                "did not settle on target under unlimited jerk: " + limiter.getPosition(),
                Math.abs(limiter.getPosition() - target) < 0.01);
    }

    /**
     * Under realistic loop-timing jitter (16 ms ± 4 ms, as in ControlLab), the profile must still
     * settle to rest on the target without a sustained velocity ring. Regression for the
     * inverse-dt command path: a short sample inflated {@code error/dt} and {@code (Δ)/dt}, and
     * the next longer sample integrated that energy past the target.
     */
    @Test
    public void settlesWithoutVelocityRingUnderDtJitter() {
        final double nominalDt = 0.016;
        final double jitter = 0.008; // +/- 4 ms
        final double target = 1.5708;
        final long seed = 42L;

        CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
        Random rng = new Random(seed);

        // Run long enough to arrive and hold.
        for (int i = 0; i < 500; i++) {
            double dt = nominalDt + (rng.nextDouble() - 0.5) * jitter;
            limiter.update(target, dt);
        }

        // Hold window: profile must be at rest, not ringing.
        double maxHoldAbsVel = 0.0;
        double maxHoldAbsAccel = 0.0;
        double maxHoldPosError = 0.0;
        for (int i = 0; i < 200; i++) {
            double dt = nominalDt + (rng.nextDouble() - 0.5) * jitter;
            limiter.update(target, dt);
            maxHoldAbsVel = Math.max(maxHoldAbsVel, Math.abs(limiter.getVelocity()));
            maxHoldAbsAccel = Math.max(maxHoldAbsAccel, Math.abs(limiter.getAcceleration()));
            maxHoldPosError =
                    Math.max(maxHoldPosError, Math.abs(limiter.getPosition() - target));
        }

        assertTrue(
                "did not settle on target under dt jitter: error=" + maxHoldPosError,
                maxHoldPosError < 1e-6);
        assertTrue(
                "velocity still ringing under dt jitter: |v|max=" + maxHoldAbsVel,
                maxHoldAbsVel < 1e-6);
        assertTrue(
                "acceleration still dithering under dt jitter: |a|max=" + maxHoldAbsAccel,
                maxHoldAbsAccel < 1e-6);
    }

    /**
     * Extreme short samples (below {@link CascadedRateLimiter#MIN_COMMAND_DT}) must not inflate
     * landing or discrete-derivative commands into a ring when mixed with longer steps.
     */
    @Test
    public void shortSamplesDoNotInflateCommandsIntoARing() {
        CascadedRateLimiter limiter = new CascadedRateLimiter(V_MAX, A_MAX, A_MAX, J_MAX, 0.0);
        double target = 0.5;
        // Alternating 1 ms and 20 ms steps — worst-case inverse-dt swing before the floor.
        double[] steps = {0.001, 0.020};
        for (int i = 0; i < 800; i++) {
            limiter.update(target, steps[i % 2]);
        }
        assertTrue(
                "did not settle under alternating short/long dt: pos=" + limiter.getPosition(),
                Math.abs(limiter.getPosition() - target) < 1e-6);
        assertTrue(
                "velocity ring under alternating short/long dt: v=" + limiter.getVelocity(),
                Math.abs(limiter.getVelocity()) < 1e-6);
        assertTrue(
                "acceleration dither under alternating short/long dt: a="
                        + limiter.getAcceleration(),
                Math.abs(limiter.getAcceleration()) < 1e-6);
    }
}
