package org.marsroboticsassociation.controllib.mechanism;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class RuckigProfilerTest {

    private static final double DT = 0.016; // typical FTC loop

    /** Step until the profile settles exactly on the target, with a wall-clock budget. */
    private static int runToTarget(RuckigProfiler p, double target, double dt, double maxSimTime) {
        int steps = 0;
        int budget = (int) Math.ceil(maxSimTime / dt);
        while (steps < budget) {
            p.update(target, dt);
            steps++;
            if (p.getPosition() == target && p.getVelocity() == 0.0 && p.getAcceleration() == 0.0) {
                return steps;
            }
        }
        throw new AssertionError("did not settle on " + target + " within " + maxSimTime
                + " s: p=" + p.getPosition() + " v=" + p.getVelocity() + " a=" + p.getAcceleration());
    }

    // ---------------------------------------------------------------
    // The headline: stops are planned, not clamped
    // ---------------------------------------------------------------

    @Test
    void stopIsJerkBoundedAllTheWayIntoTheTarget() {
        double maxV = 2.0, maxA = 4.0, maxD = 4.0, maxJ = 20.0;
        RuckigProfiler p = new RuckigProfiler(maxV, maxA, maxD, maxJ, 0.0);

        double prevA = p.getAcceleration();
        double prevV = p.getVelocity();
        double maxJerkObserved = 0.0;
        int steps = 0;
        while (steps < 2000) {
            p.update(1.0, DT);
            steps++;
            maxJerkObserved = Math.max(maxJerkObserved, Math.abs(p.getAcceleration() - prevA) / DT);
            assertTrue(Math.abs(p.getVelocity()) <= maxV + 1e-9, "velocity limit");
            assertTrue(Math.abs(p.getVelocity() - prevV) <= Math.max(maxA, maxD) * DT + 1e-9,
                    "accel limit step " + steps);
            prevA = p.getAcceleration();
            prevV = p.getVelocity();
            if (p.getPosition() == 1.0 && p.getVelocity() == 0.0 && p.getAcceleration() == 0.0) {
                break;
            }
        }
        // Lands exactly (the final step snaps when the remaining plan fits inside dt)
        assertEquals(1.0, p.getPosition(), 0.0, "exact landing");
        assertEquals(0.0, p.getVelocity(), 0.0);
        // Jerk stays bounded through the entire stop, including the very last steps into the
        // target: the deceleration is planned, not clamped.
        assertTrue(maxJerkObserved <= maxJ * 1.05 + 1e-9,
                "jerk bounded through the stop; observed " + maxJerkObserved);
        assertTrue(steps < 2000, "settled");
    }

    @Test
    void asymmetricDecelInNegativeDirectionStaysInTravelFrameBounds() {
        double aAccel = 1.0, aDecel = 5.0;
        RuckigProfiler p = new RuckigProfiler(1.0, aAccel, aDecel, 10.0, 0.0);
        int steps = runToTarget(p, -1.0, DT, 10.0);
        assertTrue(steps > 10);

        // Re-run sampling accelerations: moving in −, speeding up is a >= -aAccel,
        // braking is a <= +aDecel.
        RuckigProfiler q = new RuckigProfiler(1.0, aAccel, aDecel, 10.0, 0.0);
        for (int i = 0; i < steps; ++i) {
            q.update(-1.0, DT);
            double a = q.getAcceleration();
            assertTrue(a >= -aAccel - 1e-9, "accel bound (speeding up in −): " + a);
            assertTrue(a <= aDecel + 1e-9, "decel bound (braking in −): " + a);
        }
    }

    // ---------------------------------------------------------------
    // Back-EMF-style per-loop limit rewrites
    // ---------------------------------------------------------------

    @Test
    void survivesBackEmfStyleAccelDecay() {
        // Accel authority collapses with speed (back-EMF), braking held at a conservative
        // constant — the §7 usage pattern. The profile must stay continuous and still land.
        double maxV = 2.0, maxJ = 20.0;
        RuckigProfiler p = new RuckigProfiler(maxV, 4.0, 2.0, maxJ, 0.0);

        double prevP = p.getPosition();
        double prevA = p.getAcceleration();
        boolean settled = false;
        for (int i = 0; i < 1500; ++i) {
            double speed = Math.abs(p.getVelocity());
            p.setMaxAcceleration(Math.max(0.0, 4.0 - 1.8 * speed));
            p.setMaxDeceleration(2.0); // conservative braking ceiling
            p.update(3.0, DT);
            assertTrue(Math.abs(p.getPosition() - prevP) <= maxV * DT + 1e-9,
                    "position continuous at step " + i);
            assertTrue(Math.abs(p.getAcceleration() - prevA) <= maxJ * DT * 1.05 + 1e-9,
                    "jerk bounded under decaying limits at step " + i);
            prevP = p.getPosition();
            prevA = p.getAcceleration();
            if (p.getPosition() == 3.0 && p.getVelocity() == 0.0) {
                settled = true;
                break;
            }
        }
        assertTrue(settled, "lands despite per-loop limit rewrites");
    }

    @Test
    void velocityCapDroppedBelowCurrentSpeedBrakesViaPreTrajectory() {
        RuckigProfiler p = new RuckigProfiler(2.0, 4.0, 4.0, 20.0, 0.0);
        // Get up to speed
        while (p.getVelocity() < 1.5) {
            p.update(10.0, DT);
        }
        // Drop the cap below current speed: Ruckig absorbs this with a brake pre-trajectory.
        p.setMaxVelocity(0.75);
        double prevV = p.getVelocity();
        boolean belowCap = false;
        for (int i = 0; i < 1500; ++i) {
            p.update(10.0, DT);
            assertTrue(Math.abs(p.getVelocity() - prevV) <= 4.0 * DT + 1e-9,
                    "velocity continuous while braking to the new cap");
            prevV = p.getVelocity();
            if (belowCap) {
                assertTrue(p.getVelocity() <= 0.75 + 1e-9, "stays under the new cap");
            } else if (p.getVelocity() <= 0.75) {
                belowCap = true;
            }
            if (p.getPosition() == 10.0 && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertTrue(belowCap, "came down to the new velocity cap");
        assertEquals(10.0, p.getPosition(), 0.0, "still lands on the target");
    }

    @Test
    void velocityCeilingChatterDoesNotFreezeTheProfile() {
        // Regression: MotorMechanismController evaluates the back-EMF velocity ceiling at the
        // pre-update profile state, so a profile cruising at the ceiling sits a hair ABOVE each
        // freshly lowered ceiling — while the accel ceiling is ~0 by construction at sustainable
        // velocity. From that exact state Ruckig has no feasible plan (result -110), and a
        // hold-on-error fallback freezes the profile forever. The profiler must clamp into the
        // band and keep making progress instead.
        RuckigProfiler p = new RuckigProfiler(2.0, 4.0, 4.0, 480.0, 0.0);
        while (p.getVelocity() < 1.99) {
            p.update(50.0, DT);
        }
        double posBefore = p.getPosition();
        double velBefore = p.getVelocity();
        for (int i = 0; i < 60; ++i) {
            // Each loop the ceiling lands just below the current speed and the speed-up
            // authority is revoked — the exact controller-induced chatter regime.
            p.setMaxVelocity(p.getVelocity() * 0.997);
            p.setMaxAcceleration(0.0);
            p.update(50.0, DT);
        }
        double advanced = p.getPosition() - posBefore;
        assertTrue(advanced > velBefore * DT * 60 * 0.5,
                "profile keeps moving through ceiling chatter; advanced=" + advanced);
        assertTrue(p.getVelocity() <= velBefore + 1e-9, "no speed gained without authority");
        assertTrue(p.getVelocity() > 0.5 * velBefore, "no spurious hard braking either");
    }

    @Test
    void zeroVelocityAuthorityHoldsPosition() {
        RuckigProfiler p = new RuckigProfiler(2.0, 4.0, 4.0, 20.0, 0.0);
        p.setMaxVelocity(0.0);
        for (int i = 0; i < 200; ++i) {
            p.update(1.0, DT);
        }
        assertTrue(Math.abs(p.getPosition()) < 1e-6, "no authority: setpoint holds");
    }

    // ---------------------------------------------------------------
    // Robustness
    // ---------------------------------------------------------------

    @Test
    void unlimitedJerkRunsSecondOrderProfiles() {
        double maxA = 3.0;
        RuckigProfiler p = new RuckigProfiler(1.5, maxA, maxA, RuckigProfiler.UNLIMITED_JERK, 0.0);
        double prevV = p.getVelocity();
        int steps = 0;
        while (steps < 1000) {
            p.update(2.0, DT);
            steps++;
            assertTrue(Math.abs(p.getVelocity() - prevV) <= maxA * DT * 1.05 + 1e-9,
                    "second-order accel bound");
            prevV = p.getVelocity();
            if (p.getPosition() == 2.0 && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(2.0, p.getPosition(), 0.0);
    }

    @Test
    void jitteryDtStillLandsWithBoundedJerk() {
        double maxJ = 20.0;
        RuckigProfiler p = new RuckigProfiler(2.0, 4.0, 4.0, maxJ, 0.0);
        Random rng = new Random(42);
        double prevA = p.getAcceleration();
        int steps = 0;
        while (steps < 2000) {
            double dt = 0.005 + rng.nextDouble() * 0.020; // 5–25 ms
            p.update(-2.0, dt);
            steps++;
            assertTrue(Math.abs(p.getAcceleration() - prevA) <= maxJ * dt * 1.05 + 1e-9,
                    "jerk bounded under dt jitter at step " + steps);
            prevA = p.getAcceleration();
            if (p.getPosition() == -2.0 && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(-2.0, p.getPosition(), 0.0);
        assertTrue(steps < 2000, "settled");
    }

    @Test
    void invalidDtAndTargetAreNoOps() {
        RuckigProfiler p = new RuckigProfiler(2.0, 4.0, 4.0, 20.0, 5.0);
        p.update(1.0, DT); // move a little
        double pos = p.getPosition();
        double vel = p.getVelocity();
        p.update(1.0, 0.0);
        p.update(1.0, -0.1);
        p.update(1.0, Double.NaN);
        p.update(Double.NaN, DT);
        p.update(Double.POSITIVE_INFINITY, DT);
        assertEquals(pos, p.getPosition(), 0.0, "no-op inputs leave state untouched");
        assertEquals(vel, p.getVelocity(), 0.0);
    }

    @Test
    void resetSnapsToRest() {
        RuckigProfiler p = new RuckigProfiler(2.0, 4.0, 4.0, 20.0, 0.0);
        for (int i = 0; i < 20; ++i) {
            p.update(5.0, DT);
        }
        p.reset(1.25);
        assertEquals(1.25, p.getPosition(), 0.0);
        assertEquals(0.0, p.getVelocity(), 0.0);
        assertEquals(0.0, p.getAcceleration(), 0.0);
    }

    @Test
    void validatesLimitsAtConstructionAndRewrite() {
        assertThrows(IllegalArgumentException.class,
                () -> new RuckigProfiler(-1.0, 1, 1, 1, 0));
        assertThrows(IllegalArgumentException.class,
                () -> new RuckigProfiler(1, 1, 1, 0.0, 0), "zero jerk rejected");
        assertThrows(IllegalArgumentException.class,
                () -> new RuckigProfiler(1, 1, 1, Double.NaN, 0));
        RuckigProfiler p = new RuckigProfiler(1, 1, 1, 1, 0);
        assertThrows(IllegalArgumentException.class, () -> p.setMaxDeceleration(-2));
        p.setMaxVelocity(0.0); // zero is a legitimate ceiling ("no authority")
    }
}
