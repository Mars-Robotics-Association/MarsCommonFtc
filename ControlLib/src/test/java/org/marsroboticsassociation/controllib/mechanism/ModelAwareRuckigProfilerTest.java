package org.marsroboticsassociation.controllib.mechanism;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ModelAwareRuckigProfilerTest {

    private static final double DT = 0.016;
    private static final double AVAILABLE_VOLTS = 10.5; // 12 V bus minus 1.5 V feedback margin

    /** High-kV arm: the sustainable-velocity ceiling sits far below the mechanical cap. */
    private static ArmModel highKvArm() {
        return new ArmModel(0.3, 4.0, 0.35, 3.5, 0.0);
    }

    private static ModelAwareRuckigProfiler profiler(ArmModel model, double p0) {
        ModelAwareRuckigProfiler p = new ModelAwareRuckigProfiler(model, 8.0, 12.0, 480.0, p0);
        p.setAvailableVoltage(AVAILABLE_VOLTS);
        return p;
    }

    @Test
    void velocityNeverExceedsTheLocalModelCeiling() {
        // A falling-ceiling stretch: a
        // descent from the back stop (225°) toward 90°, where the sustainable-velocity ceiling
        // drops as the arm approaches straight-down-over-the-top (180°). The profiler's own
        // one-step lookahead must keep the cruise inside the ceiling at every sample.
        ArmModel model = highKvArm();
        double target = Math.toRadians(90);
        ModelAwareRuckigProfiler p = profiler(model, Math.toRadians(225));

        int steps = 0;
        while (steps < 3000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS);
            p.update(target, DT);
            steps++;
            double ceiling = model.maxSustainableVelocity(
                    AVAILABLE_VOLTS, p.getPosition(), target - p.getPosition());
            assertTrue(Math.abs(p.getVelocity()) <= ceiling + 1e-3,
                    "cruise inside the local ceiling at step " + steps
                            + ": |v|=" + Math.abs(p.getVelocity()) + " ceiling=" + ceiling);
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(target, p.getPosition(), 0.0, "arrives exactly");
        assertTrue(steps < 3000, "settled");
    }

    @Test
    void stopStaysJerkBoundedUnderModelCeilings() {
        ArmModel model = highKvArm();
        double target = 0.0;
        ModelAwareRuckigProfiler p = profiler(model, Math.toRadians(90));

        double prevA = p.getAcceleration();
        int steps = 0;
        while (steps < 3000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS);
            p.update(target, DT);
            steps++;
            assertTrue(Math.abs(p.getAcceleration() - prevA) <= 480.0 * DT * 1.05 + 1e-9,
                    "jerk bounded under model ceilings at step " + steps);
            prevA = p.getAcceleration();
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(target, p.getPosition(), 0.0);
        assertTrue(steps < 3000, "settled");
    }

    @Test
    void heavyGravityDescentRecoversFromOvershootAndClimbsBack() {
        // With heavy gravity (kCos=8 against 10.5 V), the
        // braking ceiling collapses toward a horizontal target, so a descent can carry wrong-way
        // speed past the target. Pushing back toward the target is physically braking, so it must
        // get the braking authority; a limit mapping keyed to the travel direction hands it the
        // acceleration ceiling instead — zero at speed against heavy gravity — and the profile
        // freezes just past the target forever (no plan, and the clamped state needs the same
        // missing authority).
        ArmModel model = new ArmModel(0.3, 1.2, 0.35, 8.0, 0.0);
        double target = 0.0;
        ModelAwareRuckigProfiler p =
                new ModelAwareRuckigProfiler(model, 8.0, 12.0, 480.0, Math.toRadians(225));

        double maxOvershootDeg = 0.0;
        int steps = 0;
        while (steps < 4000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS);
            p.update(target, DT);
            steps++;
            maxOvershootDeg = Math.max(maxOvershootDeg, Math.toDegrees(target - p.getPosition()));
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(target, p.getPosition(), 0.0, "descent lands at the target");
        assertTrue(steps < 4000, "descent settled");
        // The target-position braking ceiling keeps the planned stop honest: overshoot past the
        // horizontal target stays negligible.
        assertTrue(maxOvershootDeg < 0.5, "overshoot bounded: " + maxOvershootDeg + "°");

        // From the landed state, climbing back against the gravity must also work.
        double up = Math.toRadians(90);
        steps = 0;
        while (steps < 4000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS);
            p.update(up, DT);
            steps++;
            if (p.getPosition() == up && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(up, p.getPosition(), 0.0, "ascent arrives against heavy gravity");
        assertTrue(steps < 4000, "ascent settled");
    }

    @Test
    void noVoltageMeansNoAuthority() {
        // setAvailableVoltage never called after construction default of zero: ceilings are all
        // zero, the setpoint must hold rather than move or teleport.
        ArmModel model = highKvArm();
        ModelAwareRuckigProfiler p =
                new ModelAwareRuckigProfiler(model, 8.0, 12.0, 480.0, Math.toRadians(45));
        for (int i = 0; i < 100; ++i) {
            p.update(Math.toRadians(90), DT);
        }
        assertEquals(Math.toRadians(45), p.getPosition(), 1e-6, "no volts, no motion");
    }

    @Test
    void mechanicalCapsStillBindWhenTheModelAllowsMore() {
        // Weak gravity, low kV: the model ceilings sit far above the configured caps, so the
        // configured caps must be the binding limits.
        ArmModel model = new ArmModel(0.1, 0.5, 0.2, 0.5, 0.0);
        double maxVel = 2.0;
        ModelAwareRuckigProfiler p = new ModelAwareRuckigProfiler(model, maxVel, 6.0, 100.0, 0.0);
        double target = 10.0;
        double peakVel = 0.0;
        int steps = 0;
        while (steps < 3000) {
            p.setAvailableVoltage(AVAILABLE_VOLTS);
            p.update(target, DT);
            steps++;
            peakVel = Math.max(peakVel, Math.abs(p.getVelocity()));
            if (p.getPosition() == target && p.getVelocity() == 0.0) {
                break;
            }
        }
        assertEquals(target, p.getPosition(), 0.0);
        assertTrue(peakVel <= maxVel + 1e-9, "configured cap binds: peak=" + peakVel);
        assertTrue(peakVel > maxVel * 0.95, "cruises near the configured cap: peak=" + peakVel);
    }
}
