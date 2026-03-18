package org.marsroboticsassociation.controllib.motion;

import org.junit.jupiter.api.Test;

import java.util.concurrent.atomic.AtomicLong;

import static org.junit.jupiter.api.Assertions.*;

class PositionTrajectoryManagerTest {

    // Silent telemetry stub
    private static final org.marsroboticsassociation.controllib.util.TelemetryAddData NO_OP_TELEMETRY =
            (caption, format, value) -> {};

    /** Build a manager with a controllable clock (nanoseconds). */
    private static PositionTrajectoryManager makeManager(AtomicLong clockNs,
                                                         double vMax, double aMaxAccel,
                                                         double aMaxDecel, double jMax,
                                                         double pTol) {
        return new PositionTrajectoryManager(
                vMax, aMaxAccel, aMaxDecel, jMax, pTol, NO_OP_TELEMETRY,
                clockNs::get);
    }

    // ---------------------------------------------------------------

    @Test
    void setTarget_plansTrajectory_positionMovesTowardTarget() {
        AtomicLong clock = new AtomicLong(0);
        PositionTrajectoryManager m = makeManager(clock, 5, 3, 3, 10, 0.01);

        m.setTarget(50.0);

        // Estimate an upper bound on travel time: 50 / 5 = 10 s, add 4 s margin
        long tfNs = (long) 14e9;
        clock.set(tfNs);
        m.update();

        assertEquals(50.0, m.getPosition(), 1e-2, "should reach target after tf");
        assertEquals(0.0,  m.getVelocity(),  0.1,  "should be at rest");
    }

    @Test
    void resetFromMeasurement_rebuildsFromInjectedState() {
        AtomicLong clock = new AtomicLong(0);
        PositionTrajectoryManager m = makeManager(clock, 5, 3, 3, 10, 0.01);

        m.setTarget(100.0);

        // Advance partway, then inject a measurement
        clock.set((long) 2e9);
        m.update();
        m.resetFromMeasurement(20.0, 0.0, 0.0);

        // Verify getters reflect injected state immediately
        assertEquals(20.0, m.getPosition(), 1e-9);
        assertEquals(0.0,  m.getVelocity(),  1e-9);

        // Advance well past end of new trajectory
        clock.set((long) 30e9);
        m.update();
        assertEquals(100.0, m.getPosition(), 1e-2, "should still reach original target");
    }

    @Test
    void targetChange_withinTolerance_doesNotReplan() {
        AtomicLong clock = new AtomicLong(0);
        // Tolerance = 1.0 unit
        PositionTrajectoryManager m = makeManager(clock, 5, 3, 3, 10, 1.0);

        m.setTarget(50.0);

        // Advance a bit so the trajectory starts
        clock.set((long) 0.5e9);
        m.update();
        double posAfterFirst = m.getPosition();

        // Command a target within tolerance — should NOT replan
        m.setTarget(50.4);

        clock.set((long) 1e9);
        m.update();

        // If replanning had occurred, the position trajectory would restart from 0
        // and the value would likely be < posAfterFirst. Without replan, it continues.
        assertTrue(m.getPosition() >= posAfterFirst,
                "position should continue increasing (no replan within tolerance)");
    }

    @Test
    void updateConfig_affectsNextTrajectory() {
        AtomicLong clock = new AtomicLong(0);
        PositionTrajectoryManager m = makeManager(clock, 5, 3, 3, 10, 0.01);

        m.setTarget(100.0);
        clock.set((long) 1e9);
        m.update();
        double tfFast = new SCurvePosition(0, 100, 0, 0, 5, 3, 3, 10).getTotalTime();

        // Slow it down
        m.updateConfig(2, 1, 1, 5);
        m.resetFromMeasurement(0, 0, 0);

        double tfSlow = new SCurvePosition(0, 100, 0, 0, 2, 1, 1, 5).getTotalTime();

        assertTrue(tfSlow > tfFast, "slower config should take longer");
        clock.set((long) (tfSlow * 1.05e9));
        m.update();
        assertEquals(100.0, m.getPosition(), 0.5, "should reach target with slow config");
    }

    @Test
    void getters_returnCachedValues_fromLastUpdate() {
        AtomicLong clock = new AtomicLong(0);
        PositionTrajectoryManager m = makeManager(clock, 5, 3, 3, 10, 0.01);

        m.setTarget(50.0);
        clock.set((long) 1e9);
        m.update();

        // Advance clock further WITHOUT calling update — getters should return stale values
        clock.set((long) 5e9);

        double p = m.getPosition();
        double v = m.getVelocity();
        double a = m.getAcceleration();

        // Call update now
        m.update();

        // After update the position should have advanced (trajectory is in progress)
        assertTrue(m.getPosition() > p,
                "position should increase after update at later time; cached=" + p
                + " new=" + m.getPosition());
        assertTrue(Double.isFinite(m.getVelocity()));
        assertTrue(Double.isFinite(m.getAcceleration()));
    }
}
