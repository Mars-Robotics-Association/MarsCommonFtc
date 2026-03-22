package org.marsroboticsassociation.controllab.trajectory;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

class TrajectoryEngineTest {

    @Test
    void scurvePosition_initialState_atZeroAtRest() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        assertEquals(0.0, engine.getPosition(), 1e-9);
        assertEquals(0.0, engine.getVelocity(), 1e-9);
        assertEquals(0.0, engine.getAcceleration(), 1e-9);
        assertFalse(engine.isMoving());
        assertTrue(engine.hasPosition());
    }

    @Test
    void scurvePosition_goTo_eventuallyReachesTarget() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        engine.applyParamsAndGoTo(100.0);
        assertTrue(engine.isMoving());

        // Step 30 seconds worth of 20 ms ticks
        for (int i = 0; i < 1500; i++) engine.tick();

        assertFalse(engine.isMoving(), "should have stopped");
        assertEquals(100.0, engine.getPosition(), 0.1);
        assertEquals(0.0, engine.getVelocity(), 0.1);
    }

    @Test
    void scurvePosition_interrupt_replansFromCurrentState() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        engine.applyParamsAndGoTo(100.0);

        // Advance partway
        for (int i = 0; i < 50; i++) engine.tick(); // 1 second
        double midPos = engine.getPosition();
        assertTrue(midPos > 0 && midPos < 100, "should be mid-trajectory");

        // Interrupt — go back toward -100
        engine.applyParamsAndGoTo(-100.0);
        assertTrue(engine.isMoving());

        // Advance to completion
        for (int i = 0; i < 2500; i++) engine.tick();
        assertEquals(-100.0, engine.getPosition(), 0.5);
    }

    @Test
    void scurvePosition_newParamsApplyOnButtonPress_notImmediately() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        engine.applyParamsAndGoTo(100.0);

        // Change params mid-move — should NOT take effect yet
        engine.setPositionParams(2, 1, 1, 5); // much slower
        for (int i = 0; i < 50; i++) engine.tick(); // 1.0 s

        // Position should still be progressing at the original (fast) rate.
        // At 1.0 s with vMax=10, aMax=5, jMax=50, position ≈ 2.26 (clearly > 1.0).
        // If slow params (vMax=2, aMax=1) had taken effect it would be only ~0.09.
        assertTrue(engine.getPosition() > 1.0, "original fast params still in effect");

        // Now press button — new params apply
        engine.applyParamsAndGoTo(0.0);
        for (int i = 0; i < 3000; i++) engine.tick();
        assertEquals(0.0, engine.getPosition(), 0.5);
    }

    @Test
    void sinCurvePosition_initialState_atZeroAtRest() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        assertEquals(0.0, engine.getPosition(), 1e-9);
        assertEquals(0.0, engine.getVelocity(), 1e-9);
        assertEquals(0.0, engine.getAcceleration(), 1e-9);
        assertFalse(engine.isMoving());
        assertTrue(engine.hasPosition());
    }

    @Test
    void sinCurvePosition_goTo_eventuallyReachesTarget() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        engine.applyParamsAndGoTo(100.0);
        assertTrue(engine.isMoving());

        // Step 30 seconds worth of 20 ms ticks
        for (int i = 0; i < 1500; i++) engine.tick();

        assertFalse(engine.isMoving(), "should have stopped");
        assertEquals(100.0, engine.getPosition(), 0.1);
        assertEquals(0.0, engine.getVelocity(), 0.1);
    }

    @Test
    void scurveVelocity_hasNoPosition() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY);
        engine.setVelocityParams(1197, 2669, 800);
        assertFalse(engine.hasPosition());
    }

    @Test
    void scurveVelocity_goTo_eventuallyReachesTargetVelocity() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY);
        engine.setVelocityParams(1197, 2669, 800);
        engine.applyParamsAndGoTo(3000.0);

        for (int i = 0; i < 500; i++) engine.tick();

        assertFalse(engine.isMoving(), "should have settled");
        assertEquals(3000.0, engine.getVelocity(), 5.0);
    }

    @Test
    void ruckig_goTo_eventuallyReachesTarget() {
        boolean available = TrajectoryEngine.isRuckigAvailable();
        Assumptions.assumeTrue(available, "Ruckig JNI not available; skipping");

        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.RUCKIG);
        engine.setRuckigParams(10, 5, 50);
        engine.applyParamsAndGoTo(100.0);

        for (int i = 0; i < 1500; i++) engine.tick();
        assertFalse(engine.isMoving());
        assertEquals(100.0, engine.getPosition(), 0.5);
        assertEquals(0.0, engine.getVelocity(), 0.1);
    }
}
