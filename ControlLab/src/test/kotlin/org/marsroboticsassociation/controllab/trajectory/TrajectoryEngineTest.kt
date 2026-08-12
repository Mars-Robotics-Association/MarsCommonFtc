package org.marsroboticsassociation.controllab.trajectory

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment

class TrajectoryEngineTest {

    @Test
    fun scurvePosition_initialState_atZeroAtRest() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        assertEquals(0.0, engine.getPosition(), 1e-9)
        assertEquals(0.0, engine.getVelocity(), 1e-9)
        assertEquals(0.0, engine.getAcceleration(), 1e-9)
        assertFalse(engine.isMoving())
        assertTrue(engine.hasPosition())
    }

    @Test
    fun scurvePosition_goTo_eventuallyReachesTarget() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)
        assertTrue(engine.isMoving())

        // Step 30 seconds worth of 20 ms ticks
        for (i in 0 until 1500) engine.tick()

        assertFalse(engine.isMoving(), "should have stopped")
        assertEquals(100.0, engine.getPosition(), 0.1)
        assertEquals(0.0, engine.getVelocity(), 0.1)
    }

    @Test
    fun scurvePosition_interrupt_replansFromCurrentState() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)

        // Advance partway
        for (i in 0 until 50) engine.tick() // 1 second
        val midPos = engine.getPosition()
        assertTrue(midPos > 0 && midPos < 100, "should be mid-trajectory")

        // Interrupt — go back toward -100
        engine.applyParamsAndGoTo(-100.0)
        assertTrue(engine.isMoving())

        // Advance to completion
        for (i in 0 until 2500) engine.tick()
        assertEquals(-100.0, engine.getPosition(), 0.5)
    }

    @Test
    fun scurvePosition_newParamsApplyOnButtonPress_notImmediately() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)

        // Change params mid-move — should NOT take effect yet
        engine.setPositionParams(2.0, 1.0, 1.0, 5.0) // much slower
        for (i in 0 until 50) engine.tick() // 1.0 s

        // Position should still be progressing at the original (fast) rate.
        // At 1.0 s with vMax=10, aMax=5, jMax=50, position ≈ 2.26 (clearly > 1.0).
        // If slow params (vMax=2, aMax=1) had taken effect it would be only ~0.09.
        assertTrue(engine.getPosition() > 1.0, "original fast params still in effect")

        // Now press button — new params apply
        engine.applyParamsAndGoTo(0.0)
        for (i in 0 until 3000) engine.tick()
        assertEquals(0.0, engine.getPosition(), 0.5)
    }

    @Test
    fun sinCurvePosition_initialState_atZeroAtRest() {
        val engine = TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        assertEquals(0.0, engine.getPosition(), 1e-9)
        assertEquals(0.0, engine.getVelocity(), 1e-9)
        assertEquals(0.0, engine.getAcceleration(), 1e-9)
        assertFalse(engine.isMoving())
        assertTrue(engine.hasPosition())
    }

    @Test
    fun sinCurvePosition_goTo_eventuallyReachesTarget() {
        val engine = TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)
        assertTrue(engine.isMoving())

        // Step 30 seconds worth of 20 ms ticks
        for (i in 0 until 1500) engine.tick()

        assertFalse(engine.isMoving(), "should have stopped")
        assertEquals(100.0, engine.getPosition(), 0.1)
        assertEquals(0.0, engine.getVelocity(), 0.1)
    }

    @Test
    fun scurveVelocity_hasNoPosition() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY)
        engine.setVelocityParams(1197.0, 2669.0, 800.0)
        assertFalse(engine.hasPosition())
    }

    @Test
    fun scurveVelocity_goTo_eventuallyReachesTargetVelocity() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY)
        engine.setVelocityParams(1197.0, 2669.0, 800.0)
        engine.applyParamsAndGoTo(3000.0)

        for (i in 0 until 500) engine.tick()

        assertFalse(engine.isMoving(), "should have settled")
        assertEquals(3000.0, engine.getVelocity(), 5.0)
    }

    @Test
    fun svgExport_supportedForPositionTrajectoryTypes() {
        assertTrue(TrajectoryEngine(TrajectoryType.SCURVE_POSITION).supportsExactSvgExport())
        assertTrue(TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY).supportsExactSvgExport())
        assertTrue(TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION).supportsExactSvgExport())
    }

    @Test
    fun scurvePosition_exactSvgModel_containsExpectedSeries() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)

        val model = requireNotNull(engine.buildExactSvgModel())

        assertEquals(4, model.series.size)
        assertTrue(model.xMax > model.xMin)
        assertFalse(model.series[0].segments.isEmpty())
    }

    @Test
    fun scurveVelocity_exactSvgModel_containsExpectedSeries() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY)
        engine.setVelocityParams(1197.0, 2669.0, 800.0)
        engine.applyParamsAndGoTo(3000.0)

        val model = requireNotNull(engine.buildExactSvgModel())

        assertEquals(3, model.series.size)
        assertTrue(model.xMax > model.xMin)
        assertFalse(model.series[0].segments.isEmpty())
    }

    @Test
    fun sinCurvePosition_svgModel_containsExpectedSeries() {
        val engine = TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)

        val model = requireNotNull(engine.buildExactSvgModel())

        assertEquals(4, model.series.size)
        assertTrue(model.xMax > model.xMin)
        assertFalse(model.series[0].segments.isEmpty())
        assertFalse(model.series[0].segments[0] is PolynomialCurveSegment)
    }

    @Test
    fun scurvePosition_svgModel_retainsPolynomialSegments() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)

        val model = requireNotNull(engine.buildExactSvgModel())

        assertTrue(
            model.series
                .stream()
                .flatMap { series -> series.segments.stream() }
                .allMatch(PolynomialCurveSegment::class.java::isInstance)
        )
    }
}
