package org.marsroboticsassociation.controllab.arm

import kotlin.math.round
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test

/**
 * The ticks&lt;-&gt;radians conversion in [MechanismArmAdapter] is the only unit-conversion seam in
 * the Arm tab (the plant reports encoder ticks/TPS while the mechanism controller and EKF work in
 * rad / rad·s⁻¹), so it is pinned here.
 */
class MechanismArmAdapterConversionTest {

    companion object {
        private const val TICKS_PER_REV = 28
        private const val GEAR_RATIO = 100.0
        private val TICKS_PER_RAD = TICKS_PER_REV * GEAR_RATIO / (2.0 * Math.PI)
    }

    @Test
    fun ticksToRad_zeroOffset_isTicksOverTicksPerRad() {
        val ticks = round(TICKS_PER_RAD).toInt() // one radian of output shaft
        val rad = MechanismArmAdapter.ticksToRad(ticks, TICKS_PER_REV, GEAR_RATIO, 0.0)
        assertEquals(1.0, rad, 1e-3)
    }

    @Test
    fun ticksToRad_appliesEncoderZeroOffset() {
        val offset = -Math.PI / 4
        val rad = MechanismArmAdapter.ticksToRad(0, TICKS_PER_REV, GEAR_RATIO, offset)
        assertEquals(offset, rad, 1e-12)
    }

    @Test
    fun ticksToRad_halfRevOfOutputShaft() {
        // Half an output-shaft revolution = 28*100/2 ticks = pi radians.
        val ticks = TICKS_PER_REV * GEAR_RATIO.toInt() / 2
        val rad = MechanismArmAdapter.ticksToRad(ticks, TICKS_PER_REV, GEAR_RATIO, 0.0)
        assertEquals(Math.PI, rad, 1e-9)
    }

    @Test
    fun tpsToRadPerSec_roundTripsWithTicksPerRad() {
        val tps = 2.0 * TICKS_PER_RAD // 2 rad/s at the output shaft
        val radPerSec = MechanismArmAdapter.tpsToRadPerSec(tps, TICKS_PER_REV, GEAR_RATIO)
        assertEquals(2.0, radPerSec, 1e-9)
    }

    @Test
    fun tpsToRadPerSec_isOffsetIndependentAndLinear() {
        assertEquals(0.0, MechanismArmAdapter.tpsToRadPerSec(0.0, TICKS_PER_REV, GEAR_RATIO), 1e-12)
        val a = MechanismArmAdapter.tpsToRadPerSec(100.0, TICKS_PER_REV, GEAR_RATIO)
        val b = MechanismArmAdapter.tpsToRadPerSec(200.0, TICKS_PER_REV, GEAR_RATIO)
        assertEquals(2.0, b / a, 1e-9)
    }
}
