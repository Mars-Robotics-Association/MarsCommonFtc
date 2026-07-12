package org.marsroboticsassociation.controllab.arm;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * The ticks&lt;-&gt;radians conversion in {@link MechanismArmAdapter} is the only unit-conversion seam
 * in the Arm tab (the plant reports encoder ticks/TPS while the mechanism controller and EKF work in
 * rad / rad·s⁻¹), so it is pinned here.
 */
class MechanismArmAdapterConversionTest {

    private static final int TICKS_PER_REV = 28;
    private static final double GEAR_RATIO = 100.0;
    private static final double TICKS_PER_RAD = TICKS_PER_REV * GEAR_RATIO / (2.0 * Math.PI);

    @Test
    void ticksToRad_zeroOffset_isTicksOverTicksPerRad() {
        int ticks = (int) Math.round(TICKS_PER_RAD); // one radian of output shaft
        double rad = MechanismArmAdapter.ticksToRad(ticks, TICKS_PER_REV, GEAR_RATIO, 0.0);
        assertEquals(1.0, rad, 1e-3);
    }

    @Test
    void ticksToRad_appliesEncoderZeroOffset() {
        double offset = -Math.PI / 4;
        double rad = MechanismArmAdapter.ticksToRad(0, TICKS_PER_REV, GEAR_RATIO, offset);
        assertEquals(offset, rad, 1e-12);
    }

    @Test
    void ticksToRad_halfRevOfOutputShaft() {
        // Half an output-shaft revolution = 28*100/2 ticks = pi radians.
        int ticks = TICKS_PER_REV * (int) GEAR_RATIO / 2;
        double rad = MechanismArmAdapter.ticksToRad(ticks, TICKS_PER_REV, GEAR_RATIO, 0.0);
        assertEquals(Math.PI, rad, 1e-9);
    }

    @Test
    void tpsToRadPerSec_roundTripsWithTicksPerRad() {
        double tps = 2.0 * TICKS_PER_RAD; // 2 rad/s at the output shaft
        double radPerSec = MechanismArmAdapter.tpsToRadPerSec(tps, TICKS_PER_REV, GEAR_RATIO);
        assertEquals(2.0, radPerSec, 1e-9);
    }

    @Test
    void tpsToRadPerSec_isOffsetIndependentAndLinear() {
        assertEquals(0.0, MechanismArmAdapter.tpsToRadPerSec(0.0, TICKS_PER_REV, GEAR_RATIO), 1e-12);
        double a = MechanismArmAdapter.tpsToRadPerSec(100.0, TICKS_PER_REV, GEAR_RATIO);
        double b = MechanismArmAdapter.tpsToRadPerSec(200.0, TICKS_PER_REV, GEAR_RATIO);
        assertEquals(2.0, b / a, 1e-9);
    }
}
