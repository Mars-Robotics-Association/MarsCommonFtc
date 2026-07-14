package com.ruckig;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * M3 gate: brake when state is beyond limits.
 */
class BrakeTest {

    @Test
    void accelerationBeyondLimitProducesBrake() {
        BrakeProfile b = new BrakeProfile();
        b.get_position_brake_trajectory(
                /*v0*/ 0.0, /*a0*/ 3.0,
                /*vMax*/ 2.0, /*vMin*/ -2.0,
                /*aMax*/ 1.0, /*aMin*/ -1.0,
                /*jMax*/ 4.0);
        assertTrue(b.t[0] > 0.0, "expected brake phase for a0 > aMax");
        assertTrue(b.j[0] != 0.0);
    }

    @Test
    void velocityBeyondLimitProducesBrake() {
        BrakeProfile b = new BrakeProfile();
        b.get_position_brake_trajectory(
                /*v0*/ 3.0, /*a0*/ 0.0,
                /*vMax*/ 1.0, /*vMin*/ -1.0,
                /*aMax*/ 2.0, /*aMin*/ -2.0,
                /*jMax*/ 5.0);
        assertTrue(b.t[0] > 0.0, "expected brake for v0 > vMax");
    }

    @Test
    void withinLimitsNoBrake() {
        BrakeProfile b = new BrakeProfile();
        b.get_position_brake_trajectory(0.0, 0.0, 2.0, -2.0, 1.0, -1.0, 4.0);
        assertTrue(b.t[0] == 0.0 && b.t[1] == 0.0);
    }

    @Test
    void finalizeIntegratesState() {
        BrakeProfile b = new BrakeProfile();
        b.get_velocity_brake_trajectory(/*a0*/ 2.0, /*aMax*/ 1.0, /*aMin*/ -1.0, /*jMax*/ 4.0);
        double[] state = new double[] {0.0, 0.0, 2.0};
        b.finalize(state);
        assertTrue(b.duration > 0);
        // After braking accel should be near aMax
        assertTrue(Math.abs(state[2] - 1.0) < 1e-9, "a after brake=" + state[2]);
    }

    @Test
    void copyFromIndependent() {
        BrakeProfile a = new BrakeProfile();
        a.duration = 1.0;
        a.t[0] = 0.5;
        BrakeProfile b = new BrakeProfile();
        b.copyFrom(a);
        a.t[0] = 0;
        assertTrue(b.t[0] == 0.5);
        assertTrue(b.duration == 1.0);
    }
}
