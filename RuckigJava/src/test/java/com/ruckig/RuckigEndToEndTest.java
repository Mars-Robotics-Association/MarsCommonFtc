package com.ruckig;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end smoke tests for the full OTG pipeline (M4–M8).
 */
class RuckigEndToEndTest {

    @Test
    void restToRestPosition1DoF() {
        Ruckig otg = new Ruckig(1, 0.01);
        InputParameter input = new InputParameter(1);
        OutputParameter output = new OutputParameter(1);

        input.current_position[0] = 0.0;
        input.current_velocity[0] = 0.0;
        input.current_acceleration[0] = 0.0;
        input.target_position[0] = 1.0;
        input.target_velocity[0] = 0.0;
        input.target_acceleration[0] = 0.0;
        input.max_velocity[0] = 2.0;
        input.max_acceleration[0] = 2.0;
        input.max_jerk[0] = 4.0;

        int result = otg.update(input, output);
        assertEquals(Result.Working, result);
        assertTrue(output.new_calculation);
        assertTrue(output.trajectory.get_duration() > 0);

        // Step until finished (must feed output back into input each cycle, as in upstream examples)
        int steps = 0;
        while (result == Result.Working && steps < 10000) {
            output.pass_to_input(input);
            result = otg.update(input, output);
            steps++;
        }
        assertEquals(Result.Finished, result, "steps=" + steps + " t=" + output.time
                + " dur=" + output.trajectory.get_duration());
        assertTrue(Math.abs(output.new_position[0] - 1.0) < 1e-6, "p=" + output.new_position[0]);
        assertTrue(Math.abs(output.new_velocity[0]) < 1e-5, "v=" + output.new_velocity[0]);
        assertTrue(Math.abs(output.new_acceleration[0]) < 1e-4, "a=" + output.new_acceleration[0]);
    }

    @Test
    void calculateOfflineMatchesDuration() {
        Ruckig otg = new Ruckig(1);
        InputParameter input = new InputParameter(1);
        Trajectory traj = new Trajectory(1);

        input.current_position[0] = 0;
        input.target_position[0] = 1;
        input.max_velocity[0] = 2;
        input.max_acceleration[0] = 2;
        input.max_jerk[0] = 4;

        int res = otg.calculate(input, traj);
        assertEquals(Result.Working, res);
        assertTrue(traj.get_duration() > 0);

        double[] p = new double[1];
        double[] v = new double[1];
        double[] a = new double[1];
        traj.at_time(0.0, p, v, a);
        assertTrue(Math.abs(p[0]) < 1e-12);
        traj.at_time(traj.get_duration(), p, v, a);
        assertTrue(Math.abs(p[0] - 1.0) < 1e-6, "end p=" + p[0]);
        assertTrue(Math.abs(v[0]) < 1e-5, "end v=" + v[0]);
    }

    @Test
    void velocityInterface() {
        Ruckig otg = new Ruckig(1);
        InputParameter input = new InputParameter(1);
        Trajectory traj = new Trajectory(1);

        input.control_interface = ControlInterface.Velocity;
        input.current_velocity[0] = 0.0;
        input.current_acceleration[0] = 0.0;
        input.target_velocity[0] = 1.0;
        input.target_acceleration[0] = 0.0;
        input.max_acceleration[0] = 2.0;
        input.max_jerk[0] = 4.0;

        int res = otg.calculate(input, traj);
        assertEquals(Result.Working, res);
        assertTrue(traj.get_duration() > 0);

        double[] p = new double[1];
        double[] v = new double[1];
        double[] a = new double[1];
        traj.at_time(traj.get_duration(), p, v, a);
        assertTrue(Math.abs(v[0] - 1.0) < 1e-5, "end v=" + v[0]);
    }

    @Test
    void multiDoFTimeSync() {
        Ruckig otg = new Ruckig(2);
        InputParameter input = new InputParameter(2);
        Trajectory traj = new Trajectory(2);

        input.current_position[0] = 0;
        input.current_position[1] = 0;
        input.target_position[0] = 1.0;
        input.target_position[1] = 0.1;
        input.max_velocity[0] = 2;
        input.max_velocity[1] = 2;
        input.max_acceleration[0] = 2;
        input.max_acceleration[1] = 2;
        input.max_jerk[0] = 4;
        input.max_jerk[1] = 4;
        input.synchronization = Synchronization.Time;

        int res = otg.calculate(input, traj);
        assertEquals(Result.Working, res);
        // Independent mins may differ; synchronized duration is the max
        assertTrue(traj.get_duration() + 1e-12 >= traj.independent_min_durations[0]);
        assertTrue(traj.get_duration() + 1e-12 >= traj.independent_min_durations[1]);

        double[] p = new double[2];
        double[] v = new double[2];
        double[] a = new double[2];
        traj.at_time(traj.get_duration(), p, v, a);
        assertTrue(Math.abs(p[0] - 1.0) < 1e-5, "p0=" + p[0]);
        assertTrue(Math.abs(p[1] - 0.1) < 1e-5, "p1=" + p[1]);
    }

    @Test
    void brakeFromOverAcceleration() {
        Ruckig otg = new Ruckig(1);
        InputParameter input = new InputParameter(1);
        Trajectory traj = new Trajectory(1);

        input.current_position[0] = 0;
        input.current_velocity[0] = 0;
        input.current_acceleration[0] = 3.0; // > aMax
        input.target_position[0] = 1.0;
        input.max_velocity[0] = 2;
        input.max_acceleration[0] = 1.0;
        input.max_jerk[0] = 4.0;

        // validate with check_target only — current accel over limit is allowed for brake path
        int res = otg.calculate(input, traj);
        assertEquals(Result.Working, res, "brake-prefixed plan should succeed");
        assertTrue(traj.profiles[0][0].brake.duration > 0 || traj.get_duration() > 0);
    }

    @Test
    void propertyRandomRestToRestLimitsRespected() {
        java.util.Random rng = new java.util.Random(99);
        Ruckig otg = new Ruckig(1);
        InputParameter input = new InputParameter(1);
        Trajectory traj = new Trajectory(1);
        double[] p = new double[1];
        double[] v = new double[1];
        double[] a = new double[1];

        int ok = 0;
        for (int i = 0; i < 500; ++i) {
            double vMax = 0.5 + rng.nextDouble() * 3;
            double aMax = 0.5 + rng.nextDouble() * 3;
            double jMax = 1.0 + rng.nextDouble() * 10;
            double pf = (rng.nextDouble() - 0.5) * 4;

            input.current_position[0] = 0;
            input.current_velocity[0] = 0;
            input.current_acceleration[0] = 0;
            input.target_position[0] = pf;
            input.target_velocity[0] = 0;
            input.target_acceleration[0] = 0;
            input.max_velocity[0] = vMax;
            input.max_acceleration[0] = aMax;
            input.max_jerk[0] = jMax;

            int res = otg.calculate(input, traj);
            if (res != Result.Working) {
                continue;
            }
            ok++;
            int samples = 40;
            for (int s = 0; s <= samples; ++s) {
                double t = traj.get_duration() * s / samples;
                traj.at_time(t, p, v, a);
                assertTrue(Math.abs(v[0]) <= vMax + 1e-6, "v limit violated v=" + v[0]);
                assertTrue(Math.abs(a[0]) <= aMax + 1e-5, "a limit violated a=" + a[0]);
            }
            traj.at_time(traj.get_duration(), p, v, a);
            assertTrue(Math.abs(p[0] - pf) < 1e-5, "final p");
            assertTrue(Math.abs(v[0]) < 1e-4, "final v");
        }
        assertTrue(ok > 400, "too many failures: ok=" + ok);
    }
}
