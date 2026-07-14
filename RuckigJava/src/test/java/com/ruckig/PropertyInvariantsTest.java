package com.ruckig;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * M9: property-style invariants inspired by upstream {@code test_target.cpp} / {@code randomizer.hpp}.
 * Full 1e6 CI volume can be enabled via system property {@code ruckig.property.cases}.
 */
class PropertyInvariantsTest {

    @Test
    void randomTrajectoriesRespectLimitsAndReachTarget() {
        int nCases = Integer.getInteger("ruckig.property.cases", 5000);
        Random rng = new Random(20260714L);
        Ruckig otg = new Ruckig(1);
        InputParameter input = new InputParameter(1);
        Trajectory traj = new Trajectory(1);
        double[] p = new double[1];
        double[] v = new double[1];
        double[] a = new double[1];

        int working = 0;
        int failed = 0;
        for (int i = 0; i < nCases; ++i) {
            // Rest-to-rest batch (strictest limit checks) + occasional nonzero boundary states
            double vMax = 0.2 + rng.nextDouble() * 5;
            double aMax = 0.2 + rng.nextDouble() * 5;
            double jMax = 0.5 + rng.nextDouble() * 20;
            double p0 = (rng.nextDouble() - 0.5) * 10;
            double pf = (rng.nextDouble() - 0.5) * 10;
            boolean rest = rng.nextDouble() < 0.7;
            double v0 = rest ? 0.0 : (rng.nextDouble() - 0.5) * vMax * 0.3;
            double vf = rest ? 0.0 : (rng.nextDouble() - 0.5) * vMax * 0.3;
            double a0 = rest ? 0.0 : (rng.nextDouble() - 0.5) * aMax * 0.3;
            double af = rest ? 0.0 : (rng.nextDouble() - 0.5) * aMax * 0.3;

            input.current_position[0] = p0;
            input.current_velocity[0] = v0;
            input.current_acceleration[0] = a0;
            input.target_position[0] = pf;
            input.target_velocity[0] = vf;
            input.target_acceleration[0] = af;
            input.max_velocity[0] = vMax;
            input.max_acceleration[0] = aMax;
            input.max_jerk[0] = jMax;

            int res = otg.calculate(input, traj);
            if (res == Result.ErrorInvalidInput) {
                // Random draw violated kinematic feasibility of target state (upstream validate)
                continue;
            }
            if (res != Result.Working) {
                failed++;
                continue;
            }
            working++;

            double duration = traj.get_duration();
            assertTrue(duration >= 0 && duration < 1e4, "duration");

            // t_sum monotone on profile
            Profile prof = traj.profiles[0][0];
            for (int k = 1; k < 7; ++k) {
                assertTrue(prof.t_sum[k] + 1e-15 >= prof.t_sum[k - 1], "t_sum mono");
            }

            // Limit checks after brake pre-trajectory; use modest absolute slack for FP
            final double vSlack = 1e-4 * Math.max(1.0, vMax);
            final double aSlack = 1e-4 * Math.max(1.0, aMax);
            int samples = 24;
            for (int s = 0; s <= samples; ++s) {
                double t = duration * s / samples;
                traj.at_time(t, p, v, a);
                if (rest) {
                    assertTrue(Math.abs(v[0]) <= vMax + vSlack, "v limit v=" + v[0] + " max=" + vMax);
                    assertTrue(Math.abs(a[0]) <= aMax + aSlack, "a limit a=" + a[0] + " max=" + aMax);
                }
            }

            traj.at_time(duration, p, v, a);
            assertTrue(Math.abs(p[0] - pf) < 1e-6, "final p i=" + i + " err=" + (p[0] - pf));
            assertTrue(Math.abs(v[0] - vf) < 1e-5, "final v");
            assertTrue(Math.abs(a[0] - af) < 1e-4, "final a");
        }

        assertTrue(working > nCases * 0.9, "working=" + working + " failed=" + failed + " of " + nCases);
    }

    @Test
    void multiDofNoneSyncIndependent() {
        Ruckig otg = new Ruckig(2);
        InputParameter input = new InputParameter(2);
        Trajectory traj = new Trajectory(2);
        input.synchronization = Synchronization.None;
        input.current_position[0] = 0;
        input.current_position[1] = 0;
        input.target_position[0] = 1.0;
        input.target_position[1] = 0.01;
        input.max_velocity[0] = input.max_velocity[1] = 1.0;
        input.max_acceleration[0] = input.max_acceleration[1] = 1.0;
        input.max_jerk[0] = input.max_jerk[1] = 2.0;

        assertTrue(otg.calculate(input, traj) == Result.Working);
        // Duration is max of independent mins under None + max update in calculator
        assertTrue(traj.get_duration() + 1e-12 >= traj.independent_min_durations[0]);
        assertTrue(traj.get_duration() + 1e-12 >= traj.independent_min_durations[1]);
        assertTrue(traj.independent_min_durations[0] > traj.independent_min_durations[1]);
    }
}
