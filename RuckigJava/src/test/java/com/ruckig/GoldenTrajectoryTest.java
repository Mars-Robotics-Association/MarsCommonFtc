package com.ruckig;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Differential checks against golden JSON generated from the C++ reference
 * (upstream v0.17.3, see tools/generate_golden.py) — plan §5.
 */
class GoldenTrajectoryTest {

    private static final double DUR_REL = 1e-12;
    private static final double KIN_ABS = 1e-9;
    private static final double KIN_REL = 1e-10;

    @Test
    void sampleM0MatchesJavaPort() throws Exception {
        runSuite("/golden/sample_m0.json");
    }

    @Test
    void differentialV1MatchesCppReference() throws Exception {
        runSuite("/golden/differential_v1.json");
    }

    private static void runSuite(String resource) throws Exception {
        Map<String, Object> root = GoldenVectorLoader.asObject(
                GoldenVectorLoader.loadResource(resource));
        List<Object> cases = GoldenVectorLoader.asArray(root.get("cases"));
        assertTrue(cases.size() >= 1, "empty suite " + resource);
        // Accumulate failures so one bad case cannot mask the rest of the suite.
        StringBuilder failures = new StringBuilder();
        int failed = 0;
        for (Object cObj : cases) {
            Map<String, Object> c = GoldenVectorLoader.asObject(cObj);
            try {
                runCase(c);
            } catch (AssertionError e) {
                failed++;
                if (failed <= 25) {
                    failures.append("\n  ").append(e.getMessage());
                }
            }
        }
        if (failed > 0) {
            throw new AssertionError(
                    failed + "/" + cases.size() + " golden cases failed in " + resource
                            + " (first " + Math.min(failed, 25) + " shown):" + failures);
        }
    }

    private static void runCase(Map<String, Object> c) {
        int dofs = GoldenVectorLoader.asInt(c.get("dofs"));
        Map<String, Object> inpMap = GoldenVectorLoader.asObject(c.get("input"));

        Ruckig otg = new Ruckig(dofs);
        InputParameter input = new InputParameter(dofs);
        Trajectory traj = new Trajectory(dofs);

        fill(input.current_position, inpMap.get("current_position"));
        fill(input.current_velocity, inpMap.get("current_velocity"));
        fill(input.current_acceleration, inpMap.get("current_acceleration"));
        fill(input.target_position, inpMap.get("target_position"));
        fill(input.target_velocity, inpMap.get("target_velocity"));
        fill(input.target_acceleration, inpMap.get("target_acceleration"));
        fill(input.max_velocity, inpMap.get("max_velocity"));
        fill(input.max_acceleration, inpMap.get("max_acceleration"));
        fill(input.max_jerk, inpMap.get("max_jerk"));

        if (inpMap.containsKey("min_velocity")) {
            input.min_velocity = GoldenVectorLoader.asDoubleArray(inpMap.get("min_velocity"));
        }
        if (inpMap.containsKey("min_acceleration")) {
            input.min_acceleration = GoldenVectorLoader.asDoubleArray(inpMap.get("min_acceleration"));
        }
        if (inpMap.containsKey("control_interface")) {
            input.control_interface = ControlInterface.valueOf(
                    GoldenVectorLoader.asString(inpMap.get("control_interface")));
        }
        if (inpMap.containsKey("synchronization")) {
            input.synchronization = Synchronization.valueOf(
                    GoldenVectorLoader.asString(inpMap.get("synchronization")));
        }
        if (inpMap.containsKey("minimum_duration")) {
            input.setMinimumDuration(GoldenVectorLoader.asDouble(inpMap.get("minimum_duration")));
        }

        int result = otg.calculate(input, traj);
        assertEquals(GoldenVectorLoader.asInt(c.get("result")), result, "result " + c.get("id"));
        if (result < 0) {
            return; // error case: result code is the whole golden
        }

        double expectedDuration = GoldenVectorLoader.asDouble(c.get("duration"));
        GoldenCompare.assertClose("duration " + c.get("id"), expectedDuration, traj.get_duration(), DUR_REL, 0.0);

        List<Object> samples = GoldenVectorLoader.asArray(c.get("samples"));
        double[] p = new double[dofs];
        double[] v = new double[dofs];
        double[] a = new double[dofs];
        for (Object sObj : samples) {
            Map<String, Object> s = GoldenVectorLoader.asObject(sObj);
            double t = GoldenVectorLoader.asDouble(s.get("t"));
            traj.at_time(t, p, v, a);
            String at = c.get("id") + " @t=" + t;
            GoldenCompare.assertCloseArray("p " + at, GoldenVectorLoader.asDoubleArray(s.get("p")), p, KIN_REL, KIN_ABS);
            GoldenCompare.assertCloseArray("v " + at, GoldenVectorLoader.asDoubleArray(s.get("v")), v, KIN_REL, KIN_ABS);
            GoldenCompare.assertCloseArray("a " + at, GoldenVectorLoader.asDoubleArray(s.get("a")), a, KIN_REL, KIN_ABS);
        }
        assertTrue(samples.size() >= 1);
    }

    private static void fill(double[] dst, Object arrObj) {
        double[] src = GoldenVectorLoader.asDoubleArray(arrObj);
        System.arraycopy(src, 0, dst, 0, dst.length);
    }
}
