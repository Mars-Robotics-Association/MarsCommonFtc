package com.ruckig;

import org.junit.jupiter.api.Test;

import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * M0 gate: harness loads a hand-written sample vector; dummy comparison fails correctly.
 */
class GoldenHarnessTest {

    @Test
    void loadsSampleGoldenVector() throws Exception {
        Object root = GoldenVectorLoader.loadResource("/golden/sample_m0.json");
        Map<String, Object> obj = GoldenVectorLoader.asObject(root);
        assertEquals("0.17.3", GoldenVectorLoader.asString(obj.get("version")));
        assertEquals("m0_sample", GoldenVectorLoader.asString(obj.get("suite")));

        List<Object> cases = GoldenVectorLoader.asArray(obj.get("cases"));
        assertEquals(1, cases.size());

        Map<String, Object> c0 = GoldenVectorLoader.asObject(cases.get(0));
        assertEquals("rest_to_rest_1dof", GoldenVectorLoader.asString(c0.get("id")));
        assertEquals(1, GoldenVectorLoader.asInt(c0.get("dofs")));
        assertEquals(0, GoldenVectorLoader.asInt(c0.get("result")));
        assertEquals(2.0, GoldenVectorLoader.asDouble(c0.get("duration")), 0.0);

        Map<String, Object> input = GoldenVectorLoader.asObject(c0.get("input"));
        double[] target = GoldenVectorLoader.asDoubleArray(input.get("target_position"));
        assertEquals(1.0, target[0], 0.0);

        List<Object> samples = GoldenVectorLoader.asArray(c0.get("samples"));
        assertEquals(3, samples.size());
    }

    @Test
    void dummyComparisonFailsWhenValuesDiffer() {
        // Gate: a deliberately wrong expected value is rejected by the comparison helper.
        AssertionError err = assertThrows(AssertionError.class, () -> {
            GoldenCompare.assertClose("duration", 1.0, 1.5, 1e-12, 0.0);
        });
        assertNotNull(err.getMessage());
        assertTrue(err.getMessage().contains("duration"));
    }

    @Test
    void dummyComparisonPassesWhenValuesMatch() {
        GoldenCompare.assertClose("duration", 1.5, 1.5, 1e-12, 0.0);
        GoldenCompare.assertClose("sample_p", 1.0, 1.0 + 1e-13, 1e-12, 1e-10);
    }
}
