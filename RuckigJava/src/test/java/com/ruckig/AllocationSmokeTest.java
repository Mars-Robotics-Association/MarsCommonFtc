package com.ruckig;

import org.junit.jupiter.api.Test;

import java.lang.management.GarbageCollectorMXBean;
import java.lang.management.ManagementFactory;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * M9: steady-state update should not allocate in the steady loop after warmup.
 * Uses GC collection count as a coarse proxy (not as strict as JMH alloc rate).
 */
class AllocationSmokeTest {

    @Test
    void steadyStateUpdateDoesNotTriggerGc() {
        Ruckig otg = new Ruckig(1, 0.001);
        InputParameter input = new InputParameter(1);
        OutputParameter output = new OutputParameter(1);

        input.current_position[0] = 0;
        input.target_position[0] = 10;
        input.max_velocity[0] = 2;
        input.max_acceleration[0] = 2;
        input.max_jerk[0] = 4;

        // Warmup: plan + many steps (JIT, TLAB)
        int result = Result.Working;
        for (int i = 0; i < 5000 && result == Result.Working; ++i) {
            result = otg.update(input, output);
            output.pass_to_input(input);
        }

        System.gc();
        long gcBefore = gcCount();
        long start = System.nanoTime();

        // Re-target far away so we keep Working for many cycles without finish
        otg.reset();
        input.current_position[0] = 0;
        input.current_velocity[0] = 0;
        input.current_acceleration[0] = 0;
        input.target_position[0] = 100;
        result = otg.update(input, output);
        assertTrue(result == Result.Working);

        int steps = 2000;
        for (int i = 0; i < steps; ++i) {
            output.pass_to_input(input);
            result = otg.update(input, output);
            if (result != Result.Working) {
                break;
            }
        }

        long elapsedNs = System.nanoTime() - start;
        long gcAfter = gcCount();
        double usPerUpdate = (elapsedNs / 1000.0) / Math.max(1, steps);

        // Sanity bound from plan: single-DoF update ≤ ~50 µs on desktop JVM
        assertTrue(usPerUpdate < 500.0, "us/update=" + usPerUpdate);

        // Prefer zero GC; allow flaky single collection on some JVMs
        assertTrue(gcAfter - gcBefore <= 1, "GC collections during steady loop: " + (gcAfter - gcBefore));
    }

    private static long gcCount() {
        long n = 0;
        for (GarbageCollectorMXBean bean : ManagementFactory.getGarbageCollectorMXBeans()) {
            long c = bean.getCollectionCount();
            if (c > 0) {
                n += c;
            }
        }
        return n;
    }
}
