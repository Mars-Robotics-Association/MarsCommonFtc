package org.marsroboticsassociation.controllab.trajectory;

import org.junit.jupiter.api.Test;
import java.util.List;
import static org.junit.jupiter.api.Assertions.*;

class RollingBufferTest {

    @Test
    void add_singlePoint_appearsInAllSeries() {
        RollingBuffer buf = new RollingBuffer(10.0, 50);
        buf.add(0.0, 1.0, 2.0, 3.0, 4.0);
        assertEquals(List.of(0.0), buf.getTimes());
        assertEquals(List.of(1.0), buf.getPositions());
        assertEquals(List.of(2.0), buf.getVelocities());
        assertEquals(List.of(3.0), buf.getAccelerations());
        assertEquals(List.of(4.0), buf.getTargets());
    }

    @Test
    void add_evictsPointsOutsideWindow() {
        RollingBuffer buf = new RollingBuffer(1.0, 50); // 1-second window
        buf.add(0.0, 0.0, 0.0, 0.0, 0.0);
        buf.add(0.5, 1.0, 0.0, 0.0, 1.0);
        buf.add(1.1, 2.0, 0.0, 0.0, 2.0); // t=0.0 is now >1s before t=1.1, should be evicted
        List<Double> times = buf.getTimes();
        assertFalse(times.contains(0.0), "t=0.0 should have been evicted");
        assertTrue(times.contains(0.5));
        assertTrue(times.contains(1.1));
    }

    @Test
    void add_capacityExceeded_evictsOldest() {
        RollingBuffer buf = new RollingBuffer(100.0, 3); // capacity 3
        buf.add(0.0, 0.0, 0.0, 0.0, 0.0);
        buf.add(1.0, 1.0, 0.0, 0.0, 1.0);
        buf.add(2.0, 2.0, 0.0, 0.0, 2.0);
        buf.add(3.0, 3.0, 0.0, 0.0, 3.0); // evicts t=0.0
        assertEquals(3, buf.getTimes().size());
        assertFalse(buf.getTimes().contains(0.0));
        assertTrue(buf.getTimes().contains(3.0));
        assertEquals(List.of(1.0, 2.0, 3.0), buf.getTargets());
    }

    @Test
    void clear_removesAllPoints() {
        RollingBuffer buf = new RollingBuffer(10.0, 50);
        buf.add(0.0, 1.0, 2.0, 3.0, 4.0);
        buf.clear();
        assertTrue(buf.getTimes().isEmpty());
        assertTrue(buf.getTargets().isEmpty());
    }
}
