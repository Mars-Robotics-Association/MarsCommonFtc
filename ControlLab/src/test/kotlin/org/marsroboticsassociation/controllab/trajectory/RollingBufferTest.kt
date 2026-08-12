package org.marsroboticsassociation.controllab.trajectory

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.assertFalse
import org.junit.jupiter.api.Assertions.assertThrows
import org.junit.jupiter.api.Assertions.assertTrue
import org.junit.jupiter.api.Test

class RollingBufferTest {

    @Test
    fun add_singlePoint_appearsInAllSeries() {
        val buf = RollingBuffer(10.0, 50)
        buf.add(0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
        assertEquals(listOf(0.0), buf.getTimes())
        assertEquals(listOf(1.0), buf.getPositions())
        assertEquals(listOf(2.0), buf.getVelocities())
        assertEquals(listOf(3.0), buf.getAccelerations())
        assertEquals(listOf(4.0), buf.getTargets())
        assertEquals(listOf(5.0), buf.getMaxMotorAccels())
    }

    @Test
    fun defaultConstructor_allocatesEveryNamedStream() {
        val buf = RollingBuffer(10.0, 50)
        buf.add(0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
        // Every named accessor is in range for a buffer from the convenience constructor.
        assertEquals(listOf(5.0), buf.getMaxMotorAccels())
    }

    @Test
    fun add_wrongValueCount_isRejected() {
        val buf = RollingBuffer(10.0, 50)
        assertThrows(IllegalArgumentException::class.java) { buf.add(0.0, 1.0, 2.0) }
    }

    @Test
    fun add_evictsPointsOutsideWindow() {
        val buf = RollingBuffer(1.0, 50) // 1-second window
        buf.add(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        buf.add(0.5, 1.0, 0.0, 0.0, 1.0, 0.0)
        buf.add(1.1, 2.0, 0.0, 0.0, 2.0, 0.0) // t=0.0 is >1s before t=1.1, so it is evicted
        val times = buf.getTimes()
        assertFalse(times.contains(0.0), "t=0.0 should have been evicted")
        assertTrue(times.contains(0.5))
        assertTrue(times.contains(1.1))
    }

    @Test
    fun add_capacityExceeded_evictsOldest() {
        val buf = RollingBuffer(100.0, 3) // capacity 3
        buf.add(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        buf.add(1.0, 1.0, 0.0, 0.0, 1.0, 0.0)
        buf.add(2.0, 2.0, 0.0, 0.0, 2.0, 0.0)
        buf.add(3.0, 3.0, 0.0, 0.0, 3.0, 0.0) // evicts t=0.0
        assertEquals(3, buf.getTimes().size)
        assertFalse(buf.getTimes().contains(0.0))
        assertTrue(buf.getTimes().contains(3.0))
        assertEquals(listOf(1.0, 2.0, 3.0), buf.getTargets())
    }

    @Test
    fun clear_removesAllPoints() {
        val buf = RollingBuffer(10.0, 50)
        buf.add(0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
        buf.clear()
        assertTrue(buf.getTimes().isEmpty())
        assertTrue(buf.getTargets().isEmpty())
    }
}
