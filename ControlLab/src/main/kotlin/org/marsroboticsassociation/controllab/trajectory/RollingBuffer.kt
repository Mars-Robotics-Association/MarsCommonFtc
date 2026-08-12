package org.marsroboticsassociation.controllab.trajectory

/**
 * Fixed-capacity circular buffer that stores multiple double-valued data streams and evicts entries
 * older than [windowSeconds] from the head of the logical queue.
 */
class RollingBuffer(
    private val windowSeconds: Double,
    private val capacity: Int,
    private val numStreams: Int,
) {
    private val times = DoubleArray(capacity)
    private val data = Array(numStreams) { DoubleArray(capacity) }

    private var head = 0
    private var size = 0

    /**
     * Convenience constructor allocating exactly the [NAMED_STREAMS] streams that the named
     * accessors below read.
     */
    constructor(windowSeconds: Double, capacity: Int) : this(windowSeconds, capacity, NAMED_STREAMS)

    fun add(time: Double, vararg values: Double) {
        if (values.size != numStreams) {
            throw IllegalArgumentException(
                String.format("Expected %d values, but got %d", numStreams, values.size)
            )
        }

        // Evict entries outside the time window
        while (size > 0 && (time - times[head]) > windowSeconds) {
            head = (head + 1) % capacity
            size--
        }

        // If at capacity, overwrite the oldest entry
        if (size == capacity) {
            head = (head + 1) % capacity
        } else {
            size++
        }
        val tail = (head + size - 1) % capacity
        times[tail] = time
        for (i in 0 until numStreams) {
            data[i][tail] = values[i]
        }
    }

    fun clear() {
        head = 0
        size = 0
    }

    fun getTimes(): List<Double> = toList(times)

    fun getData(streamIndex: Int): List<Double> {
        if (streamIndex < 0 || streamIndex >= numStreams) {
            throw IndexOutOfBoundsException("Stream index out of range: $streamIndex")
        }
        return toList(data[streamIndex])
    }

    // --- Named accessors for the conventional streams 0..4 ---

    fun getPositions(): List<Double> = getData(0)

    fun getVelocities(): List<Double> = getData(1)

    fun getAccelerations(): List<Double> = getData(2)

    fun getTargets(): List<Double> = getData(3)

    fun getMaxMotorAccels(): List<Double> = getData(4)

    private fun toList(arr: DoubleArray): List<Double> {
        val out = ArrayList<Double>(size)
        for (i in 0 until size) {
            out.add(arr[(head + i) % capacity])
        }
        return out
    }

    companion object {
        /**
         * Streams 0..4 are conventional: position, velocity, acceleration, target, max motor accel.
         * The named accessors read them, so a buffer must allocate at least this many for those
         * accessors to be in range. Callers needing extra streams pass a larger `numStreams`.
         */
        const val NAMED_STREAMS = 5
    }
}
