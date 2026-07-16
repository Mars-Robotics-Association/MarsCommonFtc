package org.marsroboticsassociation.controllab.trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Fixed-capacity circular buffer that stores multiple double-valued data streams
 * and evicts entries older than {@code windowSeconds} from the head of the logical queue.
 */
public class RollingBuffer {

    /**
     * Streams 0..4 are conventional: position, velocity, acceleration, target, max motor accel.
     * The named accessors read them, so a buffer must allocate at least this many for those
     * accessors to be in range. Callers needing extra streams pass a larger {@code numStreams}.
     */
    public static final int NAMED_STREAMS = 5;

    private final double windowSeconds;
    private final int capacity;
    private final int numStreams;

    private final double[] times;
    private final double[][] data;

    private int head = 0;
    private int size = 0;

    public RollingBuffer(double windowSeconds, int capacity, int numStreams) {
        this.windowSeconds = windowSeconds;
        this.capacity = capacity;
        this.numStreams = numStreams;
        this.times = new double[capacity];
        this.data = new double[numStreams][capacity];
    }

    /**
     * Convenience constructor allocating exactly the {@value #NAMED_STREAMS} streams that the named
     * accessors below read.
     */
    public RollingBuffer(double windowSeconds, int capacity) {
        this(windowSeconds, capacity, NAMED_STREAMS);
    }

    public void add(double time, double... values) {
        if (values.length != numStreams) {
            throw new IllegalArgumentException(
                    String.format("Expected %d values, but got %d", numStreams, values.length));
        }

        // Evict entries outside the time window
        while (size > 0 && (time - times[head]) > windowSeconds) {
            head = (head + 1) % capacity;
            size--;
        }

        // If at capacity, overwrite the oldest entry
        if (size == capacity) {
            head = (head + 1) % capacity;
        } else {
            size++;
        }
        int tail = (head + size - 1) % capacity;
        times[tail] = time;
        for (int i = 0; i < numStreams; i++) {
            data[i][tail] = values[i];
        }
    }

    public void clear() {
        head = 0;
        size = 0;
    }

    public List<Double> getTimes() {
        return toList(times);
    }

    public List<Double> getData(int streamIndex) {
        if (streamIndex < 0 || streamIndex >= numStreams) {
            throw new IndexOutOfBoundsException("Stream index out of range: " + streamIndex);
        }
        return toList(data[streamIndex]);
    }

    // --- Named accessors for the conventional streams 0..4 ---

    public List<Double> getPositions() { return getData(0); }
    public List<Double> getVelocities() { return getData(1); }
    public List<Double> getAccelerations() { return getData(2); }
    public List<Double> getTargets() { return getData(3); }
    public List<Double> getMaxMotorAccels() { return getData(4); }

    private List<Double> toList(double[] arr) {
        List<Double> out = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            out.add(arr[(head + i) % capacity]);
        }
        return out;
    }
}
