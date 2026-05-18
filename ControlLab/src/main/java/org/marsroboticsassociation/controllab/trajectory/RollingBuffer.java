package org.marsroboticsassociation.controllab.trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Fixed-capacity circular buffer that stores multiple double-valued data streams
 * and evicts entries older than {@code windowSeconds} from the head of the logical queue.
 */
public class RollingBuffer {

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
     * Legacy constructor for 4 data streams (position, velocity, acceleration, target).
     */
    public RollingBuffer(double windowSeconds, int capacity) {
        this(windowSeconds, capacity, 4);
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

    // --- Legacy Accessors ---

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
