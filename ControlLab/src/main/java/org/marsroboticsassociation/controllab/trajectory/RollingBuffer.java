package org.marsroboticsassociation.controllab.trajectory;

import java.util.ArrayList;
import java.util.List;

/**
 * Fixed-capacity circular buffer that evicts entries older than {@code windowSeconds}
 * from the head of the logical queue.
 */
public class RollingBuffer {

    private final double windowSeconds;
    private final int capacity;

    private final double[] times;
    private final double[] positions;
    private final double[] velocities;
    private final double[] accelerations;

    private int head = 0;
    private int size = 0;

    public RollingBuffer(double windowSeconds, int capacity) {
        this.windowSeconds = windowSeconds;
        this.capacity = capacity;
        this.times         = new double[capacity];
        this.positions     = new double[capacity];
        this.velocities    = new double[capacity];
        this.accelerations = new double[capacity];
    }

    public void add(double time, double position, double velocity, double acceleration) {
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
        times[tail]         = time;
        positions[tail]     = position;
        velocities[tail]    = velocity;
        accelerations[tail] = acceleration;
    }

    public void clear() {
        head = 0;
        size = 0;
    }

    public List<Double> getTimes()         { return toList(times); }
    public List<Double> getPositions()     { return toList(positions); }
    public List<Double> getVelocities()    { return toList(velocities); }
    public List<Double> getAccelerations() { return toList(accelerations); }

    private List<Double> toList(double[] arr) {
        List<Double> out = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            out.add(arr[(head + i) % capacity]);
        }
        return out;
    }
}
