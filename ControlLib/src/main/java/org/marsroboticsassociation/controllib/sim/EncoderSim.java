package org.marsroboticsassociation.controllib.sim;

/**
 * Ring-buffer encoder sensor model that faithfully simulates the REV Hub encoder.
 *
 * <p>The REV Hub firmware samples the encoder counter every 10 ms into a ring buffer.
 * Velocity is computed as (newest − oldest) / span. With 6 entries the span is
 * 5 × 10 ms = 50 ms, producing readings quantized to multiples of 20 TPS
 * (1 tick / 0.050 s), matching observed telemetry.
 *
 * <p>This model receives the true motor velocity each step and maintains the encoder
 * state independently of the plant simulation.
 */
public class EncoderSim {

    private static final int BUFFER_SIZE = 6;
    private static final double SAMPLE_PERIOD_SEC = 0.010;

    private final int[] buffer = new int[BUFFER_SIZE];
    private int head = 0;
    private int count = 0;

    private double fractionalTicks = 0.0;
    private double timeSinceLastSampleSec = 0.0;

    public EncoderSim() {
    }

    /**
     * Advance the encoder by one time step, writing samples at each 10 ms firmware boundary.
     *
     * @param dt          time step in seconds
     * @param velocityTps true motor velocity in ticks per second
     */
    public void advance(double dt, double velocityTps) {
        double remaining = dt;
        double pos = fractionalTicks;
        double tSince = timeSinceLastSampleSec;

        while (remaining > 1e-12) {
            double timeToNext = SAMPLE_PERIOD_SEC - tSince;
            if (timeToNext <= remaining + 1e-12) {
                pos += velocityTps * timeToNext;
                writeSample(pos);
                tSince = 0.0;
                remaining -= timeToNext;
            } else {
                pos += velocityTps * remaining;
                tSince += remaining;
                remaining = 0;
            }
        }

        fractionalTicks = pos;
        timeSinceLastSampleSec = tSince;
    }

    private void writeSample(double exactPos) {
        head = (head + 1) % BUFFER_SIZE;
        buffer[head] = (int) Math.round(exactPos);
        count++;
    }

    /**
     * Returns the most recent integer tick position (equivalent to {@code getCurrentPosition()}).
     */
    public int getPosition() {
        if (count == 0) return 0;
        return buffer[head];
    }

    /**
     * Returns the velocity in TPS computed from the ring buffer, matching the REV Hub algorithm.
     *
     * <p>Velocity = (newest − oldest) / span, where span = (min(count, 6) − 1) × 0.010 s.
     * Returns 0 before 2 samples exist.
     */
    public double getVelocityTps() {
        int n = Math.min(count, BUFFER_SIZE);
        if (n < 2) return 0.0;

        int oldest = (head - n + 1 + BUFFER_SIZE) % BUFFER_SIZE;
        double spanSec = (n - 1) * SAMPLE_PERIOD_SEC;
        return (buffer[head] - buffer[oldest]) / spanSec;
    }

    public void setState(int position, double fractionalTicks) {
        this.count = position;
        this.fractionalTicks = fractionalTicks;
        // Keep existing buffer history for now
    }

    /** Clears the buffer and resets all state. */
    public void reset() {
        head = 0;
        count = 0;
        fractionalTicks = 0.0;
        timeSinceLastSampleSec = 0.0;
        for (int i = 0; i < BUFFER_SIZE; i++) buffer[i] = 0;
    }
}
