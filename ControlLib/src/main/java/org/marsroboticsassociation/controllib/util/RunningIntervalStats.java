package org.marsroboticsassociation.controllib.util;

import java.util.ArrayDeque;
import java.util.Deque;

public class RunningIntervalStats {

    private static final class Sample {
        double timestamp;   // seconds
        double intervalMs;  // milliseconds
    }

    /**
     * Max number of recycled Sample objects kept
     */
    private static final int MAX_POOL_SIZE = 20_000;

    private double windowSeconds;

    private double startTime;
    private double nowSeconds;
    private boolean hasLastTime = false;
    private double lastTime;

    // Active samples in time order
    private final Deque<Sample> samples = new ArrayDeque<>();

    // Recycled Sample objects
    private final Deque<Sample> pool = new ArrayDeque<>();

    private double sum = 0.0;
    private double sumSq = 0.0;

    public RunningIntervalStats(double windowSeconds) {
        this.windowSeconds = windowSeconds;

        for (int i = 0; i < (windowSeconds * 115); i++) {
            pool.addLast(new Sample());
        }

        startTime = System.nanoTime() / 1e9;
    }

    /**
     * Change the sliding-window size.
     * Immediately evicts samples that fall outside the new window.
     */
    public void reconfigure(double windowSeconds) {
        this.windowSeconds = windowSeconds;

        if (samples.isEmpty())
            return;

        double newestTime = samples.peekLast().timestamp;
        evictOld(newestTime);
    }

    /**
     * Record a timestamp (in seconds).
     */
    public void record() {
        nowSeconds = System.nanoTime() / 1e9;
        if (hasLastTime) {
            double intervalMs = (nowSeconds - lastTime) * 1000.0;

            samples.addLast(obtainSample(nowSeconds, intervalMs));
            sum += intervalMs;
            sumSq += intervalMs * intervalMs;

            evictOld(nowSeconds);
        }

        lastTime = nowSeconds;
        hasLastTime = true;
    }

    public double getDt() {
        if (!samples.isEmpty()) {
            return samples.peekLast().intervalMs / 1000.0;
        } else {
            return Double.NaN;
        }
    }

    public double getElapsed() {
        return nowSeconds - startTime;
    }

    public void writeTelemetry(TelemetryAddData telemetry) {
        // Show the elapsed game time
        telemetry.addData("Run Time", "%.2f", getElapsed());
        telemetry.addData("Looptime mean", "%.1f", getMeanMs());
        telemetry.addData("Looptime stdev", "%.1f", getStdDevMs());
    }

    private Sample obtainSample(double timestamp, double intervalMs) {
        Sample s = pool.pollLast();
        s = (s != null) ? s : new Sample();
        s.timestamp = timestamp;
        s.intervalMs = intervalMs;
        return s;
    }

    private void recycleSample(Sample s) {
        if (pool.size() < MAX_POOL_SIZE) {
            pool.addLast(s);
        }
    }

    private void evictOld(double nowSeconds) {
        double cutoff = nowSeconds - windowSeconds;

        while (!samples.isEmpty() && samples.peekFirst().timestamp < cutoff) {
            Sample old = samples.removeFirst();
            sum -= old.intervalMs;
            sumSq -= old.intervalMs * old.intervalMs;
            recycleSample(old);
        }
    }

    public double getMeanMs() {
        int n = samples.size();
        return n == 0 ? 0.0 : sum / n;
    }

    public double getStdDevMs() {
        int n = samples.size();
        if (n <= 1) return 0.0;

        double mean = sum / n;
        double variance = (sumSq - n * mean * mean) / (n - 1);
        return Math.sqrt(Math.max(variance, 0.0));
    }

    public int getCount() {
        return samples.size();
    }
}
