package org.marsroboticsassociation.controllib.sim;

import java.util.Random;

/**
 * Encoder sensor model that faithfully simulates the REV Hub encoder, split into its two very
 * different signals fused from one bulk read.
 *
 * <p><b>One snapshot, one delay.</b> A bulk read snapshots the whole hub state at one instant
 * {@code t_capture} and delivers it to the loop a moment later at {@code t_read = t_capture +
 * delta}; that transport delay varies (worse over an Expansion Hub's RS485 hop). Both signals below
 * therefore reflect the <em>same</em> capture instant {@code t_read - delta}. The per-read
 * {@code delta >= 0} is drawn once from a seeded RNG and shared by position and velocity (it is
 * redrawn each {@link #advance}, i.e. once per loop step). Set the std to 0 to disable jitter for
 * exact-value tests. The magnitude is hub-configurable: a small Control-Hub default and a larger
 * Expansion-Hub preset.
 *
 * <p><b>Position is live.</b> {@code getCurrentPosition()} on a real hub returns a fresh counter
 * value on every read &mdash; no 10 ms latch, no plateau. Staled by {@code delta}, the read is the
 * count as of {@code t_read - delta}, i.e. {@code round(livePosition - velocity * delta)}: a smooth,
 * velocity-scaled error (the dominant real noise when position is differenced) plus a faint bias.
 *
 * <p><b>Velocity is the only windowed signal.</b> The firmware samples the counter every 10 ms into
 * a ring buffer and reports velocity as (newest &minus; oldest) / span. With 6 entries the span is
 * 5 &times; 10 ms = 50 ms, producing readings quantized to multiples of 20 TPS (1 tick / 0.050 s),
 * matching observed telemetry. The same {@code delta} stales it too, but a 10 ms-stepped boxcar does
 * not shift smoothly: {@code delta} only changes the reading when it reaches back past the most
 * recent latch, in which case the read returns the <em>previous</em> 50 ms window. That quantized
 * snapshot-as-of-{@code t_read - delta} behavior is reproduced here.
 *
 * <p>This model receives the true motor velocity each step and maintains the encoder state
 * independently of the plant simulation.
 *
 * <p><b>Note:</b> {@link #setState(int, double)} sets the internal sample {@code count} to its
 * {@code position} argument, which is a latent oddity (the name suggests an encoder position, but it
 * seeds the sample counter). Callers rely on it only to seed {@code fractionalTicks}.
 */
public class EncoderSim {

    private static final int WINDOW_SIZE = 6; // 6 samples -> 5 * 10 ms = 50 ms velocity window
    private static final double SAMPLE_PERIOD_SEC = 0.010;
    // The ring holds more than one window so a delta reaching past the last latch can read an
    // earlier window (the quantized effect of the read-timing delay on velocity).
    private static final int RING_SIZE = 12;

    /** Control-Hub read-timing jitter: a few tenths of a millisecond. */
    public static final double CONTROL_HUB_JITTER_STD_SEC = 0.0003;

    /** Expansion-Hub read-timing jitter: the RS485 hop adds a millisecond or two. */
    public static final double EXPANSION_HUB_JITTER_STD_SEC = 0.002;

    private final int[] buffer = new int[RING_SIZE];
    private int head = 0;
    private int count = 0;

    private double fractionalTicks = 0.0;
    private double timeSinceLastSampleSec = 0.0;
    private double lastVelocityTps = 0.0;

    // Read-timing jitter: one delta = |N(0, jitterStdSec)| per loop step, shared by both signals.
    private final double jitterStdSec;
    private final Random jitterRng;
    private double pendingDelta = Double.NaN; // the current snapshot's delta, or NaN until drawn

    /** Construct with read-timing jitter disabled (exact live position). */
    public EncoderSim() {
        this(0.0, 0L);
    }

    /**
     * Construct with a custom read-timing jitter std and RNG seed.
     *
     * @param jitterStdSec standard deviation of the per-read staleness, in seconds (0 disables it)
     * @param seed         RNG seed so jittered reads are reproducible
     */
    public EncoderSim(double jitterStdSec, long seed) {
        this.jitterStdSec = jitterStdSec;
        this.jitterRng = new Random(seed);
    }

    /** A Control-Hub encoder: small read-timing jitter (a few tenths of a millisecond). */
    public static EncoderSim controlHub(long seed) {
        return new EncoderSim(CONTROL_HUB_JITTER_STD_SEC, seed);
    }

    /** An Expansion-Hub encoder: larger read-timing jitter from the RS485 hop (~1-3 ms). */
    public static EncoderSim expansionHub(long seed) {
        return new EncoderSim(EXPANSION_HUB_JITTER_STD_SEC, seed);
    }

    /**
     * Advance the encoder by one time step. Position is integrated continuously (it is live); the
     * ring buffer is written at each 10 ms firmware boundary to drive the velocity window. A fresh
     * read-timing snapshot delta is drawn on the next read after each advance.
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
        lastVelocityTps = velocityTps;
        pendingDelta = Double.NaN; // new step -> new bulk-read snapshot
    }

    private void writeSample(double exactPos) {
        head = (head + 1) % RING_SIZE;
        buffer[head] = (int) Math.round(exactPos);
        count++;
    }

    /** The shared read-timing delay for the current snapshot, drawn once and reused until advance. */
    private double snapshotDelta() {
        if (jitterStdSec <= 0.0) return 0.0;
        if (Double.isNaN(pendingDelta)) {
            pendingDelta = Math.abs(jitterRng.nextGaussian()) * jitterStdSec;
        }
        return pendingDelta;
    }

    /**
     * Returns the live integer tick position (equivalent to {@code getCurrentPosition()}), staled by
     * the snapshot's transport delay {@code delta >= 0}: {@code round(livePosition - velocity *
     * delta)}. With jitter disabled this is just the rounded live count.
     */
    public int getPosition() {
        // Position is live: it comes from the continuously-integrated count, not the ring buffer,
        // so it is available immediately (no wait for the first 10 ms sample).
        double delta = snapshotDelta();
        return (int) Math.round(fractionalTicks - lastVelocityTps * delta);
    }

    /**
     * Returns the windowed velocity in TPS, matching the REV Hub algorithm, as seen through the
     * current bulk read (staled by the shared snapshot delay).
     *
     * <p>Velocity = (newest &minus; oldest) / span over the 50 ms window. Returns 0 before 2 samples
     * exist.
     */
    public double getVelocityTps() {
        return velocityTpsStaledBy(snapshotDelta());
    }

    /**
     * The windowed velocity as it would read through a bulk read delayed by {@code deltaSec}: the
     * current 50 ms window if {@code deltaSec} does not reach past the most recent 10 ms latch,
     * otherwise the window ending at the latch {@code deltaSec} reaches back to. Exposed for
     * deterministic tests (the production path uses the randomly drawn snapshot delay).
     *
     * @param deltaSec the read-timing delay to apply, in seconds (>= 0)
     */
    public double velocityTpsStaledBy(double deltaSec) {
        int windowsBack = 0;
        if (deltaSec > timeSinceLastSampleSec) {
            windowsBack = 1 + (int) Math.floor((deltaSec - timeSinceLastSampleSec) / SAMPLE_PERIOD_SEC);
        }
        return windowVelocity(windowsBack);
    }

    /** Velocity over the 50 ms window whose newest sample is {@code offset} latches before the head. */
    private double windowVelocity(int offset) {
        int available = count - offset;
        if (available < 2) return 0.0;
        int n = Math.min(available, WINDOW_SIZE);
        int newest = ((head - offset) % RING_SIZE + RING_SIZE) % RING_SIZE;
        int oldest = ((newest - (n - 1)) % RING_SIZE + RING_SIZE) % RING_SIZE;
        double spanSec = (n - 1) * SAMPLE_PERIOD_SEC;
        return (buffer[newest] - buffer[oldest]) / spanSec;
    }

    public void setState(int position, double fractionalTicks) {
        this.count = position;
        this.fractionalTicks = fractionalTicks;
        // The sample ring keeps its history; reset() is what clears it.
    }

    /** Clears the buffer and resets all state. */
    public void reset() {
        head = 0;
        count = 0;
        fractionalTicks = 0.0;
        timeSinceLastSampleSec = 0.0;
        lastVelocityTps = 0.0;
        pendingDelta = Double.NaN;
        for (int i = 0; i < RING_SIZE; i++) buffer[i] = 0;
    }
}
