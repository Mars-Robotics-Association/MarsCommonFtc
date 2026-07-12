package org.marsroboticsassociation.controllib.mechanism;

import java.util.Random;

/**
 * Test-only model of the REV Hub's position read-timing jitter.
 *
 * <p>Even though {@code getCurrentPosition()} is a live counter, the value the control loop
 * consumes was captured at the hub's bulk-read instant and reaches the loop a little later, and
 * that transport delay varies (worse over an Expansion Hub's RS485 hop). This reproduces it: a
 * position read returns the count as of {@code t_read - delta}, i.e. {@code round(trueTicks -
 * velocity * delta)}, with {@code delta >= 0} drawn from a seeded RNG so tests stay deterministic.
 * The result is the dominant real noise when position is differenced &mdash; a velocity-scaled
 * {@code v * delta} error plus a faint bias.
 *
 * <p>The magnitude is hub-configurable: a small Control-Hub default and a larger Expansion-Hub
 * preset. A std of 0 disables jitter, giving an exact live read for exact-value assertions.
 */
public final class ReadTimingJitter {

    /** Control-Hub read-timing jitter: a few tenths of a millisecond. */
    public static final double CONTROL_HUB_STD_SEC = 0.0003;

    /** Expansion-Hub read-timing jitter: the RS485 hop adds a millisecond or two. */
    public static final double EXPANSION_HUB_STD_SEC = 0.002;

    private final double stdSec;
    private final Random rng;

    public ReadTimingJitter(double stdSec, long seed) {
        this.stdSec = stdSec;
        this.rng = new Random(seed);
    }

    public static ReadTimingJitter controlHub(long seed) {
        return new ReadTimingJitter(CONTROL_HUB_STD_SEC, seed);
    }

    public static ReadTimingJitter expansionHub(long seed) {
        return new ReadTimingJitter(EXPANSION_HUB_STD_SEC, seed);
    }

    /** Disabled jitter: position reads are exact (delta == 0). */
    public static ReadTimingJitter disabled() {
        return new ReadTimingJitter(0.0, 0L);
    }

    /** The standard deviation of the per-read staleness, in seconds. */
    public double stdSec() {
        return stdSec;
    }

    /**
     * Draw one bulk-read transport delay {@code delta >= 0}, in seconds. The caller shares this one
     * delay across the position and velocity reads of a single snapshot.
     */
    public double nextDelta() {
        return (stdSec > 0.0) ? Math.abs(rng.nextGaussian()) * stdSec : 0.0;
    }

    /**
     * Read the live position, staled by a per-read transport delay {@code delta >= 0}. Convenience
     * for callers that only need the position read; drawing a fresh delay each call.
     *
     * @param trueTicks the exact live count in ticks
     * @param velocityTps the true velocity in ticks/sec (scales the staleness error)
     * @return {@code round(trueTicks - velocity * delta)}
     */
    public int read(double trueTicks, double velocityTps) {
        return staleTicks(trueTicks, velocityTps, nextDelta());
    }

    /** Apply an already-drawn delay to a live position read: {@code round(trueTicks - v*delta)}. */
    public static int staleTicks(double trueTicks, double velocityTps, double deltaSec) {
        return (int) Math.round(trueTicks - velocityTps * deltaSec);
    }
}
