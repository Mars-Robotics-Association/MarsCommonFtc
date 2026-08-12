package org.marsroboticsassociation.controllib.mechanism

import java.util.Random
import kotlin.math.abs
import kotlin.math.roundToInt

/**
 * Test-only model of the REV Hub's position read-timing jitter.
 *
 * <p>Even though `getCurrentPosition()` is a live counter, the value the control loop consumes was
 * captured at the hub's bulk-read instant and reaches the loop a little later, and that transport
 * delay varies (worse over an Expansion Hub's RS485 hop). This reproduces it: a position read
 * returns the count as of `t_read - delta`, i.e. `round(trueTicks - velocity * delta)`, with
 * `delta >= 0` drawn from a seeded RNG so tests stay deterministic. The result is the dominant real
 * noise when position is differenced — a velocity-scaled `v * delta` error plus a faint bias.
 *
 * <p>The magnitude is hub-configurable: a small Control-Hub default and a larger Expansion-Hub
 * preset. A std of 0 disables jitter, giving an exact live read for exact-value assertions.
 */
class ReadTimingJitter(private val stdSec: Double, seed: Long) {
    private val rng = Random(seed)

    /** The standard deviation of the per-read staleness, in seconds. */
    fun stdSec(): Double = stdSec

    /**
     * Draw one bulk-read transport delay `delta >= 0`, in seconds. The caller shares this one delay
     * across the position and velocity reads of a single snapshot.
     */
    fun nextDelta(): Double = if (stdSec > 0.0) abs(rng.nextGaussian()) * stdSec else 0.0

    /**
     * Read the live position, staled by a per-read transport delay `delta >= 0`. Convenience for
     * callers that only need the position read; drawing a fresh delay each call.
     *
     * @param trueTicks the exact live count in ticks
     * @param velocityTps the true velocity in ticks/sec (scales the staleness error)
     * @return `round(trueTicks - velocity * delta)`
     */
    fun read(trueTicks: Double, velocityTps: Double): Int =
        staleTicks(trueTicks, velocityTps, nextDelta())

    companion object {
        /** Control-Hub read-timing jitter: a few tenths of a millisecond. */
        const val CONTROL_HUB_STD_SEC = 0.0003

        /** Expansion-Hub read-timing jitter: the RS485 hop adds a millisecond or two. */
        const val EXPANSION_HUB_STD_SEC = 0.002

        fun controlHub(seed: Long): ReadTimingJitter = ReadTimingJitter(CONTROL_HUB_STD_SEC, seed)

        fun expansionHub(seed: Long): ReadTimingJitter =
            ReadTimingJitter(EXPANSION_HUB_STD_SEC, seed)

        /** Disabled jitter: position reads are exact (delta == 0). */
        fun disabled(): ReadTimingJitter = ReadTimingJitter(0.0, 0L)

        /** Apply an already-drawn delay to a live position read: `round(trueTicks - v*delta)`. */
        fun staleTicks(trueTicks: Double, velocityTps: Double, deltaSec: Double): Int =
            (trueTicks - velocityTps * deltaSec).roundToInt()
    }
}
