package org.marsroboticsassociation.controllib.sim

import java.util.Random
import kotlin.math.abs
import kotlin.math.floor
import kotlin.math.round

/**
 * Encoder sensor model that faithfully simulates the REV Hub encoder, split into its two very
 * different signals fused from one bulk read.
 *
 * <p><b>One snapshot, one delay.</b> A bulk read snapshots the whole hub state at one instant
 * {@code t_capture} and delivers it to the loop a moment later at {@code t_read = t_capture +
 * delta}; that transport delay varies (worse over an Expansion Hub's RS485 hop). Both signals below
 * therefore reflect the <em>same</em> capture instant {@code t_read - delta}. The per-read {@code
 * delta >= 0} is drawn once from a seeded RNG and shared by position and velocity (it is redrawn
 * each [advance], i.e. once per loop step). Set the std to 0 to disable jitter for exact-value
 * tests. The magnitude is hub-configurable: a small Control-Hub default and a larger Expansion-Hub
 * preset.
 *
 * <p><b>Position is live.</b> {@code getCurrentPosition()} on a real hub returns a fresh counter
 * value on every read &mdash; no 10 ms latch, no plateau. Staled by {@code delta}, the read is the
 * count as of {@code t_read - delta}, i.e. {@code round(livePosition - velocity * delta)}: a
 * smooth, velocity-scaled error (the dominant real noise when position is differenced) plus a faint
 * bias.
 *
 * <p><b>Velocity is the only windowed signal.</b> The firmware samples the counter every 10 ms into
 * a ring buffer and reports velocity as (newest &minus; oldest) / span. With 6 entries the span is
 * 5 &times; 10 ms = 50 ms, producing readings quantized to multiples of 20 TPS (1 tick / 0.050 s),
 * matching observed telemetry. The same {@code delta} stales it too, but a 10 ms-stepped boxcar
 * does not shift smoothly: {@code delta} only changes the reading when it reaches back past the
 * most recent latch, in which case the read returns the <em>previous</em> 50 ms window. That
 * quantized snapshot-as-of-{@code t_read - delta} behavior is reproduced here.
 *
 * <p>This model receives the true motor velocity each step and maintains the encoder state
 * independently of the plant simulation.
 *
 * <p><b>Note:</b> [setState] sets the internal sample {@code count} to its {@code position}
 * argument, which is a latent oddity (the name suggests an encoder position, but it seeds the
 * sample counter). Callers rely on it only to seed {@code fractionalTicks}.
 */
class EncoderSim(private val jitterStdSec: Double, seed: Long) {

    private val buffer = IntArray(RING_SIZE)
    private var head = 0
    private var count = 0

    private var fractionalTicks = 0.0
    private var timeSinceLastSampleSec = 0.0
    private var lastVelocityTps = 0.0

    // Read-timing jitter: one delta = |N(0, jitterStdSec)| per loop step, shared by both signals.
    private val jitterRng = Random(seed)
    private var pendingDelta = Double.NaN // the current snapshot's delta, or NaN until drawn

    /** Construct with read-timing jitter disabled (exact live position). */
    constructor() : this(0.0, 0L)

    /**
     * Advance the encoder by one time step. Position is integrated continuously (it is live); the
     * ring buffer is written at each 10 ms firmware boundary to drive the velocity window. A fresh
     * read-timing snapshot delta is drawn on the next read after each advance.
     *
     * @param dt time step in seconds
     * @param velocityTps true motor velocity in ticks per second
     */
    fun advance(dt: Double, velocityTps: Double) {
        var remaining = dt
        var pos = fractionalTicks
        var tSince = timeSinceLastSampleSec

        while (remaining > 1e-12) {
            val timeToNext = SAMPLE_PERIOD_SEC - tSince
            if (timeToNext <= remaining + 1e-12) {
                pos += velocityTps * timeToNext
                writeSample(pos)
                tSince = 0.0
                remaining -= timeToNext
            } else {
                pos += velocityTps * remaining
                tSince += remaining
                remaining = 0.0
            }
        }

        fractionalTicks = pos
        timeSinceLastSampleSec = tSince
        lastVelocityTps = velocityTps
        pendingDelta = Double.NaN // new step -> new bulk-read snapshot
    }

    private fun writeSample(exactPos: Double) {
        head = (head + 1) % RING_SIZE
        buffer[head] = round(exactPos).toInt()
        count++
    }

    /**
     * The shared read-timing delay for the current snapshot, drawn once and reused until advance.
     */
    private fun snapshotDelta(): Double {
        if (jitterStdSec <= 0.0) return 0.0
        if (pendingDelta.isNaN()) {
            pendingDelta = abs(jitterRng.nextGaussian()) * jitterStdSec
        }
        return pendingDelta
    }

    /**
     * Returns the live integer tick position (equivalent to {@code getCurrentPosition()}), staled
     * by the snapshot's transport delay {@code delta >= 0}: {@code round(livePosition - velocity *
     * delta)}. With jitter disabled this is just the rounded live count.
     */
    val position: Int
        get() {
            // Position is live: it comes from the continuously-integrated count, not the ring
            // buffer, so it is available immediately (no wait for the first 10 ms sample).
            val delta = snapshotDelta()
            return round(fractionalTicks - lastVelocityTps * delta).toInt()
        }

    /**
     * Returns the windowed velocity in TPS, matching the REV Hub algorithm, as seen through the
     * current bulk read (staled by the shared snapshot delay).
     *
     * <p>Velocity = (newest &minus; oldest) / span over the 50 ms window. Returns 0 before 2
     * samples exist.
     */
    val velocityTps: Double
        get() = velocityTpsStaledBy(snapshotDelta())

    /**
     * The windowed velocity as it would read through a bulk read delayed by [deltaSec]: the current
     * 50 ms window if [deltaSec] does not reach past the most recent 10 ms latch, otherwise the
     * window ending at the latch [deltaSec] reaches back to. Exposed for deterministic tests (the
     * production path uses the randomly drawn snapshot delay).
     *
     * @param deltaSec the read-timing delay to apply, in seconds (>= 0)
     */
    fun velocityTpsStaledBy(deltaSec: Double): Double {
        var windowsBack = 0
        if (deltaSec > timeSinceLastSampleSec) {
            windowsBack = 1 + floor((deltaSec - timeSinceLastSampleSec) / SAMPLE_PERIOD_SEC).toInt()
        }
        return windowVelocity(windowsBack)
    }

    /** Velocity over the 50 ms window whose newest sample is [offset] latches before the head. */
    private fun windowVelocity(offset: Int): Double {
        val available = count - offset
        if (available < 2) return 0.0
        val n = minOf(available, WINDOW_SIZE)
        val newest = ((head - offset) % RING_SIZE + RING_SIZE) % RING_SIZE
        val oldest = ((newest - (n - 1)) % RING_SIZE + RING_SIZE) % RING_SIZE
        val spanSec = (n - 1) * SAMPLE_PERIOD_SEC
        return (buffer[newest] - buffer[oldest]) / spanSec
    }

    fun setState(position: Int, fractionalTicks: Double) {
        this.count = position
        this.fractionalTicks = fractionalTicks
        // The sample ring keeps its history; reset() is what clears it.
    }

    /** Clears the buffer and resets all state. */
    fun reset() {
        head = 0
        count = 0
        fractionalTicks = 0.0
        timeSinceLastSampleSec = 0.0
        lastVelocityTps = 0.0
        pendingDelta = Double.NaN
        for (i in 0 until RING_SIZE) buffer[i] = 0
    }

    companion object {
        private const val WINDOW_SIZE = 6 // 6 samples -> 5 * 10 ms = 50 ms velocity window
        private const val SAMPLE_PERIOD_SEC = 0.010
        // The ring holds more than one window so a delta reaching past the last latch can read an
        // earlier window (the quantized effect of the read-timing delay on velocity).
        private const val RING_SIZE = 12

        /** Control-Hub read-timing jitter: a few tenths of a millisecond. */
        @JvmField val CONTROL_HUB_JITTER_STD_SEC = 0.0003

        /** Expansion-Hub read-timing jitter: the RS485 hop adds a millisecond or two. */
        @JvmField val EXPANSION_HUB_JITTER_STD_SEC = 0.002

        /** A Control-Hub encoder: small read-timing jitter (a few tenths of a millisecond). */
        @JvmStatic
        fun controlHub(seed: Long): EncoderSim {
            return EncoderSim(CONTROL_HUB_JITTER_STD_SEC, seed)
        }

        /** An Expansion-Hub encoder: larger read-timing jitter from the RS485 hop (~1-3 ms). */
        @JvmStatic
        fun expansionHub(seed: Long): EncoderSim {
            return EncoderSim(EXPANSION_HUB_JITTER_STD_SEC, seed)
        }
    }
}
