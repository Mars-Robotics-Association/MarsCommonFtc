package org.marsroboticsassociation.controllib.mechanism

import kotlin.math.floor
import kotlin.math.min
import kotlin.math.roundToInt

/**
 * Test-only model of the REV Hub encoder's *velocity* signal, faithful enough to exercise the
 * Kalman filters the way a real robot would.
 *
 * <p>On a real hub the two encoder signals behave very differently. `getCurrentPosition()` is a
 * *live* counter — fresh on every read, no 10 ms latch. `getVelocity()` is the only windowed
 * signal: the firmware samples the counter every 10 ms into a small ring buffer and reports (newest
 * − oldest) / span, where with a full 6-entry buffer the span is 5 × 10 ms = 50 ms. Because both
 * endpoints are whole ticks over a fixed 50 ms window, the reported velocity is quantized to
 * multiples of 20 ticks/sec.
 *
 * <p>This class models *only* that velocity window. Position is read live (with a small read-timing
 * jitter) straight from the plant, so it does not belong here. Call [sample] once per 10 ms
 * firmware tick with the mechanism's true position in ticks, and the 50 ms quantized velocity falls
 * out on its own.
 *
 * <p>The important property for these tests: the velocity is differenced over a *precise,
 * hardware-timed* 50 ms window, independent of when your control loop happens to read it. A
 * velocity you compute yourself by differencing the live position across control-loop iterations is
 * at the mercy of the loop's jittery timing instead.
 *
 * <p>The same bulk-read transport delay that stales position stales this signal too, but a 10
 * ms-stepped boxcar does not shift smoothly: a delay only changes the reading when it reaches back
 * past the most recent latch, returning the *previous* 50 ms window. [velocityTpsStaledBy]
 * reproduces that; the plant owns the (shared) delay and the time since the last latch.
 */
class EncoderSim {
    private val buffer = IntArray(RING_SIZE)
    private var head = -1
    private var count = 0

    /** Latch one firmware sample: the current true position, in ticks, rounded to a whole tick. */
    fun sample(truePositionTicks: Double) {
        head = (head + 1) % RING_SIZE
        buffer[head] = truePositionTicks.roundToInt()
        count++
    }

    /**
     * Velocity in ticks/sec over the current 50 ms ring-buffer window, like
     * `DcMotorEx.getVelocity()`. Returns 0 until at least two samples exist.
     */
    fun getVelocityTps(): Double = windowVelocity(0)

    /**
     * The windowed velocity as it would read through a bulk read delayed by [deltaSec]: the current
     * 50 ms window if [deltaSec] does not reach past the most recent latch ([ageSinceLastSampleSec]
     * ago), otherwise the earlier window it reaches back to.
     *
     * @param deltaSec the read-timing delay to apply, in seconds (>= 0)
     * @param ageSinceLastSampleSec time since the last [sample] call, in seconds
     */
    fun velocityTpsStaledBy(deltaSec: Double, ageSinceLastSampleSec: Double): Double {
        var windowsBack = 0
        if (deltaSec > ageSinceLastSampleSec) {
            windowsBack = 1 + floor((deltaSec - ageSinceLastSampleSec) / SAMPLE_PERIOD_SEC).toInt()
        }
        return windowVelocity(windowsBack)
    }

    /** Velocity over the 50 ms window whose newest sample is [offset] latches before the head. */
    private fun windowVelocity(offset: Int): Double {
        val available = count - offset
        if (available < 2) {
            return 0.0
        }
        val n = min(available, WINDOW_SIZE)
        val newest = ((head - offset) % RING_SIZE + RING_SIZE) % RING_SIZE
        val oldest = ((newest - (n - 1)) % RING_SIZE + RING_SIZE) % RING_SIZE
        val spanSec = (n - 1) * SAMPLE_PERIOD_SEC
        return (buffer[newest] - buffer[oldest]) / spanSec
    }

    companion object {
        private const val WINDOW_SIZE = 6 // 6 samples -> 5 * 10 ms = 50 ms velocity window
        const val SAMPLE_PERIOD_SEC = 0.010
        // The ring holds more than one window so a delay reaching past the last latch can read an
        // earlier window (the quantized effect of the read-timing delay on velocity).
        private const val RING_SIZE = 12
    }
}
