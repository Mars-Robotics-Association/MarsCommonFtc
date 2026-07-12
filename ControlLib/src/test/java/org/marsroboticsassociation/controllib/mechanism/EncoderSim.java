package org.marsroboticsassociation.controllib.mechanism;

/**
 * Test-only model of the REV Hub encoder's <em>velocity</em> signal, faithful enough to exercise
 * the Kalman filters the way a real robot would.
 *
 * <p>On a real hub the two encoder signals behave very differently. {@code getCurrentPosition()} is
 * a <em>live</em> counter &mdash; fresh on every read, no 10 ms latch. {@code getVelocity()} is the
 * only windowed signal: the firmware samples the counter every 10 ms into a small ring buffer and
 * reports (newest &minus; oldest) / span, where with a full 6-entry buffer the span is 5 &times; 10
 * ms = 50 ms. Because both endpoints are whole ticks over a fixed 50 ms window, the reported
 * velocity is quantized to multiples of 20 ticks/sec.
 *
 * <p>This class models <em>only</em> that velocity window. Position is read live (with a small
 * read-timing jitter) straight from the plant, so it does not belong here. Call {@link
 * #sample(double)} once per 10 ms firmware tick with the mechanism's true position in ticks, and
 * the 50 ms quantized velocity falls out on its own.
 *
 * <p>The important property for these tests: the velocity is differenced over a <em>precise,
 * hardware-timed</em> 50 ms window, independent of when your control loop happens to read it. A
 * velocity you compute yourself by differencing the live position across control-loop iterations is
 * at the mercy of the loop's jittery timing instead.
 *
 * <p>The same bulk-read transport delay that stales position stales this signal too, but a 10
 * ms-stepped boxcar does not shift smoothly: a delay only changes the reading when it reaches back
 * past the most recent latch, returning the <em>previous</em> 50 ms window. {@link
 * #velocityTpsStaledBy(double, double)} reproduces that; the plant owns the (shared) delay and the
 * time since the last latch.
 */
public class EncoderSim {

    private static final int WINDOW_SIZE = 6; // 6 samples -> 5 * 10 ms = 50 ms velocity window
    public static final double SAMPLE_PERIOD_SEC = 0.010;
    // The ring holds more than one window so a delay reaching past the last latch can read an
    // earlier window (the quantized effect of the read-timing delay on velocity).
    private static final int RING_SIZE = 12;

    private final int[] buffer = new int[RING_SIZE];
    private int head = -1;
    private int count = 0;

    /** Latch one firmware sample: the current true position, in ticks, rounded to a whole tick. */
    public void sample(double truePositionTicks) {
        head = (head + 1) % RING_SIZE;
        buffer[head] = (int) Math.round(truePositionTicks);
        count++;
    }

    /**
     * Velocity in ticks/sec over the current 50 ms ring-buffer window, like {@code
     * DcMotorEx.getVelocity()}. Returns 0 until at least two samples exist.
     */
    public double getVelocityTps() {
        return windowVelocity(0);
    }

    /**
     * The windowed velocity as it would read through a bulk read delayed by {@code deltaSec}: the
     * current 50 ms window if {@code deltaSec} does not reach past the most recent latch ({@code
     * ageSinceLastSampleSec} ago), otherwise the earlier window it reaches back to.
     *
     * @param deltaSec the read-timing delay to apply, in seconds (>= 0)
     * @param ageSinceLastSampleSec time since the last {@link #sample} call, in seconds
     */
    public double velocityTpsStaledBy(double deltaSec, double ageSinceLastSampleSec) {
        int windowsBack = 0;
        if (deltaSec > ageSinceLastSampleSec) {
            windowsBack =
                    1 + (int) Math.floor((deltaSec - ageSinceLastSampleSec) / SAMPLE_PERIOD_SEC);
        }
        return windowVelocity(windowsBack);
    }

    /**
     * Velocity over the 50 ms window whose newest sample is {@code offset} latches before the head.
     */
    private double windowVelocity(int offset) {
        int available = count - offset;
        if (available < 2) {
            return 0.0;
        }
        int n = Math.min(available, WINDOW_SIZE);
        int newest = ((head - offset) % RING_SIZE + RING_SIZE) % RING_SIZE;
        int oldest = ((newest - (n - 1)) % RING_SIZE + RING_SIZE) % RING_SIZE;
        double spanSec = (n - 1) * SAMPLE_PERIOD_SEC;
        return (buffer[newest] - buffer[oldest]) / spanSec;
    }
}
