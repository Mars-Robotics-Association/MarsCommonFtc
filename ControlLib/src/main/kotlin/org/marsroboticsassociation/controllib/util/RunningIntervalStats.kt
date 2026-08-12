package org.marsroboticsassociation.controllib.util

import java.util.ArrayDeque
import kotlin.math.max
import kotlin.math.sqrt

class RunningIntervalStats(private var windowSeconds: Double) {

    private class Sample {
        var timestamp: Double = 0.0 // seconds
        var intervalMs: Double = 0.0 // milliseconds
    }

    private var startTime: Double
    private var nowSeconds: Double = 0.0
    private var hasLastTime = false
    private var lastTime: Double = 0.0

    private val samples = ArrayDeque<Sample>()
    private val pool = ArrayDeque<Sample>()

    private var sum = 0.0
    private var sumSq = 0.0

    init {
        var i = 0
        val n = (windowSeconds * 115).toInt()
        while (i < n) {
            pool.addLast(Sample())
            i++
        }
        startTime = System.nanoTime() / 1e9
    }

    /**
     * Change the sliding-window size. Immediately evicts samples that fall outside the new window.
     */
    fun reconfigure(windowSeconds: Double) {
        this.windowSeconds = windowSeconds
        if (samples.isEmpty()) return
        val newestTime = samples.peekLast().timestamp
        evictOld(newestTime)
    }

    /** Record a timestamp (in seconds). */
    fun record() {
        nowSeconds = System.nanoTime() / 1e9
        if (hasLastTime) {
            val intervalMs = (nowSeconds - lastTime) * 1000.0
            samples.addLast(obtainSample(nowSeconds, intervalMs))
            sum += intervalMs
            sumSq += intervalMs * intervalMs
            evictOld(nowSeconds)
        }
        lastTime = nowSeconds
        hasLastTime = true
    }

    val dt: Double
        get() =
            if (!samples.isEmpty()) {
                samples.peekLast().intervalMs / 1000.0
            } else {
                Double.NaN
            }

    val elapsed: Double
        get() = nowSeconds - startTime

    fun writeTelemetry(telemetry: TelemetryAddData) {
        telemetry.addData("Run Time", "%.2f", elapsed)
        telemetry.addData("Looptime mean", "%.1f", meanMs)
        telemetry.addData("Looptime stdev", "%.1f", stdDevMs)
    }

    private fun obtainSample(timestamp: Double, intervalMs: Double): Sample {
        val s = pool.pollLast() ?: Sample()
        s.timestamp = timestamp
        s.intervalMs = intervalMs
        return s
    }

    private fun recycleSample(s: Sample) {
        if (pool.size < MAX_POOL_SIZE) {
            pool.addLast(s)
        }
    }

    private fun evictOld(nowSeconds: Double) {
        val cutoff = nowSeconds - windowSeconds
        while (!samples.isEmpty() && samples.peekFirst().timestamp < cutoff) {
            val old = samples.removeFirst()
            sum -= old.intervalMs
            sumSq -= old.intervalMs * old.intervalMs
            recycleSample(old)
        }
    }

    val meanMs: Double
        get() {
            val n = samples.size
            return if (n == 0) 0.0 else sum / n
        }

    val stdDevMs: Double
        get() {
            val n = samples.size
            if (n <= 1) return 0.0
            val mean = sum / n
            val variance = (sumSq - n * mean * mean) / (n - 1)
            return sqrt(max(variance, 0.0))
        }

    val count: Int
        get() = samples.size

    companion object {
        /** Max number of recycled Sample objects kept. */
        private const val MAX_POOL_SIZE = 20_000
    }
}
