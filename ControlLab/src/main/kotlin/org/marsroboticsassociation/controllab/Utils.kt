package org.marsroboticsassociation.controllab

import java.io.BufferedWriter
import java.io.IOException
import java.nio.file.Files
import java.nio.file.Path
import kotlin.math.sqrt

object Utils {

    /**
     * Estimate lag (seconds) between raw and filtered signals using discrete cross-correlation.
     * Returns lag in seconds where positive means filtered lags behind raw. This computes
     * correlation for integer sample shifts only.
     */
    @JvmStatic
    fun estimateLagSeconds(raw: List<Double>, filt: List<Double>, time: List<Double>): Double {
        val n = minOf(raw.size, filt.size)
        if (n < 3) return 0.0

        // zero-mean
        val meanR = raw.take(n).average()
        val meanF = filt.take(n).average()

        val r = DoubleArray(n)
        val f = DoubleArray(n)
        for (i in 0 until n) {
            r[i] = raw[i] - meanR
            f[i] = filt[i] - meanF
        }

        // precompute std for normalization
        var stdR = 0.0
        var stdF = 0.0
        for (i in 0 until n) {
            stdR += r[i] * r[i]
            stdF += f[i] * f[i]
        }
        stdR = sqrt(stdR)
        stdF = sqrt(stdF)
        if (stdR == 0.0 || stdF == 0.0) return 0.0 // constant signals

        val maxLag = minOf(n / 2, 500) // limit lag in samples
        var bestCorr = Double.NEGATIVE_INFINITY
        var bestLag = 0

        for (lag in -maxLag..maxLag) {
            var cross = 0.0
            var sumR2 = 0.0
            var sumF2 = 0.0
            var count = 0

            for (i in 0 until n) {
                val j = i + lag
                if (j < 0 || j >= n) continue

                val ri = r[i]
                val fj = f[j]

                cross += ri * fj
                sumR2 += ri * ri
                sumF2 += fj * fj
                count++
            }

            if (count < 3) continue // too few samples

            val corr = cross / sqrt(sumR2 * sumF2)

            if (corr > bestCorr) {
                bestCorr = corr
                bestLag = lag
            }
        }

        // compute average dt
        var dtAvg = 0.0
        val dtCount = minOf(n - 1, 1000)
        for (i in 1..dtCount) {
            dtAvg += time[i] - time[i - 1]
        }
        dtAvg /= dtCount

        return -bestLag * dtAvg // positive lag = filtered behind raw
    }

    @JvmStatic
    @Throws(IOException::class)
    fun exportToCsv(outFile: Path, time: List<Double>, raw: List<Double>, filtered: List<Double>) {
        Files.newBufferedWriter(outFile).use { bw: BufferedWriter ->
            bw.write("time,raw,filtered")
            bw.newLine()
            val n = minOf(time.size, raw.size, filtered.size)
            for (i in 0 until n) {
                bw.write("${time[i]},${raw[i]},${filtered[i]}")
                bw.newLine()
            }
        }
    }
}
