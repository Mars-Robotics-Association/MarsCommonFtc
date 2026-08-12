package org.marsroboticsassociation.controllab

import com.univocity.parsers.csv.CsvParser
import com.univocity.parsers.csv.CsvParserSettings
import com.univocity.parsers.csv.UnescapedQuoteHandling
import java.io.IOException
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Paths
import java.util.Collections
import java.util.LinkedHashMap
import java.util.Objects
import java.util.OptionalDouble

class CsvSignal private constructor() {
    private val columns: LinkedHashMap<String, MutableList<Double>> = LinkedHashMap()
    private var timeKey: String? = null
    private var dataKey: String? = null
    private var windowStart = Double.NEGATIVE_INFINITY
    private var windowEnd = Double.POSITIVE_INFINITY
    private var rowCount = 0

    fun select(timeColumn: String, dataColumn: String): CsvSignal {
        if (!columns.containsKey(timeColumn) || !columns.containsKey(dataColumn)) {
            throw IllegalArgumentException("Column not found: $timeColumn or $dataColumn")
        }
        this.timeKey = timeColumn
        this.dataKey = dataColumn
        return this
    }

    fun window(start: Double, end: Double): CsvSignal {
        this.windowStart = start
        this.windowEnd = end
        return this
    }

    fun time(): List<Double> {
        return slice(columns[Objects.requireNonNull(timeKey)]!!)
    }

    fun data(): List<Double> {
        return slice(columns[Objects.requireNonNull(dataKey)]!!)
    }

    private fun slice(src: List<Double>): List<Double> {
        val key = timeKey ?: throw IllegalStateException("timeKey not set")
        val t = columns[key]!!
        val out = ArrayList<Double>()
        for (i in src.indices) {
            val tv = t[i]
            if (tv.isNaN()) continue
            if (tv >= windowStart && tv <= windowEnd) out.add(src[i])
        }
        return out
    }

    fun minTime(): OptionalDouble {
        return if (columns.containsKey(timeKey)) {
            columns[timeKey]!!.stream().mapToDouble { it }.filter { d -> !d.isNaN() }.min()
        } else {
            OptionalDouble.empty()
        }
    }

    fun maxTime(): OptionalDouble {
        return if (columns.containsKey(timeKey)) {
            columns[timeKey]!!.stream().mapToDouble { it }.filter { d -> !d.isNaN() }.max()
        } else {
            OptionalDouble.empty()
        }
    }

    fun headers(): Set<String> {
        return Collections.unmodifiableSet(columns.keys)
    }

    fun rowCount(): Int {
        return rowCount
    }

    companion object {
        @JvmStatic
        @Throws(IOException::class)
        fun load(path: String): CsvSignal {
            val p = Paths.get(path)

            val settings = CsvParserSettings()
            settings.setLineSeparatorDetectionEnabled(true)
            settings.setDelimiterDetectionEnabled(true, ',', ';', '\t', '|')
            settings.setQuoteDetectionEnabled(true)

            settings.setSkipEmptyLines(true)
            settings.setIgnoreLeadingWhitespaces(true)
            settings.setIgnoreTrailingWhitespaces(true)
            settings.setUnescapedQuoteHandling(UnescapedQuoteHandling.STOP_AT_DELIMITER)
            settings.setNullValue("")
            settings.setEmptyValue("")

            settings.format.setComment('\u0000')

            settings.setHeaderExtractionEnabled(true)

            settings.maxCharsPerColumn = 1_000_000
            settings.maxColumns = 10_000

            val parser = CsvParser(settings)
            val r = Files.newBufferedReader(p, StandardCharsets.UTF_8)
            parser.beginParsing(r)

            val headers = parser.context.headers()
            val sig = CsvSignal()
            for (h in headers) {
                sig.columns[h] = ArrayList()
            }

            while (true) {
                val row = parser.parseNext() ?: break
                for (i in headers.indices) {
                    val h = headers[i]
                    val s = if (i < row.size) row[i] else ""
                    var `val` = Double.NaN
                    if (s != null && s.isNotBlank()) {
                        try {
                            `val` = s.trim().toDouble()
                        } catch (_: NumberFormatException) {
                            `val` = Double.NaN // non-numeric fields (logs etc.)
                        }
                    }
                    sig.columns[h]!!.add(`val`)
                }
                sig.rowCount++
            }

            parser.stopParsing()
            return sig
        }
    }
}
