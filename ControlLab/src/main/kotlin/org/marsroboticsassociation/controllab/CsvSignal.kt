package org.marsroboticsassociation.controllab

import com.univocity.parsers.csv.CsvParser
import com.univocity.parsers.csv.CsvParserSettings
import com.univocity.parsers.csv.UnescapedQuoteHandling
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Paths

class CsvSignal private constructor() {
    private val columns = linkedMapOf<String, MutableList<Double>>()
    private var timeKey: String? = null
    private var dataKey: String? = null
    private var windowStart = Double.NEGATIVE_INFINITY
    private var windowEnd = Double.POSITIVE_INFINITY
    var rowCount = 0
        private set

    fun select(timeColumn: String, dataColumn: String): CsvSignal {
        require(timeColumn in columns && dataColumn in columns) {
            "Column not found: $timeColumn or $dataColumn"
        }
        timeKey = timeColumn
        dataKey = dataColumn
        return this
    }

    fun window(start: Double, end: Double): CsvSignal {
        windowStart = start
        windowEnd = end
        return this
    }

    fun time(): List<Double> = slice(column(requireTimeKey()))

    fun data(): List<Double> = slice(column(requireDataKey()))

    private fun slice(src: List<Double>): List<Double> {
        val t = column(requireTimeKey())
        return buildList {
            for (i in src.indices) {
                val tv = t[i]
                if (!tv.isNaN() && tv in windowStart..windowEnd) add(src[i])
            }
        }
    }

    fun minTime(): Double? = timeKey?.let { column(it) }?.filterNot { it.isNaN() }?.minOrNull()

    fun maxTime(): Double? = timeKey?.let { column(it) }?.filterNot { it.isNaN() }?.maxOrNull()

    fun headers(): Set<String> = columns.keys.toSet()

    private fun requireTimeKey(): String = checkNotNull(timeKey) { "time column not selected" }

    private fun requireDataKey(): String = checkNotNull(dataKey) { "data column not selected" }

    private fun column(key: String): MutableList<Double> =
        columns[key] ?: error("unknown column: $key")

    companion object {
        @JvmStatic
        fun load(path: String): CsvSignal {
            val p = Paths.get(path)

            val settings =
                CsvParserSettings().apply {
                    setLineSeparatorDetectionEnabled(true)
                    setDelimiterDetectionEnabled(true, ',', ';', '\t', '|')
                    setQuoteDetectionEnabled(true)
                    setSkipEmptyLines(true)
                    setIgnoreLeadingWhitespaces(true)
                    setIgnoreTrailingWhitespaces(true)
                    setUnescapedQuoteHandling(UnescapedQuoteHandling.STOP_AT_DELIMITER)
                    setNullValue("")
                    setEmptyValue("")
                    format.setComment('\u0000')
                    setHeaderExtractionEnabled(true)
                    maxCharsPerColumn = 1_000_000
                    maxColumns = 10_000
                }

            val parser = CsvParser(settings)
            val sig = CsvSignal()
            Files.newBufferedReader(p, StandardCharsets.UTF_8).use { reader ->
                parser.beginParsing(reader)
                val headers = parser.context.headers()
                for (h in headers) {
                    sig.columns[h] = mutableListOf()
                }

                while (true) {
                    val row = parser.parseNext() ?: break
                    for (i in headers.indices) {
                        val raw = if (i < row.size) row[i] else ""
                        val parsed =
                            if (!raw.isNullOrBlank()) {
                                raw.trim().toDoubleOrNull() ?: Double.NaN
                            } else {
                                Double.NaN
                            }
                        sig.column(headers[i]).add(parsed)
                    }
                    sig.rowCount++
                }
                parser.stopParsing()
            }
            return sig
        }
    }
}
