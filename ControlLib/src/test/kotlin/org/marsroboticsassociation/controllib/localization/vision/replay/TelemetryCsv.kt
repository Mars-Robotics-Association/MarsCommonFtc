package org.marsroboticsassociation.controllib.localization.vision.replay

import java.io.BufferedReader
import java.io.IOException
import java.io.InputStreamReader
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.util.regex.Pattern

/**
 * A header-indexed reader for the bank-localizer telemetry CSVs. It keys every value by *column
 * name* (never position) and yields [Double.NaN] for a column absent in an older file, so the
 * replay harness tolerates schema drift across runs.
 *
 * <p>Reads both plain `.csv` and brotli-compressed `.csv.br` recordings transparently: a `.br` path
 * is streamed through the `brotli` CLI (override with `-Dbrotli.bin=`), so a recording compressed
 * for storage still replays. Plain files need no external tool.
 */
class TelemetryCsv
private constructor(
    private val columns: List<String>,
    private val rows: List<DoubleArray>,
) {
    private val index = HashMap<String, Int>()

    init {
        for (i in columns.indices) {
            index[columns[i]] = i
        }
    }

    fun size(): Int = rows.size

    fun columns(): List<String> = columns

    fun has(col: String): Boolean = index.containsKey(col)

    /** Value of [col] in [row], or NaN if the column is absent. */
    fun get(row: Int, col: String): Double {
        val i = index[col] ?: return Double.NaN
        return rows[row][i]
    }

    companion object {
        /**
         * Java `String.split(",", -1)` — keeps trailing empty fields (Kotlin Regex.split rejects
         * limit -1).
         */
        private val COMMA = Pattern.compile(",", Pattern.LITERAL)

        private fun splitCsv(line: String): Array<String> = COMMA.split(line, -1)

        @Throws(IOException::class)
        fun read(path: Path): TelemetryCsv {
            open(path).use { r ->
                val headerLine = r.readLine() ?: throw IOException("empty CSV: $path")
                val cols = ArrayList<String>()
                for (c in splitCsv(headerLine)) {
                    cols.add(c.trim())
                }
                val data = ArrayList<DoubleArray>()
                var line: String?
                while (r.readLine().also { line = it } != null) {
                    if (line!!.isEmpty()) {
                        continue
                    }
                    val parts = splitCsv(line!!)
                    val vals = DoubleArray(cols.size)
                    for (i in cols.indices) {
                        vals[i] = if (i < parts.size) parseDouble(parts[i]) else Double.NaN
                    }
                    data.add(vals)
                }
                return TelemetryCsv(cols, data)
            }
        }

        @Throws(IOException::class)
        private fun open(path: Path): BufferedReader {
            if (!path.fileName.toString().endsWith(".br")) {
                return Files.newBufferedReader(path)
            }
            val brotli = System.getProperty("brotli.bin", "brotli")
            val p =
                ProcessBuilder(brotli, "-dc", path.toAbsolutePath().toString())
                    .redirectError(ProcessBuilder.Redirect.INHERIT)
                    .start()
            return BufferedReader(InputStreamReader(p.inputStream, StandardCharsets.UTF_8))
        }

        /** Parses a cell; a non-numeric token or a blank becomes NaN. */
        private fun parseDouble(s: String): Double {
            val t = s.trim()
            if (t.isEmpty()) {
                return Double.NaN
            }
            return try {
                t.toDouble()
            } catch (_: NumberFormatException) {
                Double.NaN
            }
        }
    }
}
