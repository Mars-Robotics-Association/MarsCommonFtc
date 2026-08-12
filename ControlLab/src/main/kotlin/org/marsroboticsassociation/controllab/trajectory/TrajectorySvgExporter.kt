package org.marsroboticsassociation.controllab.trajectory

import java.awt.BasicStroke
import java.awt.Color
import java.io.IOException
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.text.DecimalFormat
import java.text.DecimalFormatSymbols
import java.util.Locale
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.max
import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment

class TrajectorySvgExporter private constructor() {

    companion object {
        private const val WIDTH = 900
        private const val HEIGHT = 600
        private const val LEFT = 72
        private const val RIGHT = 24
        private const val TOP = 40
        private const val BOTTOM = 54
        private const val LEGEND_WIDTH = 170.0
        private const val APPROXIMATION_TOLERANCE_PX = 0.25
        private const val MAX_APPROXIMATION_DEPTH = 12
        private val DECIMAL = DecimalFormat("0.###", DecimalFormatSymbols.getInstance(Locale.US))

        @JvmStatic
        @Throws(IOException::class)
        fun export(path: Path, model: TrajectorySvgModel) {
            val svg = buildSvg(model)
            Files.writeString(path, svg, StandardCharsets.UTF_8)
        }

        @JvmStatic
        fun buildSvg(model: TrajectorySvgModel): String {
            val xMin = model.xMin
            val xMax = model.xMax
            val xRange = max(xMax - xMin, 1e-9)
            var minY = model.minY
            var maxY = model.maxY
            if (abs(maxY - minY) < 1e-9) {
                val pad = max(1.0, abs(maxY) * 0.1 + 1.0)
                minY -= pad
                maxY += pad
            } else {
                val pad = (maxY - minY) * 0.08
                minY -= pad
                maxY += pad
            }

            val plotWidth = (WIDTH - LEFT - RIGHT).toDouble()
            val plotHeight = (HEIGHT - TOP - BOTTOM).toDouble()

            val out = StringBuilder(8192)
            out.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n")
            out.append("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"")
                .append(WIDTH)
                .append("\" height=\"")
                .append(HEIGHT)
                .append("\" viewBox=\"0 0 ")
                .append(WIDTH)
                .append(' ')
                .append(HEIGHT)
                .append("\">\n")
            out.append("  <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n")
            out.append("  <text x=\"")
                .append(WIDTH / 2)
                .append(
                    "\" y=\"24\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"18\">Trajectory</text>\n"
                )

            appendAxes(out, xMin, xMax, xRange, minY, maxY, plotWidth, plotHeight)
            val legend = chooseLegendBox(model, xMin, xRange, minY, maxY, plotWidth, plotHeight)
            appendLegend(out, model, legend)

            for (series in model.series) {
                if (series.segments.isEmpty()) continue
                out.append("  <path d=\"")
                appendSeriesPath(
                    out,
                    series.segments,
                    xMin,
                    xRange,
                    minY,
                    maxY,
                    plotWidth,
                    plotHeight,
                )
                out.append("\" fill=\"none\" stroke=\"")
                    .append(color(series.color))
                    .append("\" stroke-width=\"")
                    .append(DECIMAL.format(series.strokeWidth.toDouble()))
                    .append("\"")
                if (series.stroke != null) {
                    out.append(" stroke-dasharray=\"")
                        .append(strokeDashArray(series.stroke))
                        .append("\"")
                }
                out.append("/>\n")
            }

            out.append("</svg>\n")
            return out.toString()
        }

        private fun appendAxes(
            out: StringBuilder,
            xMin: Double,
            xMax: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ) {
            val y0 = TOP + plotHeight

            out.append(
                "  <g font-family=\"sans-serif\" font-size=\"11\" fill=\"#444\" stroke=\"#d0d0d0\" stroke-width=\"1\">\n"
            )
            out.append("    <rect x=\"")
                .append(LEFT)
                .append("\" y=\"")
                .append(TOP)
                .append("\" width=\"")
                .append(DECIMAL.format(plotWidth))
                .append("\" height=\"")
                .append(DECIMAL.format(plotHeight))
                .append("\" fill=\"none\" stroke=\"#bdbdbd\"/>\n")

            for (i in 0..5) {
                val t = xMin + xRange * i / 5.0
                val x = mapX(t, xMin, xRange, plotWidth)
                out.append("    <line x1=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y1=\"")
                    .append(TOP)
                    .append("\" x2=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y2=\"")
                    .append(DECIMAL.format(y0))
                    .append("\"/>\n")
                out.append("    <text x=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y=\"")
                    .append(HEIGHT - 18)
                    .append("\" text-anchor=\"middle\" stroke=\"none\">")
                    .append(DECIMAL.format(t))
                    .append("</text>\n")
            }

            for (i in 0..5) {
                val v = minY + (maxY - minY) * i / 5.0
                val y = mapY(v, minY, maxY, plotHeight)
                out.append("    <line x1=\"")
                    .append(LEFT)
                    .append("\" y1=\"")
                    .append(DECIMAL.format(y))
                    .append("\" x2=\"")
                    .append(DECIMAL.format(LEFT + plotWidth))
                    .append("\" y2=\"")
                    .append(DECIMAL.format(y))
                    .append("\"/>\n")
                out.append("    <text x=\"")
                    .append(LEFT - 8)
                    .append("\" y=\"")
                    .append(DECIMAL.format(y + 4))
                    .append("\" text-anchor=\"end\" stroke=\"none\">")
                    .append(DECIMAL.format(v))
                    .append("</text>\n")
            }

            out.append("  </g>\n")
            out.append("  <text x=\"")
                .append(LEFT + plotWidth / 2.0)
                .append("\" y=\"")
                .append(HEIGHT - 4)
                .append(
                    "\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\">Time (s)</text>\n"
                )
            out.append("  <text x=\"18\" y=\"")
                .append(TOP + plotHeight / 2.0)
                .append(
                    "\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" transform=\"rotate(-90 18 "
                )
                .append(DECIMAL.format(TOP + plotHeight / 2.0))
                .append(")\">Value</text>\n")
        }

        private fun appendLegend(out: StringBuilder, model: TrajectorySvgModel, legend: LegendBox) {
            var x = legend.lineX()
            var y = legend.firstLineY()
            out.append("  <g font-family=\"sans-serif\" font-size=\"12\">\n")
            out.append("    <rect x=\"")
                .append(DECIMAL.format(legend.x))
                .append("\" y=\"")
                .append(DECIMAL.format(legend.y))
                .append("\" width=\"170\" height=\"")
                .append(DECIMAL.format(legend.height))
                .append("\" fill=\"white\" fill-opacity=\"0.9\" stroke=\"#c8c8c8\"/>\n")
            for (series in model.series) {
                out.append("    <line x1=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y1=\"")
                    .append(DECIMAL.format(y))
                    .append("\" x2=\"")
                    .append(DECIMAL.format(x + 24))
                    .append("\" y2=\"")
                    .append(DECIMAL.format(y))
                    .append("\" stroke=\"")
                    .append(color(series.color))
                    .append("\" stroke-width=\"")
                    .append(DECIMAL.format(series.strokeWidth.toDouble()))
                    .append("\"")
                if (series.stroke != null) {
                    out.append(" stroke-dasharray=\"")
                        .append(strokeDashArray(series.stroke))
                        .append("\"")
                }
                out.append("/>\n")
                out.append("    <text x=\"")
                    .append(DECIMAL.format(x + 32))
                    .append("\" y=\"")
                    .append(DECIMAL.format(y + 4))
                    .append("\" fill=\"#333\">")
                    .append(escape(series.label))
                    .append("</text>\n")
                y += 20
            }
            out.append("  </g>\n")
        }

        private fun chooseLegendBox(
            model: TrajectorySvgModel,
            xMin: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ): LegendBox {
            val height = model.series.size * 20.0 + 8.0
            val candidates =
                arrayOf(
                    doubleArrayOf(LEFT + 8.0, TOP + 8.0),
                    doubleArrayOf(LEFT + plotWidth - LEGEND_WIDTH - 8.0, TOP + 8.0),
                    doubleArrayOf(LEFT + 8.0, TOP + plotHeight - height - 8.0),
                    doubleArrayOf(
                        LEFT + plotWidth - LEGEND_WIDTH - 8.0,
                        TOP + plotHeight - height - 8.0,
                    ),
                )

            var best: LegendBox? = null
            var bestScore = Double.POSITIVE_INFINITY
            for (candidate in candidates) {
                val box = LegendBox(candidate[0], candidate[1], LEGEND_WIDTH, height)
                val score =
                    scoreLegendBox(box, model, xMin, xRange, minY, maxY, plotWidth, plotHeight)
                if (score < bestScore) {
                    bestScore = score
                    best = box
                }
            }
            return best ?: LegendBox(LEFT + 8.0, TOP + 8.0, LEGEND_WIDTH, height)
        }

        private fun scoreLegendBox(
            box: LegendBox,
            model: TrajectorySvgModel,
            xMin: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ): Double {
            var score = 0.0
            for (series in model.series) {
                for (segment in series.segments) {
                    val duration = segment.duration()
                    if (duration <= 0) continue
                    val samples = max(6, ceil(duration * 12.0).toInt())
                    for (i in 0..samples) {
                        val t = segment.startTime() + duration * i / samples
                        val x = mapX(t, xMin, xRange, plotWidth)
                        val y = mapY(segment.valueAt(t), minY, maxY, plotHeight)
                        if (box.contains(x, y)) {
                            score += 1.0
                        } else {
                            val dx = box.distanceX(x)
                            val dy = box.distanceY(y)
                            val distanceSq = dx * dx + dy * dy
                            if (distanceSq < 900.0) {
                                score += 0.15
                            }
                        }
                    }
                }
            }
            return score
        }

        private fun appendSeriesPath(
            out: StringBuilder,
            segments: List<TrajectoryCurveSegment>,
            xMin: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ) {
            if (allPolynomialSegments(segments)) {
                appendPolynomialSeriesPath(
                    out,
                    segments.map { it as PolynomialCurveSegment },
                    xMin,
                    xRange,
                    minY,
                    maxY,
                    plotWidth,
                    plotHeight,
                )
                return
            }

            var moved = false
            var prevEndTime = Double.NaN
            var prevEndValue = Double.NaN
            for (segment in segments) {
                if (segment.duration() <= 0) continue
                val startValue = segment.valueAt(segment.startTime())
                val x0 = mapX(segment.startTime(), xMin, xRange, plotWidth)
                val y0 = mapY(startValue, minY, maxY, plotHeight)
                val contiguous = moved && abs(segment.startTime() - prevEndTime) < 1e-9
                val sameValue = contiguous && abs(startValue - prevEndValue) < 1e-9
                if (!moved || !contiguous) {
                    out.append("M ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ')
                    moved = true
                } else if (!sameValue) {
                    out.append("L ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ')
                }
                appendApproximatedSegment(
                    out,
                    segment,
                    xMin,
                    xRange,
                    minY,
                    maxY,
                    plotWidth,
                    plotHeight,
                    0,
                )
                prevEndTime = segment.endTime()
                prevEndValue = segment.valueAt(segment.endTime())
            }
        }

        private fun appendPolynomialSeriesPath(
            out: StringBuilder,
            segments: List<PolynomialCurveSegment>,
            xMin: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ) {
            var moved = false
            var prevEndTime = Double.NaN
            var prevEndValue = Double.NaN
            for (segment in segments) {
                if (segment.duration() <= 0) continue
                val startValue = segment.valueAt(segment.startTime())
                val endValue = segment.valueAt(segment.endTime())
                val x0 = mapX(segment.startTime(), xMin, xRange, plotWidth)
                val y0 = mapY(startValue, minY, maxY, plotHeight)
                val x3 = mapX(segment.endTime(), xMin, xRange, plotWidth)
                val y3 = mapY(endValue, minY, maxY, plotHeight)
                val m0 = segment.slopeAt(segment.startTime())
                val m1 = segment.slopeAt(segment.endTime())
                val dx = x3 - x0
                val y1 = y0 - scaleSlope(m0, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0
                val y2 = y3 + scaleSlope(m1, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0
                val x1 = x0 + dx / 3.0
                val x2 = x3 - dx / 3.0
                val contiguous = moved && abs(segment.startTime() - prevEndTime) < 1e-9
                val sameValue = contiguous && abs(startValue - prevEndValue) < 1e-9
                if (!moved || !contiguous) {
                    out.append("M ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ')
                    moved = true
                } else if (!sameValue) {
                    out.append("L ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ')
                }
                out.append("C ")
                    .append(DECIMAL.format(x1))
                    .append(' ')
                    .append(DECIMAL.format(y1))
                    .append(' ')
                    .append(DECIMAL.format(x2))
                    .append(' ')
                    .append(DECIMAL.format(y2))
                    .append(' ')
                    .append(DECIMAL.format(x3))
                    .append(' ')
                    .append(DECIMAL.format(y3))
                    .append(' ')
                prevEndTime = segment.endTime()
                prevEndValue = endValue
            }
        }

        private fun allPolynomialSegments(segments: List<TrajectoryCurveSegment>): Boolean {
            for (segment in segments) {
                if (segment !is PolynomialCurveSegment) {
                    return false
                }
            }
            return true
        }

        private fun appendApproximatedSegment(
            out: StringBuilder,
            segment: TrajectoryCurveSegment,
            xMin: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
            depth: Int,
        ) {
            val cubic =
                cubicForSegment(
                    segment,
                    segment.startTime(),
                    segment.endTime(),
                    xMin,
                    xRange,
                    minY,
                    maxY,
                    plotWidth,
                    plotHeight,
                )
            if (
                depth < MAX_APPROXIMATION_DEPTH &&
                    exceedsApproximationTolerance(
                        cubic,
                        segment,
                        minY,
                        maxY,
                        plotHeight,
                        APPROXIMATION_TOLERANCE_PX,
                    )
            ) {
                val mid = (segment.startTime() + segment.endTime()) * 0.5
                if (mid > segment.startTime() + 1e-12 && mid < segment.endTime() - 1e-12) {
                    appendApproximatedSegment(
                        out,
                        segment.clippedTo(segment.startTime(), mid)!!,
                        xMin,
                        xRange,
                        minY,
                        maxY,
                        plotWidth,
                        plotHeight,
                        depth + 1,
                    )
                    appendApproximatedSegment(
                        out,
                        segment.clippedTo(mid, segment.endTime())!!,
                        xMin,
                        xRange,
                        minY,
                        maxY,
                        plotWidth,
                        plotHeight,
                        depth + 1,
                    )
                    return
                }
            }

            out.append("C ")
                .append(DECIMAL.format(cubic.x1))
                .append(' ')
                .append(DECIMAL.format(cubic.y1))
                .append(' ')
                .append(DECIMAL.format(cubic.x2))
                .append(' ')
                .append(DECIMAL.format(cubic.y2))
                .append(' ')
                .append(DECIMAL.format(cubic.x3))
                .append(' ')
                .append(DECIMAL.format(cubic.y3))
                .append(' ')
        }

        private fun cubicForSegment(
            segment: TrajectoryCurveSegment,
            startTime: Double,
            endTime: Double,
            xMin: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ): Cubic {
            val startValue = segment.valueAt(startTime)
            val endValue = segment.valueAt(endTime)
            val x0 = mapX(startTime, xMin, xRange, plotWidth)
            val y0 = mapY(startValue, minY, maxY, plotHeight)
            val x3 = mapX(endTime, xMin, xRange, plotWidth)
            val y3 = mapY(endValue, minY, maxY, plotHeight)
            val m0 = segment.slopeAt(startTime)
            val m1 = segment.slopeAt(endTime)
            val dx = x3 - x0
            val x1 = x0 + dx / 3.0
            val x2 = x3 - dx / 3.0
            val y1 = y0 - scaleSlope(m0, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0
            val y2 = y3 + scaleSlope(m1, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0
            return Cubic(x0, y0, x1, y1, x2, y2, x3, y3, startTime, endTime)
        }

        private fun exceedsApproximationTolerance(
            cubic: Cubic,
            segment: TrajectoryCurveSegment,
            minY: Double,
            maxY: Double,
            plotHeight: Double,
            tolerancePx: Double,
        ): Boolean {
            val samples = doubleArrayOf(0.25, 0.5, 0.75)
            for (u in samples) {
                val time = cubic.startTime + (cubic.endTime - cubic.startTime) * u
                val actualY = mapY(segment.valueAt(time), minY, maxY, plotHeight)
                val cubicY = cubic.yAt(u)
                if (abs(actualY - cubicY) > tolerancePx) {
                    return true
                }
            }
            return false
        }

        private fun mapX(t: Double, xMin: Double, xRange: Double, plotWidth: Double): Double {
            return LEFT + ((t - xMin) / xRange) * plotWidth
        }

        private fun mapY(value: Double, minY: Double, maxY: Double, plotHeight: Double): Double {
            return TOP + plotHeight - ((value - minY) / (maxY - minY)) * plotHeight
        }

        private fun scaleSlope(
            slope: Double,
            xRange: Double,
            minY: Double,
            maxY: Double,
            plotWidth: Double,
            plotHeight: Double,
        ): Double {
            return slope * (xRange / plotWidth) * (plotHeight / (maxY - minY))
        }

        private fun color(color: Color): String {
            return String.format("#%02x%02x%02x", color.red, color.green, color.blue)
        }

        private fun strokeDashArray(stroke: BasicStroke): String {
            val dash = stroke.dashArray
            if (dash == null || dash.isEmpty()) return ""
            val out = StringBuilder()
            for (i in dash.indices) {
                if (i > 0) out.append(',')
                out.append(DECIMAL.format(dash[i].toDouble()))
            }
            return out.toString()
        }

        private fun escape(text: String): String {
            return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
        }

        private data class LegendBox(
            val x: Double,
            val y: Double,
            val width: Double,
            val height: Double,
        ) {
            fun lineX(): Double = x + 10.0

            fun firstLineY(): Double = y + 14.0

            fun contains(px: Double, py: Double): Boolean {
                return px >= x && px <= x + width && py >= y && py <= y + height
            }

            fun distanceX(px: Double): Double {
                if (px < x) return x - px
                if (px > x + width) return px - (x + width)
                return 0.0
            }

            fun distanceY(py: Double): Double {
                if (py < y) return y - py
                if (py > y + height) return py - (y + height)
                return 0.0
            }
        }

        private data class Cubic(
            val x0: Double,
            val y0: Double,
            val x1: Double,
            val y1: Double,
            val x2: Double,
            val y2: Double,
            val x3: Double,
            val y3: Double,
            val startTime: Double,
            val endTime: Double,
        ) {
            fun yAt(u: Double): Double {
                val inv = 1.0 - u
                return inv * inv * inv * y0 +
                    3.0 * inv * inv * u * y1 +
                    3.0 * inv * u * u * y2 +
                    u * u * u * y3
            }
        }
    }
}
