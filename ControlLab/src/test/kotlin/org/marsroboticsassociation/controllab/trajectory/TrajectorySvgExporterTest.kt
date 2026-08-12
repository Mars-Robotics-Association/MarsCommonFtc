package org.marsroboticsassociation.controllab.trajectory

import java.awt.Color
import java.util.regex.Pattern
import kotlin.math.abs
import kotlin.math.max
import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Assertions.fail
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment

class TrajectorySvgExporterTest {

    companion object {
        private const val WIDTH = 900.0
        private const val HEIGHT = 600.0
        private const val LEFT = 72.0
        private const val RIGHT = 24.0
        private const val TOP = 40.0
        private const val BOTTOM = 54.0
        private val PLOT_WIDTH = WIDTH - LEFT - RIGHT
        private val PLOT_HEIGHT = HEIGHT - TOP - BOTTOM

        private fun assertSeriesPathMatchesModel(
            path: String,
            segments: List<TrajectoryCurveSegment>,
            model: TrajectorySvgModel,
            paddedMinY: Double,
            paddedMaxY: Double,
        ) {
            for (cubic in parseCubics(path)) {
                for (u in doubleArrayOf(0.25, 0.5, 0.75)) {
                    val x = cubic.xAt(u)
                    val y = cubic.yAt(u)
                    val time = model.xMin + ((x - LEFT) / PLOT_WIDTH) * (model.xMax - model.xMin)
                    val actual = valueAt(segments, time)
                    val actualY = mapY(actual, paddedMinY, paddedMaxY)
                    assertEquals(
                        actualY,
                        y,
                        0.35,
                        "SVG approximation drifted from the analytic model",
                    )
                }
            }
        }

        private fun valueAt(segments: List<TrajectoryCurveSegment>, time: Double): Double {
            for (segment in segments) {
                if (time >= segment.startTime() - 1e-9 && time <= segment.endTime() + 1e-9) {
                    return segment.valueAt(max(segment.startTime(), minOf(segment.endTime(), time)))
                }
            }
            return fail("No segment covered time $time")
        }

        private fun pathData(svg: String): List<String> {
            val matcher = Pattern.compile("<path d=\"([^\"]+)\"").matcher(svg)
            val paths = ArrayList<String>()
            while (matcher.find()) {
                paths.add(matcher.group(1))
            }
            return paths
        }

        private fun parseOnlyCubic(path: String): Cubic {
            val cubics = parseCubics(path)
            assertEquals(1, cubics.size)
            return cubics[0]
        }

        private fun parseCubics(path: String): List<Cubic> {
            val matcher = Pattern.compile("[A-Z]|-?\\d+(?:\\.\\d+)?").matcher(path)
            val tokens = ArrayList<String>()
            while (matcher.find()) {
                tokens.add(matcher.group())
            }

            val cubics = ArrayList<Cubic>()
            var currentX = Double.NaN
            var currentY = Double.NaN
            var i = 0
            while (i < tokens.size) {
                val token = tokens[i++]
                when (token) {
                    "M" -> {
                        currentX = tokens[i++].toDouble()
                        currentY = tokens[i++].toDouble()
                    }
                    "L" -> {
                        currentX = tokens[i++].toDouble()
                        currentY = tokens[i++].toDouble()
                    }
                    "C" -> {
                        val x1 = tokens[i++].toDouble()
                        val y1 = tokens[i++].toDouble()
                        val x2 = tokens[i++].toDouble()
                        val y2 = tokens[i++].toDouble()
                        val x3 = tokens[i++].toDouble()
                        val y3 = tokens[i++].toDouble()
                        cubics.add(Cubic(currentX, currentY, x1, y1, x2, y2, x3, y3))
                        currentX = x3
                        currentY = y3
                    }
                    else -> fail("Unexpected path token: $token")
                }
            }
            return cubics
        }

        private fun paddedRange(minY: Double, maxY: Double): DoubleArray {
            if (abs(maxY - minY) < 1e-9) {
                val pad = max(1.0, abs(maxY) * 0.1 + 1.0)
                return doubleArrayOf(minY - pad, maxY + pad)
            }
            val pad = (maxY - minY) * 0.08
            return doubleArrayOf(minY - pad, maxY + pad)
        }

        private fun mapY(value: Double, minY: Double, maxY: Double): Double {
            return TOP + PLOT_HEIGHT - ((value - minY) / (maxY - minY)) * PLOT_HEIGHT
        }
    }

    @Test
    fun polynomialSeries_usesOriginalSingleCubicGeometry() {
        val segment = PolynomialCurveSegment(0.0, 1.0, 0.0, 1.0, 0.0, 0.0)
        val model =
            TrajectorySvgModel(
                0.0,
                1.0,
                0.0,
                1.0,
                listOf(TrajectorySvgSeries("Line", listOf(segment), Color.BLACK, 2.0f, null)),
            )

        val svg = TrajectorySvgExporter.buildSvg(model)
        val cubic = parseOnlyCubic(pathData(svg)[0])

        val paddedMinY = -0.08
        val paddedMaxY = 1.08
        assertEquals(LEFT, cubic.x0, 1e-3)
        assertEquals(mapY(0.0, paddedMinY, paddedMaxY), cubic.y0, 1e-3)
        assertEquals(LEFT + PLOT_WIDTH, cubic.x3, 1e-3)
        assertEquals(mapY(1.0, paddedMinY, paddedMaxY), cubic.y3, 1e-3)
        assertEquals(LEFT + PLOT_WIDTH / 3.0, cubic.x1, 1e-3)
        assertEquals(LEFT + 2.0 * PLOT_WIDTH / 3.0, cubic.x2, 1e-3)
        assertEquals(cubic.y0 - PLOT_HEIGHT / 3.0 / (paddedMaxY - paddedMinY), cubic.y1, 1e-3)
        assertEquals(cubic.y3 + PLOT_HEIGHT / 3.0 / (paddedMaxY - paddedMinY), cubic.y2, 1e-3)
    }

    @Test
    fun sinusoidalSeries_staysCloseToAnalyticTrajectoryInSvgSpace() {
        val engine = TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION)
        engine.setPositionParams(10.0, 5.0, 5.0, 50.0)
        engine.applyParamsAndGoTo(100.0)

        val model = requireNotNull(engine.buildExactSvgModel())

        val svg = TrajectorySvgExporter.buildSvg(model)
        val paths = pathData(svg)
        assertEquals(4, paths.size)

        val padded = paddedRange(model.minY, model.maxY)
        assertSeriesPathMatchesModel(
            paths[0],
            model.series[0].segments,
            model,
            padded[0],
            padded[1],
        )
        assertSeriesPathMatchesModel(
            paths[1],
            model.series[1].segments,
            model,
            padded[0],
            padded[1],
        )
        assertSeriesPathMatchesModel(
            paths[2],
            model.series[2].segments,
            model,
            padded[0],
            padded[1],
        )
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
    ) {
        fun xAt(u: Double): Double {
            val inv = 1.0 - u
            return inv * inv * inv * x0 +
                3.0 * inv * inv * u * x1 +
                3.0 * inv * u * u * x2 +
                u * u * u * x3
        }

        fun yAt(u: Double): Double {
            val inv = 1.0 - u
            return inv * inv * inv * y0 +
                3.0 * inv * inv * u * y1 +
                3.0 * inv * u * u * y2 +
                u * u * u * y3
        }
    }
}
