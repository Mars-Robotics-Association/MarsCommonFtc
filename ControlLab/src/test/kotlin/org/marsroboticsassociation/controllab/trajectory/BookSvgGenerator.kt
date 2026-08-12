package org.marsroboticsassociation.controllab.trajectory

import java.awt.BasicStroke
import java.awt.Color
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import kotlin.math.min
import org.junit.jupiter.api.Test
import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment
import org.marsroboticsassociation.controllib.motion.SinCurvePosition

/**
 * Generates SVG diagrams for the book chapters on motion profiles.
 *
 * <p>Run with: ./gradlew :ControlLab:test --tests "*BookSvgGenerator"
 */
class BookSvgGenerator {

    companion object {
        private val BOOK_DIR = Path.of("../book")

        private fun runToCompletion(engine: TrajectoryEngine) {
            var limit = 100_000
            while (engine.isMoving() && limit-- > 0) {
                engine.tick()
            }
        }

        private fun writeSvg(path: Path, model: TrajectorySvgModel) {
            Files.createDirectories(path.parent)
            // Platform-native line endings, so the regenerated file matches the git checkout
            // (CRLF on Windows under autocrlf) instead of churning the working tree on every
            // full test run.
            val svg =
                TrajectorySvgExporter.buildSvg(model)
                    .replace("\r\n", "\n")
                    .replace("\n", System.lineSeparator())
            Files.writeString(path, svg, StandardCharsets.UTF_8)
        }

        private fun buildSinModel(profile: SinCurvePosition, target: Double): TrajectorySvgModel {
            val series = ArrayList<TrajectorySvgSeries>()
            series.add(
                TrajectorySvgSeries(
                    "Position (units)",
                    profile.positionSegments(),
                    Color(0x2E, 0x86, 0xDE),
                    2.0f,
                    null,
                )
            )
            series.add(
                TrajectorySvgSeries(
                    "Velocity (units/s)",
                    profile.velocitySegments(),
                    Color(0xE6, 0x7E, 0x22),
                    2.0f,
                    null,
                )
            )
            series.add(
                TrajectorySvgSeries(
                    "Acceleration (units/s\u00b2)",
                    profile.accelerationSegments(),
                    Color(0x27, 0xAE, 0x60),
                    2.0f,
                    null,
                )
            )
            series.add(
                TrajectorySvgSeries(
                    "Target",
                    listOf(
                        PolynomialCurveSegment(0.0, profile.getTotalTime(), target, 0.0, 0.0, 0.0)
                    ),
                    Color(0x95, 0xA5, 0xA6),
                    1.5f,
                    BasicStroke(
                        1.5f,
                        BasicStroke.CAP_BUTT,
                        BasicStroke.JOIN_MITER,
                        10.0f,
                        floatArrayOf(6.0f, 4.0f),
                        0.0f,
                    ),
                )
            )
            var minVal = Double.POSITIVE_INFINITY
            var maxVal = Double.NEGATIVE_INFINITY
            for (s in series) {
                for (seg in s.segments) {
                    minVal = min(minVal, seg.minValue())
                    maxVal = maxOf(maxVal, seg.maxValue())
                }
            }
            return TrajectorySvgModel(
                0.0,
                profile.getTotalTime(),
                if (minVal.isFinite()) minVal else -1.0,
                if (maxVal.isFinite()) maxVal else 1.0,
                series,
            )
        }
    }

    // ---------------------------------------------------------------
    // Chapter 5: Trapezoidal profile (SCurve with very high jMax)
    // ---------------------------------------------------------------

    @Test
    fun generateTrapezoidalProfile() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        // Very high jMax makes the jerk phases negligible → trapezoidal shape
        engine.setPositionParams(
            /* vMax */ 10.0, /* aAccel */
            5.0, /* aDecel */
            5.0, /* jMax */
            100000.0,
        )
        engine.applyParamsAndGoTo(40.0)
        runToCompletion(engine)

        val model = requireNotNull(engine.buildExactSvgModel())
        writeSvg(BOOK_DIR.resolve("trapezoidal-profile.svg"), model)
        engine.dispose()
    }

    // ---------------------------------------------------------------
    // Chapter 6: S-curve position profile
    // ---------------------------------------------------------------

    @Test
    fun generateSCurveProfile() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_POSITION)
        engine.setPositionParams(
            /* vMax */ 10.0, /* aAccel */
            5.0, /* aDecel */
            5.0, /* jMax */
            20.0,
        )
        engine.applyParamsAndGoTo(40.0)
        runToCompletion(engine)

        val model = requireNotNull(engine.buildExactSvgModel())
        writeSvg(BOOK_DIR.resolve("scurve-position-profile.svg"), model)
        engine.dispose()
    }

    @Test
    fun generateSinCurveProfile() {
        val engine = TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION)
        engine.setPositionParams(
            /* vMax */ 10.0, /* aAccel */
            5.0, /* aDecel */
            5.0, /* jMax */
            20.0,
        )
        engine.applyParamsAndGoTo(40.0)
        runToCompletion(engine)

        val model = requireNotNull(engine.buildExactSvgModel())
        writeSvg(BOOK_DIR.resolve("sincurve-position-profile.svg"), model)
        engine.dispose()
    }

    @Test
    fun generateSCurveVelocityProfile() {
        val engine = TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY)
        engine.setVelocityParams(/* aMax */ 50.0, /* jInc */ 200.0, /* jDec */ 200.0)
        engine.applyParamsAndGoTo(400.0)
        runToCompletion(engine)

        val model = requireNotNull(engine.buildExactSvgModel())
        writeSvg(BOOK_DIR.resolve("scurve-velocity-profile.svg"), model)
        engine.dispose()
    }

    // ---------------------------------------------------------------
    // Chapter 7: Sinusoidal trajectory variants
    // ---------------------------------------------------------------

    @Test
    fun generateSinCurveAsymmetricProfile() {
        val engine = TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION)
        // Asymmetric: fast acceleration, slow deceleration
        engine.setPositionParams(
            /* vMax */ 10.0, /* aAccel */
            8.0, /* aDecel */
            3.0, /* jMax */
            20.0,
        )
        engine.applyParamsAndGoTo(40.0)
        runToCompletion(engine)

        val model = requireNotNull(engine.buildExactSvgModel())
        writeSvg(BOOK_DIR.resolve("sincurve-asymmetric-profile.svg"), model)
        engine.dispose()
    }

    @Test
    fun generateSinCurveReversalProfile() {
        // Build directly to set initial velocity (engine API doesn't expose initial state)
        val profile =
            SinCurvePosition(
                0.0,
                50.0, /* v0 */
                -5.0, /* a0 */
                0.0, /* vMax */
                8.0, /* aAccel */
                4.0, /* aDecel */
                4.0, /* jMax */
                12.0,
            )
        val model = buildSinModel(profile, 50.0)
        writeSvg(BOOK_DIR.resolve("sincurve-reversal-profile.svg"), model)
    }
}
