package org.marsroboticsassociation.controllab.trajectory;

import org.junit.jupiter.api.Test;
import org.marsroboticsassociation.controllib.motion.SinCurvePosition;
import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment;
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment;

import java.awt.BasicStroke;
import java.awt.Color;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

/**
 * Generates SVG diagrams for the book chapters on motion profiles.
 *
 * <p>Run with: ./gradlew :ControlLab:test --tests "*BookSvgGenerator"
 */
class BookSvgGenerator {

    private static final Path BOOK_DIR = Path.of("../book");

    // ---------------------------------------------------------------
    // Chapter 5: Trapezoidal profile (SCurve with very high jMax)
    // ---------------------------------------------------------------

    @Test
    void generateTrapezoidalProfile() throws IOException {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        // Very high jMax makes the jerk phases negligible → trapezoidal shape
        engine.setPositionParams(/* vMax */ 10, /* aAccel */ 5, /* aDecel */ 5, /* jMax */ 100000);
        engine.applyParamsAndGoTo(40.0);
        runToCompletion(engine);

        TrajectorySvgModel model = engine.buildExactSvgModel();
        writeSvg(BOOK_DIR.resolve("trapezoidal-profile.svg"), model);
        engine.dispose();
    }

    // ---------------------------------------------------------------
    // Chapter 6: S-curve position profile
    // ---------------------------------------------------------------

    @Test
    void generateSCurveProfile() throws IOException {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_POSITION);
        engine.setPositionParams(/* vMax */ 10, /* aAccel */ 5, /* aDecel */ 5, /* jMax */ 20);
        engine.applyParamsAndGoTo(40.0);
        runToCompletion(engine);

        TrajectorySvgModel model = engine.buildExactSvgModel();
        writeSvg(BOOK_DIR.resolve("scurve-position-profile.svg"), model);
        engine.dispose();
    }

    @Test
    void generateSinCurveProfile() throws IOException {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION);
        engine.setPositionParams(/* vMax */ 10, /* aAccel */ 5, /* aDecel */ 5, /* jMax */ 20);
        engine.applyParamsAndGoTo(40.0);
        runToCompletion(engine);

        TrajectorySvgModel model = engine.buildExactSvgModel();
        writeSvg(BOOK_DIR.resolve("sincurve-position-profile.svg"), model);
        engine.dispose();
    }

    @Test
    void generateSCurveVelocityProfile() throws IOException {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SCURVE_VELOCITY);
        engine.setVelocityParams(/* aMax */ 50, /* jInc */ 200, /* jDec */ 200);
        engine.applyParamsAndGoTo(400.0);
        runToCompletion(engine);

        TrajectorySvgModel model = engine.buildExactSvgModel();
        writeSvg(BOOK_DIR.resolve("scurve-velocity-profile.svg"), model);
        engine.dispose();
    }

    // ---------------------------------------------------------------
    // Chapter 7: Sinusoidal trajectory variants
    // ---------------------------------------------------------------

    @Test
    void generateSinCurveAsymmetricProfile() throws IOException {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION);
        // Asymmetric: fast acceleration, slow deceleration
        engine.setPositionParams(/* vMax */ 10, /* aAccel */ 8, /* aDecel */ 3, /* jMax */ 20);
        engine.applyParamsAndGoTo(40.0);
        runToCompletion(engine);

        TrajectorySvgModel model = engine.buildExactSvgModel();
        writeSvg(BOOK_DIR.resolve("sincurve-asymmetric-profile.svg"), model);
        engine.dispose();
    }

    @Test
    void generateSinCurveReversalProfile() throws IOException {
        // Build directly to set initial velocity (engine API doesn't expose initial state)
        SinCurvePosition profile = new SinCurvePosition(
                0, 50, /* v0 */ -5, /* a0 */ 0, /* vMax */ 8,
                /* aAccel */ 4, /* aDecel */ 4, /* jMax */ 12);
        TrajectorySvgModel model = buildSinModel(profile, 50);
        writeSvg(BOOK_DIR.resolve("sincurve-reversal-profile.svg"), model);
    }

    // ---------------------------------------------------------------
    // Helpers
    // ---------------------------------------------------------------

    private static void runToCompletion(TrajectoryEngine engine) {
        int limit = 100_000;
        while (engine.isMoving() && limit-- > 0) {
            engine.tick();
        }
    }

    private static void writeSvg(Path path, TrajectorySvgModel model) throws IOException {
        Files.createDirectories(path.getParent());
        String svg = TrajectorySvgExporter.buildSvg(model);
        Files.writeString(path, svg, StandardCharsets.UTF_8);
    }

    private static TrajectorySvgModel buildSinModel(SinCurvePosition profile, double target) {
        List<TrajectorySvgSeries> series = new ArrayList<>();
        series.add(new TrajectorySvgSeries(
                "Position (units)", profile.positionSegments(),
                new Color(0x2E, 0x86, 0xDE), 2.0f, null));
        series.add(new TrajectorySvgSeries(
                "Velocity (units/s)", profile.velocitySegments(),
                new Color(0xE6, 0x7E, 0x22), 2.0f, null));
        series.add(new TrajectorySvgSeries(
                "Acceleration (units/s\u00b2)", profile.accelerationSegments(),
                new Color(0x27, 0xAE, 0x60), 2.0f, null));
        series.add(new TrajectorySvgSeries(
                "Target",
                List.of(new PolynomialCurveSegment(0.0, profile.getTotalTime(), target, 0, 0, 0)),
                new Color(0x95, 0xA5, 0xA6), 1.5f,
                new BasicStroke(1.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER,
                        10.0f, new float[]{6.0f, 4.0f}, 0.0f)));
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (var s : series) {
            for (var seg : s.segments()) {
                min = Math.min(min, seg.minValue());
                max = Math.max(max, seg.maxValue());
            }
        }
        return new TrajectorySvgModel(0.0, profile.getTotalTime(),
                Double.isFinite(min) ? min : -1, Double.isFinite(max) ? max : 1, series);
    }
}
