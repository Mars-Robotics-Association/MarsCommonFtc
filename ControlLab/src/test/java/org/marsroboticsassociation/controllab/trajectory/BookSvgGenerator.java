package org.marsroboticsassociation.controllab.trajectory;

import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;

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
}
