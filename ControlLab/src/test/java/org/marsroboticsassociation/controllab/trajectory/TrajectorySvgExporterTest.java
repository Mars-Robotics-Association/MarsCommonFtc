package org.marsroboticsassociation.controllab.trajectory;

import static org.junit.jupiter.api.Assertions.*;

import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment;
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment;
import org.junit.jupiter.api.Test;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

class TrajectorySvgExporterTest {

    private static final double WIDTH = 900.0;
    private static final double HEIGHT = 600.0;
    private static final double LEFT = 72.0;
    private static final double RIGHT = 24.0;
    private static final double TOP = 40.0;
    private static final double BOTTOM = 54.0;
    private static final double PLOT_WIDTH = WIDTH - LEFT - RIGHT;
    private static final double PLOT_HEIGHT = HEIGHT - TOP - BOTTOM;

    @Test
    void polynomialSeries_usesOriginalSingleCubicGeometry() {
        PolynomialCurveSegment segment = new PolynomialCurveSegment(0.0, 1.0, 0.0, 1.0, 0.0, 0.0);
        TrajectorySvgModel model =
                new TrajectorySvgModel(
                        0.0,
                        1.0,
                        0.0,
                        1.0,
                        List.of(new TrajectorySvgSeries("Line", List.of(segment), Color.BLACK, 2.0f, null)));

        String svg = TrajectorySvgExporter.buildSvg(model);
        Cubic cubic = parseOnlyCubic(pathData(svg).get(0));

        double paddedMinY = -0.08;
        double paddedMaxY = 1.08;
        assertEquals(LEFT, cubic.x0, 1e-3);
        assertEquals(mapY(0.0, paddedMinY, paddedMaxY), cubic.y0, 1e-3);
        assertEquals(LEFT + PLOT_WIDTH, cubic.x3, 1e-3);
        assertEquals(mapY(1.0, paddedMinY, paddedMaxY), cubic.y3, 1e-3);
        assertEquals(LEFT + PLOT_WIDTH / 3.0, cubic.x1, 1e-3);
        assertEquals(LEFT + 2.0 * PLOT_WIDTH / 3.0, cubic.x2, 1e-3);
        assertEquals(cubic.y0 - PLOT_HEIGHT / 3.0 / (paddedMaxY - paddedMinY), cubic.y1, 1e-3);
        assertEquals(cubic.y3 + PLOT_HEIGHT / 3.0 / (paddedMaxY - paddedMinY), cubic.y2, 1e-3);
    }

    @Test
    void sinusoidalSeries_staysCloseToAnalyticTrajectoryInSvgSpace() {
        TrajectoryEngine engine = new TrajectoryEngine(TrajectoryType.SIN_CURVE_POSITION);
        engine.setPositionParams(10, 5, 5, 50);
        engine.applyParamsAndGoTo(100.0);

        TrajectorySvgModel model = engine.buildExactSvgModel();

        assertNotNull(model);
        String svg = TrajectorySvgExporter.buildSvg(model);
        List<String> paths = pathData(svg);
        assertEquals(4, paths.size());

        double[] paddedRange = paddedRange(model.minY(), model.maxY());
        assertSeriesPathMatchesModel(paths.get(0), model.series().get(0).segments(), model, paddedRange[0], paddedRange[1]);
        assertSeriesPathMatchesModel(paths.get(1), model.series().get(1).segments(), model, paddedRange[0], paddedRange[1]);
        assertSeriesPathMatchesModel(paths.get(2), model.series().get(2).segments(), model, paddedRange[0], paddedRange[1]);
    }

    private static void assertSeriesPathMatchesModel(
            String path,
            List<TrajectoryCurveSegment> segments,
            TrajectorySvgModel model,
            double paddedMinY,
            double paddedMaxY) {
        for (Cubic cubic : parseCubics(path)) {
            for (double u : new double[] {0.25, 0.5, 0.75}) {
                double x = cubic.xAt(u);
                double y = cubic.yAt(u);
                double time = model.xMin() + ((x - LEFT) / PLOT_WIDTH) * (model.xMax() - model.xMin());
                double actual = valueAt(segments, time);
                double actualY = mapY(actual, paddedMinY, paddedMaxY);
                assertEquals(actualY, y, 0.35, "SVG approximation drifted from the analytic model");
            }
        }
    }

    private static double valueAt(List<TrajectoryCurveSegment> segments, double time) {
        for (TrajectoryCurveSegment segment : segments) {
            if (time >= segment.startTime() - 1e-9 && time <= segment.endTime() + 1e-9) {
                return segment.valueAt(Math.max(segment.startTime(), Math.min(segment.endTime(), time)));
            }
        }
        fail("No segment covered time " + time);
        return Double.NaN;
    }

    private static List<String> pathData(String svg) {
        Matcher matcher = Pattern.compile("<path d=\"([^\"]+)\"").matcher(svg);
        List<String> paths = new ArrayList<>();
        while (matcher.find()) {
            paths.add(matcher.group(1));
        }
        return paths;
    }

    private static Cubic parseOnlyCubic(String path) {
        List<Cubic> cubics = parseCubics(path);
        assertEquals(1, cubics.size());
        return cubics.get(0);
    }

    private static List<Cubic> parseCubics(String path) {
        Matcher matcher = Pattern.compile("[A-Z]|-?\\d+(?:\\.\\d+)?").matcher(path);
        List<String> tokens = new ArrayList<>();
        while (matcher.find()) {
            tokens.add(matcher.group());
        }

        List<Cubic> cubics = new ArrayList<>();
        double currentX = Double.NaN;
        double currentY = Double.NaN;
        int i = 0;
        while (i < tokens.size()) {
            String token = tokens.get(i++);
            switch (token) {
                case "M" -> {
                    currentX = Double.parseDouble(tokens.get(i++));
                    currentY = Double.parseDouble(tokens.get(i++));
                }
                case "L" -> {
                    currentX = Double.parseDouble(tokens.get(i++));
                    currentY = Double.parseDouble(tokens.get(i++));
                }
                case "C" -> {
                    double x1 = Double.parseDouble(tokens.get(i++));
                    double y1 = Double.parseDouble(tokens.get(i++));
                    double x2 = Double.parseDouble(tokens.get(i++));
                    double y2 = Double.parseDouble(tokens.get(i++));
                    double x3 = Double.parseDouble(tokens.get(i++));
                    double y3 = Double.parseDouble(tokens.get(i++));
                    cubics.add(new Cubic(currentX, currentY, x1, y1, x2, y2, x3, y3));
                    currentX = x3;
                    currentY = y3;
                }
                default -> fail("Unexpected path token: " + token);
            }
        }
        return cubics;
    }

    private static double[] paddedRange(double minY, double maxY) {
        if (Math.abs(maxY - minY) < 1e-9) {
            double pad = Math.max(1.0, Math.abs(maxY) * 0.1 + 1.0);
            return new double[] {minY - pad, maxY + pad};
        }
        double pad = (maxY - minY) * 0.08;
        return new double[] {minY - pad, maxY + pad};
    }

    private static double mapY(double value, double minY, double maxY) {
        return TOP + PLOT_HEIGHT - ((value - minY) / (maxY - minY)) * PLOT_HEIGHT;
    }

    private record Cubic(double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3) {
        double xAt(double u) {
            double inv = 1.0 - u;
            return inv * inv * inv * x0
                    + 3.0 * inv * inv * u * x1
                    + 3.0 * inv * u * u * x2
                    + u * u * u * x3;
        }

        double yAt(double u) {
            double inv = 1.0 - u;
            return inv * inv * inv * y0
                    + 3.0 * inv * inv * u * y1
                    + 3.0 * inv * u * u * y2
                    + u * u * u * y3;
        }
    }
}
