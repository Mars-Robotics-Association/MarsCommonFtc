package org.marsroboticsassociation.controllab.trajectory;

import org.marsroboticsassociation.controllib.motion.PolynomialCurveSegment;
import org.marsroboticsassociation.controllib.motion.TrajectoryCurveSegment;

import java.awt.Color;
import java.awt.BasicStroke;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.util.List;
import java.util.Locale;

final class TrajectorySvgExporter {

    private static final int WIDTH = 900;
    private static final int HEIGHT = 600;
    private static final int LEFT = 72;
    private static final int RIGHT = 24;
    private static final int TOP = 40;
    private static final int BOTTOM = 54;
    private static final double LEGEND_WIDTH = 170.0;
    private static final double APPROXIMATION_TOLERANCE_PX = 0.25;
    private static final int MAX_APPROXIMATION_DEPTH = 12;
    private static final DecimalFormat DECIMAL =
            new DecimalFormat("0.###", DecimalFormatSymbols.getInstance(Locale.US));

    private TrajectorySvgExporter() {}

    static void export(Path path, TrajectorySvgModel model) throws IOException {
        String svg = buildSvg(model);
        Files.writeString(path, svg, StandardCharsets.UTF_8);
    }

    static String buildSvg(TrajectorySvgModel model) {
        double xMin = model.xMin();
        double xMax = model.xMax();
        double xRange = Math.max(xMax - xMin, 1e-9);
        double minY = model.minY();
        double maxY = model.maxY();
        if (Math.abs(maxY - minY) < 1e-9) {
            double pad = Math.max(1.0, Math.abs(maxY) * 0.1 + 1.0);
            minY -= pad;
            maxY += pad;
        } else {
            double pad = (maxY - minY) * 0.08;
            minY -= pad;
            maxY += pad;
        }

        double plotWidth = WIDTH - LEFT - RIGHT;
        double plotHeight = HEIGHT - TOP - BOTTOM;

        StringBuilder out = new StringBuilder(8192);
        out.append("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        out.append("<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"")
                .append(WIDTH)
                .append("\" height=\"")
                .append(HEIGHT)
                .append("\" viewBox=\"0 0 ")
                .append(WIDTH)
                .append(' ')
                .append(HEIGHT)
                .append("\">\n");
        out.append("  <rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n");
        out.append("  <text x=\"")
                .append(WIDTH / 2)
                .append("\" y=\"24\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"18\">Trajectory</text>\n");

        appendAxes(out, xMin, xMax, xRange, minY, maxY, plotWidth, plotHeight);
        LegendBox legend =
                chooseLegendBox(model, xMin, xRange, minY, maxY, plotWidth, plotHeight);
        appendLegend(out, model, legend);

        for (TrajectorySvgSeries series : model.series()) {
            if (series.segments().isEmpty()) continue;
            out.append("  <path d=\"");
            appendSeriesPath(out, series.segments(), xMin, xRange, minY, maxY, plotWidth, plotHeight);
            out.append("\" fill=\"none\" stroke=\"")
                    .append(color(series.color()))
                    .append("\" stroke-width=\"")
                    .append(DECIMAL.format(series.strokeWidth()))
                    .append("\"");
            if (series.stroke() != null) {
                out.append(" stroke-dasharray=\"")
                        .append(strokeDashArray(series.stroke()))
                        .append("\"");
            }
            out.append("/>\n");
        }

        out.append("</svg>\n");
        return out.toString();
    }

    private static void appendAxes(
            StringBuilder out,
            double xMin,
            double xMax,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        double x0 = LEFT;
        double y0 = TOP + plotHeight;

        out.append("  <g font-family=\"sans-serif\" font-size=\"11\" fill=\"#444\" stroke=\"#d0d0d0\" stroke-width=\"1\">\n");
        out.append("    <rect x=\"")
                .append(LEFT)
                .append("\" y=\"")
                .append(TOP)
                .append("\" width=\"")
                .append(DECIMAL.format(plotWidth))
                .append("\" height=\"")
                .append(DECIMAL.format(plotHeight))
                .append("\" fill=\"none\" stroke=\"#bdbdbd\"/>\n");

        for (int i = 0; i <= 5; i++) {
            double t = xMin + xRange * i / 5.0;
            double x = mapX(t, xMin, xRange, plotWidth);
            out.append("    <line x1=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y1=\"")
                    .append(TOP)
                    .append("\" x2=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y2=\"")
                    .append(DECIMAL.format(y0))
                    .append("\"/>\n");
            out.append("    <text x=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y=\"")
                    .append(HEIGHT - 18)
                    .append("\" text-anchor=\"middle\" stroke=\"none\">")
                    .append(DECIMAL.format(t))
                    .append("</text>\n");
        }

        for (int i = 0; i <= 5; i++) {
            double v = minY + (maxY - minY) * i / 5.0;
            double y = mapY(v, minY, maxY, plotHeight);
            out.append("    <line x1=\"")
                    .append(LEFT)
                    .append("\" y1=\"")
                    .append(DECIMAL.format(y))
                    .append("\" x2=\"")
                    .append(DECIMAL.format(LEFT + plotWidth))
                    .append("\" y2=\"")
                    .append(DECIMAL.format(y))
                    .append("\"/>\n");
            out.append("    <text x=\"")
                    .append(LEFT - 8)
                    .append("\" y=\"")
                    .append(DECIMAL.format(y + 4))
                    .append("\" text-anchor=\"end\" stroke=\"none\">")
                    .append(DECIMAL.format(v))
                    .append("</text>\n");
        }

        out.append("  </g>\n");
        out.append("  <text x=\"")
                .append(LEFT + plotWidth / 2.0)
                .append("\" y=\"")
                .append(HEIGHT - 4)
                .append("\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\">Time (s)</text>\n");
        out.append("  <text x=\"18\" y=\"")
                .append(TOP + plotHeight / 2.0)
                .append("\" text-anchor=\"middle\" font-family=\"sans-serif\" font-size=\"12\" transform=\"rotate(-90 18 ")
                .append(DECIMAL.format(TOP + plotHeight / 2.0))
                .append(")\">Value</text>\n");
    }

    private static void appendLegend(StringBuilder out, TrajectorySvgModel model, LegendBox legend) {
        double x = legend.lineX();
        double y = legend.firstLineY();
        out.append("  <g font-family=\"sans-serif\" font-size=\"12\">\n");
        out.append("    <rect x=\"")
                .append(DECIMAL.format(legend.x()))
                .append("\" y=\"")
                .append(DECIMAL.format(legend.y()))
                .append("\" width=\"170\" height=\"")
                .append(DECIMAL.format(legend.height()))
                .append("\" fill=\"white\" fill-opacity=\"0.9\" stroke=\"#c8c8c8\"/>\n");
        for (TrajectorySvgSeries series : model.series()) {
            out.append("    <line x1=\"")
                    .append(DECIMAL.format(x))
                    .append("\" y1=\"")
                    .append(DECIMAL.format(y))
                    .append("\" x2=\"")
                    .append(DECIMAL.format(x + 24))
                    .append("\" y2=\"")
                    .append(DECIMAL.format(y))
                    .append("\" stroke=\"")
                    .append(color(series.color()))
                    .append("\" stroke-width=\"")
                    .append(DECIMAL.format(series.strokeWidth()))
                    .append("\"");
            if (series.stroke() != null) {
                out.append(" stroke-dasharray=\"").append(strokeDashArray(series.stroke())).append("\"");
            }
            out.append("/>\n");
            out.append("    <text x=\"")
                    .append(DECIMAL.format(x + 32))
                    .append("\" y=\"")
                    .append(DECIMAL.format(y + 4))
                    .append("\" fill=\"#333\">")
                    .append(escape(series.label()))
                    .append("</text>\n");
            y += 20;
        }
        out.append("  </g>\n");
    }

    private static LegendBox chooseLegendBox(
            TrajectorySvgModel model,
            double xMin,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        double height = model.series().size() * 20.0 + 8.0;
        double[][] candidates = {
            {LEFT + 8.0, TOP + 8.0},
            {LEFT + plotWidth - LEGEND_WIDTH - 8.0, TOP + 8.0},
            {LEFT + 8.0, TOP + plotHeight - height - 8.0},
            {LEFT + plotWidth - LEGEND_WIDTH - 8.0, TOP + plotHeight - height - 8.0}
        };

        LegendBox best = null;
        double bestScore = Double.POSITIVE_INFINITY;
        for (double[] candidate : candidates) {
            LegendBox box = new LegendBox(candidate[0], candidate[1], LEGEND_WIDTH, height);
            double score = scoreLegendBox(box, model, xMin, xRange, minY, maxY, plotWidth, plotHeight);
            if (score < bestScore) {
                bestScore = score;
                best = box;
            }
        }
        return best != null ? best : new LegendBox(LEFT + 8.0, TOP + 8.0, LEGEND_WIDTH, height);
    }

    private static double scoreLegendBox(
            LegendBox box,
            TrajectorySvgModel model,
            double xMin,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        double score = 0.0;
        for (TrajectorySvgSeries series : model.series()) {
            for (TrajectoryCurveSegment segment : series.segments()) {
                double duration = segment.duration();
                if (duration <= 0) continue;
                int samples = Math.max(6, (int) Math.ceil(duration * 12.0));
                for (int i = 0; i <= samples; i++) {
                    double t = segment.startTime() + duration * i / samples;
                    double x = mapX(t, xMin, xRange, plotWidth);
                    double y = mapY(segment.valueAt(t), minY, maxY, plotHeight);
                    if (box.contains(x, y)) {
                        score += 1.0;
                    } else {
                        double dx = box.distanceX(x);
                        double dy = box.distanceY(y);
                        double distanceSq = dx * dx + dy * dy;
                        if (distanceSq < 900.0) {
                            score += 0.15;
                        }
                    }
                }
            }
        }
        return score;
    }

    private static void appendSeriesPath(
            StringBuilder out,
            List<TrajectoryCurveSegment> segments,
            double xMin,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        if (allPolynomialSegments(segments)) {
            appendPolynomialSeriesPath(
                    out,
                    segments.stream().map(PolynomialCurveSegment.class::cast).toList(),
                    xMin,
                    xRange,
                    minY,
                    maxY,
                    plotWidth,
                    plotHeight);
            return;
        }

        boolean moved = false;
        double prevEndTime = Double.NaN;
        double prevEndValue = Double.NaN;
        for (TrajectoryCurveSegment segment : segments) {
            if (segment.duration() <= 0) continue;
            double startValue = segment.valueAt(segment.startTime());
            double x0 = mapX(segment.startTime(), xMin, xRange, plotWidth);
            double y0 = mapY(startValue, minY, maxY, plotHeight);
            boolean contiguous = moved && Math.abs(segment.startTime() - prevEndTime) < 1e-9;
            boolean sameValue = contiguous && Math.abs(startValue - prevEndValue) < 1e-9;
            if (!moved || !contiguous) {
                out.append("M ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ');
                moved = true;
            } else if (!sameValue) {
                out.append("L ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ');
            }
            appendApproximatedSegment(
                    out, segment, xMin, xRange, minY, maxY, plotWidth, plotHeight, 0);
            prevEndTime = segment.endTime();
            prevEndValue = segment.valueAt(segment.endTime());
        }
    }

    private static void appendPolynomialSeriesPath(
            StringBuilder out,
            List<PolynomialCurveSegment> segments,
            double xMin,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        boolean moved = false;
        double prevEndTime = Double.NaN;
        double prevEndValue = Double.NaN;
        for (PolynomialCurveSegment segment : segments) {
            if (segment.duration() <= 0) continue;
            double startValue = segment.valueAt(segment.startTime());
            double endValue = segment.valueAt(segment.endTime());
            double x0 = mapX(segment.startTime(), xMin, xRange, plotWidth);
            double y0 = mapY(startValue, minY, maxY, plotHeight);
            double x3 = mapX(segment.endTime(), xMin, xRange, plotWidth);
            double y3 = mapY(endValue, minY, maxY, plotHeight);
            double m0 = segment.slopeAt(segment.startTime());
            double m1 = segment.slopeAt(segment.endTime());
            double dx = x3 - x0;
            double y1 = y0 - scaleSlope(m0, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0;
            double y2 = y3 + scaleSlope(m1, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0;
            double x1 = x0 + dx / 3.0;
            double x2 = x3 - dx / 3.0;
            boolean contiguous = moved && Math.abs(segment.startTime() - prevEndTime) < 1e-9;
            boolean sameValue = contiguous && Math.abs(startValue - prevEndValue) < 1e-9;
            if (!moved || !contiguous) {
                out.append("M ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ');
                moved = true;
            } else if (!sameValue) {
                out.append("L ")
                        .append(DECIMAL.format(x0))
                        .append(' ')
                        .append(DECIMAL.format(y0))
                        .append(' ');
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
                    .append(' ');
            prevEndTime = segment.endTime();
            prevEndValue = endValue;
        }
    }

    private static boolean allPolynomialSegments(List<TrajectoryCurveSegment> segments) {
        for (TrajectoryCurveSegment segment : segments) {
            if (!(segment instanceof PolynomialCurveSegment)) {
                return false;
            }
        }
        return true;
    }

    private static void appendApproximatedSegment(
            StringBuilder out,
            TrajectoryCurveSegment segment,
            double xMin,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight,
            int depth) {
        Cubic cubic =
                cubicForSegment(segment, segment.startTime(), segment.endTime(), xMin, xRange, minY, maxY, plotWidth, plotHeight);
        if (depth < MAX_APPROXIMATION_DEPTH
                && exceedsApproximationTolerance(
                        cubic, segment, minY, maxY, plotHeight, APPROXIMATION_TOLERANCE_PX)) {
            double mid = (segment.startTime() + segment.endTime()) * 0.5;
            if (mid > segment.startTime() + 1e-12 && mid < segment.endTime() - 1e-12) {
                appendApproximatedSegment(
                        out,
                        segment.clippedTo(segment.startTime(), mid),
                        xMin,
                        xRange,
                        minY,
                        maxY,
                        plotWidth,
                        plotHeight,
                        depth + 1);
                appendApproximatedSegment(
                        out,
                        segment.clippedTo(mid, segment.endTime()),
                        xMin,
                        xRange,
                        minY,
                        maxY,
                        plotWidth,
                        plotHeight,
                        depth + 1);
                return;
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
                .append(' ');
    }

    private static Cubic cubicForSegment(
            TrajectoryCurveSegment segment,
            double startTime,
            double endTime,
            double xMin,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        double startValue = segment.valueAt(startTime);
        double endValue = segment.valueAt(endTime);
        double x0 = mapX(startTime, xMin, xRange, plotWidth);
        double y0 = mapY(startValue, minY, maxY, plotHeight);
        double x3 = mapX(endTime, xMin, xRange, plotWidth);
        double y3 = mapY(endValue, minY, maxY, plotHeight);
        double m0 = segment.slopeAt(startTime);
        double m1 = segment.slopeAt(endTime);
        double dx = x3 - x0;
        double x1 = x0 + dx / 3.0;
        double x2 = x3 - dx / 3.0;
        double y1 = y0 - scaleSlope(m0, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0;
        double y2 = y3 + scaleSlope(m1, xRange, minY, maxY, plotWidth, plotHeight) * dx / 3.0;
        return new Cubic(x0, y0, x1, y1, x2, y2, x3, y3, startTime, endTime);
    }

    private static boolean exceedsApproximationTolerance(
            Cubic cubic,
            TrajectoryCurveSegment segment,
            double minY,
            double maxY,
            double plotHeight,
            double tolerancePx) {
        double[] samples = {0.25, 0.5, 0.75};
        for (double u : samples) {
            double time = cubic.startTime + (cubic.endTime - cubic.startTime) * u;
            double actualY = mapY(segment.valueAt(time), minY, maxY, plotHeight);
            double cubicY = cubic.yAt(u);
            if (Math.abs(actualY - cubicY) > tolerancePx) {
                return true;
            }
        }
        return false;
    }

    private static double mapX(double t, double xMin, double xRange, double plotWidth) {
        return LEFT + ((t - xMin) / xRange) * plotWidth;
    }

    private static double mapY(double value, double minY, double maxY, double plotHeight) {
        return TOP + plotHeight - ((value - minY) / (maxY - minY)) * plotHeight;
    }

    private static double scaleSlope(
            double slope,
            double xRange,
            double minY,
            double maxY,
            double plotWidth,
            double plotHeight) {
        return slope * (xRange / plotWidth) * (plotHeight / (maxY - minY));
    }

    private static String color(Color color) {
        return String.format("#%02x%02x%02x", color.getRed(), color.getGreen(), color.getBlue());
    }

    private static String strokeDashArray(BasicStroke stroke) {
        float[] dash = stroke.getDashArray();
        if (dash == null || dash.length == 0) return "";
        StringBuilder out = new StringBuilder();
        for (int i = 0; i < dash.length; i++) {
            if (i > 0) out.append(',');
            out.append(DECIMAL.format(dash[i]));
        }
        return out.toString();
    }

    private static String escape(String text) {
        return text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;");
    }

    private record LegendBox(double x, double y, double width, double height) {
        double lineX() {
            return x + 10.0;
        }

        double firstLineY() {
            return y + 14.0;
        }

        boolean contains(double px, double py) {
            return px >= x && px <= x + width && py >= y && py <= y + height;
        }

        double distanceX(double px) {
            if (px < x) return x - px;
            if (px > x + width) return px - (x + width);
            return 0.0;
        }

        double distanceY(double py) {
            if (py < y) return y - py;
            if (py > y + height) return py - (y + height);
            return 0.0;
        }
    }

    private record Cubic(
            double x0,
            double y0,
            double x1,
            double y1,
            double x2,
            double y2,
            double x3,
            double y3,
            double startTime,
            double endTime) {
        double yAt(double u) {
            double inv = 1.0 - u;
            return inv * inv * inv * y0
                    + 3.0 * inv * inv * u * y1
                    + 3.0 * inv * u * u * y2
                    + u * u * u * y3;
        }
    }
}
