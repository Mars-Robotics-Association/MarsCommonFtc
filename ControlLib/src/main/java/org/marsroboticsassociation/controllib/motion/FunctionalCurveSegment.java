package org.marsroboticsassociation.controllib.motion;

import java.util.function.DoubleUnaryOperator;

/**
 * Analytic curve segment backed by value and slope functions over an absolute time interval.
 */
public final class FunctionalCurveSegment implements TrajectoryCurveSegment {

    private static final int EXTREMA_SAMPLES = 128;

    private final double startTime;
    private final double endTime;
    private final DoubleUnaryOperator valueFunction;
    private final DoubleUnaryOperator slopeFunction;

    public FunctionalCurveSegment(
            double startTime,
            double endTime,
            DoubleUnaryOperator valueFunction,
            DoubleUnaryOperator slopeFunction) {
        this.startTime = startTime;
        this.endTime = endTime;
        this.valueFunction = valueFunction;
        this.slopeFunction = slopeFunction;
    }

    @Override
    public double startTime() {
        return startTime;
    }

    @Override
    public double endTime() {
        return endTime;
    }

    @Override
    public double valueAt(double time) {
        return valueFunction.applyAsDouble(time);
    }

    @Override
    public double slopeAt(double time) {
        return slopeFunction.applyAsDouble(time);
    }

    @Override
    public double minValue() {
        return sampledExtremum(false);
    }

    @Override
    public double maxValue() {
        return sampledExtremum(true);
    }

    @Override
    public TrajectoryCurveSegment shiftedBy(double deltaTime) {
        return new FunctionalCurveSegment(
                startTime + deltaTime,
                endTime + deltaTime,
                time -> valueAt(time - deltaTime),
                time -> slopeAt(time - deltaTime));
    }

    @Override
    public TrajectoryCurveSegment clippedTo(double newStartTime, double newEndTime) {
        double clippedStart = Math.max(startTime, newStartTime);
        double clippedEnd = Math.min(endTime, newEndTime);
        if (clippedEnd <= clippedStart) return null;
        return new FunctionalCurveSegment(clippedStart, clippedEnd, this::valueAt, this::slopeAt);
    }

    private double sampledExtremum(boolean max) {
        if (duration() <= 0) return valueAt(startTime);
        double best = valueAt(startTime);
        double endValue = valueAt(endTime);
        best = max ? Math.max(best, endValue) : Math.min(best, endValue);
        for (int i = 1; i < EXTREMA_SAMPLES; i++) {
            double t = startTime + duration() * i / EXTREMA_SAMPLES;
            double value = valueAt(t);
            best = max ? Math.max(best, value) : Math.min(best, value);
        }
        return best;
    }
}
