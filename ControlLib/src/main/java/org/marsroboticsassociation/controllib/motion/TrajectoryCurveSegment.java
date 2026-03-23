package org.marsroboticsassociation.controllib.motion;

public interface TrajectoryCurveSegment {

    double startTime();

    double endTime();

    default double duration() {
        return endTime() - startTime();
    }

    double valueAt(double time);

    double slopeAt(double time);

    double minValue();

    double maxValue();

    TrajectoryCurveSegment shiftedBy(double deltaTime);

    TrajectoryCurveSegment clippedTo(double newStartTime, double newEndTime);
}
