package org.marsroboticsassociation.controllib.motion;

/**
 * Piecewise polynomial segment over an absolute time interval.
 *
 * <p>The polynomial is evaluated in local time {@code dt = t - startTime}:
 *
 * <pre>
 * value(dt) = c0 + c1*dt + c2*dt^2 + c3*dt^3
 * </pre>
 */
public record PolynomialCurveSegment(
        double startTime, double endTime, double c0, double c1, double c2, double c3)
        implements TrajectoryCurveSegment {

    public double duration() {
        return endTime - startTime;
    }

    public double valueAt(double time) {
        double dt = time - startTime;
        return ((c3 * dt + c2) * dt + c1) * dt + c0;
    }

    public double slopeAt(double time) {
        double dt = time - startTime;
        return (3.0 * c3 * dt + 2.0 * c2) * dt + c1;
    }

    public double minValue() {
        return extremum(false);
    }

    public double maxValue() {
        return extremum(true);
    }

    public PolynomialCurveSegment shiftedBy(double deltaTime) {
        return new PolynomialCurveSegment(
                startTime + deltaTime, endTime + deltaTime, c0, c1, c2, c3);
    }

    public PolynomialCurveSegment clippedTo(double newStartTime, double newEndTime) {
        double clippedStart = Math.max(startTime, newStartTime);
        double clippedEnd = Math.min(endTime, newEndTime);
        if (clippedEnd <= clippedStart) return null;

        double h = clippedStart - startTime;
        double newC0 = valueAt(clippedStart);
        double newC1 = slopeAt(clippedStart);
        double newC2 = c2 + 3.0 * c3 * h;
        return new PolynomialCurveSegment(clippedStart, clippedEnd, newC0, newC1, newC2, c3);
    }

    private double extremum(boolean max) {
        double best = valueAt(startTime);
        double endValue = valueAt(endTime);
        best = max ? Math.max(best, endValue) : Math.min(best, endValue);

        double duration = duration();
        if (duration <= 0) return best;

        if (Math.abs(c3) < 1e-12) {
            if (Math.abs(c2) < 1e-12) return best;
            double dt = -c1 / (2.0 * c2);
            if (dt > 0 && dt < duration) {
                double value = ((c3 * dt + c2) * dt + c1) * dt + c0;
                best = max ? Math.max(best, value) : Math.min(best, value);
            }
            return best;
        }

        double a = 3.0 * c3;
        double b = 2.0 * c2;
        double c = c1;
        double disc = b * b - 4.0 * a * c;
        if (disc < 0) return best;
        double sqrtDisc = Math.sqrt(disc);
        double dt1 = (-b + sqrtDisc) / (2.0 * a);
        double dt2 = (-b - sqrtDisc) / (2.0 * a);
        if (dt1 > 0 && dt1 < duration) {
            double value = ((c3 * dt1 + c2) * dt1 + c1) * dt1 + c0;
            best = max ? Math.max(best, value) : Math.min(best, value);
        }
        if (dt2 > 0 && dt2 < duration) {
            double value = ((c3 * dt2 + c2) * dt2 + c1) * dt2 + c0;
            best = max ? Math.max(best, value) : Math.min(best, value);
        }
        return best;
    }
}
