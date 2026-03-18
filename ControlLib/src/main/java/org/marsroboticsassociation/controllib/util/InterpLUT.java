package org.marsroboticsassociation.controllib.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * A lookup table that performs monotone cubic Hermite spline interpolation between control points.
 * This is originally from FtcLib
 *
 * <p>Typical usage:
 * <pre>{@code
 * InterpLUT lut = new InterpLUT();
 * lut.add(0.0, 0.0);
 * lut.add(10.0, 0.5);
 * lut.add(20.0, 1.0);
 * lut.createLUT();  // must be called before get()
 *
 * double y = lut.get(5.0);  // interpolated value between the first two points
 * }</pre>
 *
 * <p>Control points must be added in strictly increasing order of x-value via {@link #add(double, double)}.
 * After all points have been added, call {@link #createLUT()} once to compute the spline coefficients.
 * Then use {@link #get(double)} to query interpolated values. Querying values outside the x-range of the
 * control points (inclusive of the endpoints) will throw {@link IllegalArgumentException}.
 *
 * <p>The spline passes through each control point exactly. If the y-values are monotonic (non-decreasing or
 * non-increasing), the interpolated values between them will also be monotonic (Fritsch-Carlson method).
 *
 * @see LinInterpTable for a simpler linear interpolation alternative that does not require sorted insertion
 * @see LUT for a nearest-value lookup table without interpolation
 */
public class InterpLUT {

    private List<Double> mX = new ArrayList<>();
    private List<Double> mY = new ArrayList<>();
    private List<Double> mM = new ArrayList<>();

    private InterpLUT(List<Double> x, List<Double> y, List<Double> m) {
        mX = x;
        mY = y;
        mM = m;
    }

    public InterpLUT() {
    }

    /**
     * Adds a control point to the table. Points must be added in strictly increasing order of {@code input}
     * (the x-value). At least two points must be added before calling {@link #createLUT()}.
     *
     * @param input  the x-value of the control point
     * @param output the y-value of the control point
     */
    public void add(double input, double output) {
        mX.add(input);
        mY.add(output);
    }

    /**
     * Computes the monotone cubic spline coefficients from the added control points. Must be called exactly
     * once after all {@link #add(double, double)} calls and before any {@link #get(double)} calls.
     *
     * <p>The control points must have strictly increasing x-values. The spline is guaranteed to pass through
     * each control point exactly, and if the y-values are monotonic the interpolated curve will be too.
     *
     * @throws IllegalArgumentException if fewer than 2 points were added, or if x-values are not strictly increasing.
     */
    public void createLUT() {
        List<Double> x = this.mX;
        List<Double> y = this.mY;

        if (x == null || y == null || x.size() != y.size() || x.size() < 2) {
            throw new IllegalArgumentException("There must be at least two control "
                    + "points and the arrays must be of equal length.");
        }

        final int n = x.size();
        Double[] d = new Double[n - 1]; // could optimize this out
        Double[] m = new Double[n];

        // Compute slopes of secant lines between successive points.
        for (int i = 0; i < n - 1; i++) {
            Double h = x.get(i + 1) - x.get(i);
            if (h <= 0f) {
                throw new IllegalArgumentException("The control points must all "
                        + "have strictly increasing X values.");
            }
            d[i] = (y.get(i + 1) - y.get(i)) / h;
        }

        // Initialize the tangents as the average of the secants.
        m[0] = d[0];
        for (int i = 1; i < n - 1; i++) {
            m[i] = (d[i - 1] + d[i]) * 0.5f;
        }
        m[n - 1] = d[n - 2];

        // Update the tangents to preserve monotonicity.
        for (int i = 0; i < n - 1; i++) {
            if (d[i] == 0f) { // successive Y values are equal
                m[i] = Double.valueOf(0f);
                m[i + 1] = Double.valueOf(0f);
            } else {
                double a = m[i] / d[i];
                double b = m[i + 1] / d[i];
                double h = Math.hypot(a, b);
                if (h > 9f) {
                    double t = 3f / h;
                    m[i] = t * a * d[i];
                    m[i + 1] = t * b * d[i];
                }
            }
        }
        mX = x;
        mY = y;
        mM = Arrays.asList(m);
    }

    /**
     * Returns the interpolated y-value for the given x-value using the precomputed spline.
     * {@link #createLUT()} must have been called before this method.
     *
     * <p>If the input exactly matches a control point's x-value, the corresponding y-value is returned
     * without interpolation. Querying values outside the x-range of the control points throws
     * {@link IllegalArgumentException}.
     *
     * @param input the x-value to interpolate at (must be within the control point range, inclusive)
     * @return the interpolated y-value
     * @throws IllegalArgumentException if {@code input} is outside the domain of the control points
     */
    public double get(double input) {
        // Handle the boundary cases.
        final int n = mX.size();
        if (Double.isNaN(input)) {
            return input;
        }
        if (input < mX.get(0)) {
            throw new IllegalArgumentException("User requested value outside of bounds of LUT. Bounds are: " + mX.get(0).toString() + " to " + mX.get(n - 1).toString() + ". Value provided was: " + input);
        }
        if (input == mX.get(0)) {
            return mY.get(0);
        }
        if (input > mX.get(n - 1)) {
            throw new IllegalArgumentException("User requested value outside of bounds of LUT. Bounds are: " + mX.get(0).toString() + " to " + mX.get(n - 1).toString() + ". Value provided was: " + input);
        }
        if (input == mX.get(n - 1)) {
            return mY.get(n - 1);
        }

        // Find the index 'i' of the last point with smaller X.
        // We know this will be within the spline due to the boundary tests.
        int i = 0;
        while (input >= mX.get(i + 1)) {
            i += 1;
            if (input == mX.get(i)) {
                return mY.get(i);
            }
        }

        // Perform cubic Hermite spline interpolation.
        double h = mX.get(i + 1) - mX.get(i);
        double t = (input - mX.get(i)) / h;
        return (mY.get(i) * (1 + 2 * t) + h * mM.get(i) * t) * (1 - t) * (1 - t)
                + (mY.get(i + 1) * (3 - 2 * t) + h * mM.get(i + 1) * (t - 1)) * t * t;
    }

    // For debugging.
    @Override
    public String toString() {
        StringBuilder str = new StringBuilder();
        final int n = mX.size();
        str.append("[");
        for (int i = 0; i < n; i++) {
            if (i != 0) {
                str.append(", ");
            }
            str.append("(").append(mX.get(i));
            str.append(", ").append(mY.get(i));
            str.append(": ").append(mM.get(i)).append(")");
        }
        str.append("]");
        return str.toString();
    }

}
