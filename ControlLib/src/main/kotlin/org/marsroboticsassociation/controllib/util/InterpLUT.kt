package org.marsroboticsassociation.controllib.util

import kotlin.math.hypot

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
 * <p>Control points must be added in strictly increasing order of x-value via [add]. After all
 * points have been added, call [createLUT] once to compute the spline coefficients. Then use [get]
 * to query interpolated values. Querying values outside the x-range of the control points
 * (inclusive of the endpoints) will throw [IllegalArgumentException].
 *
 * <p>The spline passes through each control point exactly. If the y-values are monotonic
 * (non-decreasing or non-increasing), the interpolated values between them will also be monotonic
 * (Fritsch-Carlson method).
 *
 * @see LinInterpTable for a simpler linear interpolation alternative that does not require sorted
 *   insertion
 * @see LUT for a nearest-value lookup table without interpolation
 */
class InterpLUT {
    private var mX: MutableList<Double> = mutableListOf()
    private var mY: MutableList<Double> = mutableListOf()
    private var mM: List<Double> = emptyList()

    /**
     * Adds a control point to the table. Points must be added in strictly increasing order of
     * [input] (the x-value). At least two points must be added before calling [createLUT].
     *
     * @param input the x-value of the control point
     * @param output the y-value of the control point
     */
    fun add(input: Double, output: Double) {
        mX.add(input)
        mY.add(output)
    }

    /**
     * Computes the monotone cubic spline coefficients from the added control points. Must be called
     * exactly once after all [add] calls and before any [get] calls.
     *
     * <p>The control points must have strictly increasing x-values. The spline is guaranteed to
     * pass through each control point exactly, and if the y-values are monotonic the interpolated
     * curve will be too.
     *
     * @throws IllegalArgumentException if fewer than 2 points were added, or if x-values are not
     *   strictly increasing.
     */
    fun createLUT() {
        val x = mX
        val y = mY

        require(x.size == y.size && x.size >= 2) {
            "There must be at least two control points and the arrays must be of equal length."
        }

        val n = x.size
        val d = Array(n - 1) { 0.0 }
        val m = Array(n) { 0.0 }

        // Compute slopes of secant lines between successive points.
        for (i in 0 until n - 1) {
            val h = x[i + 1] - x[i]
            require(h > 0.0) { "The control points must all have strictly increasing X values." }
            d[i] = (y[i + 1] - y[i]) / h
        }

        // Initialize the tangents as the average of the secants.
        m[0] = d[0]
        for (i in 1 until n - 1) {
            m[i] = (d[i - 1] + d[i]) * 0.5
        }
        m[n - 1] = d[n - 2]

        // Update the tangents to preserve monotonicity.
        for (i in 0 until n - 1) {
            if (d[i] == 0.0) {
                m[i] = 0.0
                m[i + 1] = 0.0
            } else {
                val a = m[i] / d[i]
                val b = m[i + 1] / d[i]
                val h = hypot(a, b)
                if (h > 9.0) {
                    val t = 3.0 / h
                    m[i] = t * a * d[i]
                    m[i + 1] = t * b * d[i]
                }
            }
        }
        mX = x
        mY = y
        mM = m.toList()
    }

    /**
     * Returns the interpolated y-value for the given x-value using the precomputed spline.
     * [createLUT] must have been called before this method.
     *
     * <p>If the input exactly matches a control point's x-value, the corresponding y-value is
     * returned without interpolation. Querying values outside the x-range of the control points
     * throws [IllegalArgumentException].
     *
     * @param input the x-value to interpolate at (must be within the control point range,
     *   inclusive)
     * @return the interpolated y-value
     * @throws IllegalArgumentException if [input] is outside the domain of the control points
     */
    fun get(input: Double): Double {
        val n = mX.size
        if (input.isNaN()) {
            return input
        }
        if (input < mX[0]) {
            throw IllegalArgumentException(
                "User requested value outside of bounds of LUT. Bounds are: ${mX[0]} to ${mX[n - 1]}. Value provided was: $input"
            )
        }
        if (input == mX[0]) {
            return mY[0]
        }
        if (input > mX[n - 1]) {
            throw IllegalArgumentException(
                "User requested value outside of bounds of LUT. Bounds are: ${mX[0]} to ${mX[n - 1]}. Value provided was: $input"
            )
        }
        if (input == mX[n - 1]) {
            return mY[n - 1]
        }

        // Find the index 'i' of the last point with smaller X.
        var i = 0
        while (input >= mX[i + 1]) {
            i += 1
            if (input == mX[i]) {
                return mY[i]
            }
        }

        // Perform cubic Hermite spline interpolation.
        val h = mX[i + 1] - mX[i]
        val t = (input - mX[i]) / h
        return (mY[i] * (1 + 2 * t) + h * mM[i] * t) * (1 - t) * (1 - t) +
            (mY[i + 1] * (3 - 2 * t) + h * mM[i + 1] * (t - 1)) * t * t
    }

    override fun toString(): String =
        mX.indices.joinToString(prefix = "[", postfix = "]") { i ->
            "(${mX[i]}, ${mY[i]}: ${mM.getOrElse(i) { 0.0 }})"
        }
}
