package com.ruckig;

import static org.junit.jupiter.api.Assertions.fail;

/**
 * Comparison helpers for golden-vector differential tests (plan §5).
 */
public final class GoldenCompare {

    private GoldenCompare() {}

    /**
     * Relative or absolute closeness: |a-b| &lt;= max(absTol, relTol * max(|a|,|b|)).
     */
    public static boolean isClose(double a, double b, double relTol, double absTol) {
        if (Double.isNaN(a) || Double.isNaN(b)) {
            return Double.isNaN(a) && Double.isNaN(b);
        }
        if (Double.isInfinite(a) || Double.isInfinite(b)) {
            return a == b;
        }
        double diff = Math.abs(a - b);
        double scale = Math.max(Math.abs(a), Math.abs(b));
        return diff <= Math.max(absTol, relTol * scale);
    }

    public static void assertClose(String label, double expected, double actual, double relTol, double absTol) {
        if (!isClose(expected, actual, relTol, absTol)) {
            fail(label + ": expected " + expected + " but was " + actual
                    + " (relTol=" + relTol + ", absTol=" + absTol + ")");
        }
    }

    public static void assertCloseArray(String label, double[] expected, double[] actual, double relTol, double absTol) {
        if (expected.length != actual.length) {
            fail(label + ": length " + expected.length + " vs " + actual.length);
        }
        for (int i = 0; i < expected.length; ++i) {
            assertClose(label + "[" + i + "]", expected[i], actual[i], relTol, absTol);
        }
    }
}
