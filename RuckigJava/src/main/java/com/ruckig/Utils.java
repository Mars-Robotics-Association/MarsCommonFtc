package com.ruckig;

/**
 * Port of {@code ruckig/utils.hpp} (upstream v0.17.3).
 * Transliteration: keep arithmetic order identical to C++.
 */
public final class Utils {

    /** Upstream: {@code DBL_EPSILON} / {@code std::numeric_limits<double>::epsilon()}. */
    public static final double DBL_EPSILON = Math.ulp(1.0); // 2.220446049250313e-16

    private Utils() {}

    /** {@code pow2} from roots.hpp / shared helpers. */
    public static double pow2(double v) {
        return v * v;
    }

    /**
     * C++ {@code std::min} semantics: {@code (b < a) ? b : a}.
     * Differs from {@link Math#min} on NaN and signed zero.
     */
    public static double cppMin(double a, double b) {
        return (b < a) ? b : a;
    }

    /**
     * C++ {@code std::max} semantics: {@code (a < b) ? b : a}.
     * Differs from {@link Math#max} on NaN and signed zero.
     */
    public static double cppMax(double a, double b) {
        return (a < b) ? b : a;
    }

    /**
     * Integrate with constant jerk for duration t.
     * Returns new position, velocity, acceleration via out arrays (length ≥ 1) or the
     * three-slot outPVA buffer: outPVA[0]=p, outPVA[1]=v, outPVA[2]=a.
     */
    public static void integrate(double t, double p0, double v0, double a0, double j, double[] outPVA) {
        outPVA[0] = p0 + t * (v0 + t * (a0 / 2 + t * j / 6));
        outPVA[1] = v0 + t * (a0 + t * j / 2);
        outPVA[2] = a0 + t * j;
    }

    /** Scalar integrate returning p only (for extrema). */
    public static double integrateP(double t, double p0, double v0, double a0, double j) {
        return p0 + t * (v0 + t * (a0 / 2 + t * j / 6));
    }

    public static double integrateV(double t, double v0, double a0, double j) {
        return v0 + t * (a0 + t * j / 2);
    }

    public static double integrateA(double t, double a0, double j) {
        return a0 + t * j;
    }

    public static String join(double[] array, boolean highPrecision) {
        StringBuilder ss = new StringBuilder();
        for (int i = 0; i < array.length; ++i) {
            if (i > 0) {
                ss.append(", ");
            }
            if (highPrecision) {
                ss.append(String.format(java.util.Locale.US, "%.16g", array[i]));
            } else {
                ss.append(array[i]);
            }
        }
        return ss.toString();
    }

    public static void copyDoubles(double[] src, double[] dst) {
        System.arraycopy(src, 0, dst, 0, src.length);
    }

    public static boolean arraysEqual(double[] a, double[] b) {
        if (a == b) {
            return true;
        }
        if (a == null || b == null || a.length != b.length) {
            return false;
        }
        for (int i = 0; i < a.length; ++i) {
            if (Double.doubleToLongBits(a[i]) != Double.doubleToLongBits(b[i])) {
                // Also treat NaN==NaN as equal like operator== on vectors of doubles typically for state?
                // C++ double == is false for NaN. Match IEEE.
                if (a[i] != b[i]) {
                    return false;
                }
            }
        }
        return true;
    }

    public static boolean boolArraysEqual(boolean[] a, boolean[] b) {
        if (a == b) {
            return true;
        }
        if (a == null || b == null || a.length != b.length) {
            return false;
        }
        for (int i = 0; i < a.length; ++i) {
            if (a[i] != b[i]) {
                return false;
            }
        }
        return true;
    }
}
