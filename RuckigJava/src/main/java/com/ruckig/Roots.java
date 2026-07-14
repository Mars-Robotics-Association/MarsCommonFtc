package com.ruckig;

/**
 * Port of {@code ruckig/roots.hpp} (upstream v0.17.3).
 * Faithful transliteration — operation order, epsilons, and control flow match C++.
 */
public final class Roots {

    /** Upstream: {@code constexpr double tolerance {1e-14};} */
    public static final double TOLERANCE = 1e-14;

    private static final double COS120 = -0.50;
    private static final double SIN120 = 0.866025403784438646764;
    private static final double M_PI = Math.PI;

    private Roots() {}

    // -------------------------------------------------------------------------
    // Stack-based set (no java.util collections on hot path)
    // -------------------------------------------------------------------------

    /**
     * Fixed-capacity set of doubles. {@link #sortedBegin} sorts on access like C++
     * {@code Set::begin()}.
     */
    public static class DoubleSet {
        private final double[] data;
        private int size;

        public DoubleSet(int capacity) {
            data = new double[capacity];
            size = 0;
        }

        public void clear() {
            size = 0;
        }

        public void insert(double value) {
            data[size] = value;
            ++size;
        }

        public int size() {
            return size;
        }

        /** Sort in place and return the number of elements (C++ begin()/end() span). */
        public int sortedSize() {
            // Insertion sort (small N ≤ 4) — stable, no allocation
            for (int i = 1; i < size; ++i) {
                double key = data[i];
                int j = i - 1;
                while (j >= 0 && data[j] > key) {
                    data[j + 1] = data[j];
                    --j;
                }
                data[j + 1] = key;
            }
            return size;
        }

        public double get(int i) {
            return data[i];
        }

        public double[] data() {
            return data;
        }
    }

    /** Set that only inserts non-negative values. */
    public static final class PositiveDoubleSet extends DoubleSet {
        public PositiveDoubleSet(int capacity) {
            super(capacity);
        }

        @Override
        public void insert(double value) {
            if (value >= 0) {
                super.insert(value);
            }
        }
    }

    /**
     * Fixed-capacity set of (first, second) pairs, sorted by first then second
     * (matches {@code std::pair} ordering). Used in position step2 only.
     */
    public static final class PairSet {
        private final double[] first;
        private final double[] second;
        private int size;

        public PairSet(int capacity) {
            first = new double[capacity];
            second = new double[capacity];
            size = 0;
        }

        public void clear() {
            size = 0;
        }

        public void insert(double a, double b) {
            first[size] = a;
            second[size] = b;
            ++size;
        }

        public int sortedSize() {
            for (int i = 1; i < size; ++i) {
                double kf = first[i];
                double ks = second[i];
                int j = i - 1;
                while (j >= 0 && (first[j] > kf || (first[j] == kf && second[j] > ks))) {
                    first[j + 1] = first[j];
                    second[j + 1] = second[j];
                    --j;
                }
                first[j + 1] = kf;
                second[j + 1] = ks;
            }
            return size;
        }

        public double firstAt(int i) {
            return first[i];
        }

        public double secondAt(int i) {
            return second[i];
        }
    }

    // -------------------------------------------------------------------------
    // Solvers
    // -------------------------------------------------------------------------

    /**
     * Calculate all non-negative roots of {@code a*x^3 + b*x^2 + c*x + d = 0}.
     * Caller-owned out set is cleared and filled (capacity ≥ 3).
     */
    public static void solveCubic(double a, double b, double c, double d, PositiveDoubleSet roots) {
        roots.clear();

        if (Math.abs(d) < Utils.DBL_EPSILON) {
            // First solution is x = 0
            roots.insert(0.0);

            // Converting to a quadratic equation
            d = c;
            c = b;
            b = a;
            a = 0.0;
        }

        if (Math.abs(a) < Utils.DBL_EPSILON) {
            if (Math.abs(b) < Utils.DBL_EPSILON) {
                // Linear equation
                if (Math.abs(c) > Utils.DBL_EPSILON) {
                    roots.insert(-d / c);
                }

            } else {
                // Quadratic equation
                final double discriminant = c * c - 4 * b * d;
                if (discriminant >= 0) {
                    final double inv2b = 1.0 / (2 * b);
                    final double y = Math.sqrt(discriminant);
                    roots.insert((-c + y) * inv2b);
                    roots.insert((-c - y) * inv2b);
                }
            }

        } else {
            // Cubic equation
            final double inva = 1.0 / a;
            final double invaa = inva * inva;
            final double bb = b * b;
            final double bover3a = b * inva / 3;
            final double p = (a * c - bb / 3) * invaa;
            final double halfq = (2 * bb * b - 9 * a * b * c + 27 * a * a * d) / 54 * invaa * inva;
            final double yy = p * p * p / 27 + halfq * halfq;

            if (yy > Utils.DBL_EPSILON) {
                // Sqrt is positive: one real solution
                final double y = Math.sqrt(yy);
                final double uuu = -halfq + y;
                final double vvv = -halfq - y;
                final double www = Math.abs(uuu) > Math.abs(vvv) ? uuu : vvv;
                final double w = Math.cbrt(www);
                roots.insert(w - p / (3 * w) - bover3a);
            } else if (yy < -Utils.DBL_EPSILON) {
                // Sqrt is negative: three real solutions
                final double x = -halfq;
                final double y = Math.sqrt(-yy);
                double theta;
                double r;

                // Convert to polar form
                if (Math.abs(x) > Utils.DBL_EPSILON) {
                    theta = (x > 0.0) ? Math.atan(y / x) : (Math.atan(y / x) + M_PI);
                    r = Math.sqrt(x * x - yy);
                } else {
                    // Vertical line
                    theta = M_PI / 2;
                    r = y;
                }
                // Calculate cube root
                theta /= 3;
                r = 2 * Math.cbrt(r);
                // Convert to complex coordinate
                final double ux = Math.cos(theta) * r;
                final double uyi = Math.sin(theta) * r;

                roots.insert(ux - bover3a);
                roots.insert(ux * COS120 - uyi * SIN120 - bover3a);
                roots.insert(ux * COS120 + uyi * SIN120 - bover3a);
            } else {
                // Sqrt is zero: two real solutions
                final double www = -halfq;
                final double w = 2 * Math.cbrt(www);

                roots.insert(w - bover3a);
                roots.insert(w * COS120 - bover3a);
            }
        }
    }

    /**
     * Solve resolvent equation of corresponding Quartic equation.
     * The input x must be of length 3. Number of zeros are returned.
     */
    public static int solveResolvent(double[] x, double a, double b, double c) {
        a /= 3;
        final double a2 = a * a;
        double q = a2 - b / 3;
        final double r = (a * (2 * a2 - b) + c) / 2;
        final double r2 = r * r;
        final double q3 = q * q * q;

        if (r2 < q3) {
            final double qsqrt = Math.sqrt(q);
            final double t = Utils.cppMin(Utils.cppMax(r / (q * qsqrt), -1.0), 1.0);
            q = -2 * qsqrt;

            final double theta = Math.acos(t) / 3;
            final double ux = Math.cos(theta) * q;
            final double uyi = Math.sin(theta) * q;
            x[0] = ux - a;
            x[1] = ux * COS120 - uyi * SIN120 - a;
            x[2] = ux * COS120 + uyi * SIN120 - a;
            return 3;

        } else {
            double A = -Math.cbrt(Math.abs(r) + Math.sqrt(r2 - q3));
            if (r < 0.0) {
                A = -A;
            }
            final double B = (0.0 == A ? 0.0 : q / A);

            x[0] = (A + B) - a;
            x[1] = -(A + B) / 2 - a;
            x[2] = Math.sqrt(3) * (A - B) / 2;
            if (Math.abs(x[2]) < Utils.DBL_EPSILON) {
                x[2] = x[1];
                return 2;
            }

            return 1;
        }
    }

    /**
     * Calculate all non-negative roots of the monic quartic:
     * {@code x^4 + a*x^3 + b*x^2 + c*x + d = 0}.
     */
    public static void solveQuartMonic(double a, double b, double c, double d, PositiveDoubleSet roots) {
        roots.clear();

        if (Math.abs(d) < Utils.DBL_EPSILON) {
            if (Math.abs(c) < Utils.DBL_EPSILON) {
                roots.insert(0.0);

                final double D = a * a - 4 * b;
                if (Math.abs(D) < Utils.DBL_EPSILON) {
                    roots.insert(-a / 2);
                } else if (D > 0.0) {
                    final double sqrtD = Math.sqrt(D);
                    roots.insert((-a - sqrtD) / 2);
                    roots.insert((-a + sqrtD) / 2);
                }
                return;
            }

            if (Math.abs(a) < Utils.DBL_EPSILON && Math.abs(b) < Utils.DBL_EPSILON) {
                roots.insert(0.0);
                roots.insert(-Math.cbrt(c));
                return;
            }
        }

        final double a3 = -b;
        final double b3 = a * c - 4 * d;
        final double c3 = -a * a * d - c * c + 4 * b * d;

        final double[] x3 = new double[3];
        final int number_zeroes = solveResolvent(x3, a3, b3, c3);

        double y = x3[0];
        // Choosing Y with maximal absolute value.
        if (number_zeroes != 1) {
            if (Math.abs(x3[1]) > Math.abs(y)) {
                y = x3[1];
            }
            if (Math.abs(x3[2]) > Math.abs(y)) {
                y = x3[2];
            }
        }

        double q1, q2, p1, p2;

        double D = y * y - 4 * d;
        if (Math.abs(D) < Utils.DBL_EPSILON) {
            q1 = q2 = y / 2;
            D = a * a - 4 * (b - y);
            if (Math.abs(D) < Utils.DBL_EPSILON) {
                p1 = p2 = a / 2;
            } else {
                final double sqrtD = Math.sqrt(D);
                p1 = (a + sqrtD) / 2;
                p2 = (a - sqrtD) / 2;
            }
        } else {
            final double sqrtD = Math.sqrt(D);
            q1 = (y + sqrtD) / 2;
            q2 = (y - sqrtD) / 2;
            p1 = (a * q1 - c) / (q1 - q2);
            p2 = (c - a * q2) / (q1 - q2);
        }

        final double eps = 16 * Utils.DBL_EPSILON;

        D = p1 * p1 - 4 * q1;
        if (Math.abs(D) < eps) {
            roots.insert(-p1 / 2);
        } else if (D > 0.0) {
            final double sqrtD = Math.sqrt(D);
            roots.insert((-p1 - sqrtD) / 2);
            roots.insert((-p1 + sqrtD) / 2);
        }

        D = p2 * p2 - 4 * q2;
        if (Math.abs(D) < eps) {
            roots.insert(-p2 / 2);
        } else if (D > 0.0) {
            final double sqrtD = Math.sqrt(D);
            roots.insert((-p2 - sqrtD) / 2);
            roots.insert((-p2 + sqrtD) / 2);
        }
    }

    /** Overload: monic quartic coeffs {@code [a,b,c,d]} for {@code x^4 + a x^3 + ...}. */
    public static void solveQuartMonic(double[] polynom, PositiveDoubleSet roots) {
        solveQuartMonic(polynom[0], polynom[1], polynom[2], polynom[3], roots);
    }

    /**
     * Evaluate polynomial of length N (highest degree first) at x.
     * {@code p[0]} multiplies {@code x^{N-1}}, {@code p[N-1]} is constant term.
     */
    public static double polyEval(double[] p, int n, double x) {
        double retVal = 0.0;
        if (n == 0) {
            return retVal;
        }

        if (Math.abs(x) < Utils.DBL_EPSILON) {
            retVal = p[n - 1];
        } else if (x == 1.0) {
            for (int i = n - 1; i >= 0; i--) {
                retVal += p[i];
            }
        } else {
            double xn = 1.0;

            for (int i = n - 1; i >= 0; i--) {
                retVal += p[i] * xn;
                xn *= x;
            }
        }

        return retVal;
    }

    public static double polyEval(double[] p, double x) {
        return polyEval(p, p.length, x);
    }

    /**
     * Derivative coefficients into {@code deriv} of length {@code n-1}.
     */
    public static void polyDerivative(double[] coeffs, int n, double[] deriv) {
        for (int i = 0; i < n - 1; ++i) {
            deriv[i] = (n - 1 - i) * coeffs[i];
        }
    }

    /**
     * Monic derivative: leading coeff of result is 1.
     * {@code monicCoeffs} length n, {@code deriv} length n-1.
     */
    public static void polyMonicDerivative(double[] monicCoeffs, int n, double[] deriv) {
        deriv[0] = 1.0;
        for (int i = 1; i < n - 1; ++i) {
            deriv[i] = (n - 1 - i) * monicCoeffs[i] / (n - 1);
        }
    }

    /**
     * Safe Newton/bisection: single zero of p(x) inside [lbound, ubound].
     * Requirements: p(lbound)*p(ubound) &lt; 0, lbound &lt; ubound.
     * Scratch: {@code deriv} length ≥ n-1.
     */
    public static double shrinkInterval(double[] p, int n, double l, double h, double[] deriv) {
        final double fl = polyEval(p, n, l);
        final double fh = polyEval(p, n, h);
        if (fl == 0.0) {
            return l;
        }
        if (fh == 0.0) {
            return h;
        }
        if (fl > 0.0) {
            double tmp = l;
            l = h;
            h = tmp;
        }

        double rts = (l + h) / 2;
        double dxold = Math.abs(h - l);
        double dx = dxold;
        polyDerivative(p, n, deriv);
        double f = polyEval(p, n, rts);
        double df = polyEval(deriv, n - 1, rts);
        double temp;
        final int maxIts = 128;
        for (int j = 0; j < maxIts; j++) {
            if ((((rts - h) * df - f) * ((rts - l) * df - f) > 0.0) || (Math.abs(2 * f) > Math.abs(dxold * df))) {
                dxold = dx;
                dx = (h - l) / 2;
                rts = l + dx;
                if (l == rts) {
                    break;
                }
            } else {
                dxold = dx;
                dx = f / df;
                temp = rts;
                rts -= dx;
                if (temp == rts) {
                    break;
                }
            }

            if (Math.abs(dx) < TOLERANCE) {
                break;
            }

            f = polyEval(p, n, rts);
            df = polyEval(deriv, n - 1, rts);
            if (f < 0.0) {
                l = rts;
            } else {
                h = rts;
            }
        }

        return rts;
    }

    /** Convenience: allocates temporary derivative buffer (not for hot path). */
    public static double shrinkInterval(double[] p, double l, double h) {
        double[] deriv = new double[p.length - 1];
        return shrinkInterval(p, p.length, l, h, deriv);
    }
}
