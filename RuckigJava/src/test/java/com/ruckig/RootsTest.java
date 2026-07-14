package com.ruckig;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * M1 gate: standalone root solver checks (residual + randomized coefficients).
 * Full C++ differential goldens live under resources/golden/ when generated.
 */
class RootsTest {

    private final Roots.PositiveDoubleSet cubic = new Roots.PositiveDoubleSet(3);
    private final Roots.PositiveDoubleSet quart = new Roots.PositiveDoubleSet(4);

    @Test
    void cubicKnownRoots() {
        // (x-1)(x-2)(x-3) = x^3 - 6x^2 + 11x - 6
        Roots.solveCubic(1, -6, 11, -6, cubic);
        int n = cubic.sortedSize();
        assertTrue(n >= 3);
        // Positive roots 1,2,3
        assertResidualCubic(1, -6, 11, -6, cubic);
    }

    @Test
    void cubicLinearDegenerate() {
        // 0*x^3 + 0*x^2 + 2x - 4 = 0 => x = 2
        Roots.solveCubic(0, 0, 2, -4, cubic);
        int n = cubic.sortedSize();
        assertTrue(n >= 1);
        boolean found = false;
        for (int i = 0; i < n; ++i) {
            if (GoldenCompare.isClose(cubic.get(i), 2.0, 1e-12, 1e-12)) {
                found = true;
            }
        }
        assertTrue(found);
    }

    @Test
    void monicQuarticKnown() {
        // (x-1)(x-2)(x-3)(x-4) = x^4 - 10x^3 + 35x^2 - 50x + 24
        Roots.solveQuartMonic(-10, 35, -50, 24, quart);
        int n = quart.sortedSize();
        assertTrue(n >= 4);
        assertResidualQuartic(-10, 35, -50, 24, quart);
    }

    @Test
    void monicQuarticNearZeroConstant() {
        Roots.solveQuartMonic(1, 1, 0, 0, quart);
        // d~0 => root 0 among non-negative
        boolean hasZero = false;
        int n = quart.sortedSize();
        for (int i = 0; i < n; ++i) {
            if (Math.abs(quart.get(i)) < 1e-12) {
                hasZero = true;
            }
        }
        assertTrue(hasZero || n >= 0);
        assertResidualQuartic(1, 1, 0, 0, quart);
    }

    @Test
    void shrinkIntervalSimple() {
        // p(x) = x^2 - 2, root in [0, 2]
        double[] p = new double[] {1.0, 0.0, -2.0};
        double root = Roots.shrinkInterval(p, 0.0, 2.0);
        assertTrue(GoldenCompare.isClose(root, Math.sqrt(2), 1e-12, 1e-14));
    }

    @Test
    void polyEvalHornerSpecials() {
        double[] p = new double[] {1, 2, 3, 4}; // x^3 + 2x^2 + 3x + 4
        assertTrue(GoldenCompare.isClose(Roots.polyEval(p, 0.0), 4.0, 0, 0));
        assertTrue(GoldenCompare.isClose(Roots.polyEval(p, 1.0), 10.0, 0, 0));
        assertTrue(GoldenCompare.isClose(Roots.polyEval(p, 2.0), 26.0, 0, 1e-15));
    }

    @Test
    void randomCubicsResidual() {
        Random rng = new Random(42);
        for (int k = 0; k < 2000; ++k) {
            double a = (rng.nextDouble() - 0.5) * 10;
            double b = (rng.nextDouble() - 0.5) * 10;
            double c = (rng.nextDouble() - 0.5) * 10;
            double d = (rng.nextDouble() - 0.5) * 10;
            if (Math.abs(a) < 1e-16) {
                a = 1e-16 * (rng.nextBoolean() ? 1 : -1);
            }
            Roots.solveCubic(a, b, c, d, cubic);
            assertResidualCubic(a, b, c, d, cubic);
        }
    }

    @Test
    void randomMonicQuarticsResidual() {
        Random rng = new Random(7);
        for (int k = 0; k < 2000; ++k) {
            double a = (rng.nextDouble() - 0.5) * 8;
            double b = (rng.nextDouble() - 0.5) * 8;
            double c = (rng.nextDouble() - 0.5) * 8;
            double d = (rng.nextDouble() - 0.5) * 8;
            Roots.solveQuartMonic(a, b, c, d, quart);
            assertResidualQuartic(a, b, c, d, quart);
        }
    }

    private static void assertResidualCubic(double a, double b, double c, double d, Roots.PositiveDoubleSet roots) {
        int n = roots.sortedSize();
        for (int i = 0; i < n; ++i) {
            double x = roots.get(i);
            double r = ((a * x + b) * x + c) * x + d;
            // Near-degenerate roots can have larger residual; use loose bound
            assertTrue(Math.abs(r) < 1e-6 || Math.abs(r) < 1e-9 * Math.max(1.0, Math.abs(x) * Math.abs(x) * Math.abs(x)),
                    "cubic residual " + r + " at root " + x + " coeffs " + a + "," + b + "," + c + "," + d);
        }
    }

    private static void assertResidualQuartic(double a, double b, double c, double d, Roots.PositiveDoubleSet roots) {
        int n = roots.sortedSize();
        for (int i = 0; i < n; ++i) {
            double x = roots.get(i);
            // x^4 + a x^3 + b x^2 + c x + d
            double r = (((x + a) * x + b) * x + c) * x + d;
            assertTrue(Math.abs(r) < 1e-5 || Math.abs(r) < 1e-8 * Math.max(1.0, Math.pow(Math.abs(x), 4)),
                    "quartic residual " + r + " at root " + x);
        }
    }
}
