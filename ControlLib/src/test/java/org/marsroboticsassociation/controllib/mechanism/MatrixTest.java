package org.marsroboticsassociation.controllib.mechanism;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertThrows;

import org.junit.Test;

public class MatrixTest {

    private static void assertMatrixEquals(double[][] expected, Matrix actual, double tol) {
        assertEquals("row count", expected.length, actual.rows());
        assertEquals("col count", expected[0].length, actual.cols());
        for (int r = 0; r < expected.length; r++) {
            for (int c = 0; c < expected[0].length; c++) {
                assertEquals("entry [" + r + "][" + c + "]", expected[r][c], actual.get(r, c), tol);
            }
        }
    }

    @Test
    public void identityHasOnesOnDiagonal() {
        assertMatrixEquals(
                new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, Matrix.identity(3), 0.0);
    }

    @Test
    public void multiplyMatchesHandComputation() {
        Matrix a = new Matrix(new double[][] {{1, 2}, {3, 4}});
        Matrix b = new Matrix(new double[][] {{5, 6}, {7, 8}});
        assertMatrixEquals(new double[][] {{19, 22}, {43, 50}}, a.times(b), 1e-12);
    }

    @Test
    public void transposeSwapsRowsAndColumns() {
        Matrix a = new Matrix(new double[][] {{1, 2, 3}, {4, 5, 6}});
        assertMatrixEquals(new double[][] {{1, 4}, {2, 5}, {3, 6}}, a.transpose(), 0.0);
    }

    @Test
    public void plusAndMinusAreElementwise() {
        Matrix a = new Matrix(new double[][] {{1, 2}, {3, 4}});
        Matrix b = new Matrix(new double[][] {{10, 20}, {30, 40}});
        assertMatrixEquals(new double[][] {{11, 22}, {33, 44}}, a.plus(b), 0.0);
        assertMatrixEquals(new double[][] {{9, 18}, {27, 36}}, b.minus(a), 0.0);
    }

    @Test
    public void inverseTimesOriginalIsIdentity() {
        Matrix a = new Matrix(new double[][] {{4, 7}, {2, 6}});
        assertMatrixEquals(new double[][] {{1, 0}, {0, 1}}, a.times(a.inverse()), 1e-12);

        Matrix b = new Matrix(new double[][] {{2, 1, 1}, {1, 3, 2}, {1, 0, 0}});
        assertMatrixEquals(
                new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, b.times(b.inverse()), 1e-12);
    }

    @Test
    public void singularMatrixCannotBeInverted() {
        Matrix singular = new Matrix(new double[][] {{1, 2}, {2, 4}});
        assertThrows(ArithmeticException.class, singular::inverse);
    }

    @Test
    public void nonSquareMatrixCannotBeInverted() {
        Matrix wide = new Matrix(new double[][] {{1, 2, 3}, {4, 5, 6}});
        assertThrows(IllegalArgumentException.class, wide::inverse);
    }

    @Test
    public void mismatchedShapesRejected() {
        Matrix a = new Matrix(new double[][] {{1, 2}});
        Matrix b = new Matrix(new double[][] {{1, 2}, {3, 4}});
        assertThrows(IllegalArgumentException.class, () -> a.plus(b)); // 1x2 + 2x2
        assertThrows(IllegalArgumentException.class, () -> a.times(a)); // 1x2 * 1x2
    }
}
