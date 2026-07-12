package org.marsroboticsassociation.controllib.mechanism;

/**
 * A small, plain dense matrix with just enough arithmetic for the Kalman filter examples in this
 * package: add, subtract, multiply, transpose, and invert.
 *
 * <p>Matrices here are immutable. Every operation returns a new {@code Matrix} rather than changing
 * the one you called it on, which keeps the Kalman math easy to read and impossible to corrupt by
 * accident.
 *
 * <p>This is not a fast or general-purpose linear-algebra library. The matrices in these filters
 * are tiny (2x2 and smaller), so clarity matters far more than speed.
 */
public class Matrix {
    private final double[][] data;
    private final int rows;
    private final int cols;

    /** Builds a matrix from a 2D array. The array is copied, so later changes to it are ignored. */
    public Matrix(double[][] values) {
        this.rows = values.length;
        this.cols = values[0].length;
        this.data = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            if (values[r].length != cols) {
                throw new IllegalArgumentException("Every row must have the same length");
            }
            System.arraycopy(values[r], 0, data[r], 0, cols);
        }
    }

    /** Builds a column vector (an n-by-1 matrix) from a list of numbers. */
    public static Matrix column(double... values) {
        double[][] d = new double[values.length][1];
        for (int i = 0; i < values.length; i++) {
            d[i][0] = values[i];
        }
        return new Matrix(d);
    }

    /** Builds an n-by-n identity matrix: 1s on the diagonal, 0s everywhere else. */
    public static Matrix identity(int n) {
        double[][] d = new double[n][n];
        for (int i = 0; i < n; i++) {
            d[i][i] = 1.0;
        }
        return new Matrix(d);
    }

    public int rows() {
        return rows;
    }

    public int cols() {
        return cols;
    }

    public double get(int row, int col) {
        return data[row][col];
    }

    /** Returns this matrix plus another of the same shape. */
    public Matrix plus(Matrix other) {
        requireSameShape(other);
        double[][] result = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                result[r][c] = data[r][c] + other.data[r][c];
            }
        }
        return new Matrix(result);
    }

    /** Returns this matrix minus another of the same shape. */
    public Matrix minus(Matrix other) {
        requireSameShape(other);
        double[][] result = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                result[r][c] = data[r][c] - other.data[r][c];
            }
        }
        return new Matrix(result);
    }

    /**
     * Returns the matrix product this * other. The number of columns in this matrix must equal the
     * number of rows in the other.
     */
    public Matrix times(Matrix other) {
        if (cols != other.rows) {
            throw new IllegalArgumentException(
                    "Cannot multiply a "
                            + rows
                            + "x"
                            + cols
                            + " by a "
                            + other.rows
                            + "x"
                            + other.cols);
        }
        double[][] result = new double[rows][other.cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < other.cols; c++) {
                double sum = 0.0;
                for (int k = 0; k < cols; k++) {
                    sum += data[r][k] * other.data[k][c];
                }
                result[r][c] = sum;
            }
        }
        return new Matrix(result);
    }

    /** Returns every entry multiplied by a single number. */
    public Matrix times(double scalar) {
        double[][] result = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                result[r][c] = data[r][c] * scalar;
            }
        }
        return new Matrix(result);
    }

    /** Returns the transpose: rows become columns and columns become rows. */
    public Matrix transpose() {
        double[][] result = new double[cols][rows];
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                result[c][r] = data[r][c];
            }
        }
        return new Matrix(result);
    }

    /**
     * Returns the inverse of a square matrix using Gauss-Jordan elimination with partial pivoting.
     * The inverse is the matrix that, multiplied by this one, gives the identity. Throws if the
     * matrix is not square or cannot be inverted.
     */
    public Matrix inverse() {
        if (rows != cols) {
            throw new IllegalArgumentException("Only square matrices can be inverted");
        }
        int n = rows;
        double[][] a = new double[n][n];
        double[][] inv = new double[n][n];
        for (int r = 0; r < n; r++) {
            System.arraycopy(data[r], 0, a[r], 0, n);
            inv[r][r] = 1.0;
        }

        for (int col = 0; col < n; col++) {
            // Find the row with the largest entry in this column and use it as the
            // pivot. This keeps the arithmetic stable.
            int pivot = col;
            for (int r = col + 1; r < n; r++) {
                if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) {
                    pivot = r;
                }
            }
            if (Math.abs(a[pivot][col]) < 1e-12) {
                throw new ArithmeticException("Matrix is singular and cannot be inverted");
            }
            swapRows(a, col, pivot);
            swapRows(inv, col, pivot);

            // Scale the pivot row so the pivot entry becomes 1.
            double pivotValue = a[col][col];
            for (int c = 0; c < n; c++) {
                a[col][c] /= pivotValue;
                inv[col][c] /= pivotValue;
            }

            // Clear this column out of every other row.
            for (int r = 0; r < n; r++) {
                if (r == col) {
                    continue;
                }
                double factor = a[r][col];
                for (int c = 0; c < n; c++) {
                    a[r][c] -= factor * a[col][c];
                    inv[r][c] -= factor * inv[col][c];
                }
            }
        }
        return new Matrix(inv);
    }

    private static void swapRows(double[][] m, int i, int j) {
        if (i == j) {
            return;
        }
        double[] tmp = m[i];
        m[i] = m[j];
        m[j] = tmp;
    }

    private void requireSameShape(Matrix other) {
        if (rows != other.rows || cols != other.cols) {
            throw new IllegalArgumentException("Matrix shapes do not match");
        }
    }
}
