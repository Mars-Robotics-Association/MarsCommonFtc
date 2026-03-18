// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math;

import edu.wpi.first.math.numbers.N1;

import java.util.Objects;

import org.ejml.MatrixDimensionException;
import org.ejml.data.Complex_F64;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.ejml.interfaces.linsol.LinearSolverDense;
import org.ejml.simple.SimpleMatrix;

/**
 * A shape-safe wrapper over Efficient Java Matrix Library (EJML) matrices.
 *
 * <p>This class is intended to be used alongside the state space library.
 *
 * @param <R> The number of rows in this matrix.
 * @param <C> The number of columns in this matrix.
 */
public class Matrix<R extends Num, C extends Num>
        //implements ProtobufSerializable, StructSerializable
{
    /**
     * Storage for underlying EJML matrix.
     */
    protected final SimpleMatrix m_storage;

    /**
     * Constructs an empty zero matrix of the given dimensions.
     *
     * @param rows    The number of rows of the matrix.
     * @param columns The number of columns of the matrix.
     */
    public Matrix(Nat<R> rows, Nat<C> columns) {
        this.m_storage =
                new SimpleMatrix(
                        Objects.requireNonNull(rows).getNum(), Objects.requireNonNull(columns).getNum());
    }

    /**
     * Constructs a new {@link Matrix} with the given storage. Caller should make sure that the
     * provided generic bounds match the shape of the provided {@link Matrix}.
     *
     * @param rows    The number of rows of the matrix.
     * @param columns The number of columns of the matrix.
     * @param storage The double array to back this value.
     */
    public Matrix(Nat<R> rows, Nat<C> columns, double[] storage) {
        this.m_storage = new SimpleMatrix(rows.getNum(), columns.getNum(), true, storage);
    }

    /**
     * Constructs a new {@link Matrix} with the given storage. Caller should make sure that the
     * provided generic bounds match the shape of the provided {@link Matrix}.
     *
     * <p>NOTE:It is not recommend to use this constructor unless the {@link SimpleMatrix} API is
     * absolutely necessary due to the desired function not being accessible through the {@link
     * Matrix} wrapper.
     *
     * @param storage The {@link SimpleMatrix} to back this value.
     */
    public Matrix(SimpleMatrix storage) {
        this.m_storage = Objects.requireNonNull(storage);
    }

    /**
     * Constructs a new matrix with the storage of the supplied matrix.
     *
     * @param other The {@link Matrix} to copy the storage of.
     */
    public Matrix(Matrix<R, C> other) {
        this.m_storage = Objects.requireNonNull(other).getStorage().copy();
    }

    /**
     * Gets the underlying {@link SimpleMatrix} that this {@link Matrix} wraps.
     *
     * <p>NOTE:The use of this method is heavily discouraged as this removes any guarantee of type
     * safety. This should only be called if the {@link SimpleMatrix} API is absolutely necessary due
     * to the desired function not being accessible through the {@link Matrix} wrapper.
     *
     * @return The underlying {@link SimpleMatrix} storage.
     */
    public SimpleMatrix getStorage() {
        return m_storage;
    }

    /**
     * Gets the number of columns in this matrix.
     *
     * @return The number of columns, according to the internal storage.
     */
    public final int getNumCols() {
        return this.m_storage.numCols();
    }

    /**
     * Gets the number of rows in this matrix.
     *
     * @return The number of rows, according to the internal storage.
     */
    public final int getNumRows() {
        return this.m_storage.numRows();
    }

    /**
     * Get an element of this matrix.
     *
     * @param row The row of the element.
     * @param col The column of the element.
     * @return The element in this matrix at row,col.
     */
    public final double get(int row, int col) {
        return this.m_storage.get(row, col);
    }

    /**
     * Sets the value at the given indices.
     *
     * @param row   The row of the element.
     * @param col   The column of the element.
     * @param value The value to insert at the given location.
     */
    public final void set(int row, int col, double value) {
        this.m_storage.set(row, col, value);
    }

    /**
     * Sets a row to a given row vector.
     *
     * @param row The row to set.
     * @param val The row vector to set the given row to.
     */
    public final void setRow(int row, Matrix<N1, C> val) {
        this.m_storage.setRow(row, 0, Objects.requireNonNull(val).m_storage.getDDRM().getData());
    }

    /**
     * Sets a column to a given column vector.
     *
     * @param column The column to set.
     * @param val    The column vector to set the given row to.
     */
    public final void setColumn(int column, Matrix<R, N1> val) {
        this.m_storage.setColumn(column, 0, Objects.requireNonNull(val).m_storage.getDDRM().getData());
    }

    /**
     * Sets all the elements in "this" matrix equal to the specified value.
     *
     * @param value The value each element is set to.
     */
    public void fill(double value) {
        this.m_storage.fill(value);
    }

    /**
     * Returns the diagonal elements inside a vector or square matrix.
     *
     * <p>If "this" {@link Matrix} is a vector then a square matrix is returned. If a "this" {@link
     * Matrix} is a matrix then a vector of diagonal elements is returned.
     *
     * @return The diagonal elements inside a vector or a square matrix.
     */
    public final Matrix<R, C> diag() {
        return new Matrix<>(this.m_storage.diag());
    }

    /**
     * Returns the largest element of this matrix.
     *
     * @return The largest element of this matrix.
     */
    public final double max() {
        return CommonOps_DDRM.elementMax(this.m_storage.getDDRM());
    }

    /**
     * Returns the absolute value of the element in this matrix with the largest absolute value.
     *
     * @return The absolute value of the element with the largest absolute value.
     */
    public final double maxAbs() {
        return CommonOps_DDRM.elementMaxAbs(this.m_storage.getDDRM());
    }

    /**
     * Returns the smallest element of this matrix.
     *
     * @return The smallest element of this matrix.
     */
    public final double minInternal() {
        return CommonOps_DDRM.elementMin(this.m_storage.getDDRM());
    }

    /**
     * Calculates the mean of the elements in this matrix.
     *
     * @return The mean value of this matrix.
     */
    public final double mean() {
        return this.elementSum() / this.m_storage.getNumElements();
    }

    /**
     * Multiplies this matrix with another that has C rows.
     *
     * <p>As matrix multiplication is only defined if the number of columns in the first matrix
     * matches the number of rows in the second, this operation will fail to compile under any other
     * circumstances.
     *
     * @param other The other matrix to multiply by.
     * @param <C2>  The number of columns in the second matrix.
     * @return The result of the matrix multiplication between "this" and the given matrix.
     */
    public final <C2 extends Num> Matrix<R, C2> times(Matrix<C, C2> other) {
        return new Matrix<>(this.m_storage.mult(Objects.requireNonNull(other).m_storage));
    }

    /**
     * Multiplies all the elements of this matrix by the given scalar.
     *
     * @param value The scalar value to multiply by.
     * @return A new matrix with all the elements multiplied by the given value.
     */
    public Matrix<R, C> times(double value) {
        return new Matrix<>(this.m_storage.scale(value));
    }

    /**
     * Returns a matrix which is the result of an element by element multiplication of "this" and
     * other.
     *
     * <p>c<sub>i,j</sub> = a<sub>i,j</sub>*other<sub>i,j</sub>
     *
     * @param other The other {@link Matrix} to perform element multiplication on.
     * @return The element by element multiplication of "this" and other.
     */
    public final Matrix<R, C> elementTimes(Matrix<R, C> other) {
        return new Matrix<>(this.m_storage.elementMult(Objects.requireNonNull(other).m_storage));
    }

    /**
     * Subtracts the given value from all the elements of this matrix.
     *
     * @param value The value to subtract.
     * @return The resultant matrix.
     */
    public final Matrix<R, C> minus(double value) {
        return new Matrix<>(this.m_storage.minus(value));
    }

    /**
     * Subtracts the given matrix from this matrix.
     *
     * @param value The matrix to subtract.
     * @return The resultant matrix.
     */
    public final Matrix<R, C> minus(Matrix<R, C> value) {
        return new Matrix<>(this.m_storage.minus(Objects.requireNonNull(value).m_storage));
    }

    /**
     * Adds the given value to all the elements of this matrix.
     *
     * @param value The value to add.
     * @return The resultant matrix.
     */
    public final Matrix<R, C> plus(double value) {
        return new Matrix<>(this.m_storage.plus(value));
    }

    /**
     * Adds the given matrix to this matrix.
     *
     * @param value The matrix to add.
     * @return The resultant matrix.
     */
    public final Matrix<R, C> plus(Matrix<R, C> value) {
        return new Matrix<>(this.m_storage.plus(Objects.requireNonNull(value).m_storage));
    }

    /**
     * Divides all elements of this matrix by the given value.
     *
     * @param value The value to divide by.
     * @return The resultant matrix.
     */
    public Matrix<R, C> div(int value) {
        return new Matrix<>(this.m_storage.divide(value));
    }

    /**
     * Divides all elements of this matrix by the given value.
     *
     * @param value The value to divide by.
     * @return The resultant matrix.
     */
    public Matrix<R, C> div(double value) {
        return new Matrix<>(this.m_storage.divide(value));
    }

    /**
     * Calculates the transpose, Mᵀ of this matrix.
     *
     * @return The transpose matrix.
     */
    public final Matrix<C, R> transpose() {
        return new Matrix<>(this.m_storage.transpose());
    }

    /**
     * Returns a copy of this matrix.
     *
     * @return A copy of this matrix.
     */
    public final Matrix<R, C> copy() {
        return new Matrix<>(this.m_storage.copy());
    }

    /**
     * Returns the inverse matrix of "this" matrix.
     *
     * @return The inverse of "this" matrix.
     * @throws org.ejml.data.SingularMatrixException If "this" matrix is non-invertable.
     */
    public final Matrix<R, C> inv() {
        return new Matrix<>(this.m_storage.invert());
    }

    /**
     * Returns the solution x to the equation Ax = b, where A is "this" matrix.
     *
     * <p>The matrix equation could also be written as x = A<sup>-1</sup>b. Where the pseudo inverse
     * is used if A is not square.
     *
     * <p>Note that this method does not support solving using a QR decomposition with full-pivoting,
     * as only column-pivoting is supported. For full-pivoting, use {@link
     * #solveFullPivHouseholderQr}.
     *
     * @param <C2> Columns in b.
     * @param b    The right-hand side of the equation to solve.
     * @return The solution to the linear system.
     */
    public final <C2 extends Num> Matrix<C, C2> solve(Matrix<R, C2> b) {
        return new Matrix<>(this.m_storage.solve(Objects.requireNonNull(b).m_storage));
    }

    /**
     * Solves the least-squares problem Ax=B using a QR decomposition with full pivoting, where this
     * matrix is A.
     *
     * @param <R2>  Number of rows in B.
     * @param <C2>  Number of columns in B.
     * @param other The B matrix.
     * @return The solution matrix.
     */
    public final <R2 extends Num, C2 extends Num> Matrix<C, C2> solveFullPivHouseholderQr(
            Matrix<R2, C2> other) {
        DMatrixRMaj A = this.m_storage.getDDRM().copy();
        DMatrixRMaj B = other.m_storage.getDDRM().copy();
        DMatrixRMaj X = new DMatrixRMaj(A.numCols, B.numCols);
        LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.leastSquaresQrPivot(true, false);
        if (!solver.setA(A)) {
            throw new RuntimeException("solveFullPivHouseholderQr: failed to decompose A");
        }
        solver.solve(B, X);
        return new Matrix<>(SimpleMatrix.wrap(X));
    }

    /**
     * Computes the matrix exponential. This method only works for square matrices, and will
     * otherwise throw an {@link MatrixDimensionException}.
     *
     * <p>Uses the degree-13 Padé approximant with scaling and squaring (Higham 2005).
     *
     * @return The exponential of A.
     */
    public final Matrix<R, C> exp() {
        if (this.getNumRows() != this.getNumCols()) {
            throw new MatrixDimensionException(
                    "Non-square matrices cannot be exponentiated! "
                            + "This matrix is "
                            + this.getNumRows()
                            + " x "
                            + this.getNumCols());
        }
        int n = this.getNumRows();
        DMatrixRMaj result = matExp(this.m_storage.getDDRM().copy(), n);
        return new Matrix<>(SimpleMatrix.wrap(result));
    }

    /**
     * Computes the matrix exponential using a degree-13 Padé approximant with scaling and
     * squaring (Higham 2005).
     *
     * @param A Input matrix (modified in place during computation; pass a copy).
     * @param n Dimension of the square matrix.
     * @return exp(A).
     */
    private static DMatrixRMaj matExp(DMatrixRMaj A, int n) {
        // Degree-13 Padé coefficients (Higham, "The Scaling and Squaring Method
        // for the Matrix Exponential Revisited", SIAM J. Matrix Anal. Appl. 2005).
        final double[] b = {
            64764752532480000.0, 32382376266240000.0, 7771770303897600.0,
            1187353796428800.0,  129060195264000.0,   10559470521600.0,
            670442572800.0,      33522128640.0,        1323241920.0,
            40840800.0,          960960.0,             16380.0,
            182.0,               1.0
        };
        final double theta13 = 5.371920351148152;

        // Determine scaling: scale A by 2^{-s} so ||A/2^s||_1 <= theta13.
        double normA = NormOps_DDRM.inducedP1(A);
        int s = 0;
        if (normA > theta13) {
            s = (int) Math.ceil(Math.log(normA / theta13) / Math.log(2.0));
            CommonOps_DDRM.scale(Math.pow(2.0, -s), A);
        }

        // Compute matrix powers: A2 = A^2, A4 = A^4, A6 = A^6.
        DMatrixRMaj I   = CommonOps_DDRM.identity(n);
        DMatrixRMaj A2  = new DMatrixRMaj(n, n);
        DMatrixRMaj A4  = new DMatrixRMaj(n, n);
        DMatrixRMaj A6  = new DMatrixRMaj(n, n);
        DMatrixRMaj tmp = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(A, A, A2);
        CommonOps_DDRM.mult(A2, A2, A4);
        CommonOps_DDRM.mult(A2, A4, A6);

        // U = A * (A6*(b[13]*A6 + b[11]*A4 + b[9]*A2) + b[7]*A6 + b[5]*A4 + b[3]*A2 + b[1]*I)
        DMatrixRMaj inner = new DMatrixRMaj(n, n);
        CommonOps_DDRM.scale(b[13], A6, inner);
        CommonOps_DDRM.addEquals(inner, b[11], A4);
        CommonOps_DDRM.addEquals(inner, b[9],  A2);
        CommonOps_DDRM.mult(A6, inner, tmp);
        CommonOps_DDRM.addEquals(tmp, b[7], A6);
        CommonOps_DDRM.addEquals(tmp, b[5], A4);
        CommonOps_DDRM.addEquals(tmp, b[3], A2);
        CommonOps_DDRM.addEquals(tmp, b[1], I);
        DMatrixRMaj U = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(A, tmp, U);

        // V = A6*(b[12]*A6 + b[10]*A4 + b[8]*A2) + b[6]*A6 + b[4]*A4 + b[2]*A2 + b[0]*I
        DMatrixRMaj Vtmp = new DMatrixRMaj(n, n);
        CommonOps_DDRM.scale(b[12], A6, Vtmp);
        CommonOps_DDRM.addEquals(Vtmp, b[10], A4);
        CommonOps_DDRM.addEquals(Vtmp, b[8],  A2);
        DMatrixRMaj V = new DMatrixRMaj(n, n);
        CommonOps_DDRM.mult(A6, Vtmp, V);
        CommonOps_DDRM.addEquals(V, b[6], A6);
        CommonOps_DDRM.addEquals(V, b[4], A4);
        CommonOps_DDRM.addEquals(V, b[2], A2);
        CommonOps_DDRM.addEquals(V, b[0], I);

        // exp(A/2^s) ≈ (V - U)^{-1} * (V + U).
        DMatrixRMaj P = new DMatrixRMaj(n, n); // V + U (numerator)
        DMatrixRMaj Q = new DMatrixRMaj(n, n); // V - U (denominator)
        CommonOps_DDRM.add(V, U, P);
        CommonOps_DDRM.subtract(V, U, Q);
        DMatrixRMaj R = new DMatrixRMaj(n, n);
        CommonOps_DDRM.solve(Q, P, R);

        // Squaring phase: exp(A) = exp(A/2^s)^{2^s}.
        DMatrixRMaj R2 = new DMatrixRMaj(n, n);
        for (int i = 0; i < s; i++) {
            CommonOps_DDRM.mult(R, R, R2);
            DMatrixRMaj swap = R;
            R = R2;
            R2 = swap;
        }
        return R;
    }

    /**
     * Computes the matrix power. This method only works for square matrices, and will otherwise throw
     * an {@link MatrixDimensionException}.
     *
     * <p>Uses eigendecomposition: A^p = V * D^p * V^{-1}. For complex conjugate eigenvalue pairs the
     * contribution is computed using real arithmetic to keep the result real.
     *
     * @param exponent The exponent.
     * @return A raised to the given power.
     */
    public final Matrix<R, C> pow(double exponent) {
        if (this.getNumRows() != this.getNumCols()) {
            throw new MatrixDimensionException(
                    "Non-square matrices cannot be raised to a power! "
                            + "This matrix is "
                            + this.getNumRows()
                            + " x "
                            + this.getNumCols());
        }
        int n = this.getNumRows();

        // Eigendecompose A.  EJML stores complex conjugate pairs consecutively:
        //   eigenvalue[k]   = a + bi  (b > 0)
        //   eigenvalue[k+1] = a - bi
        //   getEigenVector(k)   = Re(v),  getEigenVector(k+1) = Im(v)
        EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(n, true);
        eig.decompose(this.m_storage.getDDRM().copy());

        // Build the real eigenvector matrix V (columns = Re/Im parts as above).
        DMatrixRMaj V = new DMatrixRMaj(n, n);
        for (int i = 0; i < n; i++) {
            DMatrixRMaj vec = eig.getEigenVector(i);
            if (vec != null) {
                for (int row = 0; row < n; row++) {
                    V.set(row, i, vec.get(row, 0));
                }
            }
        }

        // V^{-1}: rows are the left-eigenvector duals.
        DMatrixRMaj Vinv = new DMatrixRMaj(n, n);
        CommonOps_DDRM.invert(V, Vinv);

        // Accumulate A^p = sum_k  lambda_k^p * outer(V[:,k], Vinv[k,:]).
        DMatrixRMaj result = new DMatrixRMaj(n, n);
        for (int i = 0; i < n; ) {
            Complex_F64 ev = eig.getEigenvalue(i);

            if (Math.abs(ev.imaginary) < 1e-10) {
                // ── Real eigenvalue ──────────────────────────────────────────
                double r  = Math.abs(ev.real);
                double rp = (r == 0.0) ? 0.0 : Math.pow(r, exponent);
                // For a negative real eigenvalue λ = -|λ|:
                //   λ^p = |λ|^p * e^{i·p·π};  real part = |λ|^p * cos(p·π).
                double lambda_p = (ev.real >= 0.0)
                        ? rp
                        : rp * Math.cos(exponent * Math.PI);

                // Contribution: lambda_p * outer(V[:,i], Vinv[i,:])
                for (int row = 0; row < n; row++) {
                    double vri = V.get(row, i);
                    for (int col = 0; col < n; col++) {
                        result.set(row, col, result.get(row, col)
                                + lambda_p * vri * Vinv.get(i, col));
                    }
                }
                i++;

            } else if (ev.imaginary > 0.0) {
                // ── Complex conjugate pair at (i, i+1) ───────────────────────
                // v_r = V[:,i],  v_i = V[:,i+1]
                // u_r = Vinv[i,:],  u_i = Vinv[i+1,:]
                // lambda^p = r^p * (cos(p*theta) + i*sin(p*theta))
                // Contribution (real part after conjugate summation):
                //   2 * [ (c*v_r - s*v_i)*u_r^T  +  (s*v_r + c*v_i)*u_i^T ]
                // where c = r^p*cos(p*theta), s = r^p*sin(p*theta).
                double r     = Math.hypot(ev.real, ev.imaginary);
                double theta = Math.atan2(ev.imaginary, ev.real);
                double rp    = (r == 0.0) ? 0.0 : Math.pow(r, exponent);
                double c     = rp * Math.cos(exponent * theta);
                double s     = rp * Math.sin(exponent * theta);

                for (int row = 0; row < n; row++) {
                    double vr = V.get(row, i);
                    double vi = V.get(row, i + 1);
                    double p1 = c * vr - s * vi;
                    double p2 = s * vr + c * vi;
                    for (int col = 0; col < n; col++) {
                        double ur = Vinv.get(i, col);
                        double ui = Vinv.get(i + 1, col);
                        result.set(row, col, result.get(row, col)
                                + 2.0 * (p1 * ur + p2 * ui));
                    }
                }
                i += 2; // skip the conjugate (negative imaginary) entry

            } else {
                // Negative-imaginary eigenvalue: already consumed as part of its
                // conjugate pair when we processed the positive-imaginary partner.
                i++;
            }
        }

        return new Matrix<>(SimpleMatrix.wrap(result));
    }

    /**
     * Returns the determinant of this matrix.
     *
     * @return The determinant of this matrix.
     */
    public final double det() {
        return this.m_storage.determinant();
    }

    /**
     * Computes the Frobenius normal of the matrix.
     *
     * <p>normF = Sqrt{ &sum;<sub>i=1:m</sub> &sum;<sub>j=1:n</sub> { a<sub>ij</sub><sup>2</sup>} }
     *
     * @return The matrix's Frobenius normal.
     */
    public final double normF() {
        return this.m_storage.normF();
    }

    /**
     * Computes the induced p = 1 matrix norm.
     *
     * <p>||A||<sub>1</sub>= max(j=1 to n; sum(i=1 to m; |a<sub>ij</sub>|))
     *
     * @return The norm.
     */
    public final double normIndP1() {
        return NormOps_DDRM.inducedP1(this.m_storage.getDDRM());
    }

    /**
     * Computes the sum of all the elements in the matrix.
     *
     * @return Sum of all the elements.
     */
    public final double elementSum() {
        return this.m_storage.elementSum();
    }

    /**
     * Computes the trace of the matrix.
     *
     * @return The trace of the matrix.
     */
    public final double trace() {
        return this.m_storage.trace();
    }

    /**
     * Returns a matrix which is the result of an element by element power of "this" and b.
     *
     * <p>c<sub>i,j</sub> = a<sub>i,j</sub> ^ b
     *
     * @param b Scalar.
     * @return The element by element power of "this" and b.
     */
    public final Matrix<R, C> elementPower(double b) {
        return new Matrix<>(this.m_storage.elementPower(b));
    }

    /**
     * Returns a matrix which is the result of an element by element power of "this" and b.
     *
     * <p>c<sub>i,j</sub> = a<sub>i,j</sub> ^ b
     *
     * @param b Scalar.
     * @return The element by element power of "this" and b.
     */
    public final Matrix<R, C> elementPower(int b) {
        return new Matrix<>(this.m_storage.elementPower(b));
    }

    /**
     * Extracts a given row into a row vector with new underlying storage.
     *
     * @param row The row to extract a vector from.
     * @return A row vector from the given row.
     */
    public final Matrix<N1, C> extractRowVector(int row) {
        return new Matrix<>(this.m_storage.extractVector(true, row));
    }

    /**
     * Extracts a given column into a column vector with new underlying storage.
     *
     * @param column The column to extract a vector from.
     * @return A column vector from the given column.
     */
    public final Matrix<R, N1> extractColumnVector(int column) {
        return new Matrix<>(this.m_storage.extractVector(false, column));
    }

    /**
     * Extracts a matrix of a given size and start position with new underlying storage.
     *
     * @param <R2>        Number of rows to extract.
     * @param <C2>        Number of columns to extract.
     * @param height      The number of rows of the extracted matrix.
     * @param width       The number of columns of the extracted matrix.
     * @param startingRow The starting row of the extracted matrix.
     * @param startingCol The starting column of the extracted matrix.
     * @return The extracted matrix.
     */
    public final <R2 extends Num, C2 extends Num> Matrix<R2, C2> block(
            Nat<R2> height, Nat<C2> width, int startingRow, int startingCol) {
        return new Matrix<>(
                this.m_storage.extractMatrix(
                        startingRow,
                        startingRow + Objects.requireNonNull(height).getNum(),
                        startingCol,
                        startingCol + Objects.requireNonNull(width).getNum()));
    }

    /**
     * Extracts a matrix of a given size and start position with new underlying storage.
     *
     * @param <R2>        Number of rows to extract.
     * @param <C2>        Number of columns to extract.
     * @param height      The number of rows of the extracted matrix.
     * @param width       The number of columns of the extracted matrix.
     * @param startingRow The starting row of the extracted matrix.
     * @param startingCol The starting column of the extracted matrix.
     * @return The extracted matrix.
     */
    public final <R2 extends Num, C2 extends Num> Matrix<R2, C2> block(
            int height, int width, int startingRow, int startingCol) {
        return new Matrix<>(
                this.m_storage.extractMatrix(
                        startingRow, startingRow + height, startingCol, startingCol + width));
    }

    /**
     * Assign a matrix of a given size and start position.
     *
     * @param <R2>        Rows in block assignment.
     * @param <C2>        Columns in block assignment.
     * @param startingRow The row to start at.
     * @param startingCol The column to start at.
     * @param other       The matrix to assign the block to.
     */
    public <R2 extends Num, C2 extends Num> void assignBlock(
            int startingRow, int startingCol, Matrix<R2, C2> other) {
        this.m_storage.insertIntoThis(
                startingRow, startingCol, Objects.requireNonNull(other).m_storage);
    }

    /**
     * Extracts a submatrix from the supplied matrix and inserts it in a submatrix in "this". The
     * shape of "this" is used to determine the size of the matrix extracted.
     *
     * @param <R2>        Number of rows to extract.
     * @param <C2>        Number of columns to extract.
     * @param startingRow The starting row in the supplied matrix to extract the submatrix.
     * @param startingCol The starting column in the supplied matrix to extract the submatrix.
     * @param other       The matrix to extract the submatrix from.
     */
    public <R2 extends Num, C2 extends Num> void extractFrom(
            int startingRow, int startingCol, Matrix<R2, C2> other) {
        CommonOps_DDRM.extract(
                other.m_storage.getDDRM(), startingRow, startingCol, this.m_storage.getDDRM());
    }

    /**
     * Decompose "this" matrix using Cholesky Decomposition. If the "this" matrix is zeros, it will
     * return the zero matrix.
     *
     * @param lowerTriangular Whether we want to decompose to the lower triangular Cholesky matrix.
     * @return The decomposed matrix.
     * @throws RuntimeException if the matrix could not be decomposed(i.e. is not positive
     *                          semidefinite).
     */
    public Matrix<R, C> lltDecompose(boolean lowerTriangular) {
        SimpleMatrix temp = m_storage.copy();

        CholeskyDecomposition_F64<DMatrixRMaj> chol =
                DecompositionFactory_DDRM.chol(temp.numRows(), lowerTriangular);
        if (!chol.decompose(temp.getMatrix())) {
            // check that the input is not all zeros -- if they are, we special case and return all
            // zeros.
            var matData = temp.getDDRM().data;
            var isZeros = true;
            for (double matDatum : matData) {
                isZeros &= Math.abs(matDatum) < 1e-6;
            }
            if (isZeros) {
                return new Matrix<>(new SimpleMatrix(temp.numRows(), temp.numCols()));
            }

            throw new RuntimeException("Cholesky decomposition failed! Input matrix:\n" + m_storage);
        }

        return new Matrix<>(SimpleMatrix.wrap(chol.getT(null)));
    }

    /**
     * Returns the row major data of this matrix as a double array.
     *
     * @return The row major data of this matrix as a double array.
     */
    public double[] getData() {
        return m_storage.getDDRM().getData();
    }

    /**
     * Creates the identity matrix of the given dimension.
     *
     * @param dim The dimension of the desired matrix as a {@link Nat}.
     * @param <D> The dimension of the desired matrix as a generic.
     * @return The DxD identity matrix.
     */
    public static <D extends Num> Matrix<D, D> eye(Nat<D> dim) {
        return new Matrix<>(SimpleMatrix.identity(Objects.requireNonNull(dim).getNum()));
    }

    /**
     * Creates the identity matrix of the given dimension.
     *
     * @param dim The dimension of the desired matrix as a {@link Num}.
     * @param <D> The dimension of the desired matrix as a generic.
     * @return The DxD identity matrix.
     */
    public static <D extends Num> Matrix<D, D> eye(D dim) {
        return new Matrix<>(SimpleMatrix.identity(Objects.requireNonNull(dim).getNum()));
    }

    /**
     * Reassigns dimensions of a {@link Matrix} to allow for operations with other matrices that have
     * wildcard dimensions.
     *
     * @param <R1> Row dimension to assign.
     * @param <C1> Column dimension to assign.
     * @param mat  The {@link Matrix} to remove the dimensions from.
     * @return The matrix with reassigned dimensions.
     */
    public static <R1 extends Num, C1 extends Num> Matrix<R1, C1> changeBoundsUnchecked(
            Matrix<?, ?> mat) {
        return new Matrix<>(mat.m_storage);
    }

    /**
     * Checks if another {@link Matrix} is identical to "this" one within a specified tolerance.
     *
     * <p>This will check if each element is in tolerance of the corresponding element from the other
     * {@link Matrix} or if the elements have the same symbolic meaning. For two elements to have the
     * same symbolic meaning they both must be either Double.NaN, Double.POSITIVE_INFINITY, or
     * Double.NEGATIVE_INFINITY.
     *
     * <p>NOTE:It is recommended to use {@link Matrix#isEqual(Matrix, double)} over this method when
     * checking if two matrices are equal as {@link Matrix#isEqual(Matrix, double)} will return false
     * if an element is uncountable. This method should only be used when uncountable elements need to
     * be compared.
     *
     * @param other     The {@link Matrix} to check against this one.
     * @param tolerance The tolerance to check equality with.
     * @return true if this matrix is identical to the one supplied.
     */
    public boolean isIdentical(Matrix<?, ?> other, double tolerance) {
        return MatrixFeatures_DDRM.isIdentical(
                this.m_storage.getDDRM(), other.m_storage.getDDRM(), tolerance);
    }

    /**
     * Checks if another {@link Matrix} is equal to "this" within a specified tolerance.
     *
     * <p>This will check if each element is in tolerance of the corresponding element from the other
     * {@link Matrix}.
     *
     * <p>tol &ge; |a<sub>ij</sub> - b<sub>ij</sub>|
     *
     * @param other     The {@link Matrix} to check against this one.
     * @param tolerance The tolerance to check equality with.
     * @return true if this matrix is equal to the one supplied.
     */
    public boolean isEqual(Matrix<?, ?> other, double tolerance) {
        return MatrixFeatures_DDRM.isEquals(
                this.m_storage.getDDRM(), other.m_storage.getDDRM(), tolerance);
    }

    /**
     * Performs an inplace Cholesky rank update (or downdate).
     *
     * <p>If this matrix contains L where A = LLᵀ before the update, it will contain L where LLᵀ = A +
     * σvvᵀ after the update.
     *
     * @param v               Vector to use for the update.
     * @param sigma           Sigma to use for the update.
     * @param lowerTriangular Whether this matrix is lower triangular.
     */
    public void rankUpdate(Matrix<R, N1> v, double sigma, boolean lowerTriangular) {
        int n = this.getNumRows();
        double[] mat = this.getData();
        double[] vec = v.getData();
        if (lowerTriangular) {
            lltRankUpdateLower(mat, n, vec, sigma);
        } else {
            lltRankUpdateUpper(mat, n, vec, sigma);
        }
    }

    /**
     * In-place rank-1 update/downdate of a lower-triangular Cholesky factor in row-major storage.
     *
     * <p>If L satisfies A = LLᵀ before the call, afterwards LLᵀ = A + sigma * v * vᵀ.
     *
     * <p>Uses Givens rotations for updates (sigma &gt; 0) and a direct recurrence for downdates
     * (sigma &lt; 0). Returns early without modifying the matrix further if a downdate would make
     * it non-positive-definite.
     *
     * @param mat   Row-major backing array of the n×n lower-triangular matrix, modified in place.
     * @param n     Dimension of the matrix.
     * @param vec   Update vector v of length n.
     * @param sigma Scaling factor; positive for update, negative for downdate.
     */
    private static void lltRankUpdateLower(double[] mat, int n, double[] vec, double sigma) {
        double[] temp = new double[n];
        if (sigma > 0) {
            // Givens-rotation-based rank-1 update.
            double sqrtSigma = Math.sqrt(sigma);
            for (int i = 0; i < n; i++) {
                temp[i] = sqrtSigma * vec[i];
            }
            for (int i = 0; i < n; i++) {
                // Compute Givens rotation that maps [L[i,i], -temp[i]] -> [r, 0].
                double a = mat[i * n + i];
                double b = -temp[i];
                double r = Math.hypot(a, b);
                double c, s;
                if (r == 0.0) {
                    c = 1.0;
                    s = 0.0;
                } else {
                    c = a / r;
                    s = b / r;
                }
                mat[i * n + i] = r;
                // Apply rotation to column-i tail and temp tail.
                // Note: column access in row-major has stride n (cache-unfriendly),
                // matching Eigen's behavior with the same storage layout.
                for (int k = i + 1; k < n; k++) {
                    double x = mat[k * n + i];
                    double y = temp[k];
                    mat[k * n + i] = c * x + s * y;
                    temp[k] = -s * x + c * y;
                }
            }
        } else {
            // Rank-1 downdate (sigma < 0).
            System.arraycopy(vec, 0, temp, 0, n);
            double beta = 1.0;
            for (int j = 0; j < n; j++) {
                double Ljj = mat[j * n + j];
                double dj = Ljj * Ljj;
                double wj = temp[j];
                double swj2 = sigma * wj * wj;
                double gamma = dj * beta + swj2;
                double x = dj + swj2 / beta;
                if (x <= 0.0) {
                    // Downdate would make the matrix non-positive-definite; abort.
                    return;
                }
                double nLjj = Math.sqrt(x);
                mat[j * n + j] = nLjj;
                beta += swj2 / dj;
                // Update temp first, then column (order matters for correctness).
                double wjOverLjj = wj / Ljj;
                double colScale = nLjj / Ljj;
                double tempScale = (gamma != 0.0) ? nLjj * sigma * wj / gamma : 0.0;
                for (int k = j + 1; k < n; k++) {
                    double Lkj = mat[k * n + j];
                    temp[k] -= wjOverLjj * Lkj;
                    mat[k * n + j] = colScale * Lkj + tempScale * temp[k];
                }
            }
        }
    }

    /**
     * In-place rank-1 update/downdate of an upper-triangular Cholesky factor in row-major storage.
     *
     * <p>Equivalent to transposing the upper triangle and applying {@link #lltRankUpdateLower}.
     * Row access in the upper triangle is contiguous in row-major storage, so the access pattern
     * here is cache-friendly (unlike the lower-triangular case).
     *
     * @param mat   Row-major backing array of the n×n upper-triangular matrix, modified in place.
     * @param n     Dimension of the matrix.
     * @param vec   Update vector v of length n.
     * @param sigma Scaling factor; positive for update, negative for downdate.
     */
    private static void lltRankUpdateUpper(double[] mat, int n, double[] vec, double sigma) {
        double[] temp = new double[n];
        if (sigma > 0) {
            // Givens-rotation-based rank-1 update on rows (transposed-lower view).
            double sqrtSigma = Math.sqrt(sigma);
            for (int i = 0; i < n; i++) {
                temp[i] = sqrtSigma * vec[i];
            }
            for (int i = 0; i < n; i++) {
                double a = mat[i * n + i];
                double b = -temp[i];
                double r = Math.hypot(a, b);
                double c, s;
                if (r == 0.0) {
                    c = 1.0;
                    s = 0.0;
                } else {
                    c = a / r;
                    s = b / r;
                }
                mat[i * n + i] = r;
                // Row-i tail is contiguous in row-major storage.
                for (int k = i + 1; k < n; k++) {
                    double x = mat[i * n + k];
                    double y = temp[k];
                    mat[i * n + k] = c * x + s * y;
                    temp[k] = -s * x + c * y;
                }
            }
        } else {
            // Rank-1 downdate on rows (transposed-lower view).
            System.arraycopy(vec, 0, temp, 0, n);
            double beta = 1.0;
            for (int j = 0; j < n; j++) {
                double Ujj = mat[j * n + j];
                double dj = Ujj * Ujj;
                double wj = temp[j];
                double swj2 = sigma * wj * wj;
                double gamma = dj * beta + swj2;
                double x = dj + swj2 / beta;
                if (x <= 0.0) {
                    return;
                }
                double nUjj = Math.sqrt(x);
                mat[j * n + j] = nUjj;
                beta += swj2 / dj;
                double wjOverUjj = wj / Ujj;
                double rowScale = nUjj / Ujj;
                double tempScale = (gamma != 0.0) ? nUjj * sigma * wj / gamma : 0.0;
                for (int k = j + 1; k < n; k++) {
                    double Ujk = mat[j * n + k];
                    temp[k] -= wjOverUjj * Ujk;
                    mat[j * n + k] = rowScale * Ujk + tempScale * temp[k];
                }
            }
        }
    }

    @Override
    public String toString() {
        return m_storage.toString();
    }

    /**
     * Checks if an object is equal to this {@link Matrix}.
     *
     * <p>a<sub>ij</sub> == b<sub>ij</sub>
     *
     * @param other The Object to check against this {@link Matrix}.
     * @return true if the object supplied is a {@link Matrix} and is equal to this matrix.
     */
    @Override
    public boolean equals(Object other) {
        return this == other
                || other instanceof Matrix<?, ?> matrix
                && !MatrixFeatures_DDRM.hasUncountable(matrix.m_storage.getDDRM())
                && MatrixFeatures_DDRM.isEquals(this.m_storage.getDDRM(), matrix.m_storage.getDDRM());
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_storage);
    }

    /**
     * Creates an implementation of the {@link Protobuf} interface for matrices.
     *
     * @param <R>  The number of rows of the matrices this serializer processes.
     * @param <C>  The number of cols of the matrices this serializer processes.
     * @param rows The number of rows of the matrices this serializer processes.
     * @param cols The number of cols of the matrices this serializer processes.
     * @return The protobuf implementation.
     */
//    public static <R extends Num, C extends Num> MatrixProto<R, C> getProto(
//            Nat<R> rows, Nat<C> cols) {
//        return new MatrixProto<>(rows, cols);
//    }

    /**
     * Creates an implementation of the {@link Struct} interfaces for matrices.
     *
     * @param <R>  The number of rows of the matrices this serializer processes.
     * @param <C>  The number of cols of the matrices this serializer processes.
     * @param rows The number of rows of the matrices this serializer processes.
     * @param cols The number of cols of the matrices this serializer processes.
     * @return The struct implementation.
     */
//    public static <R extends Num, C extends Num> MatrixStruct<R, C> getStruct(
//            Nat<R> rows, Nat<C> cols) {
//        return new MatrixStruct<>(rows, cols);
//    }
}
