// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.first.math;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.CholeskyDecomposition_F64;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.ejml.simple.SimpleMatrix;

/** DARE solver utility functions. */
public final class DARE {
  private DARE() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Computes the unique stabilizing solution X to the discrete-time algebraic Riccati equation.
   *
   * <p>AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
   *
   * <p>This internal function skips expensive precondition checks for increased performance. The
   * solver may hang if any of the following occur:
   *
   * <ul>
   *   <li>Q isn't symmetric positive semidefinite
   *   <li>R isn't symmetric positive definite
   *   <li>The (A, B) pair isn't stabilizable
   *   <li>The (A, C) pair where Q = CᵀC isn't detectable
   * </ul>
   *
   * <p>Only use this function if you're sure the preconditions are met.
   *
   * @param <States> Number of states.
   * @param <Inputs> Number of inputs.
   * @param A System matrix.
   * @param B Input matrix.
   * @param Q State cost matrix.
   * @param R Input cost matrix.
   * @return Solution of DARE.
   */
  public static <States extends Num, Inputs extends Num> Matrix<States, States> dareNoPrecond(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<States, States> Q,
      Matrix<Inputs, Inputs> R) {
    DMatrixRMaj result =
        sdaCore(
            A.getStorage().getDDRM(),
            B.getStorage().getDDRM(),
            Q.getStorage().getDDRM(),
            R.getStorage().getDDRM());
    return new Matrix<>(new SimpleMatrix(result));
  }

  /**
   * Computes the unique stabilizing solution X to the discrete-time algebraic Riccati equation.
   *
   * <p>AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0
   *
   * <p>This is equivalent to solving the original DARE:
   *
   * <p>A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0
   *
   * <p>where A₂ and Q₂ are a change of variables:
   *
   * <p>A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ
   *
   * <p>This overload of the DARE is useful for finding the control law uₖ that minimizes the
   * following cost function subject to xₖ₊₁ = Axₖ + Buₖ.
   *
   * <pre>
   *     ∞ [xₖ]ᵀ[Q  N][xₖ]
   * J = Σ [uₖ] [Nᵀ R][uₖ] ΔT
   *    k=0
   * </pre>
   *
   * <p>This is a more general form of the following. The linear-quadratic regulator is the feedback
   * control law uₖ that minimizes the following cost function subject to xₖ₊₁ = Axₖ + Buₖ:
   *
   * <pre>
   *     ∞
   * J = Σ (xₖᵀQxₖ + uₖᵀRuₖ) ΔT
   *    k=0
   * </pre>
   *
   * <p>This can be refactored as:
   *
   * <pre>
   *     ∞ [xₖ]ᵀ[Q 0][xₖ]
   * J = Σ [uₖ] [0 R][uₖ] ΔT
   *    k=0
   * </pre>
   *
   * <p>This internal function skips expensive precondition checks for increased performance. The
   * solver may hang if any of the following occur:
   *
   * <ul>
   *   <li>Q₂ isn't symmetric positive semidefinite
   *   <li>R isn't symmetric positive definite
   *   <li>The (A₂, B) pair isn't stabilizable
   *   <li>The (A₂, C) pair where Q₂ = CᵀC isn't detectable
   * </ul>
   *
   * <p>Only use this function if you're sure the preconditions are met.
   *
   * @param <States> Number of states.
   * @param <Inputs> Number of inputs.
   * @param A System matrix.
   * @param B Input matrix.
   * @param Q State cost matrix.
   * @param R Input cost matrix.
   * @param N State-input cross-term cost matrix.
   * @return Solution of DARE.
   */
  public static <States extends Num, Inputs extends Num> Matrix<States, States> dareNoPrecond(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<States, States> Q,
      Matrix<Inputs, Inputs> R,
      Matrix<States, Inputs> N) {
    DMatrixRMaj Amat = A.getStorage().getDDRM();
    DMatrixRMaj Bmat = B.getStorage().getDDRM();
    DMatrixRMaj Qmat = Q.getStorage().getDDRM();
    DMatrixRMaj Rmat = R.getStorage().getDDRM();
    DMatrixRMaj Nmat = N.getStorage().getDDRM();

    // A₂ = A − BR⁻¹Nᵀ, Q₂ = Q − NR⁻¹Nᵀ
    DMatrixRMaj[] transformed = applyNTransform(Amat, Bmat, Qmat, Rmat, Nmat);
    DMatrixRMaj A2 = transformed[0];
    DMatrixRMaj Q2 = transformed[1];

    DMatrixRMaj result = sdaCore(A2, Bmat, Q2, Rmat);
    return new Matrix<>(new SimpleMatrix(result));
  }

  /**
   * Computes the unique stabilizing solution X to the discrete-time algebraic Riccati equation.
   *
   * <p>AᵀXA − X − AᵀXB(BᵀXB + R)⁻¹BᵀXA + Q = 0
   *
   * @param <States> Number of states.
   * @param <Inputs> Number of inputs.
   * @param A System matrix.
   * @param B Input matrix.
   * @param Q State cost matrix.
   * @param R Input cost matrix.
   * @return Solution of DARE.
   * @throws IllegalArgumentException if Q isn't symmetric positive semidefinite.
   * @throws IllegalArgumentException if R isn't symmetric positive definite.
   * @throws IllegalArgumentException if the (A, B) pair isn't stabilizable.
   * @throws IllegalArgumentException if the (A, C) pair where Q = CᵀC isn't detectable.
   */
  public static <States extends Num, Inputs extends Num> Matrix<States, States> dare(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<States, States> Q,
      Matrix<Inputs, Inputs> R) {
    DMatrixRMaj Amat = A.getStorage().getDDRM();
    DMatrixRMaj Bmat = B.getStorage().getDDRM();
    DMatrixRMaj Qmat = Q.getStorage().getDDRM();
    DMatrixRMaj Rmat = R.getStorage().getDDRM();

    checkPreconditions(Amat, Bmat, Qmat, Rmat);
    DMatrixRMaj result = sdaCore(Amat, Bmat, Qmat, Rmat);
    return new Matrix<>(new SimpleMatrix(result));
  }

  /**
   * Computes the unique stabilizing solution X to the discrete-time algebraic Riccati equation.
   *
   * <p>AᵀXA − X − (AᵀXB + N)(BᵀXB + R)⁻¹(BᵀXA + Nᵀ) + Q = 0
   *
   * <p>This is equivalent to solving the original DARE:
   *
   * <p>A₂ᵀXA₂ − X − A₂ᵀXB(BᵀXB + R)⁻¹BᵀXA₂ + Q₂ = 0
   *
   * <p>where A₂ and Q₂ are a change of variables:
   *
   * <p>A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ
   *
   * <p>This overload of the DARE is useful for finding the control law uₖ that minimizes the
   * following cost function subject to xₖ₊₁ = Axₖ + Buₖ.
   *
   * <pre>
   *     ∞ [xₖ]ᵀ[Q  N][xₖ]
   * J = Σ [uₖ] [Nᵀ R][uₖ] ΔT
   *    k=0
   * </pre>
   *
   * <p>This is a more general form of the following. The linear-quadratic regulator is the feedback
   * control law uₖ that minimizes the following cost function subject to xₖ₊₁ = Axₖ + Buₖ:
   *
   * <pre>
   *     ∞
   * J = Σ (xₖᵀQxₖ + uₖᵀRuₖ) ΔT
   *    k=0
   * </pre>
   *
   * <p>This can be refactored as:
   *
   * <pre>
   *     ∞ [xₖ]ᵀ[Q 0][xₖ]
   * J = Σ [uₖ] [0 R][uₖ] ΔT
   *    k=0
   * </pre>
   *
   * @param <States> Number of states.
   * @param <Inputs> Number of inputs.
   * @param A System matrix.
   * @param B Input matrix.
   * @param Q State cost matrix.
   * @param R Input cost matrix.
   * @param N State-input cross-term cost matrix.
   * @return Solution of DARE.
   * @throws IllegalArgumentException if Q₂ isn't symmetric positive semidefinite.
   * @throws IllegalArgumentException if R isn't symmetric positive definite.
   * @throws IllegalArgumentException if the (A₂, B) pair isn't stabilizable.
   * @throws IllegalArgumentException if the (A₂, C) pair where Q₂ = CᵀC isn't detectable.
   */
  public static <States extends Num, Inputs extends Num> Matrix<States, States> dare(
      Matrix<States, States> A,
      Matrix<States, Inputs> B,
      Matrix<States, States> Q,
      Matrix<Inputs, Inputs> R,
      Matrix<States, Inputs> N) {
    DMatrixRMaj Amat = A.getStorage().getDDRM();
    DMatrixRMaj Bmat = B.getStorage().getDDRM();
    DMatrixRMaj Qmat = Q.getStorage().getDDRM();
    DMatrixRMaj Rmat = R.getStorage().getDDRM();
    DMatrixRMaj Nmat = N.getStorage().getDDRM();

    // Check R symmetric before attempting to compute R⁻¹
    checkRSymmetric(Rmat);
    // Check R positive definite (R⁻¹ is needed for the transform)
    checkRPD(Rmat);

    // A₂ = A − BR⁻¹Nᵀ, Q₂ = Q − NR⁻¹Nᵀ
    DMatrixRMaj[] transformed = applyNTransform(Amat, Bmat, Qmat, Rmat, Nmat);
    DMatrixRMaj A2 = transformed[0];
    DMatrixRMaj Q2 = transformed[1];

    // Check remaining preconditions on the transformed matrices
    checkQSymmetric(Q2);
    checkQPSD(Q2);
    checkStabilizability(A2, Bmat);
    checkDetectability(A2, Q2);

    DMatrixRMaj result = sdaCore(A2, Bmat, Q2, Rmat);
    return new Matrix<>(new SimpleMatrix(result));
  }

  /**
   * Computes A₂ = A − BR⁻¹Nᵀ and Q₂ = Q − NR⁻¹Nᵀ.
   *
   * @return Array of {A₂, Q₂}.
   */
  private static DMatrixRMaj[] applyNTransform(
      DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj Q, DMatrixRMaj R, DMatrixRMaj N) {
    int n = A.numRows;
    int m = B.numCols;

    // Solve R * X = Nᵀ → X = R⁻¹Nᵀ
    DMatrixRMaj NT = new DMatrixRMaj(m, n);
    CommonOps_DDRM.transpose(N, NT);
    DMatrixRMaj RinvNT = new DMatrixRMaj(m, n);
    CommonOps_DDRM.solve(R.copy(), NT, RinvNT);

    // A₂ = A − B(R⁻¹Nᵀ)
    DMatrixRMaj BRinvNT = new DMatrixRMaj(n, n);
    CommonOps_DDRM.mult(B, RinvNT, BRinvNT);
    DMatrixRMaj A2 = new DMatrixRMaj(n, n);
    CommonOps_DDRM.subtract(A, BRinvNT, A2);

    // Q₂ = Q − N(R⁻¹Nᵀ)
    DMatrixRMaj NRinvNT = new DMatrixRMaj(n, n);
    CommonOps_DDRM.mult(N, RinvNT, NRinvNT);
    DMatrixRMaj Q2 = new DMatrixRMaj(n, n);
    CommonOps_DDRM.subtract(Q, NRinvNT, Q2);

    return new DMatrixRMaj[] {A2, Q2};
  }

  /**
   * Checks all preconditions for dare(A, B, Q, R): R symmetric, R PD, Q symmetric, Q PSD,
   * stabilizability, and detectability.
   */
  private static void checkPreconditions(
      DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj Q, DMatrixRMaj R) {
    checkRSymmetric(R);
    checkRPD(R);
    checkQSymmetric(Q);
    checkQPSD(Q);
    checkStabilizability(A, B);
    checkDetectability(A, Q);
  }

  private static void checkRSymmetric(DMatrixRMaj R) {
    DMatrixRMaj RT = new DMatrixRMaj(R.numCols, R.numRows);
    CommonOps_DDRM.transpose(R, RT);
    DMatrixRMaj diff = new DMatrixRMaj(R.numRows, R.numCols);
    CommonOps_DDRM.subtract(R, RT, diff);
    if (NormOps_DDRM.normF(diff) > 1e-10) {
      throw new IllegalArgumentException("R is not symmetric.");
    }
  }

  private static void checkRPD(DMatrixRMaj R) {
    CholeskyDecomposition_F64<DMatrixRMaj> chol =
        DecompositionFactory_DDRM.chol(R.numRows, true);
    if (!chol.decompose(R.copy())) {
      throw new IllegalArgumentException("R is not positive definite.");
    }
  }

  private static void checkQSymmetric(DMatrixRMaj Q) {
    DMatrixRMaj QT = new DMatrixRMaj(Q.numCols, Q.numRows);
    CommonOps_DDRM.transpose(Q, QT);
    DMatrixRMaj diff = new DMatrixRMaj(Q.numRows, Q.numCols);
    CommonOps_DDRM.subtract(Q, QT, diff);
    if (NormOps_DDRM.normF(diff) > 1e-10) {
      throw new IllegalArgumentException("Q is not symmetric.");
    }
  }

  private static void checkQPSD(DMatrixRMaj Q) {
    int n = Q.numRows;
    EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(n, false, true);
    eig.decompose(Q.copy());
    for (int i = 0; i < n; i++) {
      if (eig.getEigenvalue(i).real < 0.0) {
        throw new IllegalArgumentException("Q is not positive semidefinite.");
      }
    }
  }

  private static void checkStabilizability(DMatrixRMaj A, DMatrixRMaj B) {
    if (!StateSpaceUtil.isStabilizable(A, B)) {
      throw new IllegalArgumentException("The (A, B) pair is not stabilizable.");
    }
  }

  /**
   * Checks detectability of (A, C) where Q = CᵀC, by extracting C via symmetric eigendecomposition
   * Q = VDVᵀ → C = √D · Vᵀ, then checking isStabilizable(Aᵀ, Cᵀ).
   */
  private static void checkDetectability(DMatrixRMaj A, DMatrixRMaj Q) {
    int n = Q.numRows;

    // Q = VDVᵀ, C = √D · Vᵀ  (so CᵀC = V√D·√DVᵀ = VDVᵀ = Q)
    EigenDecomposition_F64<DMatrixRMaj> eig = DecompositionFactory_DDRM.eig(n, true, true);
    eig.decompose(Q.copy());

    // Build C: row i = √λᵢ · vᵢᵀ
    DMatrixRMaj C = new DMatrixRMaj(n, n);
    for (int i = 0; i < n; i++) {
      double sqrtEv = Math.sqrt(Math.max(0.0, eig.getEigenvalue(i).real));
      DMatrixRMaj v = eig.getEigenVector(i);
      for (int j = 0; j < n; j++) {
        C.set(i, j, sqrtEv * v.get(j, 0));
      }
    }

    // isDetectable(A, C) = isStabilizable(Aᵀ, Cᵀ)
    DMatrixRMaj AT = new DMatrixRMaj(n, n);
    CommonOps_DDRM.transpose(A, AT);
    DMatrixRMaj CT = new DMatrixRMaj(n, n);
    CommonOps_DDRM.transpose(C, CT);

    if (!StateSpaceUtil.isStabilizable(AT, CT)) {
      throw new IllegalArgumentException(
          "The (A, C) pair where Q = CᵀC is not detectable.");
    }
  }

  /**
   * Implements the Structured Doubling Algorithm (SDA) for the DARE.
   *
   * <p>Reference: E. K.-W. Chu, H.-Y. Fan, W.-W. Lin & C.-S. Wang, "Structure-Preserving
   * Algorithms for Periodic Discrete-Time Algebraic Riccati Equations", International Journal of
   * Control, 77:8, 767-788, 2004. DOI: 10.1080/00207170410001714988
   *
   * @param A System matrix (n × n).
   * @param B Input matrix (n × m).
   * @param Q State cost matrix (n × n), symmetric PSD.
   * @param R Input cost matrix (m × m), symmetric PD.
   * @return Solution X to the DARE (n × n).
   */
  private static DMatrixRMaj sdaCore(
      DMatrixRMaj A, DMatrixRMaj B, DMatrixRMaj Q, DMatrixRMaj R) {
    int n = A.numRows;

    // G₀ = B R⁻¹ Bᵀ
    // Solve R * X = Bᵀ → X = R⁻¹Bᵀ
    DMatrixRMaj BT = new DMatrixRMaj(B.numCols, B.numRows);
    CommonOps_DDRM.transpose(B, BT);
    DMatrixRMaj RinvBT = new DMatrixRMaj(R.numRows, BT.numCols);
    CommonOps_DDRM.solve(R.copy(), BT, RinvBT);
    DMatrixRMaj G_k = new DMatrixRMaj(n, n);
    CommonOps_DDRM.mult(B, RinvBT, G_k);

    DMatrixRMaj A_k = A.copy();
    DMatrixRMaj H_k = new DMatrixRMaj(n, n);
    DMatrixRMaj H_k1 = Q.copy();

    DMatrixRMaj W = new DMatrixRMaj(n, n);
    DMatrixRMaj V1 = new DMatrixRMaj(n, n);
    DMatrixRMaj V2 = new DMatrixRMaj(n, n);
    DMatrixRMaj tmp1 = new DMatrixRMaj(n, n);
    DMatrixRMaj tmp2 = new DMatrixRMaj(n, n);
    DMatrixRMaj diffH = new DMatrixRMaj(n, n);

    do {
      // H_k ← H_k1  (save current value for convergence check)
      H_k.setTo(H_k1);

      // W = I + GₖHₖ
      CommonOps_DDRM.mult(G_k, H_k, W);
      CommonOps_DDRM.addEquals(W, CommonOps_DDRM.identity(n));

      // Solve WV₁ = Aₖ for V₁
      CommonOps_DDRM.solve(W.copy(), A_k, V1);

      // Solve WV₂ = Gₖ for V₂ (W and Gₖ are symmetric)
      CommonOps_DDRM.solve(W.copy(), G_k, V2);

      // Gₖ₊₁ = Gₖ + Aₖ V₂ Aₖᵀ
      CommonOps_DDRM.mult(A_k, V2, tmp1);         // tmp1 = Aₖ V₂
      CommonOps_DDRM.multTransB(tmp1, A_k, tmp2); // tmp2 = (Aₖ V₂) Aₖᵀ
      CommonOps_DDRM.addEquals(G_k, tmp2);

      // Hₖ₊₁ = Hₖ + V₁ᵀ Hₖ Aₖ
      CommonOps_DDRM.multTransA(V1, H_k, tmp1); // tmp1 = V₁ᵀ Hₖ
      CommonOps_DDRM.mult(tmp1, A_k, tmp2);      // tmp2 = V₁ᵀ Hₖ Aₖ
      // H_k1 ← H_k, then add increment
      H_k1.setTo(H_k);
      CommonOps_DDRM.addEquals(H_k1, tmp2);

      // Aₖ₊₁ = Aₖ V₁
      CommonOps_DDRM.mult(A_k, V1, tmp1);
      A_k.setTo(tmp1);

      // Convergence: ‖Hₖ₊₁ − Hₖ‖_F ≤ 1e-10 ‖Hₖ₊₁‖_F
      CommonOps_DDRM.subtract(H_k1, H_k, diffH);
    } while (NormOps_DDRM.normF(diffH) > 1e-10 * NormOps_DDRM.normF(H_k1));

    return H_k1;
  }
}
