# Chapter 11: Linear Algebra for Robotics

Parts I and II controlled motors with scalar math: a single voltage, a single velocity, a single acceleration. State-space control replaces these scalars with vectors and matrices. A flywheel's state is a 1×1 vector (velocity). An elevator's state is a 2×1 vector (position and velocity). A differential drivetrain's state is a 2×1 vector (left velocity and right velocity). The matrices that connect these vectors encode the physics of the system — how voltage produces acceleration, how velocity decays due to friction, how sensors observe the hidden state.

This chapter introduces the linear algebra foundation that the rest of Part III builds on: the `Matrix` and `Vector` classes, the `Nat`/`Num` type-level dimension system, and the `DARE` solver that computes optimal gains for both the Kalman filter and the LQR controller.

## 11.1 Why Type-Safe Dimensions?

A 2×2 matrix times a 3×1 vector is undefined — the inner dimensions don't match. In raw array math, this is a runtime crash. In MarsCommonFtc's linear algebra (ported from WPILib), it's a **compile-time error**.

The trick is encoding the matrix dimensions in the Java type system:

```java
Matrix<N2, N2> A;   // 2×2 system matrix
Matrix<N2, N1> B;   // 2×1 input matrix
Matrix<N2, N1> x;   // 2×1 state vector

Matrix<N2, N1> result = A.times(x);     // ✓ 2×2 · 2×1 → 2×1
Matrix<N2, N1> wrong  = A.times(B);     // ✓ 2×2 · 2×1 → 2×1
// A.times(someN3Matrix) would not compile — dimension mismatch
```

This means dimension errors are caught by `javac`, not by a `MatrixDimensionException` at 2 AM during a match. The cost is some generic type noise, but the safety is worth it.

## 11.2 Num and Nat: Numbers as Types

The dimension encoding uses two classes:

**`Num`** is the abstract base. Each subclass represents a specific number:

```java
public abstract class Num {
    public abstract int getNum();
}
```

The concrete number classes (`N0`, `N1`, `N2`, ..., `N20`) are auto-generated. Each is trivial:

```java
public final class N2 extends Num {
    public static final Nat<N2> instance = new Nat<>() {
        public int getNum() { return 2; }
    };
    public int getNum() { return 2; }
}
```

**`Nat<T>`** is the interface that provides concrete instances of these number types:

```java
public interface Nat<T extends Num> {
    int getNum();

    static Nat<N1> N1() { return N1.instance; }
    static Nat<N2> N2() { return N2.instance; }
    // ... through N20
}
```

When you write `Nat.N2()`, you get a runtime object that knows it represents the number 2. When you use `N2` as a type parameter, the compiler knows the dimension at compile time.

### Why Both?

`Num` lives in the type system — it's a generic bound. `Nat` lives at runtime — it's a value you can pass to constructors. You need both because Java's generics are erased at runtime. A `Matrix<N2, N2>` knows its dimensions at compile time, but to allocate the underlying array, the constructor needs a runtime integer:

```java
// Nat provides the runtime integer for allocation
Matrix<N2, N2> m = new Matrix<>(Nat.N2(), Nat.N2());

// N2 provides the compile-time type for shape checking
Matrix<N2, N1> x = new Matrix<>(Nat.N2(), Nat.N1());
```

The system supports dimensions 0 through 20. No FTC mechanism needs more than 20 states — most use 1 or 2.

## 11.3 The Matrix Class

`Matrix<R extends Num, C extends Num>` wraps EJML's `SimpleMatrix` with type-safe dimensions. The generic parameters `R` and `C` encode the number of rows and columns.

### Construction

```java
// Zero matrix
Matrix<N2, N2> zero = new Matrix<>(Nat.N2(), Nat.N2());

// From raw data (row-major)
Matrix<N2, N2> A = new Matrix<>(Nat.N2(), Nat.N2(), new double[]{
    0, 1,
    0, -kV/kA
});

// Identity matrix
Matrix<N2, N2> eye = Matrix.eye(Nat.N2());
```

For small matrices, `MatBuilder.fill()` is more readable:

```java
Matrix<N2, N2> A = MatBuilder.fill(Nat.N2(), Nat.N2(),
    0,     1,
    0, -kV/kA);
```

### Arithmetic

All arithmetic methods return new matrices — `Matrix` is effectively immutable for arithmetic:

```java
Matrix<N2, N2> sum     = A.plus(B);       // element-wise addition
Matrix<N2, N2> diff    = A.minus(B);      // element-wise subtraction
Matrix<N2, N1> product = A.times(x);      // matrix multiplication
Matrix<N2, N2> scaled  = A.times(2.0);    // scalar multiplication
Matrix<N2, N2> half    = A.div(2.0);      // scalar division
```

Matrix multiplication is shape-checked at compile time. `A.times(x)` compiles because `A` is `Matrix<N2, N2>` and `x` is `Matrix<N2, N1>` — the inner dimension `N2` matches. The result is `Matrix<N2, N1>`.

### Key Operations

**Transpose:**
```java
Matrix<N1, N2> AT = A.transpose();  // rows and columns swap in the type too
```

**Inverse:**
```java
Matrix<N2, N2> Ainv = A.inv();
```

**Solve** (`A.solve(B)` computes $A^{-1}B$ without explicitly forming the inverse, which is more numerically stable):
```java
Matrix<N2, N1> x = A.solve(b);  // solves Ax = b
```

**Matrix exponential** ($e^{A}$):
```java
Matrix<N2, N2> expA = A.exp();
```

The matrix exponential is central to state-space control — it converts a continuous-time system matrix into its discrete-time equivalent. Section 12.3 explains why.

**Element access:**
```java
double a01 = A.get(0, 1);   // row 0, column 1
A.set(0, 1, 3.14);          // mutates in place
```

**Block operations:**
```java
// Extract a submatrix
Matrix<N2, N2> block = bigMatrix.block(2, 2, 0, 0);  // 2×2 block at (0,0)

// Assign a block into a larger matrix
bigMatrix.assignBlock(0, 0, smallMatrix);
```

### Accessing the Underlying EJML Matrix

When you need an EJML operation that `Matrix` doesn't wrap:

```java
SimpleMatrix raw = A.getStorage();
DMatrixRMaj ddrm = A.getStorage().getDDRM();
```

This escape hatch is used internally by the DARE solver and the Kalman filter for performance-critical operations. Application code should rarely need it.

## 11.4 The Vector Class

`Vector<R>` extends `Matrix<R, N1>` — it's a column vector with convenience methods:

```java
Vector<N2> v = VecBuilder.fill(3.0, 4.0);

double element = v.get(0);      // 3.0 — single-index access
double length  = v.norm();      // 5.0 — Euclidean norm
Vector<N2> u   = v.unit();      // (0.6, 0.8) — unit vector
double d       = v.dot(other);  // dot product
```

`VecBuilder.fill()` is the standard way to create small vectors:

```java
// 1-element (flywheel velocity)
VecBuilder.fill(targetRadPerSec)

// 2-element (elevator position, velocity)
VecBuilder.fill(0.0, 1.0 / kA)
```

### Vector in State-Space Code

Throughout the state-space library, vectors represent:

- **State vectors** ($\mathbf{x}$): the system's internal state (position, velocity, ...)
- **Input vectors** ($\mathbf{u}$): control inputs (voltage, ...)
- **Output vectors** ($\mathbf{y}$): sensor measurements (encoder reading, ...)
- **Reference vectors** ($\mathbf{r}$): desired state (target velocity, ...)

For a flywheel, all four are `Vector<N1>` — a single scalar wrapped in a vector for type compatibility. For an elevator, the state and output are `Vector<N2>`, the input is `Vector<N1>`.

## 11.5 The DARE Solver

The Discrete-time Algebraic Riccati Equation (DARE) appears twice in every state-space controller:

1. The **Kalman filter** solves DARE to compute the initial error covariance $P$
2. The **LQR controller** solves DARE to compute the optimal feedback gain $K$

Both are the same equation with different inputs. The DARE finds the unique stabilizing solution $X$ to:

$$A^\top X A - X - A^\top X B (B^\top X B + R)^{-1} B^\top X A + Q = 0$$

### What the Variables Mean

The meaning depends on context:

| Variable | In LQR | In Kalman |
|----------|--------|-----------|
| $A$ | Discrete system matrix | Discrete system matrix transposed |
| $B$ | Discrete input matrix | Output matrix transposed |
| $Q$ | State cost matrix | Discretized process noise |
| $R$ | Input cost matrix | Discretized measurement noise |
| $X$ | Optimal cost-to-go | Initial error covariance |

The equation is the same — only the interpretation changes. This is a deep duality in optimal control: the Kalman filter is the "dual" of the LQR, and both reduce to solving the same DARE.

### The Structured Doubling Algorithm

MarsCommonFtc solves the DARE using the Structured Doubling Algorithm (SDA), which converges quadratically — each iteration doubles the number of correct digits. The implementation in `DARE.sdaCore()` typically converges in 20–30 iterations to machine precision.

The algorithm maintains three matrices $(A_k, G_k, H_k)$ and iterates:

```java
do {
    // W = I + GₖHₖ
    // V₁ = W⁻¹Aₖ
    // V₂ = W⁻¹Gₖ
    // Gₖ₊₁ = Gₖ + Aₖ V₂ Aₖᵀ
    // Hₖ₊₁ = Hₖ + V₁ᵀ Hₖ Aₖ
    // Aₖ₊₁ = Aₖ V₁
} while (‖Hₖ₊₁ − Hₖ‖_F > 1e-10 ‖Hₖ₊₁‖_F);
```

The convergence test uses the Frobenius norm ratio: the iteration stops when the relative change in $H$ drops below $10^{-10}$. The solution is $X = H_\infty$.

### Precondition Checks

The `dare()` method (with precondition checks) validates four properties before solving:

1. **Q is symmetric positive semidefinite** — verified by checking that all eigenvalues are ≥ 0
2. **R is symmetric positive definite** — verified by attempting a Cholesky decomposition
3. **(A, B) is stabilizable** — checked via the PBH (Popov-Belevitch-Hautus) rank test
4. **(A, C) is detectable** — where $Q = C^\top C$, checked by testing stabilizability of $(A^\top, C^\top)$

If any check fails, DARE throws an `IllegalArgumentException` with a diagnostic message. The `dareNoPrecond()` variant skips these checks for performance when the caller guarantees the preconditions hold.

### Stabilizability: The PBH Test

A pair $(A, B)$ is stabilizable if all uncontrollable eigenvalues of $A$ lie strictly inside the unit circle ($|\lambda| < 1$). Informally: even if some states can't be directly controlled, they must decay on their own.

The PBH test checks this by examining each eigenvalue $\lambda$ with $|\lambda| \ge 1$:

$$\text{rank}\begin{bmatrix} \lambda I - A & B \end{bmatrix} = n$$

If the rank equals $n$ (the number of states), that eigenvalue is controllable. If any eigenvalue with $|\lambda| \ge 1$ fails this test, the system is not stabilizable — there exists an unstable mode that the input cannot reach.

`StateSpaceUtil.isStabilizable()` implements this with SVD-based rank computation, handling complex eigenvalues by constructing a real block matrix.

### The Cross-Term Variant

DARE also supports a state-input cross-term cost $N$:

$$A^\top X A - X - (A^\top X B + N)(B^\top X B + R)^{-1}(B^\top X A + N^\top) + Q = 0$$

This is reduced to the standard form via the substitution $A_2 = A - BR^{-1}N^\top$ and $Q_2 = Q - NR^{-1}N^\top$, then solved with the same SDA core.

## 11.6 Building and Composing Linear Systems

The `Discretization` utility converts continuous-time matrices to discrete-time. This is essential because control theory models physics in continuous time ($\dot{x} = Ax + Bu$) but digital controllers operate in discrete time ($x_{k+1} = A_d x_k + B_d u_k$).

### discretizeA

The simplest conversion — discretize just the system matrix:

```java
Matrix<States, States> discA = Discretization.discretizeA(contA, dtSeconds);
```

This computes $A_d = e^{A \cdot dt}$ using the matrix exponential. For a 1×1 system, this is just the scalar exponential $e^{a \cdot dt}$.

### discretizeAB

Discretize both A and B simultaneously, which is more accurate than discretizing them separately:

```java
var pair = Discretization.discretizeAB(contA, contB, dtSeconds);
var discA = pair.getFirst();
var discB = pair.getSecond();
```

The method constructs the block matrix and takes its exponential:

$$\begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix} = \exp\left(\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} \cdot dt\right)$$

This ensures that $A_d$ and $B_d$ are consistent — they come from the same matrix exponential, so the discrete-time system exactly matches the continuous-time system at the sample instants.

### discretizeAQ

Discretize A and the process noise covariance Q:

```java
var pair = Discretization.discretizeAQ(contA, contQ, dtSeconds);
var discA = pair.getFirst();
var discQ = pair.getSecond();
```

This uses the Van Loan method: construct the block matrix $M = \begin{bmatrix} -A & Q \\ 0 & A^\top \end{bmatrix}$, compute $\Phi = e^{M \cdot dt}$, and extract:

$$A_d = \Phi_{22}^\top, \quad Q_d = A_d \cdot \Phi_{12}$$

This is used by the Kalman filter to discretize the process noise at each timestep.

### discretizeR

Measurement noise discretization is simpler:

$$R_d = R / dt$$

The continuous measurement noise covariance has units of [measurement² · s]. Dividing by $dt$ converts to discrete units [measurement²]. The $R/dt$ scaling comes from the continuous-to-discrete covariance conversion — measurement noise doesn't accumulate over time, since measurements are instantaneous samples.

## 11.7 Summary

The linear algebra layer provides three things:

- **Type-safe dimensions** via `Num`/`Nat` — the compiler catches mismatched matrix sizes before the code runs. `Matrix<N2, N2>` cannot be multiplied by `Matrix<N3, N1>`.

- **Clean arithmetic** via `Matrix` and `Vector` — wrapping EJML's `SimpleMatrix` with shape-checked operations. The `exp()` method (matrix exponential) and `solve()` method (linear system solve) are the workhorses of state-space discretization.

- **The DARE solver** — the mathematical core of both LQR and Kalman filter gain computation. The Structured Doubling Algorithm converges quadratically, and the precondition checks catch modeling errors early.

With this foundation in place, Chapter 12 introduces the state-space framework that combines these tools into plant models, observers, and controllers.
