# Chapter 12: State-Space Fundamentals

Chapters 3 and 4 controlled motors with feedforward and PID — scalar gains applied to scalar errors. This works well for simple mechanisms, but it has limits. PID can't optimally trade off between tracking accuracy and control effort. You can add low-pass filters to smooth noisy sensor readings before they reach the PID loop, but that introduces phase lag — the smoother the signal, the more delayed it is. And while you *can* pair a Kalman filter with a PID controller, state-space control integrates estimation and control into a single unified framework.

State-space control addresses all three. It represents the system as a set of first-order differential equations (the **plant model**), estimates the true state from noisy measurements (the **Kalman filter**), and computes the optimal control input (the **LQR controller**). This chapter covers the theoretical framework and the MarsCommonFtc classes that implement it.

## 12.1 What Is a State?

The **state** of a system is the minimum set of variables that, together with the future input trajectory, completely determines the system's future behavior. For a flywheel, the state is angular velocity — if you know the current velocity and the applied voltage, you can predict the velocity at any future time. For an elevator, the state is position *and* velocity — position alone isn't enough because you also need to know how fast it's moving.

State-space control organizes these variables into vectors:

| Symbol | Name | Meaning |
|--------|------|---------|
| $\mathbf{x}$ | State vector | Internal system state (position, velocity, ...) |
| $\mathbf{u}$ | Input vector | Control inputs (voltage, ...) |
| $\mathbf{y}$ | Output vector | Sensor measurements |

The system's dynamics are described by two equations:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$$
$$\mathbf{y} = C\mathbf{x} + D\mathbf{u}$$

The first is the **state equation**: how the state evolves over time. The second is the **output equation**: what the sensors report.

The four matrices encode the physics:

- **A** (system matrix): How each state affects the rate of change of every other state. For a flywheel, $A = [-k_V/k_A]$ — velocity decays exponentially due to back-EMF.
- **B** (input matrix): How each input affects the rate of change of each state. For a flywheel, $B = [1/k_A]$ — voltage produces acceleration.
- **C** (output matrix): Which states the sensors can observe. If you measure velocity directly, $C = [1]$.
- **D** (feedthrough matrix): Direct influence of inputs on outputs. Almost always zero for FTC mechanisms.

## 12.2 The LinearSystem Class

`LinearSystem<States, Inputs, Outputs>` holds the four matrices:

```java
public class LinearSystem<States extends Num, Inputs extends Num, Outputs extends Num> {
    private final Matrix<States, States> m_A;
    private final Matrix<States, Inputs>  m_B;
    private final Matrix<Outputs, States> m_C;
    private final Matrix<Outputs, Inputs> m_D;
}
```

The type parameters encode the system dimensions. A flywheel is `LinearSystem<N1, N1, N1>` (1 state, 1 input, 1 output). An elevator is `LinearSystem<N2, N1, N2>` (2 states, 1 input, 2 outputs).

The constructor validates that all matrix elements are finite — catching NaN and infinity early, since these usually indicate a modeling error (division by zero in a motor constant, for example).

`LinearSystem` provides two computation methods:

**`calculateX()`** — propagates the state forward by one timestep:

```java
public Matrix<States, N1> calculateX(
        Matrix<States, N1> x, Matrix<Inputs, N1> clampedU, double dtSeconds) {
    var discABpair = Discretization.discretizeAB(m_A, m_B, dtSeconds);
    return discABpair.getFirst().times(x).plus(discABpair.getSecond().times(clampedU));
}
```

This discretizes A and B for the given timestep, then computes $\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k$. Note that it re-discretizes every call — this handles FTC's variable loop timing correctly.

**`calculateY()`** — computes the expected sensor output:

```java
public Matrix<Outputs, N1> calculateY(Matrix<States, N1> x, Matrix<Inputs, N1> clampedU) {
    return m_C.times(x).plus(m_D.times(clampedU));
}
```

This is $\mathbf{y} = C\mathbf{x} + D\mathbf{u}$, used by the Kalman filter to compare predicted measurements against actual sensor readings.

## 12.3 LinearSystemId: Creating Plant Models

You rarely construct `LinearSystem` by hand. Instead, `LinearSystemId` provides factory methods that derive the A, B, C, D matrices from physical parameters or characterization constants.

### From Characterization Constants (kV, kA)

The most common approach for FTC. If you've characterized your mechanism and found $k_V$ (volts per unit velocity) and $k_A$ (volts per unit acceleration), you can create a plant directly:

**Velocity system** (1 state: velocity):

```java
LinearSystem<N1, N1, N1> plant = LinearSystemId.identifyVelocitySystem(kV, kA);
```

This creates the system:

$$A = [-k_V/k_A], \quad B = [1/k_A], \quad C = [1], \quad D = [0]$$

The physics: the voltage equation $V = k_V v + k_A \dot{v}$ rearranges to $\dot{v} = -\frac{k_V}{k_A}v + \frac{1}{k_A}V$. The A matrix coefficient $-k_V/k_A$ is the inverse of the motor's time constant — it determines how quickly velocity decays toward zero when no voltage is applied.

**Position system** (2 states: position, velocity):

```java
LinearSystem<N2, N1, N2> plant = LinearSystemId.identifyPositionSystem(kV, kA);
```

This adds a position state that integrates velocity:

$$A = \begin{bmatrix} 0 & 1 \\ 0 & -k_V/k_A \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\ 1/k_A \end{bmatrix}, \quad C = I_2, \quad D = \begin{bmatrix} 0 \\ 0 \end{bmatrix}$$

The top row says: position changes at a rate equal to velocity. The bottom row is the same velocity dynamics as the 1-state system.

### From Motor Constants (DCMotor)

When you know the motor's physical parameters rather than characterization constants:

```java
// Flywheel: 1 state (velocity)
LinearSystem<N1, N1, N1> flywheel = LinearSystemId.createFlywheelSystem(
    motor,              // DCMotor with Kt, Kv, R
    JKgMetersSquared,   // moment of inertia
    gearing);           // gear ratio (output/input)

// Elevator: 2 states (position, velocity)
LinearSystem<N2, N1, N2> elevator = LinearSystemId.createElevatorSystem(
    motor, massKg, radiusMeters, gearing);

// Arm: 2 states (angle, angular velocity)
LinearSystem<N2, N1, N2> arm = LinearSystemId.createSingleJointedArmSystem(
    motor, JKgSquaredMeters, gearing);
```

These methods derive A and B from first principles using the motor's torque constant ($K_t$), velocity constant ($K_v$), and winding resistance ($R$). The resulting matrices are equivalent to what `identifyVelocitySystem()` produces, but parameterized differently.

### Why identifyVelocitySystem Is Preferred for FTC

The `DCMotor`-based factories need the motor's electrical constants ($K_t$, $K_v$, $R$) and the mechanism's physical properties (inertia, gearing). In FTC, these are often unknown or unreliable — motor specs vary between manufacturers, gear ratios include belt/chain losses, and moments of inertia are hard to calculate for complex mechanisms.

The characterization-based factories (`identifyVelocitySystem`, `identifyPositionSystem`) bypass all of this. You run the motor at several voltages, measure the resulting velocities and accelerations, fit a line, and get $k_V$ and $k_A$ directly. The plant model matches *your specific mechanism*, not a theoretical one.

`FlywheelStateSpace` uses `identifyVelocitySystem(kV, kA)` for exactly this reason.

## 12.4 Continuous to Discrete: Why and How

The state-space equations $\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u}$ describe continuous-time dynamics — the state changes smoothly and instantaneously. But a digital controller runs in discrete time: it reads sensors, computes outputs, and applies them at discrete intervals (every 10–30 ms in FTC).

The conversion uses the matrix exponential, as covered in Chapter 11:

$$A_d = e^{A \cdot dt}, \quad \begin{bmatrix} A_d & B_d \\ 0 & I \end{bmatrix} = \exp\left(\begin{bmatrix} A & B \\ 0 & 0 \end{bmatrix} \cdot dt\right)$$

The discrete-time state equation becomes:

$$\mathbf{x}_{k+1} = A_d \mathbf{x}_k + B_d \mathbf{u}_k$$

### Variable Timesteps

FTC's loop timing is non-deterministic — the SDK doesn't guarantee a fixed loop period. A loop iteration might take 10 ms or 30 ms depending on I2C bus traffic, garbage collection, and OpMode complexity.

This matters because $A_d$ and $B_d$ depend on $dt$. Using the wrong $dt$ produces the wrong state prediction. The MarsCommonFtc approach: **re-discretize every loop iteration using the actual elapsed time.**

`LinearSystem.calculateX()` calls `Discretization.discretizeAB(m_A, m_B, dtSeconds)` with the real measured timestep. The Kalman filter's `predict()` method does the same:

```java
public void predict(Matrix<Inputs, N1> u, double dtSeconds) {
    final var discPair = Discretization.discretizeAQ(m_plant.getA(), m_contQ, dtSeconds);
    final var discA = discPair.getFirst();
    final var discQ = discPair.getSecond();

    m_xHat = m_plant.calculateX(m_xHat, u, dtSeconds);
    m_P = discA.times(m_P).times(discA.transpose()).plus(discQ);

    m_dtSeconds = dtSeconds;
}
```

This is more expensive than using precomputed discrete matrices (it involves a matrix exponential each iteration), but it's correct — and for 1×1 or 2×2 systems, the cost is negligible.

## 12.5 The Kalman Filter

The Kalman filter is an **optimal state estimator** for linear systems with Gaussian noise. It answers the question: given a noisy sensor measurement and an imperfect model, what is the best estimate of the true state?

### The Two-Step Dance

Each control loop iteration, the Kalman filter performs two steps:

**Predict** — use the model to project the state forward:

$$\hat{\mathbf{x}}_{k+1}^- = A_d \hat{\mathbf{x}}_k + B_d \mathbf{u}_k$$
$$P_{k+1}^- = A_d P_k A_d^\top + Q_d$$

The state estimate $\hat{\mathbf{x}}$ moves forward according to the plant model. The error covariance $P$ grows — the longer we run without a measurement, the less certain we are.

**Correct** — incorporate the sensor measurement:

$$K = P^- C^\top (C P^- C^\top + R_d)^{-1}$$
$$\hat{\mathbf{x}}_{k+1}^+ = \hat{\mathbf{x}}_{k+1}^- + K(\mathbf{y} - C\hat{\mathbf{x}}_{k+1}^- - D\mathbf{u})$$
$$P_{k+1}^+ = (I - KC)P^-(I - KC)^\top + KR_dK^\top$$

The Kalman gain $K$ determines how much to trust the measurement versus the prediction. If $P$ is large (model uncertain) and $R$ is small (sensor accurate), $K$ is large — trust the sensor. If $P$ is small and $R$ is large, $K$ is small — trust the model.

### The KalmanFilter Class

```java
KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
    Nat.N1(), Nat.N1(),           // states, outputs
    plant,                         // LinearSystem
    VecBuilder.fill(3.0),         // state standard deviations (rad/s)
    VecBuilder.fill(0.01),        // measurement standard deviations (rad/s)
    0.020);                        // nominal dt (seconds)
```

The constructor takes two noise vectors:

- **State standard deviations** — how much the model might be wrong per second. Higher values mean "don't trust the model much, rely more on measurements."
- **Measurement standard deviations** — how noisy the sensor is. Higher values mean "don't trust measurements much, rely more on the model."

These are continuous-time standard deviations. The constructor converts them to covariance matrices ($Q = \text{diag}(\sigma_i^2)$, $R = \text{diag}(\sigma_i^2)$), discretizes them, and solves the DARE to find the initial error covariance $P$.

### Tuning the Noise Model

The noise standard deviations are the primary tuning knobs:

| Parameter | Effect of increasing |
|-----------|---------------------|
| Model std-dev | Trust measurements more → faster response, more noise in estimate |
| Measurement std-dev | Trust model more → smoother estimate, slower response |

For a flywheel with a noisy encoder (20 TPS quantization, 10 TPS jitter), typical values might be:

```java
modelStdDevRadPerSec = 3.0;       // model is approximate
measurementStdDevRadPerSec = 0.01; // encoder is fairly accurate
```

This says: "the model might be off by 3 rad/s per second, but the encoder is accurate to 0.01 rad/s." The filter trusts the encoder heavily and uses the model mainly for interpolation between measurements.

### Joseph Form for Numerical Stability

The covariance update in the correct step uses the Joseph form:

$$P^+ = (I - KC)P^-(I - KC)^\top + KR_dK^\top$$

This is algebraically equivalent to the simpler $(I - KC)P^-$, but numerically superior — it guarantees $P$ remains symmetric positive semidefinite even with floating-point rounding. The simpler form can lose symmetry over time, eventually causing the filter to diverge.

### The Solve Trick for Computing K

Computing $K = PC^\top S^{-1}$ naively requires inverting $S$. The implementation avoids this by solving the transposed linear system:

```java
// K = (S.solve(CP))ᵀ
final Matrix<States, Outputs> K = S.solve(C.times(m_P)).transpose();
```

This is more numerically stable and often faster than explicit inversion, especially for multi-output systems.

## 12.6 The Linear Quadratic Regulator (LQR)

LQR answers the question: what control input $\mathbf{u}$ minimizes the total cost of both state error and control effort over an infinite time horizon?

The cost function is:

$$J = \sum_{k=0}^{\infty} (\mathbf{x}_k^\top Q \mathbf{x}_k + \mathbf{u}_k^\top R \mathbf{u}_k)$$

- The $Q$ matrix penalizes state error — large $Q$ means "I care a lot about tracking accuracy"
- The $R$ matrix penalizes control effort — large $R$ means "I want to use less voltage"

The optimal control law is:

$$\mathbf{u} = K(\mathbf{r} - \hat{\mathbf{x}})$$

where $K$ is the LQR gain matrix, $\mathbf{r}$ is the reference (desired state), and $\hat{\mathbf{x}}$ is the estimated state (from the Kalman filter). This looks like proportional control — and it is, but with an optimally computed gain.

### Computing K

The gain $K$ comes from solving the DARE:

$$S = \text{DARE}(A_d, B_d, Q, R)$$
$$K = (B_d^\top S B_d + R)^{-1} B_d^\top S A_d$$

The implementation:

```java
var discABPair = Discretization.discretizeAB(A, B, dtSeconds);
var discA = discABPair.getFirst();
var discB = discABPair.getSecond();

var S = DARE.dare(discA, discB, Q, R);

// K = (BᵀSB + R)⁻¹BᵀSA
m_K = discB.transpose().times(S).times(discB).plus(R)
      .solve(discB.transpose().times(S).times(discA));
```

### Bryson's Rule for Tuning

LQR tuning uses Bryson's rule: set the diagonal of $Q$ and $R$ to the inverse square of the maximum acceptable excursion.

```java
LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
    plant,
    VecBuilder.fill(8.0),    // qelms: max acceptable velocity error (rad/s)
    VecBuilder.fill(12.0),   // relms: max acceptable voltage (V)
    0.020);                   // dt
```

The `qelms` and `relms` vectors are **tolerances**, not raw cost weights. `StateSpaceUtil.makeCostMatrix()` converts them:

```java
public static <Elements extends Num> Matrix<Elements, Elements> makeCostMatrix(
        Matrix<Elements, N1> tolerances) {
    for (int i = 0; i < tolerances.getNumRows(); i++) {
        if (tolerances.get(i, 0) == Double.POSITIVE_INFINITY) {
            result.set(i, i, 0.0);
        } else {
            result.set(i, i, 1.0 / Math.pow(tolerances.get(i, 0), 2));
        }
    }
    return result;
}
```

A tolerance of 8.0 rad/s becomes a cost of $1/64$. A tolerance of 12.0 V becomes a cost of $1/144$. Tighter tolerances mean higher costs, which make the controller more aggressive about that variable.

Setting a tolerance to `Double.POSITIVE_INFINITY` sets the cost to zero — the controller doesn't care about that state or input at all.

### LQR vs. PID

LQR is optimal in a way PID cannot be:

| Property | PID | LQR |
|----------|-----|-----|
| Gains | Tuned manually in gain space | Tuned manually in cost-weight space (Q, R) |
| Multi-state | Separate loops that interact | Unified optimal gain |
| Trade-off | Implicit, hard to reason about | Explicit Q and R matrices |
| Integral action | kI term | Not built-in (augment state with integrator) |

For single-state systems (like a flywheel), LQR's gain $K$ is equivalent to a well-tuned proportional gain — but computed from the physics rather than trial and error. The real advantage appears in multi-state systems, where LQR automatically balances the coupling between states.

### Latency Compensation

FTC's control loop has latency — time between reading a sensor and applying the motor command. LQR gains can be adjusted to compensate:

```java
controller.latencyCompensate(plant, dtSeconds, inputDelaySeconds);
```

This modifies $K$ by projecting it through the system dynamics:

$$K_{\text{comp}} = K \cdot (A_d - B_d K)^{t_{\text{delay}} / dt}$$

The compensated gain accounts for where the system will be when the command actually takes effect, rather than where it was when the sensor was read.

## 12.7 The LinearSystemLoop

`LinearSystemLoop` ties the plant, controller, and observer together into a single control loop:

```java
LinearSystemLoop<N1, N1, N1> loop = new LinearSystemLoop<>(
    plant,       // LinearSystem
    controller,  // LinearQuadraticRegulator
    observer,    // KalmanFilter
    maxVoltage,  // input clamp
    dtSeconds);  // nominal timestep
```

It also creates a `LinearPlantInversionFeedforward` internally. This feedforward computes the voltage needed to track the reference trajectory:

$$\mathbf{u}_{ff} = B_d^+ (\mathbf{r}_{k+1} - A_d \mathbf{r}_k)$$

where $B_d^+$ is the pseudoinverse of $B_d$. The total control input is:

$$\mathbf{u} = K(\mathbf{r} - \hat{\mathbf{x}}) + \mathbf{u}_{ff}$$

The LQR corrects for state error; the feedforward provides the baseline voltage for the desired trajectory. This is the same feedforward-plus-feedback architecture from Part I, but with optimally computed gains.

### The Control Loop Cycle

Each iteration:

```java
// 1. Set the desired state
loop.setNextR(VecBuilder.fill(targetRadPerSec));

// 2. Correct: incorporate the latest sensor measurement
loop.correct(VecBuilder.fill(measuredRadPerSec));

// 3. Predict: compute control output and project state forward
loop.predict(dt);

// 4. Read the control output
double voltage = loop.getU(0);
```

Inside `predict()`:

```java
public void predict(double dtSeconds) {
    var u = clampInput(
        m_controller.calculate(getObserver().getXhat(), m_nextR)
            .plus(m_feedforward.calculate(m_nextR)));
    getObserver().predict(u, dtSeconds);
}
```

The order matters: **correct before predict**. Correcting first incorporates the latest measurement into the state estimate, then the prediction uses that updated estimate to compute the control output and project the state forward. If you predict first, you're computing the control output based on a stale estimate.

### Input Clamping

The loop clamps the total control input (LQR + feedforward) to the maximum voltage:

```java
u -> StateSpaceUtil.desaturateInputVector(u, maxVoltageVolts)
```

For single-input systems, this is a simple clamp. For multi-input systems (like a differential drivetrain), `desaturateInputVector` scales all inputs proportionally to preserve the direction while respecting the magnitude limit.

## 12.8 Summary

State-space control has three components:

1. **The plant model** (`LinearSystem`) — encodes the system's physics in A, B, C, D matrices. Created from characterization constants via `LinearSystemId.identifyVelocitySystem(kV, kA)` or from motor parameters via `createFlywheelSystem()`.

2. **The observer** (`KalmanFilter`) — estimates the true state from noisy measurements. Tuned by setting model and measurement standard deviations. Re-discretizes at each timestep to handle FTC's variable loop timing.

3. **The controller** (`LinearQuadraticRegulator`) — computes the optimal feedback gain by solving the DARE. Tuned using Bryson's rule: set tolerances for acceptable state error and control effort.

`LinearSystemLoop` combines all three with a plant-inversion feedforward and input clamping, providing a clean predict-correct cycle.

The theory is general, but the next chapter makes it concrete: `FlywheelStateSpace` applies this entire framework to control a flywheel velocity — 1 state, 1 input, 1 output, and a Kalman filter that smooths noisy encoder measurements better than any fixed-cutoff low-pass filter.
