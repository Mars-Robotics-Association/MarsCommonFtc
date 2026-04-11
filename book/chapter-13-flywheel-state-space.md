# Chapter 13: Flywheel State-Space Controller

Chapters 11 and 12 built the theory: plant models, Kalman filters, LQR controllers. This chapter applies all of it to a concrete mechanism — a flywheel shooter controlled by `FlywheelStateSpace`. The system has one state (angular velocity), one input (voltage), and one output (encoder velocity). It's the simplest possible state-space controller, which makes it the ideal place to see every piece working together.

## 13.1 Why State-Space for a Flywheel?

Chapter 3 showed a simple flywheel controller: feedforward for the bulk of the voltage, proportional feedback for the residual. This works well in simulation. On a real robot, it has two problems:

**Sensor noise.** The REV Hub measures velocity by counting encoder ticks in a 50 ms window, then differentiating. For a 28-tick encoder at 2000 TPS, that's about 100 ticks per 50 ms sample — but the quantization to whole ticks still produces 20 TPS steps. The raw velocity signal has staircase artifacts that cause the feedback term to jitter.

A low-pass filter helps, but it introduces phase lag. A 5 Hz cutoff smooths the noise but delays the velocity estimate by ~30 ms. During spin-up, the controller sees a velocity that's behind the true value, overshoots, and oscillates.

**The Kalman filter solves this.** It uses the plant model to predict where velocity *should* be, then nudges the prediction toward the measurement. The result is smoother than the raw encoder and faster than a low-pass filter — the model gives the filter a *prediction* of where the state is heading, so it can track rapid changes without waiting for multiple noisy samples to confirm.

**Optimal gain computation.** PID tuning is trial and error. LQR tuning is a trade-off specification: "I want velocity within 8 rad/s of target, and I'm willing to use up to 12 V to get there." The DARE solver turns these tolerances into a gain that's mathematically optimal for the given plant. If the plant characterization changes (different motor, different gearing), recomputing the gain takes one line of code, not a tuning session.

## 13.2 The FlywheelStateSpace Architecture

`FlywheelStateSpace` wraps a `LinearSystemLoop<N1, N1, N1>` with FTC-specific concerns: unit conversion (TPS to rad/s), battery voltage normalization, acceleration estimation for readiness detection, and telemetry.

```
                  ┌─────────────────────────────────────────┐
                  │           LinearSystemLoop              │
                  │                                         │
   target TPS ──► │  ┌───────────┐    ┌─────────────────┐  │
   (as rad/s)     │  │    LQR    │───►│   Feedforward    │  │ voltage
                  │  │ controller│    │ (plant inversion)│──┼──────► motor
                  │  └─────┬─────┘    └─────────────────┘  │
                  │        │  ▲                             │
                  │        │  │ x̂ (state estimate)         │
                  │        ▼  │                             │
                  │  ┌────────┴──┐                          │
   encoder TPS ──┼─►│  Kalman   │                          │
   (as rad/s)     │  │  Filter   │                          │
                  │  └───────────┘                          │
                  └─────────────────────────────────────────┘
```

### The Params Class

All tuning parameters live in `FlywheelStateSpace.Params`:

```java
public static class Params {
    // Plant — WPILib SI units (V·s/rad, V·s²/rad)
    public double kV = 12.5 * 28 / (2632.1 * 2 * Math.PI);  // V·s/rad
    public double kA = 12.5 * 28 / (2087.9 * 2 * Math.PI);  // V·s²/rad
    public int ticksPerRev = 28;

    // Kalman observer noise model
    public double modelStdDevRadPerSec = 3.0;
    public double measurementStdDevRadPerSec = 0.01;

    // LQR cost weights
    public double qVelocityRadPerSec = 8.0;
    public double rVoltage = 12.0;

    public double dtSeconds = 0.020;

    public double readyThresholdTps = 30.0;
    public double readyAccelToleranceTps2 = 50.0;
    public double accelLpfCutoffHz = 2.0;
}
```

The parameters fall into three groups:

**Plant characterization** — `kV` and `kA` are the feedforward constants from Chapter 3, converted to WPILib SI units (V·s/rad and V·s²/rad). The default values come from a characterization run at 12.5 V with a 28-tick encoder:

```
kV = voltage × ticksPerRev / (maxTps × 2π)
kA = voltage × ticksPerRev / (maxAccelTps × 2π)
```

These are not the same as the `SimpleMotorFeedforward` constants from Chapter 3. The Chapter 3 values are in V/(ticks/s) and V/(ticks/s²); the state-space values are in V·s/rad and V·s²/rad. The conversion factor is `ticksPerRev / (2π)`.

**Kalman tuning** — `modelStdDevRadPerSec` and `measurementStdDevRadPerSec` control the filter's trust balance. The defaults (3.0 and 0.01) heavily trust the encoder, using the model mainly for interpolation.

**LQR tuning** — `qVelocityRadPerSec` and `rVoltage` set the cost trade-off. The default (8.0 rad/s tolerance, 12.0 V tolerance) produces a moderately aggressive controller that doesn't saturate the voltage output.

## 13.3 Building the Loop

The `buildLoop()` method constructs the complete control loop:

```java
private static LinearSystemLoop<N1, N1, N1> buildLoop(Params p, double maxVoltage) {
    LinearSystem<N1, N1, N1> plant =
            LinearSystemId.identifyVelocitySystem(p.kV, p.kA);

    KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
            Nat.N1(), Nat.N1(),
            plant,
            VecBuilder.fill(p.modelStdDevRadPerSec),
            VecBuilder.fill(p.measurementStdDevRadPerSec),
            p.dtSeconds);

    LinearQuadraticRegulator<N1, N1, N1> controller = new LinearQuadraticRegulator<>(
            plant,
            VecBuilder.fill(p.qVelocityRadPerSec),
            VecBuilder.fill(p.rVoltage),
            p.dtSeconds);

    return new LinearSystemLoop<>(plant, controller, observer, maxVoltage, p.dtSeconds);
}
```

Let's trace what happens for the default parameters.

### Step 1: The Plant

`identifyVelocitySystem(kV, kA)` creates a 1×1 system:

$$A = [-k_V/k_A], \quad B = [1/k_A], \quad C = [1], \quad D = [0]$$

With the default values ($k_V \approx 0.0267$, $k_A \approx 0.0337$):

$$A \approx [-0.793], \quad B \approx [29.68]$$

The A coefficient tells us the time constant: $\tau = k_A/k_V \approx 1.26$ seconds. Without voltage, velocity decays to 37% of its current value in 1.26 seconds.

### Step 2: The Kalman Filter

The constructor:

1. Creates the covariance matrices: $Q = [9.0]$ (model), $R = [0.0001]$ (measurement)
2. Discretizes A and Q at the nominal timestep (20 ms)
3. Solves the DARE for the initial error covariance P

With the default noise model, the filter trusts measurements heavily. The Kalman gain will be close to 1.0, meaning the estimate closely tracks the encoder — but with the model providing continuity between samples.

### Step 3: The LQR Controller

The constructor:

1. Uses Bryson's rule: $Q = [1/8^2] = [0.0156]$, $R = [1/12^2] = [0.00694]$
2. Discretizes A and B at the nominal timestep
3. Solves the DARE for the optimal cost-to-go S
4. Computes $K = (B^\top S B + R)^{-1} B^\top S A$

The resulting K is a single scalar — the proportional gain for the velocity error. With the defaults, K ≈ 0.04 V/(rad/s), meaning a 1 rad/s velocity error produces about 0.04 V of corrective voltage.

### Step 4: The Loop

`LinearSystemLoop` wraps everything and adds a plant-inversion feedforward. For a 1×1 velocity system, the feedforward computes:

$$u_{ff} = B_d^{-1}(r_{k+1} - A_d r_k)$$

If the reference is constant ($r_{k+1} = r_k$), this simplifies to:

$$u_{ff} = B_d^{-1}(1 - A_d) r = \frac{k_V}{1} \cdot v_{\text{target}}$$

which is exactly the feedforward voltage from Chapter 3 (minus the $k_S$ static friction term — state-space doesn't model Coulomb friction explicitly).

## 13.4 The Update Cycle

Each loop iteration calls `update(dt, hubVoltage)`:

```java
public void update(double dt, double hubVoltage) {
    if (dt < 1e-6) return;

    double twoPI = 2.0 * Math.PI;
    double measuredRadPerSec = motor.getVelocity() * twoPI / PARAMS.ticksPerRev;
    double targetRadPerSec   = targetTps           * twoPI / PARAMS.ticksPerRev;

    loop.setNextR(VecBuilder.fill(targetRadPerSec));
    loop.correct(VecBuilder.fill(measuredRadPerSec));
    loop.predict(dt);

    lastVoltageCmded = loop.getU(0);
    double voltage = MathUtil.clamp(lastVoltageCmded, -hubVoltage, hubVoltage);
    lastPower = MathUtil.clamp(voltage / hubVoltage, -1.0, 1.0);

    motor.setPower(targetTps == 0 ? 0 : lastPower);

    // Acceleration estimation for isReady()
    double currentEstimatedTps = getEstimatedTps();
    double rawAccel = (currentEstimatedTps - prevEstimatedTps) / dt;
    accelLpf.update(rawAccel, dt);
    prevEstimatedTps = currentEstimatedTps;
}
```

### Unit Conversion

All state-space math is done in SI units (rad/s). The encoder reports TPS. The conversion happens at the boundary:

```java
// TPS → rad/s (input)
double measuredRadPerSec = motor.getVelocity() * 2π / ticksPerRev;

// rad/s → TPS (output)
double estimatedTps = loop.getXHat(0) * ticksPerRev / 2π;
```

This keeps the plant model, Kalman filter, and LQR controller in consistent units. The $k_V$ and $k_A$ parameters are also in SI units, so all the math is dimensionally correct.

### Correct-Then-Predict Order

The loop calls `correct()` before `predict()`. This is important:

1. **correct** — the Kalman filter incorporates the encoder measurement, updating the state estimate $\hat{x}$ to account for the latest sensor data
2. **predict** — the LQR computes $u = K(r - \hat{x}) + u_{ff}$, then the Kalman filter projects the state forward by $dt$

If you reverse the order, the LQR computes its output based on a state estimate that hasn't yet seen the latest measurement — it's always one step behind.

### Voltage Normalization

The loop outputs a voltage command (e.g., 8.3 V). To send this to the motor, divide by the hub voltage to get a power in [-1, 1]:

```java
lastPower = MathUtil.clamp(voltage / hubVoltage, -1.0, 1.0);
```

This automatically compensates for battery sag. If the battery drops from 13 V to 11 V, the same voltage command produces a larger power value, maintaining consistent motor behavior.

### Zero-Setpoint Coasting

When the target is zero, the motor is set to zero power regardless of the loop output:

```java
motor.setPower(targetTps == 0 ? 0 : lastPower);
```

Without this, the LQR would actively brake the flywheel — computing negative voltage to drive velocity toward zero. For a flywheel, passive coasting (zero power) is preferred because it avoids wasting battery and mechanical stress.

## 13.5 Readiness Detection

`isReady()` determines whether the flywheel has reached its target velocity and settled:

```java
public boolean isReady() {
    return targetTps != 0
            && Math.abs(getEstimatedTps() - targetTps) < PARAMS.readyThresholdTps
            && Math.abs(accelLpf.getValue()) < PARAMS.readyAccelToleranceTps2;
}
```

Three conditions must all be true:

1. **Non-zero setpoint** — readiness is meaningless if the target is zero
2. **Velocity within threshold** — the Kalman-estimated velocity must be within 30 TPS of the target
3. **Acceleration settled** — the low-pass filtered acceleration must be below 50 TPS/s

The acceleration check prevents false positives during spin-up. As the flywheel accelerates toward 2000 TPS, it will briefly pass through the 1970–2030 TPS window. Without the acceleration check, `isReady()` would return true during this transient — and the game code might fire a ball before the flywheel has actually settled.

### Acceleration Estimation

The acceleration is estimated by differentiating consecutive Kalman estimates:

```java
double rawAccel = (currentEstimatedTps - prevEstimatedTps) / dt;
accelLpf.update(rawAccel, dt);
```

Using the Kalman estimate rather than the raw encoder avoids amplifying measurement noise through differentiation. The biquad low-pass filter (2 Hz cutoff) further smooths the acceleration signal.

## 13.6 Initialization and Reset

The constructor seeds the Kalman observer to the current encoder velocity:

```java
double initialRadPerSec = motor.getVelocity() * 2 * Math.PI / PARAMS.ticksPerRev;
loop.reset(VecBuilder.fill(initialRadPerSec));
```

Without this, the observer starts at zero. If the flywheel is already spinning (e.g., after a brief pause in autonomous), the first correction sees a huge residual ($y - C\hat{x}$), causing a large state jump and a voltage spike.

The `reset()` method reinitializes the observer for the same reason:

```java
public void reset() {
    double currentRadPerSec = motor.getVelocity() * 2.0 * Math.PI / PARAMS.ticksPerRev;
    loop.reset(VecBuilder.fill(currentRadPerSec));
}
```

Call this when transitioning from idle to active control, or after the flywheel has been externally disturbed.

## 13.7 Handling FTC's Non-Deterministic Loop Timing

FTC OpModes don't run at a fixed rate. A loop iteration might take 15 ms or 35 ms depending on I2C bus congestion, vision pipeline load, and Java garbage collection. The state-space controller handles this through two mechanisms:

**Variable-dt prediction.** The `predict(dt)` call passes the actual elapsed time, not a nominal value. Inside, `Discretization.discretizeAQ()` recomputes $A_d$ and $Q_d$ for the true timestep. This means the plant model accurately tracks the system regardless of loop timing.

**Nominal-dt gain computation.** The Kalman gain and LQR gain are computed once at construction using the nominal 20 ms timestep. These gains are slightly suboptimal at non-nominal timesteps, but the error is small for the range of FTC loop periods (10–30 ms). Recomputing the gains every iteration would require solving the DARE each loop — feasible for 1×1 systems, but unnecessarily expensive.

The practical result: the controller works correctly across the range of FTC loop timings without any user intervention.

## 13.8 Simulation Tests

`FlywheelStateSpaceTest` validates the controller against `FlywheelMotorSim` — a first-order linear velocity model with configurable noise and quantization:

```java
private static double step(FlywheelStateSpace flywheel,
                           FlywheelTestFixture.SimMotorAdapter adapter,
                           FlywheelMotorSim sim, Random rng) {
    double dt = Math.max(0.001, 0.020 + rng.nextGaussian() * 0.004);
    flywheel.update(dt, FlywheelTestFixture.HUB_VOLTAGE);
    sim.step(dt, adapter.lastPower, FlywheelTestFixture.HUB_VOLTAGE);
    return dt;
}
```

Each test step uses a normally-distributed timestep (mean 20 ms, σ 4 ms) to exercise the variable-dt handling. The simulator adds 10 TPS Gaussian noise and 20 TPS quantization to the velocity measurement, matching real hardware conditions.

### Spin-Up Convergence

```java
flywheel.setTps(2000);
// ... run 800 steps ...
assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0);
assertTrue(flywheel.isReady());
```

The test verifies that the true velocity converges to within 30 TPS of the setpoint and that `isReady()` returns true. The 800-step limit (about 16 seconds) is generous — the flywheel typically converges in 2–3 seconds.

### Disturbance Recovery

```java
sim.setDisturbanceVoltage(-0.5);    // apply 0.5 V drag
// ... run 50 steps ...
sim.setDisturbanceVoltage(0.0);     // remove disturbance
// ... run 300 steps ...
assertEquals(2000.0, sim.getTrueVelocityTps(), 30.0);
```

A -0.5 V disturbance simulates a ball entering the launcher — sudden drag that the model doesn't predict. The Kalman filter detects the discrepancy between the predicted and measured velocity, adjusts the state estimate, and the LQR increases voltage to compensate. After the disturbance is removed, the controller returns to steady state.

### Setpoint Step-Down

```java
flywheel.setTps(1000);   // step from 2000 to 1000
// ... run 300 steps ...
assertEquals(1000.0, sim.getTrueVelocityTps(), 30.0);
```

This tests the controller's response to a large setpoint change. The LQR reduces voltage (possibly to zero or negative) to let the flywheel coast down, and the feedforward adjusts to the new steady-state voltage.

### Kalman Estimate Accuracy

```java
double estimatedTps = flywheel.getEstimatedTps();
double trueTps = sim.getTrueVelocityTps();
assertEquals(trueTps, estimatedTps, 25.0);
```

At steady state, the Kalman estimate must track the true velocity within 25 TPS — tighter than the 30 TPS setpoint tolerance. This verifies that the observer is producing useful estimates despite the 20 TPS quantization floor.

## 13.9 Comparing State-Space to PID + Feedforward

Both approaches control the same flywheel. How do they compare?

| Aspect | PID + Feedforward | FlywheelStateSpace |
|--------|-------------------|--------------------|
| Steady-state accuracy | Good (feedforward handles most of the work) | Good (plant-inversion feedforward + LQR) |
| Noise rejection | Depends on filter choice; trade-off between smoothness and lag | Kalman filter: smooth *and* fast |
| Spin-up response | Depends on kP tuning | Optimal for given Q/R costs |
| Disturbance rejection | kP and kI compensate | LQR + Kalman detects and corrects |
| Tuning | kP, kI, kD, filter cutoff | kV, kA, 2 noise std-devs, 2 cost tolerances |
| Battery compensation | Manual voltage scaling | Built into the voltage normalization |

The biggest practical difference is the **Kalman filter**. `FlywheelStateSpace.getEstimatedTps()` provides a cleaner velocity signal than any fixed-cutoff low-pass filter, because the filter uses the plant model for prediction instead of just averaging past samples. This makes `isReady()` more reliable — it can distinguish between the flywheel genuinely settling at target and the flywheel briefly passing through the target during a transient.

## 13.10 Tuning Guide

### Step 1: Characterize the Motor

Run the mechanism at several constant voltages and measure the steady-state velocity. Fit $k_V$ and $k_A$ as described in Chapter 3. Convert to SI units:

```java
PARAMS.kV = voltage * ticksPerRev / (maxTps * 2 * Math.PI);
PARAMS.kA = voltage * ticksPerRev / (maxAccelTps * 2 * Math.PI);
```

### Step 2: Set the Noise Model

Start with the defaults:

```java
PARAMS.modelStdDevRadPerSec = 3.0;
PARAMS.measurementStdDevRadPerSec = 0.01;
```

If the estimate is too noisy (jitters at steady state), increase `measurementStdDevRadPerSec` to trust the model more. If the estimate lags during rapid changes, increase `modelStdDevRadPerSec` to trust the encoder more.

### Step 3: Set the LQR Costs

Start with:

```java
PARAMS.qVelocityRadPerSec = 8.0;   // "I want velocity within 8 rad/s"
PARAMS.rVoltage = 12.0;             // "I can use up to 12 V"
```

To make the controller more aggressive (faster convergence, more voltage), decrease `qVelocityRadPerSec`. To make it gentler (less voltage, slower convergence), increase it. `rVoltage` controls how much the controller is willing to "spend" in voltage — lower values allow more voltage use.

### Step 4: Set the Readiness Thresholds

```java
PARAMS.readyThresholdTps = 30.0;       // velocity window
PARAMS.readyAccelToleranceTps2 = 50.0; // acceleration settling
```

Tighten `readyThresholdTps` if balls are launching at inconsistent speeds. Tighten `readyAccelToleranceTps2` if `isReady()` triggers too early during spin-up transients.

### Step 5: Validate with Telemetry

`writeTelemetry()` reports the target, estimated velocity, measured velocity, voltage command, and motor power:

```java
flywheel.writeTelemetry();
```

Watch for:
- **Estimated vs. measured velocity gap** — should be small at steady state. A persistent gap means the plant model is inaccurate.
- **Voltage saturation** — if `voltage cmd` is always at the battery voltage, the controller is asking for more than the motor can deliver. Reduce the setpoint or lower the LQR aggressiveness.
- **isReady() timing** — should become true shortly after the flywheel reaches target velocity, not during the approach.

## 13.11 Summary

`FlywheelStateSpace` applies the full state-space framework from Chapters 11–12 to a single-state velocity system:

- **Plant model** — `LinearSystemId.identifyVelocitySystem(kV, kA)` creates a 1×1 linear system from characterization constants, avoiding the need for motor spec sheets.

- **Kalman filter** — estimates velocity from noisy encoder measurements with less lag than a low-pass filter. The model-based prediction provides continuity that fixed-cutoff filters cannot match.

- **LQR controller** — computes the optimal proportional gain from cost tolerances. Combined with the plant-inversion feedforward, it produces the same feedforward-plus-feedback architecture as Part I, but with gains derived from the physics rather than trial and error.

- **FTC integration** — variable-dt support, TPS↔rad/s conversion, battery voltage normalization, acceleration-based readiness detection, and zero-setpoint coasting.

The next chapter extends this foundation to nonlinear state estimation with the Extended and Unscented Kalman filters, and introduces `FlywheelBallTracker` — a Kalman filter that augments the flywheel state with a ball-drag disturbance term to detect ball launches in real time.
