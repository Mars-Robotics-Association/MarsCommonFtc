# Chapter 3: Feedforward Control Theory

Feedforward is the simplest form of motor control that uses a model of the physical system. Instead of reacting to error like PID does, feedforward *predicts* the voltage needed to achieve a desired motion and applies it directly. This chapter covers the kS/kV/kA model, the physics of back-EMF, and the four feedforward classes in WpiMath.

## 3.1 Feedback vs. Feedforward

Consider a flywheel that needs to spin up from rest to 3000 ticks per second. A pure feedback controller works like this:

1. Read current velocity: 0 TPS
2. Compute error: 3000 - 0 = 3000
3. Multiply by kP: 3000 * 0.001 = 3.0
4. Clamp to 1.0, send to motor
5. Repeat at 50 Hz

This works, but it is fundamentally reactive. The controller does not know how much voltage the motor actually needs — it just pushes harder when the error is large and eases off when the error shrinks. The result is either sluggish response (low kP) or overshoot and oscillation (high kP).

A feedforward controller works differently:

1. The target velocity is 3000 TPS
2. The model says: at 3000 TPS, back-EMF consumes `kV * 3000` volts, friction consumes `kS` volts
3. Total voltage needed: `kS + kV * 3000`
4. Convert to power: `voltage / battery_voltage`
5. Send to motor

No error, no gain tuning, no oscillation. The controller applies exactly the voltage the physics demands. If the model is accurate, the motor reaches the target velocity with zero steady-state error — no integral term required.

In practice, no model is perfect. Feedforward gets you 90% of the way there, and a small feedback term cleans up the remaining 10%. This is the architecture used throughout MarsCommonFtc: **feedforward for the bulk of the work, feedback for the residual**.

## 3.2 The Voltage Balance Equation

A DC motor obeys a simple voltage balance at every instant:

$$V_{\text{applied}} = V_{\text{friction}} + V_{\text{back-EMF}} + V_{\text{acceleration}}$$

Each term has a physical meaning:

**$V_{\text{friction}}$ ($k_S$)** — The voltage needed to overcome static friction and stiction in the motor bearings, gear train, and mechanism. This is a constant offset that must be exceeded before the motor moves at all. Typical values range from 0.3 V to 1.5 V depending on the mechanism.

**$V_{\text{back-EMF}}$ ($k_V \cdot v$)** — The voltage consumed by the back-EMF effect. As the motor spins, the rotating magnets generate a voltage that opposes the applied voltage. This term grows linearly with velocity. The constant $k_V$ (volts per unit velocity) is determined by the motor's magnetic strength and gear ratio.

**$V_{\text{acceleration}}$ ($k_A \cdot a$)** — The voltage needed to accelerate the rotating mass. This term is proportional to acceleration. The constant $k_A$ (volts per unit acceleration) is determined by the total inertia of the motor, gears, and mechanism.

Written as a single equation:

$$V = k_S \mathop{\text{sign}}(v) + k_V v + k_A a$$

The $\mathop{\text{sign}}(v)$ term ensures that friction always opposes motion, regardless of direction.

## 3.3 Back-EMF: The Velocity Ceiling

Back-EMF is the reason a motor cannot spin infinitely fast. As velocity increases, the back-EMF term $k_V v$ grows, consuming more and more of the available voltage. Eventually, all voltage is consumed by friction and back-EMF, leaving nothing for acceleration:

$$V_{\text{battery}} = k_S + k_V \cdot v_{\max}$$

Solving for the maximum velocity:

$$v_{\max} = \frac{V_{\text{battery}} - k_S}{k_V}$$

This is the **absolute ceiling** — no controller, no amount of gain tuning, no algorithm can make the motor exceed this velocity at the given battery voltage. With a 12.5 V battery, $k_S$ = 0.89 V, and $k_V$ = 12.5/2632.1 V/TPS:

```
v_max = (12.5 - 0.89) / (12.5/2632.1) = 2442 TPS
```

As the battery sags to 11.5 V, the ceiling drops:

```
v_max = (11.5 - 0.89) / (12.5/2632.1) = 2231 TPS
```

This is why battery voltage compensation matters. Without it, the same power command produces different velocities at different battery states. With it, the controller scales its output to maintain consistent behavior.

### The Time Constant

The ratio $k_A / k_V$ is the **time constant** of the motor system — the time it takes for velocity to reach approximately 63% of its steady-state value under full voltage. For a typical goBILDA 5000-series flywheel:

$$\tau = \frac{k_A}{k_V} = \frac{12.5/2087.9}{12.5/2632.1} = 1.26 \text{ seconds}$$

This means the flywheel reaches 63% of its target velocity in about 1.26 seconds under full voltage. The full analytical solution for velocity over time is:

$$v(t) = v_{ss} - (v_{ss} - v_0) \cdot e^{-(k_V/k_A) t}$$

Where $v_{ss}$ is the steady-state velocity and $v_0$ is the initial velocity. The acceleration decays exponentially:

$$a(t) = a_0 \cdot e^{-(k_V/k_A) t}$$

This exponential behavior is not a limitation of the controller — it is the physics of the motor itself. No amount of tuning can make a first-order system respond faster than its time constant allows.

## 3.4 SimpleMotorFeedforward

`SimpleMotorFeedforward` is the WPILib class for velocity-based systems like flywheels, conveyors, and linear slides. It implements the kS/kV/kA model with both continuous and discrete formulations.

```java
SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA, dt);
```

### Continuous Formulation (Legacy)

The deprecated `calculate(velocity, acceleration)` method applies the voltage balance directly:

```java
public double calculate(double velocity, double acceleration) {
    return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
}
```

This is simple and intuitive, but it assumes the acceleration can change instantaneously — which is physically impossible for a system with inertia.

### Discrete Formulation (Recommended)

The current `calculateWithVelocities(currentVelocity, nextVelocity)` method uses matrix exponential discretization. Given the current velocity and the desired velocity one timestep later, it computes the exact voltage that bridges the gap:

```java
public double calculateWithVelocities(double currentVelocity, double nextVelocity) {
    if (kA < 1e-4) {
        // Negligible inertia: fall back to continuous form
        double acceleration = (nextVelocity - currentVelocity) / dt;
        return kS * Math.signum(nextVelocity) + kV * nextVelocity + kA * acceleration;
    }

    // Matrix exponential discretization
    double A = -kV / kA;
    double B = 1.0 / kA;
    double Ad = Math.exp(A * dt);
    double Bd = (Ad - 1.0) / A * B;

    // Solve for voltage that achieves nextVelocity
    double u = (nextVelocity - Ad * currentVelocity) / Bd;
    return MathUtil.clamp(u, -12.0, 12.0);
}
```

The discrete formulation is more accurate because it respects the system's dynamics over the timestep. It answers the question: "what constant voltage, applied for `dt` seconds, will take the motor from `currentVelocity` to `nextVelocity`?"

Note that the output is clamped to ±12.0 V — a hard-coded nominal voltage. In practice, controllers that use this method divide by the live battery voltage when converting to a power command (see Section 3.8), so the hard-coded clamp serves as a safety bound rather than the compensation mechanism.

### Achievable Velocity and Acceleration

`SimpleMotorFeedforward` provides methods to compute the physical limits of the system:

```java
double maxVel = ff.maxAchievableVelocity(batteryVoltage, acceleration);
double minVel = ff.minAchievableVelocity(batteryVoltage, acceleration);
double maxAccel = ff.maxAchievableAcceleration(batteryVoltage, velocity);
double minAccel = ff.minAchievableAcceleration(batteryVoltage, velocity);
```

These are used by trajectory generators to ensure that motion profiles respect the motor's physical capabilities. Chapter 10 covers this in detail with the back-EMF-aware profile tuning methods.

### Default Gains

The default gains in MarsCommonFtc are based on a goBILDA 5000-series motor at 12.5 V:

```java
public double kV = 12.5 / 2632.1;   // ~0.00475 V/TPS
public double kA = 12.5 / 2087.9;   // ~0.00599 V/TPS^2
public double kS = 0.8931;           // V
```

These numbers come from empirical characterization: the motor was tested at various power levels, and the resulting steady-state velocities and step responses were fit to the kS/kV/kA model. Chapter 10 covers the characterization process.

## 3.5 ArmFeedforward

`ArmFeedforward` extends the model to account for gravity. An arm mechanism has an additional voltage term that depends on the arm's angle:

$$V = k_S \mathop{\text{sign}}(v) + k_G \cos(\theta) + k_V v + k_A a$$

The $k_G \cos(\theta)$ term represents the voltage needed to hold the arm against gravity. When the arm is horizontal ($\theta = 0$), gravity torque is maximum and $\cos(0) = 1$. When the arm is vertical ($\theta = 90^\circ$), gravity torque is zero and $\cos(90^\circ) = 0$.

```java
ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA, dt);
double voltage = ff.calculate(angle, velocity, acceleration);
```

### The Discrete Solver

When `kA` is significant (>= 0.1), `ArmFeedforward.calculateWithVelocities()` uses a sophisticated iterative solver. The arm dynamics are nonlinear due to the `cos(angle)` gravity term, so the matrix exponential approach used by `SimpleMotorFeedforward` does not apply directly.

Instead, the solver uses RK4 integration with Newton's method and backtracking line search to find the voltage $u$ such that integrating the nonlinear arm dynamics for one timestep yields the target velocity:

$$\frac{d\omega}{dt} = -\frac{k_V}{k_A} \omega + \frac{1}{k_A} \left( u - k_S \mathop{\text{sign}}(\omega) - k_G \cos(\theta) \right)$$

This is the same physics model used in `ArmMotorSim` for simulation. The feedforward solver essentially runs the simulation in reverse: given the desired outcome, what voltage produces it?

### Achievable Limits with Gravity

The `maxAchievableVelocity()` and `maxAchievableAcceleration()` methods account for gravity:

```java
double maxVel = ff.maxAchievableVelocity(batteryVoltage, angle);
```

At the horizontal position, gravity consumes $k_G$ volts, leaving less voltage for motion. At the vertical position, gravity is zero and the full voltage budget is available. This is why arm trajectory limits must be angle-dependent — the same velocity that is achievable at the top of the arc may be impossible at the bottom.

## 3.6 ElevatorFeedforward

`ElevatorFeedforward` is similar to `SimpleMotorFeedforward` but adds a constant gravity term $k_G$:

$$V = k_S \mathop{\text{sign}}(v) + k_G + k_V v + k_A a$$

Unlike an arm, an elevator's gravity load is constant regardless of position (assuming a vertical mechanism). The $k_G$ term is the voltage needed to hold the elevator stationary against gravity.

```java
ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA, dt);
double voltage = ff.calculateWithVelocities(currentVelocity, nextVelocity);
```

The discrete formulation uses the same matrix exponential discretization as `SimpleMotorFeedforward`, with `kG` added as a constant offset after solving for the dynamic voltage.

## 3.7 DifferentialDriveFeedforward

`DifferentialDriveFeedforward` handles the coupled dynamics of a differential drive (tank/skid-steer) chassis. It uses separate gains for linear and angular motion:

```java
DifferentialDriveFeedforward ff = new DifferentialDriveFeedforward(
    kVLinear, kALinear, kVAngular, kAAngular, trackwidth, dt);
```

Internally, it uses `LinearPlantInversionFeedforward` with a drivetrain linear system model. The plant inversion approach generalizes the kS/kV/kA concept to multi-input, multi-output systems:

$$u_{ff} = B^{+}(r_{k+1} - A r_k)$$

Where $B^{+}$ is the pseudoinverse of the $B$ matrix, $A$ is the discretized state transition matrix, and $r_k$ and $r_{k+1}$ are the current and next reference states.

## 3.8 Feedforward in Controllers

### FlywheelSimple

`FlywheelSimple` applies the continuous feedforward formula directly, without using the WPILib class:

```java
double ff_voltage = PARAMS.kS * Math.signum(profiledVelocity)
        + PARAMS.kV * profiledVelocity
        + PARAMS.kA * accel;
```

The acceleration comes from a motion profile that ramps velocity toward the target. The acceleration is capped by the back-EMF limit:

```java
double accelLimit = (hubVoltage - PARAMS.kS - PARAMS.kV * profiledVelocity) / PARAMS.kA;
```

This ensures the feedforward voltage never exceeds the available battery voltage. The total power command is:

```java
double fb_voltage = kP * (targetVelocity - actualVelocity);
double totalVoltage = ff_voltage + fb_voltage;
lastPower = MathUtil.clamp(totalVoltage / hubVoltage, -1.0, 1.0);
```

### VelocityMotorPF

`VelocityMotorPF` uses the same feedforward formula but with an important twist: the proportional gain is suppressed during acceleration:

```java
double ff = kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
ff /= voltage;  // battery compensation

double kPEffective = kP * Math.max(0, 1.0 - Math.abs(acceleration) / accelMax);
double fb = kPEffective * (targetVelocity - actualVelocity);

motor.setPower(ff + fb);
```

At peak acceleration, `kPEffective` is zero — the controller runs on pure feedforward. As acceleration drops to zero, `kPEffective` approaches `kP` and the feedback term engages fully. This prevents the feedback controller from fighting the feedforward during transients, which is a common source of overshoot.

### ArmController

`ArmController` uses `ArmFeedforward` with a critical detail: the gravity compensation term uses the **predicted actual position**, not the trajectory position:

```java
// Predict where the arm actually is (accounting for latency)
double predictedPosRad = position + velocity * latency;
double predictedVelRad = velocity;

// Feedforward: position from measurement prediction, velocity from trajectory
double nextVel = trajVel + trajAccel * dt;
double ffVoltage = feedforward.calculate(predictedPosRad, trajVel, nextVel, dt);

// PD feedback on trajectory error
double pdVoltage = kP * (trajPos - predictedPosRad)
                 + kD * (trajVel - predictedVelRad);

// Total output
double totalVoltage = ffVoltage + pdVoltage;
lastPower = MathUtil.clamp(totalVoltage / hubVoltage, -1.0, 1.0);
```

Notice the mixed sources in the feedforward call: the position (`predictedPosRad`) comes from measurement prediction for accurate gravity compensation, while the velocity (`trajVel`, `nextVel`) comes from the trajectory for smooth dynamics. Using the predicted position for gravity compensation is essential. If the arm is lagging behind the trajectory (due to latency or disturbance), the trajectory position overestimates the arm's actual angle. Computing gravity at the trajectory position would apply the wrong compensation voltage, causing the arm to sag or overshoot.

Additionally, the Kalman filter input is gravity-corrected:

```java
double gravityVoltage = kG * Math.cos(predictedPosRad);
double frictionVoltage = kS * Math.signum(predictedVelRad);
lastLinearVoltage = actualVoltage - gravityVoltage - frictionVoltage;
```

By subtracting gravity and friction from the applied voltage before feeding it to the linear Kalman filter, the observer sees the linearized plant dynamics and produces a more accurate state estimate.

### VerticalArmController

`VerticalArmController` uses a different architecture: feedback linearization + LQR instead of PD. The `ArmFeedforward` is used only for computing move limits, not for the control law itself. The control law has three layers:

```java
// Layer 1: Cancel gravity and friction
double uCancel = kG * Math.cos(predictedPosRad)
               + kS * Math.signum(predictedVelRad);

// Layer 2a: Linear feedforward on the linearized plant
double uFfLinear = kV * trajVel + kA * trajAccel;

// Layer 2b: LQR feedback
double uLqr = lqr.calculate(state, reference).get(0, 0);

// Total
double totalVoltage = uCancel + uFfLinear + uLqr;
```

This approach separates concerns cleanly: Layer 1 cancels the nonlinearities, Layer 2 controls the resulting linear system, and Layer 3 (the trajectory manager) generates smooth setpoints. Chapter 16 covers this architecture in detail.

## 3.9 Characterizing Your Mechanism

The feedforward model is only as good as its gains. Finding kS, kV, kA, and kG requires empirical characterization — running the mechanism and measuring its response.

### The kS Test (Stiction)

Ramp power slowly from zero until the mechanism moves. The power at which motion begins, multiplied by battery voltage, is $k_S$:

$$k_S = \text{power}_{\text{start}} \times V_{\text{battery}}$$

For a typical goBILDA 5000-series motor, $k_S$ ranges from 0.3 V (well-lubricated flywheel) to 1.5 V (heavy arm with high friction).

### The kV Test (Steady-State Velocity)

Apply a known power level, wait for the mechanism to reach steady state, and measure the velocity:

$$k_V = \frac{\text{power} \times V_{\text{battery}} - k_S}{v_{ss}}$$

Repeat at several power levels and fit a line. The slope is $k_V$.

### The kA Test (Step Response)

Apply a step power command and measure the velocity over time. The velocity follows an exponential approach:

$$v(t) = v_{ss} - (v_{ss} - v_0) \cdot e^{-t/\tau}$$

Taking the natural log:

$$\ln\left(1 - \frac{v(t)}{v_{ss}}\right) = -\frac{t}{\tau}$$

Fit a line to $\ln(1 - v/v_{ss})$ versus time. The slope is $-1/\tau$. Then:

$$k_A = \tau \cdot k_V$$

### The kG Test (Gravity)

For an arm or elevator, find the power needed to hold the mechanism stationary against gravity:

$$k_G = \text{power}_{\text{hold}} \times V_{\text{battery}}$$

For an arm, measure at the horizontal position (maximum gravity torque). For an elevator, any position works since gravity is constant.

### Automated Characterization

`FlywheelsFeedforwardTuning` in the MarsCommonFtc codebase automates this process with a four-phase routine:

1. **Stiction detection** — Ramp power at 0.03/s until velocity exceeds 50 TPS
2. **Steady-state sweep** — Step from stiction to full power in N steps, collecting average velocity and voltage at each
3. **Linear regression** — Fit `voltage = kS + kV * velocity` via least-squares
4. **Step response** — Apply step power, fit the exponential decay to extract tau, then compute `kA = tau * kV`

The results are displayed in copy-paste format for direct use in controller configuration.

## 3.10 Units Matter

The feedforward gains are unit-dependent. MarsCommonFtc uses two different velocity units:

**TPS (ticks per second)** — Used by flywheel controllers. The encoder ticks per revolution depend on the motor and gear ratio. For a goBILDA 5000-series motor with 1120 CPR at the output shaft:

$$k_V = \frac{12.5}{2632.1} \text{ V/TPS} \qquad k_A = \frac{12.5}{2087.9} \text{ V/TPS}^2$$

**rad/s (radians per second)** — Used by arm controllers. This is the output shaft angular velocity in SI units:

$$k_V = \frac{12.5 \times 28}{2632.1 \times 2\pi} \text{ V·s/rad} \qquad k_A = \frac{12.5 \times 28}{2087.9 \times 2\pi} \text{ V·s}^2\text{/rad}$$

The conversion factor is $\frac{\text{gear\_ratio}}{2\pi \cdot \text{CPR}}$. Mixing units is a common source of bugs. Always verify that your gains match the units your controller expects.

## 3.11 When Feedforward Is Not Enough

Feedforward works beautifully when the model is accurate and the system is well-behaved. But real mechanisms have disturbances that the model does not capture:

- **Friction changes** as bearings wear or temperature shifts
- **Battery voltage sags** under heavy load faster than the controller can read it
- **External forces** push on the mechanism (a game piece hitting a flywheel, an arm colliding with a field element)
- **Model errors** — the kS/kV/kA model is a first-order approximation that ignores higher-order effects like cogging torque and magnetic saturation

In these cases, feedforward alone produces a steady-state error. The feedback term catches what feedforward misses. The art of controller tuning is balancing the two: enough feedforward to do the heavy lifting, enough feedback to handle the residuals, but not so much feedback that the system becomes oscillatory or unstable.

Chapter 4 covers PID feedback in detail. Chapter 12 introduces state-space control, which unifies feedforward and feedback into a single framework.

## 3.12 Summary

Feedforward control predicts the voltage needed to achieve a desired motion using a physical model of the motor system. The kS/kV/kA model captures friction, back-EMF, and inertia in three parameters that can be measured empirically. WpiMath provides four feedforward classes — `SimpleMotorFeedforward`, `ArmFeedforward`, `ElevatorFeedforward`, and `DifferentialDriveFeedforward` — each tailored to a specific mechanism type.

The key insights are:

- **Back-EMF sets a velocity ceiling** that no controller can exceed
- **Feedforward gets you 90% there** — feedback handles the remaining 10%
- **Battery voltage compensation** is essential for consistent behavior
- **Gravity compensation must use the actual position**, not the trajectory position
- **Units matter** — gains are unit-dependent and must match the controller's expectations

With feedforward providing the foundation, the next chapter examines PID feedback — the reactive counterpart that cleans up what the model misses.
