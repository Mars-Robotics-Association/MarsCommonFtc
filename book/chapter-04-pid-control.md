# Chapter 4: PID Control

PID control is the most widely used feedback strategy in robotics. It is simple, intuitive, and effective for a broad range of mechanisms. This chapter covers how `PIDController` works in WpiMath, how to tune kP, kI, and kD, and — just as importantly — when PID is the right tool and when it is not.

## 4.1 The PID Formula

The PID controller computes its output from three terms:

$$u = k_P \, e + k_I \int e \, dt + k_D \frac{de}{dt}$$

Each term addresses a different aspect of the control problem:

**Proportional (kP * error)** — Reacts to the current error. Large error produces large output. This is the primary corrective force. Too little kP and the system is sluggish. Too much kP and the system oscillates.

**Integral (kI * integral(error))** — Accumulates error over time. Even a small persistent error will eventually build up enough integral to drive the output. This eliminates steady-state error caused by unmodeled disturbances (friction changes, gravity, battery sag). Too much kI causes overshoot and oscillation.

**Derivative (kD * derivative(error))** — Reacts to the rate of change of error. It acts as a damper, reducing output when the error is changing rapidly. This reduces overshoot and smooths the response. Too much kD amplifies sensor noise.

In WpiMath, the implementation computes these terms discretely at each timestep:

```java
public double calculate(double measurement) {
    m_prevError = m_error;
    m_error = m_setpoint - measurement;
    m_errorDerivative = (m_error - m_prevError) / m_period;

    // Integral with clamping
    m_totalError = MathUtil.clamp(
        m_totalError + m_error * m_period,
        m_minimumIntegral / m_ki,
        m_maximumIntegral / m_ki);

    return m_kp * m_error + m_ki * m_totalError + m_kd * m_errorDerivative;
}
```

The derivative is computed from consecutive error measurements divided by the loop period. This is a critical detail that has implications for how PID interacts with state estimation, as we will see later in this chapter.

## 4.2 Tuning kP

Start with kP alone. Set kI and kD to zero. Increase kP until the system reaches the setpoint with acceptable speed but without excessive oscillation.

The proportional term is the simplest form of feedback: double the error, double the output. For a position-controlled mechanism, kP has units of output per unit position. If kP = 15 V/rad and the position error is 0.1 rad, the proportional output is 1.5 V.

### The Proportional Band

A useful mental model is the **proportional band** — the error range that produces full output:

$$e_{\text{full}} = \frac{1}{k_P}$$

With $k_P$ = 0.010 V/TPS (the `FlywheelSimple` default), an error of 100 TPS produces 1.0 V of feedback. At full battery (12.5 V), the proportional band is 1250 TPS — errors larger than this are saturated.

For `ArmController` with $k_P$ = 15 V/rad, the proportional band is $1/15$ = 0.067 rad = 3.8 degrees. An error larger than 3.8 degrees commands more than 12 V of output.

### When kP Is Too Low

The system reaches the setpoint slowly and may never get there if there is a persistent disturbance. A flywheel with low kP will settle below the target velocity because the remaining error is too small to produce enough feedback voltage to overcome friction.

### When kP Is Too High

The system overshoots the setpoint, then undershoots, then overshoots again — oscillating around the target. Each oscillation may grow (unstable) or shrink (stable but poorly damped). The derivative term addresses this.

## 4.3 Tuning kD

The derivative term damps oscillation by reacting to the *rate of change* of error. When the error is decreasing rapidly (the system is approaching the setpoint quickly), the derivative term reduces output to prevent overshoot.

The derivative of error is:

$$\frac{de}{dt} = \frac{e_k - e_{k-1}}{\Delta t}$$

With $k_D$ = 1.0 V/(rad/s) and a velocity error changing at 10 rad/s per cycle, the derivative output is 10 V — a strong damping force.

### Derivative on Measurement vs. Derivative on Error

WpiMath's `PIDController` computes the derivative of **error**, not measurement. These are equivalent when the setpoint is constant, but differ when the setpoint changes:

- **Derivative on error**: A step change in setpoint produces an infinite derivative spike (error jumps, previous error does not). This causes a large output spike.
- **Derivative on measurement**: The derivative is computed from consecutive measurements, which are continuous. No spike occurs on setpoint changes.

WpiMath uses derivative on error. This is fine when combined with setpoint ramping (ProfiledPIDController), because the setpoint changes smoothly and the error derivative remains bounded.

### Noise Amplification

The derivative term amplifies high-frequency noise. If the sensor reading has 1 tick of noise and the loop runs at 50 Hz, the derivative sees 50 ticks/s of noise. With kD = 0.1, this produces 5 V of noise in the output.

The solution is to filter the measurement before it reaches the PID controller. Chapter 20 covers signal filtering in detail. In practice, a first-order low-pass filter with a cutoff frequency above the system bandwidth but below the noise frequency works well.

## 4.4 Tuning kI and Integral Windup

The integral term eliminates steady-state error by accumulating error over time. Even a tiny persistent error of 0.01 units will eventually build up enough integral to produce meaningful output.

### When You Need kI

You need kI when there is a **persistent disturbance** that the feedforward and proportional terms cannot handle:

- Unmodeled friction that changes with temperature
- Gravity variations that the feedforward model does not capture
- External forces (a game piece pressing against a mechanism)

In MarsCommonFtc, kI is rarely used because feedforward handles the steady-state load. `ArmController` uses gravity feedforward (`kG * cos(angle)`) instead of integral. `FlywheelSimple` uses velocity feedforward (`kV * velocity`) instead of integral. The small remaining error is handled by kP alone.

### Integral Windup

When the output is saturated (clamped to ±1.0), the integral term continues to accumulate. When the error finally reverses, the integral must "unwind" before the output can change direction. This causes large overshoot.

WpiMath provides two anti-windup mechanisms:

**IZone** — When the error exceeds a threshold, the integral resets to zero:

```java
if (Math.abs(m_error) > m_iZone) {
    m_totalError = 0;
}
```

This prevents integral from building up during large transients when the system is far from the setpoint. The integral only engages when the system is close enough that steady-state error matters.

**Integral clamping** — The integral term's contribution is bounded:

```java
m_totalError = MathUtil.clamp(
    m_totalError + m_error * m_period,
    m_minimumIntegral / m_ki,
    m_maximumIntegral / m_ki);
```

The maximum integral contribution is `m_ki * m_maximumIntegral`. Setting `m_maximumIntegral = 0.2` ensures the integral term can never produce more than 20% of full output, regardless of how long it has been accumulating.

## 4.5 Continuous Input

Absolute encoders report angles from 0 to 2π (or 0 to 360 degrees). When the setpoint is 359 degrees and the measurement is 1 degree, the raw error is 358 degrees — but the actual shortest path is -2 degrees.

`PIDController` handles this with continuous input:

```java
pid.enableContinuousInput(0, 2 * Math.PI);
```

When continuous input is enabled, the error is wrapped using `MathUtil.inputModulus()`:

```java
double errorBound = (maximumInput - minimumInput) / 2.0;
m_error = MathUtil.inputModulus(m_setpoint - m_measurement, -errorBound, errorBound);
```

`inputModulus(value, -errorBound, errorBound)` wraps `value` into the range `[-errorBound, errorBound)`. For a 0 to 2π range, errorBound = π, so the error is always in `[-π, π)`.

`ProfiledPIDController` goes further: it also re-centers the goal and setpoint positions around the measurement, ensuring the trapezoid profile takes the shortest path:

```java
double goalMinDistance = MathUtil.inputModulus(m_goal.position - measurement, -errorBound, errorBound);
double setpointMinDistance = MathUtil.inputModulus(m_setpoint.position - measurement, -errorBound, errorBound);
m_goal.position = goalMinDistance + measurement;
m_setpoint.position = setpointMinDistance + measurement;
```

## 4.6 Setpoint Ramping with ProfiledPIDController

A raw PID controller reacts instantly to setpoint changes. If the setpoint jumps from 0 to 1000 TPS, the error is 1000 and the proportional output is `kP * 1000` — likely saturated. The motor slams to full power, overshoots, and oscillates.

`ProfiledPIDController` solves this by ramping the setpoint through a `TrapezoidProfile`:

```java
ProfiledPIDController pid = new ProfiledPIDController(
    kP, kI, kD,
    new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

pid.setGoal(1000);  // goal is 1000 TPS
double output = pid.calculate(currentVelocity);  // setpoint ramps toward 1000
```

Each call to `calculate()` advances the profiled setpoint toward the goal according to the velocity and acceleration constraints. The PID controller then computes its output based on the error between the measurement and the **profiled setpoint**, not the raw goal:

```java
m_setpoint = m_profile.calculate(getPeriod(), m_setpoint, m_goal);
return m_controller.calculate(measurement, m_setpoint.position);
```

The result is smooth motion: the setpoint accelerates at `maxAcceleration` until it reaches `maxVelocity`, cruises, then decelerates to the goal. The PID controller only sees small errors because the setpoint moves gradually.

### Resetting the Profile

When the controller is first enabled, the setpoint starts at zero. If the current velocity is 500 TPS, the initial error is 500 and the output spikes. Call `reset()` to seed the setpoint:

```java
pid.reset(currentVelocity, currentAcceleration);
pid.setGoal(1000);
```

## 4.7 Tolerance and atSetpoint

`PIDController` tracks whether it has reached the setpoint within a tolerance:

```java
pid.setTolerance(0.05, Double.POSITIVE_INFINITY);  // default
```

The first parameter is the position tolerance. The second is the velocity (error derivative) tolerance. `atSetpoint()` returns true only when **both** conditions are met:

```java
public boolean atSetpoint() {
    return Math.abs(m_error) < m_errorTolerance
        && Math.abs(m_errorDerivative) < m_errorDerivativeTolerance;
}
```

The default position tolerance is 0.05 (units depend on your system). The default velocity tolerance is infinity, meaning it is ignored. For a position-controlled arm, you might set:

```java
pid.setTolerance(Math.toRadians(2), Math.toRadians(5));  // 2 degrees, 5 deg/s
```

`ProfiledPIDController` adds `atGoal()`, which requires both `atSetpoint()` and that the profiled setpoint has reached the goal:

```java
public boolean atGoal() {
    return m_controller.atSetpoint() && m_goal.equals(m_setpoint);
}
```

This is stricter than `atSetpoint()` — it ensures the mechanism has not just reached *a* point within tolerance, but the *goal* itself.

## 4.8 Why MarsCommonFtc Avoids PIDController for Mechanisms

Despite providing `PIDController` and `ProfiledPIDController`, MarsCommonFtc's mechanism controllers (`ArmController`, `VerticalArmController`, `FlywheelSimple`, `VelocityMotorPF`) do **not** use `PIDController` internally. They implement feedback manually. There are deliberate reasons for this.

### Reason 1: Derivative Conflict with State Estimation

`PIDController` computes the derivative from consecutive error measurements:

```java
m_errorDerivative = (m_error - m_prevError) / m_period;
```

But `ArmController` uses a Kalman filter to produce a velocity estimate that is far more accurate than any finite difference of position measurements. Using `PIDController` would discard the Kalman filter's velocity estimate and replace it with a noisier derivative.

From the arm controller design notes:

> "Decision: manual PD (`kP * posError + kD * velError`) rather than `PIDController`, because PIDController computes its own derivative from consecutive position measurements, which conflicts with using the Kalman-predicted velocity for the derivative term."

### Reason 2: Feedforward Handles Steady-State

The integral term exists to eliminate steady-state error. But when feedforward accurately models the system, there is no steady-state error to eliminate. The gravity term in `ArmFeedforward` handles the constant gravity load. The velocity term in `SimpleMotorFeedforward` handles the back-EMF load. The integral term is redundant.

From the design notes:

> "For a 2-state system (position, velocity), LQR produces exactly two gains — mathematically equivalent to PD. PID's integral term was not needed because the feedforward handles the steady-state gravity load and the Kalman filter eliminates the measurement lag that was causing oscillation."

### Reason 3: Adaptive Feedback Gain

`VelocityMotorPF` suppresses the proportional gain during acceleration:

```java
double kPEffective = kP * Math.max(0, 1.0 - Math.abs(acceleration) / accelMax);
```

`PIDController` has no mechanism for varying its gains dynamically. Implementing this with `PIDController` would require manually adjusting kP each cycle, which defeats the purpose of using the class.

### Reason 4: Feedback Clamping

`FlywheelSimple` clamps the feedback voltage to a fraction of the battery voltage:

```java
fb_voltage = MathUtil.clamp(fb_voltage, -fbMax * hubVoltage, fbMax * hubVoltage);
```

This ensures feedback never overwhelms feedforward. While `PIDController` has integral clamping, it has no built-in output clamping that is aware of battery voltage — the kind needed here to scale the feedback ceiling dynamically.

These are not criticisms of `PIDController` — it is a well-designed class that works perfectly for many applications. The point is that MarsCommonFtc's mechanism controllers have specific requirements that are better served by manual feedback implementation.

## 4.9 When PID Is the Right Choice

PID is the right choice when:

**The system is approximately first-order** — A flywheel, a conveyor, a single-speed mechanism. The dynamics are dominated by a single time constant, and a proportional term (plus feedforward) is sufficient.

**You need simplicity** — PID has three tunable parameters that map intuitively to behavior. Team members who are new to control theory can understand and tune PID. State-space controllers require understanding of linear algebra, observability, and controllability.

**The setpoint changes slowly** — If the mechanism moves infrequently and holds position, PID with setpoint ramping works well. The derivative term handles the deceleration phase, and the proportional term holds position.

**You do not have a good model** — Feedforward and state-space require a physical model of the system. If you cannot characterize kS, kV, kA, or if the system is too complex to model, PID is the fallback. Tune kP until it works, add kD to damp oscillation, add kI if there is steady-state error.

## 4.10 When PID Is Not Enough

PID is not enough when:

**The system is multi-state** — An arm has position and velocity. A drivetrain has x, y, heading, and wheel velocities. PID treats each state independently, ignoring the coupling between them. State-space control (LQR) handles multi-state systems optimally.

**There is significant latency** — The FTC control loop runs at non-deterministic timing, and sensor readings are delayed. PID reacts to the current error, which may be stale. A Kalman filter predicts the current state from delayed measurements, and the controller acts on the prediction.

**The system is nonlinear** — An arm's gravity torque varies with angle. A flywheel's back-EMF varies with velocity. PID uses constant gains that are tuned for one operating point. Gain scheduling or feedback linearization handles nonlinearity.

**You need optimal performance** — PID tuning is art. LQR tuning is math. You specify how much you care about position error versus control effort (the Q and R matrices), and LQR computes the optimal gains. Chapter 12 covers this in detail.

## 4.11 The BangBangController

WpiMath includes a `BangBangController` for completeness. It outputs either 0 or 1 (full power or no power) based on whether the measurement is below or above the setpoint:

```java
public double calculate(double measurement, double setpoint) {
    return measurement < setpoint ? 1.0 : 0.0;
}
```

This is not a joke — bang-bang control works surprisingly well for velocity control of high-inertia mechanisms. The motor runs at full power until it approaches the target, then cuts off. The inertia carries it the rest of the way. The key requirement is that the motor controllers are set to **coast** (not brake), so the mechanism coasts to a stop rather than slamming into the setpoint.

The asymmetric design (0 or 1, never negative) means it cannot slow down an over-speeding mechanism. It is only suitable for systems where overshoot is naturally limited by back-EMF.

## 4.12 Tuning Workflow

A practical tuning workflow for a flywheel mechanism:

**Step 1: Characterize feedforward.** Run the feedforward characterization routine (Chapter 3) to find kS, kV, and kA. Enter these into the controller.

**Step 2: Set kP to zero.** Verify that feedforward alone gets the mechanism close to the target. There will be some steady-state error — this is expected.

**Step 3: Increase kP.** Raise kP until the steady-state error is acceptable. The feedback term should be small compared to feedforward — if feedback is doing most of the work, the feedforward model is wrong.

**Step 4: Check overshoot.** If the mechanism overshoots the target, increase kD (or reduce kP). For a flywheel, overshoot is usually not a problem because back-EMF naturally limits velocity.

**Step 5: Verify at different battery voltages.** Run the mechanism at full battery and at sagged battery (run another mechanism simultaneously). The behavior should be consistent. If not, check the voltage compensation.

For an arm mechanism, the workflow is similar but with additional steps:

**Step 2b: Verify gravity compensation.** Hold the arm at various angles with zero velocity target. The arm should not sag or drift. If it sags at the horizontal position, kG is too low. If it drifts upward, kG is too high.

**Step 3b: Tune PD on position error.** Command a small move (10 degrees). Adjust kP until the move completes in acceptable time. Adjust kD to eliminate overshoot. The test values in `ArmControllerTest` (kP = 15, kD = 1.0) are a reasonable starting point for a typical arm.

## 4.13 Summary

PID control is a reactive strategy that computes output from the current error, its accumulation, and its rate of change. WpiMath provides `PIDController` for general use, `ProfiledPIDController` for setpoint ramping, and `BangBangController` for high-inertia velocity systems.

The key insights are:

- **Start with kP** — it is the primary corrective force
- **Add kD to damp oscillation** — but beware noise amplification
- **Use kI sparingly** — feedforward handles steady-state error in well-modeled systems
- **Enable continuous input** for absolute encoders
- **Use tolerance to detect completion** — both position and velocity
- **MarsCommonFtc implements feedback manually** in mechanism controllers to integrate with Kalman filters, adaptive gains, and battery compensation

PID gets the job done for many mechanisms. But for multi-state systems, nonlinear dynamics, and optimal performance, state-space control offers a more principled approach. Part III of this book covers state-space control in depth, starting with the linear algebra foundations in Chapter 11.
