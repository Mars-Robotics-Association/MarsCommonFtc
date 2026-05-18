# Vertical Arm Controller Design Report

## Motivation

`ArmController` works, but its structure evolved incrementally. The gravity-corrected Kalman input — the most important implementation discovery — was a late fix bolted onto a feedforward + PD design. The gravity subtraction happens at the end of `update()` as a fixup, not as a first-class concept. `VerticalArmController` is a greenfield redesign that makes **feedback linearization** the organizing principle, and replaces hand-tuned PD with **LQR** for automatic gain computation.

The hardware constraints are unchanged: same REV Hub encoder (10 ms sampling, 50 ms velocity window, 20 TPS quantization), same 25-40 ms sensor-to-actuator latency, same variable loop timing (16 ms +/- 5 ms).

## Architecture: Three Layers

```
Layer 3:  Trajectory         SCurvePosition via PositionTrajectoryManager
Layer 2:  Linear Control     LQR feedback  +  linear feedforward (kv*v + ka*a)
Layer 1:  Linearization      Cancel gravity + friction at predicted angle
```

The key idea: cancel the nonlinear terms (gravity and friction) first, reducing the arm to a pure linear DC motor. Everything above — the Kalman filter, the LQR, the feedforward — operates on that clean linear system. The voltage decomposition is explicit in every control cycle:

```
u_total = u_cancel + u_ff_linear + u_lqr
```

Each term is independently loggable and debuggable.

### Comparison with ArmController

In `ArmController`, the feedforward computes gravity + friction + velocity + acceleration as a single voltage via `ArmFeedforward.calculate()`. The gravity subtraction for the Kalman filter then backs out gravity and friction at the end. The same physical concept — linearization — is expressed, but split across feedforward computation and Kalman input correction, making it harder to reason about.

In `VerticalArmController`, gravity + friction cancellation is computed once (`u_cancel`, step 6 of the control loop) and used in two places: added to the output voltage (step 9) and subtracted for the Kalman input (step 10). The symmetry is obvious in the code.

## Layer 1: Feedback Linearization

Each cycle, compute the voltage needed to cancel gravity and static friction at the **predicted actual arm angle**:

```
u_cancel = kg * cos(predicted_pos) + ks * signum(predicted_vel)
```

Using the predicted angle (not the trajectory angle) is critical. Gravity acts at the arm's physical position. If gravity compensation were indexed off the trajectory position, a gravity-assisted overshoot would under-compensate (trajectory is behind the arm, so `cos(traj_pos)` is wrong), creating positive feedback.

After cancellation, the remaining dynamics are a linear motor described by `kv` and `ka`.

## Layer 2: LQR + Linear Feedforward

### Linear feedforward

The trajectory provides position, velocity, and acceleration setpoints. The linear feedforward computes the voltage the linearized motor needs to follow the velocity and acceleration profile:

```
u_ff_linear = kv * traj_vel + ka * traj_accel
```

This is the same as the `kv` and `ka` terms from `ArmFeedforward`, but computed directly rather than extracted from the full 4-term formula. The `ks` and `kg` terms are already handled by Layer 1.

### LQR feedback

The Linear Quadratic Regulator closes the loop on tracking error:

```
u_lqr = K * (reference - state)
      = K * ([traj_pos - pred_pos, traj_vel - pred_vel]')
```

where `K` is a 1x2 gain matrix computed at construction from the plant model and three cost weights:

| Parameter | Meaning | Default |
|-----------|---------|---------|
| `qPosition` | Position tolerance (rad) — smaller = tighter tracking | 0.5 |
| `qVelocity` | Velocity tolerance (rad/s) — smaller = more aggressive damping | 5.0 |
| `rVoltage` | Control effort penalty (V) — larger = gentler output | 12.0 |

WPILib constructs Q = diag(1/q^2) and R = diag(1/r^2) internally, then solves the discrete algebraic Riccati equation (DARE) to find the optimal K.

### Why LQR over PD

For a 2-state position system, LQR is structurally identical to PD — both produce two gains multiplied by position and velocity error. The advantages are practical:

1. **Auto-tuning**: When `kv` or `ka` change (different motor, gearing, or arm mass), the gains adjust automatically. PD gains would need manual re-tuning.

2. **Intuitive cost weights**: `qPosition = 0.5` means "I tolerate 0.5 radians of position error" — more interpretable than `kP = 15` (volts per radian).

3. **Codebase consistency**: `FlywheelStateSpace` already uses LQR + Kalman. Using the same pattern for the arm establishes a project-wide convention.

### Why not LinearSystemLoop

`FlywheelStateSpace` bundles LQR + Kalman + plant into WPILib's `LinearSystemLoop`. `VerticalArmController` cannot use `LinearSystemLoop` because we need to manually subtract gravity/friction from the Kalman filter's input voltage each cycle. Separate `KalmanFilter` and `LinearQuadraticRegulator` objects give us that control.

## State Estimation: Kalman Filter

A linear Kalman filter observes the gravity-free plant (2 states: position, velocity; 1 input: linearized voltage; 2 outputs: measured position, measured velocity).

### Linearized voltage input

The Kalman filter must see the **linearized** voltage — the total applied voltage minus the gravity/friction cancellation:

```
u_linear = (clamped_power * hub_voltage) - u_cancel
```

This is the same insight as `ArmController`'s "gravity-corrected Kalman input", but expressed directly as a consequence of the feedback linearization architecture. If the linearized motor receives `u_linear` volts, the Kalman's linear plant model predicts the correct state evolution.

Using the **clamped** power (after saturation to [-1, 1]) is important. When the motor saturates, the unclamped voltage was never applied. Feeding an unrealistically large voltage to the observer would cause divergent state predictions.

### Latency compensation

After the Kalman update, the state is forward-predicted by 30 ms to compensate for sensor-to-actuator latency:

```
predicted_state = plant.calculateX(kalman_estimate, u_linear, 0.030)
```

All downstream decisions — coast check, replan check, gravity cancellation, LQR feedback — use this predicted state.

## LQR Gain Tuning Constraint

During development, aggressive LQR gains (K0 > ~50 V/rad) caused the Kalman estimate to diverge from reality. The mechanism:

1. High gains produce large correction voltages, frequently saturating the motor at full power.
2. Under saturation, `u_cancel` is computed at the **estimated** angle, but the arm is at a different angle. The linearized voltage fed to the Kalman is therefore incorrect.
3. The Kalman's state prediction drifts. The drift biases the next gravity cancellation, which biases the next linearized voltage, creating a positive feedback loop between estimation error and control error.

With moderate gains (K ~[20, 2]), the motor rarely saturates during steady-state tracking. Transient saturation during fast moves is acceptable because the Kalman rapidly corrects once the arm decelerates.

The default cost weights (`qPosition=0.5, qVelocity=5.0, rVoltage=12.0`) produce K ~[20, 2], comparable to `ArmController`'s hand-tuned PD (kP=15, kD=1). This is not a coincidence — both controllers face the same saturation-driven estimation coupling, and the viable gain range is set by the physics, not the control strategy.

This is the fundamental tradeoff of feedback linearization under state estimation error: the linearization quality depends on the estimate, but the estimate depends on the linearization. Higher gains stress this coupling. PD and LQR have the same constraint; LQR just makes it easier to reach the viable range without trial-and-error.

## Trajectory Management

Reuses `PositionTrajectoryManager` + `SCurvePosition` unchanged. The S-curve provides 7-phase jerk-limited profiles with asymmetric acceleration/deceleration limits.

### Per-move gravity-aware limits

Before each trajectory, `computeMoveLimits()` caps velocity, acceleration, and deceleration based on the worst-case gravity torque across the move's angular sweep. The sweep is split at the midpoint: the first half constrains acceleration, the second half constrains deceleration. `ArmFeedforward.maxAchievableAcceleration()` and `maxAchievableVelocity()` are queried with the live hub voltage.

On replan (tracking error exceeds threshold), limits are recomputed for the remaining sweep from the predicted position to the target.

### Replanning

If the predicted position diverges from the trajectory by more than `replanThresholdRad` (default 15 degrees), the trajectory is replanned from the predicted state. This handles disturbances (someone pushes the arm) and initial transients. The replan preserves position and velocity continuity but not jerk continuity.

## Hard Stop Handling

Coast (cut power) when **both** the target and the predicted position are near the same hard stop. The dual check prevents false coasting when the arm is merely passing through a stop zone on its way to a distant target.

When waking from coast, the Kalman state is reset to the nearest hard stop angle with zero velocity, and a new trajectory is planned from there.

## Control Loop Summary

```
 1. Read sensors           measuredPos = ticksToRad(encoder)
                            measuredVel = tpsToRadPerSec(encoder)

 2. Kalman update           correct(u_linear_prev, [measuredPos, measuredVel])
                            predict(u_linear_prev, dt)

 3. Latency compensation    predicted = plant.calculateX(xhat, u_linear, 0.030)

 4. Coast check             if target AND position near same hard stop: coast, return

 5. Trajectory update       sample trajectory; replan if tracking error > threshold

 6. Layer 1: Linearize      u_cancel = kg*cos(predPos) + ks*sign(predVel)

 7. Layer 2a: Feedforward   u_ff = kv*trajVel + ka*trajAccel

 8. Layer 2b: Feedback      u_lqr = K * [trajPos - predPos, trajVel - predVel]'

 9. Output                  u_total = u_cancel + u_ff + u_lqr
                            power = clamp(u_total / hubVoltage, -1, 1)

10. Kalman bookkeeping      u_linear = (power * hubVoltage) - u_cancel
```

Step 10 closes the loop: the linearized voltage from this cycle becomes the Kalman input for the next cycle.

## API Surface

Drop-in replacement for `ArmController`. Same methods: `setTarget()`, `update()`, `isAtTarget()`, `getEstimatedPositionRad()`, `getTrajectoryPositionRad()`, `getMode()`, `reset()`, `writeTelemetry()`. Same `Mode` enum (`TRACKING`, `COASTING`). Same static `Params` class pattern.

The only `Params` difference: `kP`/`kD` are replaced by `qPosition`/`qVelocity`/`rVoltage`.

## Files

| File | Purpose |
|------|---------|
| `ControlLib/.../control/VerticalArmController.java` | Controller (~320 lines) |
| `ControlLib/.../control/VerticalArmControllerTest.java` | 12 unit tests |

## Reused Components

| Component | Source | Used for |
|-----------|--------|----------|
| `PositionTrajectoryManager` + `SCurvePosition` | `controllib/motion/` | Trajectory generation and replanning |
| `KalmanFilter<N2,N1,N2>` | WPILib | State estimation on linearized plant |
| `LinearQuadraticRegulator<N2,N1,N2>` | WPILib | Feedback gain computation |
| `LinearSystemId.identifyPositionSystem()` | WPILib | Plant model from kv/ka |
| `ArmFeedforward` | WPILib | Only for `maxAchievableAcceleration/Velocity` in `computeMoveLimits` |
| `ArmMotorSim` + `EncoderSim` | `controllib/sim/` | Simulation for testing |

## Open Questions

- **On-robot validation**: The LQR gain tuning constraint (K must stay moderate to avoid saturation-driven Kalman divergence) was discovered in simulation. On-robot validation may reveal a different viable gain range due to real friction, backlash, and compliance that the simulation doesn't capture.
- **Sysid**: Feedforward gains, encoder zero offset, and LQR cost weights need empirical tuning. The defaults are from simulation.
- **Velocity source**: Like `ArmController`, this controller uses `getVelocity()` directly. The Kalman filter's velocity estimate could be substituted if it proves better on hardware.
