# Arm Controller Design Report

## Problem

The previous arm controller used simple feedforward (`Ks*sign(v) + Kg*cos(theta) + Kv*v + Ka*a`) plus PD on position error. This caused oscillation, particularly when the arm moved in the direction gravity was pulling. The feedback controller reacted to stale, noisy sensor data, overshot, and overcorrected. The gravity-assisted direction was worst because the system lacked natural damping in that direction — the feedforward subtracted voltage to slow down, and any phase lag in feedback caused overshoot.

## Physical System

- Single-joint vertical arm, 100:1 gear ratio, 28 ticks/rev motor encoder
- 270 degrees range of motion: 45 degrees below horizontal in front (encoder 0) to 45 degrees below horizontal in back
- 5 degrees of backlash in the gearbox
- Motor encoder only (no absolute encoder), measured through the backlash
- REV Hub encoder: position sampled at 100 Hz, velocity from a 50 ms rolling window quantized to 20 TPS
- Control loop ~16 ms with 5 ms std dev jitter, plus occasional 5 ms spikes from I2C sensor reads
- Encoder samples are asynchronous to the control loop (0-10 ms stale, unknown timestamp)

## Design Goals

### 1. Smooth trajectories with asymmetric accel/decel

The arm moves through a gravity field. When gravity assists the motion, less motor torque is needed to accelerate but more is needed to decelerate. Symmetric profiles would either under-utilize the motor on gravity-assisted moves or exceed torque limits on gravity-opposed moves. S-curve (jerk-limited) profiles prevent mechanical shock and reduce excitation of structural resonances.

Decision: use `SCurvePosition` via `PositionTrajectoryManager` with separate acceleration and deceleration limits. The trajectory is computed once per move and sampled at each loop iteration. If tracking error exceeds a configurable threshold, the trajectory is replanned from the current state using `resetFromMeasurement()`.

### 2. Accurate gravity compensation via feedforward

The `cos(theta)` gravity term dominates the arm's dynamics across the 270 degree range. Feedforward must handle this — the feedback controller should only deal with residual errors.

Decision: use WPiLib's `ArmFeedforward` with the RK4-accurate `calculateWithVelocities` method (via the variable-dt overload). The feedforward computes the voltage needed for the trajectory's velocity and acceleration targets, with gravity compensation at the **predicted actual arm angle** (not the trajectory position). This is critical — gravity acts at the arm's actual angle, and indexing gravity compensation off the trajectory position caused instability in the previous controller: when the arm got ahead of the profile in the gravity-assisted direction, the profile-based gravity comp was too weak, under-compensating and letting the arm accelerate further ahead in a positive feedback loop.

### 3. Latency compensation via Kalman filter

The sensor-to-actuator latency is roughly 25-40 ms: up to 10 ms from async encoder sampling, 25 ms center-of-window delay on velocity, and one loop period before the computed voltage is applied. Without compensation, the feedback controller reacts to where the arm was, not where it is.

Decision: a linear Kalman filter on the 2-state motor dynamics model (position, velocity) estimates current state and predicts forward by a configurable latency compensation interval. The linear model does not include gravity — gravity is handled entirely by feedforward (see "Gravity-corrected Kalman input" below).

### 4. Velocity measurement from `getVelocity()`

During the design discussion, there was initial excitement about the Kalman filter deriving velocity from position measurements using the plant model. However, experience with the flywheel controller showed that `getVelocity()` — despite its 50 ms window, 20 TPS quantization, and 25 ms latency — was unbeatable as a velocity source. The asynchronous sampling (0-10 ms unknown staleness) introduces noise that the Kalman filter cannot model or remove, because the timing uncertainty is not in the measurement but in the timestamp.

Decision: the Kalman filter observes both position and velocity directly from the encoder (C matrix = identity). It does not attempt to derive velocity from position differentiation.

### 5. PD feedback, not PID or LQR

For a 2-state system (position, velocity), LQR produces exactly two gains — mathematically equivalent to PD. LQR provides a principled way to pick the gains from Q and R matrices, but the resulting controller is the same. PID's integral term was not needed because the feedforward handles the steady-state gravity load and the Kalman filter eliminates the measurement lag that was causing oscillation.

Decision: manual PD (`kP * posError + kD * velError`) rather than `PIDController`, because PIDController computes its own derivative from consecutive position measurements, which conflicts with using the Kalman-predicted velocity for the derivative term.

### 6. Coast near hard stops

When the arm is within 10 degrees of a hard stop and the target is also near that stop, the controller cuts power and lets the arm rest against the physical stop.

## Implementation Decisions

These emerged during implementation and testing, not from the initial design discussion.

### Gravity-corrected Kalman input

**The most important implementation discovery.** The linear Kalman filter's plant model is `dx/dt = Ax + Bu` where A and B capture only the motor's Kv/Ka dynamics — no gravity, no friction. The commanded voltage `u` includes gravity compensation from the feedforward (`Kg*cos(theta)`) and friction compensation (`Ks*sign(v)`). If this full voltage is passed to the Kalman filter, the linear model interprets it all as motor dynamics input and predicts far more acceleration than actually occurs. This caused a consistent ~6 degree steady-state bias that did not improve with higher PD gains.

The fix: subtract the gravity and friction terms from the voltage before passing it to the Kalman filter.

```
u_linear = u_actual - Kg*cos(theta) - Ks*sign(v)
```

This way the linear model sees only the portion of voltage that produces linear dynamics, and its predictions match reality. The `u_actual` is the clamped voltage (what the motor actually received after power saturation), not the unclamped desired voltage — this prevents observer divergence when the motor saturates.

### Coast zone requires both target and position

The initial implementation coasted whenever the arm position was near a hard stop. This caused the arm to immediately coast and fall when starting from the front hard stop, even when targeting a position far away. The fix: coast only when both the target and the current position are near the same hard stop. This allows the arm to pass through the stop zone on its way to a distant target without cutting power.

### Trajectory replanning from predicted state

When tracking error exceeds the threshold, the trajectory is replanned from the Kalman filter's predicted state (after latency compensation), not from the raw encoder measurement. This avoids injecting measurement noise into the trajectory initial conditions.

### Variable dt handling

The control loop has significant timing jitter (16 ms +/- 5 ms). Three components need real dt:
- `KalmanFilter.predict(u, dt)` re-discretizes the plant model each call
- `ArmFeedforward.calculate(angle, currentVel, nextVel, dt)` (deprecated overload) constructs a fresh feedforward with the actual dt for RK4 accuracy
- The trajectory manager uses its own wall clock internally, independent of dt

### Per-move trajectory limits

The S-curve trajectory limits (max velocity, acceleration, deceleration) are computed per-move based on the worst-case gravity torque across the sweep, rather than using fixed global values. This allows the arm to move faster when gravity is favorable and remain safe when it isn't.

For each move, the sweep is split at the midpoint. The worst-case angle in the first half determines the acceleration limit; the worst-case angle in the second half determines the deceleration limit. The worst-case angle across the full sweep determines the max velocity limit. "Worst case" is where `|cos(theta)|` is maximized — the candidates are the sweep endpoints plus any horizontal crossing (theta = 0 or theta = -pi) within the range. All computed limits are capped at the global PARAMS maximums, which serve as a mechanical safety ceiling.

The limits are computed using `ArmFeedforward.maxAchievableAcceleration` and `maxAchievableVelocity` with the live hub voltage. Acceleration queries use `PARAMS.maxVelRad` as the velocity argument (conservative: back-EMF at max velocity leaves the least torque headroom).

On replan (tracking error exceeds threshold), limits are recomputed for the remaining sweep from the predicted position to the target. This gives tighter limits for the remainder when the worst-case angles have already been passed.

## Architecture

```
SCurvePosition --> trajectory (p, v, a setpoints)
                        |
                        +---> ArmFeedforward(predicted angle, traj v, traj a) ---> ff voltage
                        |
                        +---> compare with predicted state ---> PD ---> fb voltage
                        |
KalmanFilter ---> predict forward (latency comp) ---> predicted state
                        |
                total voltage = ff + fb
                power = clamp(voltage / hubVoltage, -1, 1)
```

The Kalman filter receives `u_linear` (gravity/friction subtracted). The feedforward receives the trajectory velocity/acceleration targets but uses the **predicted actual position** for gravity compensation — gravity torque depends on where the arm physically is, not where the trajectory expects it. PD feedback operates on the error between the latency-compensated predicted state and the trajectory setpoint.

## Files

| File | Purpose |
|------|---------|
| `ControlLib/.../control/ArmController.java` | Main controller class |
| `ControlLib/.../sim/ArmMotorSim.java` | Nonlinear plant simulation with gravity, hard stops, EncoderSim |
| `ControlLib/.../control/ArmControllerTest.java` | 7 unit tests |
| `ControlLib/.../hardware/IMotor.java` | Added `getPosition()` to interface |

## Open Questions

- **Sysid**: feedforward gains (Ks, Kg, Kv, Ka), encoder zero offset, and possibly S-curve constraints need to be found empirically. Out of scope for this implementation.
- **Coast at front hard stop**: in simulation, coasting at the front stop causes the arm to fall because gravity pulls it away. On the real robot, physical contact with the stop may hold the arm. If not, the coast zone may need to be asymmetric (only at the back stop, or different behavior per stop).
- **Velocity source**: `getVelocity()` was assumed to be the best velocity source based on flywheel experience. This should be validated on the arm — if the Kalman filter's velocity estimate turns out to be better in this context (different dynamics, different noise profile), the measurement noise covariance can be retuned.
