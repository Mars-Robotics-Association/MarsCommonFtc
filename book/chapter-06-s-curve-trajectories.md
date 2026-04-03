# Chapter 6: S-Curve Jerk-Limited Trajectories

The trapezoidal profile from Chapter 5 bounds velocity and acceleration, but it has a hidden problem: acceleration changes instantaneously at phase boundaries. The derivative of acceleration — called **jerk** — is infinite at these points. Infinite jerk means infinite force rate-of-change, which excites mechanical resonances, causes gear backlash, and produces audible noise. S-curve profiles solve this by bounding jerk explicitly.

## 6.1 Why Jerk Matters

Consider a trapezoidal velocity profile accelerating from rest. At t = 0, acceleration jumps from 0 to `maxAcceleration` in zero time. The motor torque jumps correspondingly. The gear teeth slam into each other. The mechanism flexes. A high-speed camera would show the structure ringing like a bell.

Now consider the same move with an S-curve profile. Acceleration ramps smoothly from 0 to `maxAcceleration` over a finite time. The torque ramps smoothly. The mechanism does not ring. The move is quieter, gentler on the hardware, and — counterintuitively — often faster because the mechanism can use higher acceleration limits without exciting resonances.

### The Jerk Hierarchy

Each level of the motion hierarchy bounds one more derivative:

| Profile Type | Bounds | Jerk |
|---|---|---|
| Step command | Nothing | Infinite |
| Trapezoidal | Velocity, acceleration | Infinite at transitions |
| S-curve (linear jerk) | Velocity, acceleration, jerk | Discontinuous at phase boundaries |
| S-curve (sinusoidal) | Velocity, acceleration, jerk | Continuous everywhere |

MarsCommonFtc provides both linear-jerk and sinusoidal S-curve profiles.

## 6.2 The 7-Phase S-Curve

The piecewise-linear S-curve (`SCurvePosition`) divides the move into seven phases, each with constant jerk:

```
acceleration
  ^
  |     T2 (constant a)
  |    ┌──────────┐
  |   /            \
  |  / T1           \ T3
  | /  (ramp up)     \ (ramp down)
  |/                  \
  +────────────────────\──────────────> time
                        \            /
                         \ T5       / T7
                          \ (ramp) / (ramp)
                           \      /
                            \────/
                            T6 (constant -a)

velocity
  ^
  |         /──────────────\
  |        /                \
  |       /                  \
  |      /                    \
  |     /                      \
  +----/                        \---> time
```

The seven phases are:

| Phase | Jerk | Acceleration | Description |
|---|---|---|---|
| T1 | `+jMax` | 0 → `aMaxAccel` | Ramp up acceleration |
| T2 | 0 | constant `aMaxAccel` | Hold maximum acceleration |
| T3 | `-jMax` | `aMaxAccel` → 0 | Ramp down acceleration to cruise |
| T4 | 0 | 0 | Cruise at `vPeak` |
| T5 | `-jMax` | 0 → `-aMaxDecel` | Ramp up deceleration |
| T6 | 0 | constant `-aMaxDecel` | Hold maximum deceleration |
| T7 | `+jMax` | `-aMaxDecel` → 0 | Ramp down deceleration to stop |

Each phase is a cubic polynomial in time:

$$p(t) = p_0 + v_0 t + \tfrac{1}{2} a_0 t^2 + \tfrac{1}{6} j t^3$$

$$v(t) = v_0 + a_0 t + \tfrac{1}{2} j t^2$$

$$a(t) = a_0 + j t$$

The jerk is constant within each phase, so the acceleration is linear, the velocity is quadratic, and the position is cubic.

### Phase Durations

The durations of T1, T3, T5, and T7 are determined by the jerk and acceleration limits:

$$T_1 = T_3 = \frac{a_{\max,\text{accel}}}{j_{\max}} \qquad T_5 = T_7 = \frac{a_{\max,\text{decel}}}{j_{\max}}$$

The durations of T2, T4, and T6 depend on the total distance. If the distance is short, T2, T4, and T6 may be zero — the profile becomes purely triangular in acceleration and velocity.

### The vPeak Solver

The key unknown in the profile is `vPeak` — the maximum velocity reached during cruise. It is found by binary search. The total distance as a function of `vPeak` is strictly increasing, so bisection converges reliably:

```java
double lo = v0, hi = vMax;
for (int i = 0; i < 64; i++) {
    double mid = (lo + hi) * 0.5;
    double d = accelDistance(v0, mid, aMaxAccel, jMax)
             + decelDistance(mid, aMaxDecel, jMax);
    if (d < distRemaining) lo = mid;
    else hi = mid;
}
```

Sixty-four iterations give 2^-64 precision — far more than any physical system can distinguish.

## 6.3 Non-Zero Initial Conditions

Real mechanisms are rarely at rest when a new target is commanded. The arm is already moving. The flywheel is already spinning. The S-curve must handle non-zero initial velocity **and** non-zero initial acceleration.

`SCurvePosition` handles this with two optional prefix phases:

### The a0 Prefix

If the initial acceleration is non-zero, a prefix phase applies jerk for `|a0|/jMax` seconds to bring acceleration to zero:

```
a(t) = a0 + j * t   where j = -sign(a0) * jMax
```

After this phase, acceleration is zero and the main 7-phase profile begins.

### Helpful Acceleration Preservation

There is an important optimization: if the initial acceleration is already pointing toward the target, it is **helpful** and should not be zeroed. For example, if the arm is moving upward toward its target and accelerating upward, the existing acceleration is useful and the a0 prefix is skipped.

```java
if (vStart * dir < 0 && a0 * dir >= 0) {
    // Velocity is away from target but a0 is already decelerating it — preserve a0
    aStart = a0;
}
```

### The Braking Prefix

If the initial velocity is in the **wrong direction** (away from the target), a braking prefix brings velocity to zero before the main profile begins. This is a 3-sub-phase jerk-limited deceleration:

```
Phase B1: Apply -jMax to reduce acceleration (if needed)
Phase B2: Hold constant deceleration
Phase B3: Apply +jMax to bring acceleration to zero at v=0
```

The braking profile can be triangular (no constant-deceleration phase) or trapezoidal, depending on how much velocity needs to be shed.

## 6.4 Asymmetric Acceleration and Deceleration

Many mechanisms have different acceleration and deceleration limits. A flywheel can accelerate quickly (full voltage applied) but decelerates slowly (only back-EMF and friction to slow it down, unless active braking is used). An arm accelerates faster going down (gravity assists) than going up (gravity opposes).

`SCurvePosition` accepts independent `aMaxAccel` and `aMaxDecel` parameters:

```java
new SCurvePosition(
    p0, pTarget,       // start and target position
    v0, a0,            // initial velocity and acceleration
    vMax,              // maximum velocity
    aMaxAccel,         // maximum acceleration
    aMaxDecel,         // maximum deceleration (independent)
    jMax               // maximum jerk
);
```

The phase durations reflect the asymmetry:

$$T_1 = T_3 = \frac{a_{\max,\text{accel}}}{j_{\max}} \qquad T_5 = T_7 = \frac{a_{\max,\text{decel}}}{j_{\max}}$$

The triangular threshold also differs for acceleration and deceleration:

$$\Delta v_{\text{accel,min}} = \frac{a_{\max,\text{accel}}^2}{j_{\max}} \qquad \Delta v_{\text{decel,min}} = \frac{a_{\max,\text{decel}}^2}{j_{\max}}$$

If the velocity change during acceleration is less than `vAccelMin`, the acceleration phase is triangular (no constant-acceleration segment). Similarly for deceleration.

## 6.5 SCurveVelocity: Velocity-to-Velocity Profiles

`SCurvePosition` generates position trajectories (p, v, a as functions of time). `SCurveVelocity` generates velocity trajectories (v, a as functions of time) for velocity-controlled mechanisms like flywheels.

The structure is simpler — only three phases:

| Phase | Jerk | Acceleration | Description |
|---|---|---|---|
| T1 | `+jInc` or `-jInc` | `a0` → `aPeak` | Ramp acceleration |
| T2 | 0 | constant `aPeak` | Hold acceleration |
| T3 | `-jDec` | `aPeak` → 0 | Ramp acceleration to zero |

The profile transitions from velocity `v0` to velocity `v1` with bounded jerk. The peak acceleration $a_{\text{peak}}$ is computed from the velocity change and jerk limits. For a triangular profile (no constant-acceleration phase):

$$\Delta v = \frac{a_{\text{peak}}^2 - a_0^2}{2 j_{\text{inc}}} + \frac{a_{\text{peak}}^2}{2 j_{\text{dec}}} \quad \Rightarrow \quad a_{\text{peak}}^2 = \frac{2 \Delta v + a_0^2 / j_{\text{inc}}}{1/j_{\text{inc}} + 1/j_{\text{dec}}}$$

`SCurveVelocity` supports asymmetric jerk: `jInc` for increasing acceleration and `jDec` for decreasing. This matters because the back-EMF constraint is asymmetric — accelerating into high velocity is more voltage-constrained than decelerating.

### Single-Phase Fallback

When the velocity change is too small to even null the initial acceleration, a single constant-jerk phase is used:

```java
if (dv is too small) {
    // Single phase: apply constant jerk for the entire transition
    double t = solveQuadraticForTime(dv, a0, jMax);
}
```

This avoids the pathological case where the three-phase decomposition produces negative phase durations.

## 6.6 Back-EMF-Aware Limit Finding

The S-curve profile needs acceleration and jerk limits as inputs. But what are reasonable values? For a flywheel, the limits are constrained by back-EMF: at high velocity, less voltage is available for acceleration.

`SCurveVelocity` provides two binary search methods to find the maximum limits that do not violate the voltage constraint:

### findMaxAMax

Finds the maximum acceleration achievable from rest (v = 0) given the voltage budget:

```java
double aMax = SCurveVelocity.findMaxAMax(
    v0, v1,      // start and target velocity
    jInc,        // increasing jerk limit
    voltage,     // battery voltage
    kS, kV, kA   // motor constants
);
```

The search binary-searches over `aMax`, simulating the profile at 1000 sample points and checking whether the required voltage exceeds the available voltage at each point:

```java
double availableVoltage = voltage - kS - kV * Math.abs(v);
double motorAMax = availableVoltage / kA;
if (a > motorAMax) violates = true;
```

### findMaxJDec

Finds the maximum deceleration jerk that does not violate the voltage constraint:

```java
double jDec = SCurveVelocity.findMaxJDec(
    v0, v1, a0, aMax, jInc,
    voltage, kS, kV, kA
);
```

Both methods use 30 iterations of binary search, which gives sufficient precision for any practical mechanism. Chapter 10 covers these methods in the context of automated profile tuning.

## 6.7 Using S-Curves in Controllers

### ArmController with SCurvePosition

`ArmController` uses `PositionTrajectoryManager` with `SCurvePosition` as the default trajectory type:

```java
trajectory = new PositionTrajectoryManager(
    maxVelRad, maxAccelRad, maxDecelRad, maxJerkRad,
    toleranceRad, telemetry, clock);
// Uses SCurvePosition by default
```

Each cycle, the trajectory manager samples the current S-curve state:

```java
trajectory.update();
double trajPos = trajectory.getPosition();
double trajVel = trajectory.getVelocity();
double trajAccel = trajectory.getAcceleration();
```

These values feed into the feedforward and feedback computations:

```java
// Feedforward uses trajectory velocity and acceleration
double nextVel = trajVel + trajAccel * dt;
double ffVoltage = feedforward.calculate(predictedPosRad, trajVel, nextVel, dt);

// PD feedback uses trajectory vs. predicted state error
double pdVoltage = kP * (trajPos - predictedPosRad)
                 + kD * (trajVel - predictedVelRad);
```

### VelocityMotorPF with SCurveVelocity

`VelocityMotorPF` uses `VelocityTrajectoryManager` with `SCurveVelocity`:

```java
trajectory = new VelocityTrajectoryManager(
    aMax, jInc, jDec, toleranceTps, telemetry);
```

The trajectory provides velocity and acceleration for feedforward:

```java
double v = trajectory.getVelocity();
double a = trajectory.getAcceleration();
double ff = kS * Math.signum(v) + kV * v + kA * a;
ff /= voltage;  // battery compensation
```

The proportional gain is suppressed during high acceleration:

```java
double kPEffective = kP * Math.max(0, 1.0 - Math.abs(a) / aMax);
double power = ff + kPEffective * (v - actualVelocity);
```

At peak acceleration, kP is zero and the controller runs on pure feedforward. As acceleration drops to zero, full kP engages.

## 6.8 Mid-Motion Replanning

The `PositionTrajectoryManager` and `VelocityTrajectoryManager` support mid-motion replanning. When the target changes or a tracking error is detected, a new trajectory is seeded from the current state:

```java
// Sample current trajectory state
double lastP = currentTrajectory.getPosition(currentTime);
double lastV = currentTrajectory.getVelocity(currentTime);
double lastA = currentTrajectory.getAcceleration(currentTime);

// Create new trajectory from current state to new target
currentTrajectory = factory.create(
    lastP, newTarget, lastV, lastA,
    vMax, aMaxAccel, aMaxDecel, jMax);
```

The new trajectory starts from the current position, velocity, and acceleration, ensuring **C2 continuity** (position, velocity, and acceleration are continuous; jerk is not). The jerk discontinuity at the replan boundary is an inherent limitation of the piecewise-linear S-curve.

### Replan on Tracking Error

`ArmController` monitors the tracking error and replans when it exceeds a threshold:

```java
if (Math.abs(predictedPosRad - trajPos) > replanThresholdRad) {
    double[] limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage);
    trajectory.updateConfig(limits[0], limits[1], limits[2], maxJerkRad);
    trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad);
}
```

This handles disturbances: if the arm is pushed off the trajectory, the controller replans from the actual position rather than trying to catch up to the original trajectory.

## 6.9 SinCurvePosition: Smooth Acceleration Transitions

`SinCurvePosition` replaces the piecewise-linear acceleration ramps of `SCurvePosition` with **raised-cosine transitions**:

$$a(t') = \frac{A}{2}\left(1 + s \cos\left(\frac{\pi t'}{T}\right)\right)$$

Where $s = -1$ for acceleration onset ($0 \to a_{\max}$), $s = +1$ for acceleration offset ($a_{\max} \to 0$), and $s = 0$ for the constant phase.

### Why Sinusoidal?

The raised-cosine shape has two key properties:

1. **Smooth acceleration at phase boundaries** — The derivative of acceleration (jerk) is zero at the start and end of each transition. There are no jerk discontinuities anywhere in the profile.

2. **Same duration as linear jerk** — The transition from 0 to $a_{\max}$ takes exactly $a_{\max}/j_{\max}$ seconds, matching the piecewise-linear S-curve. The only difference is the shape of the transition.

The peak instantaneous jerk is $\pi j_{\max} / 2$, slightly higher than the linear-jerk $j_{\max}$, but the jerk is continuous — it ramps smoothly through zero at the phase boundaries.

### Smart Phase Merging

`SinCurvePosition` merges adjacent phases when possible, reducing the number of segments and improving smoothness:

**Case A: a0 prefix merged with braking onset** — When both a0 prefix and braking are needed and point in the same direction, they are combined into a single general arc rather than two separate segments.

**Case B: Handoff arc** — When the braking offset phase and the T1 onset phase point in the same direction, they are replaced by a single smooth handoff arc from the braking acceleration to the main acceleration.

**Midpoint combined** — When there is no cruise phase (T4 = 0), the T3 offset and T5 onset are merged into a single arc from `aPeakAccel` to `-aPeakDecel`, avoiding a jerk discontinuity at the acceleration-to-deceleration transition.

### Distance Integration

The raised-cosine acceleration shape requires different integral constants than the linear-jerk case:

$$C_{\text{onset}} = \frac{\pi^2 - 4}{4\pi^2} \approx 0.1487 \qquad C_{\text{offset}} = \frac{\pi^2 + 4}{4\pi^2} \approx 0.3513$$

These constants appear in the distance calculation for each phase. The general arc evaluation (`evalPgen`, `evalVgen`, `evalAgen`) handles arbitrary `aStart → aEnd` transitions, not just 0 to `aMax`.

## 6.10 Choosing Between SCurvePosition and SinCurvePosition

### Use SCurvePosition When

**You want simplicity and speed.** The piecewise-linear formulation uses only cubic polynomials — fast to evaluate and easy to reason about.

**Jerk discontinuities are acceptable.** For most FTC mechanisms, the jerk discontinuities at phase boundaries are not a practical problem. The mechanism is not stiff enough to ring at the frequencies involved.

**You need the default.** `PositionTrajectoryManager` uses `SCurvePosition` by default.

### Use SinCurvePosition When

**You want the smoothest possible motion.** The continuous jerk eliminates all acceleration discontinuities. This matters for delicate mechanisms, high-speed cameras, or when you want to impress the judges.

**You are running ControlLab.** The trajectory visualization tab shows the smooth sinusoidal transitions clearly, making it easier to verify profile correctness.

**You want factory injection.** `PositionTrajectoryManager` accepts a factory function, so you can inject `SinCurvePosition::new` as the trajectory type:

```java
trajectory = new PositionTrajectoryManager(
    maxVel, maxAccel, maxDecel, maxJerk, tolerance, telemetry, clock,
    SinCurvePosition::new);  // factory injection
```

## 6.11 Summary

S-curve profiles bound jerk explicitly, eliminating the acceleration discontinuities of trapezoidal profiles. `SCurvePosition` uses piecewise-linear jerk for simplicity. `SCurveVelocity` generates velocity-to-velocity profiles for flywheels. `SinCurvePosition` uses raised-cosine transitions for smooth acceleration everywhere.

The key insights are:

- **Jerk is the derivative of acceleration** — bounding it reduces mechanical stress
- **7 phases** for position S-curves: ramp up accel, hold, ramp down, cruise, ramp up decel, hold, ramp down
- **3 phases** for velocity S-curves: ramp accel, hold, ramp down
- **Non-zero initial conditions** require prefix phases for acceleration nulling and velocity braking
- **Asymmetric limits** handle mechanisms that accelerate and decelerate differently
- **Back-EMF-aware binary search** finds the maximum limits that respect the voltage constraint
- **Mid-motion replanning** seeds new trajectories from the current state for C2 continuity

The next chapter covers sinusoidal trajectories in more depth, and Chapter 8 introduces online trajectory generation with the Ruckig library for real-time multi-DOF planning.
