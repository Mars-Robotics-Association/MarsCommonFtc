# Chapter 9: Trajectory Management & Replanning

The trajectory profiles from Chapters 5–7 are pure math: given initial conditions and a target, they compute the complete time-parameterized path. But a real mechanism needs more than a profile. It needs a clock to convert wall time to trajectory time. It needs to detect when the target changes and create a new profile from the current state. It needs to avoid replanning on noise — tiny target jitters that would produce a flood of unnecessary trajectory recomputes. And it needs telemetry so the driver station shows what the trajectory is doing.

`PositionTrajectoryManager` and `VelocityTrajectoryManager` provide this management layer. They wrap the trajectory profiles from Chapters 5–7 and add clock management, tolerance-based change detection, mid-motion replanning with continuity guarantees, state injection from external measurements, and telemetry output.

## 9.1 PositionTrajectoryManager

`PositionTrajectoryManager` manages a single `PositionTrajectory` (typically an `SCurvePosition` or `SinCurvePosition`) and handles all the bookkeeping around planning, replanning, and sampling.

### Architecture

The manager holds:

- **A trajectory factory** — a `TrajectoryFactory` functional interface that creates new trajectories. Defaults to `SCurvePosition::new`, but can be replaced with `SinCurvePosition::new` or any other implementation.
- **A clock** — a `LongSupplier` returning nanoseconds. Defaults to `System::nanoTime` in production; injectable for testing.
- **Cached kinematic state** — the last-sampled position, velocity, and acceleration.
- **A `SetOnChange<Double>` wrapper** — triggers replanning only when the target changes by more than a configurable tolerance.
- **Motion limits** — `vMax`, `aMaxAccel`, `aMaxDecel`, `jMax`, updatable at runtime.

### The TrajectoryFactory Interface

The factory decouples the manager from any specific trajectory implementation:

```java
@FunctionalInterface
public interface TrajectoryFactory {
    PositionTrajectory create(
            double p0, double pTarget,
            double v0, double a0,
            double vMax, double aMaxAccel, double aMaxDecel, double jMax);
}
```

The default is `SCurvePosition::new`. To use sinusoidal trajectories:

```java
PositionTrajectoryManager manager = new PositionTrajectoryManager(
        maxVelRad, maxAccelRad, maxDecelRad, maxJerkRad,
        toleranceRad, telemetry, clock,
        SinCurvePosition::new);
```

No other code changes are needed. The manager creates, samples, and replans trajectories through the factory regardless of the underlying implementation.

### Constructor Overloads

Four constructors cover the common combinations:

```java
// Production: System.nanoTime clock, SCurvePosition factory
public PositionTrajectoryManager(vMax, aMaxAccel, aMaxDecel, jMax,
        pChangeTolerance, telemetry)

// Test: injectable clock, SCurvePosition factory
public PositionTrajectoryManager(vMax, aMaxAccel, aMaxDecel, jMax,
        pChangeTolerance, telemetry, clock)

// Production: System.nanoTime clock, custom factory
public PositionTrajectoryManager(vMax, aMaxAccel, aMaxDecel, jMax,
        pChangeTolerance, telemetry, factory)

// Full: injectable clock, custom factory
public PositionTrajectoryManager(vMax, aMaxAccel, aMaxDecel, jMax,
        pChangeTolerance, telemetry, clock, factory)
```

All four delegate to the base constructor, which initializes the state to position 0, velocity 0, acceleration 0, and creates an initial trajectory to position 0 (a trivial no-op trajectory).

## 9.2 The Update Loop

The manager's `update()` method is called once per control loop iteration. It does two things:

1. **Sample the trajectory** at the current wall time and cache the result.
2. **Check for pending replans** and execute them if needed.

```java
public void update() {
    double seconds = (clock.getAsLong() - startTime) / 1e9;

    lastP = currentTrajectory.getPosition(seconds);
    lastV = currentTrajectory.getVelocity(seconds);
    lastA = currentTrajectory.getAcceleration(seconds);

    telemetry.addData("trajectory position", "%.3f", lastP);
    telemetry.addData("trajectory velocity", "%.3f", lastV);
    telemetry.addData("trajectory acceleration", "%.1f", lastA);

    if (!Double.isNaN(pendingTarget)) {
        changeTrajectory(pendingTarget);
    }
}
```

The sampling happens first, then the replan. This ordering matters: when `setTarget()` is called, the new target is stored as `pendingTarget` via `SetOnChange`, and `update()` is triggered. The current trajectory is sampled at the current time to get the latest `lastP/lastV/lastA`, and then `changeTrajectory()` creates a new trajectory seeded from that state. This ensures the new trajectory starts exactly where the old one left off.

### Clock Management

The manager converts nanosecond wall time to trajectory-local seconds:

```java
double seconds = (clock.getAsLong() - startTime) / 1e9;
```

`startTime` is reset every time a new trajectory is created. The trajectory is sampled at `seconds` — the elapsed time since the trajectory was planned. If `seconds` exceeds `getTotalTime()`, the trajectory returns its final state (target position, zero velocity, zero acceleration).

The injectable clock is essential for testing. Instead of depending on real time, tests use an `AtomicLong` that they advance manually:

```java
AtomicLong clock = new AtomicLong(0);
PositionTrajectoryManager m = new PositionTrajectoryManager(
        5, 3, 3, 10, 0.01, NO_OP_TELEMETRY, clock::get);

m.setTarget(50.0);
clock.set((long) 14e9);  // jump to t = 14 seconds
m.update();

assertEquals(50.0, m.getPosition(), 0.01);
```

## 9.3 Tolerance-Based Change Detection

Not every `setTarget()` call should trigger a replan. If the target changes by 0.001 radians due to sensor noise or rounding, replanning would waste computation and produce a barely-different trajectory. `SetOnChange<Double>` prevents this.

### SetOnChange

`SetOnChange` wraps a value and fires a callback only when the value changes by more than a configurable epsilon:

```java
targetPosition = SetOnChange.ofDouble(
        0.0,              // initial value
        pChangeTolerance, // epsilon
        (p) -> {          // callback: fires only on significant change
            pendingTarget = p;
            update();
        });
```

The `DoubleBackend` inside `SetOnChange` implements two behaviors:

**Zero snap.** Values with magnitude less than epsilon are snapped to exactly zero. This prevents the trajectory from hunting around the origin due to floating-point noise.

**Epsilon comparison.** The callback only fires if `|newValue - currentValue| > epsilon`. Small changes are silently absorbed.

```java
private double snap(double v) {
    return Math.abs(v) < epsilon ? 0.0 : v;
}

private boolean changed(double v) {
    if (epsilon == 0.0) return v != value;
    return Math.abs(v - value) > epsilon;
}
```

The tolerance is specified in the manager's constructor. Typical values:

| Mechanism | Tolerance | Units |
|---|---|---|
| Arm position | `Math.toRadians(0.5)` | 0.5 degrees |
| Elevator position | 0.01 | meters |
| Turret angle | `Math.toRadians(1.0)` | 1 degree |

If the tolerance is too small, every call to `setTarget()` replans. If too large, the mechanism ignores legitimate target changes. Start with the mechanism's encoder resolution — there's no point in replanning for a target change smaller than one encoder tick.

## 9.4 Mid-Motion Replanning

When the target changes significantly (beyond tolerance), `changeTrajectory()` creates a new trajectory from the current sampled state:

```java
private void changeTrajectory(double pTarget) {
    currentTrajectory = factory.create(
            lastP, pTarget, lastV, lastA,
            vMax, aMaxAccel, aMaxDecel, jMax);
    startTime = clock.getAsLong();
    pendingTarget = Double.NaN;
}
```

The key insight: **the new trajectory is seeded from `lastP`, `lastV`, and `lastA`** — the position, velocity, and acceleration sampled from the old trajectory at the current time. This guarantees continuity:

- **Position is continuous.** The new trajectory starts at `lastP`, which is where the old trajectory was.
- **Velocity is continuous.** The new trajectory starts at `lastV`.
- **Acceleration is continuous.** The new trajectory starts at `lastA`.
- **Jerk is not continuous.** The new trajectory's a0 prefix starts with jerk = 0 (by design of the S-curve and sinusoidal prefix phases), but the old trajectory's jerk at the replan instant is generally non-zero.

The jerk discontinuity is small in practice. The a0 prefix in `SCurvePosition` and `SinCurvePosition` ramps acceleration to zero at $j_{\max}$ or with a sinusoidal shape, starting gently from the inherited acceleration. Rapid target changes produce visible acceleration nodes in telemetry, but individual replans are nearly invisible in the position trace.

### Same-Direction Replanning

When the target changes but the direction doesn't (e.g., target changes from 100 to 50 while moving forward), the new trajectory starts from the current velocity and decelerates smoothly. No braking prefix is needed because the velocity is already in the correct direction.

### Direction Reversal

When the target reverses (e.g., moving forward at velocity +10 when the target changes from +100 to -100), the new trajectory's braking prefix brings the velocity to zero before the main 7-phase profile begins. This produces a smooth deceleration, pause, and re-acceleration in the opposite direction — all computed automatically by the trajectory profile.

### Test Verification

The manager tests verify continuity across both scenarios:

```java
@Test
void midMove_sameDirection_velocityIsContinuous() {
    // Setup: cruising at v ≈ 10 toward target 100
    m.setTarget(100.0);
    clock.set((long) 1e9);
    m.update();
    double vBefore = m.getVelocity();

    // Change target to 50 (same direction)
    m.setTarget(50.0);
    clock.set((long) (1e9 + 20_000_000L));
    m.update();
    double vAfter = m.getVelocity();

    assertEquals(vBefore, vAfter, 2.0);
}

@Test
void midMove_targetReversal_velocityIsContinuous() {
    // Same setup: cruising at v ≈ 10
    m.setTarget(100.0);
    clock.set((long) 1e9);
    m.update();
    double vBefore = m.getVelocity();

    // Reverse target
    m.setTarget(-100.0);
    clock.set((long) (1e9 + 20_000_000L));
    m.update();
    double vAfter = m.getVelocity();

    assertEquals(vBefore, vAfter, 2.0);
}
```

The 2.0 tolerance on the velocity comparison accounts for the 20 ms tick during which the new trajectory begins to decelerate. The velocity change over a single tick is small compared to the cruise velocity.

## 9.5 State Injection

Sometimes the trajectory's internal state diverges from reality. The mechanism might be physically disturbed (someone pushes the arm), the encoder might glitch, or the controller might need to re-seed the trajectory after a mode transition. `resetFromMeasurement()` handles this:

```java
public void resetFromMeasurement(double measuredP, double measuredV, double measuredA) {
    this.lastP = measuredP;
    this.lastV = measuredV;
    this.lastA = measuredA;
    this.pendingTarget = Double.NaN;
    changeTrajectory(targetPosition.get());
}
```

This overwrites the cached state with directly measured values and creates a new trajectory from the measured state to the current target. It's used in two main scenarios:

**Tracking error recovery.** When the gap between the trajectory and the measured position exceeds a threshold, the controller injects the measured state:

```java
if (Math.abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
    trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
    trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad);
    trajectory.update();
}
```

**Mode transitions.** When an arm wakes from coasting at a hard stop, the controller resets the trajectory from the hard stop position with zero velocity:

```java
kalman.setXhat(VecBuilder.fill(hardStopAngle, 0));
trajectory.resetFromMeasurement(hardStopAngle, 0);
```

The zero-acceleration overload `resetFromMeasurement(p, v)` is a convenience — it calls `resetFromMeasurement(p, v, 0.0)`. Use this when you don't have a reliable acceleration measurement (which is common — accelerometers are noisy and most FTC encoders don't provide acceleration).

## 9.6 Runtime Configuration Updates

Motion limits can be changed at runtime without creating a new manager:

```java
public void updateConfig(double vMax, double aMaxAccel, double aMaxDecel, double jMax) {
    this.vMax = Math.abs(vMax);
    this.aMaxAccel = Math.abs(aMaxAccel);
    this.aMaxDecel = Math.abs(aMaxDecel);
    this.jMax = Math.abs(jMax);
}
```

The `Math.abs()` calls normalize the signs — the manager stores magnitudes and lets the trajectory profiles handle direction. The new limits take effect on the next trajectory plan (the next `changeTrajectory()` call, whether from a target change or a `resetFromMeasurement()`).

This is critical for gravity-aware arm control. `ArmController.computeMoveLimits()` computes per-move limits based on the worst-case gravity torque across the sweep and updates the trajectory manager before each move:

```java
double[] limits = computeMoveLimits(fromRad, targetAngleRad, hubVoltage);
trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
trajectory.setTarget(targetAngleRad);
```

Chapter 10 explains how these per-move limits are computed from motor physics.

## 9.7 VelocityTrajectoryManager

`VelocityTrajectoryManager` mirrors `PositionTrajectoryManager` but wraps `SCurveVelocity` instead of `PositionTrajectory`. It manages velocity targets instead of position targets.

### Key Differences from PositionTrajectoryManager

**No position tracking.** The manager caches velocity and acceleration but not position. The mechanism's position is a side effect of velocity control, not the primary target.

**Asymmetric jerk.** `VelocityTrajectoryManager` stores `jInc` and `jDec` separately:

```java
public void updateConfig(double aMax, double jInc, double jDec) {
    this.aMax = Math.abs(aMax);
    this.jInc = Math.abs(jInc);
    this.jDec = Math.abs(jDec);
}
```

This supports flywheel-style motors that ramp up quickly but settle slowly (`jInc >> jDec`).

**Opposing acceleration clamping.** When changing direction, the initial acceleration might oppose the new target. `changeTrajectory()` clamps it:

```java
private void changeTrajectory(double vTarget) {
    double dv = vTarget - lastV;

    if (Math.abs(dv) < 1e-6) {
        // Trivial: already at target
        currentTrajectory = new SCurveVelocity(lastV, lastV, 0.0, aMax, jInc, jDec);
    } else {
        double dir = Math.signum(dv);
        double effectiveA0 = lastA;

        // Clamp initial acceleration to not overshoot
        if (dir * effectiveA0 > aMax) {
            effectiveA0 = dir * aMax;
        }

        currentTrajectory = new SCurveVelocity(
                lastV, vTarget, effectiveA0, aMax, jInc, jDec);
    }

    startTime = clock.getAsLong();
    pendingTarget = Double.NaN;
}
```

The clamping prevents a pathological case: if the current acceleration is much larger than `aMax` (which can happen after a config update that lowers limits), passing it directly to `SCurveVelocity` would produce an invalid profile. Clamping to `dir * aMax` ensures the initial acceleration is within the new limits.

**No trajectory factory.** `VelocityTrajectoryManager` always uses `SCurveVelocity`. There is no sinusoidal-jerk equivalent for velocity trajectories.

### Usage

```java
VelocityTrajectoryManager velManager = new VelocityTrajectoryManager(
        aMax,               // max acceleration magnitude
        jMax,               // jerk limit (symmetric by default)
        vChangeTolerance,   // velocity change tolerance
        telemetry);

// Asymmetric jerk: fast ramp-up, slow settle
velManager.updateConfig(aMax, jInc, jDec);

// Command target velocity
velManager.setTarget(1500);  // e.g., RPM for a flywheel

// Each loop:
velManager.update();
double targetVel = velManager.getVelocity();
double targetAccel = velManager.getAcceleration();
```

## 9.8 Telemetry and Debugging

Both managers emit telemetry on every `update()` call:

**PositionTrajectoryManager:**
- `trajectory position` — current trajectory position (format: `%.3f`)
- `trajectory velocity` — current trajectory velocity (format: `%.3f`)
- `trajectory acceleration` — current trajectory acceleration (format: `%.1f`)

**VelocityTrajectoryManager:**
- `trajectory velocity` — current trajectory velocity (format: `%.3f`)
- `trajectory acceleration` — current trajectory acceleration (format: `%.1f`)

The telemetry interface is `TelemetryAddData`:

```java
@FunctionalInterface
public interface TelemetryAddData {
    void addData(String caption, String format, Object value);
}
```

In production, this maps to the FTC SDK's `telemetry.addData()`. In tests, use a no-op lambda:

```java
TelemetryAddData NO_OP = (caption, format, value) -> {};
```

### Debugging Replans

When debugging unexpected behavior, watch for:

1. **Rapid oscillation in acceleration.** This indicates the target is changing faster than the trajectory can settle. Increase the change tolerance or reduce the control loop's target update rate.

2. **Velocity discontinuity after a replan.** This should not happen — if it does, it's a bug in the trajectory profile. The manager tests verify continuity across replans for both `SCurvePosition` and `SinCurvePosition`.

3. **Trajectory position diverging from measured position.** The trajectory is a feedforward reference, not a closed-loop controller. If the mechanism can't follow the trajectory (due to load, friction, or stalling), the trajectory position will run ahead. The mechanism controller is responsible for detecting this divergence and calling `resetFromMeasurement()` — see `ArmController`'s replan check.

### Inspecting the Current Trajectory

The `getCurrentTrajectory()` method exposes the underlying trajectory for inspection:

```java
PositionTrajectory traj = manager.getCurrentTrajectory();
if (traj instanceof SCurvePosition s) {
    System.out.println("vPeak = " + s.vPeak);
    System.out.println("totalTime = " + s.getTotalTime());
}
```

In ControlLab, this is used to build SVG models for exact trajectory visualization.

## 9.9 How Controllers Use Trajectory Managers

The trajectory managers are building blocks for higher-level controllers. Here's how `ArmController` integrates `PositionTrajectoryManager`:

### Initialization

```java
trajectory = new PositionTrajectoryManager(
        PARAMS.maxVelRad, PARAMS.maxAccelRad, PARAMS.maxDecelRad, PARAMS.maxJerkRad,
        Math.toRadians(0.5), telemetry, clock);

// Seed to current position
trajectory.resetFromMeasurement(posRad, velRad);
```

### Setting Targets

```java
public void setTarget(double angleRad, double hubVoltage) {
    targetAngleRad = MathUtil.clamp(angleRad, PARAMS.minAngleRad, PARAMS.maxAngleRad);

    // Compute gravity-aware limits for this move
    double[] limits = computeMoveLimits(fromRad, targetAngleRad, hubVoltage);
    trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
    trajectory.setTarget(targetAngleRad);
}
```

The controller updates the motion limits before each move based on the worst-case gravity torque across the planned sweep. This ensures the trajectory doesn't command accelerations the motor can't achieve (Chapter 10).

### Per-Loop Update

```java
// Sample trajectory
trajectory.update();
double trajPos = trajectory.getPosition();
double trajVel = trajectory.getVelocity();
double trajAccel = trajectory.getAcceleration();

// Replan if tracking error too large
if (Math.abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
    double[] limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage);
    trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
    trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad);
    trajectory.update();
    trajPos = trajectory.getPosition();
    trajVel = trajectory.getVelocity();
    trajAccel = trajectory.getAcceleration();
}

// Feedforward from trajectory
double ffVoltage = feedforward.calculate(predictedPosRad, trajVel, nextVel, dt);

// PD feedback on trajectory tracking error
double posError = trajPos - predictedPosRad;
double velError = trajVel - predictedVelRad;
double pdVoltage = PARAMS.kP * posError + PARAMS.kD * velError;
```

The trajectory manager provides the feedforward reference (what the mechanism should be doing), and the controller adds feedback (correcting for what it's actually doing). This separation is the central architectural pattern of MarsCommonFtc's mechanism controllers.

## 9.10 Summary

The trajectory managers wrap the offline profiles from Chapters 5–7 with the bookkeeping needed for real-time use:

- **`PositionTrajectoryManager`** — manages a `PositionTrajectory` with clock-based sampling, tolerance-gated replanning, state injection, and telemetry
- **`VelocityTrajectoryManager`** — same pattern for `SCurveVelocity`, with asymmetric jerk and opposing-acceleration clamping
- **`TrajectoryFactory`** — functional interface for swapping trajectory implementations (`SCurvePosition::new`, `SinCurvePosition::new`)
- **`SetOnChange<Double>`** — epsilon-based change detection with zero snap, preventing replan floods from noisy targets
- **Mid-motion replanning** — new trajectory seeded from `lastP/lastV/lastA`, guaranteeing p/v/a continuity (but not jerk)
- **`resetFromMeasurement()`** — injects measured state for tracking error recovery and mode transitions
- **`updateConfig()`** — runtime limit updates for gravity-aware and voltage-aware per-move tuning
- **Telemetry** — position, velocity, and acceleration emitted on every `update()` call

Chapter 10 explains how the per-move trajectory limits are computed from motor physics, using the voltage balance equation to ensure the trajectory never commands more acceleration than the motor can deliver.
