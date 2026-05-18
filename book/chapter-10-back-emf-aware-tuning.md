# Chapter 10: Back-EMF-Aware Profile Tuning

Chapters 5–7 built trajectory profiles with user-specified limits: maximum velocity, acceleration, and jerk. But where do those limits come from? A common approach is to set them by feel — start with conservative values, increase until the mechanism shakes or misses targets, then back off. This works, but it leaves performance on the table and gives no principled way to adapt when the battery drains or the load changes.

The real limit on how fast a DC motor can accelerate is **back-EMF**. As the motor spins faster, it generates a voltage that opposes the supply. The voltage available for producing torque (and therefore acceleration) shrinks as speed increases. At some speed, all the supply voltage is consumed by back-EMF plus friction, and no voltage is left for acceleration — the motor has reached its top speed.

This chapter derives the voltage-constrained acceleration limit from the motor's feedforward parameters, shows how `SCurveVelocity.findMaxJDec()` and `findMaxAMax()` use binary search to find the tightest limits that don't violate the voltage constraint, and explains how `ArmController` computes per-move gravity-aware trajectory limits.

## 10.1 The Voltage Budget

Chapter 3 introduced the voltage balance equation for a DC motor:

$$V = k_S \, \text{sign}(v) + k_V \, v + k_A \, a$$

This says: the voltage applied to the motor is consumed by three things:

1. **Static friction** ($k_S$) — a fixed voltage to overcome friction, in the direction of motion
2. **Back-EMF** ($k_V \cdot v$) — voltage proportional to speed that subtracts from the supply in the winding loop
3. **Acceleration** ($k_A \cdot a$) — voltage proportional to acceleration, producing torque

Rearranging for the maximum achievable acceleration:

$$a_{\max}(v) = \frac{V_{\text{supply}} - k_S - k_V |v|}{k_A}$$

This is the `maxAchievableAcceleration` method from `SimpleMotorFeedforward`:

```java
public double maxAchievableAcceleration(double maxVoltage, double velocity) {
    return (maxVoltage - ks * Math.signum(velocity) - velocity * kv) / ka;
}
```

The key insight: **the maximum achievable acceleration decreases linearly with speed.** At zero speed, the motor can produce maximum torque. At the motor's free speed ($v_{\text{free}} = (V - k_S) / k_V$ for positive velocity, or $(V + k_S) / k_V$ for negative velocity — the $k_S$ sign flips with direction), there is no voltage left for acceleration — the motor can maintain speed but not increase it.

### The Voltage Budget Diagram

Think of the supply voltage as a fixed budget that must be divided among three consumers:

```
|←————————————— V_supply ——————————————→|
|  kS  |    kV · |v|     |   kA · a    |
|      |   (back-EMF)    | (available   |
|      |                 |  for torque) |
```

As velocity increases, the back-EMF slice grows, squeezing the acceleration slice. At $v_{\text{free}}$, the back-EMF slice fills the entire budget minus friction, leaving nothing for acceleration.

### For Arms: Gravity Consumes Voltage Too

For arm mechanisms, the voltage equation includes a gravity term:

$$V = k_S \, \text{sign}(v) + k_G \cos(\theta) + k_V \, v + k_A \, a$$

The gravity voltage $k_G \cos(\theta)$ depends on the arm angle $\theta$ measured from horizontal. At horizontal ($\theta = 0$), gravity consumes maximum voltage. Pointing straight up or down ($\theta = \pm\pi/2$), gravity consumes none.

`ArmFeedforward.maxAchievableAcceleration()` includes this term:

```java
public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
    return (maxVoltage - ks * Math.signum(velocity) - Math.cos(angle) * kg - velocity * kv) / ka;
}
```

Similarly, `maxAchievableVelocity()` gives the maximum speed at a given angle and acceleration:

```java
public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
    return (maxVoltage - ks - Math.cos(angle) * kg - acceleration * ka) / kv;
}
```

## 10.2 Why Trajectory Limits Must Respect Back-EMF

When a trajectory commands acceleration $a$ at velocity $v$, the motor needs voltage $k_S + k_V |v| + k_A |a|$. If this exceeds the supply voltage, the motor can't follow the trajectory. The controller will command full power but the mechanism will lag behind. The trajectory position runs ahead of the actual position, the tracking error grows, and the controller eventually replans — wasting time and producing rough motion.

Worse, the tracking error may trigger safety logic. `ArmController` replans when the tracking error exceeds 15 degrees. If the trajectory consistently commands infeasible accelerations, the arm enters a replan cycle: plan, fall behind, replan, fall behind, replan. The motion is jerky and slow — the opposite of what the trajectory was designed to produce.

The solution: **compute trajectory limits from the motor's voltage budget**, not from arbitrary constants. If the motor can only produce 3 rad/s² of acceleration at the expected velocity, set `aMaxAccel = 3`, not 10. The trajectory will be slower but *achievable* — the mechanism can actually follow it.

## 10.3 Binary Search for Voltage-Safe Limits

`SCurveVelocity` provides two static methods that find the maximum jerk or acceleration that a trajectory can use without violating the motor's voltage constraint. Both use binary search: they construct a candidate trajectory, sample it at many points, check if any sample violates the voltage constraint, and bisect.

### findMaxJDec: Maximum Decreasing-Jerk Limit

`findMaxJDec()` finds the largest `jDec` (jerk when ramping acceleration down) that keeps the trajectory within the motor's voltage budget:

```java
public static double findMaxJDec(
        double v0, double v1, double a0, double aMax, double jInc,
        double voltage, double kS, double kV, double kA) {
    return findMaxJDec(v0, v1, a0, aMax, jInc, voltage, kS, kV, kA, 30, 1000);
}
```

The extended overload takes iteration count and sample count:

```java
public static double findMaxJDec(
        double v0, double v1, double a0, double aMax, double jInc,
        double voltage, double kS, double kV, double kA,
        int iterations, int samples) {
    if (kA <= 0) return Double.POSITIVE_INFINITY;
    if (voltage <= kS) return 0.0;
    if (aMax <= 0 || !Double.isFinite(aMax)) return 0.0;
    if (jInc <= 0) return 0.0;
    if (Math.abs(v1 - v0) < 1e-9) return Double.POSITIVE_INFINITY;

    double low = 1;
    double high = 5000;
    double best = low;

    for (int iter = 0; iter < iterations; iter++) {
        double mid = (low + high) / 2;

        SCurveVelocity traj = new SCurveVelocity(v0, v1, a0, aMax, jInc, mid);
        double totalTime = traj.getTotalTime();
        if (!Double.isFinite(totalTime)) {
            high = mid;
            continue;
        }

        boolean violates = false;
        for (int i = 0; i <= samples; i++) {
            double t = totalTime * i / samples;
            double v = traj.getVelocity(t);
            double a = traj.getAcceleration(t);

            double availableVoltage = voltage - kS - kV * Math.abs(v);
            if (availableVoltage <= 0) {
                violates = true;
                break;
            }
            double motorAMax = availableVoltage / kA;
            if (a > motorAMax) {
                violates = true;
                break;
            }
        }

        if (violates) {
            high = mid;
        } else {
            best = mid;
            low = mid;
        }
    }

    return best;
}
```

The algorithm:

1. **Degenerate input guards.** Before searching, several conditions are checked that would either produce meaningless results or cause NaN/Inf to propagate into the trajectory:
   - `kA <= 0` — no acceleration constant means no voltage limit on acceleration → `POSITIVE_INFINITY`
   - `voltage <= kS` — motor can't overcome static friction at any velocity; no `jDec` value is safe → `0.0`
   - `aMax <= 0` or not finite — a non-positive or infinite `aMax` produces a degenerate trajectory (infinite rise time or NaN timing); typically caused by a bad `findMaxAMax` result → `0.0`
   - `jInc <= 0` — a zero or negative `jInc` produces a near-infinite acceleration rise time, causing the 1000 samples to cover only the near-zero-velocity start and miss violations → `0.0`
   - `v0 ≈ v1` — trivial trajectory with zero total time; `jDec` has no effect → `POSITIVE_INFINITY`

2. **Search range.** `jDec` is searched between 1 and 5000 (rad/s³ or equivalent units). The lower bound avoids degenerate zero-jerk trajectories; the upper bound covers any practical FTC mechanism.

3. **Construct candidate.** For each midpoint, construct an `SCurveVelocity` with that `jDec` value. All other parameters (`v0`, `v1`, `a0`, `aMax`, `jInc`) are fixed. If the resulting `totalTime` is not finite (which can happen if the parameters are near-degenerate despite passing the guards), the candidate is treated as violating.

4. **Sample and check.** Sample the trajectory at 1000 evenly-spaced points. At each point, compute the voltage the motor would need:

   $$V_{\text{needed}} = k_S + k_V |v(t)| + k_A \cdot a(t)$$

   (This assumes positive velocity throughout the move; for negative velocity, $k_S$ flips sign.) Compare against the supply voltage. If $V_{\text{needed}} > V_{\text{supply}}$ at any sample, the trajectory violates the constraint.

5. **Bisect.** If the candidate violates, search lower. If it's safe, search higher.

6. **Return the best.** After 30 iterations, the search has converged to within $5000 / 2^{30} \approx 5 \times 10^{-6}$ of the true maximum.

### Why jDec and Not jMax?

The asymmetric jerk model uses `jInc` (jerk when acceleration is increasing) and `jDec` (jerk when acceleration is decreasing). Back-EMF violations typically occur near the *end* of acceleration, where velocity is high and back-EMF is large. At that point, acceleration is ramping *down* — controlled by `jDec`. By searching for the maximum safe `jDec`, the trajectory can ramp up quickly (using the full `jInc`) but slow down gently enough to stay within the voltage budget.

### findMaxAMax: Maximum Acceleration Limit

`findMaxAMax()` searches for the largest `aMax` that stays within the voltage constraint:

```java
public static double findMaxAMax(
        double v0, double v1, double jInc,
        double voltage, double kS, double kV, double kA) {
    if (kA <= 0) return Double.POSITIVE_INFINITY;
    if (voltage <= kS) return 0.0;
    if (jInc <= 0) return 0.0;

    double motorAMaxAtZero = (voltage - kS) / kA;
    double low = motorAMaxAtZero;
    double high = 5000;
    double best = low;

    for (int iter = 0; iter < 30; iter++) {
        double mid = (low + high) / 2;

        SCurveVelocity traj = new SCurveVelocity(v0, v1, 0, mid, jInc, jInc);
        // ... same sampling and violation check ...
    }

    return best;
}
```

The key difference from `findMaxJDec`: the search starts at `motorAMaxAtZero = (voltage - kS) / kA`, which is the maximum acceleration the motor can produce at zero velocity. Since the profile's peak acceleration occurs near zero velocity (where the most voltage headroom exists), this value is typically feasible — the acceleration ramps down by the time velocity is high. The binary search finds the highest `aMax` that works across the entire trajectory without violating the voltage constraint at any point.

`findMaxAMax` uses symmetric jerk (`jInc = jDec`) and zero initial acceleration (`a0 = 0`), since it's computing a limit for the profile's peak acceleration — not tuning the jerk shape.

The degenerate input guards work the same way as `findMaxJDec`, with two cases specific to this function:
- `voltage <= kS` — if the supply voltage can't overcome static friction, `motorAMaxAtZero` would be negative or zero, and every binary-search candidate would violate. Without the guard, `best` is initialized to that negative value and returned unchanged since no candidate ever passes. The guard returns `0.0` instead.
- `jInc <= 0` — the search trajectories would have near-infinite rise times, causing samples to cluster at near-zero velocity and miss violations at higher speeds.

## 10.4 The Voltage Constraint Surface

The relationship between velocity and achievable acceleration is linear:

$$a_{\max}(v) = \frac{V - k_S - k_V |v|}{k_A}$$

This defines a **constraint surface** in $(v, a)$ space. Any point below the line is achievable; any point above it exceeds the motor's voltage capacity.

A well-tuned S-curve trajectory stays below this line at every point. The trajectory's acceleration trace sweeps through $(v, a)$ space as it progresses from rest to cruise to rest. If the trace touches or crosses the constraint line, the motor can't deliver the commanded acceleration.

The worst-case point is usually at the transition from acceleration to cruise — when velocity is near `vPeak` and acceleration is still ramping down. This is exactly where `jDec` matters: a gentler ramp-down (lower `jDec`) means the acceleration drops earlier, keeping the trajectory below the constraint line even as velocity climbs.

### Validation in Tests

`SCurveVelocityTest` includes a back-EMF violation check that samples a trajectory at 10,000 points and verifies every sample is within the voltage budget:

```java
for (int i = 0; i <= samples; i++) {
    double t = totalTime * i / samples;
    double v = trajectory.getVelocity(t);
    double a = trajectory.getAcceleration(t);

    double availableVoltage = MOTOR_VOLTAGE - KS - KV * Math.abs(v);
    double motorAMax = availableVoltage / KA;

    assertTrue(a <= motorAMax + tolerance,
            String.format("Back-EMF violation at t=%.4f: a=%.2f > motorAMax=%.2f", t, a, motorAMax));
}
```

This is the definitive test: 10,000 samples over a sub-second trajectory is far denser than any feature of the constraint surface, so missing a violation between samples is effectively impossible.

## 10.5 Per-Move Gravity-Aware Limits

For arm mechanisms, the voltage budget changes with arm angle because gravity consumes a varying amount of voltage. `ArmController.computeMoveLimits()` computes trajectory limits that account for this variation across the planned sweep.

### The Sweep Model

A move from angle `fromRad` to angle `toRad` is split at the midpoint:

```java
double midRad = (fromRad + toRad) / 2.0;
```

The first half constrains acceleration (the arm is speeding up). The second half constrains deceleration (the arm is slowing down). This is a simplification — the actual accel-to-decel transition depends on the profile's phase structure, not the angular midpoint. The approach errs on the side of caution: computing the worst-case angle over a wider range than strictly necessary produces limits that are at least as conservative as the true constraint.

### Worst-Case Angle

Within each half of the sweep, the worst-case angle is the one where $|\cos(\theta)|$ is maximized — where gravity torque is greatest:

```java
static double worstCaseAngle(double a, double b) {
    double lo = Math.min(a, b);
    double hi = Math.max(a, b);

    double bestAngle = a;
    double bestAbsCos = Math.abs(Math.cos(a));

    double absCosB = Math.abs(Math.cos(b));
    if (absCosB > bestAbsCos) {
        bestAngle = b;
        bestAbsCos = absCosB;
    }

    // Check horizontal crossings: 0 and -π
    // Note: only checks these two crossings. Arms that sweep through +π
    // or other multiples of π would need additional crossing checks.
    if (lo <= 0 && hi >= 0) {
        bestAngle = 0;
        bestAbsCos = 1.0;
    }
    if (lo <= -Math.PI && hi >= -Math.PI) {
        if (1.0 > bestAbsCos) {
            bestAngle = -Math.PI;
        }
    }

    return bestAngle;
}
```

The candidates are:
- The sweep endpoints
- The horizontal angles 0 and $-\pi$ (if they fall within the sweep)

At horizontal, $|\cos(\theta)| = 1.0$, which is the global maximum — the motor fights maximum gravity. If the sweep crosses horizontal, that's the worst case regardless of the endpoints.

### Computing Limits

```java
double[] computeMoveLimits(double fromRad, double toRad, double hubVoltage) {
    double midRad = (fromRad + toRad) / 2.0;

    double worstAccelAngle = worstCaseAngle(fromRad, midRad);
    double worstDecelAngle = worstCaseAngle(midRad, toRad);
    double worstVelAngle   = worstCaseAngle(fromRad, toRad);

    double accel = feedforward.maxAchievableAcceleration(
            hubVoltage, worstAccelAngle, PARAMS.maxVelRad);
    double decel = feedforward.maxAchievableAcceleration(
            hubVoltage, worstDecelAngle, PARAMS.maxVelRad);
    double vel   = feedforward.maxAchievableVelocity(
            hubVoltage, worstVelAngle, 0);

    vel   = Math.min(Math.max(vel,   0), PARAMS.maxVelRad);
    accel = Math.min(Math.max(accel, 0), PARAMS.maxAccelRad);
    decel = Math.min(Math.max(decel, 0), PARAMS.maxDecelRad);

    return new double[] { vel, accel, decel };
}
```

Three limits are computed independently:

1. **Max velocity.** Queried at the worst-case angle over the full sweep, with zero acceleration. This gives the velocity ceiling where back-EMF plus gravity equals the supply voltage.

2. **Max acceleration.** Queried at the worst-case angle in the first half of the sweep, at `maxVelRad`. Using the max velocity is conservative: acceleration is hardest when velocity is high (back-EMF is large) and gravity is worst.

3. **Max deceleration.** Same query but using the worst-case angle in the second half of the sweep.

Each result is clamped to $[0, \text{PARAMS.max}]$ — the per-move limit can reduce the global limit but never increase it.

### Why Conservative Is Correct

The per-move limits are deliberately conservative: they use the worst-case angle and the maximum velocity to compute the tightest possible constraint. In practice, the motor will have more voltage headroom during parts of the move (when the angle is more favorable or the velocity is lower). This means the trajectory is slower than theoretically optimal, but it is *always achievable*.

The alternative — computing exact time-varying limits and feeding them into the profile — would require a profile generator that accepts velocity-dependent acceleration limits. This is possible (it's essentially what the online replanning loop does in Chapter 8) but adds significant complexity. The conservative approach is simpler and works well in practice.

### When Limits Are Recomputed

Per-move limits are computed in two places:

**At `setTarget()`.** When a new target is commanded, the limits are computed for the sweep from the current position to the target:

```java
double[] limits = computeMoveLimits(fromRad, targetAngleRad, hubVoltage);
trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
trajectory.setTarget(targetAngleRad);
```

**At replan.** When the tracking error exceeds the threshold, limits are recomputed for the remaining sweep (from the current predicted position to the target):

```java
double[] limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage);
trajectory.updateConfig(limits[0], limits[1], limits[2], PARAMS.maxJerkRad);
trajectory.resetFromMeasurement(predictedPosRad, predictedVelRad);
```

The replan limits may differ from the original because the remaining sweep is shorter and may not cross the worst-case angle.

## 10.6 Battery Voltage Compensation

The voltage budget depends on the supply voltage, which changes as the battery drains. A fully charged FTC battery delivers about 13.8 V; under heavy load it can drop below 12 V. This 15% voltage reduction directly reduces the achievable acceleration.

`ArmController` passes the live hub voltage to `computeMoveLimits()`:

```java
public void setTarget(double angleRad) {
    setTarget(angleRad, motor.getHubVoltage());
}
```

And again at replan:

```java
public void update(double dt, double hubVoltage) {
    // ...
    if (Math.abs(predictedPosRad - trajPos) > PARAMS.replanThresholdRad) {
        double[] limits = computeMoveLimits(predictedPosRad, targetAngleRad, hubVoltage);
        // ...
    }
}
```

The hub voltage is sampled from the motor's `getHubVoltage()` method — typically reading the REV Hub's voltage sensor. By using the live voltage, the per-move limits automatically adjust as the battery drains:

- **Fresh battery (13.8 V):** Higher limits, faster trajectories
- **Drained battery (12 V):** Lower limits, slower but still achievable trajectories
- **Under heavy load (11 V):** Further reduced limits, conservative motion

This is a form of **gain scheduling**: the trajectory parameters adapt to the current operating conditions. The controller doesn't need to be re-tuned for different battery states — the voltage-aware limits handle it automatically.

## 10.7 Tuning Workflow

### Step 1: Characterize the Motor

Determine $k_S$, $k_V$, $k_A$ (and $k_G$ for arms) from a characterization run (Chapter 3). These are the feedforward gains that model the motor's voltage-to-motion relationship.

### Step 2: Compute Theoretical Limits

With the feedforward parameters, compute the theoretical maximums:

```java
// Maximum acceleration at zero velocity
double aMaxTheoretical = (voltage - kS) / kA;

// Maximum velocity at zero acceleration
double vMaxTheoretical = (voltage - kS) / kV;

// For arms: at the worst-case angle (horizontal)
double aMaxArm = (voltage - kS - kG) / kA;
double vMaxArm = (voltage - kS - kG) / kV;
```

These are upper bounds. The actual usable limits are lower because you need margin for:
- Battery sag under load
- Acceleration at non-zero velocity
- Friction variations
- Model inaccuracy

### Step 3: Set Profile Limits

Start with limits at 70–80% of the theoretical maximums:

```java
PARAMS.maxVelRad   = 0.75 * vMaxTheoretical;
PARAMS.maxAccelRad = 0.75 * aMaxTheoretical;
PARAMS.maxDecelRad = 0.75 * aMaxTheoretical;
PARAMS.maxJerkRad  = PARAMS.maxAccelRad * 10;  // ratio of 10 means ~0.1s to ramp from zero to max acceleration
```

### Step 4: Validate with ControlLab

Use the ControlLab trajectory tab to visualize the profile. The back-EMF violation detection feature overlays the motor's voltage constraint on the acceleration plot. If the trajectory's acceleration trace crosses the constraint line at any point, the profile is infeasible — reduce the limits.

### Step 5: Use findMaxJDec / findMaxAMax for Velocity Profiles

For flywheel velocity profiles, use the binary search methods to find exact limits:

```java
double safeJDec = SCurveVelocity.findMaxJDec(
        0,           // v0: starting velocity
        targetRPM,   // v1: target velocity
        0,           // a0: starting acceleration
        aMax,        // aMax: acceleration limit
        jInc,        // jInc: increasing jerk
        12.0,        // voltage: nominal supply
        kS, kV, kA); // motor characterization

double safeAMax = SCurveVelocity.findMaxAMax(
        0,           // v0
        targetRPM,   // v1
        jInc,        // jInc
        12.0,        // voltage
        kS, kV, kA);
```

Then use the results to configure the velocity manager:

```java
velManager.updateConfig(safeAMax, jInc, safeJDec);
```

### Step 6: Enable Per-Move Limits for Arms

For arm mechanisms using `ArmController`, the per-move limits are computed automatically from the feedforward gains and the live battery voltage. The only configuration needed is setting the PARAMS maximums as global caps:

```java
ArmController.PARAMS.maxVelRad   = 4.0;   // absolute ceiling
ArmController.PARAMS.maxAccelRad = 8.0;
ArmController.PARAMS.maxDecelRad = 8.0;
ArmController.PARAMS.maxJerkRad  = 40.0;
```

`computeMoveLimits()` will reduce these per-move based on gravity and voltage. The PARAMS values serve as upper bounds — the per-move limits can only go lower, never higher.

## 10.8 Summary

Back-EMF limits the achievable acceleration of a DC motor. Trajectory profiles that ignore this limit command infeasible motion, causing tracking errors, replans, and jerky behavior. MarsCommonFtc provides three levels of voltage-aware tuning:

- **Voltage balance equation** — $a_{\max}(v) = (V - k_S - k_V |v|) / k_A$ gives the speed-dependent acceleration ceiling. For arms, add $k_G \cos(\theta)$ for gravity.

- **Binary search methods** — `SCurveVelocity.findMaxJDec()` and `findMaxAMax()` construct candidate trajectories, sample them at 1000 points, and bisect to find the tightest limit that doesn't violate the voltage constraint. Use these for velocity profiles (flywheels, drivetrains).

- **Per-move gravity-aware limits** — `ArmController.computeMoveLimits()` finds the worst-case gravity angle across the planned sweep, queries `ArmFeedforward.maxAchievableAcceleration()` at that angle and the maximum velocity, and caps the trajectory limits accordingly. Limits are recomputed at each target change and at each tracking-error replan, using the live battery voltage.

The result: trajectories that are as fast as the motor can physically deliver, with automatic adaptation to battery state, arm angle, and load. The mechanism follows the trajectory smoothly because the trajectory never asks for more than the motor can give.

This chapter concludes Part II: Motion Profiling. Part III introduces state-space control — the Kalman filters and LQR controllers that the mechanism controllers use for state estimation and optimal feedback.
