# Chapter 3 Objections

## 3.3 — "No amount of tuning can make a first-order system respond faster than its time constant allows"

This is misleading. The time constant tau = kA/kV describes the open-loop response to a *constant* voltage step. A controller with voltage headroom can absolutely drive the system faster than the open-loop time constant by applying more voltage during the transient and backing off as the target is approached. The time constant is a property of the open-loop plant, not a hard speed limit on the closed-loop system. A student reading this will incorrectly conclude that a 1.26-second time constant means the flywheel physically cannot reach 63% faster than 1.26 seconds, which is false — a bang-bang controller would get there faster.

## 3.4 — Code listing for `calculateWithVelocities` disagrees with actual source in several places

Three discrepancies between the code shown in the chapter and `SimpleMotorFeedforward.java`:

1. **kA threshold**: The chapter shows `kA < 1e-4`; the actual code uses `kA < 1e-9`.
2. **signum argument**: The chapter shows `Math.signum(nextVelocity)` in the small-kA fallback; the actual code uses `Math.signum(currentVelocity)` in the discrete path and `Math.signum(nextVelocity)` only in the near-zero-kA path (which is correct — the actual code matches WPILib semantics).
3. **kS term missing from discrete path**: The chapter's discrete path computes `(nextVelocity - Ad * currentVelocity) / Bd` and clamps it, omitting the `kS * Math.signum(currentVelocity)` term. The actual code adds that term: `ks * Math.signum(currentVelocity) + 1.0 / B_d * (nextVelocity - A_d * currentVelocity)`.
4. **Clamp**: The chapter shows `MathUtil.clamp(u, -12.0, 12.0)`. The actual code does not clamp.

If the intent is to show simplified pseudocode, say so. Otherwise the listing will confuse anyone who reads the actual source alongside the chapter.

## 3.5 — `maxAchievableVelocity` API shown with wrong number of arguments

The chapter shows:

```java
double maxVel = ff.maxAchievableVelocity(batteryVoltage, angle);
```

The actual `ArmFeedforward.maxAchievableVelocity` takes three arguments: `(maxVoltage, angle, acceleration)`. Omitting the acceleration parameter will not compile.

## 3.8 — `VelocityMotorPF` kPEffective formula does not match the actual code

The chapter shows:

```java
double kPEffective = kP * Math.max(0, 1.0 - Math.abs(acceleration) / accelMax);
```

The actual code uses a time-since-acceleration-ended ramp:

```java
double kPEffective = (kpRampSec < 1e-9)
        ? (Math.abs(a) < 1e-6 ? _config.kP : 0.0)
        : _config.kP * MathUtil.clamp(accelZeroElapsedSec / kpRampSec, 0.0, 1.0);
```

These are meaningfully different strategies. The chapter's version scales kP by how close acceleration is to its maximum; the real code holds kP at zero during any nonzero acceleration and then ramps it up over a configurable time window after acceleration reaches zero. The narrative description ("at peak acceleration, kPEffective is zero; as acceleration drops to zero, kPEffective approaches kP") matches the chapter's formula but not the actual code, so a student trying to reconcile the two will be confused.

## 3.9 — Log-linearization derivation silently assumes v_0 = 0

The general velocity formula is shown with v_0:

$$v(t) = v_{ss} - (v_{ss} - v_0) \cdot e^{-t/\tau}$$

But the log derivation that follows only works when v_0 = 0:

$$\ln\left(1 - \frac{v(t)}{v_{ss}}\right) = -\frac{t}{\tau}$$

With nonzero v_0 the left-hand side becomes $\ln\left(\frac{v_{ss} - v(t)}{v_{ss} - v_0}\right)$. Since the kA test section says "apply a step power command" (implying from rest), this is probably fine in practice, but the math as written is inconsistent with the formula two lines above it.

## 3.10 — "1120 CPR at the output shaft" is unexplained and potentially wrong

The chapter states "For a goBILDA 5000-series motor with 1120 CPR at the output shaft" but then the conversion formulas use 28 (the motor-shaft encoder resolution), not 1120. The codebase consistently uses `ticksPerRev = 28` with a separate gear ratio parameter. The 1120 figure (28 * 40) would be the effective output-shaft resolution for a specific gear ratio, but no gear ratio is mentioned in the text, and the conversion formula in the same section does not use 1120 anywhere. This will confuse readers trying to reproduce the math.

## 3.10 — kA units written incorrectly

The chapter writes kA units as `V/TPS^2`. Dimensionally, kA has units of voltage per (velocity/time) = voltage * time / velocity. In TPS units that is `V·s/TPS` or equivalently `V/(TPS/s)`. Writing `V/TPS^2` implies volts per ticks-squared-per-second-squared, which is dimensionally wrong and will trip up any student who tries to do a unit check.

## 3.2 — kS described as "the y-intercept of a voltage-vs-velocity linear regression" and also as the voltage at zero velocity

Calling kS "the voltage the line predicts at zero velocity" is correct for the regression intercept, but it risks confusion with the physical meaning. Coulomb friction is the minimum voltage needed to sustain motion once already moving — it is emphatically *not* the voltage needed to start from rest (static friction / stiction is higher). The chapter never makes this distinction. A student could read this and think applying kS volts to a stationary motor will start it moving, which is generally false.
