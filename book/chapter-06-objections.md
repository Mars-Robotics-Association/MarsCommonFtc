# Chapter 6 Objections

## 6.1 "Infinite jerk means infinite force rate-of-change"

Jerk is the time derivative of acceleration, and by Newton's second law (F = ma for constant mass), the rate of change of force is m * jerk. So infinite jerk does mean infinite force rate-of-change for a rigid body. The sentence is technically correct.

However, the clause that follows -- "which excites mechanical resonances, causes gear backlash, and produces audible noise" -- conflates several distinct phenomena. Gear backlash is a geometric property of the gear mesh; it exists regardless of the jerk profile. What infinite jerk does is cause impulsive loading at the backlash crossings, which is different from "causing" backlash. Consider rewording to something like "which slams through gear backlash, excites mechanical resonances, and produces audible noise."

**Severity:** minor. The intuition is right; the wording is slightly imprecise.

## 6.3 Braking prefix described as 3 sub-phases, but phase B3 never exists

The chapter describes the braking prefix as a "3-sub-phase jerk-limited deceleration" with:

> Phase B1: Apply -jMax to reduce acceleration (if needed)
> Phase B2: Hold constant deceleration
> Phase B3: Apply +jMax to bring acceleration to zero at v=0

In the actual code, `tBrk3 = 0` unconditionally in both the trapezoidal and triangular branches. The braking prefix intentionally does *not* ramp acceleration back to zero -- it hands off a non-zero acceleration to the main 7-phase profile (stored as `aBrakeHandoff`). The chapter's description of phase B3 ("bring acceleration to zero at v=0") is wrong. The braking prefix ends with v=0 but acceleration still nonzero, and that residual acceleration is carried into T1 of the main profile.

**Severity:** significant. A reader implementing this from the chapter's description would produce a different (and less efficient) profile than what the code actually does.

## 6.3 "Helpful Acceleration Preservation" example is misleading

The prose says:

> if the arm is moving upward toward its target and accelerating upward, the existing acceleration is useful and the a0 prefix is skipped.

But the code condition (`v0 * dir < -1e-9 && a0pos > 1e-9`) requires that velocity is *away* from the target (wrong direction) while acceleration is *toward* the target. The described scenario -- "moving toward its target and accelerating toward it" -- would not trigger this code path at all; in that case the a0 prefix would fire normally to null the acceleration before the 7-phase section.

The correct description is: if the velocity is in the wrong direction but the acceleration is already decelerating the mechanism (pointing toward the target), then that acceleration is helpful for braking and should be preserved rather than zeroed.

**Severity:** significant. The prose describes the opposite of what the code does.

## 6.5 SCurveVelocity phase table oversimplified

The 3-phase table says T3 uses `-jDec`. But whether the jerk is positive or negative depends on direction. In the code, the signed jerk for phase 3 is `-dir * jDec`. More importantly, the chapter omits that T1 can also use `jDec` (not `jInc`) when the initial acceleration exceeds the peak -- i.e., when the first phase is *decreasing* acceleration rather than increasing it. The table implies T1 always increases acceleration, which is not the case.

**Severity:** minor. The simplification is understandable for exposition, but the asymmetric jerk discussion later in the same section relies on the reader understanding which jerk applies in which phase, making the omission confusing.

## 6.6 findMaxAMax described as finding "maximum acceleration achievable from rest (v = 0)"

The section header and prose say findMaxAMax finds "the maximum acceleration achievable from rest (v = 0)." The actual method signature is `findMaxAMax(v0, v1, jInc, voltage, kS, kV, kA)` and it takes arbitrary `v0` and `v1` -- it does not assume rest. The binary search simulates the full trajectory from `v0` to `v1` and checks the voltage constraint at every sample point. The "from rest" claim is wrong.

**Severity:** moderate. A reader who trusts the description would not realize the method works for arbitrary initial velocities, and might write unnecessary wrapper code.

## 6.7 kPEffective formula does not match the actual code

The chapter shows:

```java
double kPEffective = kP * Math.max(0, 1.0 - Math.abs(a) / aMax);
```

The actual `VelocityMotorPF` code uses a time-based ramp, not an acceleration-ratio formula:

```java
double kPEffective = (kpRampSec < 1e-9)
        ? (Math.abs(a) < 1e-6 ? _config.kP : 0.0)
        : _config.kP * MathUtil.clamp(accelZeroElapsedSec / kpRampSec, 0.0, 1.0);
```

The real code tracks how long acceleration has been near zero (`accelZeroElapsedSec`) and ramps kP in over a configurable `kpRampMs` window. This is qualitatively different: the chapter's formula would restore kP instantly when acceleration drops to zero, while the real code intentionally delays kP engagement to let the mechanism settle. The chapter's version was the old behavior (before commit f7f6304); the text was not updated.

**Severity:** significant. The code snippet is presented as what the controller does, but it shows stale behavior. A reader tuning their controller from this description would not understand why kP doesn't engage immediately when acceleration reaches zero.

## 6.8 "C2 continuity" claim at replan boundary

The chapter says mid-motion replanning ensures "C2 continuity (position, velocity, and acceleration are continuous; jerk is not)." This is correct as a statement about the trajectory's mathematical properties at the replan point. However, calling it "C2 continuity" is slightly misleading in context: C2 continuity is a property of a single function, not of two independently computed trajectories spliced together. The old and new trajectories share the same p, v, a at the splice point by construction, but that is not the same as saying the composite function is C2 in the classical sense. In particular, the left-sided jerk (from the old trajectory) and the right-sided jerk (from the new trajectory) are generally different, so the composite is C2 but not C3. The statement is correct but could confuse a reader who later learns the formal definition.

**Severity:** nitpick. Technically defensible but worth a clarifying parenthetical.

## 6.2 Phase duration formulas imply symmetric jerk within accel/decel halves

Section 6.2 states:

> T1 = T3 = aMaxAccel / jMax

This equality T1 = T3 holds for the standard case (aStart = 0), but in the code the actual T1 and T3 can differ when there is a nonzero handoff acceleration from braking (`aStart != 0`). In that case T1 = (aMaxAccel - aStart) / jMax while T3 = aMaxAccel / jMax. The formulas as written are only correct for the zero-initial-acceleration case, which the chapter should note since section 6.3 (non-zero initial conditions) immediately follows.

**Severity:** minor. The formulas are correct for the nominal case but incomplete given the chapter's own discussion of non-zero initial conditions.
