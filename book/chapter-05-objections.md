# Chapter 5 Objections

## 5.1 "acceleration and jerk are bounded"

The bullet list in Section 5.1 claims a motion profile provides "Reduced mechanical stress -- acceleration and jerk are bounded." A trapezoidal profile bounds acceleration but has *infinite* jerk at phase transitions. The chapter itself acknowledges this in Section 5.8 ("The trapezoidal profile has infinite jerk at phase transitions"). The bullet should say acceleration is bounded, not jerk. Jerk bounding is the job of the S-curve profile introduced in Chapter 6.

## 5.1 "No overshoot"

The bullet claims "No overshoot -- the setpoint decelerates to a stop at the target." This is only true for the *setpoint trajectory*. Whether the physical mechanism overshoots depends on how well the feedback controller tracks the profiled setpoint. A poorly tuned PID with a profiled setpoint can still overshoot. The claim should be qualified: the *profile* does not overshoot; the mechanism won't overshoot *if the controller tracks the profile well*.

## 5.2 "The duration equals the acceleration duration"

Phase 3 description says "The duration equals the acceleration duration." This is only true when the initial and final velocities are equal (typically both zero). If the initial velocity is nonzero and the final velocity is zero, the acceleration and deceleration durations differ. The source code computes `m_endAccel = accelerationTime - cutoffBegin` and decel duration as `accelerationTime - cutoffEnd`, which are different when cutoffBegin != cutoffEnd. The text should say the durations are equal only in the symmetric (start and end at rest) case.

## 5.2 Triangular peak velocity formula

The formula for peak velocity in the triangular case is given as:

$$v_{\text{peak}} = \sqrt{d \cdot a_{\max}}$$

This is wrong. For a triangular profile starting and ending at rest, the mechanism accelerates for half the distance and decelerates for the other half. From $d/2 = v^2/(2a)$, the correct formula is:

$$v_{\text{peak}} = \sqrt{d \cdot a_{\max}}$$

Wait -- that actually checks out: $d/2 = v^2/(2a)$ gives $v = \sqrt{d \cdot a}$. However, `d` here is the `fullTrapezoidDist`, not the raw distance. The source code computes `fullTrapezoidDist = cutoffDistBegin + (goal.position - m_current.position) + cutoffDistEnd`, which includes corrections for nonzero initial/final velocities. The text presents the formula as if `d` is the raw distance to travel, which is only correct when starting and ending at rest. Should clarify that `d` is the equivalent full-trapezoid distance, not the raw displacement.

## 5.2 Velocity clamping description is misleading

Section 5.2 says "if the current velocity exceeds maxVelocity, it is clamped to maxVelocity and the profile begins decelerating immediately." Looking at the source code, the velocity is clamped via `Math.copySign(maxVelocity, m_current.velocity)`, which preserves the sign. But the profile does not "begin decelerating immediately" -- it begins the normal profile calculation from the clamped velocity. Whether it decelerates depends on the goal. The phrasing implies the profile always decelerates after clamping, which is not necessarily the case.

## 5.3 Open-loop usage example is dubious

The direct-usage example commands the motor with `motor.setPower(state.velocity / maxVelocity)`, which is pure open-loop velocity control. This is a questionable example to show students because it will only work if the motor's velocity-to-power relationship is perfectly linear and there are no disturbances. The text even says "In practice, you rarely use TrapezoidProfile directly," but showing a bad open-loop example first may leave the wrong impression. Consider either labeling it as "open-loop (no feedback)" more explicitly, or replacing it with a feedforward + feedback example.

## 5.4 "PID tracks the profiled setpoint position"

The inline comment says the PID controller tracks the profiled setpoint's *position* only. This is accurate to the source code (`m_controller.calculate(measurement, m_setpoint.position)`), but worth noting that the profiled setpoint also has a velocity component that is completely ignored. A more complete controller would use the setpoint velocity as a feedforward term. The text doesn't mention this missed opportunity, which may mislead students into thinking ProfiledPIDController is a complete solution when it is leaving performance on the table.

## 5.5 Steady-state velocity formula includes kS out of nowhere

The steady-state velocity equation introduces kS without explanation:

$$v_{ss} = \frac{u_{\max} - k_S}{k_V}$$

The ExponentialProfile source code computes `maxVelocity()` as `-maxInput * B / A`, which with `B = 1/kA` and `A = -kV/kA` gives `maxInput / kV`. There is no kS term anywhere in ExponentialProfile. The formula with kS may be the "real physics" steady-state velocity of a motor with static friction, but the ExponentialProfile does not model static friction. Presenting these as equal is misleading -- the profile's steady-state velocity is `maxInput / kV`, not `(maxInput - kS) / kV`.

## 5.7 timeLeftUntil API mismatch

Section 5.7 shows `profile.timeLeftUntil(targetPosition)` as if it takes a single position argument. Looking at the source code:
- `TrapezoidProfile.timeLeftUntil(double target)` takes a single double -- this matches.
- `ExponentialProfile.timeLeftUntil(State current, State goal)` takes two State objects -- this does NOT match.

The text presents a unified API but the two profile types have different signatures for this method. Since the section covers both profiles, this is misleading.

## 5.6 "you cannot cap acceleration without modifying the input voltage"

This is true but undersells the situation. You *can* reduce `maxInput` below the actual battery voltage to limit acceleration in the exponential profile. The text makes it sound like there is no way to limit acceleration at all, when in fact reducing maxInput is a straightforward (if coarse) mechanism. Worth mentioning.

## 5.5 "first-order system dynamics" terminology

The chapter repeatedly calls the motor model $dv/dt = Av + Bu$ a "first-order system." This is correct in the state-space sense (one state variable, velocity), but the full motor model with position as a second state variable is second-order. The ExponentialProfile only profiles the velocity state, treating position as the integral of velocity. This is fine but could confuse students who later encounter the full second-order motor model. A brief note clarifying that "first-order" refers to the velocity dynamics specifically would help.
