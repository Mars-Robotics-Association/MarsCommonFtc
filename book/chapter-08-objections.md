# Chapter 8 Objections

## 8.2 -- "Closed-form solution" overstates what Ruckig does

> "Where `SCurvePosition` uses binary search to find `vPeak`, Ruckig solves
> the time-optimal profile analytically. This is faster and avoids the
> iterative convergence issues that binary search can encounter near edge
> cases."

And in 8.8:

> "**Closed-form solver.** No binary search for `vPeak`; the solution is
> computed directly. This avoids the 64-iteration bisection loop and its
> associated edge cases."

Ruckig is not purely closed-form. It solves quartic and lower-degree
polynomials analytically, but for higher-degree cases it uses a
Newton-with-bisection-fallback root finder (`shrink_interval` in
`roots.hpp`, up to 128 iterations). It also enumerates a large combinatorial
space of profile types (control signs, reached-limit cases) and validates
each candidate. This is fundamentally different from a single closed-form
expression.

The contrast with `SCurvePosition` is also misleading. SCurvePosition's
64-iteration bisection converges reliably because the objective is
monotonic -- "iterative convergence issues near edge cases" is a theoretical
concern that the chapter doesn't substantiate. Meanwhile Ruckig's own
iterative root-finding has the same class of numerical concerns.

The real advantage of Ruckig's solver is that it handles arbitrary initial
and final states (non-zero velocity and acceleration) analytically in many
common cases, whereas SCurvePosition assumes a0 = 0. But "closed-form" is
not an accurate description of the algorithm as a whole.

Suggested fix: say "Ruckig uses algebraic solutions for common profile
cases, avoiding the simple bisection search that `SCurvePosition` uses for
`vPeak`." Drop the claim that the solution is purely analytical or
closed-form, and drop the unsupported dig about "convergence issues."

## 8.8 -- "Ruckig's community edition uses symmetric acceleration limits" is wrong

> "**Asymmetric acceleration.** `SCurvePosition` supports independent
> `aMaxAccel` and `aMaxDecel` limits. Ruckig's community edition uses
> symmetric acceleration limits."

Ruckig's community edition supports asymmetric limits via the optional
`min_velocity` and `min_acceleration` fields on `InputParameter`. When
unset, they default to the negation of `max_velocity` and
`max_acceleration` (i.e., symmetric), but setting them to different
magnitudes gives you asymmetric limits. This is visible in
`input_parameter.hpp` and is exercised in the solver code in
`calculator_target.hpp`.

The actual limitation is in MarsCommonFtc's JNI wrapper (`ruckig_jni.cpp`
and `RuckigInput.java`), which only exposes the `max_*` fields and never
sets `min_velocity` or `min_acceleration`. That is a wrapper limitation,
not a Ruckig limitation.

Suggested fix: change to something like "**Asymmetric acceleration.**
`SCurvePosition` supports independent `aMaxAccel` and `aMaxDecel` limits.
Ruckig supports this via `min_acceleration`, but the MarsCommonFtc JNI
wrapper does not currently expose it." The comparison table row should be
updated similarly.

## 8.1 -- "The result is time-optimal by construction" is ambiguous

> "If the operator changes the target mid-move, the planner doesn't need to
> splice two profiles together -- it simply plans a new trajectory from the
> current state to the new target on the next cycle. The result is
> time-optimal by construction."

Each individual re-plan is time-optimal from the current state to the
current target. But the overall motion (original target, then changed
target) is not globally time-optimal -- the system spent some time heading
toward the wrong target. A reader could easily take "time-optimal by
construction" to mean the entire motion is optimal, which it is not.

Suggested fix: say "Each re-plan is time-optimal from the current state to
the new target" to make the scope explicit.

## 8.5 -- "pure feedforward loop" is a misnomer

> "This is a pure feedforward loop -- the output of each cycle becomes the
> input of the next. No sensor feedback is involved."

The example code feeds the planner's own output back as its next input.
That is not feedforward -- feedforward means the control signal is computed
from a reference without any feedback path. What the example shows is
closer to open-loop simulation (or iterating a discrete map). It would
only become feedforward if the output were sent to actuators while a
separate sensor-based loop handled disturbance rejection.

Calling this "feedforward" will confuse students who later encounter the
term in its standard control-theory meaning (Chapter 3 already defines
feedforward correctly).

Suggested fix: call it an "open-loop" iteration or "pure reference
generation" loop instead of "feedforward."

## 8.5 -- Multi-DOF example: "the slower axis runs at full speed"

> "Each axis still respects its own limits -- the slower axis runs at full
> speed, while the faster axis is slowed down."

This is only true in a narrow sense. The "slower axis" (the one that would
take the longest if planned independently) runs its time-optimal profile,
which may or may not saturate all of its limits. A short-distance move
might never reach max velocity. Saying it "runs at full speed" implies it
is always velocity-limited, which is misleading.

Suggested fix: say "the limiting axis runs its time-optimal profile, while
the other axes are stretched to match its duration."
