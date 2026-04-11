# Chapter 9 Objections

## 9.4 -- "The new trajectory's a0 prefix starts with jerk = 0" is wrong for SCurvePosition

> "Jerk is not continuous. The new trajectory's a0 prefix starts with jerk = 0
> (by design of the S-curve and sinusoidal prefix phases), but the old
> trajectory's jerk at the replan instant is generally non-zero."

For `SCurvePosition`, the a0 prefix applies constant jerk of magnitude
$j_{\max}$ from $t = 0$ (specifically, `jPrefix = ±dir * jMax`). There is
nothing zero about it. The prefix uses the maximum jerk rate to null out
the inherited acceleration as quickly as possible.

For `SinCurvePosition`, the prefix uses a quarter-cosine shape whose
derivative (jerk) is indeed zero at $t = 0$. So the claim is true for
sinusoidal trajectories but false for the default S-curve trajectories.

Since the default factory is `SCurvePosition::new`, most users will hit the
wrong case first.

Suggested fix: drop the parenthetical about jerk = 0 entirely. The
important point is already stated: jerk is not continuous across replans.
If you want to explain why, say something like "the new trajectory picks its
own jerk profile from the inherited acceleration, with no attempt to match
the old trajectory's jerk."

## 9.4 -- "starting gently from the inherited acceleration" mischaracterizes the S-curve prefix

> "The a0 prefix in `SCurvePosition` and `SinCurvePosition` ramps
> acceleration to zero at $j_{\max}$ or with a sinusoidal shape, starting
> gently from the inherited acceleration."

"Starting gently" is accurate for `SinCurvePosition` (the cosine shape has
zero derivative at $t = 0$, so jerk onset is gradual). It is inaccurate for
`SCurvePosition`, where the prefix applies full $j_{\max}$ immediately at
$t = 0$ -- there is no gentle onset. The prefix is designed to be fast, not
smooth.

Suggested fix: say something like "The a0 prefix in `SCurvePosition` ramps
acceleration to zero at constant $j_{\max}$, while `SinCurvePosition` uses a
quarter-cosine shape that starts and ends with zero jerk."

## 9.4 -- "acceleration nodes" is the wrong word

> "Rapid target changes produce visible acceleration nodes in telemetry"

In vibration and signal analysis, a "node" is a point of zero amplitude (as
opposed to an antinode, which is a peak). What you mean here is probably
"kinks" or "spikes" -- sharp features in the acceleration trace caused by
jerk discontinuities at the replan boundary. Calling them "nodes" will
confuse anyone who later encounters the word in a physics or signal
processing context.

Suggested fix: replace "acceleration nodes" with "acceleration kinks" or
"sharp corners in the acceleration trace."

## 9.4 -- Same-direction replanning description is oversimplified

> "When the target changes but the direction doesn't (e.g., target changes
> from 100 to 50 while moving forward), the new trajectory starts from the
> current velocity and decelerates smoothly. No braking prefix is needed
> because the velocity is already in the correct direction."

"Decelerates smoothly" implies the mechanism always needs to slow down for
a same-direction target change. That is only true when the new target is
closer than the old one. If the new target is farther away (say, 100 to
150), the mechanism may accelerate, cruise, or continue unchanged. The
sentence reads as though same-direction replanning always means
deceleration.

Suggested fix: drop "and decelerates smoothly" or rephrase to something
like "the new trajectory continues from the current velocity toward the new
target -- accelerating, cruising, or decelerating as needed."

## 9.2 -- Narrative says setTarget triggers update(), but that coupling is surprising and under-explained

> "when `setTarget()` is called, the new target is stored as `pendingTarget`
> via `SetOnChange`, and `update()` is triggered."

The text describes the mechanism correctly, but breezes past the fact that
`setTarget()` synchronously calls `update()` via the `SetOnChange` callback.
This means calling `setTarget()` has the side effect of sampling the
trajectory and emitting telemetry. For a reader learning the API, this is
surprising -- most "set" methods are inert. A student who calls `setTarget()`
followed by `update()` in their loop will get two samples per iteration
whenever the target actually changes.

This is not a factual error in the chapter, but it is a place where the
chapter could prevent confusion by calling it out explicitly, e.g. "Note
that `setTarget()` calls `update()` internally when the target changes, so
you do not need to call `update()` again immediately after `setTarget()`."
