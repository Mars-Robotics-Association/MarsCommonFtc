# Chapter 10 Objections

## 10.1 — kS described as "back-EMF" consumer

The voltage budget diagram and item 2 label `kS` as "static friction" but the broader narrative repeatedly calls back-EMF the thing that limits acceleration. That is fine. However, the sentence "Back-EMF ($k_V \cdot v$) — voltage proportional to speed, **opposing the supply**" is misleading. Back-EMF opposes the *change in current* (Lenz's law), which reduces the net voltage available to drive current through the winding resistance. Saying it "opposes the supply" makes it sound like the motor is pushing voltage backwards into the battery. A student who later studies Kirchhoff's voltage law around the motor loop will be confused. Suggest: "voltage proportional to speed that **subtracts from the supply in the winding loop**" or similar.

## 10.1 — Free-speed formula ignores sign of kS

The free-speed expression $v_{\text{free}} = (V - k_S) / k_V$ is correct for positive velocity (where $\text{sign}(v) = +1$). But the preceding `maxAchievableAcceleration` code uses `ks * Math.signum(velocity)`, which means at negative velocities the kS term flips sign and the free speed would be $(V + k_S) / k_V$. The chapter doesn't mention this asymmetry. For a student who tries the formula for a move in the negative direction, they will get the wrong answer.

## 10.3 — findMaxAMax lower bound is wrong in the chapter's description

The chapter says: "the lower bound starts at `motorAMaxAtZero = (voltage - kS) / kA`, which is the maximum acceleration the motor can produce at zero velocity. This is a theoretical maximum — the actual achievable acceleration is lower because the motor isn't at zero velocity for the entire move."

This is self-contradictory. If `motorAMaxAtZero` is a theoretical *maximum* and the actual result is *lower*, then it should be the *upper* bound of the binary search, not the lower bound. The code sets `low = motorAMaxAtZero` and `high = 5000`, so the search can only return values *at or above* `motorAMaxAtZero`. It can never return a value below the theoretical maximum at zero velocity. The prose says the search finds a value lower than this bound, but the code cannot do that.

In practice the search probably always returns `motorAMaxAtZero` (since any higher `aMax` is infeasible), making the binary search a no-op. The chapter should either explain why `motorAMaxAtZero` is actually a correct *lower* bound (i.e., explain that the profile's acceleration is at its peak near v=0 and ramps down by the time velocity is high, so this value is usually feasible), or acknowledge that the search range is questionable and the method effectively just returns the zero-velocity maximum.

## 10.3 — "the voltage the motor would need" formula drops signum

The voltage-needed formula in the binary search explanation is written as:

$$V_{\text{needed}} = k_S + k_V |v(t)| + k_A \cdot a(t)$$

The actual feedforward equation uses $k_S \cdot \text{sign}(v)$, not just $k_S$. The code in `findMaxJDec` also uses bare `kS` (no signum). For trajectories that only move in the positive direction this is equivalent, but the chapter presents this as the general voltage balance, which it is not. This should at least note the assumption that velocity is positive throughout the move.

## 10.4 — Incorrect justification for sampling sufficiency

The chapter claims: "if the trajectory passes at 10,000 samples, it won't violate the voltage constraint at any point between samples (since the violation surface is linear in velocity and the trajectory's acceleration is piecewise-linear in time)."

The acceleration is piecewise-linear in time (correct — constant jerk in each phase). But the velocity is piecewise-*quadratic* in time (integral of piecewise-linear acceleration). The constraint boundary $a_{\max}(v) = (V - k_S - k_V |v|) / k_A$ is linear in $v$, but since $v(t)$ is quadratic in $t$, the constraint boundary expressed as a function of time is *not* linear. The difference $a(t) - a_{\max}(v(t))$ is a quadratic-minus-linear = quadratic function of time within each phase, so its maximum can fall between sample points.

In practice 10,000 samples over a sub-second trajectory is dense enough that missing a violation is essentially impossible, but the theoretical justification given is wrong. Either drop the parenthetical or fix it to say something like "since 10,000 samples over a trajectory of this duration is far denser than any feature of the constraint surface."

## 10.5 — worstCaseAngle only checks 0 and -pi crossings

The code (and the chapter's listing) checks horizontal crossings at $\theta = 0$ and $\theta = -\pi$, but not $\theta = +\pi$ or any other multiples of $\pi$. For an arm that sweeps through angles above $+\pi$ (or below $-\pi$ for that matter), the code would miss a worst-case horizontal crossing. This may be intentional if the arm's range of motion is constrained, but the chapter doesn't mention this assumption. A student building a continuously-rotating arm or an arm with a different zero convention could be bitten by this.

## 10.5 — "split at the midpoint" is not conservative in general

The chapter says the sweep is split at the midpoint and claims "by computing limits at the worst-case angle in each half, the trajectory is guaranteed to be achievable everywhere." This guarantee only holds if the actual transition from acceleration to deceleration happens at or after the midpoint angle. For short moves or moves with high jerk, the S-curve profile may transition to deceleration well before the angular midpoint. In that case, the "worst accel angle" is computed over a range that extends past where the profile is actually accelerating, and the "worst decel angle" misses the early part of deceleration. The result is still conservative (the worst-case angle over a wider range is at least as bad as the worst-case over the correct range), but the *assignment* of which limit applies to which phase could be wrong — the accel limit could be overly pessimistic while the decel limit is insufficiently pessimistic if the worst-case angle for decel falls in the first half of the sweep. The chapter should note this is a simplification that errs on the side of caution overall but doesn't perfectly partition the constraint.

## 10.7 — "jerk = 10x acceleration" rule of thumb is unjustified

Step 3 recommends `maxJerkRad = maxAccelRad * 10` with the aside "jerk = 10x acceleration is a reasonable start." No physical or empirical reasoning is given. For a student, this looks like a magic number. Jerk has units of rad/s^3 while acceleration has units of rad/s^2, so the ratio has units of 1/s — it corresponds to "the acceleration ramp takes about 0.1 seconds." That framing (time to ramp from zero to max acceleration) would be much more useful to a student tuning their robot, and would let them reason about whether 0.1s is appropriate for their mechanism.
