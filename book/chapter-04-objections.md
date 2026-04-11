# Chapter 4 Objections

## 4.1 — Code snippet does not match actual source

The `calculate()` code block in 4.1 omits the continuous-input branch and the
IZone guard that appear in the real `PIDController.calculate()`. The actual
method has an `if (m_continuous)` path for computing the error and an
`if (Math.abs(m_error) > m_iZone)` check before accumulating the integral. The
snippet as written implies the integral always accumulates unconditionally, which
contradicts Section 4.4's own discussion of IZone. Either show the full method or
add a comment like `// (simplified — IZone and continuous-input handling omitted)`.

## 4.2 — Proportional band formula conflates voltage and normalized output

The proportional band is introduced as `e_full = 1 / kP`, but the text then
says "an error larger than 3.8 degrees commands more than 12 V of output." That
only works when the PID output is in volts and the saturation limit happens to be
12 V. WpiMath's `PIDController` has no output saturation at all — it will happily
return 150 V if the math says so. The proportional band concept is fine, but the
text should clarify that the saturation limit depends on whatever clamping the
caller applies downstream, not on any limit inside `PIDController` itself.

## 4.3 — "velocity error changing at 10 rad/s per cycle" is a units mismatch

The example says kD = 1.0 V/(rad/s) and "a velocity error changing at 10 rad/s
per cycle" yields 10 V. But the derivative term operates on the rate of change
of error, not the rate of change of velocity error. If this is a position
controller, the error derivative has units of rad/s, and kD has units of
V/(rad/s), so 1.0 * 10 = 10 V is numerically correct. But describing it as
"velocity error changing at 10 rad/s per cycle" confuses the derivative of
position error (which is velocity) with the derivative of velocity error (which
is acceleration). The example should say something like "error decreasing at
10 rad/s" to match the units.

## 4.3 — "infinite derivative spike" is technically imprecise

The text says a step change in setpoint produces "an infinite derivative spike."
In the discrete implementation the spike is finite: it equals
`(new_error - old_error) / period`, which is large but bounded. Calling it
infinite is the continuous-time description; the discrete code shown two
paragraphs earlier produces a finite (but potentially very large) kick. A small
clarification like "effectively infinite in continuous time, and very large in
the discrete implementation" would be more accurate.

## 4.4 — Integral clamping explanation has the sense of the division inverted

The text says:

> The maximum integral contribution is `m_ki * m_maximumIntegral`. Setting
> `m_maximumIntegral = 0.2` ensures the integral term can never produce more than
> 20% of full output.

Looking at the source, `setIntegratorRange(min, max)` sets `m_minimumIntegral`
and `m_maximumIntegral`, and the clamp is:

```java
m_totalError = clamp(m_totalError + ..., m_minimumIntegral / m_ki, m_maximumIntegral / m_ki);
```

So `m_totalError` is clamped to `m_maximumIntegral / m_ki`, and the integral
contribution is `m_ki * m_totalError <= m_ki * (m_maximumIntegral / m_ki) = m_maximumIntegral`.
The maximum integral *contribution* is `m_maximumIntegral` directly, not
`m_ki * m_maximumIntegral`. The text has it backwards. Setting
`m_maximumIntegral = 0.2` caps the integral contribution to 0.2 (in whatever
units the output is in), regardless of kI.

## 4.6 — "setpoint jumps ... the proportional output is kP * 1000 — likely saturated"

The example says the setpoint jumps from 0 to 1000 TPS and the proportional
output is `kP * 1000`. With the FlywheelSimple default kP of 0.010 V/TPS, that
is only 10 V — not saturated. The text is trying to motivate ProfiledPIDController
but the worked numbers don't support "likely saturated." Either pick numbers that
actually saturate or soften the claim.

## 4.8 Reason 3 — Code snippet does not match the actual VelocityMotorPF implementation

The chapter shows:

```java
double kPEffective = kP * Math.max(0, 1.0 - Math.abs(acceleration) / accelMax);
```

The actual `VelocityMotorPF.updateInternal()` uses a time-based ramp after
acceleration reaches zero:

```java
double kPEffective = (kpRampSec < 1e-9)
        ? (Math.abs(a) < 1e-6 ? _config.kP : 0.0)
        : _config.kP * MathUtil.clamp(accelZeroElapsedSec / kpRampSec, 0.0, 1.0);
```

These are materially different strategies. The code in the chapter suppresses kP
proportionally to instantaneous acceleration. The real code suppresses kP
entirely during acceleration and then ramps it in over a configurable time window
after acceleration ends. The chapter should show what the code actually does, or
at minimum note that this is a simplified illustration.

## 4.9 — "The system is approximately first-order" is misleading guidance

The text says PID is the right choice when "the system is approximately
first-order" and gives a flywheel as the example. But a DC motor driving a
flywheel is a second-order system (electrical + mechanical time constants), and
PID works fine on it. The real criterion is not the system order but whether the
dominant dynamics are simple enough that PID's limited structure can handle them.
Saying "first-order" may confuse students who later learn that their flywheel is
actually second-order and wonder if PID was wrong all along.

## 4.10 — "PID treats each state independently, ignoring the coupling between them"

This is stated under "The system is multi-state," but PID does not inherently
treat states independently. A single-loop PID on position implicitly couples
position and velocity through the derivative term (the D term is effectively a
velocity gain). The real limitation is that PID gives you at most two gain knobs
for the full state (P for position, D for velocity) and cannot independently
weight more than two states or handle cross-coupling between different actuator
channels. The phrasing as written suggests PID literally ignores velocity, which
contradicts the earlier kD discussion.

## 4.11 — Bang-bang description says "cuts off ... inertia carries it the rest of the way"

The text says "the motor runs at full power until it approaches the target, then
cuts off. The inertia carries it the rest of the way." This is not how the
BangBangController works — it outputs 1 whenever measurement < setpoint, with no
"approaching" logic. It does not anticipate the target and cut off early. The
inertia of the flywheel plus the natural back-EMF ceiling prevent significant
overshoot, but the controller itself has no notion of approaching or coasting to
a stop. The description makes it sound like a smarter algorithm than it is.
