# Back-EMF and Flywheel Motion Profiles

## What is Back-EMF?

When a DC motor spins, it acts as a generator. The spinning magnets in the motor create a voltage that opposes the applied voltage — this is **back-EMF** (electromotive force).

Think of it like air resistance while running: the faster you go, the harder it is to go faster. With a motor: the faster it spins, the more voltage is "stolen" by the generation effect.

### The Voltage Balance

At any instant, the applied voltage is consumed by three things:

```
V_applied = V_friction + V_back-EMF + V_acceleration
```

In feedforward terms:

```
V = kS + kV·v + kA·a
```

| Term | Meaning |
|------|---------|
| kS   | Coulomb friction (voltage needed to overcome static friction) |
| kV   | Back-EMF constant (voltage per unit velocity) |
| kA   | Acceleration constant (voltage to accelerate the mass) |

As velocity increases, kV·v grows, leaving less voltage for acceleration.

## Maximum Speed

At steady state (acceleration = 0), all voltage goes to friction and back-EMF:

```
V = kS + kV·v_ss
```

Solving for v_ss:

\[
v_{ss} = \frac{V - kS}{kV}
\]

This is the **absolute maximum velocity** your motor can reach at a given voltage. To go faster, you'd need more voltage (higher battery, boost capacitor) or reduce friction.

## Analytical Motion Profile

Given no acceleration cap (just back-EMF physics), the velocity follows a first-order ODE:

\[
\frac{dv}{dt} = \frac{V - kS - kV \cdot v}{kA}
\]

### Acceleration Phase (target > initial velocity)

The solution is exponential approach to steady-state:

\[
v(t) = v_{ss} + (v_0 - v_{ss}) \cdot e^{-\frac{kV}{kA} \cdot t}
\]

Where:
- \( v_0 \) is the initial velocity at t = 0
- \( v_{ss} = (V - kS) / kV \) is the steady-state velocity
- \( kV/kA \) determines the time constant

### Velocity at any time

\[
v(t) = v_{ss} - (v_{ss} - v_0) \cdot e^{-\frac{kV}{kA} \cdot t}
\]

### Acceleration at any time

Differentiating velocity:

\[
a(t) = \frac{dv}{dt} = (v_0 - v_{ss}) \cdot \left(-\frac{kV}{kA}\right) \cdot e^{-\frac{kV}{kA} \cdot t}
\]

Or more compactly:

\[
a(t) = a_0 \cdot e^{-\frac{kV}{kA} \cdot t}
\]

Where \( a_0 = (V - kS - kV \cdot v_0) / kA \) is the initial acceleration.

Both velocity and acceleration decay exponentially toward zero as the motor approaches steady-state.

## Deceleration Phase

When decelerating (target < current velocity):

\[
v(t) = v_{target} + (v_0 - v_{target}) \cdot e^{+\frac{kV}{kA} \cdot t}
\]

## Practical Implications

1. **You cannot command a velocity above v_ss** — the motor will asymptotically approach v_ss regardless of what you set as target.

2. **The time constant is kA/kV** — this is how long it takes for velocity to get ~63% of the way to steady-state.

3. **The iterative algorithm in FlywheelSimple** implicitly handles this by recomputing acceleration each timestep. The analytic form lets you compute the profile ahead of time if desired.

## Summary

| Formula | Description |
|---------|-------------|
| \( v_{ss} = (V - kS) / kV \) | Maximum velocity (steady-state) |
| \( v(t) = v_{ss} + (v_0 - v_{ss}) \cdot e^{-(kV/kA)t} \) | Velocity over time (acceleration) |
| \( a(t) = a_0 \cdot e^{-(kV/kA)t} \) | Acceleration over time |
