# Plan: Port Ruckig (Community) to Java

This document is a handoff plan for porting the [Ruckig](https://github.com/pantor/ruckig)
online trajectory generation library (Community version, MIT license) to pure Java, for use
in this repo's ControlLib mechanism stack and on FTC robot controllers.

**Audience:** the model/engineer executing the port. Read this whole document before writing
any code. The milestones are ordered to de-risk the hardest parts first and every milestone
has an acceptance gate — do not proceed past a gate with failing tests.

---

## 1. Context and goal

ControlLib currently profiles mechanism setpoints with `CascadedRateLimiter` (a greedy
one-step filter — cannot finish a move smoothly; it rides the stopping curve and clamps)
and has a hand-written S-curve planner (`SCurvePosition`, target-at-rest only). Ruckig is
the reference OTG algorithm: each control cycle it computes the time-optimal, jerk-limited
trajectory from an arbitrary state `(p, v, a)` to an arbitrary target state `(pf, vf, af)`,
under `vMax`, asymmetric `aMax`, and `jMax` limits, including brake pre-trajectories when
the current state is infeasible, and multi-DoF time synchronization.

**Goal:** a faithful, allocation-free-in-steady-state, pure-Java port of the Ruckig
Community algorithm, validated against the C++ implementation by golden-vector differential
testing.

**Non-negotiable ground rules:**

1. **This is a transliteration, not a reimplementation.** Keep upstream's file structure,
   class names, function names, variable names (even terse ones like `vd`, `tf`, `h1`),
   constants, and *order of arithmetic operations*. Floating-point math is not associative;
   an innocent refactor changes which profile candidate wins at a boundary and the golden
   tests will fail in ways that are miserable to debug. Idiomatic-Java cleanups are
   forbidden in the core algorithm files, forever — diffability against upstream is a
   feature we are buying on purpose.
2. **Pin the upstream version.** Port from tag `v0.17.3` (latest as of 2026-07). Record the
   tag and commit SHA in the module README. All golden vectors must be generated from the
   same tag.
3. **Golden harness before algorithm.** Milestone 0 builds the differential test rig; no
   algorithm code is ported until it exists.
4. **Never silently fix an upstream bug.** If the port appears to expose a defect in
   upstream, reproduce it in C++/Python first, then record it in `DIVERGENCES.md` with a
   link, and match upstream behavior anyway (the golden tests demand it).
5. **Java 8 language level.** The library must compile at Java 8 source compatibility so it
   works under OnBotJava as well as Android Studio. No records, no `var`, no switch
   expressions, no `Math.fma` (Java 9+).

---

## 2. Scope

### In scope (Community feature set)

| Upstream file(s) | What it is |
|---|---|
| `roots.hpp` | Polynomial solvers: cubic (Cardano + trig reduction), monic quartic (resolvent cubic), Horner eval, Newton–bisection `shrink_interval` (tol ≈ 1e-14), stack-based `Set`/`PositiveSet` root containers |
| `profile.hpp` | `Profile` struct: `t[7]`, `t_sum[7]`, `j[7]`, `a[8]`, `v[8]`, `p[8]`, embedded `BrakeProfile brake, accel`; the templated `check<ControlSigns, ReachedLimits>` validators; the epsilon constants |
| `brake.hpp` / `brake.cpp` | Brake pre-trajectories for infeasible initial states |
| `position.hpp`, `position_third_step1.cpp`, `position_third_step2.cpp` | Jerk-limited position interface. Step 1 = min-duration profile + blocked intervals; Step 2 = profile for a *given* duration (synchronization) |
| `velocity.hpp`, `velocity_third_step1.cpp`, `velocity_third_step2.cpp` | Jerk-limited velocity interface (much simpler; port first as architecture shakedown) |
| `position_second_step*.cpp`, `position_first_step*.cpp`, `velocity_second_step*.cpp` | Second-order (no jerk limit) and first-order interfaces — port after third-order works |
| `block.hpp` | Blocked-interval representation used for multi-DoF sync |
| `calculator_target.hpp`, `calculator.hpp` | The per-DoF Step1/Step2 orchestration and time synchronization |
| `trajectory.hpp` | Trajectory container + `at_time` sampling |
| `input_parameter.hpp`, `output_parameter.hpp`, `result.hpp`, `error.hpp`, `utils.hpp`, `ruckig.hpp` | API surface: `InputParameter`, `OutputParameter`, `Result` codes, `Ruckig` update loop |

Also in scope: synchronization modes (`Time`, `TimeIfNecessary`, `Phase`, `None`), per-DoF
`enabled` flags, `minimum_duration`, discrete durations if trivially portable.

### Out of scope

- `calculator_cloud.hpp` / `cloud_client.cpp` (Pro/cloud waypoint features, networking).
- Pro-only features: intermediate waypoints, position/velocity range constraints, tracking
  interface.
- Custom vector-type templates (Eigen wrappers): the port supports exactly one storage
  strategy, `double[]` sized at construction (upstream's `DynamicDOFs` mode). Do not
  attempt to mirror the compile-time `DOFs` template.
- `benchmark_target.cpp` (a simple JMH or loop-time benchmark replaces it).

---

## 3. Module layout

Follow the `WpiMath/` precedent in this repo (a ported library kept as its own Gradle
module with upstream naming):

```
RuckigJava/
  src/main/java/com/ruckig/        <- package com.ruckig (upstream domain is ruckig.com)
    Roots.java
    Profile.java                   (+ BrakeProfile, ControlSigns, ReachedLimits, Direction)
    Brake.java
    PositionThirdOrderStep1.java
    PositionThirdOrderStep2.java
    VelocityThirdOrderStep1.java
    VelocityThirdOrderStep2.java
    ... (one Java file per upstream translation unit / header, same names)
    InputParameter.java
    OutputParameter.java
    Trajectory.java
    Ruckig.java
    Result.java
  src/test/java/com/ruckig/
  src/test/resources/golden/       <- generated golden vectors (JSON), checked in
  tools/generate_golden.py         <- generation script (runs against pip ruckig)
  LICENSE                          <- upstream MIT license, verbatim
  README.md                        <- upstream tag + SHA, divergences pointer
  DIVERGENCES.md
```

Keep ControlLib integration (adapters) **out** of this module; `RuckigJava` must have zero
dependencies beyond the JDK.

---

## 4. Milestones and acceptance gates

Ordered so each stage is testable in isolation and the scariest numerics come early.

**M0 — Test rig first.**
Gradle module skeleton; JSON golden-vector loader; `tools/generate_golden.py` that
`pip install ruckig==<matching version>` (verify the PyPI version corresponds to tag
v0.17.3; if not, build the Python module from the pinned tag) and emits vectors (format in
§5). *Gate:* harness loads a hand-written sample vector and a dummy test fails correctly.

**M1 — `Roots`.**
Port `roots.hpp` verbatim: `solve_cubic`, `solve_quart_monic`, `solve_resolvent`,
`poly_eval`, `poly_derivative`, `poly_monic_derivative`, `shrink_interval`, and the
stack-based `Set`/`PositiveSet` (port as a small fixed-capacity double-array class with
insertion count — no `java.util` collections). *Gate:* standalone golden vectors — random
coefficient sets (including near-degenerate: leading coeff ~1e-16, repeated roots, all-real
vs one-real cubics) solved by the C++/Python side, Java roots match to 1e-12 absolute or
relative, plus residual check `|poly_eval(root)| < tol`.

**M2 — `Profile` + check functions.**
Port the struct, the epsilon constants **verbatim with a comment naming the upstream
constant** (`v_eps = 1e-12`, `a_eps = 1e-12`, `j_eps = 1e-12`, `p_precision = 1e-8`,
`v_precision = 1e-8`, `a_precision = 1e-10`, `t_precision = 1e-12`, `t_max = 1e12`), the
enums (`ControlSigns.UDDU/UDUD`, `ReachedLimits`, `Direction`), `set_boundary`, and the
`check*` family. The C++ `check<ControlSigns, ReachedLimits>` template pair becomes plain
method parameters (two enum args) or a small set of overloads — measure nothing yet, just
match behavior; do **not** get clever with polymorphism. *Gate:* unit tests that integrate
known 7-phase profiles forward and confirm `check` accepts them and rejects
limit-violating variants.

**M3 — `Brake`.**
Brake pre-trajectory computation (`BrakeProfile`: velocity/acceleration outside limits at
start). *Gate:* golden vectors whose inputs start beyond limits (|v0| > vMax, |a0| > aMax)
match C++ brake durations and post-brake states.

**M4 — Velocity interface, third order (`VelocityThirdOrderStep1/Step2`).**
Small, exercises the whole pipeline (candidates → check → block). *Gate:* single-DoF
velocity-interface golden vectors pass end to end through a ported `Trajectory.at_time`.

**M5 — Position interface Step 1 (the core).**
`position_third_step1.cpp` — the largest and hardest file: all profile candidates (UDDU/
UDUD × reached-limit combinations), quartic solving, and blocked-interval extraction.
*Gate:* single-DoF golden vectors — minimum duration matches to 1e-12 relative AND sampled
kinematics match (§5); brake-prefixed cases included.

**M6 — Step 2 + multi-DoF synchronization.**
`position_third_step2.cpp`, `block.hpp`, `calculator_target.hpp`: profiles for a prescribed
duration, blocked-interval intersection, sync-mode handling. *Gate:* multi-DoF golden
vectors (2–6 DoF) with Time/Phase/None sync all pass; includes cases where the naive common
duration falls inside another DoF's blocked interval.

**M7 — Second- and first-order interfaces.**
Straightforward after M5/M6. *Gate:* corresponding golden vectors.

**M8 — Public API + OTG stepping.**
`Ruckig.update(input, output)` loop, `InputParameter` validation, `Result` codes,
`OutputParameter.pass_to_input`. *Gate:* **trace tests** — recorded multi-cycle C++ OTG
sessions (state fed back each cycle, target/limits changed mid-flight) replayed in Java;
every cycle's `new_position/velocity/acceleration` must match. This catches state-handoff
bugs that one-shot tests structurally cannot.

**M9 — Property tests + performance pass.**
Port the invariant checks from upstream `test/test_target.cpp` + `randomizer.hpp` as a Java
property test (randomized inputs; assert: limits respected within 1e-12, final state
reached within `p/v/a_precision`, `t_sum` monotone, Step2(Step1-duration) reproduces
Step1). Run ≥ 10⁶ random cases in CI. Then the allocation audit: zero allocations in
steady-state `update()` (verify with a loop + `GarbageCollectorMXBean` count assertion or
JMH `gc.alloc.rate.norm ≈ 0`). *Gate:* property suite green; allocation test green;
single-DoF `update()` ≤ ~50 µs on desktop JVM (sanity bound, not a target).

---

## 5. Golden-vector harness (the core de-risker)

Upstream's own confidence comes from billions of randomized trajectories checked against
invariants. The port gets a stronger tool: differential testing against upstream itself.

**Generation** (`tools/generate_golden.py`, run offline, output checked into
`src/test/resources/golden/`):

- Input distributions mirroring upstream `test/randomizer.hpp` — plus directed edge
  batches: zero distance; target behind with wrong-way v0; a0 at/over limit (brake);
  |v0| > vMax (brake); vf ≠ 0 and af ≠ 0 targets; tiny limits (1e-6) and huge limits
  (1e6) mixed across DoFs; limits equal to each other; states exactly on limits;
  1-DoF through 6-DoF; each sync mode; disabled DoFs.
- Per case record: full `InputParameter`, `Result` code, trajectory duration, independent
  min-durations per DoF, and **sampled kinematics** `(p, v, a)` at ~32 times spread over
  `[0, duration]` plus the exact phase-boundary times.
- Trace files for M8: per-cycle sequences (≥ 200 cycles) with mid-run target and limit
  changes, recording each cycle's output state and result code.

**Comparison rules (important):**

- Compare **sampled kinematics**, not internal phase arrays. Two profile candidates can be
  equally time-optimal; a last-ulp difference can flip which one wins while producing an
  equally valid trajectory. Phase arrays are diagnostics only.
- Start at tight tolerances (relative 1e-12 on duration, absolute 1e-10 on sampled
  kinematics scaled by limit magnitudes). A faithful transliteration should get close to
  bit-identical. Every tolerance loosening must be justified per-case in a comment — a
  needed loosening usually means an operation-order divergence worth finding.
- When a case fails, bisect by layer: does `Roots` agree on that case's polynomials? Does
  `check` accept the C++ winning candidate? This is why M1/M2 have standalone goldens.

---

## 6. Pitfall catalog (C++ → Java)

Read this before porting each file; most of these have bitten every C++→Java port ever done.

1. **Value semantics vs references — the #1 bug source.** In C++, `Profile` and `Block`
   are copied by value on assignment and when stored in candidate arrays/optionals. In
   Java, `=` aliases. Every upstream `profile = other`, pass-by-value, or "stash the best
   candidate" must become an explicit deep `copyFrom(other)` (including the embedded
   `BrakeProfile brake, accel` and all arrays). Grep the C++ for `Profile ` locals and
   assignments and audit each one. Symptom if missed: later candidates corrupt the stored
   winner; goldens fail only on multi-candidate cases.
2. **`std::min`/`std::max` vs `Math.min`/`Math.max` NaN semantics differ.**
   C++ `std::min(a,b)` is `(b<a) ? b : a` — with NaN it silently picks an operand;
   `Math.min` propagates NaN. Also `Math.min(0.0, -0.0)` handles signed zero, C++ doesn't.
   Where NaN can flow (root solvers, division by near-zero), transliterate as the explicit
   ternary, not `Math.min`. Add a helper `cppMin/cppMax` and use it mechanically.
3. **Reused mutable state between candidates.** Upstream loops try candidates by
   overwriting the same `Profile` fields, relying on `check` to fully overwrite what
   matters. Do not "helpfully" reset fields between candidates and do not skip
   assignments that look redundant — order and completeness of writes must match.
4. **`std::optional<T>` → nullable is a trap for hot paths.** Use a boolean return with a
   caller-owned out-object (`boolean calculateBlock(Block out)`), preserving upstream's
   write-into-me pattern. No `java.util.Optional` anywhere in the core.
5. **Templates.** `Ruckig<DOFs, CustomVector>` → dynamic only: `double[]` sized at
   construction. `check<ControlSigns, ReachedLimits>` template params → enum arguments.
   `poly_eval<N>` → int length parameter or per-N overloads. Never emulate templates with
   generics over boxed `Double` — that allocates.
6. **Unsigned `size_t` loops.** `for (size_t i = n - 1; i < n; --i)`-style reverse loops
   and `size_t` underflow idioms must be re-derived, not transliterated.
7. **Uninitialized POD vs Java zero-init.** C++ leaves struct members uninitialized; Java
   zeroes them. This *hides* read-before-write bugs rather than causing them, but it means
   a golden mismatch can trace to upstream reading garbage it happens to have written
   earlier — if you find such a case, see ground rule 4.
8. **Math function mapping.** `std::cbrt`→`Math.cbrt`, `std::copysign`→`Math.copySign`,
   `std::abs`→`Math.abs`, `std::atan2`→`Math.atan2` — all fine. `std::pow` with negative
   base and fractional exponent → NaN in both. `roots.hpp` uses only
   `sqrt/cbrt/cos/acos/atan` — no `fma`, no `hypot` (good: `Math.hypot` is slow and
   `Math.fma` is Java 9+).
9. **`DBL_EPSILON`** = `Math.ulp(1.0)` = 2.220446049250313e-16. Define it once as a named
   constant; do not substitute a rounder number.
10. **Keep every epsilon verbatim.** The `1e-12`/`1e-8`/`1e-14` constants in `profile.hpp`
    and `roots.hpp` are load-bearing and co-tuned. Changing one "for robustness" breaks
    candidate acceptance in ways the property tests catch only statistically.
11. **Operation order.** `a*b + a*c` ≠ `a*(b+c)` in floating point. Compilers don't reorder
    FP by default (and Ruckig doesn't build with `-ffast-math`), so upstream source order
    is the spec. Transliterate expressions token-for-token, parentheses included.
12. **No allocation in `update()`.** Preallocate in the `Ruckig`/calculator objects: all
    candidate `Profile`s, `Block`s, root-set buffers, and the `Trajectory`. No
    `new`, no boxing, no varargs, no enhanced-for over collections, no streams, no
    lambdas capturing state, in anything reachable from `update()` or `at_time()`.
    `InputParameter`/`OutputParameter` are caller-owned and reused.
13. **Exceptions only at the API rim.** Core returns `Result` codes like upstream
    (`Working`, `Finished`, `ErrorInvalidInput`, ...). Input validation may throw from a
    convenience wrapper, never from the algorithm.
14. **Bool-to-int and implicit conversions.** C++ allows `t < 0.0` chains mixed with
    integer arithmetic and implicit bool conversion; make each conversion explicit and
    double-check comparisons that upstream writes as `!(x > 0)` (NaN-rejecting) vs
    `x <= 0` (NaN-accepting) — they differ, and this codebase's own
    `CascadedRateLimiter` uses that distinction deliberately.
15. **Porting process hygiene.** One upstream file per PR/commit, with its gate tests.
    Keep a `PORTING_LOG.md` mapping each Java file to its upstream file + SHA. Do not
    start a file until its dependencies' gates are green.

---

## 7. Integration notes (after M9 — separate effort, separate PRs)

Not part of the port proper, but the port should not paint us into a corner:

- **Adapter, not rewrite:** implement ControlLib's `PositionTrajectory` /
  `PositionTrajectoryManager.TrajectoryFactory` on top of `com.ruckig.Trajectory` for
  drop-in comparison against `SCurvePosition`, and a `RuckigProfiler` with the same
  per-loop surface as `CascadedRateLimiter` (`update(target, dt)`, `get*`,
  `setMax*`) for `MotorMechanismController`.
- **Velocity-dependent (back-EMF) limits are not Ruckig's problem to solve.** The
  controller rewrites limits every loop; the adapter replans per cycle from the profile's
  own state (Ruckig's intended usage). Plan the *deceleration* limit conservatively
  (evaluate the model's decel ceiling near zero speed for the remaining move), or the
  shrinking brake authority reintroduces the late-braking clamp the port exists to kill.
  See conversation/design notes from 2026-07-14.
- Ruckig's nonzero target-velocity support is the capability `SCurvePosition` lacks —
  keep it exposed through the adapter rather than hardcoding `vf = af = 0`.
- **Limit-frame mapping.** Ruckig's asymmetric limits are *signed*: `max_acceleration`
  bounds +a, optional `min_acceleration` bounds −a (defaults to the negative max). There
  is no `min_jerk` — jerk is symmetric, which is fine (ControlLib's `SCurvePosition` is
  symmetric-jerk too, and back-EMF does not constrain jerk). ControlLib's asymmetry is in
  the *travel* frame (accelerating vs braking relative to motion direction). The adapter
  must remap per plan: for a move in the + direction, `max_acceleration = aAccel`,
  `min_acceleration = -aDecel`; flipped for a − move. Per-cycle replanning keeps the
  mapping fresh, but a single plan containing a direction reversal (brake-then-reverse)
  gets the wrong bound on one segment — acceptable under replanning, but a deliberate,
  documented choice in the adapter, not an accident.

---

## 8. Definition of done

- [ ] Every in-scope upstream file has a Java counterpart, named and structured 1:1,
      with upstream tag + SHA recorded.
- [ ] All golden-vector suites pass at documented tolerances (roots, brake, velocity,
      position Step1, Step2/sync, second/first order, OTG traces).
- [ ] Property suite (ported upstream invariants) green over ≥ 10⁶ randomized cases in CI.
- [ ] Zero-allocation steady-state `update()` verified by test.
- [ ] Compiles at Java 8 source level; no dependencies beyond the JDK.
- [ ] `LICENSE` (MIT, upstream copyright) present; `DIVERGENCES.md` and `PORTING_LOG.md`
      complete (empty divergences is the expected outcome).
