# Chapter 8: Online Trajectory Generation with Ruckig

Chapters 5 through 7 built trajectory profiles **offline**: the entire path from start to finish is computed at construction time and then sampled with `getPosition(t)`. This works well for single-axis mechanisms, but it has a structural limitation — the profile is a pure function of time, computed once from a fixed initial state and a fixed target. If the target changes, a new profile must be constructed and spliced in (Chapter 9 covers that splicing). And each axis is planned independently; there is no built-in way to synchronize a multi-degree-of-freedom system so that all axes arrive at their targets simultaneously.

**Online trajectory generation** takes a different approach. Instead of computing the entire profile up front, the planner is called once per control cycle. Each call receives the mechanism's current kinematic state and the desired target, and returns the state for the next cycle. The planner internally computes the time-optimal jerk-limited trajectory from the current state to the target, but it only reveals one time step's worth of that trajectory. On the next cycle, the planner is called again with the new state — which might reflect a changed target, an external disturbance, or simply the output of the previous cycle fed back as input.

The Ruckig library implements this pattern. It computes time-optimal, jerk-limited trajectories for an arbitrary number of degrees of freedom, synchronized so that all DOFs reach their targets at the same time. MarsCommonFtc wraps Ruckig through JNI to make it available from Java.

## 8.1 What Is Online Trajectory Generation?

The term "online" means the trajectory is computed in real time, as opposed to "offline" where the trajectory is pre-computed before motion begins. The distinction matters for two reasons:

**Reactivity.** An online planner can respond to target changes instantly. If the operator changes the target mid-move, the planner doesn't need to splice two profiles together — it simply plans a new trajectory from the current state to the new target on the next cycle. The result is time-optimal by construction.

**Multi-DOF synchronization.** When multiple axes must arrive simultaneously (e.g., an X-Y gantry or a multi-joint arm), an online planner can compute synchronized profiles that respect independent per-axis limits while coordinating arrival times. The offline profiles in Chapters 5–7 plan each axis independently; synchronization would require post-hoc time scaling.

The cost is a native library dependency. Ruckig is written in C++ and requires a JNI bridge to use from Java. The offline profiles are pure Java and have no native dependencies.

### Offline vs. Online: When to Use Each

| Property | Offline (SCurvePosition, etc.) | Online (Ruckig) |
|---|---|---|
| Language | Pure Java | C++ via JNI |
| Planning | Full trajectory at construction | One time step per call |
| Target changes | Requires manager-level replanning | Built-in, automatic |
| Multi-DOF | Independent per-axis | Synchronized arrival |
| Asymmetric accel/decel | Yes (SCurvePosition) | Community edition: no |
| Sinusoidal jerk | Yes (SinCurvePosition) | No |
| FTC deployment | Always available | Requires RuckigNative AAR |
| ControlLab desktop | Always available | Requires native build |

For most FTC mechanisms — single-axis arms, elevators, flywheels — the offline profiles are simpler, more flexible (asymmetric limits, sinusoidal jerk), and require no native code. Use Ruckig when you need multi-DOF synchronization or when the online replanning model is a better fit for your control architecture.

## 8.2 The Ruckig Library

Ruckig (Real-time Online Trajectory Generation with Kinematic Limits) is an open-source C++ library created by Lars Berscheid. It solves the **time-optimal jerk-limited trajectory problem**: given current and target positions, velocities, and accelerations, plus per-axis limits on velocity, acceleration, and jerk, find the trajectory that reaches the target in minimum time while respecting all limits.

The algorithm is based on the same 7-phase S-curve structure described in Chapter 6, but with two key additions:

1. **Closed-form solution.** Where `SCurvePosition` uses binary search to find `vPeak`, Ruckig solves the time-optimal profile analytically. This is faster and avoids the iterative convergence issues that binary search can encounter near edge cases.

2. **Multi-DOF synchronization.** For $n$ degrees of freedom, Ruckig computes the time-optimal profile for each axis independently, finds the slowest axis, and then stretches the faster axes to match. The result is that all axes arrive simultaneously, each respecting its own limits.

MarsCommonFtc includes Ruckig as a Git submodule at the repository root:

```
MarsCommonFtc/
├── ruckig/                  ← Git submodule (pantor/ruckig)
│   ├── include/ruckig/      ← C++ headers
│   └── src/ruckig/          ← C++ algorithm sources
├── RuckigNative/            ← JNI bridge module (Android AAR)
│   ├── src/main/cpp/        ← JNI wrapper (ruckig_jni.cpp)
│   └── CMakeLists.txt       ← Native build configuration
└── ControlLib/              ← Java wrapper classes
    └── .../ruckig/
        ├── RuckigController.java
        ├── RuckigInput.java
        ├── RuckigOutput.java
        └── RuckigResult.java
```

## 8.3 The JNI Bridge

Java cannot call C++ directly. The Java Native Interface (JNI) bridges the gap: Java declares `native` methods, and a shared library (`.so` on Android, `.dll` on Windows) implements them in C++.

### The C++ Side: RuckigHandle

The JNI wrapper caches the Ruckig planner and its I/O parameter objects in a heap-allocated struct to avoid re-allocating them on every `update()` call:

```cpp
struct RuckigHandle {
    int dofs;
    Ruckig<DynamicDOFs> ruckig;
    InputParameter<DynamicDOFs>  input;
    OutputParameter<DynamicDOFs> output;

    RuckigHandle(int dofs, double cycleTime)
        : dofs(dofs),
          ruckig(dofs, cycleTime),
          input(dofs),
          output(dofs) {}
};
```

`DynamicDOFs` is a template parameter (value 0) that tells Ruckig to use runtime-sized vectors instead of fixed-size arrays. This lets the same compiled library handle any number of degrees of freedom.

Three JNI functions form the complete API:

**`nativeCreate(dofs, cycleTime)`** — Allocates a `RuckigHandle` on the heap and returns its address as a `long`. The `cycleTime` parameter tells Ruckig how far forward to step on each `update()` call.

**`nativeUpdate(handle, ...)`** — The core function. It copies nine Java arrays (current state, target state, and limits) into the C++ `InputParameter`, calls `ruckig.update()`, copies the three output arrays (new position, velocity, acceleration) back to Java, and extracts the planned trajectory duration. Returns the result code as an `int`.

```cpp
Result result = h->ruckig.update(inp, out);

env->SetDoubleArrayRegion(outPos, 0, dofs, out.new_position.data());
env->SetDoubleArrayRegion(outVel, 0, dofs, out.new_velocity.data());
env->SetDoubleArrayRegion(outAcc, 0, dofs, out.new_acceleration.data());

double dur = out.trajectory.get_duration();
env->SetDoubleArrayRegion(outDuration, 0, 1, &dur);
```

**`nativeDestroy(handle)`** — Deletes the heap-allocated `RuckigHandle`. Called from `RuckigController.close()`.

### The Java Side: RuckigController

`RuckigController` is a thin wrapper that loads the native library, holds the opaque handle, and translates between Java arrays and JNI calls:

```java
public class RuckigController implements AutoCloseable {

    static {
        System.loadLibrary("ruckig_jni");
    }

    private final long handle;
    private final int dofs;
    private final double[] durationBuf = new double[1];

    public RuckigController(int dofs, double cycleTime) {
        this.dofs = dofs;
        this.handle = nativeCreate(dofs, cycleTime);
    }

    public RuckigResult update(RuckigInput in, RuckigOutput out) {
        int code = nativeUpdate(handle,
                in.currentPosition,     in.currentVelocity,     in.currentAcceleration,
                in.targetPosition,      in.targetVelocity,      in.targetAcceleration,
                in.maxVelocity,         in.maxAcceleration,     in.maxJerk,
                out.newPosition,        out.newVelocity,        out.newAcceleration,
                durationBuf);
        out.duration = durationBuf[0];
        return RuckigResult.fromCode(code);
    }

    @Override
    public void close() {
        nativeDestroy(handle);
    }
}
```

The `durationBuf` trick avoids allocating a new `double[1]` on every update. Since the JNI function writes the duration into an array, a reusable buffer eliminates GC pressure.

`RuckigController` implements `AutoCloseable`, so it can be used in try-with-resources blocks to ensure the native memory is freed.

## 8.4 Input and Output

### RuckigInput

`RuckigInput` holds the nine arrays that define the planning problem for one cycle:

```java
public class RuckigInput {
    public double[] currentPosition;
    public double[] currentVelocity;
    public double[] currentAcceleration;
    public double[] targetPosition;
    public double[] targetVelocity;
    public double[] targetAcceleration;
    public double[] maxVelocity;
    public double[] maxAcceleration;
    public double[] maxJerk;

    public RuckigInput(int dofs) {
        // allocates all arrays with length dofs
    }
}
```

All arrays have length `dofs`. For a single-axis mechanism, each array has one element. The `target` arrays specify not just the target position, but also the desired velocity and acceleration at the target — typically both zero for a rest-to-rest move, but non-zero targets are supported for through-motions.

### RuckigOutput

`RuckigOutput` holds the planner's answer: the kinematic state one cycle into the future.

```java
public class RuckigOutput {
    public double[] newPosition;
    public double[] newVelocity;
    public double[] newAcceleration;
    public double duration;  // total planned trajectory duration (seconds)

    public RuckigOutput(int dofs) {
        // allocates position/velocity/acceleration arrays
    }
}
```

The `duration` field is the total time of the planned trajectory from the current state to the target. It decreases on each cycle as the mechanism progresses. When the trajectory completes, `update()` returns `FINISHED` instead of `WORKING`.

### RuckigResult

The result enum maps the C++ `ruckig::Result` codes:

```java
public enum RuckigResult {
    WORKING(0),              // trajectory in progress
    FINISHED(1),             // target reached
    ERROR(-1),               // generic error
    ERROR_INVALID_INPUT(-100),
    ERROR_TRAJECTORY_DURATION(-101),
    ERROR_ZERO_LIMITS(-104),
    ERROR_EXECUTION_TIME_CALCULATION(-110),
    ERROR_SYNCHRONIZATION_CALCULATION(-111);
}
```

In normal operation, `update()` returns `WORKING` on every cycle until the mechanism reaches the target, then returns `FINISHED`. Error codes indicate invalid inputs (e.g., zero limits, infeasible constraints). Your control loop should check for errors and handle them — typically by logging the error and holding the current state.

## 8.5 Usage Pattern

The fundamental Ruckig usage pattern is a feedback loop:

1. Set the current state and target in `RuckigInput`
2. Call `update()` to get the next state
3. Apply the output to the mechanism (or feed it back as the next cycle's input)
4. Repeat until `FINISHED`

### 1-DOF Example: Position Move

```java
try (RuckigController ruckig = new RuckigController(1, 0.020)) {
    RuckigInput  in  = new RuckigInput(1);
    RuckigOutput out = new RuckigOutput(1);

    // Initial state: at rest at position 0
    in.currentPosition[0]     = 0;
    in.currentVelocity[0]     = 0;
    in.currentAcceleration[0] = 0;

    // Target: position 100, at rest
    in.targetPosition[0]      = 100;
    in.targetVelocity[0]      = 0;
    in.targetAcceleration[0]  = 0;

    // Kinematic limits
    in.maxVelocity[0]         = 10;
    in.maxAcceleration[0]     = 5;
    in.maxJerk[0]             = 50;

    RuckigResult result;
    do {
        result = ruckig.update(in, out);

        // Feed output back as next cycle's input
        in.currentPosition[0]     = out.newPosition[0];
        in.currentVelocity[0]     = out.newVelocity[0];
        in.currentAcceleration[0] = out.newAcceleration[0];

        // Apply to mechanism:
        // motor.setTargetPosition(out.newPosition[0]);

    } while (result == RuckigResult.WORKING);
}
```

This is a pure feedforward loop — the output of each cycle becomes the input of the next. No sensor feedback is involved. The trajectory is time-optimal by construction and respects the velocity, acceleration, and jerk limits on every cycle.

### Changing Targets Mid-Motion

The power of online trajectory generation is that you can change the target at any time:

```java
// Halfway through the move, change the target
if (someCondition) {
    in.targetPosition[0] = 50;  // new target
}
result = ruckig.update(in, out);
```

Ruckig re-plans the entire trajectory from the current state to the new target on each `update()` call. There is no need to splice profiles or manage continuity manually — the planner handles it internally.

### Multi-DOF Synchronized Motion

For a 2-DOF mechanism (e.g., X and Y axes of a gantry):

```java
try (RuckigController ruckig = new RuckigController(2, 0.020)) {
    RuckigInput  in  = new RuckigInput(2);
    RuckigOutput out = new RuckigOutput(2);

    // Axis 0: X
    in.currentPosition[0] = 0;
    in.targetPosition[0]  = 100;
    in.maxVelocity[0]     = 10;
    in.maxAcceleration[0] = 5;
    in.maxJerk[0]         = 50;

    // Axis 1: Y (different limits)
    in.currentPosition[1] = 0;
    in.targetPosition[1]  = 30;
    in.maxVelocity[1]     = 8;
    in.maxAcceleration[1] = 3;
    in.maxJerk[1]         = 20;

    // ... zero out velocities, accelerations, targets ...

    RuckigResult result;
    do {
        result = ruckig.update(in, out);
        in.currentPosition[0] = out.newPosition[0];
        in.currentVelocity[0] = out.newVelocity[0];
        in.currentAcceleration[0] = out.newAcceleration[0];
        in.currentPosition[1] = out.newPosition[1];
        in.currentVelocity[1] = out.newVelocity[1];
        in.currentAcceleration[1] = out.newAcceleration[1];
    } while (result == RuckigResult.WORKING);
}
```

Both axes arrive at their targets at the same time. Ruckig computes the time-optimal profile for each axis independently, identifies the slowest, and stretches the faster axes to match. Each axis still respects its own limits — the slower axis runs at full speed, while the faster axis is slowed down.

## 8.6 Building the Native Library

Ruckig requires a C++20 compiler. The build process is different for Android (on-robot) and desktop (ControlLab).

### Android (On-Robot)

The `RuckigNative` module is an Android library (AAR) that compiles the JNI wrapper and Ruckig sources using the Android NDK:

```
RuckigNative/
├── build.gradle            ← Android library plugin + NDK config
├── CMakeLists.txt          ← Native build: JNI wrapper + Ruckig sources
└── src/main/cpp/
    └── ruckig_jni.cpp      ← JNI implementation
```

The `CMakeLists.txt` compiles 12 source files into a single shared library:

- `ruckig_jni.cpp` — the JNI bridge
- 11 Ruckig algorithm files: `brake.cpp`, `position_first_step1.cpp` through `position_third_step2.cpp`, and `velocity_second_step1.cpp` through `velocity_third_step2.cpp`

The Gradle build configures the NDK for `arm64-v8a` and `armeabi-v7a` (the two ABI variants used by REV Control Hubs and Android phones). NDK r25 or later is required for C++20 support.

To include Ruckig in your FTC project, add the RuckigNative module dependency alongside ControlLib:

```gradle
dependencies {
    implementation project(':ControlLib')
    implementation project(':RuckigNative')
}
```

The AAR bundles the compiled `.so` files, and `System.loadLibrary("ruckig_jni")` loads the correct variant at runtime.

### Desktop (ControlLab)

ControlLab's `build.gradle` includes a CMake-based native build that compiles the same sources for the host platform. On Windows, it uses `vswhere` to locate Visual Studio and wraps the CMake invocation through `vcvarsall.bat` to make MSVC tools available. The resulting `.dll` (or `.so` on Linux) is placed on `java.library.path` at runtime.

The desktop build enables ControlLab's Ruckig trajectory visualization tab. If the native library is not available (e.g., because a C++ compiler is not installed), ControlLab gracefully degrades — the Ruckig option is disabled with a message explaining why.

### Availability Check

`TrajectoryEngine` performs a one-time availability check at class load:

```java
private static final boolean RUCKIG_AVAILABLE;

static {
    boolean ok = false;
    try {
        Class.forName("org.marsroboticsassociation.controllib.motion.ruckig.RuckigController");
        try (var r = new RuckigController(1, 0.020)) {
            ok = true;
        }
    } catch (UnsatisfiedLinkError | Exception e) {
        ok = false;
    }
    RUCKIG_AVAILABLE = ok;
}
```

This two-step check first verifies the class is on the classpath, then verifies the native library can actually be loaded and initialized. An `UnsatisfiedLinkError` means the `.so`/`.dll` is missing; any other exception means something is wrong with the Ruckig initialization.

## 8.7 Ruckig in ControlLab

The ControlLab trajectory visualization tab supports Ruckig as one of four trajectory types. When selected, it displays three parameter sliders:

- **vMax** — maximum velocity
- **aMax** — maximum acceleration
- **jMax** — maximum jerk

The simulation runs at 20 ms per tick, matching the typical FTC control loop period. Each tick calls `ruckig.update()`, feeds the output back as input, and plots the resulting position, velocity, and acceleration traces:

```java
case RUCKIG:
    var result = ruckig.update(rIn, rOut);
    rIn.currentPosition[0]     = rOut.newPosition[0];
    rIn.currentVelocity[0]     = rOut.newVelocity[0];
    rIn.currentAcceleration[0] = rOut.newAcceleration[0];
    lastP = rOut.newPosition[0];
    lastV = rOut.newVelocity[0];
    lastA = rOut.newAcceleration[0];
    if (result != RuckigResult.WORKING) {
        ruckigFinished = true;
        moving = false;
    }
    break;
```

Unlike the S-curve profiles, Ruckig does not currently support exact SVG export in ControlLab. The S-curve profiles expose their piecewise polynomial structure through `positionSegments()`, `velocitySegments()`, and `accelerationSegments()` methods, which can be rendered as exact curves. Ruckig's trajectory is sampled point-by-point, so the plot is a discrete approximation at the 20 ms tick rate.

## 8.8 Comparison with MarsCommonFtc's S-Curve Profiles

The S-curve profiles and Ruckig solve the same fundamental problem — time-optimal jerk-limited motion — but with different trade-offs:

### Ruckig's Advantages

- **Closed-form solver.** No binary search for `vPeak`; the solution is computed directly. This avoids the 64-iteration bisection loop and its associated edge cases.
- **Multi-DOF synchronization.** Built-in support for coordinating multiple axes. The S-curve profiles would require external time-scaling logic.
- **Online replanning.** Target changes are handled naturally on each cycle. No need for a separate manager layer to handle replanning.

### S-Curve Advantages

- **Pure Java.** No native library, no JNI, no NDK dependency. Works on any platform without compilation.
- **Asymmetric acceleration.** `SCurvePosition` supports independent `aMaxAccel` and `aMaxDecel` limits. Ruckig's community edition uses symmetric acceleration limits.
- **Sinusoidal jerk.** `SinCurvePosition` provides jerk-continuous trajectories with raised-cosine transitions. Ruckig uses piecewise-constant jerk (same as `SCurvePosition`).
- **Full trajectory access.** The offline profiles expose the complete trajectory function: `getPosition(t)` for any `t ∈ [0, tf]`. Ruckig only reveals one time step at a time.
- **Exact SVG export.** The polynomial/trigonometric structure enables analytically exact visualization. Ruckig trajectories must be sampled.
- **Back-EMF-aware tuning.** `SCurveVelocity.findMaxJDec()` and `findMaxAMax()` can search for limits that respect motor voltage constraints (Chapter 10). Ruckig has no built-in voltage awareness.

### When to Choose Each

**Use the S-curve profiles (default) when:**
- You control a single axis (arm, elevator, flywheel)
- You need asymmetric acceleration or sinusoidal jerk
- You want pure Java with no native dependencies
- You need full trajectory access for analysis or visualization
- You need back-EMF-aware limit tuning

**Use Ruckig when:**
- You need multi-DOF synchronized motion
- You prefer the online replanning model over explicit trajectory management
- You are comfortable with the native library dependency

## 8.9 Summary

Ruckig provides time-optimal, jerk-limited online trajectory generation for multi-DOF systems. MarsCommonFtc wraps it through a JNI bridge:

- **`RuckigController`** — Java wrapper holding a C++ `Ruckig<DynamicDOFs>` instance behind an opaque `long` handle
- **`RuckigInput`** — nine `double[]` arrays: current state, target state, and kinematic limits
- **`RuckigOutput`** — three `double[]` arrays for the next state, plus the planned trajectory duration
- **`RuckigResult`** — enum mapping C++ result codes: `WORKING`, `FINISHED`, or one of six error variants
- **Usage pattern** — call `update()` once per control cycle, feed output back as next input
- **Build** — Android via NDK (AAR); desktop via CMake + MSVC/GCC
- **Graceful degradation** — `TrajectoryEngine.isRuckigAvailable()` checks at class load; ControlLab disables the Ruckig tab if the native library is missing

For most FTC mechanisms, the offline S-curve profiles are the better default — simpler, more flexible, and pure Java. Ruckig is the right tool when you need multi-DOF synchronization or prefer the online planning model.

Chapter 9 describes the trajectory management layer that wraps the offline profiles, handling mid-motion replanning, tolerance-based change detection, and telemetry.
