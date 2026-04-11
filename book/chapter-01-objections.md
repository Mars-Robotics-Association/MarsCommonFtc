# Chapter 1 Objections

## 1.3 — WpiMath package claim

> `org.marsroboticsassociation.wpimath.*` -- classes from WpiMath

Section 1.6 states that WpiMath classes live under `org.marsroboticsassociation.wpimath.*`. They do not. The WpiMath module retains the upstream WPILib package namespace `edu.wpi.first.math.*`. Only ControlLib uses the `org.marsroboticsassociation` namespace. The bullet in 1.6 should be corrected or removed.

## 1.3 — "downstream projects declare EJML as compileOnly"

> WpiMath depends on EJML 0.44.0 for matrix operations. To prevent classpath conflicts, downstream projects declare it as `compileOnly`...

WpiMath declares EJML as `api`, not `compileOnly`. The `api` configuration is what allows ControlLib (a downstream project) to see EJML on its compile classpath without re-declaring it. The shadow JAR relocation in ControlLib is the actual mechanism that prevents classpath conflicts, not a `compileOnly` declaration. The paragraph conflates two different things.

## 1.3 — ControlLib filter list overstates what ControlLib contains

> Signal filters -- variable-dt low-pass, biquad, median, and slew rate limiters

Median filter (`MedianFilter`) and slew rate limiter (`SlewRateLimiter`) live in WpiMath (`edu.wpi.first.math.filter`), not in ControlLib. ControlLib's own filter package contains `IIR1LowPassVarDt`, `BiquadLowPassVarDt`, `KalmanFilter`, and `LowPassFilter`. The list should either be corrected to reflect what ControlLib actually provides or should note that some filters come from WpiMath.

## 1.3 — ControlLib trajectory list includes "trapezoidal"

> Trajectory generators -- S-curve, sinusoidal, trapezoidal, and Ruckig-based jerk-limited profiles

`TrapezoidProfile` lives in WpiMath (`edu.wpi.first.math.trajectory`), not in ControlLib. ControlLib's motion package has S-curve and sinusoidal profiles, plus the Ruckig wrapper. Trapezoidal should be attributed to WpiMath or the sentence reworded.

## 1.5 — "compileOnly" explanation is misleading

> `compileOnly` -- available at compile time but not included in the APK (the robot controller already provides EJML)

The parenthetical claim that "the robot controller already provides EJML" is stated as fact but is not obviously true. The FTC Robot Controller SDK does not ship EJML as a bundled dependency. If this claim is wrong, `compileOnly` would cause a runtime `ClassNotFoundException`. The shadow JAR already bundles a relocated copy of EJML, so it is unclear why WpiMath also needs to be available at runtime independently. This needs clarification -- either the robot controller really does provide EJML (cite where), or the `compileOnly` usage has a different justification.

## 1.5 — Shadow JAR explanation

> If MarsCommonFtc uses EJML 0.44.0 and another library uses EJML 0.39.0, you get a classpath conflict.

This is fine as a simplified explanation, but the term "classpath conflict" could mislead readers into thinking the build will fail. The more precise problem is a runtime version mismatch: the classloader picks one version, and the other library may call methods or use fields that do not exist in that version. For a student audience this might be fine, but "classpath conflict" without further explanation could leave them confused about what actually goes wrong.

## 1.2 / 1.4 — Submodule framing assumes the reader's project is the outer repo

The chapter is written as though MarsCommonFtc is always consumed as a submodule inside a separate robot project. But the repository being described (MarsCommonFtc itself) is the library, not the consumer. The `.gitmodules` in this repository declares only one submodule -- `ruckig` -- not MarsCommonFtc. Section 1.4 shows a `.gitmodules` example declaring `MarsCommonFtc` as a submodule, which would appear in the robot project's repo, not here. This is not wrong per se, but the chapter silently switches perspective between "this is MarsCommonFtc" and "this is your robot project that contains MarsCommonFtc." A sentence or two acknowledging the switch would reduce confusion.
