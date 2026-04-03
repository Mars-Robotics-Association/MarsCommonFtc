# Chapter 1: Getting Started with FTC Java Development

This chapter walks you through the foundation of a modern FTC codebase built around MarsCommonFtc — a shared library that provides production-ready control algorithms, math utilities, and simulation tools. By the end of this chapter, you will understand how the project is structured, how shared code flows into your robot project, and how to build and run everything.

## 1.1 The Problem with Copy-Paste Code

Every FTC team eventually faces the same dilemma: you write a PID controller for your arm, and it works well enough. Next season, you copy it into a new project. Then you copy it again for a slide mechanism. Then another team wants it. Suddenly you have five slightly different versions of the same code, and fixing a bug means patching all five copies.

MarsCommonFtc solves this by treating control code as a **shared library** — a single source of truth that multiple robot projects consume. When a bug is fixed or an algorithm improved, every project that uses the library benefits immediately.

## 1.2 Project Structure

A MarsCommonFtc-based robot project has two major parts: the **robot project** itself and the **MarsCommonFtc submodule** nested inside it.

```
your-robot-project/
├── TeamCode/                  # Your robot-specific code
│   ├── build.gradle
│   └── src/
│       └── main/java/
│           └── org/firstinspires/ftc/teamcode/
│               ├── subsystems/
│               ├── commands/
│               └── ...
├── MarsCommonFtc/             # Git submodule — shared library
│   ├── WpiMath/               # Ported WPILib math library
│   ├── ControlLib/            # Motor controllers, trajectories, filters
│   ├── ControlLab/            # Desktop visualization app
│   ├── RuckigNative/          # JNI wrapper for Ruckig trajectory library
│   └── ruckig/                # Nested submodule: C++ Ruckig library
├── settings.gradle            # Gradle project configuration
├── build.gradle               # Root build configuration
└── build.common.gradle        # Shared build settings
```

The `MarsCommonFtc/` directory is not a copy — it is a **git submodule** pointing to a remote repository. This means the code lives in its own repository with its own version history, and your robot project references a specific commit.

## 1.3 The Four Modules

MarsCommonFtc is organized into four modules, each serving a distinct purpose:

### WpiMath

A ported subset of WPILib's math library — the standard robotics mathematics library used in FRC. It provides geometry types (`Pose2d`, `Translation2d`, `Rotation2d`), kinematics (`MecanumDriveKinematics`, `DifferentialDriveOdometry`), trajectory generation (`TrapezoidProfile`, `TrajectoryGenerator`), controllers (`PIDController`, `ProfiledPIDController`, `RamseteController`), feedforward models (`SimpleMotorFeedforward`, `ArmFeedforward`, `ElevatorFeedforward`), state estimation (`KalmanFilter`, `ExtendedKalmanFilter`), and linear algebra (`Matrix`, `Vector`, `DARE` solver).

WpiMath depends on EJML 0.44.0 for matrix operations. To prevent classpath conflicts, downstream projects declare it as `compileOnly` — the classes are available at compile time but the actual EJML dependency is provided by the robot controller runtime.

### ControlLib

The core control library. This is where you will spend most of your time. It contains:

- **Motor controllers** — abstract base classes and concrete implementations for arm, flywheel, elevator, and velocity control
- **Trajectory generators** — S-curve, sinusoidal, trapezoidal, and Ruckig-based jerk-limited profiles
- **Signal filters** — variable-dt low-pass, biquad, median, and slew rate limiters
- **Localization** — field pose estimation with latency-compensated Kalman filtering
- **Simulation** — physics models for flywheels and arms that run on your development machine
- **Utilities** — lookup tables, interpolation, and change-detection wrappers

ControlLib is built as a **shadow JAR** — a fat JAR that bundles its dependencies with relocated package names. EJML is relocated from `org.ejml` to `controllib.shadow.org.ejml` to avoid conflicts with any other version of EJML that might be on the classpath. In your `build.gradle`, you consume it with:

```groovy
implementation project(path: ':ControlLib', configuration: 'shadow')
```

### ControlLab

A Swing-based desktop application for testing and visualizing algorithms without touching the robot. It has three tabs:

- **Filter Tab** — load CSV telemetry, apply filters, visualize raw versus filtered signals, and estimate filter lag via cross-correlation
- **Trajectory Tab** — interactive trajectory visualization with limit sliders and back-EMF violation detection
- **Flywheel Tab** — real-time flywheel simulation with controller comparison and plant randomization

ControlLab requires CMake on your PATH because it builds the Ruckig JNI library for desktop.

### RuckigNative

A JNI bridge to the Ruckig C++ library for real-time multi-degree-of-freedom jerk-limited trajectory generation. It builds native `.so` libraries for Android (`arm64-v8a` and `armeabi-v7a`) and desktop. The C++ source lives in a nested git submodule at `MarsCommonFtc/ruckig/`.

## 1.4 Git Submodules: How Shared Code Flows In

The mechanism that connects MarsCommonFtc to your robot project is a **git submodule**. A submodule records a specific commit from a remote repository and checks it out at a fixed path within your project.

### The `.gitmodules` File

At the root of your robot project, `.gitmodules` declares the submodule:

```ini
[submodule "MarsCommonFtc"]
    path = MarsCommonFtc
    url = https://github.com/Mars-Robotics-Association/MarsCommonFtc.git
```

This tells git: "when cloning this repository, also fetch the MarsCommonFtc repository and place it in the `MarsCommonFtc/` directory."

### Cloning with Submodules

If you clone the repository fresh, include the `--recurse-submodules` flag:

```bash
git clone --recurse-submodules <repository-url>
```

If you already cloned without submodules, initialize and update them:

```bash
git submodule init
git submodule update
```

### Pinning Versions

A submodule does not track `main` or a branch by default — it points to a **specific commit**. The recommended workflow is to pin to a git tag:

```bash
cd MarsCommonFtc
git fetch --tags
git checkout v1.1.0
cd ..
git add MarsCommonFtc
git commit -m "Update MarsCommonFtc to v1.1.0"
```

This records the commit SHA that `v1.1.0` points to. Anyone who clones your repository will get exactly that version, regardless of what happens to `main` in the MarsCommonFtc repository.

### Updating the Submodule

When a new version of MarsCommonFtc is released:

```bash
cd MarsCommonFtc
git fetch
git checkout v1.2.0
cd ..
git add MarsCommonFtc
git commit -m "Update MarsCommonFtc to v1.2.0"
```

Always test your robot code after updating. A new version may introduce breaking changes.

## 1.5 Gradle Integration

Gradle needs to know that the modules inside `MarsCommonFtc/` are part of your build. This is done in `settings.gradle` using **project directory redirects**:

```groovy
// --- MarsCommonFtc modules ---
include ':WpiMath'
project(':WpiMath').projectDir = file('MarsCommonFtc/WpiMath')

include ':ControlLib'
project(':ControlLib').projectDir = file('MarsCommonFtc/ControlLib')

include ':ControlLab'
project(':ControlLab').projectDir = file('MarsCommonFtc/ControlLab')

include ':RuckigNative'
project(':RuckigNative').projectDir = file('MarsCommonFtc/RuckigNative')
```

This pattern tells Gradle: "there is a project called `:ControlLib`, and its source code lives in `MarsCommonFtc/ControlLib/`." The project names are flat top-level names — you reference them as `:ControlLib`, not `:MarsCommonFtc:ControlLib`.

### Dependencies in TeamCode

Your robot code lives in the `TeamCode` module. Its `build.gradle` declares which MarsCommonFtc modules it needs:

```groovy
dependencies {
    // ControlLib shadow JAR — includes relocated EJML
    implementation project(path: ':ControlLib', configuration: 'shadow')

    // WpiMath — compile only, EJML provided by robot controller
    compileOnly project(':WpiMath')

    // RuckigNative — JNI for real-time trajectory generation
    implementation project(':RuckigNative')

    // Test dependencies
    testImplementation project(path: ':ControlLib', configuration: 'shadow')
    testImplementation project(':WpiMath')
}
```

Note the different configurations:

- `implementation` — the dependency is included in the final APK and available at compile time
- `compileOnly` — available at compile time but not included in the APK (the robot controller already provides EJML)
- `configuration: 'shadow'` — consumes the fat JAR output of the Shadow plugin, not the regular JAR

### Why Shadow JAR?

The shadow JAR solves a real problem: EJML is a large linear algebra library, and different libraries may depend on different versions. If MarsCommonFtc uses EJML 0.44.0 and another library uses EJML 0.39.0, you get a classpath conflict. By relocating EJML to `controllib.shadow.org.ejml` inside the shadow JAR, ControlLib carries its own private copy that cannot conflict with anything else.

## 1.6 Import Statements and Package Names

When you use MarsCommonFtc classes in your code, you import them from the `org.marsroboticsassociation` package namespace:

```java
import org.marsroboticsassociation.controllib.hardware.IMotor;
import org.marsroboticsassociation.controllib.control.FlywheelSimple;
import org.marsroboticsassociation.controllib.control.FlywheelStateSpace;
import org.marsroboticsassociation.controllib.sim.FlywheelMotorSim;
import org.marsroboticsassociation.controllib.util.SetOnChange;
import org.marsroboticsassociation.controllib.util.LUT;
```

The package structure mirrors the module structure:

- `org.marsroboticsassociation.controllib.*` — classes from ControlLib
- `org.marsroboticsassociation.wpimath.*` — classes from WpiMath
- `org.marsroboticsassociation.controllib.sim.*` — simulation classes
- `org.marsroboticsassociation.controllib.filter.*` — signal filters
- `org.marsroboticsassociation.controllib.localization.*` — pose estimation

## 1.7 Build Workflow

### Building for the Robot

From the project root:

```bash
./gradlew :TeamCode:assembleDebug
```

This compiles TeamCode and all its dependencies, producing an APK that can be deployed to the robot controller phone.

### Running Tests

MarsCommonFtc modules include JUnit 5 tests. Run them from the project root:

```bash
./gradlew :ControlLib:test
./gradlew :WpiMath:test
```

Tests in ControlLib use the simulation classes (`FlywheelMotorSim`, `ArmMotorSim`) to validate controllers without hardware. This is a powerful workflow: you can write and test control logic on your development machine, then deploy to the robot with confidence.

### Running ControlLab

ControlLab is a desktop application. Run it with:

```bash
./gradlew :ControlLab:run
```

This requires CMake on your PATH for the Ruckig native build. If CMake is not available, ControlLab will still compile but the Ruckig tab will not function.

## 1.8 Common Setup Issues

### Submodule Not Cloned

Symptom: `MarsCommonFtc/` directory is empty or missing.

Fix:
```bash
git submodule init
git submodule update
```

### Gradle Cannot Find Modules

Symptom: `Project with path ':ControlLib' could not be found.`

Check `settings.gradle` — the `projectDir` paths must point to the correct locations inside `MarsCommonFtc/`. Also verify that the submodule is cloned and not empty.

### Stale Build Directories

You may find empty directories named `WpiMath/`, `ControlLib/`, `ControlLab/`, or `RuckigNative/` at the project root, containing only `build/` or `.gradle/` subdirectories. These are remnants from a previous configuration before the submodule approach was adopted. They are safe to delete.

### NDK Requirements for RuckigNative

RuckigNative requires Android NDK r25 or later with C++20 support. If you get a CMake error during build, install or update the NDK through Android Studio's SDK Manager or by setting the `ANDROID_NDK_HOME` environment variable.

## 1.9 The Architecture in One Picture

```
┌─────────────────────────────────────────────────────┐
│                    TeamCode                          │
│         (Your robot-specific code)                   │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ArmControl│  │Flywheel  │  │Localization      │  │
│  │Subsystem │  │Subsystem │  │Subsystem         │  │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
│       │              │                 │            │
├───────┼──────────────┼─────────────────┼────────────┤
│       ▼              ▼                 ▼            │
│              ControlLib (shadow JAR)                │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │ArmCtrl   │  │Flywheel  │  │FieldPoseEstimator│  │
│  │SCurve    │  │Simple    │  │PinpointPose      │  │
│  │VelocityMotorPF       │  │TrajectoryManager   │  │
│  └────┬─────┘  └────┬─────┘  └────────┬─────────┘  │
│       │              │                 │            │
├───────┼──────────────┼─────────────────┼────────────┤
│       ▼              ▼                 ▼            │
│              WpiMath                                  │
│                                                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────────────┐  │
│  │PIDCtrl   │  │Pose2d    │  │KalmanFilter      │  │
│  │Trapezoid │  │Kinematics│  │SimpleMotorFF     │  │
│  └──────────┘  └──────────┘  └──────────────────┘  │
│                                                      │
├─────────────────────────────────────────────────────┤
│              RuckigNative (JNI)                      │
│         ┌─────────────────────────────┐             │
│         │  Ruckig C++ (submodule)     │             │
│         └─────────────────────────────┘             │
└─────────────────────────────────────────────────────┘
```

TeamCode depends on ControlLib, which depends on WpiMath. RuckigNative sits alongside ControlLib and provides real-time trajectory generation through JNI. ControlLab is a separate desktop application that uses the same ControlLib and WpiMath modules for offline testing.

## 1.10 Summary

MarsCommonFtc transforms how you build robot code:

- **One library, many robots** — shared control algorithms with versioned releases
- **Git submodules** — pinned versions, clean updates, no copy-paste
- **ProjectDir redirects** — simple Gradle integration without publishing artifacts
- **Shadow JAR** — dependency isolation through package relocation
- **Simulation first** — test controllers on your desktop before touching hardware

With the project structure in place, the next chapter explores the foundation of all mechanism control: the motor abstraction layer.
