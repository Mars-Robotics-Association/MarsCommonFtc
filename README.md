# MarsCommonFtc

Shared Java modules for MARS Robotics Association FTC teams. This repo is consumed as a **git submodule** so that multiple robot projects can share the same control, math, and native code without duplication.

## Modules

| Module | Description |
|--------|-------------|
| **ControlLib** | Motion profiling, filters, motor controllers, localization utilities, and simulation tools. Publishes a shadow JAR for use in Android projects. |
| **ControlLab** | Desktop application for testing and visualizing ControlLib algorithms (requires cmake for native ruckig builds). |
| **WpiMath** | Ported subset of WPILib's math library — geometry, kinematics, trajectory generation, and linear algebra. |
| **RuckigNative** | JNI bridge to the [ruckig](https://github.com/pantor/ruckig) online trajectory generation library (included as a nested submodule). |

## Quick start

Add MarsCommonFtc as a submodule in your robot project:

```bash
git submodule add https://github.com/Mars-Robotics-Association/MarsCommonFtc.git MarsCommonFtc
cd MarsCommonFtc && git checkout v1.0.0 && cd ..
git submodule update --init --recursive
```

Then add the module redirects to your `settings.gradle`:

```groovy
include ':WpiMath'
project(':WpiMath').projectDir = file('MarsCommonFtc/WpiMath')

include ':ControlLib'
project(':ControlLib').projectDir = file('MarsCommonFtc/ControlLib')

include ':ControlLab'
project(':ControlLab').projectDir = file('MarsCommonFtc/ControlLab')

include ':RuckigNative'
project(':RuckigNative').projectDir = file('MarsCommonFtc/RuckigNative')
```

Your existing dependency declarations (e.g. `implementation project(':ControlLib')`) work without changes.

## Standalone build

MarsCommonFtc can also build independently for development and testing:

```bash
./gradlew :WpiMath:build :ControlLib:build :ControlLib:shadowJar
./gradlew :WpiMath:test :ControlLib:test
```

## Full setup guide

See [docs/SETUP.md](docs/SETUP.md) for detailed instructions on prerequisites, SourceTree setup, updating to new versions, and troubleshooting.
