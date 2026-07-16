# MarsCommonFtc

Shared Java modules for MARS Robotics Association FTC teams. This repo is consumed as a **git submodule** so that multiple robot projects can share the same control, math, and native code without duplication.

## Modules

| Module | Description |
|--------|-------------|
| **ControlLib** | Motion profiling, filters, motor controllers, localization utilities, and simulation tools. Publishes a shadow JAR for use in Android projects. Depends on nested [RuckigJava](https://github.com/Mars-Robotics-Association/RuckigJava) submodule. |
| **ControlLab** | Desktop application for testing and visualizing ControlLib algorithms. |
| **WpiMath** | Ported subset of WPILib's math library — geometry, kinematics, trajectory generation, and linear algebra. |
| **RuckigJava** | Git submodule: pure-Java port of Ruckig Community OTG (currently pinned to `v0.17.3`). |

## Quick start

Add MarsCommonFtc as a submodule in your robot project and init nested submodules:

```bash
git submodule add https://github.com/Mars-Robotics-Association/MarsCommonFtc.git MarsCommonFtc
cd MarsCommonFtc && git checkout v1.0.0
git submodule update --init --recursive
cd ..
```

Then add the module redirects to your `settings.gradle`:

```groovy
include ':WpiMath'
project(':WpiMath').projectDir = file('MarsCommonFtc/WpiMath')

include ':ControlLib'
project(':ControlLib').projectDir = file('MarsCommonFtc/ControlLib')

include ':ControlLab'
project(':ControlLab').projectDir = file('MarsCommonFtc/ControlLab')

include ':RuckigJava'
project(':RuckigJava').projectDir = file('MarsCommonFtc/RuckigJava')
```

Your existing dependency declarations (e.g. `implementation project(':ControlLib')`) work without changes.

## Standalone build

Clone with submodules:

```bash
git clone --recurse-submodules https://github.com/Mars-Robotics-Association/MarsCommonFtc.git
# or after a plain clone:
git submodule update --init --recursive
```

Then:

```bash
./gradlew :WpiMath:build :ControlLib:build :ControlLib:shadowJar
./gradlew :WpiMath:test :ControlLib:test :RuckigJava:test
```

## Full setup guide

See [docs/SETUP.md](docs/SETUP.md) for detailed instructions on prerequisites, SourceTree setup, updating to new versions, and troubleshooting.
