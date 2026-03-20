# MarsCommonFtc Setup Guide

Shared FTC modules consumed as a git submodule: **ControlLib**, **ControlLab**, **WpiMath**, **RuckigNative**.

## Prerequisites

- Git (with submodule support)
- Android Studio (Ladybug or later recommended)
- JDK 17
- (Optional) CMake — only needed for ControlLab desktop runs

## Adding to Your Robot Project

### SourceTree

1. Open your robot project repository
2. Go to **Repository → Add Submodule**
3. Source URL: `https://github.com/Mars-Robotics-Association/MarsCommonFtc.git`
4. Local path: `MarsCommonFtc`
5. After adding, right-click the submodule → **Open Submodule** → checkout the desired tag (e.g. `v1.0.0`)

### CLI

```bash
git submodule add https://github.com/Mars-Robotics-Association/MarsCommonFtc.git MarsCommonFtc
cd MarsCommonFtc && git checkout v1.0.0 && cd ..
git submodule update --init --recursive   # required for nested ruckig submodule
git add MarsCommonFtc .gitmodules
git commit -m "add MarsCommonFtc submodule at v1.0.0"
```

> **Important:** Always use `--recurse-submodules` when cloning a project that uses MarsCommonFtc:
> ```bash
> git clone --recurse-submodules https://github.com/Mars-Robotics-Association/your-robot-project.git
> ```

## settings.gradle Configuration

Add these lines to your project's `settings.gradle` (after your own module includes):

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

Your existing dependency declarations (e.g. `implementation project(':ControlLib')`) will work without changes since the project names are the same.

## Updating to a New Version

### SourceTree

1. Open the MarsCommonFtc submodule in SourceTree
2. Fetch all remotes
3. Checkout the new tag (e.g. `v1.1.0`)
4. Go back to the parent project and commit the submodule pointer change

### CLI

```bash
cd MarsCommonFtc
git fetch
git checkout v1.1.0
cd ..
git add MarsCommonFtc
git commit -m "update MarsCommonFtc to v1.1.0"
```

## Troubleshooting

### "ruckig folder is empty"

The ruckig library is a nested submodule inside MarsCommonFtc. Run:

```bash
git submodule update --init --recursive
```

### "Could not find :ControlLib" (or similar)

Make sure your `settings.gradle` includes the `projectDir` redirects shown above. The project names must match exactly (`:ControlLib`, `:WpiMath`, `:RuckigNative`).

### NDK or cmake not found

- **NDK**: Install via Android Studio → SDK Manager → SDK Tools → NDK (Side by side)
- **cmake**: Install via Android Studio → SDK Manager → SDK Tools → CMake, or install system cmake and ensure it's on PATH
- ControlLab desktop runs require system cmake; Android builds use the Android SDK's cmake
