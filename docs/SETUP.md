# MarsCommonFtc Setup Guide

Shared FTC modules consumed as a git submodule: **ControlLib**, **ControlLab**, **WpiMath**.

## Prerequisites

- Git (with submodule support)
- Android Studio (Ladybug or later recommended)
- JDK 17

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
git add MarsCommonFtc .gitmodules
git commit -m "add MarsCommonFtc submodule at v1.0.0"
```

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

### "Could not find :ControlLib" (or similar)

Make sure your `settings.gradle` includes the `projectDir` redirects shown above. The project names must match exactly (`:ControlLib`, `:WpiMath`).
